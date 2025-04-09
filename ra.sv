//==========================================================================
// Module: ra.sv (Robotic Arm Controller)
// Author: Glen Healy
// Date  : April, 7 2025
//==========================================================================
// Description:
// - Controls 6 servo motors (joystick + button)
// - Interfaces with ADC (LTC2308) for analog joystick input
// - Operates a stepper motor via rotary encoder
// - Displays ADC values on 7-segment display
// - Supports servo motor switching via JOY_SEL button
//==========================================================================
// Edited: Mikhail Rego, adding & debugging stepper module
// Date  : April, 8 2025
//==========================================================================

module ra (
    input  logic CLOCK_50, reset_n,             // System clock and active-low reset
    input  logic enc1_a, enc1_b, enc2_a, enc2_b, // Rotary encoder inputs
    input  logic ADC_SDO,                        // ADC serial output
    input  logic s1, s2,                         // Buttons for servo 2–4
    input  logic JOY_SEL,                        // Toggle selected servo (2–4)
    output logic [6:0] leds,                     // 7-segment LED outputs
    output logic [3:0] ct,                       // 7-segment digit select
    output logic ADC_CONVST, ADC_SDI, ADC_SCK,   // ADC control signals
    output logic [6:0] debug_leds,               // Debug output for encoder
    output logic servo_pwm,                      // PWM: servo 1 (gripper rotate)
    output logic servo_pwm2, servo_pwm3,         // PWM: servo 2 (base), 3 (mid)
    output logic servo_pwm4, servo_pwm5,         // PWM: servo 4 (tip), 5 (gripper close)
    output logic servo_pwm6,                     // PWM: servo 6 (mirror of servo 2)
    output logic red, green, blue,               // LEDs indicate selected motor
    output logic dir_pulse, step_pulse           // Stepper motor control
);

    //==========================================================================
    // Clock + Internal State
    //==========================================================================
    logic [15:0] clk_div_count;                  // Display refresh divider
    logic [4:0]  clk_div_adc;                    // ADC clock divider
    logic        adc_clk;                        // ~1.56 MHz ADC SPI clock

    logic [1:0]  digit;                          // 7-seg digit selector
    logic [3:0]  disp_digit;                     // 7-seg decoded nibble

    logic [2:0]  chan;                           // Selected ADC channel
    logic [11:0] adc_value;                      // ADC raw output
    logic [11:0] adc_ch0_value, adc_ch1_value;   // ADC channel 0 & 1 values

    logic toggle_state;                          // Toggles ADC channel
    logic [15:0] adc_toggle_counter;             // Rate control for toggling

    //==========================================================================
    // Servo Angle Registers
    //==========================================================================
    logic [7:0] angle;      // Servo 1: gripper spinner
    logic [7:0] angle2;     // Servo 2: base
    logic [7:0] angle3;     // Servo 3: mid
    logic [7:0] angle4;     // Servo 4: tip
    logic [7:0] angle5;     // Servo 5: gripper close
    logic [7:0] angle6;     // Servo 6: mirror of servo 2

    // Update timers for respective motor groups
    logic [25:0] update_counter;    // Servo 1
    logic [25:0] update_counter2;   // Servos 2–4
    logic [25:0] update_counter5;   // Servo 5

    //==========================================================================
    // Encoder and Directional Pulse Signals
    //==========================================================================
    logic cwIn, ccwIn;              // Raw encoder 1 outputs
    logic cw2In, ccw2In;            // Raw encoder 2 outputs
    logic cw, ccw;                  // Debounced pulse output (encoder 1)
    logic cw2, ccw2;                // Debounced pulse output (encoder 2)

    //==========================================================================
    // JOY_SEL Button State
    //==========================================================================
    logic JOY_SEL_prev;            // Edge detection
    logic JOY_SEL_stable;          // Debounced state
    logic [19:0] debounce_counter; // JOY_SEL debounce
    logic [1:0] current_motor;     // 0: servo2, 1: servo3, 2: servo4

    //==========================================================================
    // Clock Dividers
    //==========================================================================
    always_ff @(posedge CLOCK_50 or negedge reset_n)
        clk_div_adc <= (!reset_n) ? 0 : clk_div_adc + 1;

    assign adc_clk = clk_div_adc[4];  // ~1.56 MHz clock

    always_ff @(posedge CLOCK_50 or negedge reset_n)
        clk_div_count <= (!reset_n) ? 0 : clk_div_count + 1;

    assign digit = clk_div_count[15:14];

    //==========================================================================
    // Rotary Encoder Modules
    //==========================================================================
    encoder enc1 (.clk(CLOCK_50), .a(enc1_a), .b(enc1_b), .cw(cwIn), .ccw(ccwIn));     // Encoder 1
    encoder enc2 (.clk(CLOCK_50), .a(enc2_a), .b(enc2_b), .cw(cw2In), .ccw(ccw2In));   // Encoder 2

    enc_down encDown1 (.clk(CLOCK_50), .reset_n(reset_n), .cw_in(cwIn),  .ccw_in(ccwIn),  .cw_out(cw),  .ccw_out(ccw));   // Encoder 1 → 1 pulse/3 clicks
    enc_down encDown2 (.clk(CLOCK_50), .reset_n(reset_n), .cw_in(cw2In), .ccw_in(ccw2In), .cw_out(cw2), .ccw_out(ccw2));  // Encoder 2 → stepper input

    assign debug_leds[6] = cw;
    assign debug_leds[5] = ccw;

    //==========================================================================
    // ADC Interface with Toggle Logic (Channel 0 and 1)
    //==========================================================================
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            toggle_state <= 0;
            adc_toggle_counter <= 0;
        end else begin
            adc_toggle_counter <= adc_toggle_counter + 1;
            if (adc_toggle_counter >= 16'd15000) begin
                toggle_state <= ~toggle_state;
                adc_toggle_counter <= 0;
            end
        end
    end

    assign chan = (toggle_state) ? 3'd1 : 3'd0;

    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            adc_ch0_value <= 0;
            adc_ch1_value <= 0;
        end else if (toggle_state == 0)
            adc_ch0_value <= adc_value;
        else
            adc_ch1_value <= adc_value;
    end

    adcInterface adc_inst1 (  // LTC2308 SPI Interface
        .clk(adc_clk), .reset_n(reset_n), .chan(chan), .result(adc_value),
        .ADC_CONVST(ADC_CONVST), .ADC_SDI(ADC_SDI), .ADC_SCK(ADC_SCK), .ADC_SDO(ADC_SDO)
    );

    //==========================================================================
    // Servo Motor Control Logic
    //==========================================================================

    // Servo 1: Joystick X-axis
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin angle <= 90; update_counter <= 0; end
        else begin
            update_counter <= update_counter + 1;
            if (update_counter >= 26'd1_500_000) begin
                update_counter <= 0;
                if (adc_ch0_value > 12'h750) angle <= (angle < 180) ? angle + 2 : 180;
                else if (adc_ch0_value < 12'h500) angle <= (angle > 0) ? angle - 2 : 0;
            end
        end
    end

    // Servo 5: Joystick Y-axis
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin angle5 <= 90; update_counter5 <= 0; end
        else begin
            update_counter5 <= update_counter5 + 1;
            if (update_counter5 >= 26'd1_500_000) begin
                update_counter5 <= 0;
                if (adc_ch1_value > 12'h750) angle5 <= (angle5 < 180) ? angle5 + 2 : 180;
                else if (adc_ch1_value < 12'h500) angle5 <= (angle5 > 0) ? angle5 - 2 : 0;
            end
        end
    end

    // Button-Controlled Servos: 2, 3, 4 (selected via JOY_SEL)
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            angle2 <= 90; angle3 <= 90; angle4 <= 90; angle6 <= 90;
            update_counter2 <= 0;
        end else begin
            update_counter2 <= update_counter2 + 1;
            if (update_counter2 >= 26'd1_000_000) begin
                update_counter2 <= 0;
                case (current_motor)
                    2'd0: begin
                        if (!s1 && s2) angle2 <= (angle2 < 180) ? angle2 + 1 : 180;
                        else if (!s2 && s1) angle2 <= (angle2 > 0) ? angle2 - 1 : 0;
                        angle6 <= 180 - angle2;  // Mirror for Motor 6
                    end
                    2'd1: begin
                        if (!s1 && s2) angle3 <= (angle3 < 180) ? angle3 + 1 : 180;
                        else if (!s2 && s1) angle3 <= (angle3 > 0) ? angle3 - 1 : 0;
                    end
                    2'd2: begin
                        if (!s1 && s2) angle4 <= (angle4 < 180) ? angle4 + 1 : 180;
                        else if (!s2 && s1) angle4 <= (angle4 > 0) ? angle4 - 1 : 0;
                    end
                endcase
            end
        end
    end

    //==========================================================================
    // Motor Selector (JOY_SEL Button) + LED Indicator
    //==========================================================================
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            current_motor <= 0;
            JOY_SEL_prev <= 1;
            debounce_counter <= 0;
            JOY_SEL_stable <= 1;
        end else begin
            if (JOY_SEL_prev != JOY_SEL) debounce_counter <= 0;
            else if (debounce_counter < 20'd800_000)
                debounce_counter <= debounce_counter + 1;
            else
                JOY_SEL_stable <= JOY_SEL;

            if (JOY_SEL_stable && !JOY_SEL)
                current_motor <= (current_motor == 2'd2) ? 2'd0 : current_motor + 1;

            JOY_SEL_prev <= JOY_SEL;
        end
    end

    always_comb begin
        red   = (current_motor == 2'd1);
        green = (current_motor == 2'd0);
        blue  = (current_motor == 2'd2);
    end

    //==========================================================================
    // Servo PWM Generators
    //==========================================================================
    pwm_generator pwm_inst  (.clk(CLOCK_50), .reset_n(reset_n), .angle(angle),  .pwm_out(servo_pwm));   // Servo 1
    pwm_generator pwm_inst2 (.clk(CLOCK_50), .reset_n(reset_n), .angle(angle2), .pwm_out(servo_pwm2));  // Servo 2
    pwm_generator pwm_inst3 (.clk(CLOCK_50), .reset_n(reset_n), .angle(angle3), .pwm_out(servo_pwm3));  // Servo 3
    pwm_generator pwm_inst4 (.clk(CLOCK_50), .reset_n(reset_n), .angle(angle4), .pwm_out(servo_pwm4));  // Servo 4
    pwm_generator pwm_inst5 (.clk(CLOCK_50), .reset_n(reset_n), .angle(angle5), .pwm_out(servo_pwm5));  // Servo 5
    pwm_generator pwm_inst6 (.clk(CLOCK_50), .reset_n(reset_n), .angle(angle6), .pwm_out(servo_pwm6));  // Servo 6

    //==========================================================================
    // Stepper Motor Interface
    //==========================================================================
    logic dummy1, dummy2, dummy3;  // Unused RGB signals

    stepperInterface #(.NUM_STEPS(5), .PULSE_LENGTH(50000)) stepperInterface_inst (
        .CLOCK_50(CLOCK_50), .cw(cw2), .ccw(ccw2), .reset_n(reset_n),
        .red(dummy1), .green(dummy2), .blue(dummy3), .dir(dir_pulse), .step(step_pulse)
    );

    //==========================================================================
    // 7-Segment Display Logic (ADC Channel 0)
    //==========================================================================
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) disp_digit <= 4'hF;
        else begin
            case (digit)
                2'b00: disp_digit <= 4'd0; // CH0 label
                2'b01: disp_digit <= adc_ch0_value[11:8];
                2'b10: disp_digit <= adc_ch0_value[7:4];
                2'b11: disp_digit <= adc_ch0_value[3:0];
                default: disp_digit <= 4'hF;
            endcase
        end
    end

    decode2 decode2_inst (.digit(digit), .ct(ct));      // Digit selector
    decode7 decode7_inst (.num(disp_digit), .leds(leds)); // 7-seg encoder

endmodule
