module ra(
    input logic CLOCK_50, reset_n,    // 50 MHz system clock and reset
    input logic enc1_a, enc1_b,       // Encoder signals (unused now)
    input logic ADC_SDO,              // ADC serial data output
	 input logic s1, s2,              // ✅ Switches for second motor control
    output logic [6:0] leds,          // 7-segment display output
    output logic [3:0] ct,            // Digit control for multiplexing
    output logic ADC_CONVST, ADC_SDI, ADC_SCK,  // ADC control signals
    output logic [6:0] debug_leds,    // Separate Debug LEDs
    output logic servo_pwm,            // ✅ PWM Output for Servo
	output logic servo_pwm2,            // ✅ PWM Output for Second Motor
   output logic red, green, blue // DM LEDs
);

    // Internal Signals
    logic [1:0] digit;
    logic [3:0] disp_digit;
    logic [15:0] clk_div_count;
    logic [4:0] clk_div_adc;
	 logic [2:0] chan;
    logic adc_clk;
    logic [11:0] adc_value;     // ADC result (Joystick position)
     logic [7:0] angle, angle2;
	 logic [11:0] target_angle;  // Servo target angle
	 logic [11:0] last_adc_value; // ✅ Tracks previous ADC value
	 logic [25:0] update_counter, update_counter2;
    // ------------------------------------------------------------
    // Clock Divider for ADC (1.5625 MHz from 50 MHz)
    // ------------------------------------------------------------
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n)
            clk_div_adc <= 0;
        else
            clk_div_adc <= clk_div_adc + 1;
    end

    assign adc_clk = clk_div_adc[4];
	 // ------------------------------------------------------------
    // **Encoder Module**
    // ------------------------------------------------------------
	  encoder enc_inst (
        .clk(CLOCK_50),
        .a(enc1_a),   // ✅ Ensure the correct encoder signals are used
        .b(enc1_b),
        .cw(cw),
        .ccw(ccw)
    );

    // Debugging: Show encoder pulses on separate debug LEDs
    assign debug_leds[6] = cw;
    assign debug_leds[5] = ccw;

    // Debugging: Print encoder activity
    always_ff @(posedge CLOCK_50) begin
        $display("DEBUG: Encoder -> enc1_a = %b, enc1_b = %b, cw = %b, ccw = %b", enc1_a, enc1_b, cw, ccw);
    end

    whiteOut dmLEDS (.red(dummy), .green, .blue) ; // comment out to use BP leds

    // ------------------------------------------------------------
    // **Encoder to Channel Mapping**
    // ------------------------------------------------------------
    enc2chan chan_mapper (
        .clk(CLOCK_50),
        .reset_n(reset_n),
        .cw(cw),
        .ccw(ccw),
        .chan(chan)
    );
	 
    // ------------------------------------------------------------
    // ADC Interface (Fixed Channel 0)
    // ------------------------------------------------------------
    adcinterface adc_inst1 (
        .clk(adc_clk),         
        .reset_n(reset_n),
		  .chan(chan),      // ✅ Uses the channel selected via encoder
        .result(adc_value),
        .ADC_CONVST(ADC_CONVST),
        .ADC_SCK(ADC_SCK),     
        .ADC_SDI(ADC_SDI),
        .ADC_SDO(ADC_SDO)
    );
	 

    // ------------------------------------------------------------
    // Servo Control Logic - Adjusts Angle Based on Joystick
    // ------------------------------------------------------------
    /*always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            target_angle <= 12'd2048;  // Start at center (90°)
        end else begin
            if (adc_value > 2300) begin
                target_angle <= (target_angle < 4000) ? target_angle + 10 : 4000; // Increase angle
            end else if (adc_value < 1800) begin
                target_angle <= (target_angle > 1000) ? target_angle - 10 : 1000; // Decrease angle
            end
        end
    end*/
	 /*
	 always_ff @(posedge CLOCK_50 or negedge reset_n) begin
    if (!reset_n) begin
        target_angle <= 12'd1000;  // ✅ Start at 0° (minimum angle)
        last_adc_value <= adc_value; // ✅ Initialize tracking of last ADC value
    end else begin
        if (adc_value > last_adc_value) begin
            // ✅ If joystick is increasing, keep increasing angle
            target_angle <= (target_angle < 4000) ? target_angle + 10 : 4000;
        end else if (adc_value < last_adc_value) begin
            // ✅ If joystick is decreasing, keep decreasing angle
            target_angle <= (target_angle > 1000) ? target_angle - 10 : 1000;
        end
        last_adc_value <= adc_value; // ✅ Store the last ADC value for comparison
    end
end*/


logic direction;  // ✅ Tracks direction (0 = up, 1 = down)
//this increments up and down seemlessl
/*
always_ff @(posedge CLOCK_50 or negedge reset_n) begin
    if (!reset_n) begin
        angle <= 8'd0;          // ✅ Start at 0°
        update_counter <= 26'd0; // ✅ Reset speed counter
        direction <= 1'b0;       // ✅ Start moving up
    end else begin
        if (update_counter >= 26'd500_000) begin // ✅ Adjust speed (~9 sec per full sweep)
            update_counter <= 0; // ✅ Reset counter every update

            if (direction == 1'b0) begin
                // ✅ Moving up
                if (angle < 180)
                    angle <= angle + 1;
                else begin
                    direction <= 1'b1;  // ✅ Switch direction at 180°
                    angle <= 179;       // ✅ Ensure immediate transition without delay
                end
            end else begin
                // ✅ Moving down
                if (angle > 0)
                    angle <= angle - 1;
                else begin
                    direction <= 1'b0;  // ✅ Switch direction at 0°
                    angle <= 1;         // ✅ Ensure immediate transition without delay
                end
            end
        end else begin
            update_counter <= update_counter + 1;
        end
    end
end*/
  // ------------------------------------------------------------
    // ✅ **First Motor: Joystick-Based Servo Control**
    // ------------------------------------------------------------
always_ff @(posedge CLOCK_50 or negedge reset_n) begin
    if (!reset_n) begin
        angle <= 8'd90;         // ✅ Start at 90° (Neutral Position)
        update_counter <= 26'd0; // ✅ Reset speed counter
    end else begin
        update_counter <= update_counter + 1;

        if (update_counter >= 26'd1_500_000) begin // ✅ Faster delay (~33ms per step)
            update_counter <= 0; // ✅ Reset counter immediately after update

            if (adc_value > 16'h750) begin
                // ✅ Joystick moved right → Increase angle faster
					 //maybe should add little delays to make sure the signal has settled 
                angle <= (angle < 180) ? angle + 2 : 180;
            end else if (adc_value < 16'h500) begin
                // ✅ Joystick moved left → Decrease angle faster
                angle <= (angle > 0) ? angle - 2 : 0;
            end
            // ✅ If between `570` and `630`, it does NOT update `angle`, so it holds position.
        end
    end
end

    // ------------------------------------------------------------
    // ✅ **Second Motor: Button-Based Control**
    // ------------------------------------------------------------
	 /*always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            angle2 <= 8'd90;
            update_counter2 <= 26'd0;
        end else begin
            update_counter2 <= update_counter2 + 1;
            if (update_counter2 >= 26'd1_500_000) begin  // ✅ Faster update (~33ms per step)
                update_counter2 <= 0;
                if (s1) begin
                    angle2 <= (angle2 < 180) ? angle2 + 2 : 180;  // ✅ Increment with SW[0]
                end else if (s2) begin
                    angle2 <= (angle2 > 0) ? angle2 - 2 : 0;  // ✅ Decrement with SW[2]
                end
            end
        end
    end
*/
//
always_ff @(posedge CLOCK_50 or negedge reset_n) begin
    if (!reset_n) begin
        angle2 <= 8'd90;         // ✅ Start at 90° (Neutral Position)
    end else begin
        if (!s1) begin
            // ✅ Button `s1` pressed → Increase angle
            angle2 <= (angle2 < 180) ? angle2 + 2 : 180;
        end else if (!s2) begin
            // ✅ Button `s2` pressed → Decrease angle
            angle2 <= (angle2 > 0) ? angle2 - 2 : 0;
        end
    end
end


/*
module servo_simple (
    input logic CLOCK_50,     // 50 MHz system clock
    input logic reset_n,      // Active-low reset
    output logic servo_pwm    // PWM output to servo motor
);

    // Parameters for PWM (50 MHz clock)
    localparam int PWM_PERIOD = 1_000_000;      // 20 ms (50Hz)
    localparam int MIN_PULSE  = 50_000;         // 1 ms pulse (0°)
    localparam int MAX_PULSE  = 100_000;        // 2 ms pulse (180°)
    localparam int STEP_SIZE  = 500;            // Pulse increment size for smooth motion

    // Internal signals
    logic [19:0] counter;
    logic [19:0] pulse_width;
    logic increasing;

    // PWM generation logic
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            counter      <= 0;
            pulse_width  <= MIN_PULSE;
            increasing   <= 1'b1;
            servo_pwm    <= 0;
        end else begin
            // PWM period counter (0 to 1,000,000 counts)
            if (counter < PWM_PERIOD - 1)
                counter <= counter + 1;
            else
                counter <= 0;

            // Generate PWM output
            servo_pwm <= (counter < pulse_width) ? 1'b1 : 1'b0;

            // Update pulse width once per PWM cycle
            if (counter == PWM_PERIOD - 1) begin
                if (increasing) begin
                    if (pulse_width < MAX_PULSE)
                        pulse_width <= pulse_width + STEP_SIZE;
                    else
                        increasing <= 1'b0; // Switch to decreasing
                end else begin
                    if (pulse_width > MIN_PULSE)
                        pulse_width <= pulse_width - STEP_SIZE;
                    else
                        increasing <= 1'b1; // Switch to increasing
                end
            end
        end
    end

endmodule*?
	 
    // ------------------------------------------------------------
    // PWM Generator for Servo Control
    // ------------------------------------------------------------
    /*pwm_generator pwm_inst (
        .clk(CLOCK_50),
        .reset_n(reset_n),
        .adc_value(target_angle), // Pass processed servo angle
        .pwm_out(servo_pwm)
    );*/
	  pwm_generator pwm_inst (
        .clk(CLOCK_50),
        .reset_n(reset_n),
        .angle(angle), // ✅ Pass processed angle instead of ADC value
        .pwm_out(servo_pwm)
    );
	 pwm_generator pwm_inst2 (
        .clk(CLOCK_50),
        .reset_n(reset_n),
        .angle(angle2),
        .pwm_out(servo_pwm2)
    );

    // ------------------------------------------------------------
    // 7-Segment Display Logic (unchanged)
    // ------------------------------------------------------------
    decode2 decode2_0 (
        .digit(digit),
        .ct(ct)
    );

    decode7 decode7_0 (
        .num(disp_digit),
        .leds(leds)
    );

    // Clock Divider for Display Multiplexing
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n)
            clk_div_count <= 0;
        else
            clk_div_count <= clk_div_count + 1;
    end

    assign digit = clk_div_count[15:14];

    // Display ADC Value (shows 12-bit ADC result on 4-digit display)
    /*
	always_comb begin
        case (digit)
            2'b00: disp_digit = {1'b0, chan};       // Display ADC channel (0-7)
            2'b00: disp_digit = adc_value[11:8];  
            2'b01: disp_digit = adc_value[7:4];   
            2'b10: disp_digit = adc_value[3:0];       // Lower ADC result bits
            default: disp_digit = 4'hF;             // Blank display (Failsafe)
        endcase
    end
	 */
	 //adding timing with clock seems to have fixed the problem
	 always_ff @(posedge CLOCK_50 or negedge reset_n) begin
    if (!reset_n) begin
        disp_digit <= 4'hF;  // ✅ Default to blank on reset
    end else begin
        case (digit)
            2'b00: disp_digit <= {1'b0, chan};  // ✅ Display ADC channel (0-7)
            2'b01: disp_digit <= adc_value[11:8];  // ✅ Upper 4 bits of ADC value
            2'b10: disp_digit <= adc_value[7:4];   // ✅ Middle 4 bits of ADC value
            2'b11: disp_digit <= adc_value[3:0];   // ✅ Lower 4 bits of ADC value
            default: disp_digit <= 4'hF;  // ✅ Failsafe for blank display
        endcase
    end
end
endmodule
