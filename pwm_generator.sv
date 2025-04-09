//==========================================================================
// pwm_generator.sv
// Author     : Glen Healy, Edited by Mikhail Rego
// Description: Converts an 8-bit angle input (0–180°) into a PWM signal
//              for servo control. Output pulse width varies from 1ms (0°)
//              to 2ms (180°) with a total period of 20ms (50Hz).
//==========================================================================

module pwm_generator (
    input  logic       clk,        // 50 MHz system clock
    input  logic       reset_n,    // Active-low asynchronous reset
    input  logic [7:0] angle,      // Angle input (0–180 degrees)
    output logic       pwm_out     // PWM output for servo
);

    // === Parameters ===
    localparam int PWM_PERIOD      = 1_000_000;   // 20ms @ 50 MHz
    localparam int MIN_PULSE_WIDTH =   50_000;    // 1ms  pulse (0°)
    localparam int MAX_PULSE_WIDTH =  100_000;    // 2ms  pulse (180°)

    // === Internal Registers ===
    logic [19:0] pulse_width;     // Calculated pulse width (duty)
    logic [19:0] counter;         // PWM period counter

    // === Angle to Pulse Width Conversion ===
    always_comb begin
        pulse_width = MIN_PULSE_WIDTH + 
                      ((angle * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / 180);
    end

    // === PWM Signal Generator ===
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            counter  <= 0;
            pwm_out  <= 0;
        end else begin
            counter  <= (counter == PWM_PERIOD - 1) ? 0 : counter + 1;
            pwm_out  <= (counter < pulse_width) ? 1'b1 : 1'b0;
        end
    end

endmodule
