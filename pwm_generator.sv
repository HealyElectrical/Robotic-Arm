/*module pwm_generator (
    input logic clk,          // 50 MHz system clock
    input logic reset_n,      // Active-low reset
    input logic [11:0] adc_value, // 12-bit ADC input (0-4095)
    output logic pwm_out      // PWM output for servo
);

    // 50Hz PWM period = 20ms (50,000 clock cycles at 50MHz)
    localparam int PWM_PERIOD = 50_000_000 / 50; // 1_000_000 cycles (20ms)

    // Pulse width range: 1ms (2.5%) to 2ms (5%)
    localparam int MIN_PULSE_WIDTH = PWM_PERIOD / 20; // 1ms (50,000 cycles)
    localparam int MAX_PULSE_WIDTH = PWM_PERIOD / 10; // 2ms (100,000 cycles)

    // Register to store the calculated pulse width
    logic [19:0] pulse_width;
    logic [19:0] counter;

    // Convert ADC value (0-4095) to pulse width (1ms to 2ms)
    always_comb begin
        pulse_width = MIN_PULSE_WIDTH + ((adc_value * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / 4095);
    end

    // PWM Generation Logic
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            counter <= 0;
            pwm_out <= 0;
        end else begin
            counter <= (counter == PWM_PERIOD - 1) ? 0 : counter + 1;
            pwm_out <= (counter < pulse_width) ? 1 : 0;
        end
    end

endmodule
*/
/*module pwm_generator (
    input logic clk,          // 50 MHz system clock
    input logic reset_n,      // Active-low reset
    input logic [11:0] adc_value, // 12-bit ADC input (0-4095)
    output logic pwm_out      // PWM output for servo
);

    // 50Hz PWM period = 20ms (50,000 clock cycles at 50MHz)
    localparam int PWM_PERIOD = 1_000_000;    // 20 ms period at 50MHz clock
    localparam int MIN_PULSE_WIDTH = 50_000;  // 1 ms pulse (0°)
    localparam int MAX_PULSE_WIDTH = 100_000; // 2 ms pulse (180°)

    // Internal registers
    logic [19:0] pulse_width;
    logic [19:0] counter;

    // Convert ADC value (0-4095) to pulse width (1ms to 2ms)
    always_comb begin
        pulse_width = MIN_PULSE_WIDTH + ((adc_value * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / 4095);
    end

    // PWM Generation Logic
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            counter <= 0;
            pwm_out <= 0;
        end else begin
            counter <= (counter == PWM_PERIOD - 1) ? 0 : counter + 1;
            pwm_out <= (counter < pulse_width) ? 1 : 0;
        end
    end

endmodule*/

module pwm_generator (
    input logic clk,          // 50 MHz system clock
    input logic reset_n,      // Active-low reset
    input logic [7:0] angle,  // ✅ 8-bit angle input (0-180°)
    output logic pwm_out      // ✅ PWM output for servo
);

    // 50Hz PWM period = 20ms (50,000 clock cycles at 50MHz)
    localparam int PWM_PERIOD = 1_000_000;    // 20 ms period at 50MHz clock
    localparam int MIN_PULSE_WIDTH = 50_000;  // 1 ms pulse (0°)
    localparam int MAX_PULSE_WIDTH = 100_000; // 2 ms pulse (180°)

    // Internal registers
    logic [19:0] pulse_width;
    logic [19:0] counter;

    // ✅ Convert angle (0-180°) to pulse width (1ms to 2ms)
    always_comb begin
        pulse_width = MIN_PULSE_WIDTH + ((angle * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / 180);
    end

    // ✅ PWM Generation Logic
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            counter <= 0;
            pwm_out <= 0;
        end else begin
            counter <= (counter == PWM_PERIOD - 1) ? 0 : counter + 1;
            pwm_out <= (counter < pulse_width) ? 1 : 0;
        end
    end

endmodule

