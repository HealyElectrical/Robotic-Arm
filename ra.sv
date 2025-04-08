// ra.sv
// top level module for robotic arm
// By: Glen Healy
// 4/7/2025
/////////////////////////////////////////////////////////////////////////

module ra(
   input logic CLOCK_50, reset_n,    // 50 MHz system clock and reset
   (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *) // needed per rotary encoder
   input logic enc1_a, enc1_b,  // Encoder 1 signals    
	(* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *) // needed per rotary encoder
   input logic enc2_a, enc2_b,  // Encoder 1 signals    
   input logic ADC_SDO,              // ADC serial data output
   input logic s1, s2,              // Switches for second motor control
   input logic JOY_SEL,             //swiches between servo motors 2-4
   output logic [6:0] leds,          // 7-segment display output
   output logic [3:0] ct,            // Digit control for multiplexing
   output logic ADC_CONVST, ADC_SDI, ADC_SCK,  // ADC control signals
   output logic [6:0] debug_leds,    // Separate Debug LEDs
   output logic servo_pwm,            // ✅ PWM Output for Servo, claw gripper spinner
   output logic servo_pwm2, servo_pwm3, servo_pwm4,           // ✅ PWM Output for Second Motor
   output logic red, green, blue, // DM LEDs
	output logic dir_pulse, step_pulse // for stepper: GPIO_0: dir=[34] step=[33]

);

   // Internal Signals
   logic [1:0] digit;
   logic [3:0] disp_digit;
   logic [15:0] clk_div_count;
   logic [4:0] clk_div_adc;
   logic [2:0] chan;
   logic adc_clk;
   logic [11:0] adc_value;     // ADC result (Joystick position)
   logic [7:0] angle, angle2, angle3, angle4;  //declaring angles of servos
   logic [11:0] target_angle;  // Servo target angle
   logic [11:0] last_adc_value; // ✅ Tracks previous ADC value
   logic [25:0] update_counter, update_counter2;
	logic cw2, ccw2; // for stepper
	logic dummy1, dummy2, dummy3; // for stepper
	
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	// Stepper module 
	encoder enc_2 (.clk(CLOCK_50), .a(enc2_a), .b(enc2_b), .cw(cw2), .ccw(ccw2));
	//enc_down enc_down_1 (.clk(CLOCK_50), .reset_n(reset_n), .cw_in(cw2), .ccw_in(ccw2), .cw_out(enc2_cw), .ccw_out(enc2_ccw)) ; // future work
	
	stepperInterface #(
        .NUM_STEPS(1), // 1 => 1.8 degrees, for max o/p resolution... 5 for larger steps
        .PULSE_LENGTH(50000) // 50,000/50e6 = 0.001 s or 1ms (for 2ms pulse total)... inverse of motor speed
    ) stepperInterface_inst (
		 .CLOCK_50(CLOCK_50), 
		 .cw(cw2), 
		 .ccw(ccw2), 
		 .reset_n(reset_n), 
		 .red(dummy1), .green(dummy2), .blue(dummy3), 
		 .dir(dir_pulse), 
		 .step(step_pulse)
	 );
	
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	
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

   //whiteOut dmLEDS (.red, .green, .blue) ; // comment out to use BP leds

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

   logic direction;  // ✅ Tracks direction (0 = up, 1 = down)

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
   // ✅ **Motor Selection Control via JOY_SEL Button**
   // ------------------------------------------------------------
   // This logic detects a rising edge (button press) on JOY_SEL,
   // and cycles through the active servo motor control:
   // - 0 → Motor 2 (default)
   // - 1 → Motor 3
   // - 2 → Motor 4
   // - then wraps back around to 0 (Motor 2)
   //
   // `current_motor` holds the index of the selected motor
   // `JOY_SEL_prev` is used to detect the edge of the button
   // ------------------------------------------------------------
   logic [19:0] debounce_counter;
   logic JOY_SEL_stable;
   // ------------------------------------------------------------
   // ✅ JOY_SEL Debounce and Motor Selector
   // ------------------------------------------------------------
   // Stabilizes input and switches motors on single clean press
   // Uses a counter to ignore bouncing/transients
   // ------------------------------------------------------------
   always_ff @(posedge CLOCK_50 or negedge reset_n) begin
      if (!reset_n) begin
         current_motor     <= 2'd0;     // Start with Motor 2
         JOY_SEL_prev      <= 1'b1;
         debounce_counter  <= 0;
         JOY_SEL_stable    <= 1'b1;
      end else begin
         // Debounce: if input is stable low, increment counter
         if (JOY_SEL_prev != JOY_SEL) begin
            debounce_counter <= 0;  // Reset counter if input is changing
         end else begin
            if (debounce_counter < 20'd800_000) begin
               debounce_counter <= debounce_counter + 1;
            end else begin
               JOY_SEL_stable <= JOY_SEL;  // Register stable state after hold
            end
         end

         // Only act on stable falling edge (button press)
         if (JOY_SEL_stable && !JOY_SEL) begin
            case (current_motor)
               2'd0: current_motor <= 2'd1;
               2'd1: current_motor <= 2'd2;
               2'd2: current_motor <= 2'd0;
               default: current_motor <= 2'd0;
            endcase
         end

         JOY_SEL_prev <= JOY_SEL;
      end
   end

   // ------------------------------------------------------------
   // ✅ **Second Motor: Button-Based Control**
   // ------------------------------------------------------------
   logic [1:0] current_motor;     // 2-bit counter: 0 = M2, 1 = M3, 2 = M4
   logic JOY_SEL_prev;            // For edge detection

   always_ff @(posedge CLOCK_50 or negedge reset_n) begin
      if (!reset_n) begin
         angle2 <= 8'd90;
         angle3 <= 8'd90;
         angle4 <= 8'd90;
         update_counter2 <= 26'd0;
      end else begin
         update_counter2 <= update_counter2 + 1;

         if (update_counter2 >= 26'd1_000_000) begin
            update_counter2 <= 0;

            case (current_motor)
               2'd0: begin  // Motor 2
                  if (!s1 && s2)
                     angle2 <= (angle2 < 180) ? angle2 + 1 : 180;
                  else if (!s2 && s1)
                     angle2 <= (angle2 > 0) ? angle2 - 1 : 0;
               end
               2'd1: begin  // Motor 3
                  if (!s1 && s2)
                     angle3 <= (angle3 < 180) ? angle3 + 1 : 180;
                  else if (!s2 && s1)
                     angle3 <= (angle3 > 0) ? angle3 - 1 : 0;
               end
               2'd2: begin  // Motor 4
                  if (!s1 && s2)
                     angle4 <= (angle4 < 180) ? angle4 + 1 : 180;
                  else if (!s2 && s1)
                     angle4 <= (angle4 > 0) ? angle4 - 1 : 0;
               end
            endcase
         end
      end
   end



   // ------------------------------------------------------------
   // ✅ **LED Indicator Control for Active Motor**
   // ------------------------------------------------------------
   // This logic lights up one LED to show which motor is being controlled:
   // - Green LED when Motor 2 is selected
   // - Red LED when Motor 3 is selected
   // - Blue LED when Motor 4 is selected
   // Only one LED is ON at a time, based on `current_motor` state.
   // ------------------------------------------------------------
   always_comb begin
      // Turn off all LEDs by default
      red   = 1'b0;
      green = 1'b0;
      blue  = 1'b0;

      // Turn ON the LED corresponding to the active motor
      case (current_motor)
         2'd0: green = 1'b1;  // Motor 2
         2'd1: red   = 1'b1;  // Motor 3
         2'd2: blue  = 1'b1;  // Motor 4
         default: ;           // No LED if invalid state (shouldn't happen)
      endcase
   end



   // ------------------------------------------------------------
   // PWM Generator for Servo Control
   // ------------------------------------------------------------
   //servo motor spins claw gripper controlled by channel 0. the closing of the gripper will be controlled by channel 1.
   pwm_generator pwm_inst (
      .clk(CLOCK_50),
      .reset_n(reset_n),
      .angle(angle), // ✅ Pass processed angle instead of ADC value
      .pwm_out(servo_pwm)
   );

   //servo 2 (arm-base, might need to figure out how to get 2 inverse controls here)
   pwm_generator pwm_inst2 (
      .clk(CLOCK_50),
      .reset_n(reset_n),
      .angle(angle2),
      .pwm_out(servo_pwm2)
   );
   //servo 3 (arm-b
   pwm_generator pwm_inst3 (
      .clk(CLOCK_50),
      .reset_n(reset_n),
      .angle(angle3),
      .pwm_out(servo_pwm3)
   );
   //servo 4
   pwm_generator pwm_inst4 (
      .clk(CLOCK_50),
      .reset_n(reset_n),
      .angle(angle4),
      .pwm_out(servo_pwm4)
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