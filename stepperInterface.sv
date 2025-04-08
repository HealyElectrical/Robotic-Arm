// stepperInterface.sv
// 4/6/2025
// By: Mikhail Rego
// Description: Each input pulses a stepper motor 1.8-deg in one direction or the other
//              Signals output to A4988 (stepper driver): DIR, STEP
//              DIR = 1 for CW, 0 for CCW
//              STEP = 1ms pulse (50000 cycles high, 50000 low at 50MHz)
///////////////////////////////////////////////////////////////////////////////////////

module stepperInterface (
    input  logic CLOCK_50,
    input  logic cw, ccw, reset_n,     // Outputs from rotary encoder
    output logic red, green, blue,     // Debugging LEDs
    output logic dir, step             // A4988 stepper driver pins
);

    // === Parameters ===
    parameter int NUM_STEPS     = 1; // 1 => 1.8 degrees, for max o/p resolution... 5 for larger steps
    parameter int PULSE_LENGTH  = 50000; // 50,000/50e6 = 0.001 s or 1ms (for 2ms pulse total)... inverse of motor speed

    // === FSM ===
    typedef enum logic [1:0] {IDLE, CW, CCW, WAIT} state_t;
    state_t state, next_state;

    // === Internal Registers ===
    logic [7:0]   step_count;
    logic [15:0]  pulseLength_counter;
    logic         step_toggle;
	 
	 // === FSM Sequential Logic (FF) ===
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n)
            state <= IDLE;
        else //if (next_state != WAIT)
            state <= next_state;
    end
	 
	 // === FSM Next-State Logic (comb) ===
    always_comb begin
        case (state)
            IDLE: begin
                if (cw)
                    next_state = CW;
                else if (ccw)
                    next_state = CCW;
                else
                    next_state = IDLE;
            end

            CW, CCW: begin
                if (step_count >= NUM_STEPS)
                    next_state = WAIT;
                else
                    next_state = state;
            end

            WAIT: begin
                if (!cw && !ccw) // wait for encoder inputs to release
                    next_state = IDLE;
                else
                    next_state = WAIT;
            end

            default: next_state = IDLE;
        endcase
    end
	 
	 // === Direction Control ===
    always_ff @(posedge CLOCK_50) begin // no need to reset binary
        if (state == CW)
            dir <= 1'b1;
        else if (state == CCW)
            dir <= 1'b0;
    end
	 
	 // === Step Pulse Generator ===
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            step_toggle         <= 0;
            step_count          <= 0;
            pulseLength_counter <= 0;
        end
        else if ((state == CW || state == CCW)) begin
            if (pulseLength_counter == 0)
                step_toggle <= ~step_toggle;  // Toggle at the start of half-cycle

            if (pulseLength_counter >= PULSE_LENGTH - 1) begin // At end of half-cycle
                pulseLength_counter <= 0;

                // Count steps only at end of LOW phase (step_toggle has been low for a pulseLength)
                if (step_toggle == 0)
                    step_count <= step_count + 1;
            end
            else
                pulseLength_counter <= pulseLength_counter + 1;
        end
        else begin
            step_toggle         <= 0;
            pulseLength_counter <= 0;
            step_count          <= 0;
        end
    end
	 
	 // === Output Mapping ===
    assign step = step_toggle;

    // === LED Debug ===
    assign red   = (state == CW);
    assign green = (state == CCW);
    assign blue  = (state == IDLE); // e.g., inputs held too long

endmodule 