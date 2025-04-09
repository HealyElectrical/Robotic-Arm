//==========================================================================
// stepperInterface.sv
// Author     : Mikhail Rego
// Date       : 04/06/2025
// Description: Generates step/direction signals for A4988 stepper driver.
//              - DIR = 1 for CW, 0 for CCW
//              - STEP = 2ms pulse (1ms HIGH, 1ms LOW @50MHz)
//              - Inputs: cw, ccw (from encoder)
//==========================================================================

module stepperInterface #(
    parameter int NUM_STEPS    = 1,       // Number of steps per activation
    parameter int PULSE_LENGTH = 50000    // Half-cycle pulse length (1ms @ 50MHz)
)(
    input  logic CLOCK_50,
    input  logic cw, ccw, reset_n,
    output logic red, green, blue,
    output logic dir,
    output logic step
);

    // === FSM ===
    typedef enum logic [1:0] {IDLE, CW, CCW, WAIT} state_t;
    state_t state, next_state;

    // === Internal Registers ===
    logic [7:0]   step_count;
    logic [15:0]  pulseLength_counter;
    logic         step_toggle;

    // === FSM State Update ===
    always_ff @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // === FSM Next-State Logic ===
    always_comb begin
        case (state)
            IDLE:   next_state = (cw)  ? CW  :
                                 (ccw) ? CCW : IDLE;

            CW,
            CCW:    next_state = (step_count >= NUM_STEPS) ? WAIT : state;

            WAIT:   next_state = (!cw && !ccw) ? IDLE : WAIT;

            default: next_state = IDLE;
        endcase
    end

    // === Direction Control ===
    always_ff @(posedge CLOCK_50) begin
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
        end else if (state == CW || state == CCW) begin
            if (pulseLength_counter == 0)
                step_toggle <= ~step_toggle;

            if (pulseLength_counter >= PULSE_LENGTH - 1) begin
                pulseLength_counter <= 0;
                if (step_toggle == 0)
                    step_count <= step_count + 1;
            end else begin
                pulseLength_counter <= pulseLength_counter + 1;
            end
        end else begin
            step_toggle         <= 0;
            pulseLength_counter <= 0;
            step_count          <= 0;
        end
    end

    // === Outputs ===
    assign step  = step_toggle;
    assign red   = (state == CW);
    assign green = (state == CCW);
    assign blue  = (state == IDLE);

endmodule
