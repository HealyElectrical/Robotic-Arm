//==========================================================================
// encoder.sv
// Author     : Mikhail Rego
// Date       : 01/27/2025
// Description: Detects rotation direction from a quadrature rotary encoder.
//              Outputs a one-cycle pulse on `cw` (clockwise) or `ccw`
//              (counterclockwise) when movement is detected.
//==========================================================================

module encoder (
    input  logic clk,      // System clock
    input  logic a, b,     // Quadrature encoder signals (90° out of phase)
    output logic cw, ccw   // One-cycle direction pulses
);

    // === Internal State Tracking ===
    logic [1:0] state, prev_state;
    logic       idle;

    // === State Update ===
    always_ff @(posedge clk) begin
        prev_state <= state;     // Capture previous state
        state      <= {a, b};    // Current encoder state (a = MSB, b = LSB)
    end

    // === Rotation Detection Logic ===
    always_comb begin
        idle = (state == prev_state);  // No change → idle

        // Default outputs
        cw  = 1'b0;
        ccw = 1'b0;

        // Detect rotation if not idle
        if (!idle) begin
            case ({state, prev_state})
                // Clockwise transition patterns
                4'b1000, 4'b1110, 4'b0111, 4'b0001: cw  = 1'b1;

                // Counterclockwise transition patterns
                4'b1011, 4'b0010, 4'b0100, 4'b1101: ccw = 1'b1;

                default: begin
                    cw  = 1'b0;
                    ccw = 1'b0;
                end
            endcase
        end
    end

endmodule
