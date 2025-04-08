// File: encoder.sv
// Description: This module takes in a clk-input and two values, 90-deg shifted, from a rotary device. 
// 	It then determines which value changes first. Finally, it outputs the boolean cw or ccw as true/false.
// Author: Mikhail Rego
// Date: 2025-01-27
module encoder(
    input logic a, b, clk,         // Inputs: a, b from encoder, clock signal
    output logic cw, ccw          // Outputs: clockwise and counterclockwise pulses
);

    // Internal variables to store previous states
    // i removed the redundant "logic prev_a, prev_b" because state and prev_state captures them and more
    logic [1:0] state, prev_state;
    logic idle;

    // Sequential logic block to update previous states
    always_ff @(posedge clk) begin
        prev_state <= state;      // Save the current state as previous state
        state <= {a, b};          // Update the current state, order doesn't matter, but must be in the same always_ff
    end

    // Combinational logic block to determine rotation direction
    always_comb begin
        idle = (state == prev_state); // Set idle to true if state hasn't changed

        // Default outputs to 0
        cw = 1'b0;
        ccw = 1'b0;

        // Only check rotation when rotary device is not idle
        if (!idle) begin 
            // the old logic was "case ({a, b, prev_a, prev_b})", but the next is the same
            case ({state, prev_state})
                4'b1000, 4'b1110, 4'b0111, 4'b0001: cw = 1'b1;  // Clockwise rotation patterns, see Google Photos
                4'b1011, 4'b0010, 4'b0100, 4'b1101: ccw = 1'b1; // Counterclockwise rotation patterns, see Google Photos
                default: begin
                    cw = 1'b0;   // No rotation
                    ccw = 1'b0;
                end
            endcase
        end
    end

endmodule
