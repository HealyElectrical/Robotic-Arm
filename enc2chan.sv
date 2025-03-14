/////////////////////////////////////////////////////////////////////////////////////
// enc2chan.sv
// Author: Glen Healy (Modified by ChatGPT)
// Date: February 16, 2025
//
// Description: 
// This module converts rotary encoder pulses into a 3-bit ADC channel selection index 
// (0 to 7). The encoder generates pulses indicating clockwise (cw) or 
// counterclockwise (ccw) movement. The module increments or decrements the 
// channel index accordingly while ensuring it stays within the valid range.
//
// - On reset, the channel defaults to 0 and remains inactive.
// - The first **clockwise (cw) pulse** activates channel selection and increments 
//   the channel index, capping at channel 7.
// - A **counterclockwise (ccw) pulse** decrements the index down to channel 0.
// - The `active` flag ensures the channel remains at 0 until the first CW pulse.
//
// This module allows users to cycle through ADC channels via an encoder, 
// providing a dynamic way to select input sources for analog-to-digital conversion.
/////////////////////////////////////////////////////////////////////////////////////
// Encoder to Channel Module (similar to enc2freq structure)
module enc2chan (
    input logic clk,        // System clock
    input logic reset_n,    // Active-low reset
    input logic cw, ccw,    // Clockwise and counterclockwise pulses from encoder
    output logic [2:0] chan // Output ADC channel (0 to 7)
);

    logic [2:0] chan_index;  // 3-bit index for ADC channels
    logic active;            // Flag to indicate active channel selection

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            chan_index <= 3'd0;  // Default to channel 0
            active <= 1'b0;      // Inactive initially
        end 
        else begin
            if (cw) begin
                if (!active) begin
                    active <= 1'b1; // Activate channel selection on first CW pulse
                end else if (chan_index < 3'd7) begin
                    chan_index <= chan_index + 1; // Increment channel
                end
            end 
            else if (ccw) begin
                if (chan_index > 3'd0) begin
                    chan_index <= chan_index - 1; // Decrement channel
                end
            end
        end
    end

    // Assign channel based on active state
    assign chan = active ? chan_index : 3'd0;

endmodule


