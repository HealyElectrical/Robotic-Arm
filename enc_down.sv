//==========================================================================
// enc_down.sv
// Author     : Mikhail Rego (with help from VE)
// Date       : 04/06/2025
// Description: Downsamples noisy rotary encoder pulses by counting
//              3 consecutive pulses in either CW or CCW direction,
//              then outputs a single 1-cycle pulse (`cw_out` or `ccw_out`).
//==========================================================================

module enc_down (
    input  logic clk,
    input  logic reset_n,
    input  logic cw_in, ccw_in,      // Inputs from rotary decoder (1 pulse per transition)
    output logic cw_out, ccw_out     // One-shot output after N counts
);

    logic [1:0] cw_count, ccw_count; // 2-bit counters for up to 3 pulses

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            cw_count  <= 0;
            ccw_count <= 0;
            cw_out    <= 0;
            ccw_out   <= 0;
        end else begin
            // Default outputs LOW every cycle (single-cycle pulse)
            cw_out  <= 0;
            ccw_out <= 0;

            if (cw_in) begin
                cw_count  <= cw_count + 1;
                ccw_count <= 0; // Reset opposite direction

                if (cw_count >= 2'd2) begin  // 3+ pulses → output 1 cycle
                    cw_out   <= 1;
                    cw_count <= 0;
                end
            end
            else if (ccw_in) begin
                ccw_count <= ccw_count + 1;
                cw_count  <= 0;

                if (ccw_count >= 2'd2) begin
                    ccw_out   <= 1;
                    ccw_count <= 0;
                end
            end
            // Else: no input, counts persist
        end
    end

endmodule
