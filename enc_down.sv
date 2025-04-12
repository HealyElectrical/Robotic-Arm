// enc_down.sv
// 4/6/2025
// By: Mikhail R, with the help of VE
// takes-in 4 rotary encoder inputs, and outputs 1 cw or ccw input
// it resets all values after one clock cycle of outputting "cw_out," or "ccw_out"
////////////////////////////////////////////////////////////////////////////////////

module enc_down (
    input  logic clk, reset_n,
    input  logic cw_in, ccw_in,      // Inputs from quadrature decoder (1 pulse per phase change)
    output logic cw_out, ccw_out     // 1-cycle pulse every 4 inputs
);
    logic [1:0] cw_count, ccw_count;  // Enough for 0–3

    always_ff @(posedge clk) begin
        // Default outputs LOW each cycle (1-cycle pulse)
        cw_out  <= 0;
        ccw_out <= 0;

        if (cw_in) begin
            cw_count <= cw_count + 1;
            ccw_count <= 0; // reset opposite dir

            if (cw_count >= 2'd2) begin  // 0,1,2,3 = 4 pulses... but 3 work better...
                cw_out  <= 1;
                cw_count <= 0; // reset for next group
            end
        end
        else if (ccw_in) begin
            ccw_count <= ccw_count + 1;
            cw_count <= 0; // reset opposite dir

            if (ccw_count >= 2'd2) begin
                ccw_out <= 1;
                ccw_count <= 0;
            end
        end
        else begin
            // no input: do nothing; counts persist
        end
    end

endmodule