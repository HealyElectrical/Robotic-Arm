module sccb_clockgen (
    input  logic clk,        // 50 MHz input clock
    input  logic reset_n,
    output logic tick        // 1-cycle pulse every 20 µs
);
    // For 50 kHz tick, we need 50 MHz / 50 kHz = 1000 cycles
    localparam DIVIDER = 1000;

    logic [15:0] counter;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            counter <= 0;
            tick <= 0;
        end else begin
            if (counter == DIVIDER - 1) begin
                tick <= 1;
                counter <= 0;
            end else begin
                tick <= 0;
                counter <= counter + 1;
            end
        end
    end
endmodule

