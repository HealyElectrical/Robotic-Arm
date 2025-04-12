module frame_buffer (
    input  logic        wr_clk,      // Write clock (CAM_PCLK)
    input  logic        wr_en,       // Write enable
    input  logic [16:0] wr_addr,     // Write address (17 bits for 320x240 = 76800)
    input  logic [15:0] wr_data,     // Pixel data (RGB565)

    input  logic        rd_clk,      // Read clock (e.g., system clock or HPS)
    input  logic [16:0] rd_addr,     // Read address
    output logic [15:0] rd_data      // Output pixel data
);

    // Frame buffer: 76800 pixels (QGVA), 16 bits each
    logic [15:0] mem [0:76800-1];

    // Write port
    always_ff @(posedge wr_clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // Read port
    always_ff @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule
