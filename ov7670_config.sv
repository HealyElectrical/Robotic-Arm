module ov7670_config (
    input  logic clk,
    input  logic reset_n,
    output logic SIOC,          // SCCB clock
    inout  tri   SIOD,          // SCCB data (tri-state)
    output logic done           // High when config is complete
);

    typedef enum logic [2:0] {
        IDLE, START, WRITE_ADDR, WRITE_REG, WRITE_DATA, STOP, DONE
    } state_t;

    state_t state;
    logic [7:0] reg_addr;
    logic [7:0] reg_data;
    logic [3:0] bit_cnt;

    logic [23:0] write_buf [0:4]; // {DeviceAddr, RegAddr, Data}
    logic [2:0] config_index;

    // SCCB Device Address
    localparam logic [7:0] DEV_ADDR = 8'h42;
	 
	 logic tick;  // this will pulse at ~50kHz (20 µs intervals)

sccb_clockgen clkgen_inst (
    .clk(clk),
    .reset_n(reset_n),
    .tick(tick)
);

    // Initialization block
    initial begin
        write_buf[0] = {8'h42, 8'h12, 8'h80}; // Reset
        write_buf[1] = {8'h42, 8'h12, 8'h14}; // QVGA RGB
        write_buf[2] = {8'h42, 8'h11, 8'h01}; // Clock prescale
        write_buf[3] = {8'h42, 8'h6B, 8'h4A}; // PLL
        write_buf[4] = {8'h42, 8'h0C, 8'h04}; // RGB565
    end

    // Simplified FSM, just stubs logic (you will fill in actual timing and tri-state logic)
    always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= IDLE;
        config_index <= 0;
        done <= 0;
    end else if (tick) begin   // <- only advance on tick!
        case (state)
            IDLE: begin
                if (config_index < 5)
                    state <= START;
                else
                    state <= DONE;
            end
            START: begin
                state <= WRITE_ADDR;
            end
            WRITE_ADDR: begin
                state <= WRITE_REG;
            end
            WRITE_REG: begin
                state <= WRITE_DATA;
            end
            WRITE_DATA: begin
                state <= STOP;
            end
            STOP: begin
                config_index <= config_index + 1;
                state <= IDLE;
            end
            DONE: begin
                done <= 1;
            end
        endcase
    end
end


endmodule
