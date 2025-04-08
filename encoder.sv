module encoder (
    input  logic clk,    // Clock input
    input  logic a, b,   // Quadrature encoder signals
    output logic cw,     // Clockwise pulse output (one per full step)
    output logic ccw     // Counterclockwise pulse output (one per full step)
);

    // ------------------------------------------------------
    // ** State Registers **
    // ------------------------------------------------------
    logic [1:0] prev_state, curr_state;  // Tracks encoder position
    logic [1:0] CWcounter = 2'b00;       // Tracks CW steps (max 4)
    logic [1:0] CCWcounter = 2'b00;      // Tracks CCW steps (max 4)

    // ------------------------------------------------------
    // ** Update Encoder State on Clock Edge **
    // ------------------------------------------------------
    always_ff @(posedge clk) begin
        prev_state <= curr_state;  
        curr_state <= {a, b};  // Capture new state
    end

    // ------------------------------------------------------
    // ** Direction Detection & Pulse Output Logic **
    // ------------------------------------------------------
    always_ff @(posedge clk) begin
        cw  <= 1'b0;  // Default: No pulse
        ccw <= 1'b0;

        // **Detect valid transitions**
        case ({curr_state, prev_state})
            // **Clockwise Movement Pattern** (Gray Code: 00 → 10 → 11 → 01 → 00)
            4'b10_00, 4'b11_10, 4'b01_11, 4'b00_01: CWcounter <= CWcounter + 1;

            // **Counterclockwise Movement Pattern** (Gray Code: 00 → 01 → 11 → 10 → 00)
            4'b01_00, 4'b11_01, 4'b10_11, 4'b00_10: CCWcounter <= CCWcounter + 1;

            default: begin
                CWcounter <= CWcounter;
                CCWcounter <= CCWcounter;
            end
        endcase

        // **Output pulse after 4 transitions (1 full step)**
        if (CWcounter == 2'b11) begin  
            cw <= 1'b1;     // **Send CW pulse**
            CWcounter <= 2'b00;  // **Reset counter**
        end

        if (CCWcounter == 2'b11) begin  
            ccw <= 1'b1;    // **Send CCW pulse**
            CCWcounter <= 2'b00;  // **Reset counter**
        end
    end

endmodule