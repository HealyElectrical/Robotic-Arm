//==========================================================================
// adcInterface.sv
// Author     : Mikhail Rego
// Date       : 02/09/2025
// Description: FSM to interface with LTC2308 ADC. Continuously samples the
//              selected ADC channel and outputs the 12-bit conversion result.
//==========================================================================

module adcInterface (
    input  logic        clk,
    input  logic        reset_n,         // Active-low reset
    input  logic [2:0]  chan,            // Requested ADC channel (0–7)
    output logic [11:0] result,          // Final ADC result (12 bits)

    output logic        ADC_CONVST,      // Start-conversion signal
    output logic        ADC_SCK,         // Serial clock to ADC
    output logic        ADC_SDI,         // Serial data input to ADC
    input  logic        ADC_SDO          // Serial data output from ADC
);

    // === FSM Declaration ===
    typedef enum logic [2:0] {
        HOLD, CONVST_HIGH, CONVST_LOW, TRANSFER, WAIT
    } state_t;

    state_t state, nextState;

    // === Internal Registers ===
    logic [3:0]  transferCount;   // Counts 12 SCK cycles
    logic [11:0] configWord;      // Configuration word sent to ADC
    logic [11:0] tempResult;      // Temporary capture of ADC result

    // ============================================================================
    // FSM Next-State Logic & ADC_CONVST Control
    // ============================================================================
    always_comb begin
        nextState   = state;
        ADC_CONVST  = 1'b0;

        case (state)
            HOLD:         nextState = CONVST_HIGH;

            CONVST_HIGH: begin
                nextState  = CONVST_LOW;
                ADC_CONVST = 1'b1;  // Pulse high
            end

            CONVST_LOW:   nextState = TRANSFER;

            TRANSFER:     nextState = (transferCount >= 4'd12) ? WAIT : TRANSFER;

            WAIT:         nextState = CONVST_HIGH;

            default:      nextState = HOLD;
        endcase
    end

    // ============================================================================
    // State Register and Config Word Update
    // ============================================================================
    always_ff @(negedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state      <= HOLD;
            configWord <= 12'b0;
        end else begin
            state <= nextState;

            if (state == CONVST_HIGH) begin
                // Format: [S=1, SEQ=chan[0], BIP=chan[2], UNI=chan[1], SGL=1, x=0] + padding
                configWord <= {1'b1, chan[0], chan[2:1], 1'b1, 1'b0, 6'b000000};
            end
        end
    end

    // ============================================================================
    // Serial Data Output (ADC_SDI)
    // Send 12-bit configWord during TRANSFER phase
    // ============================================================================
    always_ff @(negedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ADC_SDI <= 1'b0;
        end else if (state == CONVST_LOW) begin
            ADC_SDI <= configWord[11];  // Load MSB before transfer
        end else if (state == TRANSFER && transferCount < 12) begin
            ADC_SDI <= configWord[11 - transferCount];
        end else begin
            ADC_SDI <= 1'b0;
        end
    end

    // ============================================================================
    // Serial Data Input (ADC_SDO) → Capture ADC bits during TRANSFER
    // ============================================================================
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            tempResult <= 12'b0;
        end else if (state == TRANSFER && transferCount < 12) begin
            tempResult[11 - transferCount] <= ADC_SDO;
        end
    end

    // ============================================================================
    // Final ADC Result Latching
    // ============================================================================
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            result <= 12'b0;
        end else if (state == TRANSFER && transferCount == 11) begin
            result <= tempResult;
        end
    end

    // ============================================================================
    // Transfer Counter: Tracks 12-bit SPI transfer
    // ============================================================================
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            transferCount <= 4'd0;
        end else if (state == TRANSFER && transferCount < 12) begin
            transferCount <= transferCount + 1;
        end else if (state == WAIT) begin
            transferCount <= 4'd0;
        end
    end

    // ============================================================================
    // ADC SPI Clock Output
    // ============================================================================
    assign ADC_SCK = (state == TRANSFER) ? clk : 1'b0;

endmodule
