// adcInterface.sv
// 2/9/2025
// By: Mikhail R
// Description: Implements FSM for LTC2308 ADC interface
// 	Continuously samples the requested ADC channel and outputs the conversion result
/////////////////////////////////////////////////////////////////////////////////////
module adcinterface(
   input logic clk, reset_n,   // reset_n is active low
   input logic [2:0] chan,     // ADC channel requested by user
   output logic [11:0] result, // ADC conversion result
   output logic ADC_CONVST, ADC_SCK, ADC_SDI, // Start-adc-Conversion bit, adc-clk, adc-in
   input logic ADC_SDO         // ADC output (to be loaded into result)
);

   // State definitions
   typedef enum logic [2:0] {HOLD, CONVST_HIGH, CONVST_LOW, TRANSFER, WAIT} state_t;
   state_t state, nextState;

   logic [3:0] transferCount;  // Counts the 12 cycles of SCK
   logic [11:0] configWord;    // 6-bit config + 6-bit padding
   logic [11:0] tempResult;    // Temporary result storage

   // Next-State Logic & Combinational Outputs
   always_comb begin
      nextState = state;
      ADC_CONVST = 0;

      case (state)
         HOLD:        nextState = CONVST_HIGH;
         CONVST_HIGH: begin nextState = CONVST_LOW; ADC_CONVST = 1; end
         CONVST_LOW:  nextState = TRANSFER;
         TRANSFER:    nextState = (transferCount >= 4'd12) ? WAIT : TRANSFER;
         WAIT:        nextState = CONVST_HIGH;
         default:     nextState = HOLD;
      endcase
   end

   // State Register
   always_ff @(negedge clk or negedge reset_n) begin
      if (!reset_n) begin
         state <= HOLD;
         //transferCount <= 4'b0;
         configWord <= 12'b0;
      end else begin
         state <= nextState;
         if (state == CONVST_HIGH)
            configWord <= {1'b1, chan[0], chan[2:1], 1'b1, 1'b0, 6'b000000};
      end
   end

   // ADC SDI Logic
  always_ff @(negedge clk or negedge reset_n) begin
      if (!reset_n) begin
         ADC_SDI <= 0;
      end else if (state == CONVST_LOW) begin
         ADC_SDI <= configWord[11]; // Preload MSB
      end else if (state == TRANSFER && transferCount < 12) begin
         ADC_SDI <= configWord[11 - transferCount]; // Send next bit
      end else begin
         ADC_SDI <= 0;
      end
   end
	


   // Capture ADC_SDO at posedge clk
   always_ff @(posedge clk or negedge reset_n) begin
      if (!reset_n) begin
         tempResult <= 12'b0;
      end else if (state == TRANSFER && transferCount < 12) begin
         tempResult[11 - transferCount] <= ADC_SDO;
      end
   end

   // Result register (fixing multiple drivers issue)
   always_ff @(posedge clk or negedge reset_n) begin
      if (!reset_n) begin
         result <= 12'b0;
      end else if (state == TRANSFER && transferCount == 11) begin
         result <= tempResult;
      end
   end

   // Transfer count logic
   always_ff @(posedge clk or negedge reset_n) begin
      if (!reset_n) begin
         transferCount <= 4'b0;
      end else if (state == TRANSFER && transferCount < 12) begin
         transferCount <= transferCount + 4'b1;
      end else if (state == WAIT) begin
         transferCount <= 4'b0;
      end
   end

   // SPI Clock (SCK) generation
   assign ADC_SCK = (state == TRANSFER) ? clk : 1'b0;

endmodule
