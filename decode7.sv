//==========================================================================
// decode7.sv
// Author     : Glen Healy
// Date       : Jan 21, 2025
// Course     : ELEX 7660 - Digital System Design
// Description: Converts a 4-bit binary input (`num`) into a 7-segment display
//              pattern for a **common cathode** display. Output (`leds`) is
//              active-low: LOW = segment ON.
//==========================================================================

module decode7 (
    input  logic [3:0] num,   // 4-bit binary input (0–15)
    output logic [7:0] leds   // 7-segment output (abcdefg + DP)
);

    // Pure combinational logic: maps `num` to segment pattern
    always_comb begin
        case (num)
            4'h0: leds = ~8'b11000000; // 0
            4'h1: leds = ~8'b11111001; // 1
            4'h2: leds = ~8'b10100100; // 2
            4'h3: leds = ~8'b10110000; // 3
            4'h4: leds = ~8'b10011001; // 4
            4'h5: leds = ~8'b10010010; // 5
            4'h6: leds = ~8'b10000010; // 6
            4'h7: leds = ~8'b11111000; // 7
            4'h8: leds = ~8'b10000000; // 8
            4'h9: leds = ~8'b10010000; // 9
            4'hA: leds = ~8'b10001000; // A
            4'hB: leds = ~8'b10000011; // B
            4'hC: leds = ~8'b11000110; // C
            4'hD: leds = ~8'b10100001; // D
            4'hE: leds = ~8'b10000110; // E
            4'hF: leds = ~8'b10001110; // F
            default: leds = ~8'b11111111; // All segments OFF
        endcase
    end

endmodule
