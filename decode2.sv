//==========================================================================
// decode2.sv
// Author     : Glen Healy
// Date       : Jan 21, 2025
// Course     : ELEX 7660 - Digital System Design
// Description: Activates one of four 7-segment display digits based on a
//              2-bit selector. Output `ct` is active-low (0 = enabled).
//==========================================================================

module decode2 (
    input  logic [1:0] digit, // 2-bit selector (0–3)
    output logic [3:0] ct     // Active-low digit enable (one-hot)
);

    // Combinational logic to drive digit select lines
    always_comb begin
        case (digit)
            2'b00:    ct = 4'b0111; // Enable digit 0
            2'b01:    ct = 4'b1011; // Enable digit 1
            2'b10:    ct = 4'b1101; // Enable digit 2
            2'b11:    ct = 4'b1110; // Enable digit 3
            default:  ct = 4'b1111; // Disable all digits (safe default)
        endcase
    end

endmodule
