module JB_Unit(
    input [63:0] operand1,
    input [63:0] operand2,
    output [63:0] jb_out
);
    assign jb_out = (operand1 + operand2) & (~64'd1);
endmodule