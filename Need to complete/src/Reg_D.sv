module Reg_D
    import DEF::*;
(
    input logic clk,
    input logic rst,
    input dw current_pc,
    input logic [31:0] inst,
    input logic stall,
    input logic jb,
    output dw current_pc_D,
    output logic [31:0] inst_D
);
//TODO
endmodule