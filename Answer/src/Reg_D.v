module Reg_D(
    input clk,
    input rst,
    input [63:0] current_pc,
    input [31:0] inst,
    input stall,
    input jb,
    //
    input waiting,
    //
    output reg [63:0] current_pc_D,
    output reg [31:0] inst_D
);
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            current_pc_D <= 64'd0;
            inst_D <= {25'd0, 7'b0010011};
        end
        else if(stall || waiting) begin
            current_pc_D <= current_pc_D;
            inst_D <= inst_D;
        end
        else if(jb) begin
            current_pc_D <= current_pc_D;
            inst_D <= {25'd0, 7'b0010011};
        end
        else begin
            current_pc_D <= current_pc;
            inst_D <= inst;
        end
    end
endmodule