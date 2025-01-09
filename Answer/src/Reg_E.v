module Reg_E(
    input clk,
    input rst,
    input [63:0] current_pc_D,
    input [63:0] rs1_data_D,
    input [63:0] rs2_data_D,
    input [63:0] sext_imm_D,
    input stall,
    input jb,
    //
    input waiting,
    //
    output reg [63:0] current_pc_E,
    output reg [63:0] rs1_data_E,
    output reg [63:0] rs2_data_E,
    output reg [63:0] sext_imm_E
);
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            current_pc_E <= 64'd0;
            rs1_data_E <= 64'd0;
            rs2_data_E <= 64'd0;
            sext_imm_E <= 64'd0;
        end
        else if (waiting) begin
            current_pc_E <= current_pc_E;
            rs1_data_E <= rs1_data_E;
            rs2_data_E <= rs2_data_E;
            sext_imm_E <= sext_imm_E;
        end
        else if(stall || jb) begin
            current_pc_E <= current_pc_E;
            rs1_data_E <= 64'd0;
            rs2_data_E <= 64'd0;
            sext_imm_E <= 64'd0;
        end
        else begin
            current_pc_E <= current_pc_D;
            rs1_data_E <= rs1_data_D;
            rs2_data_E <= rs2_data_D;
            sext_imm_E <= sext_imm_D;
        end
    end
endmodule