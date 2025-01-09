module Reg_M(
    input clk,
    input rst,
    input [63:0] alu_out_E,
    input [63:0] rs2_data_E,
    //
    input waiting,
    //
    output reg [63:0] alu_out_M,
    output reg [63:0] rs2_data_M
);
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            alu_out_M <= 64'd0;
            rs2_data_M <= 64'd0;
        end
        else begin
            if(waiting) begin
                alu_out_M <= alu_out_M;
                rs2_data_M <= rs2_data_M;
            end
            else begin
                alu_out_M <= alu_out_E;
                rs2_data_M <= rs2_data_E;  
            end
        end
    end
endmodule