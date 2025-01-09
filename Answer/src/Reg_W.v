module Reg_W(
    input clk,
    input rst,
    input [63:0] alu_out_M,
    input [63:0] ld_data_M,
    //
    input waiting,
    //
    output reg [63:0] alu_out_W,
    output reg [63:0] ld_data_W
);
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            alu_out_W <= 64'd0;
            ld_data_W <= 64'd0;
        end
        else begin
            if(waiting) begin
                alu_out_W <= alu_out_W;
                ld_data_W <= ld_data_W;
            end
            else begin
                alu_out_W <= alu_out_M;
                ld_data_W <= ld_data_M;
            end
        end
    end
endmodule