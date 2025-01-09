module Reg_PC(
    input clk,
    input rst,
    input [63:0] next_pc,
    input stall,
    //
    input waiting,
    //
    output reg [63:0] current_pc
    
);
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            current_pc <= 64'd0;
        end
        else if(stall || waiting) begin
            current_pc <= current_pc;
        end
        else begin
            current_pc <= next_pc;
        end
    end
endmodule