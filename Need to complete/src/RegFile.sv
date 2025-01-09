module RegFile
    import DEF::*;
(
    input logic clk,
    input logic rst,
    input logic [4:0] rs1_index,
    input logic [4:0] rs2_index,
    input logic w_en,
    input logic [4:0] rd_index,
    input dw rd_data,
    output dw rs1_data,
    output dw rs2_data,
    output dw reg_a0,
    output dw reg_a1
);
    dw mem[32];  // register file
    always_ff @(posedge clk or negedge rst) begin
        if (!rst) begin
            for (int i = 0; i < 32; i++) begin
                mem[i] <= 64'd0;
            end
        end else begin
            if (rd_index != 5'd0 && w_en == 1'b1) begin
                mem[rd_index] <= rd_data;
            end
        end
    end
    /* assign output read data */
    assign rs1_data = mem[rs1_index];
    assign rs2_data = mem[rs2_index];
    /* for ECALL handling */
    assign reg_a0   = mem[10];
    assign reg_a1   = mem[11];
endmodule
