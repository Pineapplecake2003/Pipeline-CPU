module Imm_Ext(
    input [31:0] inst,
    output reg [63:0] imm_ext_out
);
    always @(*) begin
        case(inst[6:2])
            `I_type_W: begin
                imm_ext_out = {{52{inst[31]}},inst[31:20]};
            end
            `I_type_arithmetic: begin
                imm_ext_out = {{52{inst[31]}},inst[31:20]};
            end
            `I_type_load: begin
                imm_ext_out = {{52{inst[31]}},inst[31:20]};
            end
            `JALR: begin
                imm_ext_out = {{52{inst[31]}},inst[31:20]};
            end
            `S_type: begin
                imm_ext_out = {{52{inst[31]}},inst[31:25],inst[11:7]};
            end
            `B_type: begin
                imm_ext_out = {{52{inst[31]}},inst[7],inst[30:25],inst[11:8],1'b0};
            end
            `LUI: begin
                imm_ext_out = {{32{inst[31]}},inst[31:12],12'b0};
            end
            `AUIPC: begin
                imm_ext_out = {{32{inst[31]}},inst[31:12],12'b0};
            end
            `JAL: begin
                imm_ext_out = {{44{inst[31]}},inst[19:12],inst[20],inst[30:21],1'b0};
            end
            default: begin
                imm_ext_out = 64'd0;
            end
        endcase
    end 
endmodule