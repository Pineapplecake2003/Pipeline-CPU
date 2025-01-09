module ImmExt
    import DEF::*;
(
    input inst_t inst,
    output dw imm_ext_out
);
    logic [6:0] opcode;
    dw tmp_imm_ext_out;

    assign opcode = inst.R_TYPE.opcode;

    always_comb begin : main_block
        unique if (opcode == OP || opcode == OP_32 || opcode == SYSTEM) begin : noimm
            /* R-type and SYSTEM type */
            tmp_imm_ext_out = 64'd0;
        end : noimm
        else if (opcode == JAL) begin : jal
            /* J-type */
            tmp_imm_ext_out = {
                {44{inst.J_TYPE.imm_20}},
                inst.J_TYPE.imm_19_12,
                inst.J_TYPE.imm_11,
                inst.J_TYPE.imm_10_1,
                1'b0
            };
        end : jal
        else if (opcode == STORE) begin : store
            /* S-type */
            tmp_imm_ext_out = {
                {52{inst.S_TYPE.imm_11_5[6]}}, inst.S_TYPE.imm_11_5, inst.S_TYPE.imm_4_0
            };
        end : store
        else if (opcode == BRANCH) begin : branch
            /* B-type */
            tmp_imm_ext_out = {
                {52{inst.B_TYPE.imm_12}},
                inst.B_TYPE.imm_11,
                inst.B_TYPE.imm_10_5,
                inst.B_TYPE.imm_4_1,
                1'b0
            };
        end : branch
        else if (opcode == LUI || opcode == AUIPC) begin : u_type
            /* U-type */
            tmp_imm_ext_out = {{32{inst.U_TYPE.imm_31_12[19]}}, inst.U_TYPE.imm_31_12, 12'b0};
        end : u_type
        else begin : i_type
            /* I-type */
            /* includes OP_IMM, OP_IMM_32, LOAD, JALR*/
            tmp_imm_ext_out = {{52{inst.I_TYPE.imm_11_0[11]}}, inst.I_TYPE.imm_11_0};
        end : i_type
    end : main_block

    assign imm_ext_out = tmp_imm_ext_out;
endmodule
