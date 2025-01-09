`include "./src/include/DEF.sv"
`include "./src/include/intf.sv"
`include "./src/PC.sv"
`include "./src/ALU.sv"
`include "./src/ImmExt.sv"
`include "./src/LDFilter.sv"
`include "./src/Memory.sv"
`include "./src/RegFile.sv"
`include "./src/BranchComp.sv"
`include "./src/Controller.sv"
`include "./src/Reg_D.sv"
`include "./src/Reg_M.sv"
`include "./src/Reg_E.sv"
`include "./src/Reg_W.sv"
module Core
    import DEF::*;
(
    input  logic clk,
    input  logic rst,
    output logic halt
);
    /************************************************************/
    /* example signal (you can add or del variable if you need) */
    /* check the signal is connected to the right position      */
    /* hint: notice some similar signal name and their stage    */
    /* example: current_pc_D, current_pc_E, current_pc_M ...    */
    /************************************************************/

    logic reg_w_en, stall;
    wb_sel_t wb_sel;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [31:0] inst, inst_dummy, inst_D;
    /* verilator lint_on UNUSEDSIGNAL */
    next_pc_sel_t next_pc_sel;
    dw next_pc, current_pc, current_pc_plus_4, alu_out_as_pc, current_pc_D, current_pc_E, current_pc_M, current_pc_W;
    dw imm_ext_out, reg_write_data, rs1_read_data, rs2_read_data, reg_a0, reg_a1;
    dw alu_out, dm_read_data, ldfilter_out_data, alu_out_W;
    dw rs1_data_D, rs2_data_D;
    D_data_sel D_rs1_data_sel, D_rs2_data_sel;
    dw rs1_data_E, rs2_data_E, imm_ext_out_E;
    E_data_sel E_rs1_data_sel, E_rs2_data_sel;
    dw rs1_or_current_pc_or_zero, rs2_or_imm, newest_rs1_data, newest_rs2_data, alu_out_M;
    alu_op1_sel_t alu_op1_sel;
    alu_op2_sel_t alu_op2_sel;
    logic is_lui;
    alu_control_packet_t alu_control;
    dw rs2_data_M, ld_data_W;;
    logic [2:0] W_f3;
    logic [4:0] rs1_index, rs2_index, W_rd;
    logic [7:0] im_w_mask, dm_w_mask;
    /************************************************************/
    //TODO
endmodule : Core
