module Controller
    import DEF::*;
(
    input logic clk,//
    input logic rst,//
    /* inst information */
    input inst_t inst,//
    /* a0 and a1 for ECALL handle */
    input dw reg_a0,
    input dw reg_a1,
    /* next PC select */
    output next_pc_sel_t next_pc_sel,//
    /* IM write control */
    output logic [7:0] im_w_mask,//
    /* Register File Control */
    output logic reg_w_en,//
    /* ALU control */
    output alu_op1_sel_t alu_op1_sel,//
    output alu_op2_sel_t alu_op2_sel,//
    output logic is_lui,//
    output alu_control_packet_t alu_control,// E_op, E_f3, E_f7
    /* Branch Comparator control */
    BranchCompControlIntf.ControllerSide bc_control,//
    /* DM write control */
    output logic [7:0] dm_w_mask, //
    /* write-back select */
    output wb_sel_t wb_sel, //
    /* halt signal (for testbench to monitor) */
    output logic halt,
    output logic stall,//
    output logic [4:0] W_rd,//
    output logic [2:0] W_f3,//
    output D_data_sel D_rs1_data_sel,//
    output D_data_sel D_rs2_data_sel,//
    output E_data_sel E_rs1_data_sel,//
    output E_data_sel E_rs2_data_sel//
);
    /************************************************************/
    /* example signal (you can add or del variable if you need) */
    /************************************************************/

    logic [6:0] D_op, E_op, M_op, W_op; 
    logic [2:0] D_f3, E_f3, M_f3;
    logic [4:0] D_rd, E_rd, M_rd, E_rs1, E_rs2, D_rs1, D_rs2;
    logic is_D_use_rs1, is_D_use_rs2, is_W_use_rd;
    logic is_E_use_rs1, is_E_use_rs2, is_M_use_rd;
    logic D_f7, E_f7;
    logic is_D_rs1_W_rd_overlap, is_D_rs2_W_rd_overlap;
    logic is_E_rs1_W_rd_overlap, is_E_rs1_M_rd_overlap, is_E_rs2_W_rd_overlap, is_E_rs2_M_rd_overlap;
    logic is_DE_overlap, is_D_rs1_E_rd_overlap, is_D_rs2_E_rd_overlap;
    /************************************************************/
    //TODO

    
/* ECALL handling */
    always_latch begin
        if (inst.I_TYPE.opcode == SYSTEM && inst.I_TYPE.imm_11_0 == ECALL_FUNC12) begin
            if (reg_a0 == 64'd0) begin : ECALL_to_halt
                halt = 1'b1;
            end : ECALL_to_halt
            else if (reg_a0 == 64'd1) begin : ECALL_to_putchar
                $display("%c", reg_a1[7:0]);
            end : ECALL_to_putchar
            else begin : ECALL_not_support
                $display("Not supported ECALL service request type!\n");
                $finish;
            end : ECALL_not_support
        end
    end
endmodule
