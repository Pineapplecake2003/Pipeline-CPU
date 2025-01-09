`include "./src/SRAM.v"
`include "./src/Reg_File.v"
`include "./src/ALU.v"
`include "./src/Decoder.v"
`include "./src/Imm_Ext.v"
`include "./src/JB_Unit.v"
`include "./src/Reg_PC.v"
`include "./src/LD_Filter.v"
`include "./src/Controller.v"
`include "./src/Reg_D.v"
`include "./src/Reg_E.v"
`include "./src/Reg_M.v"
`include "./src/Reg_W.v"
`include "./src/Data_mem.v"
`include "./src/dcache.v"

module Top (
    input clk,
    input rst
);
    wire [63:0] current_pc, current_pc_D, current_pc_E;
    wire [31:0] inst_D;
    wire [63:0] read_data, immediate, wb_data, sext_imm_E;
    wire [63:0] rs1, rs2, rs1_data_D, rs2_data_D, rs1_data_E, rs2_data_E,  rs2_data_M;
    wire [63:0] alu_op1, alu_op2, jb_op1, jb_pc, pc_plus4, next_pc, alu_out_E, alu_out_M, alu_out_W, ld_data, ld_data_W, ld_data_f;
    wire [4:0] rs1_index, rs2_index, W_rd_index, rd_index, de_opcode, E_op;
    wire [7:0] M_dm_w_en, F_im_w_en;
    wire [2:0] de_func3, E_f3, W_f3;
    wire [1:0] E_rs1_data_sel, E_rs2_data_sel;
    wire E_alu_op1_sel, E_alu_op2_sel, E_jb_op1_sel, next_pc_sel, W_wb_sel, de_func7, W_wb_en, E_f7, stall, alu_out_zero;
    wire D_rs1_data_sel, D_rs2_data_sel;
    reg [63:0] newest_rs1_data, newest_rs2_data;

    wire waiting, read, data_ready;

    Reg_PC reg_pc(
        .clk(clk), 
        .rst(rst), 
        .next_pc(next_pc), 
        .stall(stall),
        .current_pc(current_pc),
        .waiting(waiting)
    );
    SRAM im(
        .clk(clk), 
        .w_en(F_im_w_en), 
        .address(current_pc), 
        .write_data(64'd0), 
        .read_data(read_data)
    );
    Reg_D reg_D(
        .clk(clk),
        .rst(rst),
        .current_pc(current_pc),
        .inst(read_data[31:0]),
        .stall(stall),
        .jb(next_pc_sel),
        .current_pc_D(current_pc_D),
        .inst_D(inst_D),
        .waiting(waiting)
    );
    Decoder decoder(
        .inst(inst_D), 
        .dc_out_opcode(de_opcode), 
        .dc_out_func3(de_func3), 
        .dc_out_func7(de_func7),
        .dc_out_rs1_index(rs1_index),
        .dc_out_rs2_index(rs2_index),
        .dc_out_rd_index(rd_index)
    );
    Imm_Ext imm_ext(
        .inst(inst_D),
        .imm_ext_out(immediate)
    );
    Reg_File reg_file(
        .clk(clk),
        .rst(rst),
        .wb_en(W_wb_en),
        .wb_data(wb_data),
        .rd_index(W_rd_index),
        .rs1_index(rs1_index),
        .rs2_index(rs2_index),
        .rs1_data_out(rs1),
        .rs2_data_out(rs2)
    );
    //MUX2
    assign rs1_data_D = (D_rs1_data_sel)? wb_data:rs1;
    assign rs2_data_D = (D_rs2_data_sel)? wb_data:rs2;

    Reg_E reg_E(
        .clk(clk),
        .rst(rst),
        .current_pc_D(current_pc_D),
        .rs1_data_D(rs1_data_D),
        .rs2_data_D(rs2_data_D),
        .sext_imm_D(immediate),
        .stall(stall),
        .jb(next_pc_sel),
        .current_pc_E(current_pc_E),
        .rs1_data_E(rs1_data_E),
        .rs2_data_E(rs2_data_E),
        .sext_imm_E(sext_imm_E),
        .waiting(waiting)
    );

    //MUX3
    always @(*) begin
        case(E_rs1_data_sel)
        2'b00: begin
            newest_rs1_data = wb_data;
        end
        2'b01: begin
            newest_rs1_data = alu_out_M;
        end
        default: begin
            newest_rs1_data = rs1_data_E;
        end
        endcase
    end

    always @(*) begin
        case(E_rs2_data_sel)
        2'b00: begin
            newest_rs2_data = wb_data;
        end
        2'b01: begin
            newest_rs2_data = alu_out_M;
        end
        default: begin
            newest_rs2_data = rs2_data_E;
        end
        endcase
    end

    assign alu_op1 = (E_alu_op1_sel)? current_pc_E:newest_rs1_data;
    assign alu_op2 = (E_alu_op2_sel)? sext_imm_E:newest_rs2_data;
    assign jb_op1 = (E_jb_op1_sel)? current_pc_E:newest_rs1_data;

    JB_Unit jb_unit(
        .operand1(jb_op1),
        .operand2(sext_imm_E),
        .jb_out(jb_pc)
    );
    
    //MUX2
    assign pc_plus4 = current_pc + 4;
    assign next_pc = (next_pc_sel)? jb_pc:pc_plus4;

    ALU alu(
        .opcode(E_op),
        .func3(E_f3),
        .func7(E_f7),
        .operand1(alu_op1),
        .operand2(alu_op2),
        .alu_out(alu_out_E)
    );
    assign alu_out_zero = alu_out_E[0];

    Reg_M reg_M(
        .clk(clk),
        .rst(rst),
        .alu_out_E(alu_out_E),
        .rs2_data_E(newest_rs2_data),
        .alu_out_M(alu_out_M),
        .rs2_data_M(rs2_data_M),
        .waiting(waiting)
    );
    
    Reg_W reg_W(
        .clk(clk),
        .rst(rst),
        .alu_out_M(alu_out_M),
        .ld_data_M(ld_data),
        .alu_out_W(alu_out_W),
        .ld_data_W(ld_data_W),
        .waiting(waiting)
    );

    LD_Filter ld_filter(
        .func3(W_f3),
        .ld_data(ld_data_W),
        .ld_data_f(ld_data_f)
    );

    //MUX2
    assign wb_data = (W_wb_sel)? ld_data_f:alu_out_W;

    Controller controller(
        .clk(clk),
        .rst(rst),
        .de_opcode(de_opcode),
        .de_func3(de_func3),
        .de_func7(de_func7),
        .alu_out_zero(alu_out_zero),
        .de_rs1(rs1_index),
        .de_rs2(rs2_index),
        .de_rd(rd_index),
        .next_pc_sel(next_pc_sel),
        .stall(stall),
        .F_im_w_en(F_im_w_en),
        .D_rs1_data_sel(D_rs1_data_sel),
        .D_rs2_data_sel(D_rs2_data_sel),
        .E_rs1_data_sel(E_rs1_data_sel),
        .E_rs2_data_sel(E_rs2_data_sel),
        .E_jb_op1_sel(E_jb_op1_sel),
        .E_alu_op1_sel(E_alu_op1_sel),
        .E_alu_op2_sel(E_alu_op2_sel),
        .E_op(E_op),
        .E_f3(E_f3),
        .E_f7(E_f7),
        .M_dm_w_en(M_dm_w_en),
        .W_wb_en(W_wb_en),
        .W_rd(W_rd_index),
        .W_f3(W_f3),
        .W_wb_sel(W_wb_sel),
        // cache signal //
        .waiting(waiting),//
        .read(read),//
        .data_ready(data_ready)//
    );

    wire [63:0] data_out, data2mem;
    wire [7:0] mwren;
    wire [15:0] rd_addr, wr_addr;
    // cache 
    dcache Dcache (
        .clk(clk),//
        .rst(rst),//
        .address(alu_out_M[15:0]),//
        .data_in_cpu(rs2_data_M),//
        .data_in_mem(data_out),//
        .rd(read),//
        .wr(M_dm_w_en),//
        .data2cpu(ld_data),//
        .data2mem(data2mem),//
        .m_rd_address(rd_addr),//
        .m_wr_address(wr_addr),//
        .mrden(mrden),//
        .mwren(mwren),//
        .data_ready(data_ready)//
    );
    
    Data_mem dm (
        .clk(clk),//
        .write_data(data2mem),//
        .rdaddress(rd_addr),//
        .wraddress(wr_addr),//
        .rden(mrden),//
        .wren(mwren),//
        .read_data(data_out),//
        .rst(rst)//
    );

    // debug
    integer cycle_counter = 0;
    always @(current_pc) begin
        cycle_counter = cycle_counter + 1;
    end
endmodule