module Controller(
    input clk,//
    input rst,//
    input [4:0] de_opcode,//
    input [2:0] de_func3,//
    input de_func7,//
    input alu_out_zero,//
    input [4:0] de_rs1,//
    input [4:0] de_rs2,//
    input [4:0] de_rd,//
    output reg next_pc_sel,//
    output stall,//
    output [7:0] F_im_w_en, //
    output D_rs1_data_sel, //
    output D_rs2_data_sel, //
    output [1:0] E_rs1_data_sel, //
    output [1:0] E_rs2_data_sel, //
    output reg E_jb_op1_sel, 
    output reg E_alu_op1_sel, //
    output reg E_alu_op2_sel, //
    output reg [4:0] E_op, //
    output reg [2:0] E_f3, //
    output reg E_f7, //
    output reg [7:0] M_dm_w_en, //
    output reg W_wb_en, //
    output reg [4:0] W_rd, //
    output reg [2:0] W_f3, //
    output reg W_wb_sel, //
    // cache signal //
    output wire waiting,//
    output wire read,//
    input wire data_ready//
);
    reg [4:0] M_op, W_op;
    reg [2:0] M_f3;
    reg [4:0] E_rd, M_rd, E_rs1, E_rs2;
    reg is_D_use_rs1, is_D_use_rs2, is_W_use_rd;
    reg is_E_use_rs1, is_E_use_rs2, is_M_use_rd;
    wire is_D_rs1_W_rd_overlap, is_D_rs2_W_rd_overlap;
    wire is_E_rs1_W_rd_overlap, is_E_rs1_M_rd_overlap;
    wire is_E_rs2_W_rd_overlap, is_E_rs2_M_rd_overlap;
    wire is_DE_overlap, is_D_rs1_E_rd_overlap, is_D_rs2_E_rd_overlap;

    // cache signal
    assign waiting = ((M_op == `I_type_load || M_dm_w_en) && !data_ready);
    assign read = (M_op == `I_type_load);

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            E_op <= 5'b00100;
            E_f3 <= 3'd0;
            E_rd <= 5'd0;
            E_rs1 <= 5'd0;
            E_rs2 <= 5'd0;
            E_f7 <= 1'd0;
        end
        else begin
            if(waiting) begin
                E_op <= E_op;
                E_f3 <= E_f3;
                E_rd <= E_rd;
                E_rs1 <= E_rs1;
                E_rs2 <= E_rs2;
                E_f7 <= E_f7;
            end
            else if(stall || next_pc_sel) begin
                E_op <= 5'b00100;
                E_f3 <= 3'd0;
                E_rd <= 5'd0;
                E_rs1 <= 5'd0;
                E_rs2 <= 5'd0;
                E_f7 <= 1'd0;
            end
            else begin
                E_op <= de_opcode;
                E_f3 <= de_func3;
                E_rd <= de_rd;
                E_rs1 <= de_rs1;
                E_rs2 <= de_rs2;
                E_f7 <= de_func7;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            M_op <= 5'b00100;
            M_f3 <= 3'd0;
            M_rd <= 5'd0;
        end
        else begin
            if(waiting) begin
                M_op <= M_op;
                M_f3 <= M_f3;
                M_rd <= M_rd;
            end
            else begin
                M_op <= E_op;
                M_f3 <= E_f3;
                M_rd <= E_rd;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            W_op <= 5'b00100;
            W_f3 <= 3'd0;
            W_rd <= 5'd0;
        end
        else begin
            if(waiting) begin
                W_op <= W_op;
                W_f3 <= W_f3;
                W_rd <= W_rd;
            end
            else begin
                W_op <= M_op;
                W_f3 <= M_f3;
                W_rd <= M_rd;
            end
        end
    end
    //F_im_w_en
    assign F_im_w_en = 8'b00000000;

    //D_rs1_data_sel
    assign D_rs1_data_sel = (is_D_rs1_W_rd_overlap)? 1'd1:1'd0;
    //D_rs2_data_sel
    assign D_rs2_data_sel = (is_D_rs2_W_rd_overlap)? 1'd1:1'd0;

    assign is_D_rs1_W_rd_overlap = (is_D_use_rs1 & is_W_use_rd & (de_rs1 == W_rd) & (W_rd !=0));
    assign is_D_rs2_W_rd_overlap = (is_D_use_rs2 & is_W_use_rd & (de_rs2 == W_rd) & (W_rd !=0));

    //is_D_use_rs1
    always @(*) begin
        if(de_opcode == `LUI || de_opcode == `AUIPC || de_opcode == `JAL) begin
            is_D_use_rs1 = 1'd0;
        end
        else begin
            is_D_use_rs1 = 1'd1;
        end
    end
    
    //is_D_use_rs2
    always @(*) begin
        if(de_opcode == `R_type || de_opcode == `B_type || de_opcode == `S_type || de_opcode == `R_type_W) begin
            is_D_use_rs2 = 1'd1;
        end
        else begin
            is_D_use_rs2 = 1'd0;
        end
    end

    //is_W_use_rd
    always @(*) begin
        if(W_op == `B_type || W_op == `S_type) begin
            is_W_use_rd = 1'd0;
        end
        else begin
            is_W_use_rd = 1'd1;
        end
    end

    //E_rs1_data_sel
    assign E_rs1_data_sel = (is_E_rs1_M_rd_overlap)? 2'd1: (is_E_rs1_W_rd_overlap)? 2'd0:2'd2;
    //E_rs2_data_sel
    assign E_rs2_data_sel = (is_E_rs2_M_rd_overlap)? 2'd1: (is_E_rs2_W_rd_overlap)? 2'd0:2'd2;

    assign is_E_rs1_W_rd_overlap = (is_E_use_rs1 & is_W_use_rd & (E_rs1 == W_rd) & (W_rd != 0));
    assign is_E_rs1_M_rd_overlap = (is_E_use_rs1 & is_M_use_rd & (E_rs1 == M_rd) & (M_rd != 0));
    assign is_E_rs2_W_rd_overlap = (is_E_use_rs2 & is_W_use_rd & (E_rs2 == W_rd) & (W_rd != 0));
    assign is_E_rs2_M_rd_overlap = (is_E_use_rs2 & is_M_use_rd & (E_rs2 == M_rd) & (M_rd != 0));

    //is_E_use_rs1
    always @(*) begin
        if(E_op == `LUI || E_op == `AUIPC || E_op == `JAL) begin
            is_E_use_rs1 = 1'd0;
        end
        else begin
            is_E_use_rs1 = 1'd1;
        end
    end

    //is_E_use_rs2
    always @(*) begin
        if(E_op == `R_type || E_op == `B_type || E_op == `S_type || E_op == `R_type_W) begin
            is_E_use_rs2 = 1'd1;
        end
        else begin
            is_E_use_rs2 = 1'd0;
        end
    end

    //is_M_use_rd
    always @(*) begin
        if(M_op == `B_type || M_op == `S_type) begin
            is_M_use_rd = 1'd0;
        end
        else begin
            is_M_use_rd = 1'd1;
        end
    end

    //E_jb_op1_sel
    always @(*) begin
        if(E_op == `JALR) begin
            E_jb_op1_sel = 1'b0;
        end
        else begin
            E_jb_op1_sel = 1'b1;
        end
    end

    //E_alu_op1_sel
    always @(*) begin
        if(E_op == `AUIPC || E_op == `JAL || E_op == `JALR) begin
            E_alu_op1_sel = 1'b1;
        end
        else begin
            E_alu_op1_sel = 1'b0;
        end
    end

    //E_alu_op2_sel
    always @(*) begin
        if(E_op == `R_type || E_op == `B_type || E_op == `R_type_W) begin
            E_alu_op2_sel = 1'b0;
        end
        else begin
            E_alu_op2_sel = 1'b1;
        end
    end

    //M_dm_w_en
    always @(*) begin
        if(M_op == `S_type) begin
            case(M_f3)
                3'b000: begin
                    M_dm_w_en = `Byte;
                end
                3'b001: begin
                    M_dm_w_en = `Halfword;
                end
                3'b010: begin
                    M_dm_w_en = `Word;
                end
                3'b011: begin
                    M_dm_w_en = `Doubleword;
                end
                default: begin
                    M_dm_w_en = 8'b00000000;
                end
            endcase
        end 
        else begin
            M_dm_w_en = 8'b00000000;
        end
    end

    //W_wb_en
    always @(*) begin
        if(W_op == `B_type || W_op == `S_type) begin
            W_wb_en = 1'b0;
        end
        else begin
            W_wb_en = 1'b1;
        end
    end

    //W_wb_sel
    always @(*) begin
        if(W_op == `I_type_load) begin
            W_wb_sel = 1'b1;
        end
        else begin
            W_wb_sel = 1'b0;
        end
    end

    //next_pc_sel
    always @(*) begin
        if(E_op == `B_type && alu_out_zero) begin
            next_pc_sel = 1'b1;
        end
        else if(E_op == `JAL || E_op == `JALR) begin
            next_pc_sel = 1'b1;
        end
        else begin
            next_pc_sel = 1'b0;
        end
    end

    //stall
    assign stall = (E_op == `I_type_load) & is_DE_overlap;

    assign is_DE_overlap = (is_D_rs1_E_rd_overlap | is_D_rs2_E_rd_overlap);
    assign is_D_rs1_E_rd_overlap = is_D_use_rs1 & (de_rs1 == E_rd) & (E_rd != 0);
    assign is_D_rs2_E_rd_overlap = is_D_use_rs2 & (de_rs2 == E_rd) & (E_rd != 0);
endmodule