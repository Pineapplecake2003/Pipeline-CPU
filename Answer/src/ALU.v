//opcode define
`define R_type 5'b01100//
`define I_type_arithmetic 5'b00100//
`define LUI 5'b01101//
`define AUIPC 5'b00101//
`define JAL 5'b11011//
`define JALR 5'b11001//
`define B_type 5'b11000
`define I_type_load 5'b00000//
`define S_type 5'b01000
`define I_type_W 5'b00110
`define R_type_W 5'b01110

//func3 define
`define AND 3'b111
`define OR 3'b110
`define Shift_R 3'b101
`define XOR 3'b100
`define SLTU 3'b011
`define SLT 3'b010
`define SLL 3'b001
`define ADD_SUB 3'b000

module ALU(
    input [4:0] opcode,
    input [2:0] func3,
    input func7,
    input [63:0] operand1,
    input [63:0] operand2,
    output reg [63:0] alu_out
);
    reg [31:0] alu_out_32bits;
    always @(*) begin
        case(opcode)
        //R-type
        `R_type: begin
            case(func3)
            `AND: begin
                alu_out = operand1 & operand2;
            end
            `OR: begin
                alu_out = operand1 | operand2;
            end
            `XOR: begin
                alu_out = operand1 ^ operand2;
            end
            `ADD_SUB: begin
                if(!func7) begin
                    alu_out = operand1 + operand2;
                end
                else begin
                    alu_out = operand1 - operand2;
                end
            end
            `SLL: begin
                alu_out = operand1 << operand2[5:0];
            end
            `Shift_R: begin
                if(!func7) begin
                    alu_out = operand1 >> operand2[5:0];
                end
                else begin
                    alu_out = $signed(operand1) >>> operand2[5:0];
                end
            end
            `SLTU: begin
                alu_out = (operand1 < operand2)? 64'd1:64'd0;
            end
            `SLT: begin
                alu_out = ($signed(operand1) < $signed(operand2))? 64'd1:64'd0;
            end
            default: begin
                alu_out = 64'd0;
            end
            endcase
        end
        //I-type
        `I_type_arithmetic: begin
            case(func3)
            `AND: begin
                alu_out = operand1 & operand2;
            end
            `OR: begin
                alu_out = operand1 | operand2;
            end
            `XOR: begin
                alu_out = operand1 ^ operand2;
            end
            `ADD_SUB: begin
                alu_out = operand1 + operand2;
            end
            `SLL: begin
                alu_out = operand1 << operand2[5:0];
            end
            `Shift_R: begin
                if(!func7) begin
                    alu_out = operand1 >> operand2[5:0];
                end
                else begin
                    alu_out = $signed(operand1) >>> operand2[5:0];
                end
            end
            `SLTU: begin
                alu_out = (operand1 < operand2)? 64'd1:64'd0;
            end
            `SLT: begin
                alu_out = ($signed(operand1) < $signed(operand2))? 64'd1:64'd0;
            end
            default: begin
                alu_out = 64'd0;
            end
            endcase
        end
        //LUI
        `LUI: begin
            alu_out = operand2;
        end
        //AUIPC
        `AUIPC: begin
            alu_out = operand1 + operand2;
        end
        //Store
        `S_type: begin
            alu_out = operand1 + operand2;
        end
        //Load
        `I_type_load: begin
            alu_out = operand1 + operand2;
        end
        //Jal
        `JAL: begin
            alu_out = operand1 + 64'd4;
        end
        //Jalr
        `JALR: begin
            alu_out = operand1 + 64'd4;
        end
        //Branch
        `B_type: begin
            if(func3 == 3'b000 && operand1 == operand2) begin
                alu_out = 64'd1;
            end
            else if(func3 == 3'b001 && operand1 != operand2) begin
                alu_out = 64'd1;
            end
            else if(func3 == 3'b100 && $signed(operand1) < $signed(operand2)) begin
                alu_out = 64'd1;
            end
            else if(func3 == 3'b101 && $signed(operand1) >= $signed(operand2)) begin
                alu_out = 64'd1;
            end
            else if(func3 == 3'b110 && (operand1 < operand2)) begin
                alu_out = 64'd1;
            end
            else if(func3 == 3'b111 && (operand1 >= operand2)) begin
                alu_out = 64'd1;
            end
            else begin
                alu_out = 64'd0;
            end
        end
        // RV64I expansion
        `I_type_W: begin
            case(func3)
            `ADD_SUB: begin
                alu_out_32bits = operand1[31:0] + operand2[31:0];
                alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
            end
            `SLL: begin
                alu_out_32bits = operand1[31:0] << operand2[4:0];
                alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
            end
            `Shift_R: begin
                if(!func7) begin
                    alu_out_32bits = operand1[31:0] >> operand2[4:0];
                    alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
                end
                else begin
                    alu_out_32bits = $signed(operand1[31:0]) >>> operand2[4:0];
                    alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
                end
            end
            default: begin
                alu_out = 64'd0;
            end
            endcase
        end
        `R_type_W: begin
            case(func3)
            `ADD_SUB: begin
                if(!func7) begin
                    alu_out_32bits = operand1[31:0] + operand2[31:0];
                    alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
                end
                else begin
                    alu_out_32bits = operand1[31:0] - operand2[31:0];
                    alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
                end
            end
            `SLL: begin
                alu_out_32bits = operand1[31:0] << operand2[4:0];
                alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
            end
            `Shift_R: begin
                if(!func7) begin
                    alu_out_32bits = operand1[31:0] >> operand2[4:0];
                    alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
                end
                else begin
                    alu_out_32bits = $signed(operand1[31:0]) >>> operand2[4:0];
                    alu_out = {{32{alu_out_32bits[31]}}, alu_out_32bits};
                end
            end
            default: begin
                alu_out = 64'd0;
            end
            endcase
        end
        default: begin
            alu_out = 64'd0;
        end
        endcase
    end

endmodule