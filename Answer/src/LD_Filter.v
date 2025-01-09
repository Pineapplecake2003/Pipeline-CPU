//func3 define
`define Load_Byte 3'b000
`define Load_Halfword 3'b001
`define Load_Word 3'b010
`define Load_Byte_Unsigned 3'b100
`define Load_Halfword_Unsigned 3'b101
`define Load_Word_unsigned 3'b110
`define Load_Double_Word 3'b011

module LD_Filter(
    input [2:0] func3,
    input [63:0] ld_data,
    output reg [63:0] ld_data_f
);
    always @(*) begin
        case(func3)
        `Load_Byte: begin
            ld_data_f = {{56{ld_data[7]}},ld_data[7:0]};
        end
        `Load_Halfword: begin
            ld_data_f = {{48{ld_data[15]}},ld_data[15:0]};
        end
        `Load_Word: begin
            ld_data_f = {{32{ld_data[31]}},ld_data[31:0]};;
        end
        `Load_Double_Word: begin
            ld_data_f = ld_data;
        end
        `Load_Byte_Unsigned: begin
            ld_data_f = {56'd0,ld_data[7:0]};
        end
        `Load_Halfword_Unsigned: begin
            ld_data_f = {48'd0,ld_data[15:0]};
        end
        `Load_Word_unsigned: begin
            ld_data_f = {32'd0,ld_data[31:0]};
        end
        default: begin
            ld_data_f = 64'd0;
        end
        endcase
    end
endmodule