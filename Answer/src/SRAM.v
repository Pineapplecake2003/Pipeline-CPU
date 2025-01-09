//w_en define
`define Byte 8'b00000001
`define Halfword 8'b00000011
`define Word 8'b00001111
`define Doubleword 8'b11111111

module SRAM(
    input clk,
    input [7:0] w_en,
    input [63:0] address,
    input [63:0] write_data,
    output [63:0] read_data
);
    reg [7:0] mem [0:65535];

    always @(posedge clk) begin
        case(w_en)
        `Byte: begin
            mem[address] <= write_data[7:0];
        end
        `Halfword: begin
            mem[address + 1] <= write_data[15:8];
            mem[address] <= write_data[7:0];
        end
        `Word: begin
            mem[address + 3] <= write_data[31:24];
            mem[address + 2] <= write_data[23:16];
            mem[address + 1] <= write_data[15:8];
            mem[address] <= write_data[7:0];
        end
        `Doubleword: begin
            mem[address + 7] <= write_data[63:56];
            mem[address + 6] <= write_data[55:48];
            mem[address + 5] <= write_data[47:40];
            mem[address + 4] <= write_data[39:32];
            mem[address + 3] <= write_data[31:24];
            mem[address + 2] <= write_data[23:16];
            mem[address + 1] <= write_data[15:8];
            mem[address] <= write_data[7:0];
        end
        default: begin
            mem[address] <= mem[address];
        end
        endcase
    end

    assign read_data = {
        mem[address+7],
        mem[address+6],
        mem[address+5],
        mem[address+4],
        mem[address+3],
        mem[address+2],
        mem[address+1],
        mem[address]
        };


endmodule