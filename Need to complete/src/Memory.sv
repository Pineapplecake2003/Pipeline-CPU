module Memory
    import DEF::dw;
(
    input logic clk,
    input logic [7:0] w_mask,
    input logic [15:0] address,
    input dw write_data,
    output dw read_data
);
    logic [7:0] mem[65536];  // register array to mimic DRAM

    /* main memory logic */
    always @(posedge clk) begin
        if (w_mask == 8'b11111111) begin
            for (logic [15:0] i = 0; i < 8; i++) begin
                mem[address+i] <= write_data[8*i+:8];
            end
        end else if (w_mask == 8'b00001111) begin
            for (logic [15:0] i = 0; i < 4; i++) begin
                mem[address+i] <= write_data[8*i+:8];
            end
        end else if (w_mask == 8'b00000011) begin
            mem[address]   <= write_data[0+:8];
            mem[address+1] <= write_data[8+:8];
        end else if (w_mask == 8'b00000001) begin
            mem[address] <= write_data[0+:8];
        end
    end
    /* assign output read data */
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
