module LDFilter
    import DEF::*;
(
    input logic [2:0] func3,
    input dw in_data,
    output dw out_data
);
    always_comb begin
        unique case (func3)
            LB_FUNC3:  out_data = {{56{in_data[7]}}, in_data[7:0]};
            LH_FUNC3:  out_data = {{48{in_data[15]}}, in_data[15:0]};
            LW_FUNC3:  out_data = {{32{in_data[31]}}, in_data[31:0]};
            LD_FUNC3:  out_data = in_data;
            LBU_FUNC3: out_data = {56'd0, in_data[7:0]};
            LHU_FUNC3: out_data = {48'd0, in_data[15:0]};
            LWU_FUNC3: out_data = {32'd0, in_data[31:0]};
            default:   out_data = 64'd0;
        endcase
    end
endmodule
