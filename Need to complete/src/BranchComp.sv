module BranchComp
    import DEF::*;
(
    input dw operand_1,
    input dw operand_2,
    BranchCompControlIntf.BranchCompSide control
);
    assign control.BrEq = (operand_1 == operand_2) ? 1'b1 : 1'b0;
    always_comb begin
        if (control.BrUn == 1'b0) begin
            control.BrLt = ($signed(operand_1) < $signed(operand_2)) ? 1'b1 : 1'b0;
        end else begin
            control.BrLt = (operand_1 < operand_2) ? 1'b1 : 1'b0;
        end
    end
endmodule
