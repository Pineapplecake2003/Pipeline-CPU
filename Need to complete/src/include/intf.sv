interface BranchCompControlIntf;
    /* signal bundle */
    logic BrUn, BrEq, BrLt;
    /* modports */
    modport ControllerSide(output BrUn, input BrEq, input BrLt);
    modport BranchCompSide(input BrUn, output BrEq, output BrLt);
endinterface : BranchCompControlIntf
