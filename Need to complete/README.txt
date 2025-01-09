// vsc command
// vcs top_tb.sv -sverilog -full64 -R -debug_access+all +v2k +define+FSDB +PROG=prog0 +FILE=all

// -sverilog : Support systemverilog syntax (for vcs, use -sv for verid).
// -full64 : Supports 64-bit mode operation.
// -R : Execute immediately after compilation.
// -debug_access+all : Supports all debug operations, DVE option, can view all breakpoints.
// +v2k : Support verilog 2001 features.

There are 3 arguements need to be specify.
+PROG=prog@ | @ for the file you want to run. Ex.+PROG=prog0 for file "prog0".
+FILE=@     | @ for the specific testbench you want to run.
+define+FSDB| Add this command for fsdb dumpfile.

49 testbenches for each instruction:
    use "all" to run all the testbenches.
    add addi addiw addw
	and andi
	auipc
	beq bge bgeu blt bltu bne
	jal jalr
	lb lbu lh lhu lw lwu ld lui
	or ori
	sb sh sw sd
	sll slli slliw sllw
	slt slti sltiu sltu
	sra srai sraiw sraw
	srl srli srliw srlw
	sub subw
	xor xori