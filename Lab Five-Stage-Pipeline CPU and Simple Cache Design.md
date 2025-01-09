# Lab Five-Stage-Pipeline CPU and Simple Cache Design
## Critical Path
### Recall single cycle CPU
![image](https://hackmd.io/_uploads/ByeIXabS1x.png)
上圖為Lab5中實作的Single cycle CPU，CPU內零組件的所有動作都必須在一個周期內做完，且CPU的速度與其頻率成正比
$f = \frac{1}{period}$
想要提升頻率，就必須縮小週期，而最小週期為何，就需要計算 Critical Path ，也就是資料在傳遞過程中最慢的一個分支。藉由使週期等於 Critical Path，可以保證任何資料傳遞分支都可以在同個周期內正確完成，以下假設一些內部元件延遲數值作為範例。
||setup time|Imme Ext|Reg File|MUX|ALU|Branch Comp|IM/DM|LD Filter|+ 64'd4|
|---|---|---|---|---|---|---|---|---|---|
|delay time(ns)|1|3|5|1|5|3|10|3|5|
> [!NOTE]  
> 上表中，我們只假設data path的資料傳遞，Control path實際上也會影響到完整延遲，例如MUX的控制訊號有可能會比資料晚到，此時最慢的延遲就要計算控制訊號的延遲，但在這裡為了簡化計算以及給予Critical Path 的觀念，先不考慮 Control path
> 
現在，根據RISC-V指令類型計算每種指令所需要的最短拿到正確資料的時間。
* Computational Instruction
    以```and t0, t1, t2```為例
    1. 從 IM 中獲得指令: **10 ns**
    2. 不會使用到Imm Ext，故選擇從Reg File中獲得 `t0`, `t1` 兩個暫存器數值的延遲: **5 ns**
    3. 進去ALU前，會遇到`rs1`, `rs2` 的兩個分支，但Critical Path 為計算最長延遲，故這裡以兩個MUX延遲計算: **1 + 1 ns**
    4. ALU 計算 `t0`, `t1` bitwise and的結果: **5 ns**
    5. 不用寫入DM，所以選擇寫回去Reg file前最後的MUX: **1 ns**
    6. 寫入資料進`t0`，Setup time of Reg File: **1 ns**
    加總為 10 + 5 + 1 + 1 + 5 + 1 + 1 = **24 ns**，也就是Computational Instruction 正確傳遞資料所需的最短時間。
    
* Load
    以```lw t0 0(sp)```為例
    1. 從 IM 中獲得指令: **10 ns**
    2. 此類指令的會有立即數offset，此例中為`0`，但Imm Ext延遲較少，故計算Reg File中獲得 `sp` 暫存器的數值: **5 ns**
    3. `rs1`會經過兩個MUX: **1 + 1 ns**
    4. ALU的結果作為 DM 的地址，需要等 DM傳出正確資料: **10 ns**
    5. 傳出資料經過 LD filter: **3 ns**
    6. Reg file前最後的MUX: **1 ns**
    7. 寫入資料進`t0`，Setup time of Reg File: **1 ns**
    加總為 10 + 5 + 1 + 1 + 10 + 3 + 1 + 1 = **32 ns**，也就是Load Instruction 正確傳遞資料所需的最小時間。
* Store
    以```sw t0 0(sp)```為例
    1. 從 IM 中獲得指令: **10 ns**
    2. 此類指令的會有立即數offset，此例中為`0`，但Imm Ext延遲較少，故計算Reg File中獲得 `sp` 暫存器的數值: **5 ns**
    3. `rs1`會經過兩個MUX: **1 + 1 ns**
    4. ALU計算地址: **5 ns**
    5. ALU的結果作為 DM 的地址，寫入DM，setup time of DM: **1 ns**
    
    加總為 10 + 5 + 1 + 1 + 5 + 1 = **23 ns**，也就是Store Instruction 正確傳遞資料所需的最短時間。
* Jump
以```jal ra LABEL```為例
    1. 從 IM 中獲得指令: **10 ns**
    2. 此類指令的會有立即數offset，此例中為`LABEL`，但Imm Ext延遲較少，故計算Reg File的延遲: **5 ns**
    3. 從Reg File出來後，會需要計算寫入`ra` 的數值，與計算PC + 4進入`PC` 暫存器，計算數值所需延遲(**1 + 1 + 5 ns**)比計算PC + 4(**5 ns**)多，選擇ALU計算next_pc的路徑: **1 + 1 + 5 ns**
    4. 經過MUX再經過`PC`的setup time : **1 + 1ns**
    加總為 10 + 5 + 1 + 1 + 5 + 1 + 1 = **24 ns**，Jump Instruction 正確傳遞資料所需的最短時間。
`jalr`同理。
* Branch
以```beq t0, x0 LABEL```為例
條件符合的情況下所需延遲最多，故假設`t0` = 0
    1. 從 IM 中獲得指令: **10 ns**
    2. 此類指令的會有立即數offset，此例中為`LABEL`，但Imm Ext延遲較少，故計算Reg File的延遲: **5 ns**
    3. 從Reg File出來後，會需要計算branch條件是否符合，以及即將跳轉的地址，這裡同樣也是ALU分支所需延遲較多: **1 + 1 + 5 ns**
    4. 經過MUX再經過`PC`的setup time : **1 + 1ns**
    加總為 10 + 5 + 1 + 1 + 5 + 1 + 1 = **24 ns**，Branch Instruction 正確傳遞資料所需的最短時間。
> [!NOTE]  
> 若有分支的情況下會選擇延遲較多分支納入Critical Path 的計算，因為在延遲較多分支還沒得出正確結果的情況下，延遲較少分支就已經計算完了，並不影響所需最短時間。

上面例子中，Load 所需要的時間最久，為**32 ns**，所以Critical Path為 **32 ns**，最快頻率為
$\frac{1}{32\times10^{-9}}=31.25MHz$


## Pipeline
上述提到了我們可以計算Critical Path得知硬體最快的速度，但是要如何進一步加快呢，我們可以將一整個硬體切分為多個延遲差不多的小硬體，例如以下是一個輸入為12筆 32bit資料，輸出為這12筆中最大值資料的硬體，由11個輸入為兩筆32bit資料，輸出為較大者資料的Compare Unit所組成。
![未命名绘图.drawio](https://hackmd.io/_uploads/BJjLmN4rJl.png)
假設所有Compare Unit 延遲皆為 **5ns**，這硬體的Critical Path為**20 ns**
但若將中間結果暫時存入register中，即下圖所示。
![未命名绘图.drawio](https://hackmd.io/_uploads/rJK244ErJx.png)
就可以將Critical Path降至 **10ns**。被分為兩邊的小硬體在每一個週期中只作一部分的任務，每隔硬體就可以在更小的週期中將資料正確地傳遞出去。

## 5-stage pipeline CPU
將上述概念套用至Single cycle CPU中，如下圖所示。
![full](https://hackmd.io/_uploads/BJ2Y8gwLke.jpg)

我們先主要關注在data path
![datapath](https://hackmd.io/_uploads/HkO5UePU1x.jpg)

插入了4個register，為Reg_D, Reg_E, Reg_M and Reg_W用來儲存中間結果，形成5-stage，分別為
1. IF(Instruction Fetch)
![image](https://hackmd.io/_uploads/S14tRJP8Je.png)



2. ID(Instruction Decode)
![image](https://hackmd.io/_uploads/HJl5AJPIyg.png)



3. EXE(Execute)
![image](https://hackmd.io/_uploads/H1RoUlwL1e.png)

4. MEM(Memory)
![image](https://hackmd.io/_uploads/rkrj0JDIJx.png)


5. WB(Write back)
![image](https://hackmd.io/_uploads/BJZ2R1DUJg.png)


這5個stage 中，MEM(Memory) 的延遲最久，為 **10 + 1(setup time) ns**，其他stage 延遲均小於**11 ns** 所以Critical Path = **11ns**
$f = \frac{1}{11\times10^{-9}} = 90.91MHz$

* Instruction latency: 11 * 5 per instruction
一個指令在CPU內的時間變為11ns 延遲乘5倍。
* Instruction throughput(instruction per cycle): $\frac{Instruction Count}{Time}\approx\frac{1}{11}$
在前5個指令執行完後，之後的每一個指令只需要**11 ns**即可執行完，當指令夠多，整體而言只要每**11 ns**即可執行完一條指令。

## Hazards
* Control Hazard
以下列RISC-V指令為例
```asm=
li t0, 0
beq t0, x0, EQU_ZERO
addi t3, t3, 1
jal x0, NOT_EQU_ZERO
EQU_ZERO:
    addi t0, t0, 1
    li   t2, 1
    sw t1 0(sp)
NOT_EQU_ZERO:
    li t3, 6
```
經過5-stage pipeline CPU 後會如下表所示
|cycle|IF|ID|EX|MEM|WB|
|---|---|---|---|---|---|
|0|`li t0, 0`|||||
|1|`beq t0, x0, EQU_ZERO`|`li t0, 0`||||
|2|`addi t3, t3, 1`|`beq t0, x0, EQU_ZERO`|`li t0, 0`|||
|3|`jal x0, NOT_EQU_ZERO`|`addi t3, t3, 1`|`beq t0, x0, EQU_ZERO`|`li t0, 0`||
|4|`addi t0, t0, 1`|`nop`|`nop`|`beq t0, x0, EQU_ZERO`|`li t0, 0`|
|5|`li   t2, 1`|`addi t0, t0, 1`|`nop`|`nop`|`beq t0, x0, EQU_ZERO`|
|6|`sw t1 0(sp)`|`li   t2, 1`|`addi t0, t0, 1`|`nop`|`nop`|
|7|`li t3, 6`|`sw t1 0(sp)`|`li   t2, 1`|`addi t0, t0, 1`|`nop`|
|8||`li t3, 6`|`sw t1 0(sp)`|`li   t2, 1`|`addi t0, t0, 1`|
|9|||`li t3, 6`|`sw t1 0(sp)`|`li   t2, 1`|
|10||||`li t3, 6`|`sw t1 0(sp)`|
|11|||||`li t3, 6`|

因為`beq t0, x0, EQU_ZERO`是否要跳轉的結果需要在EX-stage才可得到，而已經先行獲得以下指令了，故在第4cycle中就需要把不會執行到的指令清除，變成`nop(no operation)`，為了能夠實現此功能 pipeline register中，需要有一根控制線判斷是否要將存儲的資料傳遞下去抑或是清除(`jb`)。

* Data Hazard
以下列RISC-V指令為例
```asm=
li t0, 0
li t1, 1
addi t2, t0, t1
addi t4, t0, 1
lw t3, 0(t2)
addi t3, t3, 5
```
|cycle|IF|ID|EX|MEM|WB|
|---|---|---|---|---|---|
|0|`li t0, 0`|||||
|1|`li t1, 1`|`li t0, 0`||||
|2|`addi t2, t0, t1`|`li t1, 1`|`li t0, 0`|||
|3|`addi t4, t0, 1`|`addi t2, t0, t1`|`li t1, 1`|`li t0, 0`||
|4|`lw t3, 0(t2)`|`addi t4, t0, 1`|`addi t2, t0, t1`|`li t1, 1`|`li t0, 0`|
|5|`addi t3, t3, 5`|`lw t3, 0(t2)`|`addi t4, t0, 1`|`addi t2, t0, t1`|`li t1, 1`
|6||`addi t3, t3, 5`|`lw t3, 0(t2)`|`addi t4, t0, 1`|`addi t2, t0, t1`|
|7||`addi t3, t3, 5`|`nop`|`lw t3, 0(t2)`|`addi t4, t0, 1`|
|8|||`addi t3, t3, 5`|`nop`|`lw t3, 0(t2)`|
|9||||`addi t3, t3, 5`|`nop`|
|9|||||`addi t3, t3, 5`|

1. 在第4 cycle，`li t1, 1`與`li t0, 1`的結果需要在各自WB階段才會得到，但下一個cycle `addi t2, t0, t1`，就需要用到，需要訊號線將MEM-stage, WB-stage 的輸入forwarding 到EXE-stage，就不用插入`nop`
2. 在第4 cycle，`addi t4, t0, 1`需要用到`li t0, 0`的結果，所以也需要將WB-stage的結果forwarding到D-stage。
4. 在第5cycle，牽涉到load-and-use，也就是`lw t3, 0(t2)`後面指令馬上要用到`t3`，而`t3`正確的數值需要等到MEM-stage的結果，沒有辦法透過拉訊號線的方式將資料提前，所以要插入一個`nop`延遲一個stage，而前面stage就需要保留住當前資料，所以前三個pipeline register就需要有一個控制訊號可以保留資料(`stall`)。

## Pipeline register
這裡分析每個pipeline register在特定的控制訊號中需要做些什麼。
||stall|jb|others|
|---|---|---|---|
|Reg_PC|keep(output <= output)|output <= input|output <= input|
|Reg_D|keep(output <= output)|flush(output <= nop)|output <= input|
|Reg_E|bubble(output <= nop)|flush(output <= nop)|output <= input|
|Reg_M|output <= input|output <= input|output <= input|
|Reg_W|output <= input|output <= input|output <= input|

可以看到除了`Reg_PC`, `Reg_D` 與 `Reg_E`，需要根據stall, jb訊號做出改變，其他都可以將資料直接傳遞下去。
## Controller
![image](https://hackmd.io/_uploads/Bk7Gs1w8ke.png)
Controller需要儲存EXE, MEM, WB-stage中所用到的各種opcode, f7, f3, rs1, rs2，MEM, WB-stage中不需要rs1, rs2與f7 所以只需要存三個。
|Signal|Usage|
|---|---|
|im_w_mask (8)|Prevent writing data into im|
|D_rs1_data_sel (1)|Immediately Read the Writeback rs1 data|
|D_rs2_data_sel (1)|Immediately Read the Writeback rs2 data|
|E_rs1_data_sel (2)|Get the newest rs1 data|
|E_rs2_data_sel (2)|Get the newest rs2 data|
|alu_op1_sel (1)|Select alu op1 (rs1 / pc)|
|alu_op2_sel (1)|Select alu op2 (rs2 / imm)|
|is_lui(1)|is instruction LUI?|
|alu_control(5)|Control signal to ALU in EXE stage|
|dm_w_mask (8)|Store [ byte / halfword / word /doubleword] into dm|
|reg_w_en (1)|Whether write back data into RegFile|
|W_rd_index (5)|Write back register destination index|
|W_f3 (3)|Output the func3 in W stage to LD_Filter|
|wb_sel (2)|Select writeback data((PC + 4 / Load data / ALU result))|
|||
|stall (1)|Set when there is a Load instruction in EXE stage and there is an instruction in D stage that uses the Load’s writeback data|
|next_pc_sel (1)|Set when a jump or branch is established in EXE stage|
## Code
### Control signal
1. D_rs1_data_sel
```verilog=
assign D_rs1_data_sel = is_D_rs1_W_rd_overlap ? 1'd1 : 1'd0;
assign is_D_rs1_W_rd_overlap = is_D_use_rs1 & is_W_use_rd & (D_rs1 == W_rd) & W_rd != 0;
assign is_D_use_rs1 = ...
assign is_W_use_rd = ...
```
2. D_rs2_data_sel
```verilog=
assign D_rs2_data_sel = is_D_rs2_W_rd_overlap ? 1'd1 : 1'd0;
assign is_D_rs2_W_rd_overlap = is_D_use_rs2 & is_W_use_rd & (D_rs2 == W_rd) & W_rd != 0;
assign is_D_use_rs2 = ...
assign is_W_use_rd = ...
```

![D_forward](https://hackmd.io/_uploads/Bk3NllwUye.jpg)


其中，D_rs1/D_rs2為D-stage中的rs1/rs2,其判斷邏輯為，當前D-stage的rs1/rs2是否與WB-stage的rd重複且不等於0，這時候，就需要forwarding提前將資料送回EXE-stage。


3. E_rs1_data_sel
```verilog=
assign E_rs1_data_sel = is_E_rs1_M_rd_overlap ? 2’d1 :\
is_E_rs1_W_rd_overlap ? 2’d0 : 2’d2;
assign is_E_rs1_W_rd_overlap = is_E_use_rs1 & is_W_use_rd & (E_rs1 == W_rd) & W_rd != 0;
assign is_E_rs1_M_rd_overlap = is_E_use_rs1 & is_M_use_rd & (E_rs1 == M_rd) & M_rd != 0;
assign is_E_use_rs1 = ...
assign is_W_use_rd = ...
assign is_M_use_rd = ...
```
4. E_rs2_data_sel
```verilog=
assign E_rs2_data_sel = is_E_rs2_M_rd_overlap ? 2’d1 :\
is_E_rs2_W_rd_overlap ? 2’d0 : 2’d2;
assign is_E_rs2_W_rd_overlap = is_E_use_rs2 & is_W_use_rd & (E_rs2 == W_rd) & W_rd != 0;
assign is_E_rs2_M_rd_overlap = is_E_use_rs2 & is_M_use_rd & (E_rs2 == M_rd) & M_rd != 0;
assign is_E_use_rs2 = ...
assign is_W_use_rd = ...
assign is_M_use_rd = ...
```
![E_forward](https://hackmd.io/_uploads/HkcgZlD8yx.jpg)

先行判斷是否EXE-stage與MEM-stage的rs1/rs2, rd是否有重複，如有，就選擇forwarding MEM-stage的資料，再判斷是否與WB-stage重複，如有就選擇WB-stage的資料。
5. stall

```verilog=
assign stall = (E_op == LOAD) & is_DE_overlap;
assign is_DE_overlap = (is_D_rs1_E_rd_overlap | is_D_rs2_E_rd_overlap);
assign is_D_rs1_E_rd_overlap = is_D_use_rs1 /*& is_E_use_rd*/ & (dc_rs1 == E_rd) & (E_rd != 0);
assign is_D_rs2_E_rd_overlap = is_D_use_rs2 /*& is_E_use_rd*/ & (dc_rs2 == E_rd) & (E_rd != 0);

assign is_D_use_rs1 = ...
assign is_D_use_rs2 = ...
//assign is_E_use_rd = ...
```
判斷EXE-stage的rd是否與ID-stage的rs1/rs2重疊。這裡`is_E_use_rd` 被註解掉的原因為，在`stall = (E == load) & is_DE_overlap` 的`(E == load)`就已經知道EXE-stage會不會使用到rd了，若為Load instraction，必定有rd。

6. next_pc_sel
判斷是否要因為跳轉而將指令清除，只要在EXE-stage判斷即可。
### Each Unit
以下講解一些 Five-Stage-Pipeline 相比 Single cycle CPU 多需要的模組。
1. PC.sv
與Single cycle CPU 相似，但多了一個`stall`訊號，判斷是否要維持輸出。
```verilog=
module PC
    import DEF::*;
(
    input logic clk,
    input logic rst,
    input dw next_pc,
    input logic stall,
    output dw current_pc
);
    // ... 以上述描述實現
endmodule : PC
```
2. Controller.sv
```verilog=
module Controller
    import DEF::*;
(
    input logic clk,//
    input logic rst,//
    /* inst information */
    input inst_t inst,//
    /* a0 and a1 for ECALL handle */
    input dw reg_a0,
    input dw reg_a1,
    /* next PC select */
    output next_pc_sel_t next_pc_sel,//
    /* IM write control */
    output logic [7:0] im_w_mask,//
    /* Register File Control */
    output logic reg_w_en,//
    /* ALU control */
    output alu_op1_sel_t alu_op1_sel,//
    output alu_op2_sel_t alu_op2_sel,//
    output logic is_lui,//
    output alu_control_packet_t alu_control,// E_op, E_f3, E_f7
    /* Branch Comparator control */
    BranchCompControlIntf.ControllerSide bc_control,//
    /* DM write control */
    output logic [7:0] dm_w_mask, //
    /* write-back select */
    output wb_sel_t wb_sel, //
    /* halt signal (for testbench to monitor) */
    output logic halt,
    output logic stall,//
    output logic [4:0] W_rd,//
    output logic [2:0] W_f3,//
    output D_data_sel D_rs1_data_sel,//
    output D_data_sel D_rs2_data_sel,//
    output E_data_sel E_rs1_data_sel,//
    output E_data_sel E_rs2_data_sel//
);
    /************************************************************/
    /* example signal (you can add or del variable if you need) */
    /************************************************************/

    logic [6:0] D_op, E_op, M_op, W_op; 
    logic [2:0] D_f3, E_f3, M_f3;
    logic [4:0] D_rd, E_rd, M_rd, E_rs1, E_rs2, D_rs1, D_rs2;
    logic is_D_use_rs1, is_D_use_rs2, is_W_use_rd;
    logic is_E_use_rs1, is_E_use_rs2, is_M_use_rd;
    logic D_f7, E_f7;
    logic is_D_rs1_W_rd_overlap, is_D_rs2_W_rd_overlap;
    logic is_E_rs1_W_rd_overlap, is_E_rs1_M_rd_overlap, is_E_rs2_W_rd_overlap, is_E_rs2_M_rd_overlap;
    logic is_DE_overlap, is_D_rs1_E_rd_overlap, is_D_rs2_E_rd_overlap;
    /************************************************************/
    //... 以上述描述實現

    
/* ECALL handling */
    always_latch begin
        if (inst.I_TYPE.opcode == SYSTEM && inst.I_TYPE.imm_11_0 == ECALL_FUNC12) begin
            if (reg_a0 == 64'd0) begin : ECALL_to_halt
                halt = 1'b1;
            end : ECALL_to_halt
            else if (reg_a0 == 64'd1) begin : ECALL_to_putchar
                $display("%c", reg_a1[7:0]);
            end : ECALL_to_putchar
            else begin : ECALL_not_support
                $display("Not supported ECALL service request type!\n");
                $finish;
            end : ECALL_not_support
        end
    end
endmodule

```
3. Reg_D.sv
若有`rst`，輸出歸零，否則查看`stall`與`jb`訊號判斷是否要產生`nop`或維持輸出，否則輸出維持輸入。
```verilog=
module Reg_D
    import DEF::*;
(
    input logic clk,
    input logic rst,
    input dw current_pc,
    input logic [31:0] inst,
    input logic stall,
    input logic jb,
    output dw current_pc_D,
    output logic [31:0] inst_D
);
    // ... 以上述描述實現
endmodule
```
4. Reg_E.sv
若有`rst`，輸出歸零，否則若有`stall`或`jb`訊號，產生`nop`，否則輸出維持輸入。
```verilog=
module Reg_E
    import DEF::*;
(
    input logic clk,
    input logic rst,
    input dw current_pc_D,
    input dw rs1_data_D,
    input dw rs2_data_D,
    input dw sext_imm_D,
    input logic stall,
    input logic jb,
    output dw current_pc_E,
    output dw rs1_data_E,
    output dw rs2_data_E,
    output dw sext_imm_E
);
    // ... 以上述描述實現
endmodule
```
5. Reg_M.sv
此Pipeline register 只需要在`rst`訊號為0時歸零輸出，否則將資料傳遞。
```verilog=
module Reg_M
    import DEF::*;
(
    input logic clk,
    input logic rst,
    input dw alu_out_E,
    input dw rs2_data_E,
    input dw current_pc_E,
    output dw current_pc_M,
    output dw alu_out_M,
    output dw rs2_data_M
);
    // ... 以上述描述實現
endmodule
```
6. Reg_W.sv
此Pipeline register 只需要在`rst`訊號為0時歸零輸出，否則將資料傳遞。
```verilog=
`include "./src/include/DEF.sv"
`include "./src/include/intf.sv"
`include "./src/PC.sv"
`include "./src/ALU.sv"
`include "./src/ImmExt.sv"
`include "./src/LDFilter.sv"
`include "./src/Memory.sv"
`include "./src/RegFile.sv"
`include "./src/BranchComp.sv"
`include "./src/Controller.sv"
`include "./src/Reg_D.sv"
`include "./src/Reg_M.sv"
`include "./src/Reg_E.sv"
`include "./src/Reg_W.sv"
module Core
    import DEF::*;
(
    input  logic clk,
    input  logic rst,
    output logic halt
);
    /************************************************************/
    /* example signal (you can add or del variable if you need) */
    /* check the signal is connected to the right position      */
    /* hint: notice some similar signal name and their stage    */
    /* example: current_pc_D, current_pc_E, current_pc_M ...    */
    /************************************************************/

    logic reg_w_en, stall;
    wb_sel_t wb_sel;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [31:0] inst, inst_dummy, inst_D;
    /* verilator lint_on UNUSEDSIGNAL */
    next_pc_sel_t next_pc_sel;
    dw next_pc, current_pc, current_pc_plus_4, alu_out_as_pc, current_pc_D, current_pc_E, current_pc_M, current_pc_W;
    dw imm_ext_out, reg_write_data, rs1_read_data, rs2_read_data, reg_a0, reg_a1;
    dw alu_out, dm_read_data, ldfilter_out_data, alu_out_W;
    dw rs1_data_D, rs2_data_D;
    D_data_sel D_rs1_data_sel, D_rs2_data_sel;
    dw rs1_data_E, rs2_data_E, imm_ext_out_E;
    E_data_sel E_rs1_data_sel, E_rs2_data_sel;
    dw rs1_or_current_pc_or_zero, rs2_or_imm, newest_rs1_data, newest_rs2_data, alu_out_M;
    alu_op1_sel_t alu_op1_sel;
    alu_op2_sel_t alu_op2_sel;
    logic is_lui;
    alu_control_packet_t alu_control;
    dw rs2_data_M, ld_data_W;;
    logic [2:0] W_f3;
    logic [4:0] rs1_index, rs2_index, W_rd;
    logic [7:0] im_w_mask, dm_w_mask;
    /************************************************************/
    // ... 按照上述圖示將各模組拼湊起來
endmodule : Core
```
>[!Caution]
>剩下勘誤