`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/03/2018 05:46:26 PM
// Design Name: 
// Module Name: mips_final
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mips_final(clk, PC, Instruction, ALUResult, dWriteData, WriteBackData);
    input clk;
    output [31:0] WriteBackData, ALUResult, PC, Instruction, dWriteData; 
    
    localparam RTYPE_OPCODE = 6'b0;
    localparam LW_OPCODE = 6'b100011;
    localparam SW_OPCODE = 6'b101011;
    localparam BEQ_OPCODE = 6'b000100;
    localparam BNE_OPCODE = 6'b000101;
    localparam ADDI_OPCODE = 6'b001000;
    localparam ANDI_OPCODE = 6'b001100;
    localparam ORI_OPCODE = 6'b001101;
    localparam JUMP_OPCODE = 6'b000010;
    localparam JAL_OPCODE = 6'b000011;
    
    localparam ADD_FUNCT = 6'b100000;
    localparam SUB_FUNCT = 6'b100010;
    localparam AND_FUNCT = 6'b100100;
    localparam OR_FUNCT = 6'b100101;
    localparam SLT_FUNCT = 6'b101010;
    localparam NOR_FUNCT = 6'b100111;
    localparam JR_FUNCT = 6'b001000;
    localparam ALU_AND = 4'b0000;
    localparam ALU_OR = 4'b0001;
    localparam ALU_ADD = 4'b0010;
    localparam ALU_SUB = 4'b0110;
    localparam ALU_SLT = 4'b0111;
    localparam ALU_NOR = 4'b1100;
    
    //IF Stage
    wire [31:0] IncrementedPC, ALUInput1, JumpAddress;
    reg [25:0] JumpTarget; 
    wire [9:0] InstructionAddress;
    wire PCSrc, HaltPC; 
    reg [31:0] InstructionMem[1023:0];
    reg [31:0] IncrementedPCRegID, BranchAddress, IF_PC, InstructionReg;
    reg JumpEX, JrOpcodeEX;
    
    initial begin
        $readmemh("final_instruction_memory.txt", InstructionMem);
        IncrementedPCRegID = 32'b0;
        BranchAddress = 32'b0;
        IF_PC = 32'b0;
        InstructionReg = 32'b0;
        JumpEX = 0;
        JumpTarget = 0;
        JrOpcodeEX = 0;
    end
    
    assign IncrementedPC = IF_PC + 4;
    assign InstructionAddress = IF_PC[11:2];
    assign JumpAddress = JrOpcodeEX ? ALUInput1 :
                         {IF_PC[31:28], JumpTarget, 2'b0};          
    always @(posedge clk) begin
        if (!HaltPC) begin
            if (IF_PC >= 0 && IF_PC <= 32'h00000fff)
                InstructionReg <= InstructionMem[InstructionAddress];
            if (PCSrc)
                IF_PC <= BranchAddress;
            else if (JumpEX)
                IF_PC = JumpAddress;
            else
                IF_PC <= IncrementedPC;
        IncrementedPCRegID <= IncrementedPC;
        end
    end
    
    //ID Stage
    reg [31:0] RF [31:0];
    reg [31:0] ReadDataReg1, ReadDataReg2, SextReg, IncrementedPCRegEX;
    reg [4:0] WriteRegAddress1EX, WriteRegAddress2EX, RegRsEX, RegRtEX, RegRdEX;
    reg [3:0] ALUCtrlEX;
    reg ALUSrcEX, RegDstEX, BranchEX, MemWriteEX, MemReadEX, RegWriteEX, MemtoRegEX, RegWriteWB, BranchNotEqualEX;
    wire ALUSrc, RegDst, Branch, MemWrite, MemRead, RegWrite, MemtoReg, ZeroExt, InsertNopSw, InsertNopBranch, InsertNopJump, JrOpcode, Jump, BranchNotEqual;
    wire [3:0] ALUCtrl;
    wire [4:0] ReadReg1, ReadReg2, WriteReg;
    wire [5:0] FunctField, Opcode;
    wire [31:0] ImmediateValue; 
    integer i;
    
    initial begin
        for (i = 0; i < 32; i=i+1)
            RF[i] = 0;   
        ReadDataReg1 = 32'b0;
        ReadDataReg2 = 32'b0;
        SextReg = 32'b0;
        IncrementedPCRegEX = 32'b0;
        JumpTarget = 26'b0;
        WriteRegAddress1EX = 5'b0;
        WriteRegAddress2EX = 5'b0;           
        ALUCtrlEX = 4'b0;
        ALUSrcEX = 0;
        RegDstEX = 0;
        BranchEX = 0;
        MemWriteEX = 0;
        MemReadEX = 0;
        RegWriteEX = 0;
        MemtoRegEX = 0;
        RegWriteWB = 0;
        RegRsEX = 0;
        RegRtEX = 0;
        RegRdEX = 0;
        JrOpcodeEX = 0;
        BranchNotEqualEX = 0;
    end  
    
    assign Opcode = InstructionReg[31:26];
    assign FunctField = InstructionReg[5:0];
    assign ALUCtrl = ((Opcode == RTYPE_OPCODE) & (FunctField == ADD_FUNCT)) ? ALU_ADD :
         ((Opcode == RTYPE_OPCODE) & (FunctField == SUB_FUNCT)) ? ALU_SUB :
         ((Opcode == RTYPE_OPCODE) & (FunctField == AND_FUNCT)) ? ALU_AND :
         ((Opcode == RTYPE_OPCODE) & (FunctField == OR_FUNCT)) ? ALU_OR :
         ((Opcode == RTYPE_OPCODE) & (FunctField == SLT_FUNCT)) ? ALU_SLT :
         ((Opcode == RTYPE_OPCODE) & (FunctField == NOR_FUNCT)) ? ALU_NOR :
         (Opcode == LW_OPCODE) ? ALU_ADD :
         (Opcode == SW_OPCODE) ? ALU_ADD :
         (Opcode == BEQ_OPCODE || Opcode == BNE_OPCODE) ? ALU_SUB :
         (Opcode == ADDI_OPCODE) ? ALU_ADD :
         (Opcode == ANDI_OPCODE) ? ALU_AND :
         ALU_OR;  
    assign RegDst = (Opcode == RTYPE_OPCODE) ? 1'b1 :
        1'b0;
    assign ALUSrc = ((Opcode == RTYPE_OPCODE) | (Opcode == BEQ_OPCODE) | (Opcode == BNE_OPCODE)) ? 1'b0 :
        1'b1; 
    assign MemWrite = (Opcode == SW_OPCODE) ? 1'b1 :
        1'b0;
    assign RegWrite = ((Opcode == SW_OPCODE) | (Opcode == BEQ_OPCODE) | (Opcode == BNE_OPCODE) | (Opcode == JUMP_OPCODE) | JrOpcode) ? 1'b0 :
        1'b1;
    assign MemRead = (Opcode == LW_OPCODE) ? 1'b1 :
        1'b0;     
    assign MemtoReg = (Opcode == LW_OPCODE) ? 1'b1 :
        1'b0;  
    assign Branch = (Opcode == BEQ_OPCODE) ? 1'b1 :
        1'b0;
    assign BranchNotEqual = (Opcode == BNE_OPCODE);
    assign ZeroExt = ((Opcode == ORI_OPCODE) | (Opcode == ANDI_OPCODE)) ? 1'b1 :
                 1'b0;
    assign Jump = (Opcode == JUMP_OPCODE) || (Opcode == JAL_OPCODE) || ((Opcode == RTYPE_OPCODE) && (FunctField == JR_FUNCT));
           
    assign ReadReg1 = InstructionReg[25:21];
    assign ReadReg2 = InstructionReg[20:16];
    assign ImmediateValue = ((InstructionReg[15] == 1) & (ZeroExt == 0)) ? {{16{1'b1}}, InstructionReg[15:0]} :
         {16'b0,  InstructionReg[15:0]};    
    assign JrOpcode = (Opcode == RTYPE_OPCODE) && (FunctField == JR_FUNCT);

    always @(posedge clk) begin
        ReadDataReg1 <= RF[ReadReg1];
        ReadDataReg2 <= RF[ReadReg2];
        if (RegWriteWB) begin
            RF[WriteReg] <= WriteBackData;
            if (ReadReg1 == WriteReg)
                ReadDataReg1 <= WriteBackData;
            if (ReadReg2 == WriteReg)
                ReadDataReg2 <= WriteBackData;
        end
        SextReg <= ImmediateValue;
        IncrementedPCRegEX <= IncrementedPCRegID;  
        WriteRegAddress1EX <= InstructionReg[20:16];
        WriteRegAddress2EX <= InstructionReg[15:11];
        RegRsEX <= InstructionReg[25:21];
        RegRtEX <= InstructionReg[20:16];
        RegRdEX <= InstructionReg[15:11];
        JumpTarget <= InstructionReg[25:0];
       if (!(InsertNopSw || InsertNopBranch || InsertNopJump)) begin
            ALUCtrlEX <= ALUCtrl;
            ALUSrcEX <= ALUSrc;
            RegDstEX <= RegDst;
            BranchEX <= Branch;
            MemWriteEX <= MemWrite;
            MemReadEX <= MemRead;
            RegWriteEX <= RegWrite;
            MemtoRegEX <= MemtoReg;
            JumpEX <= Jump;
            JrOpcodeEX <= JrOpcode;
            BranchNotEqualEX <= BranchNotEqual;
        end
        else begin
            ALUCtrlEX <= 0;
            ALUSrcEX <= 0;
            RegDstEX <= 0;
            BranchEX <= 0;
            MemWriteEX <= 0;
            MemReadEX <= 0;
            RegWriteEX <= 0;
            MemtoRegEX <= 0;
            JumpEX <= 0;         /////////SET TO ZERO ON NOP?
            JrOpcodeEX <= 0;
            BranchNotEqualEX <= 0;
        end        
    end
    
    //Ex Stage
    wire [31:0] ALUInput2, Input2, ALUOutput;
    wire [1:0] ForwardA, ForwardB;
    reg [31:0] WriteData, ALUOutputReg;
    reg [4:0] RegWriteAddressMEM, RegRdMEM, RegRdWB, RegRtMEM;
        reg ALUZero, BranchMEM, MemWriteMEM, MemReadMEM, MemtoRegMEM, RegWriteMEM, JumpMEM, BranchNotEqualMEM;

    initial begin 
        WriteData = 32'b0;
        ALUOutputReg = 32'b0;
        RegWriteAddressMEM = 5'b0;
        ALUZero = 0;
        BranchMEM = 0;
        MemWriteMEM = 0;
        MemReadMEM = 0;
        RegWriteMEM = 0;
        MemtoRegMEM = 0;
        RegRdMEM = 0;
        RegRdWB = 0;
        RegRtMEM = 0;
        JumpMEM = 0;
        BranchNotEqualMEM = 0;
    end
    
    assign ForwardA = ((RegWriteMEM == 1'b1) 
                      && (RegRdMEM != 0) 
                      && (RegRdMEM == RegRsEX)) ? 2'b10 :
                      ((RegWriteWB == 1'b1) 
                      && (RegRdWB != 0) 
                      //&& !((RegWriteMEM == 1'b1) && (RegRdMEM != 0) && (RegRdMEM != RegRsEX)) 
                      && (RegRdWB == RegRsEX)) ? 2'b01 :
                      2'b00;                                                                                       
    assign ForwardB = (ALUSrcEX != 1'b1)
                      && ((RegWriteMEM == 1'b1) 
                      && (RegRdMEM != 0) 
                      && (RegRdMEM == RegRtEX)) ? 2'b10 :
                      (ALUSrcEX != 1'b1)
                      && ((RegWriteWB == 1'b1) 
                      && (RegRdWB != 0) 
                      //&& !((RegWriteMEM == 1'b1) && (RegRdMEM != 0) && (RegRdMEM != RegRtEX)) 
                      && (RegRdWB == RegRtEX)) ? 2'b01 :
                      2'b00;
    assign ALUInput1 = (ForwardA == 2'b00) ? ReadDataReg1 :
                       (ForwardA == 2'b01) ? WriteBackData :
                       ALUOutputReg;
    assign Input2 = (ALUSrcEX == 0) ? ReadDataReg2 :
                    (SextReg);
    assign ALUInput2 = (ForwardB == 2'b00) ? Input2 :
                       (ForwardB == 2'b01) ? WriteBackData :
                       ALUOutputReg;
    assign ALUOutput = (JumpEX == 1'b1) ? IF_PC :
                       (ALUCtrlEX == 4'b0000) ? (ALUInput1 & ALUInput2) :
                       (ALUCtrlEX == 4'b0001) ? (ALUInput1 | ALUInput2) :
                       (ALUCtrlEX == 4'b0010) ? (ALUInput1 + ALUInput2) :
                       (ALUCtrlEX == 4'b0110) ? (ALUInput1 - ALUInput2) :
                       (ALUCtrlEX == 4'b0111) ? $signed(ALUInput1) < $signed(ALUInput2) :
                       (ALUCtrlEX == 4'b1100) ? ~(ALUInput1 | ALUInput2) :
                       ALUOutput;           
                       
    always@(posedge clk) begin
        ALUOutputReg <= ALUOutput;
        BranchAddress <= IncrementedPCRegEX + (SextReg << 2);
        ALUZero <= (ALUOutput == 0);
        WriteData <= ALUInput2;
        if (JumpEX)
            RegWriteAddressMEM <= 5'b11111;
        else if (RegDstEX)
            RegWriteAddressMEM <= WriteRegAddress2EX; //This is the same as the if statement below with RegRdMEM
        else
            RegWriteAddressMEM <= WriteRegAddress1EX;
        BranchMEM <= BranchEX; 
        MemWriteMEM <= MemWriteEX; 
        MemReadMEM <= MemReadEX; 
        MemtoRegMEM <= MemtoRegEX;
        RegWriteMEM <= RegWriteEX;
        JumpMEM <= JumpEX;
        BranchNotEqualMEM <= BranchNotEqualEX;
        if (RegDstEX)
            RegRdMEM <= RegRdEX;
        else
            RegRdMEM <= RegRtEX;
        RegRtMEM <= RegRtEX;
    end
    
    //Mem Stage
    reg MemtoRegWB, BranchTakenWB;
    reg [31:0] DataMem[1023:0];
    reg [31:0] dReadData, ALUResultMEM;
    reg [4:0] RegWriteAddressWB;
    wire [31:0] DataAddress32Bit, WriteDataFwd;
    wire [9:0] DataAddress;  
    wire ForwardToMem;
    
    
    assign DataAddress32Bit = (ALUOutputReg - 32'h00001000);
    assign DataAddress = DataAddress32Bit[11:2]; 
    assign PCSrc = (BranchMEM == 1 && ALUZero == 1) || (BranchNotEqualMEM == 1 && ALUZero == 0) ? 1'b1 :
                    1'b0;
    assign ForwardToMem = ((RegWriteWB == 1'b1) && (RegRdWB != 0) && (RegRtMEM == RegRdWB)) ? 1'b1 :
                      1'b0;
    assign WriteDataFwd = (ForwardToMem == 1'b1) ? WriteBackData :
                          WriteData;
    
    initial begin
        $readmemh("final_data_memory.txt", DataMem);
        RegWriteWB = 0;
        MemtoRegWB = 0;
        BranchTakenWB = 0;
        dReadData = 32'b0;
        ALUResultMEM = 32'b0;
        RegWriteAddressWB = 5'b0;
    end
    
    always@(posedge clk) begin
        if (ALUOutputReg >= 32'h00001000 && ALUOutputReg <= 32'h00001fff) begin
            if (MemReadMEM) 
                dReadData <= DataMem[DataAddress];
            if (MemWriteMEM)
                DataMem[DataAddress] <= WriteDataFwd;
        end
        RegWriteWB <= RegWriteMEM;
        MemtoRegWB <= MemtoRegMEM;
        ALUResultMEM <= ALUOutputReg;
        RegWriteAddressWB <= RegWriteAddressMEM;
        RegRdWB <= RegRdMEM;
        BranchTakenWB <= PCSrc;
    end
    
    //WB Stage
    assign WriteReg = RegWriteAddressWB;
    assign WriteBackData = (MemtoRegWB == 1) ? dReadData :
                 ALUResultMEM;   
                 
    //Hazard Control
    assign InsertNopSw = (MemReadEX && ((RegRtEX == InstructionReg[25:21]) || (RegRtEX == InstructionReg[20:16]))) ? 1'b1 :
                   1'b0;
    assign HaltPC = (InsertNopSw || BranchEX || BranchNotEqualEX || (Branch && !BranchEX && !BranchMEM) || (BranchNotEqual && !BranchNotEqualEX && !BranchNotEqualMEM)|| ((Jump && !JumpEX) && !InsertNopJump));
    assign InsertNopBranch = (BranchEX || BranchMEM || BranchNotEqualEX || BranchNotEqualMEM || BranchTakenWB);
    assign InsertNopJump = JumpEX || JumpMEM;
                 
    //Top Level
    assign ALUResult = ALUOutput;
    assign PC = IF_PC;
    assign Instruction = InstructionReg;
    assign dWriteData = WriteDataFwd;
endmodule
