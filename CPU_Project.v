module 	Instruction_RAM	(
	
	input [31:0] address,
	output reg [31:0] instruction);

	reg	[31:0] ram[0:31];
	
	//Type0
	assign ram[0] = 32'h08041000; //ADD Rs1[0],Rd[2],Rs2[1]; >> R[0] = 6, R[1] = 1 <<
	assign ram[1] = 32'h10864000; //SUB Rs1[1],Rd[3],Rs2[2]; >>>>>>>>> R[3] = 7-1 = 6 --> R[2] = 7 <--
	
	//Type1
	assign ram[2] = 32'h00880012; //ANDI Rs[2], Rd[4], #2;  
	assign ram[3] = 32'h08CA001A; //ADDI Rs[3], Rd[5], #3;
	assign ram[4] = 32'h1002002A; //LW Rs[0], Rd[1], #5;
	assign ram[5] = 32'h1804002A; //SW Rs[0], Rd[2], #5;
	assign ram[6] = 32'h2002002A; //BEQ Rs[0], Rd[1], #5; 
	
	//Type2
	assign ram[7] = 32'h0000002C; //J #5;
	assign ram[8] = 32'h0800007C;
	
	//Type3
	assign ram[9] = 32'h000C1286;
	assign ram[10] = 32'h180E2307;
	
		
	assign instruction = ram[address];
	
endmodule

//////// Register file
module registerFile(RA, RB, RW, clk, RegWrite, BusA, BusB, BusW);
	input clk, RegWrite;		
	input [4:0] RA, RB, RW;
	input [31:0] BusW;
	output reg [31:0] BusA;
	output reg [31:0] BusB;
	reg [31:0] registers [0:31];
		
	integer i =0;
	initial begin
		  
		for (i=0;i<32;i= i+1)begin
			registers[i]=0;
		end
		
		registers[2]= 6;
		registers[3]= 5;
		registers[4]= 7;
		registers[5]= 8;
		  
	end
	
	
	assign BusA = registers[RA];	// For reading Rs
	assign BusB = registers[RB];	// For reading second operand 

	
	always @(posedge clk)
		begin
			if (RegWrite)
				registers[RW] <= BusW;
		end
	
endmodule

module Test_registerFile();
	reg clk, RegWrite;		
	reg [4:0] RA, RB, RW;
	reg [31:0] BusW;
	wire [31:0] BusA, BusB;
	
	registerFile Label1 (.RA(RA),
		.RB(RB),
		.RW(RW),
		.clk(clk),
		.RegWrite(RegWrite),
		.BusA(BusA),
		.BusB(BusB),
		.BusW(BusW)
	);
	
	initial
		begin
			assign RegWrite =1;
			assign clk =0;
			assign RA =0;
			assign RB =1;
			assign RW =2;
			assign BusW =15;
			#20 assign clk =1;
			
			#20 assign clk =0;
			assign RA = 2;
			#20 assign clk =1;
			
			#20 assign clk =0;
			$finish;
			
			
			
			
						
		end
		
endmodule

module Test_InstRAM ();
	
	reg [31:0] address;
	wire [31:0] instruction;
	
	
	Instruction_RAM Label1 (.address(address),
		.instruction(instruction)
	);
	
	
	initial
		begin
			assign address = 0;
			#20 assign address = 1;
			#20 assign address = 2;
			#20 assign address = 3;
			#20 assign address = 4;
			#20 assign address = 5;
			#20 assign address = 6;
			#20 assign address = 7;
			#20 $finish;
			
		end
		
endmodule	

module 	Data_RAM (
	
	input [31:0] Input_data,
	input clk, read, write,
	
	input [31:0] address,
	output reg [31:0] Output_Data);
	
	reg	[31:0] ram[0:31];
	
	integer i =0;
	initial begin
		  
		for (i=0;i<32;i= i+1)begin
			ram[i]= i;
		end
		
	end
	
	always @ (posedge clk)
     begin
        if (write)
           ram[address] <= Input_data;		  
		if (read)
			Output_Data <= ram[address];
     end
	
	
endmodule

module Test_DataRAM ();
	
	reg [31:0] Input_data, address;
	reg clk, read, write;
	
	wire [31:0] Output_Data;
	
	Data_RAM Label1 (.Input_data(Input_data),
		.clk(clk),
		.read(read),
		.write(write),
		.address(address),
		.Output_Data(Output_Data)
	);
	
	initial
		begin
			assign clk=0;
			assign read = 0;
			assign write = 1;
			assign Input_data = 32'b0110;
			assign address = 0;
			#20 assign clk = 1;
			#20 assign clk =0;
			
			assign read = 0;
			assign write = 1;
			assign Input_data = 32'b1110;
			assign address = 1;
			#20 assign clk = 1;
			#20 assign clk =0;
			
			assign read = 1;
			assign write = 0;
			assign address = 0;
			#20 assign clk = 1;
			#20 assign clk =0;
			
			assign read = 1;
			assign write = 0;
			assign address = 1;
			#20 assign clk = 1;
			#20 assign clk =0;
			
			$finish;
		end
		
endmodule


/////////// Mux 4x1
module Mux4X1(In1,In2,In3,In4,Out,Sel);
	 input [31:0]In1,In2,In3,In4;
	 input [1:0]Sel;
	 output reg [31:0]Out;
	 
	 always @(*)  
		begin
		 	case(Sel)
			 2'b00: Out = In1;
			 2'b01: Out = In2;
			 2'b10: Out = In3;
			 2'b11: Out = In4;
			endcase
		end
		
endmodule
/////////// Mux 2x1
module Mux2X1(In1, In2, s,out);
	input [31:0] In1,In2;
	input s;
	output reg [31:0] out;
	
	always @(*)  
		begin
		 	case(s)
			 2'b0: out = In1;
			 2'b1: out = In2;
			endcase
		end
	

endmodule

module TestMux2x1 ();
	reg [31:0] In1,In2;
	reg s;
	wire [31:0] out;
	
	Mux2X1 Label1 (.In1(In1),
		.In2(In2),
		.s(s),
		.out(out)
	);

	initial begin
		In1 = 10;
		In2 = 15;
		s=0;
		#20 s=1;
		#20 $finish;
	end
	

endmodule

	

////////// 32-bit Adder
module Adder32( IN1, IN2, OUT, Cin, Cout);
	input [31:0] IN1, IN2;
	input Cin;
	output [31:0] OUT;
	output Cout;
	
	assign {Cout, OUT} = IN1 +IN2 + Cin;
	
endmodule

module Adder_TB();
	reg [31:0] IN1, IN2;
	reg Cin;
	wire [31:0] OUT;
	wire Cout;
	
	Adder32 Label1 (.IN1(IN1),
		.IN2(IN2),
		.OUT(OUT),
		.Cin(Cin),
		.Cout(Cout)
	);
	
	initial
		begin
			
			assign IN1 = 6;
			assign IN2 = 4;
			assign Cin = 0;
			#20 $finish;
			
		end
		
endmodule
//////// Shifter  00->SLL    01->SRL    10->SLLV    11->SLRV

module Shifter(IN,ControlBits,ShiftAmount,Rs2,OUT);
	input [31:0] IN;  
	input Rs2;
	input [1:0] ControlBits;
	input [4:0] ShiftAmount;
	output reg [31:0] OUT;
	
	always @(*)
		
		begin	
			
			if (ShiftAmount == 5'b00000)
				OUT = IN;
			else
				begin
					case (ControlBits) 
						5'b00000: OUT = IN << ShiftAmount;
						5'b00001: OUT = IN >> ShiftAmount;
						5'b00010: OUT = IN << Rs2;
						5'b00011: OUT = IN >> Rs2; 
					endcase
				end
		end 
		
endmodule 	

///////Set less than module		
module SLT_Check(sign,SLT_Result);
	input sign;
  	output reg [31:0]SLT_Result;
  
  	always @(*)
  		begin
    		if (sign == 1)
      			SLT_Result = 32'b1;
    		else	
     			SLT_Result = 32'b0;
  		end	  
		  
endmodule  


////////XOR
module XOR_32bit(XOR_Output,X,Y);

	input [31:0] X,Y;
 	output reg[31:0] XOR_Output;
	
	assign XOR_Output = X ^ Y;

endmodule

///////OR
module OR_32bit(OR_Output,X,Y);

	input [31:0] X,Y;
 	output reg[31:0] OR_Output;
	
	assign OR_Output = X | Y;

endmodule 

//////AND
module AND_32bit(AND_Output,X,Y);

	input [31:0] X,Y;
 	output reg[31:0] AND_Output;
	
	assign AND_Output = X & Y;

endmodule


////////NOT
module NOT_32bit(OUT,IN);
	
	input [31:0] IN;
	output [31:0]OUT;
	
	assign OUT= ~IN;

endmodule

module NOT_32bit_TB ();

    // Inputs
    reg [31:0] IN;
    
    // Outputs
    wire [31:0] OUT;
    
    // Instantiate the module under test
    NOT_32bit duTestt (
        .IN(IN),
        .OUT(OUT)
    );
    
    
    // Test stimulus
    initial 
		begin
        IN = 32'b1;
        
        // Apply stimulus
        #10
        
        // Change input value
        IN = 0;
        
        #10 $finish;
    end
   
endmodule


//// 00 Rtype     01 J-type      10 I-type    11 S-type 
module Alu (A,B,ALUresult, ZeroFlag,AluOp,InstructionType,SA);
	input [31:0] A,B;
	input [4:0] SA,AluOp;
	input [1:0] InstructionType;
	output reg [31:0] ALUresult;
	output reg ZeroFlag;
	
	always @(*) begin
    case (InstructionType)
      2'b00: // R-type
      begin
        case (AluOp)
          5'b00000: ALUresult <= A & B;
          5'b00001: ALUresult <= A + B;
          5'b00010: begin
            ALUresult <= A - B;
            if (ALUresult == 0)
              ZeroFlag <= 1;
          end
          5'b00011: begin
            if (A < B) begin
              ZeroFlag <= 1;
              ALUresult <= 0;
            end
          end
          default: ALUresult <= 0;
        endcase
      end
      
      2'b01: // I-type
      begin
        case (AluOp)
          5'b00000: ALUresult <= A & B;
          5'b00001: ALUresult <= A + B;
		  5'b00010: ALUresult <= A + B;
		  5'b00011: ALUresult <= A + B;
		  5'b00100: ZeroFlag <= 1;
          // Add additional cases if needed
          default: ALUresult <= ALUresult;
        endcase
      end
      
      2'b11:  //S-Type
      begin	
		ZeroFlag <= 0;
        case (AluOp)
          5'b00000: ALUresult <= A << SA; // sll
          5'b00001: ALUresult <= A >> SA; // slr
          5'b00010: ALUresult <= A << B;  // sllv
          5'b00011: ALUresult <= A >> B;  // slrv
          default:ALUresult <= ALUresult;
        endcase
      end
	  
	  default: ALUresult <= 0;
    endcase
  end
endmodule 

 ////////ALU testBench
module Alu_tb;
  reg [31:0] A, B;
  reg [4:0] SA;
  reg [1:0] AluOp, InstructionType;
  wire [31:0] ALUresult;
  wire ZeroFlag;
  
  Alu uut (
    .A(A),
    .B(B),
    .SA(SA),
    .AluOp(AluOp),
    .InstructionType(InstructionType),
    .ALUresult(ALUresult),
    .ZeroFlag(ZeroFlag)
  );
  
  initial begin
    // Test case 1
    A = 5;
    B = 3;
    SA = 2;
    AluOp = 5'b00000;  // AND
    InstructionType = 2'b00;  // R-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Test case 2
    A = 5;
    B = 3;
    SA = 0;
    AluOp = 5'b00001;  // ADD
    InstructionType = 2'b00;  // R-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Test case 3
    A = 5;
    B = 5;
    SA = 0;
    AluOp = 5'b00010;  // SUB
    InstructionType = 2'b00;  // R-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Test case 4
    A = 5;
    B = 3;
    SA = 0;
    AluOp = 5'b00011;  // SLT
    InstructionType = 2'b00;  // R-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Test case 5
    A = 5;
    B = 3;
    SA = 0;
    AluOp = 5'b00000;  // AND
    InstructionType = 2'b01;  // I-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Test case 6
    A = 5;
    B = 3;
    SA = 0;
    AluOp = 5'b00001;  // ADD
    InstructionType = 2'b01;  // I-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Test case 7
    A = 5;
    B = 3;
    SA = 2;
    AluOp = 5'b00000;  // SLL
    InstructionType = 2'b11;  // S-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Test case 8
    A = 5;
    B = 3;
    SA = 2;
    AluOp = 5'b00001;  // SRL
    InstructionType = 2'b11;  // S-type
    #10;
    $display("ALUresult: %d, ZeroFlag: %b", ALUresult, ZeroFlag);
    
    // Add more test cases if needed
    
    $finish;
  end
  
endmodule

module TB2_ALU ();
	
	reg [31:0] A, B;
    reg [4:0] SA, AluOp;
    reg [1:0] InstructionType;
    wire [31:0] ALUresult;
    wire ZeroFlag;	
	
	Alu Label1 (.A(A),
		.B(B),
		.ALUresult(ALUresult),
		.ZeroFlag(ZeroFlag),
		.AluOp(AluOp),
		.InstructionType(InstructionType),
		.SA(SA)
	);
	
	initial begin
		
		A=6;
		B=1;
		AluOp =1;
		InstructionType = 0;
		#40 AluOp =2;
		#40 $finish;
	end
	
endmodule


////////// Register 32-bit

module Reg32(dataIn, dataOut, WriteEnable,Clk);
	input [31:0] dataIn;
	input Clk, WriteEnable;
	output reg[31:0] dataOut;
	
	
	always @(posedge Clk)
		begin 
			if(WriteEnable == 1'b1)
				dataOut <= dataIn;
		end
		
endmodule

module TB_Reg32 ();
	
	reg [31:0] dataIn;
	reg Clk, WriteEnable;
	wire [31:0] dataOut;
	
	Reg32 Label1 (.dataIn(dataIn),
		.dataOut(dataOut),
		.WriteEnable(WriteEnable),
		.Clk(Clk)
	);
	
	initial
		begin
			
			assign Clk = 0;
			assign dataIn = 12;
			assign WriteEnable =1;
			#20 assign Clk =1;
			#20 $finish;
			
		end
		
endmodule
	
///////// Instruction decode 

module instructionDecode(Instruction, Function,Rs1, Rs2, Rd, SA,Immediate,Type, Stop, SignedImmediate,Unused);	
	input [31:0] Instruction;
	output reg Stop;
	output reg [1:0] Type;
	output reg [4:0] Rs1,Rs2,Rd,SA,Function;
	output reg [8:0] Unused; 
	output reg [13:0] Immediate;
	output reg [23:0] SignedImmediate;
	
	assign stop = Instruction [0];	 
	assign Type = Instruction [2:1];
	assign Function = Instruction[31:27];
	
	always @(*) begin
	if (Type == 2'b00)	//R-Type
		begin 
			assign Rs1 = Instruction[26:22]; 
			assign Rd = Instruction[21:17];
			assign Rs2 = Instruction[16:12];
			assign Unused = Instruction[11:3];
		end	
	else if (Type == 2'b01) //I-Type
		begin
			assign Rs1 = Instruction[26:22]; 
			assign Rd = Instruction[21:17];	
			assign Immediate = Instruction [16:3];
		end
	else if (Type == 2'b10) //J-Type
		begin
			assign SignedImmediate = Instruction[26:3];
		end
	else			//S-Type
		begin
			assign Rs1 = Instruction[26:22]; 
			assign Rd = Instruction[21:17];
			assign Rs2 = Instruction[16:12];
			assign SA = Instruction[11:7];
			assign Unused = Instruction[6:3];
		end	
	end
	
endmodule

module TB_instructionDecode ();

	reg [31:0] Instruction;
	wire Stop;
	wire [1:0] Type;
	wire [4:0] Rs1,Rs2,Rd,SA,Function;
	wire [8:0] Unused; 
	wire [13:0] Immediate;
	wire [23:0] SignedImmediate;
	
	instructionDecode Label1 (.Instruction(Instruction),
		.Function(Function),
		.Rs1(Rs1),
		.Rs2(Rs2),
		.Rd(Rd),
		.SA(SA),
		.Immediate(Immediate),
		.Type(Type),
		.Stop(Stop),
		.SignedImmediate(SignedImmediate),
		.Unused(Unused)
	);
	
	initial
		begin
			assign Instruction = 32'h604C000C;  //type 2
			#20 assign Instruction = 32'h604C0018; //type 1	 
			#20 assign Instruction = 32'h70BC0012; //type 3
			#20 assign Instruction = 32'h60FC00A6; //type 4
			#20 $finish;
		end
		
	
endmodule

////// Extender
module extender (immediate, ExtOp, outp);
	input [13:0] immediate;
	input ExtOp;
	output reg [31:0] outp;
	
	//We need to concatenate 16 0 bits on the left side of the immediate in order to extend the immediate as following:
	always @(*)
		begin
			if (ExtOp)
				if (immediate[13]) // we assign it zero as the extender should not do extension if the ExtOp is 0
			  		assign outp = {18'b111111111111111111, immediate};
				else 
					assign outp = {18'b0, immediate};
			else
				assign outp = {18'b0, immediate};
					
		end
			  
endmodule

module Test_extender ();
	
	reg [13:0] immediate;
	reg ExtOp;
	wire [31:0] outp;
	
	extender Label1 (.immediate(immediate),
		.ExtOp(ExtOp),
		.outp(outp)
	);
	
	initial
		
		begin
			
			assign immediate = 2;
			assign ExtOp =0;
			#20 assign immediate = 1;
			#20 assign immediate = 3;
			#20 $finish;
		end
		
endmodule

module AND_32bit_TB;

    
    reg [31:0] X;
    reg [31:0] Y;
    wire [31:0] AND_Output;
    
    
    AND_32bit test (
        .X(X),
        .Y(Y),
        .AND_Output(AND_Output)
    );
    
    // Test stimulus
    initial begin
        // Test case 1
        X = 32'b1;
        Y = 32'b1;
        #5; // Wait for some time
        
        // Test case 2
        X = 32'b1;
        Y = 32'b0;
        #5; // Wait for some time
        
        $finish;
    end

endmodule

module OR_32bit_TB;

    
    reg [31:0] X;
    reg [31:0] Y;
    wire [31:0] OR_Output;
    
    OR_32bit dut (
        .X(X),
        .Y(Y),
        .OR_Output(OR_Output)
    );
    
 
    initial begin
        // Test case 1
        X = 32'b1;
        Y = 32'b0;
        #5; // Wait for some time
        
        // Test case 2
        X = 32'b0;
        Y = 32'b0;
        #5; // Wait for some time
        
        // Test case 3
        X = 32'b1;
        Y = 32'b1;
        #5; // Wait for some time
        
        $finish;
    end

endmodule		

module XOR_32bit_TB;

  
    reg [31:0] X;
    reg [31:0] Y;
    wire [31:0] XOR_Output;
    
    XOR_32bit dut (
        .X(X),
        .Y(Y),
        .XOR_Output(XOR_Output)
    );
    
    initial begin
        // Test case 1
        X = 32'b1;
        Y = 32'b1;
        #5; // Wait for some time
       
        // Test case 2
        X = 32'b1;
        Y = 32'b0;
        #5; // Wait for some time
      
        // Test case 3
        X = 32'b0;
        Y = 32'b0;
        #5; // Wait for some time
     
        // Test case 4
        X = 32'hAAAAAAAA;
        Y = 32'h55555555;
        #5; // Wait for some time

        $finish;
    end

endmodule

/////////Stack Pointer 
module stack_operation ( read_enable, write_enable, data_in, data_out, clk);	 
	
	input wire clk;
    input wire read_enable;
    input wire write_enable;
    input wire [31:0] data_in;
    output reg [31:0] data_out;
    
	parameter STACKDEPTH = 20;
    
    reg [31:0] StackMemory [0:STACKDEPTH-1];
    
    reg [4:0] stack_pointer = 0;

	
    always @(posedge clk) begin	   
		
	    if (write_enable && stack_pointer < STACKDEPTH) begin
	       
			StackMemory[stack_pointer] = data_in;
			stack_pointer = stack_pointer + 1;
	      $display(" sppp++++ The data was inserted in the stack!");
	   
		end
    
	    if (read_enable && stack_pointer > 0) begin
	      	//StackMemory[stack_pointer]=  32'b?; 
			stack_pointer = stack_pointer - 1;
	      data_out =  StackMemory[stack_pointer];
		  ///StackMemory[stack_pointer]=  32'b?; 
		  
	      	
	    end
   
	end

endmodule

/////////// Test bench of Stack 
module stack_operation_tb;

  reg clk;
  reg read_enable;
  reg write_enable;
  reg [31:0] data_in;
  wire [31:0] data_out;

  parameter STACKDEPTH = 20;
  reg [4:0] stack_pointer = 0;
  reg [31:0] StackMemory [0:STACKDEPTH-1];

  stack_operation dut (
    .clk(clk),
    .read_enable(read_enable),
    .write_enable(write_enable),
    .data_in(data_in),
    .data_out(data_out)
  );

  // Manually control the clock
  always begin
    #4 clk = 0;
    #4 clk = 1;
  end

  // Testcase
  initial begin
    $display("Starting testbench...");
    $display("---------------------");

    // Write operations
	read_enable  =0;
    write_enable = 1;
    data_in = 10;
    #10;
    $display("Write: data_in = %d", data_in);
    $display("Stack Pointer: %d", stack_pointer);
    StackMemory[stack_pointer] = data_in;
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    write_enable = 1;
    data_in = 20;
    #10;
    $display("Write: data_in = %d", data_in);
    stack_pointer = stack_pointer + 1;
    $display("Stack Pointer: %d", stack_pointer);
    StackMemory[stack_pointer] = data_in;
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    write_enable = 1;
    data_in = 30;
    #10;
    $display("Write: data_in = %d", data_in);
    stack_pointer = stack_pointer + 1;
    $display("Stack Pointer: %d", stack_pointer);
    StackMemory[stack_pointer] = data_in;
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    // Read operations
    write_enable = 0;
    read_enable = 1;
    #10;
	$display("Stack Pointer: %d", stack_pointer);
    $display("Read: data_out = %d", data_out);
    stack_pointer = stack_pointer - 1;
    $display("Stack Pointer: %d", stack_pointer);
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    read_enable = 1;
    #10;
    $display("Read: data_out = %d", data_out);
    stack_pointer = stack_pointer - 1;
    $display("Stack Pointer: %d", stack_pointer);
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    read_enable = 1;
    #10;
    $display("Read: data_out = %d", data_out);
    stack_pointer = stack_pointer - 1;
    $display("Stack Pointer: %d", stack_pointer);
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);	 
	
	$display("Another Write operations .................")	;
    // Write operations
	read_enable = 0;
    write_enable = 1;
    data_in = 16;
    #10;
    $display("Write: data_in = %d", data_in);
    $display("Stack Pointer: %d", stack_pointer);
    StackMemory[stack_pointer] = data_in;
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    write_enable = 1;
    data_in = 22;
    #10;
    $display("Write: data_in = %d", data_in);
    stack_pointer = stack_pointer + 1;
    $display("Stack Pointer: %d", stack_pointer);
    StackMemory[stack_pointer] = data_in;
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    write_enable = 1;
    data_in = 55;
    #10;
    $display("Write: data_in = %d", data_in);
    stack_pointer = stack_pointer + 1;
    $display("Stack Pointer: %d", stack_pointer);
    StackMemory[stack_pointer] = data_in;
    $display("StackMemory[%d]: %d", stack_pointer, StackMemory[stack_pointer]);

    // Finish simulation
    $display("---------------------");
    $display("Simulation completed.");
   
	$finish;
  end
endmodule

module mainControlUnit(Function, Type, LWSTR, RegWr, AluSrc, WBdata,MemRd,MemWr,ExtOp);
	input [4:0] Function;
	input [1:0] Type;
	output reg 	LWSTR, RegWr, AluSrc,WBdata,MemRd,MemWr ,ExtOp;
	reg [6:0] All;
	
	always @(*)begin
		
		assign All = {Function,Type};
		
		begin
			case(All)
				7'b00000_00: LWSTR = 1'b0;
				7'b00001_00: LWSTR = 1'b0;
				7'b00010_00: LWSTR = 1'b0;
				7'b00011_00: LWSTR = 1'b0;
				
				7'b00000_01: LWSTR = 1'b0;
				7'b00001_01: LWSTR = 1'b0;
				7'b00010_01: LWSTR = 1'b0;
				7'b00011_01: LWSTR = 1'b1;
				7'b00100_01: LWSTR = 1'b1;
				
				7'b00000_10: LWSTR = 1'b0;
				7'b00001_10: LWSTR = 1'b0;
				
				7'b00000_11: LWSTR = 1'b0;
				7'b00001_11: LWSTR = 1'b0;
				7'b00010_11: LWSTR = 1'b0;
				7'b00011_11: LWSTR = 1'b0;
			endcase	 
		end
		
		begin
			case(All)
				7'b00000_00: RegWr = 1'b1;
				7'b00001_00: RegWr = 1'b1;
				7'b00010_00: RegWr = 1'b1;
				7'b00011_00: RegWr = 1'b0;
				
				7'b00000_01: RegWr = 1'b1;
				7'b00001_01: RegWr = 1'b1;
				7'b00010_01: RegWr = 1'b1;
				7'b00011_01: RegWr = 1'b0;
				7'b00100_01: RegWr = 1'b0;
				
				7'b00000_10: RegWr = 1'b0;
				7'b00001_10: RegWr = 1'b0;
				
				7'b00000_11: RegWr = 1'b1;
				7'b00001_11: RegWr = 1'b1;
				7'b00010_11: RegWr = 1'b1;
				7'b00011_11: RegWr = 1'b1;
			endcase
		end	
		
		begin
			case(All)
				7'b00000_00: AluSrc = 1'b0;
				7'b00001_00: AluSrc = 1'b0;
				7'b00010_00: AluSrc = 1'b0;
				7'b00011_00: AluSrc = 1'b0;
				
				7'b00000_01: AluSrc = 1'b1;
				7'b00001_01: AluSrc = 1'b1;
				7'b00010_01: AluSrc = 1'b1;
				7'b00011_01: AluSrc = 1'b1;
				7'b00100_01: AluSrc = 1'b0;
				
				7'b00000_10: AluSrc = 1'bx;
				7'b00001_10: AluSrc = 1'bx;
				
				7'b00000_11: AluSrc = 1'b0;
				7'b00001_11: AluSrc = 1'b0;
				7'b00010_11: AluSrc = 1'b0;
				7'b00011_11: AluSrc = 1'b0;
			endcase
		end	   	   
		begin
			case(All)
				7'b00000_00: WBdata = 1'b0;
				7'b00001_00: WBdata = 1'b0;
				7'b00010_00: WBdata = 1'b0;
				7'b00011_00: WBdata = 1'bx;
				
				7'b00000_01: WBdata = 1'b0;
				7'b00001_01: WBdata = 1'b0;
				7'b00010_01: WBdata = 1'b1;
				7'b00011_01: WBdata = 1'bx;
				7'b00100_01: WBdata = 1'bx;
				
				7'b00000_10: WBdata = 1'bx;
				7'b00001_10: WBdata = 1'bx;
				
				7'b00000_11: WBdata = 1'b0;
				7'b00001_11: WBdata = 1'b0;
				7'b00010_11: WBdata = 1'b0;
				7'b00011_11: WBdata = 1'b0;
			endcase
		end	   	   
		begin
			case(All)
				7'b00000_00: MemRd = 1'b0;
				7'b00001_00: MemRd = 1'b0;
				7'b00010_00: MemRd = 1'b0;
				7'b00011_00: MemRd = 1'b0;
				
				7'b00000_01: MemRd = 1'b0;
				7'b00001_01: MemRd = 1'b0;
				7'b00010_01: MemRd = 1'b1;
				7'b00011_01: MemRd = 1'b0;
				7'b00100_01: MemRd = 1'b0;
				
				7'b00000_10: MemRd = 1'b0;
				7'b00001_10: MemRd = 1'b0;
				
				7'b00000_11: MemRd = 1'b0;
				7'b00001_11: MemRd = 1'b0;
				7'b00010_11: MemRd = 1'b0;
				7'b00011_11: MemRd = 1'b0;
			endcase
		end	   	   
		begin
			case(All)
				7'b00000_00: MemWr = 1'b0;
				7'b00001_00: MemWr = 1'b0;
				7'b00010_00: MemWr = 1'b0;
				7'b00011_00: MemWr = 1'b0;
				
				7'b00000_01: MemWr = 1'b0;
				7'b00001_01: MemWr = 1'b0;
				7'b00010_01: MemWr = 1'b0;
				7'b00011_01: MemWr = 1'b1;
				7'b00100_01: MemWr = 1'b0;
				
				7'b00000_10: MemWr = 1'b0;
				7'b00001_10: MemWr = 1'b0;
				
				7'b00000_11: MemWr = 1'b0;
				7'b00001_11: MemWr = 1'b0;
				7'b00010_11: MemWr = 1'b0;
				7'b00011_11: MemWr = 1'b0;
			endcase
		end	   	   
		begin
			case(All)
				7'b00000_00: ExtOp = 1'bx;
				7'b00001_00: ExtOp = 1'bx;
				7'b00010_00: ExtOp = 1'bx;
				7'b00011_00: ExtOp = 1'bx;
				
				7'b00000_01: ExtOp = 1'b0;
				7'b00001_01: ExtOp = 1'b0;
				7'b00010_01: ExtOp = 1'b0;
				7'b00011_01: ExtOp = 1'b0;
				7'b00100_01: ExtOp = 1'bx;
				
				7'b00000_10: ExtOp = 1'b0;
				7'b00001_10: ExtOp = 1'b1;
				
				7'b00000_11: ExtOp = 1'bx;
				7'b00001_11: ExtOp = 1'bx;
				7'b00010_11: ExtOp = 1'bx;
				7'b00011_11: ExtOp = 1'bx;
			endcase
		end	
	end
	
	
endmodule

module mainControlUnit_tb;

  // Inputs
  reg [4:0] Function;
  reg [1:0] Type;

  // Outputs
  wire LWSTR, RegWr, AluSrc, WBdata, MemRd, MemWr, ExtOp;

  // Instantiate the mainControlUnit module
  mainControlUnit dut (
    .Function(Function),
    .Type(Type),
    .LWSTR(LWSTR),
    .RegWr(RegWr),
    .AluSrc(AluSrc),
    .WBdata(WBdata),
    .MemRd(MemRd),
    .MemWr(MemWr),
    .ExtOp(ExtOp)
  );

  // Test case parameters
  parameter NUM_TEST_CASES = 4;
  reg [4:0] Function_cases [NUM_TEST_CASES];
  reg [1:0] Type_cases [NUM_TEST_CASES];
  reg [7:0] Expected_outputs [NUM_TEST_CASES*7];

  // Initialize test case values and expected outputs
  initial begin
    // Test case 1
    Function_cases[0] = 5'b00000;
    Type_cases[0] = 2'b00;
    Expected_outputs[0*7 + 0] = 1'b0;
    Expected_outputs[0*7 + 1] = 1'b1;
    Expected_outputs[0*7 + 2] = 1'b0;
    Expected_outputs[0*7 + 3] = 1'b0;
    Expected_outputs[0*7 + 4] = 1'b0;
    Expected_outputs[0*7 + 5] = 1'b0;
    Expected_outputs[0*7 + 6] = 1'bx;

    // Test case 2
    Function_cases[1] = 5'b00011;
    Type_cases[1] = 2'b01;
    Expected_outputs[1*7 + 0] = 1'b0;
    Expected_outputs[1*7 + 1] = 1'b0;
    Expected_outputs[1*7 + 2] = 1'b1;
    Expected_outputs[1*7 + 3] = 1'b0;
    Expected_outputs[1*7 + 4] = 1'b0;
    Expected_outputs[1*7 + 5] = 1'b1;
    Expected_outputs[1*7 + 6] = 1'b0;

    // Test case 3
    Function_cases[2] = 5'b00001;
    Type_cases[2] = 2'b01;
    Expected_outputs[2*7 + 0] = 1'b0;
    Expected_outputs[2*7 + 1] = 1'b1;
    Expected_outputs[2*7 + 2] = 1'b1;
    Expected_outputs[2*7 + 3] = 1'b0;
    Expected_outputs[2*7 + 4] = 1'b0;
    Expected_outputs[2*7 + 5] = 1'b0;
    Expected_outputs[2*7 + 6] = 1'b0;

    // Test case 4
    Function_cases[3] = 5'b00000;
    Type_cases[3] = 2'b10;
    Expected_outputs[3*7 + 0] = 1'b0;
    Expected_outputs[3*7 + 1] = 1'b0;
    Expected_outputs[3*7 + 2] = 1'bx;
    Expected_outputs[3*7 + 3] = 1'b0;
    Expected_outputs[3*7 + 4] = 1'b0;
    Expected_outputs[3*7 + 5] = 1'b0;
    Expected_outputs[3*7 + 6] = 1'b0;

    // Apply test cases
    for (int i = 0; i < NUM_TEST_CASES; i = i + 1) begin
      Function = Function_cases[i];
      Type = Type_cases[i];
      #10; // Allow some time for the outputs to settle

      // Compare the expected outputs with the actual outputs
      if (LWSTR !== Expected_outputs[i*7 + 0] || RegWr !== Expected_outputs[i*7 + 1] ||
          AluSrc !== Expected_outputs[i*7 + 2] || WBdata !== Expected_outputs[i*7 + 3] ||
          MemRd !== Expected_outputs[i*7 + 4] || MemWr !== Expected_outputs[i*7 + 5] ||
          ExtOp !== Expected_outputs[i*7 + 6]) begin
        $display("Test case %d failed! Unexpected outputs.", i+1);
        $finish;
      end
    end

    // All test cases passed
    $display("All test cases passed!");
    $finish;
  end

endmodule

module PC_ControlUnit (Function, Type, Stop, ZeroFlag, PCSrc);
	
	input [4:0] Function;
	input [1:0] Type;
	input Stop;
	input ZeroFlag;
	output reg [1:0] PCSrc=0;
	
	reg [7:0] All;
	
	always @(*) begin
		assign All = {Function,Type,Stop};
		
		begin
			case(All)
				8'b00000_00_0: PCSrc = 2'b00;
				8'b00001_00_0: PCSrc = 2'b00;
				8'b00010_00_0: PCSrc = 2'b00;
				8'b00011_00_0: PCSrc = 2'b00;
			
				8'b00000_01_0: PCSrc = 2'b00;
				8'b00001_01_0: PCSrc = 2'b00;
				8'b00010_01_0: PCSrc = 2'b00;
				8'b00011_01_0: PCSrc = 2'b00;
				8'b00100_01_0: if (ZeroFlag)
								PCSrc = 2'b10;
			
				8'b00000_10_0: PCSrc = 2'b01;
				8'b00001_10_0: PCSrc = 2'b11;
			
				8'b00000_11_0: PCSrc = 2'b00;
				8'b00001_11_0: PCSrc = 2'b00;
				8'b00010_11_0: PCSrc = 2'b00;
				8'b00011_11_0: PCSrc = 2'b00;
			endcase
			
			end
			
		end
		
endmodule

module TestPCcontrolUnit ();
	
	reg [4:0] Function;
	reg [1:0] Type;
	reg Stop;
	reg ZeroFlag;
	wire [1:0] PCSrc;
	
	PC_ControlUnit Label1 (.Function(Function),
		.Type(Type),
		.Stop(Stop),
		.ZeroFlag(ZeroFlag),
		.PCSrc(PCSrc)
	);
	
	initial begin 
		Stop =0;
		Function = 0;
		Type =0;
		
		#20 Function = 4;
		Type =1;
		ZeroFlag = 1;
		
		#20 Function = 0;
		Type =2;
		
		#20 $finish;
		
	end
endmodule

module DataPath (clk, Result, addRs1,addRs2,addRD,TRs1,TRs2,RD, OP, Ty, ins, PC, PCSrc);
	
	input clk;
	output reg [31:0] Result;
	
	//////////////////////////////////
	output reg [31:0] TRs1;
	output reg [31:0] ins;
	output reg [31:0] TRs2;
	output reg [31:0] RD;
	output reg [4:0] OP;
	output reg [1:0] Ty, PCSrc;
	output reg [4:0] addRs1;
	output reg [4:0] addRs2;
	output reg [4:0] addRD;
	

	output reg [31:0] PC = -1;
	
	wire [31:0] BusW ;
	
	wire [31:0] instruction, BusA, BusB;
	wire [4:0] Function, Rs1,Rs2,Rd, SA;
	wire [1:0] Type;
	
	wire Stop;
	reg RegWrite =0;
	
	wire [8:0] Unused;
	wire [13:0] Immediate;
	wire [23:0] SignedImmediate;
	
	wire [31:0] ALUresult;
	
	Instruction_RAM Label1 (.address(PC),
		.instruction(instruction)
	);
	
	assign ins = instruction;
	
	instructionDecode Label2 (.Instruction(instruction),
		.Function(Function),
		.Rs1(Rs1),
		.Rs2(Rs2),
		.Rd(Rd),
		.SA(SA),
		.Immediate(Immediate),
		.Type(Type),
		.Stop(Stop),
		.SignedImmediate(SignedImmediate),
		.Unused(Unused)
	);
	
	wire LWSTR, RegWr, AluSrc,WBdata,MemRd,MemWr ,ExtOp;
	
	mainControlUnit Label12 (.Function(Function),
		.Type(Type),
		.LWSTR(LWSTR),
		.RegWr(RegWr),
		.AluSrc(AluSrc),
		.WBdata(WBdata),
		.MemRd(MemRd),
		.MemWr(MemWr),
		.ExtOp(ExtOp)
	);
	
	assign addRs1 = Rs1;
	assign addRs2 = Rs2;
	assign addRD = Rd;
	
	wire RS_2;
	
	Mux2X1 Label9 (.In1(Rs2),
		.In2(Rd),
		.s(LWSTR),
		.out(RS_2)
	);
	
	registerFile Label3 (.RA(Rs1),
		.RB(RS_2),
		.RW(Rd),
		.clk(clk),
		.RegWrite(RegWr),
		.BusA(BusA),
		.BusB(BusB),
		.BusW(BusW)
	);
							
	reg ZeroFlag;
	
	wire[31:0] B;
	wire[31:0] immediateEx;
	
	extender Label4 (.immediate(Immediate),
		.ExtOp(ExtOp),
		.outp(immediateEx)
	);
	
	Mux2X1 Label5 (.In1(BusB),
		.In2(immediateEx),
		.s(AluSrc),
		.out(B)
	);
	
	assign TRs1 = BusA;
	assign TRs2 = B;
	assign OP = Function;
	assign Ty = Type;
	
	wire [31:0] ALU_result;
	
	Alu Label6 (.A(BusA),
		.B(B),
		.ALUresult(ALU_result),
		.ZeroFlag(ZeroFlag),
		.AluOp(Function),
		.InstructionType(Type),
		.SA(SA)
	);
	
	PC_ControlUnit Label11 (.Function(Function),
		.Type(Type),
		.Stop(Stop),
		.ZeroFlag(ZeroFlag),
		.PCSrc(PCSrc)
	);
	
	wire [31:0] MemDataOut ;
	
	Data_RAM Label7 (.Input_data(BusB),
		.clk(clk),
		.read(MemRd),
		.write(MemWr),
		.address(ALU_result),
		.Output_Data(MemDataOut)
	);
	
	
	Mux2X1 Label8 (.In1(ALU_result),
		.In2(MemDataOut),
		.s(WBdata),
		.out(BusW)
	);
	
	assign Result = BusW;

	
	always @(posedge clk) begin
		
		PC <= PC+1;
		
		if(PCSrc == 2)begin
			PC <= immediateEx + PC +1;
		end
		if (PCSrc== 1)begin
			PC <= SignedImmediate;
		end
		if (PCSrc ==0)begin
			PC <= PC+1;
		end
		
			
		
	end
	
	
endmodule

