//--------------------------------------------------------------------------------------------
//
// Generated by X-HDL VHDL Translator - Version 2.0.0 Feb. 1, 2011
// Tue Jul 30 2019 21:09:57
//
//      Input file      : 
//      Component name  : armpackage
//      Author          : 
//      Company         : 
//
//      Description     : 
//
//
//--------------------------------------------------------------------------------------------

//**************************************************************************************************** 
// Constants for ARM core 
// Designed by Ruslan Lepetenok 
// Modified 30.01.2003 
//**************************************************************************************************** 

`ifndef armpackage
`define armpackage


// ARM core modes (CPSR [4:0]) 
parameter [4:0]  CUserMode = 5'b10000;
parameter [4:0]  CFIQMode = 5'b10001;
parameter [4:0]  CIRQMode = 5'b10010;
parameter [4:0]  CSVCMode = 5'b10011;
parameter [4:0]  CAbortMode = 5'b10111;
parameter [4:0]  CUndefMode = 5'b11011;
parameter [4:0]  CSystemMode = 5'b11111;

parameter [31:0] CPSRInitVal = {1'b0, 3'b110, CSVCMode};

// Exception vector adressses 
parameter [31:0] CExcAdrUndefined = 32'h0000_0004;
parameter [31:0] CExcAdrSWI = 32'h0000_0008;
parameter [31:0] CExcAdrPrfAbt = 32'h0000_000C;
parameter [31:0] CExcAdrDtAbt = 32'h0000_0010;
parameter [31:0] CExcAdrIRQ = 32'h0000_0018;
parameter [31:0] CExcAdrFIQ = 32'h0000_001C;

// Bus transaction types (TRANS[1:0]) 
parameter [1:0]  CTT_I = 2'b00;		// Internal cycle  
parameter [1:0]  CTT_C = 2'b01;		// Coprocessor register transfer cycle	 
parameter [1:0]  CTT_N = 2'b10;		// Nonsequential cycle	 
parameter [1:0]  CTT_S = 2'b11;		// Sequential cycle 

// Bus transaction sizes (SIZE[1:0]) 
parameter [1:0]  CTS_B = 2'b00;		// Byte  
parameter [1:0]  CTS_HW = 2'b01;		// Halfword (16 bit) 
parameter [1:0]  CTS_W = 2'b10;		// Word(32 bit)	 

// TBD (depends on the address multiplexer structure) 
parameter [31:0] CPCInitVal = 32'h0000_0000;

// Symbolic register names 
parameter [3:0]  CR_PC = 4'b1111;		// PC(R15) 
parameter [3:0]  CR_LR = 4'b1110;		// LR(R14) 

// Thumb 
parameter        CThumbImp = 1'b1;

// Don't care value 
parameter        CDnCr = 1'b0;

`endif