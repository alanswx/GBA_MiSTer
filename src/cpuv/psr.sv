//--------------------------------------------------------------------------------------------
//
// Generated by X-HDL VHDL Translator - Version 2.0.0 Feb. 1, 2011
// Tue Jul 30 2019 21:10:30
//
//      Input file      : 
//      Component name  : psr
//      Author          : 
//      Company         : 
//
//      Description     : 
//
//
//--------------------------------------------------------------------------------------------

//**************************************************************************************************** 
// Programm Status Registers for ARM core 
// Designed by Ruslan Lepetenok 
// Modified 23.01.2003 
//**************************************************************************************************** 

module psr(
   nRESET,
   CLK,
   CLKEN,
   DataIn,
   PSRDInSel,
   CPSRIn,
   CPSRWrEn,
   CPSROut,
   CFlForMul,
   SPSRIn,
   SPSROut,
   SPSRWrMsk,
   PSRMode
);
   `include "armpackage.vh"
	       localparam CDnCr = 1'b0;

	parameter [4:0]  CUserMode = 5'b10000;
parameter [4:0]  CFIQMode = 5'b10001;
parameter [4:0]  CIRQMode = 5'b10010;
parameter [4:0]  CSVCMode = 5'b10011;
parameter [4:0]  CAbortMode = 5'b10111;
parameter [4:0]  CUndefMode = 5'b11011;
parameter [4:0]  CSystemMode = 5'b11111;
parameter [31:0] CPSRInitVal = {1'b0, 3'b110, CSVCMode};

	
	
	
   // Global control signals 
   input         nRESET;
   input         CLK;
   input         CLKEN;
   // ALU Data in 
   input [31:0]  DataIn;
   input         PSRDInSel;
   // Current program state 
   input [31:0]  CPSRIn;
   input [31:0]  CPSRWrEn;
   output [31:0] CPSROut;
   input         CFlForMul;
   // Saved program state 
   input [31:0]  SPSRIn;
   output [31:0] SPSROut;
   input [3:0]   SPSRWrMsk;
   // PSR mode control 
   input [4:0]   PSRMode;
   
   
   reg [31:0]    CPSR;
   reg [31:0]    SPSR_FIQ;
   reg [31:0]    SPSR_SVC;
   reg [31:0]    SPSR_Abort;
   reg [31:0]    SPSR_IRQ;
   reg [31:0]    SPSR_Undef;
   wire [31:0]   SPSRWrEn;
   wire [31:0]   SPSROutMUX;
   
   // Modes  
   wire          UserMode;
   wire          FIQMode;
   wire          IRQMode;
   wire          SVCMode;
   wire          AbortMode;
   wire          UndefMode;
   
   wire [31:0]   CPSRRegIn;
   wire [31:0]   SPSRRegIn;
   
   // Long multiplication(accumulation support) 
   reg           CFlForLMul;
   
   // Mode decode logic 
   assign UserMode = (PSRMode == CUserMode | PSRMode == CSystemMode) ? 1'b1 : 
                     1'b0;
   assign FIQMode = (PSRMode == CFIQMode) ? 1'b1 : 
                    1'b0;
   assign IRQMode = (PSRMode == CIRQMode) ? 1'b1 : 
                    1'b0;
   assign SVCMode = (PSRMode == CSVCMode) ? 1'b1 : 
                    1'b0;
   assign AbortMode = (PSRMode == CAbortMode) ? 1'b1 : 
                      1'b0;
   assign UndefMode = (PSRMode == CUndefMode) ? 1'b1 : 
                      1'b0;
   
   // Masks for write into SPSR	 
   generate
      begin : xhdl0
         genvar        i;
         for (i = 3; i >= 0; i = i - 1)
         begin : SPSRWriteEnable
            assign SPSRWrEn[i * 8 + 7:i * 8] = {32{SPSRWrMsk[i]}};
         end
      end
   endgenerate
   
   // CPSR/SPSR input multiplexer 
   assign CPSRRegIn = (PSRDInSel == 1'b1) ? DataIn : 
                      CPSRIn;
   assign SPSRRegIn = (PSRDInSel == 1'b1) ? DataIn : 
                      SPSRIn;
   
   // Current program status 
   
   always @(negedge nRESET or posedge CLK)
   begin: CurrentPSR
      integer       i;
      if (nRESET == 1'b0)		// Reset 
         CPSR <= CPSRInitVal;
      else 		// Clock 
         for (i = 31; i >= 0; i = i - 1)
            if (CPSRWrEn[i] == 1'b1 & CLKEN == 1'b1)		// Clock enable 
               CPSR[i] <= CPSRRegIn[i];
   end
   
   // Saved program statuses 
   
   always @(negedge nRESET or posedge CLK)
   begin: FIQ_PSR
      integer       i;
      if (nRESET == 1'b0)		// Reset 
         SPSR_FIQ <= {32{1'b0}};
      else 		// Clock 
         for (i = 31; i >= 0; i = i - 1)
            if (FIQMode == 1'b1 & SPSRWrEn[i] == 1'b1 & CLKEN == 1'b1)		// Clock enable 
               SPSR_FIQ[i] <= SPSRRegIn[i];
   end
   
   
   always @(negedge nRESET or posedge CLK)
   begin: IRQ_PSR
      integer       i;
      if (nRESET == 1'b0)		// Reset 
         SPSR_IRQ <= {32{1'b0}};
      else 		// Clock 
         for (i = 31; i >= 0; i = i - 1)
            if (IRQMode == 1'b1 & SPSRWrEn[i] == 1'b1 & CLKEN == 1'b1)		// Clock enable 
               SPSR_IRQ[i] <= SPSRRegIn[i];
   end
   
   
   always @(negedge nRESET or posedge CLK)
   begin: SVC_PSR
      integer       i;
      if (nRESET == 1'b0)		// Reset 
         SPSR_SVC <= {32{1'b0}};
      else 		// Clock 
         for (i = 31; i >= 0; i = i - 1)
            if (SVCMode == 1'b1 & SPSRWrEn[i] == 1'b1 & CLKEN == 1'b1)		// Clock enable 
               SPSR_SVC[i] <= SPSRRegIn[i];
   end
   
   
   always @(negedge nRESET or posedge CLK)
   begin: Abort_PSR
      integer       i;
      if (nRESET == 1'b0)		// Reset 
         SPSR_Abort <= {32{1'b0}};
      else 		// Clock 
         for (i = 31; i >= 0; i = i - 1)
            if (AbortMode == 1'b1 & SPSRWrEn[i] == 1'b1 & CLKEN == 1'b1)		// Clock enable 
               SPSR_Abort[i] <= SPSRRegIn[i];
   end
   
   
   always @(negedge nRESET or posedge CLK)
   begin: Undef_PSR
      integer       i;
      if (nRESET == 1'b0)		// Reset 
         SPSR_Undef <= {32{1'b0}};
      else 		// Clock 
         for (i = 31; i >= 0; i = i - 1)
            if (UndefMode == 1'b1 & SPSRWrEn[i] == 1'b1 & CLKEN == 1'b1)		// Clock enable 
               SPSR_Undef[i] <= SPSRRegIn[i];
   end
   
   // Output multiplexers 
   assign SPSROutMUX = (CPSR[4:0] == CFIQMode) ? SPSR_FIQ : 
                       (CPSR[4:0] == CIRQMode) ? SPSR_IRQ : 
                       (CPSR[4:0] == CSVCMode) ? SPSR_SVC : 
                       (CPSR[4:0] == CAbortMode) ? SPSR_Abort : 
                       (CPSR[4:0] == CUndefMode) ? SPSR_Undef : 
                       {32{CDnCr}};
   
   assign SPSROut = SPSROutMUX;
   
   assign CPSROut = (CFlForMul == 1'b1) ? {CPSR[31:30], CFlForLMul, CPSR[28:0]} : 		// Long multiplication 
                    CPSR;
   
   
   always @(negedge nRESET or posedge CLK)
   begin: CarryForLongMul
      if (nRESET == 1'b0)		// Reset 
         CFlForLMul <= 1'b0;
      else 		// Clock 
      begin
         if (CLKEN == 1'b1)		// Clock enable 
            CFlForLMul <= CPSRIn[29];
      end
   end
   
endmodule
`undef ARMPackage
