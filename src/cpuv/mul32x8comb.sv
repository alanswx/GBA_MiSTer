//--------------------------------------------------------------------------------------------
//
// Generated by X-HDL VHDL Translator - Version 2.0.0 Feb. 1, 2011
// Tue Jul 30 2019 21:12:12
//
//      Input file      : 
//      Component name  : mul32x8comb
//      Author          : 
//      Company         : 
//
//      Description     : 
//
//
//--------------------------------------------------------------------------------------------

//**************************************************************************************************** 
// 32x8 Combinatorial Multiplier for ARM core 
// Designed by Ruslan Lepetenok 
// Modified 12.02.2003 
//**************************************************************************************************** 

module mul32x8comb(
   RmIn,
   Rs9In,
   PartialSumIn,
   PartialCarryIn,
   PartialSumOut,
   PartialCarryOut,
   UMul,
   PP4P,
   PP4M
);
   input [31:0]  RmIn;
   input [8:0]   Rs9In;
   input [63:0]  PartialSumIn;
   input [63:0]  PartialCarryIn;
   output [63:0] PartialSumOut;
   output [63:0] PartialCarryOut;
   input         UMul;
   input         PP4P;
   input         PP4M;
   
   
   // Booth decoder signals 
   wire [3:0]    ShiftPPLeft;
   wire [3:0]    NegPP;
   wire [3:0]    ClrPP;
   
   reg [63:0]    PartialProduct[4:0];
   
   wire [31+1:0] PX;		// +X 
   wire [31+1:0] MX;		// -X 
   
   // Carry save adders (CSA) signals 
   reg [63:0]    CSA_AIn[4:0];
   reg [63:0]    CSA_BIn[4:0];
   reg [63:0]    CSA_CarryIn[4:0];
   reg [63:0]    CSA_SumOut[4:0];
   reg [63:0]    CSA_CarryOut[4:0];
   
   // Added for the purpose of test only 
   wire [63:0]   SumTest;
   
   generate
      begin : xhdl0
         genvar        i;
         for (i = 3; i >= 0; i = i - 1)
         begin : BoothDecoder
            assign ShiftPPLeft[i] = (Rs9In[i * 2 + 2:i * 2] == 3'b011 | Rs9In[i * 2 + 2:i * 2] == 3'b100) ? 1'b1 : 		// 2X 
                                    1'b0;
            assign NegPP[i] = (Rs9In[i * 2 + 2] == 1'b1) ? 1'b1 : 		// -X/-2X 
                              1'b0;
            assign ClrPP[i] = (Rs9In[i * 2 + 2:i * 2] == 3'b000 | Rs9In[i * 2 + 2:i * 2] == 3'b111) ? 1'b1 : 		// Clear must have higher priority than	Neg	 
                              1'b0;
         end
      end
   endgenerate

	assign PX = (UMul == 1'b0) ? {RmIn[31], RmIn} :              // For the signed multiplication 
               {1'b0, RmIn};            // For the unsigned multiplication 
   
   assign MX = (UMul == 1'b0) ? (~({RmIn[31], RmIn})) + 1 : 		// For the signed multiplication 
               (~({1'b0, RmIn})) + 1;		// For the unsigned multiplication 
   
   always @(*) PartialProduct[0] <= //(ClrPP[0] == 1'b1) ? {(63:0][4-0)+1{1'b0}} : 		// +/-0  X-HDL
									(ClrPP[0] == 1'b1) ? 64'h0000000000000000 : 		// +/-0 
                                    (ShiftPPLeft[0] == 1'b0 & NegPP[0] == 1'b0) ? {{31{PX[32]}}, PX} : 		// +X 
                                    (ShiftPPLeft[0] == 1'b0 & NegPP[0] == 1'b1) ? {{31{MX[32]}}, MX} : 		// -X 
                                    (ShiftPPLeft[0] == 1'b1 & NegPP[0] == 1'b0) ? {{30{PX[32]}}, PX, 1'b0} : 		// +2X 
                                    (ShiftPPLeft[0] == 1'b1 & NegPP[0] == 1'b1) ? {{30{MX[32]}}, MX, 1'b0} : 		// -2X 
                                    //{(63:0][4-0)+1{CDnCr}};	// X-HDL
									64'h0000000000000000;
   
   always @(*) PartialProduct[1] <= //(ClrPP[1] == 1'b1) ? {(63:0][4-0)+1{1'b0}} : 		// +/-0  X-HDL
									(ClrPP[1] == 1'b1) ? 64'h0000000000000000 : 		// +/-0
                                    (ShiftPPLeft[1] == 1'b0 & NegPP[1] == 1'b0) ? {{29{PX[32]}}, PX, 2'b00} : 		// +X 
                                    (ShiftPPLeft[1] == 1'b0 & NegPP[1] == 1'b1) ? {{29{MX[32]}}, MX, 2'b00} : 		// -X 
                                    (ShiftPPLeft[1] == 1'b1 & NegPP[1] == 1'b0) ? {{28{PX[32]}}, PX, 3'b000} : 		// +2X 
                                    (ShiftPPLeft[1] == 1'b1 & NegPP[1] == 1'b1) ? {{28{MX[32]}}, MX, 3'b000} : 		// -2X 
                                    //{(63:0][4-0)+1{CDnCr}};	// X-HDL
									64'h0000000000000000;
   
   always @(*) PartialProduct[2] <= //(ClrPP[2] == 1'b1) ? {(63:0][4-0)+1{1'b0}} : 		// +/-0  X-HDL
									(ClrPP[2] == 1'b1) ? 64'h0000000000000000 : 		// +/-0 
                                    (ShiftPPLeft[2] == 1'b0 & NegPP[2] == 1'b0) ? {{27{PX[32]}}, PX, 4'b0000} : 		// +X 
                                    (ShiftPPLeft[2] == 1'b0 & NegPP[2] == 1'b1) ? {{27{MX[32]}}, MX, 4'b0000} : 		// -X 
                                    (ShiftPPLeft[2] == 1'b1 & NegPP[2] == 1'b0) ? {{26{PX[32]}}, PX, 5'b00000} : 		// +2X 
                                    (ShiftPPLeft[2] == 1'b1 & NegPP[2] == 1'b1) ? {{26{MX[32]}}, MX, 5'b00000} : 		// -2X 
                                    //{(63:0][4-0)+1{CDnCr}};	// X-HDL
									64'h0000000000000000;
   
   always @(*) PartialProduct[3] <= //(ClrPP[3] == 1'b1) ? {(63:0][4-0)+1{1'b0}} : 		// +/-0  X-HDL
									(ClrPP[3] == 1'b1) ? 64'h0000000000000000 : 		// +/-0 
                                    (ShiftPPLeft[3] == 1'b0 & NegPP[3] == 1'b0) ? {{25{PX[32]}}, PX, 6'b000000} : 		// +X 
                                    (ShiftPPLeft[3] == 1'b0 & NegPP[3] == 1'b1) ? {{25{MX[32]}}, MX, 6'b000000} : 		// -X 
                                    (ShiftPPLeft[3] == 1'b1 & NegPP[3] == 1'b0) ? {{24{PX[32]}}, PX, 7'b0000000} : 		// +2X 
                                    (ShiftPPLeft[3] == 1'b1 & NegPP[3] == 1'b1) ? {{24{MX[32]}}, MX, 7'b0000000} : 		// -2X 
                                    //{(63:0][4-0)+1{CDnCr}};	// X-HDL
									64'h0000000000000000;
   
   always @(*) PartialProduct[4] <= (PP4P == 1'b1) ? {{23{PX[32]}}, PX, 8'b00000000} : 		// Last cycle of the unsigned multiplication or ??? +X 
                                    (PP4M == 1'b1) ? {{23{MX[32]}}, MX, 8'b00000000} : 		// -X     
                                    //{(63:0][4-0)+1{1'b0}};	// X-HDL
									64'h0000000000000000;
   
   // Carry save adders 
   
   // CSA stage 0 
   always @(*) CSA_AIn[0] <= PartialProduct[0];
   always @(*) CSA_BIn[0] <= PartialSumIn;
   always @(*) CSA_CarryIn[0] <= {PartialCarryIn[62:0], 1'b0};
   
   // CSA stages 1 to 4(?)  
   generate
      begin : xhdl1
         genvar        i;
         for (i = 1; i <= 4; i = i + 1)
         begin : CSA_Connection
            always @(*) CSA_AIn[i] <= PartialProduct[i];
            always @(*) CSA_BIn[i] <= CSA_SumOut[i - 1];
            always @(*) CSA_CarryIn[i] <= {CSA_CarryOut[i - 1][62:0], 1'b0};
         end
      end
   endgenerate
   
   generate
      begin : xhdl2
         genvar        i;
         for (i = 4; i >= 0; i = i - 1)
         begin : CarrySaveAdders
            always @(*) CSA_SumOut[i] <= CSA_AIn[i] ^ CSA_BIn[i] ^ CSA_CarryIn[i];
            always @(*) CSA_CarryOut[i] <= (CSA_AIn[i] & CSA_BIn[i]) | ((CSA_AIn[i] | CSA_BIn[i]) & CSA_CarryIn[i]);
         end
      end
   endgenerate

   
   assign PartialSumOut = CSA_SumOut[4];
   assign PartialCarryOut = CSA_CarryOut[4];
   
   // Added for the purpose of test only 
   assign SumTest = CSA_SumOut[4] + ({CSA_CarryOut[4][62:0], 1'b0});
   
endmodule
