//--------------------------------------------------------------------------------------------
//
// Generated by X-HDL VHDL Translator - Version 2.0.0 Feb. 1, 2011
// Tue Jul 30 2019 21:09:44
//
//      Input file      : 
//      Component name  : abusmultiplexer
//      Author          : 
//      Company         : 
//
//      Description     : 
//
//
//--------------------------------------------------------------------------------------------

//**************************************************************************************************** 
// A bus multiplexer for ARM7TDMI-S processor 
// Designed by Ruslan Lepetenok 
// Modified 04.12.2002 
//**************************************************************************************************** 

module abusmultiplexer(
   RegFileAOut,
   MultiplierAOut,
   CPSROut,
   SPSROut,
   RegFileAOutSel,
   MultiplierAOutSel,
   CPSROutSel,
   SPSROutSel,
   ABusOut
);
   // Data input 
   input [31:0]  RegFileAOut;
   input [31:0]  MultiplierAOut;
   input [31:0]  CPSROut;
   input [31:0]  SPSROut;
   // Control 
   input         RegFileAOutSel;
   input         MultiplierAOutSel;
   input         CPSROutSel;
   input         SPSROutSel;
   // Data output 
   output [31:0] ABusOut;
   
   
   generate
      begin : xhdl0
         genvar        i;
         for (i = 31; i >= 0; i = i - 1)
         begin : ABusMux
            assign ABusOut[i] = (RegFileAOut[i] & RegFileAOutSel) | (MultiplierAOut[i] & MultiplierAOutSel) | (CPSROut[i] & CPSROutSel) | (SPSROut[i] & SPSROutSel);
         end
      end
   endgenerate
   
endmodule
