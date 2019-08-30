/* gba_top.sv
 *
 *  Top module for the Game Boy Advance.
 *
 *  Team N64
 */

`include "gba_core_defines.vh"
`include "gba_mmio_defines.vh"
`default_nettype none

module gba_top (
	input  logic  CLK_50M,

	input logic gba_clk,
	input logic vga_clk,

	input  logic  reset,

	input  logic [7:0] SW,

	input  logic JA1,
	output logic JA2, JA3,

	output logic [7:0] LD,

	output logic cpu_clk_o,

	output logic [31:0] ext_bus_addr,
	output logic        cart_rd,
	output logic        bios_rd,
	input  logic [31:0] cart_data,
	input  logic [31:0] bios_data,
	output logic [1:0]  cart_bus_size,

	output logic [4:0] VGA_R, VGA_G, VGA_B,
	output logic VGA_VS, VGA_HS, VGA_DE,

	output logic AC_ADR0, AC_ADR1, AC_GPIO0, AC_MCLK, AC_SCK,
	input  logic AC_GPIO1, AC_GPIO2, AC_GPIO3,
	inout  wire  AC_SDA,

	output logic [23:0] output_wave_l,
	output logic [23:0] output_wave_r,



	output logic hblank,
	output logic vblank,

	input logic [15:0] buttons,

	input logic cpu_pause
);

	reg [6:0] GBA_CLK_DIV;

	always @(posedge gba_clk or posedge reset)
	if (reset) GBA_CLK_DIV <= 0;
	else GBA_CLK_DIV <= GBA_CLK_DIV + 1'b1;

	wire clk_100 = GBA_CLK_DIV[5];
	wire clk_256 = GBA_CLK_DIV[6];

	assign cart_bus_size = bus_size;

	assign cpu_clk_o = gba_clk;

	// 16.776 MHz clock for GBA/memory system
	//logic gba_clk, clk_100, clk_256, vga_clk;

	 //clk_wiz_0 clk0 (.clk_in1(GCLK),.gba_clk, .clk_100, .clk_256, .vga_clk);


	// Buttons register output
	//logic [15:0] buttons;

	// CPU
	logic  [4:0] mode;
	logic        nIRQ;
	logic        abort;
	logic        cpu_preemptable;

	// Interrupt signals
	logic [15:0] reg_IF, reg_IE, reg_ACK;
	logic        timer0, timer1, timer2, timer3;

	// DMA
	logic        dmaActive;
	logic        dma0, dma1, dma2, dma3;
	logic  [3:0] disable_dma;
	logic        sound_req1, sound_req2;

	// Timer
	logic [15:0] internal_TM0CNT_L;
	logic [15:0] internal_TM1CNT_L;
	logic [15:0] internal_TM2CNT_L;
	logic [15:0] internal_TM3CNT_L;
	logic [15:0] TM0CNT_L, TM1CNT_L, TM2CNT_L, TM3CNT_L;

	// Memory signals
	logic [31:0] bus_addr, bus_wdata, bus_rdata;
	logic  [1:0] bus_size;
	logic        bus_pause, bus_write;


	logic [31:0] gfx_vram_A_addr, gfx_vram_B_addr, gfx_vram_C_addr;
	logic [31:0] gfx_vram_A_addr2, gfx_palette_bg_addr;
	logic [31:0] gfx_oam_addr, gfx_palette_obj_addr;
	logic [31:0] gfx_vram_A_data, gfx_vram_B_data, gfx_vram_C_data;
	logic [31:0] gfx_vram_A_data2, gfx_palette_bg_data;
	logic [31:0] gfx_oam_data, gfx_palette_obj_data;

	logic        FIFO_re_A, FIFO_re_B, FIFO_clr_A, FIFO_clr_B;
	logic [31:0] FIFO_val_A, FIFO_val_B;
	logic  [3:0] FIFO_size_A, FIFO_size_B;

	//logic  vblank, hblank;
	 logic vcount_match;
	assign vblank = (vcount >= 8'd160);
	assign hblank = (hcount >= 9'd240);

	 assign VGA_VS = vcount>=196 && vcount<200;
	 assign VGA_HS = hcount>=280 && hcount<290;

	 assign VGA_DE = !(vblank | hblank);


	assign vcount_match = (vcount == IO_reg_datas[`DISPSTAT_IDX][15:8]);

	logic [31:0] IO_reg_datas [`NUM_IO_REGS-1:0];

	logic        dsASqRst, dsBSqRst;

	// Graphics
	logic [7:0] vcount;
	logic [8:0] hcount;

	assign abort = 1'b0;

	// CPU
	cpu_top cpu (
		.clock(gba_clk),
		.reset(reset),
		.nIRQ,
		.pause(bus_pause | cpu_pause),
		.abort,
		.mode,
		.preemptable(cpu_preemptable),
		.dmaActive,
		.rdata(bus_rdata),
		.addr(bus_addr),
		.wdata(bus_wdata),
		.size(bus_size),
		.write(bus_write)
	);

	interrupt_controller intc (
		.clock(gba_clk),
		.reset(reset),
		.cpu_mode(mode),
		.nIRQ,
		.ime(IO_reg_datas[`IME_IDX][0]),
		.reg_IF,
		.reg_ACK,
		.reg_IE(IO_reg_datas[`IE_IDX][15:0]),
		.vcount,
		.hcount,
		.set_vcount(IO_reg_datas[`DISPSTAT_IDX][15:8]),
		.timer0,
		.timer1,
		.timer2,
		.timer3,
		.serial(1'b0),
		.keypad(1'b0),
		.game_pak(1'b0),
		.dma0,
		.dma1,
		.dma2,
		.dma3
	);

	// BRAM memory controller
	mem_top mem (
		.clock(gba_clk),
		.reset(reset),
		.bus_addr,
		.bus_wdata,
		.bus_rdata,
		.bus_size,
		.bus_pause,
		.bus_write,
		.dmaActive,

		.gfx_vram_A_addr,
		.gfx_vram_B_addr,
		.gfx_vram_C_addr,
		.gfx_palette_obj_addr,
		.gfx_palette_bg_addr,
		.gfx_vram_A_addr2,
		.gfx_oam_addr,

		.gfx_vram_A_data,
		.gfx_vram_B_data,
		.gfx_vram_C_data,
		.gfx_palette_obj_data,
		.gfx_palette_bg_data,
		.gfx_vram_A_data2,
		.gfx_oam_data,

		.cart_rd(cart_rd),
		.bios_rd(bios_rd),
		.ext_bus_addr(ext_bus_addr),
		.cart_data(cart_data),
		.bios_data(bios_data),

		.IO_reg_datas,

		.buttons,
		.vcount(vcount),
		.reg_IF,
		.int_acks(reg_ACK),
		.internal_TM0CNT_L,
		.internal_TM1CNT_L,
		.internal_TM2CNT_L,
		.internal_TM3CNT_L,
		.TM0CNT_L,
		.TM1CNT_L,
		.TM2CNT_L,
		.TM3CNT_L,
		.dsASqRst,
		.dsBSqRst,

		.FIFO_re_A,
		.FIFO_re_B,
		.FIFO_clr_A,
		.FIFO_clr_B,
		.FIFO_val_A,
		.FIFO_val_B,
		.FIFO_size_A,
		.FIFO_size_B,
		.vblank,
		.hblank,
		.vcount_match
	);

	graphics_system gfx (
		.gfx_vram_A_addr,
		.gfx_vram_B_addr,
		.gfx_vram_C_addr,
		.gfx_oam_addr,
		.gfx_palette_bg_addr,
		.gfx_palette_obj_addr,
		.gfx_vram_A_addr2,

		.gfx_vram_A_data,
		.gfx_vram_B_data,
		.gfx_vram_C_data,
		.gfx_oam_data,
		.gfx_palette_bg_data,
		.gfx_palette_obj_data,
		.gfx_vram_A_data2,

		.IO_reg_datas,
		.graphics_clock(gba_clk),
		.vga_clock(vga_clk),
		.reset(reset),
		.vcount,
		.hcount,
		.VGA_R,
		.VGA_G,
		.VGA_B
		, .VGA_HS, .VGA_VS
	);

	dma_top dma (
		.clk(gba_clk),
		.rst_b(~reset),
		.registers(IO_reg_datas),
		.addr(bus_addr),
		.rdata(bus_rdata),
		.wdata(bus_wdata),
		.size(bus_size),
		.wen(bus_write),
		.active(dmaActive),
		.disable_dma(),
		.irq0(dma0),
		.irq1(dma1),
		.irq2(dma2),
		.irq3(dma3),
		.mem_wait(bus_pause | cpu_pause),
		.sound_req1(sound_req1),
		.sound_req2(sound_req2),
		.vcount(vcount),
		.hcount({7'd0, hcount}),
		.cpu_preemptable(cpu_preemptable)
	);

	timer_top timers (
		.clock_16(gba_clk),
		.reset(reset),
		.IO_reg_datas,
		.internal_TM0CNT_L,
		.internal_TM1CNT_L,
		.internal_TM2CNT_L,
		.internal_TM3CNT_L,
		.TM0CNT_L,
		.TM1CNT_L,
		.TM2CNT_L,
		.TM3CNT_L,
		.genIRQ0(timer0),
		.genIRQ1(timer1),
		.genIRQ2(timer2),
		.genIRQ3(timer3)
	);

	gba_audio_top audio (
		.clk_100(clk_100),
		.clk_256,
		.gba_clk,
		.reset(reset),
		.AC_ADR0,
		.AC_ADR1,
		.AC_GPIO0,
		.AC_GPIO1,
		.AC_GPIO2,
		.AC_GPIO3,
		.AC_MCLK,
		.AC_SCK,
		.AC_SDA,
		.IO_reg_datas,
		.sound_req1,
		.sound_req2,
		.internal_TM0CNT_L,
		.internal_TM1CNT_L,
		.dsASqRst,
		.dsBSqRst,
		.SW(SW[0]),

		.output_wave_l( output_wave_l ),
		.output_wave_r( output_wave_r ),

		.FIFO_re_A,
		.FIFO_re_B,
		.FIFO_clr_A,
		.FIFO_clr_B,
		.FIFO_val_A,
		.FIFO_val_B,
		.FIFO_size_A,
		.FIFO_size_B
	);


endmodule: gba_top

`default_nettype wire
