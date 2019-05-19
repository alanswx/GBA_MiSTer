// Copyright (c) 2012-2013 Ludvig Strigeus
// This program is GPL Licensed. See COPYING for the full license.
//
// MiSTer port: Copyright (C) 2017,2018 Sorgelig

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	// SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..5 - USR1..USR4
	// Set USER_OUT to 1 to read from USER_IN.
	input   [5:0] USER_IN,
	output  [5:0] USER_OUT,

	input         OSD_STATUS
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;

assign {UART_RTS, UART_TXD, UART_DTR} = 0;

assign AUDIO_S   = 1;
assign AUDIO_MIX = 0;

assign LED_USER  = ioctl_download;
assign LED_DISK  = 0;
assign LED_POWER = 0;

assign VIDEO_ARX = status[8] ? 8'd16 : 8'd3;
assign VIDEO_ARY = status[8] ? 8'd9 : 8'd2;

assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CKE, SDRAM_CLK, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1 = 0;

`include "build_id.v"
parameter CONF_STR = {
	"GBA;;",
	"-;",
	"F,GBABIN;",
	"-;",
	"O8,Aspect ratio,3:2,16:9;",
	"O9B,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	"R0,Reset;",
	"J,A,B,Select,Start,L,R;",
	"V,v0.10.",`BUILD_DATE
};

wire        ioctl_download;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire [15:0] ioctl_data;
wire  [7:0] ioctl_index;
reg         ioctl_wait;

wire [15:0] joystick_0;
wire [15:0] joystick_1;

wire [1:0] buttons;

wire [31:0] status;

wire arm_reset = status[0];

wire forced_scandoubler;
wire ps2_kbd_clk, ps2_kbd_data;
wire [10:0] ps2_key;

reg  [31:0] sd_lba = 0;

reg         sd_rd = 0;
reg         sd_wr = 0;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;
wire        img_mounted;
wire        img_readonly;
wire [63:0] img_size;

hps_io #(.STRLEN($size(CONF_STR)>>3), .WIDE(1)) hps_io
(
	.clk_sys(clk_16m),
	.HPS_BUS(HPS_BUS),
	.conf_str(CONF_STR),

	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),

	.status(status),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_data),
	.ioctl_wait(ioctl_wait),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),

	.ps2_key(ps2_key),

	.ps2_kbd_led_use(0),
	.ps2_kbd_led_status(0)
);

wire bios_download = ioctl_download && (ioctl_index == 8'd0);
wire cart_download = ioctl_download && (ioctl_index != 8'd0);

wire clock_locked;
wire clk_sys;
wire clk_16m;


pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_ram),
	.outclk_2(clk_16m),

	.locked(clock_locked)
);

wire clk_ram;

assign SDRAM_CLK = clk_ram;

// reset after download
reg [7:0] download_reset_cnt;
wire download_reset = download_reset_cnt != 0;
always @(posedge CLK_50M) begin
	if(ioctl_download) download_reset_cnt <= 8'd255;
	else if(download_reset_cnt != 0) download_reset_cnt <= download_reset_cnt - 8'd1;
end

// hold machine in reset until first download starts
reg init_reset;
always @(posedge CLK_50M) begin
	if(!clock_locked) init_reset <= 1'b1;
	else if(ioctl_download) init_reset <= 1'b0;
end


wire  [8:0] cycle;
wire  [8:0] scanline;
wire [15:0] sample;
wire  [5:0] color;
wire        joypad_strobe;
wire  [1:0] joypad_clock;
wire [21:0] memory_addr;
wire        memory_read_cpu, memory_read_ppu;

wire  [7:0] memory_din_cpu, memory_din_ppu;
reg   [7:0] joypad_bits, joypad_bits2;
reg   [1:0] last_joypad_clock;

reg led_blink;
always @(posedge clk_16m) begin
	int cnt = 0;
	cnt <= cnt + 1;
	if(cnt == 10000000) begin
		cnt <= 0;
		led_blink <= ~led_blink;
	end;
end

wire [7:0] SW = 8'b00000000;
wire JA1;
wire JA2;
wire JA3;
wire [7:0] LD;

wire AC_ADR0;
wire AC_ADR1;
wire AC_GPIO0;
wire AC_GPIO1;
wire AC_GPIO2;
wire AC_GPIO3;
wire AC_MCLK;
wire AC_SCK;
wire AC_SDA;

wire [4:0] GBA_R;
wire [4:0] GBA_G;
wire [4:0] GBA_B;
wire GBA_HS;
wire GBA_VS;
wire GBA_DE;
wire GBA_HBLANK;
wire GBA_VBLANK;

wire [23:0] output_wave_l;
wire [23:0] output_wave_r;

assign AUDIO_L = output_wave_l[23:8];
assign AUDIO_R = output_wave_r[23:8];

// GBA button mapping...
//
// gba_buttons[0] = // A
// gba_buttons[1] = // B
// gba_buttons[2] = // Select
// gba_buttons[3] = // Start
// gba_buttons[4] = // Right
// gba_buttons[5] = // Left
// gba_buttons[6] = // Down
// gba_buttons[7] = // Up
// gba_buttons[8] = // R
// gba_buttons[9] = // L
// gba_buttons[15:10] = 6'h3F; // (set these 6 bits HIGH).
//
wire [15:0] gba_buttons = {
	6'b111111,
	joystick_0[8],
	joystick_0[9],
	joystick_0[3],
	joystick_0[2],
	joystick_0[1],
	joystick_0[0],
	joystick_0[7],
	joystick_0[6],
	joystick_0[5],
	joystick_0[4]
};


wire reset_gba = init_reset || buttons[1] || arm_reset || download_reset;

gba_top gba_top_inst
(
	.gba_clk       (clk_16m),   // 16.776 MHz.

	.vga_clk       (CLK_50M),   // 50.33 MHz.

	.BTND          (reset_gba), // input BTND (active HIGH for Reset).

	.SW            (SW),

	.JA1           (JA1),
	.JA2           (JA2),
	.JA3           (JA3),

	.LD            (LD),

	.VGA_R         (GBA_R),
	.VGA_G         (GBA_G),
	.VGA_B         (GBA_B),
	.VGA_VS        (GBA_VS),
	.VGA_HS        (GBA_HS),
	.VGA_DE        (GBA_DE),

	.AC_ADR0       (AC_ADR0),
	.AC_ADR1       (AC_ADR1),
	.AC_GPIO0      (AC_GPIO0),
	.AC_MCLK       (AC_MCLK),
	.AC_SCK        (AC_SCK),
	.AC_GPIO1      (AC_GPIO1),
	.AC_GPIO2      (AC_GPIO2),
	.AC_GPIO3      (AC_GPIO3),
	.AC_SDA        (AC_SDA),

	.output_wave_l (output_wave_l),
	.output_wave_r (output_wave_r),

	.hblank        (GBA_HBLANK),
	.vblank        (GBA_VBLANK),

	.buttons       (gba_buttons),

	.ext_bus_addr  (bus_addr[31:0]),
	.cart_data     (cart_data),
	.bios_data     (bios_data),
	.cart_rd       (cart_rd),
	.bios_rd       (bios_rd),
	.cart_bus_size (bus_size),
	.cpu_clk_o     (clk_cpu),
	.cpu_pause     (0)
);

wire        clk_cpu;
wire [31:0] bus_addr;
wire [1:0]  bus_size;

(* keep=1 *) wire [31:0] cart_data_2;

(*keep=1*) wire cart_rd;

wire [31:0] bios_data;
wire bios_rd;

dpram_dif #(
	.addr_width_a(13),
	.data_width_a(32),
	.addr_width_b(14),
	.data_width_b(16)
) bios (
	.clock(clk_16m),

	.address_a(bus_addr[13:2]),
	.q_a(bios_data),

	.address_b(ioctl_addr[13:1]),
	.data_b(ioctl_data),
	.wren_b(ioctl_wr & bios_download)
);

dpram_dif #(
	.addr_width_a(14),
	.data_width_a(32),
	.addr_width_b(15),
	.data_width_b(16)
) cart (
	.clock(clk_16m),

	.address_a(bus_addr[14:2]),
	.q_a(cart_data),

	.address_b(ioctl_addr[14:1]),
	.data_b(ioctl_data),
	.wren_b(ioctl_wr & cart_download)
);


reg [31:0] cart_data;

wire [4:0] R = GBA_R;
wire [4:0] G = GBA_G;
wire [4:0] B = GBA_B;

assign CLK_VIDEO = clk_sys;
assign VGA_SL = sl[1:0];

wire [2:0] scale = status[11:9];
wire [2:0] sl = scale ? scale - 1'd1 : 3'd0;

video_mixer #(.LINE_LENGTH(520)) video_mixer
(
	.clk_sys(CLK_VIDEO),
	.ce_pix(clk_16m),
	.ce_pix_out(CE_PIXEL),

	.scanlines(1'b0),
	.scandoubler(scale || forced_scandoubler),
	.hq2x(scale==1),
	.mono(1'b0),

	.R({R,R[2:0]}),
	.G({G,G[2:0]}),
	.B({B,B[2:0]}),

	.HSync(GBA_HS),
	.VSync(GBA_VS),
	.HBlank(GBA_HBLANK),
	.VBlank(GBA_VBLANK),

	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_VS(VGA_VS),
	.VGA_HS(VGA_HS),
	.VGA_DE(VGA_DE)
);

endmodule
