//============================================================================
//  Atari 7800 top-level for MiST
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module guest_top
(
	input         CLOCK_27,
`ifdef USE_CLOCK_50
	input         CLOCK_50,
`endif

	output        LED,
	output [VGA_BITS-1:0] VGA_R,
	output [VGA_BITS-1:0] VGA_G,
	output [VGA_BITS-1:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,

`ifdef USE_HDMI
	output        HDMI_RST,
	output  [7:0] HDMI_R,
	output  [7:0] HDMI_G,
	output  [7:0] HDMI_B,
	output        HDMI_HS,
	output        HDMI_VS,
	output        HDMI_PCLK,
	output        HDMI_DE,
	inout         HDMI_SDA,
	inout         HDMI_SCL,
	input         HDMI_INT,
`endif

	input         SPI_SCK,
	inout         SPI_DO,
	input         SPI_DI,
	input         SPI_SS2,    // data_io
	input         SPI_SS3,    // OSD
	input         CONF_DATA0, // SPI_SS for user_io

`ifdef USE_QSPI
	input         QSCK,
	input         QCSn,
	inout   [3:0] QDAT,
`endif
`ifndef NO_DIRECT_UPLOAD
	input         SPI_SS4,
`endif

	output [12:0] SDRAM_A,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nWE,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nCS,
	output  [1:0] SDRAM_BA,
	output        SDRAM_CLK,
	output        SDRAM_CKE,

`ifdef DUAL_SDRAM
	output [12:0] SDRAM2_A,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_DQML,
	output        SDRAM2_DQMH,
	output        SDRAM2_nWE,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nCS,
	output  [1:0] SDRAM2_BA,
	output        SDRAM2_CLK,
	output        SDRAM2_CKE,
`endif

	output        AUDIO_L,
	output        AUDIO_R,
`ifdef I2S_AUDIO
	output        I2S_BCK,
	output        I2S_LRCK,
	output        I2S_DATA,
`endif
`ifdef I2S_AUDIO_HDMI
	output        HDMI_MCLK,
	output        HDMI_BCK,
	output        HDMI_LRCK,
	output        HDMI_SDATA,
`endif
`ifdef PIN_REFLECTION
	output        joy_clk,
   input         joy_xclk,
	
   output        joy_load,
   input         joy_xload,
   
	input         joy_data,
   output        joy_xdata,	
`endif
`ifdef SPDIF_AUDIO
	output        SPDIF,
`endif
`ifdef USE_AUDIO_IN
	input         AUDIO_IN,
`endif
	input         UART_RX,
	output        UART_TX

);

`ifdef NO_DIRECT_UPLOAD
localparam bit DIRECT_UPLOAD = 0;
wire SPI_SS4 = 1;
`else
localparam bit DIRECT_UPLOAD = 1;
`endif

`ifdef USE_QSPI
localparam bit QSPI = 1;
assign QDAT = 4'hZ;
`else
localparam bit QSPI = 0;
`endif

`ifdef VGA_8BIT
localparam VGA_BITS = 8;
`else
localparam VGA_BITS = 6;
`endif

`ifdef USE_HDMI
localparam bit HDMI = 1;
assign HDMI_RST = 1'b1;
`else
localparam bit HDMI = 0;
`endif

`ifdef BIG_OSD
localparam bit BIG_OSD = 1;
`define SEP "-;",
`else
localparam bit BIG_OSD = 0;
`define SEP
`endif

// remove this if the 2nd chip is actually used
`ifdef DUAL_SDRAM
assign SDRAM2_A = 13'hZZZZ;
assign SDRAM2_BA = 0;
assign SDRAM2_DQML = 1;
assign SDRAM2_DQMH = 1;
assign SDRAM2_CKE = 0;
assign SDRAM2_CLK = 0;
assign SDRAM2_nCS = 1;
assign SDRAM2_DQ = 16'hZZZZ;
assign SDRAM2_nCAS = 1;
assign SDRAM2_nRAS = 1;
assign SDRAM2_nWE = 1;
`endif

`include "build_id.v"

assign LED  = ~ioctl_download & ~bk_ena;

parameter CONF_STR = {
	"A7800;;",
	"F1,A78A26BIN;",
	"F2,ROMBIN,Load BIOS;",
	"F3,PAL,Load Palette;",
	`SEP
	"P1,Video & Audio;",
	"P2,Controller;",
	"P3,System;",
	"P1O12,Scandoubler Fx,None,CRT 25%,CRT 50%,CRT 75%;",
	"P1O3,Composite blend,Off,On;",
	"P1OAB,Temperature Colors,Warm,Cool,Hot,Custom;",
	"P1OKL,Region,Auto,NTSC,PAL;",
	"P1OM,Stabilize Video,Off,On;",
	"P1O6,Show border,Yes,No;",
	"P1O7,Show overscan,No,Yes;",
	"P1OE,De-comb,Off,On;",
	"P1OR,Black&White,Off,On;",
	"P1ON,Stereo TIA,No,Yes;",
	"P2OTU,Controller1,Auto,Joystick,Paddle,SaveKey;",
	"P2OXY,Controller2,Auto,Joystick,Paddle,SaveKey;",
	"P2O4,Swap Joysticks,No,Yes;",
	"P2OS,Swap Paddle A<->B,No,Yes;",
	"P2O8,Difficulty Right,A,B;",
	"P2O9,Difficulty Left,A,B;",
	"P3O5,Bypass BIOS,Yes,No;",
	"P3OOP,High Score Cart,Auto,On,Off;",
	"P3OQ,Clear Memory,Zero,Random;",
	"P3OC,CPU Driver,TIA,Maria;",
	"P3OD,Pokey IRQ Enabled,No,Yes;",
	"P3OFJ,Bankswitching,Auto,F8,F6,FE,E0,3F,F4,P2,FA,CV,2K,UA,E7,F0,32,AR,3E,SB,WD,EF;",
	`SEP
`ifdef USE_SAVERAM
	"S0U,SAV,Load;",
	"TZ,Write SaveKey/HSCart;",
`endif
	"T0,Reset;",
	"V,v1.0.",`BUILD_DATE
};

wire [1:0] scanlines = status[2:1];
wire       blend = status[3];
wire       joyswap = status[4];
wire       bypass_bios = ~status[5];
wire       show_border = ~status[6];
wire       show_overscan = status[7];
wire       diff_right = ~status[8];
wire       diff_left = ~status[9];
wire [1:0] pal_temp = status[11:10];
wire       cpu_driver = ~status[12];
wire       pokey_irq = status[13];
wire       decomb = status[14];
wire [4:0] mapper = status[19:15];
wire [1:0] region = status[21:20];
wire       tia_stab = status[22];
wire       stereo_tia = status[23];
wire [1:0] hscart = status[25:24];
wire       clearmem = status[26];
wire       bwmode = status[27];
wire       paddleswap = status[28];
wire [1:0] controller1 = status[30:29];
wire [1:0] controller2 = status[34:33];
wire       use_tape = 0;

wire       bk_save = status[35];

////////////////////   CLOCKS   ///////////////////

wire pll_locked;
wire clk_sys, clk_vid, clk_tia;
assign SDRAM_CLK = clk_vid;

pll pll
(
`ifdef USE_CLOCK_50
	.inclk0(CLOCK_50),
`else
	.inclk0(CLOCK_27),
`endif
	.c0(clk_vid),
	.c1(clk_sys),
	.c2(clk_tia),
	.locked(pll_locked)
);


`ifdef PIN_REFLECTION
assign joy_clk = joy_xclk;
assign joy_load = joy_xload;
assign joy_xdata = joy_data;
`endif

reg reset;
always @(posedge clk_sys) begin
	reset <= buttons[1] | status[0] | ioctl_download;
end

//////////////////   MiST I/O   ///////////////////
wire [31:0] joy_0;
wire [31:0] joy_1;
wire [31:0] joy_2;
wire [31:0] joy_3;
wire [31:0] joy_4;

wire [31:0] joystick_analog_0;
wire [31:0] joystick_analog_1;

wire  [1:0] buttons;
wire [63:0] status;
wire        ypbpr;
wire        scandoubler_disable;
wire        no_csync;

wire  [8:0] mouse_x;
wire  [8:0] mouse_y;
wire  [7:0] mouse_flags;  // YOvfl, XOvfl, dy8, dx8, 1, mbtn, rbtn, lbtn
wire        mouse_strobe;

wire        key_strobe;
wire        key_pressed;
wire        key_extended;
wire  [7:0] key_code;

reg  [31:0] sd_lba;
reg         sd_rd = 0;
reg         sd_wr = 0;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;
wire        sd_buff_rd;
wire  [1:0] img_mounted;
wire [31:0] img_size;

`ifdef USE_HDMI
wire        i2c_start;
wire        i2c_read;
wire  [6:0] i2c_addr;
wire  [7:0] i2c_subaddr;
wire  [7:0] i2c_dout;
wire  [7:0] i2c_din;
wire        i2c_ack;
wire        i2c_end;
`endif

user_io #(.STRLEN($size(CONF_STR)>>3), .FEATURES(32'h0 | (BIG_OSD << 13) | (HDMI << 14))) user_io
(
	.clk_sys(clk_sys),
	.clk_sd(clk_sys),
	.SPI_SS_IO(CONF_DATA0),
	.SPI_CLK(SPI_SCK),
	.SPI_MOSI(SPI_DI),
	.SPI_MISO(SPI_DO),

	.conf_str(CONF_STR),

	.status(status),
	.scandoubler_disable(scandoubler_disable),
	.ypbpr(ypbpr),
	.no_csync(no_csync),
	.buttons(buttons),
	.joystick_0(joy_0),
	.joystick_1(joy_1),
	.joystick_2(joy_2),
	.joystick_3(joy_3),
	.joystick_4(joy_4),

	.joystick_analog_0(joystick_analog_0),
	.joystick_analog_1(joystick_analog_1),

	.mouse_x(mouse_x),
	.mouse_y(mouse_y),
	.mouse_flags(mouse_flags),
	.mouse_strobe(mouse_strobe),

	.key_strobe(key_strobe),
	.key_code(key_code),
	.key_pressed(key_pressed),
	.key_extended(key_extended),

`ifdef USE_HDMI
	.i2c_start      (i2c_start      ),
	.i2c_read       (i2c_read       ),
	.i2c_addr       (i2c_addr       ),
	.i2c_subaddr    (i2c_subaddr    ),
	.i2c_dout       (i2c_dout       ),
	.i2c_din        (i2c_din        ),
	.i2c_ack        (i2c_ack        ),
	.i2c_end        (i2c_end        ),
`endif

	.sd_conf(1'b0),
	.sd_sdhc(1'b1),
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_dout(sd_buff_dout),
	.sd_din(sd_buff_din),
	.sd_dout_strobe(sd_buff_wr),
	.sd_din_strobe(sd_buff_rd),
	.img_mounted(img_mounted),
	.img_size(img_size)
);

wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;

data_io data_io
(
	.clk_sys(clk_sys),
	.SPI_SCK(SPI_SCK),
	.SPI_DI(SPI_DI),
	.SPI_DO(SPI_DO),
	.SPI_SS2(SPI_SS2),
	.SPI_SS4(SPI_SS4),

	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index)
);

wire cart_download = ioctl_download & (ioctl_index[5:0] == 6'd1);
wire bios_download = ioctl_download & ((ioctl_index[5:0] == 6'd0) && (ioctl_index[7:6] == 0)) || (ioctl_index[5:0] == 2);
wire pal_download  = ioctl_download & (ioctl_index[5:0] == 6'd3);

reg old_cart_download;

////////////////////////////  SYSTEM  ///////////////////////////////////
logic tia_en;
logic [3:0] idump;

logic [1:0] ilatch;
logic [7:0] PAin, PBin, PAout, PBout;
wire [7:0] R,G,B;
wire HSync;
wire VSync;
wire HBlank;
wire VBlank, VBlank_orig;

wire [15:0] laudio, raudio;

wire [15:0] bios_addr;
reg [7:0] cart_data, bios_data;
reg [7:0] joy0_type, joy1_type, cart_region, cart_save;

logic [15:0] cart_flags;
logic [39:0] cart_header;
logic [31:0] cart_size;
logic [24:0] cart_addr;
logic [7:0] cart_xm;
logic cart_read;
logic [7:0] cart_data_sd;
reg cart_loaded = 0;
logic RW;

logic cart_is_7800;
logic [7:0] hsc_ram_dout, din;
logic hsc_ram_cs;
logic tia_mode;
logic ce_pix_raw;
logic [7:0] cart_din;
logic cart_rw;
wire [4:0] force_bs;
wire sc;
wire bios_sel;

logic [15:0] rnd;
wire PAread;

lfsr #(.N(15)) random(rnd);

wire region_select = ~|region ? (tia_en ? tia_pal : cart_region[0]) : region[1];
logic [3:0] iout;
logic tia_hsync;
logic tia_pal, tia_f1;
logic [7:0] rval;

always @(posedge clk_sys) begin
	rval <= rnd[7:0];
	old_cart_download <= cart_download;
end

`ifdef USE_AUDIO_IN
wire tape_pin = AUDIO_IN;
`else
wire tape_pin = UART_RX;
`endif

wire tape_in;

always @(posedge clk_sys) begin
	reg tapeD1, tapeD2;
	tapeD1 <= tape_pin;
	tapeD2 <= tapeD1;
	tape_in <= tapeD2;
end

wire [17:0] cartram_addr;
wire cartram_wr;
wire [7:0] cartram_wrdata;
wire cartram_rd;
wire [7:0] cartram_data;
wire cartram_busy;
logic [3:0] i_read;
logic halted;
logic use_sk;

logic core_paused = 0;
logic freeze_sync;
logic cpu_ce;
/*
always_ff @(posedge clk_sys) begin :core_sync
	reg old_sync;
	old_sync <= freeze_sync;
	if (old_sync ^ freeze_sync)
		core_paused <= (status[35] && OSD_STATUS) || halted;
end
*/

reg key_strobe_level;
always @(posedge clk_sys) if (key_strobe) key_strobe_level <= ~key_strobe_level;
wire [10:0] ps2_key = {key_strobe_level, key_pressed, key_extended, key_code};

Atari7800 main
(
	.clk_sys      (clk_sys),
	.reset        (reset),
	.loading      (cart_download || bios_download),
	.pause        (core_paused),

	// Video
	.RED          (R),
	.GREEN        (G),
	.BLUE         (B),
	.HSync        (HSync),
	.VSync        (VSync),
	.HBlank       (HBlank),
	.VBlank       (VBlank),
	.VBlank_orig  (VBlank_orig),
	.ce_pix       (ce_pix_raw),
	.show_border  (show_border),
	.show_overscan(show_overscan),
	.PAL          (region_select),
	.pal_temp     (pal_temp),
	.tia_mode     (tia_mode && bypass_bios),
	.bypass_bios  (bypass_bios),
	.pokey_irq    (pokey_irq),
	.hsc_en       (~use_sk && (~|hscart && (|cart_save || cart_xm[0]) ? 1'b1 : hscart[0])),
	.hsc_ram_dout (hsc_ram_dout),
	.hsc_ram_cs   (hsc_ram_cs),

	// Audio
	.AUDIO_R      (raudio), // 16 bit
	.AUDIO_L      (laudio), // 16 bit

	// Cart Interface
	.cart_out     (cart_download ? ioctl_dout[7:0] : (cart_loaded ? cart_data_sd : cart_data)),
	.cart_read    (cart_read),
	.cart_size    (cart_size),
	.cart_addr_out(cart_addr),
	.cart_flags   (cart_is_7800 ? cart_flags[15:0] : 16'd0),
	.cart_save    (cart_save),
	.cart_din     (cart_din),
	.cart_xm      (cart_is_7800 ? cart_xm : 8'h0),
	.ps2_key      (ps2_key),

	.cartram_addr   (cartram_addr),
	.cartram_wr     (cartram_wr),
	.cartram_rd     (cartram_rd),
	.cartram_wrdata (cartram_wrdata),
	.cartram_data   (cartram_data),

	// BIOS
	.bios_sel     (bios_sel),
	.bios_out     (bios_data),
	.AB           (bios_addr), // Address
	.RW           (RW), // inverted write
	.dout         (din),

	// Tia
	.idump        (idump),  // Paddle {A0, B0, A1, B1}
	.ilatch       (ilatch), // Buttons {FireB, FireA}
	.i_out        (iout),
	.tia_en       (tia_en),
	.tia_hsync    (tia_hsync),
	.use_stereo   (stereo_tia),
	.cpu_driver   (cpu_driver),
	.tia_f1       (tia_f1),
	.tia_pal      (tia_pal),
	.tia_stab     (tia_stab),

	// RIOT
	.PAin         (PAin),  // Direction {RA, LA, DA, UA, RB, LB, DB, UB}
	.PBin         (PBin),  // Port B input
	.PAout        (PAout), // Port A output
	.PBout        (PBout),  // Peanut butter
	.PAread       (PAread),

	// 2600 Cart Flags from detect2600
	.force_bs     (use_tape ? BANKAR : force_bs),
	.sc           (sc),
	.clearval     (clearmem ? rval : 8'h00),
	.random       (rval),
	.decomb       (decomb),
	.mapper       (mapper),
	.tape_in      ({use_tape, tape_in}),

	// Palette loading
	.pal_load     (pal_download),
	.pal_addr     (ioctl_addr[9:0]),
	.pal_wr       (ioctl_wr),
	.pal_data     (ioctl_dout[7:0]),
	.blend        (blend),
	.i_read       (i_read)
);

////////////////////////////  MEMORY  ///////////////////////////////////
logic [14:0] bios_mask;

detect2600 detect2600
(
	.clk        (clk_sys),
	.addr       (ioctl_addr[15:0]),
	.reset      (~old_cart_download && cart_download),
	.cart_size  (cart_size),
	.enable     (ioctl_wr & cart_download),
	.data       (ioctl_dout),
	.force_bs   (force_bs),
	.sc         (sc)
);

initial begin
	cart_header = "ATARI";
	cart_size = 32'h00008000;
	cart_flags = 0;
	cart_region = 0;
	bios_mask = 0;
	cart_save = 0;
	cart_xm = 0;
	tia_mode = 0;
end

always_ff @(posedge clk_sys) begin
	cart_is_7800 <= (cart_header == "ATARI");
	if (bios_download && ioctl_wr) // This assumes bootrom is always power of two
		bios_mask <= ioctl_addr[14:0];
	if (cart_download && ioctl_wr)
		cart_size <= (ioctl_addr - (cart_is_7800 ? 8'd128 : 1'b0)) + 1'd1; // 32 bit 1
	if (cart_download) begin
		tia_mode <= ioctl_index[7:6] != 0;
		cart_loaded <= 1;
		if (!tia_mode) begin
			case (ioctl_addr)
				'd01: cart_header[39:32] <= ioctl_dout;
				'd02: cart_header[31:24] <= ioctl_dout;
				'd03: cart_header[23:16] <= ioctl_dout;
				'd04: cart_header[15:8] <= ioctl_dout;
				'd05: cart_header[7:0] <= ioctl_dout;
				// 'd49: hcart_size[31:24] <= ioctl_dout;
				// 'd50: hcart_size[23:16] <= ioctl_dout;
				// 'd51: hcart_size[15:8] <= ioctl_dout;
				// 'd52: hcart_size[7:0] <= ioctl_dout;
				'd53: cart_flags[15:8] <= ioctl_dout;
				'd54: cart_flags[7:0] <= ioctl_dout;
				'd55: joy0_type <= ioctl_dout;   // 0=none, 1=joystick, 2=lightgun
				'd56: joy1_type <= ioctl_dout;
				'd57: cart_region <= ioctl_dout; // 0=ntsc, 1=pal
				'd58: cart_save <= ioctl_dout;   // 0=none, 1=high score cart, 2=savekey
				'd63: cart_xm <= ioctl_dout; // 1 = Has XM
			endcase
			if (!cart_is_7800) begin
				joy0_type <= 8'd1;
				joy1_type <= 8'd1;
			end
		end else begin
			joy0_type <= 8'd1;
			joy1_type <= 8'd1;
			cart_header <= '0;
			cart_flags <= 0;
			cart_region <= 0;
			cart_save <= 0;
			cart_xm <= 0;
		end
	end
end

logic [24:0] cart_write_addr, fixed_addr;
assign cart_write_addr = (ioctl_addr >= 8'd128) && cart_is_7800 ? (ioctl_addr[24:0] - 8'd128) : ioctl_addr[24:0];

spram #(
	.addr_width(14),
	.mem_name("Cart"),
	.mem_init_file("mem0.mif")
) cart
(
	.address (cart_addr),
	.clock   (clk_sys),
	.data    (),
	.wren    (),
	.q       (cart_data)
);

wire [24:0] sdram_addr = cart_download ? cart_write_addr :
                         bios_download ? {8'b10000001, ioctl_addr[14:0]} :
                         (cartram_rd || cartram_wr) ? {5'b10000, cartram_addr} :
                         bios_sel ? {8'b10000001, (bios_addr[14:0] & bios_mask)} :
												 {1'b0, cart_addr[21:0]};
wire        sdram_rd = (cart_read & ~cart_download & ~bios_download & ~reset) | cartram_rd;
wire        sdram_wr = (cart_download | bios_download) ? ioctl_wr : cartram_wr;
wire  [7:0] sdram_din = (cart_download | bios_download) ? ioctl_dout : cartram_wrdata;
assign      cartram_data = cart_data_sd;
assign      bios_data = cart_data_sd;

wire cart_busy;

sdram sdram
(
	.*,
	.SDRAM_CLK(),

	// system interface
	.clk        ( clk_vid       ),
	.init       ( !pll_locked   ),

	// cpu/chipset interface
	.ch0_addr   (sdram_addr),
	.ch0_wr     (sdram_wr),
	.ch0_din    (sdram_din),
	.ch0_rd     (sdram_rd),
	.ch0_dout   (cart_data_sd),
	.ch0_busy   (cart_busy)
);

//////////////////   VIDEO   //////////////////

mist_video #(
	.SD_HCNT_WIDTH(10),
	.COLOR_DEPTH(8),
	.USE_BLANKS(1'b1),
	.OUT_COLOR_DEPTH(VGA_BITS),
	.BIG_OSD(BIG_OSD)
	) 
mist_video
(
	.clk_sys(clk_vid),
	.scanlines(scanlines),
	.scandoubler_disable(scandoubler_disable),
	.ypbpr(ypbpr),
	.no_csync(no_csync),
	.rotate(2'b00),
	.blend(blend),
	.ce_divider(3'd0),
	.SPI_DI(SPI_DI),
	.SPI_SCK(SPI_SCK),
	.SPI_SS3(SPI_SS3),
	.HSync(HSync),
	.VSync(VSync),
	.HBlank(HBlank),
	.VBlank(VBlank),
	.R(R),
	.G(G),
	.B(B),
	.VGA_HS(VGA_HS),
	.VGA_VS(VGA_VS),
	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B)
);

////////////////////////////  HDMI  ///////////////////////////////////

`ifdef USE_HDMI
i2c_master #(15_000_000) i2c_master (
	.CLK         (clk_sys),
	.I2C_START   (i2c_start),
	.I2C_READ    (i2c_read),
	.I2C_ADDR    (i2c_addr),
	.I2C_SUBADDR (i2c_subaddr),
	.I2C_WDATA   (i2c_dout),
	.I2C_RDATA   (i2c_din),
	.I2C_END     (i2c_end),
	.I2C_ACK     (i2c_ack),

	//I2C bus
	.I2C_SCL     (HDMI_SCL),
	.I2C_SDA     (HDMI_SDA)
);

mist_video #(
	.SD_HCNT_WIDTH(10),
	.COLOR_DEPTH(8),
	.USE_BLANKS(1'b1),
	.OUT_COLOR_DEPTH(8),
	.BIG_OSD(BIG_OSD),
	.VIDEO_CLEANER(1'b1)
	) 
hdmi_video
(
	.clk_sys(clk_vid),
	.scanlines(scanlines),
	.scandoubler_disable(1'b0),
	.ypbpr(1'b0),
	.no_csync(1'b1),
	.rotate(2'b00),
	.blend(blend),
	.ce_divider(3'd0),
	.SPI_DI(SPI_DI),
	.SPI_SCK(SPI_SCK),
	.SPI_SS3(SPI_SS3),
	.HSync(HSync),
	.VSync(VSync),
	.HBlank(HBlank),
	.VBlank(VBlank),
	.R(R),
	.G(G),
	.B(B),
	.VGA_HS(HDMI_HS),
	.VGA_VS(HDMI_VS),
	.VGA_R(HDMI_R),
	.VGA_G(HDMI_G),
	.VGA_B(HDMI_B),
	.VGA_DE(HDMI_DE)
);

assign HDMI_PCLK = clk_vid;
`endif

//////////////////   AUDIO   //////////////////

hybrid_pwm_sd_2ndorder dac
(
	.clk(clk_vid),
	.reset_n(1'b1),
	.d_l(laudio),
	.q_l(AUDIO_L),
	.d_r(raudio),
	.q_r(AUDIO_R)
);

`ifdef I2S_AUDIO
i2s i2s
(
	.reset(1'b0),
	.clk(clk_vid),
	.clk_rate(32'd57_272728),

	.sclk(I2S_BCK),
	.lrclk(I2S_LRCK),
	.sdata(I2S_DATA),

	.left_chan({~laudio[15], laudio[14:0]}),
	.right_chan({~raudio[15], raudio[14:0]})
);
`ifdef I2S_AUDIO_HDMI
assign HDMI_MCLK = 0;
always @(posedge clk_vid) begin
	HDMI_BCK <= I2S_BCK;
	HDMI_LRCK <= I2S_LRCK;
	HDMI_SDATA <= I2S_DATA;
end
`endif
`endif

`ifdef SPDIF_AUDIO
spdif spdif
(
	.clk_i(clk_vid),
	.rst_i(1'b0),
	.clk_rate_i(32'd57_272728),
	.spdif_o(SPDIF),
	.sample_i({~raudio[15], raudio[14:0], ~laudio[15], laudio[14:0]})
);
`endif

////////////////////////////  INPUT  ///////////////////////////////////
wire m_up, m_down, m_left, m_right, m_fireA, m_fireB, m_fireC, m_fireD, m_fireE, m_fireF;
wire m_up2, m_down2, m_left2, m_right2, m_fire2A, m_fire2B, m_fire2C, m_fire2D, m_fire2E, m_fire2F;
wire m_tilt, m_coin1, m_coin2, m_coin3, m_coin4, m_one_player, m_two_players, m_three_players, m_four_players;

arcade_inputs #(.START1(7)) inputs (
	.clk         ( clk_sys     ),
	.key_strobe  ( key_strobe  ),
	.key_pressed ( key_pressed ),
	.key_code    ( key_code    ),
	.joystick_0  ( joy_0       ),
	.joystick_1  ( joy_1       ),
	.rotate      ( 2'b00       ),
	.orientation ( 2'b00       ),
	.joyswap     ( joyswap     ),
	.oneplayer   ( 1'b0        ),
	.controls    ( {m_tilt, m_coin4, m_coin3, m_coin2, m_coin1, m_four_players, m_three_players, m_two_players, m_one_player} ),
	.player1     ( {m_fireF, m_fireE, m_fireD, m_fireC, m_fireB, m_fireA, m_up, m_down, m_left, m_right} ),
	.player2     ( {m_fire2F, m_fire2E, m_fire2D, m_fire2C, m_fire2B, m_fire2A, m_up2, m_down2, m_left2, m_right2} )
);

logic [31:0] difference0, difference1, difference2, difference3;
wire [3:0] pread_mux1 = paddleswap ? {i_read[2], i_read[3], i_read[0], i_read[1]} : i_read;
wire [3:0] pread_mux2 = joyswap ? {pread_mux1[1:0], pread_mux1[3:2]} : pread_mux1;
wire [3:0] pad_muxa, pad_muxb;
logic [3:0] pad_wire;
logic [3:0] pad_b;

function [7:0] to_unsigned;
	input [7:0] val;
	begin
		to_unsigned = {~val[7], val[6:0]};
	end
endfunction

paddle_timer pt0 (clk_sys, 1, reset, {1'b0, to_unsigned(joystick_analog_0[ 7:0])}, ~iout[1], pread_mux2[0], pad_wire[0], difference0);
paddle_timer pt1 (clk_sys, 1, reset, {1'b0, to_unsigned(joystick_analog_0[15:8])}, ~iout[1], pread_mux2[1], pad_wire[1], difference1);
paddle_timer pt2 (clk_sys, 1, reset, {1'b0, to_unsigned(joystick_analog_1[ 7:0])}, ~iout[1], pread_mux2[2], pad_wire[2], difference2);
paddle_timer pt3 (clk_sys, 1, reset, {1'b0, to_unsigned(joystick_analog_1[15:8])}, ~iout[1], pread_mux2[3], pad_wire[3], difference3);

assign pad_muxa = paddleswap ? {~pad_b[0], ~pad_b[1], pad_wire[1:0]} : {~pad_b[1:0], pad_wire[0], pad_wire[1]};
assign pad_muxb = paddleswap ? {~pad_b[2], ~pad_b[3], pad_wire[3:2]} : {~pad_b[3:2], pad_wire[2], pad_wire[3]};
assign pad_b = {m_fire2B, m_fire2A, m_fireB, m_fireA};

wire joya_b2 = ~PBout[2] && ~tia_en && joy0_type != 5;
wire joyb_b2 = ~PBout[4] && ~tia_en && joy1_type != 5;

reg [7:0] porta_type, portb_type;
//  0 = none
//  1 = 7800 joystick
//  2 = lightgun
//  3 = paddle
//  4 = trakball
//  5 = 2600 joystick
//  6 = 2600 driving
//  7 = 2600 keypad
//  8 = ST mouse
//  9 = Amiga mouse
//  10 = AtariVox/SaveKey
//  11 = SNES2Atari
always @(*) begin
	porta_type = 0;
	if (controller1 == 0) begin
		// auto
		case (joy0_type)
			0: porta_type = 8'd0;
			1: porta_type = 8'd1;
			2: porta_type = 8'd2;
			3: porta_type = 8'd3;
			4: porta_type = 8'd4;
			5: porta_type = 8'd1;
			6: porta_type = 8'd6;
			7: porta_type = 8'd5;
			8: porta_type = 8'd8;
			9: porta_type = 8'd9;
			10: porta_type = 8'd10;
		default: porta_type = 8'd1;
	endcase
	end else begin
		case(controller1)
		2'd1: porta_type = 1; // joystick
		2'd2: porta_type = 3; // paddle
		2'd3: porta_type = 10; // SaveKey
		default:;
		endcase
	end

	portb_type = 0;
	if (controller2 == 0) begin
		// auto
		case (joy0_type)
			0: portb_type = 8'd0;
			1: portb_type = 8'd1;
			2: portb_type = 8'd2;
			3: portb_type = 8'd3;
			4: portb_type = 8'd4;
			5: portb_type = 8'd1;
			6: portb_type = 8'd6;
			7: portb_type = 8'd5;
			8: portb_type = 8'd8;
			9: portb_type = 8'd9;
			10: portb_type = 8'd10;
		default: portb_type = 8'd1;
	endcase
	end else begin
		case(controller2)
		2'd1: portb_type = 1; // joystick
		2'd2: portb_type = 3; // paddle
		2'd3: portb_type = 10; // SaveKey
		default:;
		endcase
	end

end

assign PBin[7] = diff_right;               // Right diff
assign PBin[6] = diff_left;                // Left diff
assign PBin[5] = PBout[5];                 // Unused (Not connected)
assign PBin[4] = PBout[4];                 // Unused (used for 2 button sensing)
assign PBin[3] = tia_en ? ~bwmode : (~m_fireD & ~m_fire2D);    // Pause/B&W
assign PBin[2] = PBout[2];                 // Unused (used for 2 button sensing)
assign PBin[1] = (~m_fireC & ~m_fire2C);   // Select
assign PBin[0] = ~m_one_player;            // Start/Reset

assign PAin[7:4] = porta_type == 3 ? {pad_muxa[3:2], 2'b11} : porta_type == 10 ? {1'b1,ep_do,2'b11} : {~m_right,  ~m_left,  ~m_down,  ~m_up }; // P1: R L D U
assign PAin[3:0] = portb_type == 3 ? {pad_muxb[3:2], 2'b11} : portb_type == 10 ? {1'b1,ep_do,2'b11} : {~m_right2, ~m_left2, ~m_down2, ~m_up2}; // P2: R L D U
assign ilatch[0] = porta_type == 3 ? 1'b1 : tia_en ? ~m_fireA  : (joya_b2 | ~(m_fireA  || m_fireB )); // P1 Fire
assign ilatch[1] = portb_type == 3 ? 1'b1 : tia_en ? ~m_fire2A : (joyb_b2 | ~(m_fire2A || m_fire2B)); // P2 Fire
assign idump[1:0] = porta_type == 3 ? pad_muxa[1:0] : tia_en ? {~m_fireB,  1'd0} : {m_fireA,  m_fireB}; // P2 F1, P2 F2, P1 F1, P1 F2 or Analog
assign idump[3:2] = portb_type == 3 ? pad_muxb[1:0] : tia_en ? {~m_fire2B, 1'd0} : {m_fire2A, m_fire2B}; // P2 F1, P2 F2, P1 F1, P1 F2 or Analog

//////////////////////////// BACKUP RAM /////////////////////
logic [7:0] sk_data;
logic [14:0] sk_addr;
logic sk_read, sk_write;

logic [7:0] sk_ram_do;
logic [14:0] sk_ram_addr;

assign use_sk = porta_type == 10 || portb_type == 10;

// LEFT (pin 3) SDA
// RIGHT (pin 4) SCL
wire ep_do; // SDA

EEPROM_24LC0X
#(
	.ADDR_WIDTH (15),
	.PAGE_WIDTH (6)
) savekey (
	.clk            (clk_sys),
	.ce             (1),
	.reset          (reset || ~use_sk),
	.SCL            (portb_type == 10 ? PAout[3] : PAout[7]),
	.SDA_in         (portb_type == 10 ? PAout[2] : PAout[6]),
	.SDA_out        (ep_do),
	.E_id           (0),
	.WC_n           (0),
	.data_from_ram  (hsc_ram_dout),
	.data_to_ram    (sk_ram_do),
	.ram_addr       (sk_ram_addr),
	.ram_read       (sk_read),
	.ram_write      (sk_write),
	.ram_done       (1)
);

reg bk_ena = 0;

`ifdef USE_SAVERAM
dpram_dc #(.widthad_a(14)) hsc_ram
(
	.clock_a   (clk_sys),
	.address_a (use_sk ? sk_ram_addr : bios_addr),
	.data_a    (use_sk ? sk_ram_do : din),
	.wren_a    (use_sk ? sk_write : (~RW & hsc_ram_cs)),
	.q_a       (hsc_ram_dout),

	.clock_b   (clk_sys),
	.address_b ({sd_lba[5:0],sd_buff_addr}),
	.data_b    (sd_buff_dout),
	.wren_b    (sd_buff_wr & sd_ack),
	.q_b       (sd_buff_din)
);


always @(posedge clk_sys) begin
	reg  bk_load = 0, old_load = 0, old_save = 0, old_ack, old_mounted = 0;
	reg  bk_state = 0;

	old_mounted <= img_mounted[0];
	if(~old_mounted && img_mounted[0]) begin
		bk_ena <= |img_size;
		bk_load <= |img_size;
	end

	old_load <= bk_load;
	old_save <= bk_save;
	old_ack  <= sd_ack;

	if(~old_ack & sd_ack) {sd_rd, sd_wr} <= 0;

	if(!bk_state) begin
		if(bk_ena & ((~old_load & bk_load) | (~old_save & bk_save))) begin
			bk_state <= 1;
			sd_lba <= 0;
			sd_rd <=  bk_load;
			sd_wr <= ~bk_load;
		end
	end else begin
		if(old_ack & ~sd_ack) begin
			if(&sd_lba[5:0]) begin
				bk_load <= 0;
				bk_state <= 0;
			end else begin
				sd_lba <= sd_lba + 1'd1;
				sd_rd  <=  bk_load;
				sd_wr  <= ~bk_load;
			end
		end
	end
end
`endif

endmodule

module lfsr(
	output [N-1:0] rnd
);

parameter N = 63;

lcell lc0(~(rnd[N - 1] ^ rnd[N - 3] ^ rnd[N - 4] ^ rnd[N - 6] ^ rnd[N - 10]), rnd[0]);
generate
	genvar i;
	for (i = 0; i <= N - 2; i = i + 1) begin : lcn
		lcell lc(rnd[i], rnd[i + 1]);
	end
endgenerate

endmodule