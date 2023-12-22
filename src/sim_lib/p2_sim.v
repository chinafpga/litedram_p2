//
`ifndef DLY
`define DLY #1
`endif

`timescale 1ns/1ps
//----------------------------------------------------------------------------
//
// module: ADD1_A
//
// description:     1-bit full adder with invertable a input
//
//----------------------------------------------------------------------------

module ADD1_A (
        a,
        b,
        ci,
        co,
        s
        );
        
    input       a;
    input       b;
    input       ci;
    output      co;
    output      s;
    
    parameter a_inv = "false";   //a input invert, default is "false"

    wire a_in;
    assign a_in = (a_inv == "false") ?  a : ~a;
    assign p  = a_in ^ b;
    assign pb = ~p;
    assign co = p ? ci : a_in ;
    assign s  = ci ^ p;

endmodule

// 1-bit full adder
//------------------------------------
module ADD_1BIT (
	s,
	co,
	p,
	pb,
	
	ci,
	b,
	a
);

output     s;
output     co;
output     p;
output     pb;

input      ci;
input      b;
input      a;

`ifndef CS_SW_SKEL
    assign p  = a ^ b;
    assign pb = ~p;
    assign co = p ? ci : a ;
    assign s  = ci ^ p;

specify

   (ci => co)   = (0,0);
   (a  => co)   = (0,0);
   (b  => co)   = (0,0);

   (ci => s)    = (0,0);
   (a  => s)    = (0,0);
   (b  => s)    = (0,0);

   (a  => p)    = (0,0);
   (b  => p)    = (0,0);

   (a  => pb)   = (0,0);
   (b  => pb)   = (0,0);
            
endspecify
    
`endif



endmodule

module AND2 (O, I0, I1);
        input I0;
        input I1;
        output O;

    assign O = I1&I0;
endmodule

module AND4 (
	o,
	
	a,
	b,
	c,
	d
	);

output	o;

input a;
input b;
input c;
input d;

`ifndef CS_SW_SKEL
    assign    o = (a & b) & (c & d);
    
    specify

    ( a => o ) = (0,0);
    ( b => o ) = (0,0);
    ( c => o ) = (0,0);
    ( d => o ) = (0,0);

    endspecify
`endif

endmodule

// VPERL: GENERATED_END
//
// module: CARRY_SKIP_IN
//
// description: enable the carry input from below cell or not 
// config:      1 bit to select where carry-in from.
//              set 1'b1 when the le is the first stage of adder.
//
//----------------------------------------------------------------------------

module CARRY_SKIP_IN (
	cin,
	
	c0alt,
	c0ripple,
	cskip4,
	cskip8,
	ripple,
	p47,
	p07,
	p03
`ifdef FM_HACK
	,CIN_BELOW
	,ALLOW_SKIP
`endif
);

output	cin;

input	c0alt;
input	c0ripple;
input	cskip4;
input	cskip8;
input	ripple;
input	p47;
input	p07;
input	p03;


`ifdef CS_FORMALPRO_HACK
    wire CIN_BELOW; 
    wire ALLOW_SKIP; 
`else
     `ifdef  SIMULATION
         reg       CIN_BELOW  = 1'b0;
         reg       ALLOW_SKIP = 1'b0;
     `else
		`ifdef FM_HACK
			input	CIN_BELOW;
			input	ALLOW_SKIP;
		`else
			parameter CIN_BELOW  = 1'b0;
         	parameter ALLOW_SKIP = 1'b0;
		`endif
     `endif
`endif
wire    [2:0]    sel ;
wire        p4;

`ifndef CS_SW_SKEL
    assign p4 = !p47 && !p03;
    assign sel = {!p07,p4,!ripple};
    assign cin = ( ALLOW_SKIP == 1'b0 ) 
                ? ( ( CIN_BELOW == 1'b0 ) 
                    ? c0alt 
                    : c0ripple ) 
                : (  ( CIN_BELOW == 1'b0 ) 
                     ? c0alt 
                     : ( sel == 3'b001 )     
                       ? c0ripple  
                       : ( sel == 3'b010 )     
                         ? cskip4 
                         : ( sel == 3'b100 )     
                           ? cskip8 
                           : 1'bx
                  );

    specify
       
    ( c0alt    =>    cin ) = (0,0);                     
    ( c0ripple =>    cin ) = (0,0);                     
    ( cskip4   =>    cin ) = (0,0);                 
    ( cskip8   =>    cin ) = (0,0);       
    ( p07      =>    cin ) = (0,0);                     
    ( p47      =>    cin ) = (0,0);                    
    ( ripple   =>    cin ) = (0,0);                   
    ( p03       =>    cin ) = (0,0);      
            
    endspecify
    
`endif

endmodule
// module: CARRY_SKIP_OUT
//
// description: the carry output logic 
// config:      share 1 bit with CARRY_SKIP_IN to ignore the skip4/8 from below or not 
//              set to 1'b1 when the below carry-in is used.
//
//----------------------------------------------------------------------------

module CARRY_SKIP_OUT (
	r4outb,
	p4outb,
	p8outb,

	plower4,
	p0b,
	p1b,
	p2b,
	p3b
);    

output r4outb;
output p4outb;
output p8outb;

input plower4;
input p0b;
input p1b;
input p2b;
input p3b;

`ifndef CS_SW_SKEL
    wire  p01 ;
    wire  p23 ;

    assign p01 = ~(p0b|p1b);
    assign p23 = ~(p2b|p3b);
    
    assign r4outb =   p01&p23;
    assign p4outb = !(p01&p23);
    assign p8outb = !(p01&p23&plower4);

    specify
       
        ( p0b =>  r4outb ) = (0,0);                     
        ( p1b =>  r4outb ) = (0,0);                     
        ( p2b =>  r4outb ) = (0,0);                 
        ( p3b =>  r4outb ) = (0,0);
        
        
        ( p0b =>  p4outb ) = (0,0);                     
        ( p1b =>  p4outb ) = (0,0);                     
        ( p2b =>  p4outb ) = (0,0);                 
        ( p3b =>  p4outb ) = (0,0);
        
        ( p0b =>  p8outb ) = (0,0);                     
        ( p1b =>  p8outb ) = (0,0);                     
        ( p2b =>  p8outb ) = (0,0);                 
        ( p3b =>  p8outb ) = (0,0);
        ( plower4 =>  p8outb ) = (0,0);
            
    endspecify

`endif

endmodule
module clk_fb_compensate (
	fclk0_il,
	clkfbin
	`ifdef FM_HACK
	,CFG_FBCLK_EN
	,CFG_FBCLK_INV
	`endif
	);
output	fclk0_il;

input	clkfbin;

`ifdef FM_HACK
input CFG_FBCLK_EN;
input CFG_FBCLK_INV;
`else
parameter CFG_FBCLK_EN = 1'b0;
parameter CFG_FBCLK_INV = 1'b0;
`endif

wire	clk_inv;
xor u0(clk_inv,!clkfbin,CFG_FBCLK_INV);
reg		q_ck;
always @(CFG_FBCLK_EN or clk_inv)
	if(clk_inv == 0) q_ck <= CFG_FBCLK_EN;

wire	feckb;
nand u1(feckb,q_ck,clk_inv);
and u2(fclk0_il,!feckb,CFG_FBCLK_EN);

endmodule
//----------------------------------------------------------------------------
//
// module: CFG_DYN_SWITCH_S2
//
// description: 2 bit wide 2:1 dynamic switch
// config:      2'b00 = output 0 (const)
//				2'b01 = output i0
//				2'b10 = output i1
//				2'b11 = output dynamic switch (fpsel)
//
//----------------------------------------------------------------------------
module CFG_DYN_SWITCH_S2 (
    o,
    i0,
    i1,
    fpsel
);

input        i0;
input        i1;
input        fpsel;

output        o;

		parameter         SEL = 2'b00;
   		parameter         gclk_mux = 0; //0,1,2,3,4,5,6,7

//reg gbl_clear_b=1;
//wire GSR;

//initial begin
//    gbl_clear_b = 1'b0;
//    @(posedge GSR);
//    gbl_clear_b = 1'b1;
//end
wire gbl_clear_b=1;

wire ck0 = i0;
wire ck1 = i1;
wire en0b = ~SEL[0];
wire en0  =  SEL[0];
wire en1b = ~SEL[1];
wire en1  =  SEL[1];

reg sel0_reg, sel1_reg;
reg en_ck0, en_ck1;
reg cdone_gate;

wire sel_1p0n =  fpsel & ~en_ck0;
wire sel_0p1n = ~fpsel & ~en_ck1;

always@(posedge ck0 or negedge gbl_clear_b)
begin
    if(~gbl_clear_b)
        sel0_reg <= 1'b0;
    else
        sel0_reg <= sel_0p1n;
end

always@(negedge ck0 or negedge gbl_clear_b)
begin
    if(~gbl_clear_b)
        en_ck0 <= 1'b0;
    else
        en_ck0 <= sel0_reg;
end

always@(posedge ck1 or negedge gbl_clear_b)
begin
    if(~gbl_clear_b)
        sel1_reg <= 1'b0;
    else
        sel1_reg <= sel_1p0n;
end

always@(negedge ck1 or negedge gbl_clear_b)
begin
    if(~gbl_clear_b)
        en_ck1 <= 1'b0;
    else
        en_ck1 <= sel1_reg;
end

wire enck0 = en0 & (en1 ? en_ck0 : 1'b1);
wire enck1 = en1 & (en0 ? en_ck1 : 1'b1);

wire ck_sel = {enck1, enck0} == 2'b01 ? ck0  :
              {enck1, enck0} == 2'b10 ? ck1  :
              {enck1, enck0} == 2'b00 ? 1'b0 : 1'bx;

always@(negedge ck_sel or negedge gbl_clear_b)
begin
    if(~gbl_clear_b)
        cdone_gate <= 1'b0;
    else
        cdone_gate <= 1'b1;
end

assign o = cdone_gate & ck_sel;

endmodule


module DBUF_GATE (
    clk,
    en,
    clk_out
);

    input clk;
    input en;
    output clk_out;

    reg en_latch;
    always @(*) begin
        if (clk == 0)
            en_latch <= en;
    end

    assign clk_out = clk & en_latch;

endmodule

module DBUF(in, out);
    input in;
    output out;
    
    buf (out, in);
    
endmodule

module DELAY_BUF (
    in, 
    out
);

    input in ;
    output out ;
   
    buf (out, in);

endmodule

//----------------------------------------------------------------------------
//
// module: DIV
//
// description: Logic divider with asynchronous clear input
// config:      TBD
//
//----------------------------------------------------------------------------
module DIV (
    CKOUT,
    CKIN,
    CLR
`ifdef FM_HACK
    ,DIV
`endif
);

output CKOUT;
input  CKIN;
input  CLR;

`ifdef FM_HACK
    input [3:0]  DIV;
`else
    parameter DIV = 4'b0000;
`endif

//function description 
/* ------------
CLR: 1 rst;
DIV: 0000: bypass
     0001: div 1
     0010: div 2
     0011: div 3
     0100: div4
     0101: div5
     0110: div6
     0111: div7
     1000: div8
     1001: div9
     1010: div10
     1011: div11
     1100: div12
     1101: div13
     1110: div14
     1111: div15
------------ */
reg Q1, Q2, Q3, Q4;
wire Q1N, Q2N, Q3N, Q4N;
reg CKOUT1;
wire CKOUT1B = ~CKOUT1;

wire CKOUT = DIV[3:0]==0 ? CKIN : CKOUT1;

assign Q1N = ~Q1;
assign Q2N = ~Q2;
assign Q3N = ~Q3;
assign Q4N = ~Q4;

wire SET_DIV = (DIV[0] ^ Q1N) & (DIV[1] ^ Q2N) & (DIV[2] ^ Q3N) & (DIV[3] ^ Q4N);
wire net0342 = ~(DIV[0] & CKIN);
wire net0157 = (Q3N ^ DIV[3]) & (Q2N ^ DIV[2]) & (Q1N ^ DIV[1]) & (CKOUT1B ^ 1'b1);

wire SET_DUTY = net0342 & net0157;
wire SET_DIV_DLY = ~CLR & ~SET_DIV;

wire d1 = ~CLR ^ Q1;
wire d2 = (SET_DIV_DLY & Q1) ^ Q2;
wire d3 = (SET_DIV_DLY & Q1 & Q2) ^ Q3;
wire d4 = (SET_DIV_DLY & Q1 & Q2 & Q3) ^ Q4;

//cts_dffrs I119 ( .RSTN(SET_DIV_DLY), .SETN(vcc), .CK(CKIN), .D(d1), .QB(Q1N), .Q(Q1));
//cts_dffrs I136 ( .RSTN(SET_DIV_DLY), .SETN(vcc), .CK(CKIN), .D(d2), .QB(Q2N), .Q(Q2));
//cts_dffrs I137 ( .RSTN(SET_DIV_DLY), .SETN(vcc), .CK(CKIN), .D(d3), .QB(Q3N), .Q(Q3));
//cts_dffrs I138 ( .RSTN(SET_DIV_DLY), .SETN(vcc), .CK(CKIN), .D(d4), .QB(Q4N), .Q(Q4));
//
//cts_dffrs I116 ( .RSTN(~CLR), .SETN(~SET_DIV), .CK(SET_DUTY), .D(1'b0), .QB(CKOUT1B), .Q(CKOUT1));
//
//supply1_guts V ( .vcc	(vcc) );

always @ (posedge CKIN or negedge SET_DIV_DLY) begin
    if(~SET_DIV_DLY) begin
        Q1 <= 1'b0;
        Q2 <= 1'b0;
        Q3 <= 1'b0;
        Q4 <= 1'b0;
    end else begin
        Q1 <= d1;
        Q2 <= d2;
        Q3 <= d3;
        Q4 <= d4;
   end
end 

always @ (posedge SET_DUTY or posedge CLR or posedge SET_DIV) begin
    if(SET_DIV)
        CKOUT1 <= 1'b1;
    else if(CLR)
        CKOUT1 <= 1'b0;
    else
        CKOUT1 <= 1'b0;
end

endmodule

//----------------------------------------------------------------------------
//
// module: FG6X2
//
// description: 6 input lookup-table
//        outputs include one of two LUT5 results
// config:      64 bit init value for two LUT5 and 1 bit to select LUT6
//
//----------------------------------------------------------------------------
module FG6X2 (
    x,
    xy,
    f
`ifdef FM_HACK
    ,config_data
    ,mode
`endif
);

    input    [5:0]    f;

    output        x;
    output        xy;


`ifdef CS_FORMALPRO_HACK
    wire [63:0]     config_data;
    wire            mode;
`else
     `ifdef  SIMULATION
          reg [63:0] config_data = 64'h0000000000000000; // [31:0] for LUT5_0, [63:32] for LUT5_1
          reg        mode        = 1'b0;                 // select between LUT6 and LUT5_2
     `else
        `ifdef FM_HACK
            input    [63:0]    config_data;
            input            mode;
        `else
            parameter  [63:0] config_data = 64'h0000000000000000; // [31:0] for LUT5_0, [63:32] for LUT5_1
            parameter  mode        = 1'b0;                 // select between LUT6 and LUT5_2
        `endif
     `endif
`endif

`ifdef SIMULATION    
    reg [63:0] cfg_data;
    initial #1 cfg_data = ~config_data;
`else 
    `ifndef FM_HACK
        localparam [63:0] cfg_data = ~config_data;
    `else
        wire [63:0] cfg_data = ~config_data;
    `endif
`endif

    wire    x0, x1;
    wire        y;

`ifndef CS_SW_SKEL

    
LUT5    lut_0 (
    .f4    (f[4]),
    .f3    (f[3]),
    .f2    (f[2]),
    .f1    (f[1]),
    .f0    (f[0]),
    .dx    (x0)
`ifdef FM_HACK
    ,.config_data (cfg_data[31:0])
`endif
);
`ifndef CS_FORMALPRO_HACK 
     `ifdef  SIMULATION
//        initial  lut_0.config_data=config_data[31:0];
     `else
        `ifdef FM_HACK
        //BLANK param
        `else
            defparam lut_0.config_data=cfg_data[31:0];
        `endif
     `endif
`endif

LUT5    lut_1 (
    .f4    (f[4]),
    .f3    (f[3]),
    .f2    (f[2]),
    .f1    (f[1]),
    .f0    (f[0]),
    .dx    (x1)
`ifdef FM_HACK
    ,.config_data (cfg_data[63:32])
`endif
);
`ifndef CS_FORMALPRO_HACK 
     `ifdef  SIMULATION
//      initial  lut_1.config_data=config_data[63:32];
     `else
        `ifdef FM_HACK
        //BLANK param
        `else
            defparam lut_1.config_data=cfg_data[63:32];
        `endif
     `endif
`endif

assign    x = x1;

MUX2S_L mux_xy (
    .sel    (mode_sel),
    .i0    (x0),
    .i1    (x1),
    .o    (xy) 
);

`ifndef CS_FORMALPRO_HACK 
     `ifdef  SIMULATION
      initial  mux_mode_sel.SEL=mode;
     `else
        `ifdef FM_HACK
        //BLANK param
        `else
            defparam mux_mode_sel.SEL=mode;
        `endif
     `endif
`endif
CFGMUX2S1 mux_mode_sel (
    .i0    (f[5]),
    .i1    (1'b0),
    .o    (mode_sel) 
`ifdef FM_HACK
    ,.SEL (mode)
`endif
);

`endif

endmodule

module GBUF_GATE (
    clk,
    en,
    clk_out
);

    input clk;
    input en;
    output clk_out;
    
    reg en_latch;
    always @(*) begin
        if (clk == 0)
            en_latch <= en;
    end

    assign clk_out = clk & en_latch;

endmodule

module GBUF(in, out);
    input in;
    output out;
    
    buf (out, in);
    
endmodule

module gclk_clk_gate(
                 gate_en ,
                 clk_gate_in ,
                 test_mode ,
                 clk_gate_out
);
input            gate_en ;
input            clk_gate_in ;
input            test_mode ;

output           clk_gate_out;

reg q;
always @ (*) begin
    if (clk_gate_in==1'b0)
     q = test_mode||gate_en ;
end
assign clk_gate_out = clk_gate_in & q;

endmodule



module GND (Y);
    output Y;
    
    assign Y = 0;

endmodule


//----------------------------------------------------------------------------
//
// module: IBUF
//
// description:     input buffer
//
//----------------------------------------------------------------------------

module IBUF (p, o);
    input   p;
    output  o;

    buf b1 (o, p);

endmodule


//----------------------------------------------------------------------------
//
// module: IOBUF
//
// description:     Bi-directional buffer
//
//----------------------------------------------------------------------------

module IOBUF(
    i,
    o
);
input i;
output o;
assign o = i;
endmodule

module JTAG_DBWV1(
    // Outputs
	jtag_fp_drck     , 
	jtag_fp_reset ,
	jtag_fp_sel,
	jtag_fp_capture,
	jtag_fp_shift,
	jtag_fp_update,
	jtag_fp_tdi      ,
    // Inputs
//	jtag_fp_usermode,
	jtag_fp_tdo
);

    // Outputs
	output jtag_fp_drck     ; 
	output jtag_fp_reset ;
	output [1:0] jtag_fp_sel;
	output jtag_fp_capture;
	output jtag_fp_shift;
	output jtag_fp_update;
	output jtag_fp_tdi      ;
    // Inputs
//	input  jtag_fp_usermode;
	input  jtag_fp_tdo;
    
endmodule
module JTAG_FP ( tck,tms,tdo,tdi);
		 output tck;
		 output tms;
		 output tdi;
		 input  tdo;
endmodule
//----------------------------------------------------------------------------
//
// module: LBUF
//
// description: register control for 8 REGs in LE.
// config:      1 bit enable control
//              1 bit invert sense of "en" input
//              1 bit invert clk control
//              1 bit close clk control
//              1 bit REG or LATCH selection
//              1 bit SYNC or ASYNC set/reset selection
//              1 bit ASYNC set/reset assert even if SYNC is configured (deassert with be synchronized)
//              1 bit ALLOW_SR to enable set/reset (if =0, ignores "sr" input)
//              1 bit invert sense of "sr" input
//
// notes: RST and SET inputs are active high.
//
//----------------------------------------------------------------------------
module LBUF (
        asr,
        mclkb,
        sclk,

        clk,
        en,
        sr
`ifdef FM_HACK
		,CFG_EN       
		,CFG_INV      
		,CFG_HASCLK   
		,CFG_LAT      
		,CFG_SYNC     
		,CFG_ALLOW_SR 
		,CFG_INV_EN   
		,CFG_INV_SR   
`endif
);

output  asr;
output  mclkb;
output  sclk;

input   clk;
input   en;
input   sr;


`ifdef CS_FORMALPRO_HACK
   wire        CFG_EN      ;    // enable clock gating
   wire        CFG_INV     ;    // invert clock input (emulate negedge FFs, using posedge FFs)
   wire        CFG_HASCLK   ;    // allow clock
   wire        CFG_LAT     ;    // if =1, hold master clock "open"
   wire        CFG_SYNC    ;    // synchronize set/reset assert/deassert with active clock
   wire        CFG_ALLOW_SR ;    // allow set/reset; if =0, ignore "sr" input
   wire        CFG_INV_EN ;    // treat "en" input as active-low
   wire        CFG_INV_SR ;    // treat "sr" input as active-low
`else
     `ifdef  SIMULATION
          reg        CFG_EN       = 1'b0;
          reg        CFG_INV      = 1'b0;
          reg        CFG_HASCLK   = 1'b0;
          reg        CFG_LAT      = 1'b0;
          reg        CFG_SYNC     = 1'b0;
          reg        CFG_ALLOW_SR = 1'b0;
          reg        CFG_INV_EN   = 1'b0;
          reg        CFG_INV_SR   = 1'b0;
     `else
		`ifdef FM_HACK
			input  CFG_EN       ;
			input  CFG_INV      ;
			input  CFG_HASCLK   ;
			input  CFG_LAT      ;
			input  CFG_SYNC     ;
			input  CFG_ALLOW_SR ;
			input  CFG_INV_EN   ;
			input  CFG_INV_SR   ;
		`else
			parameter  CFG_EN       = 1'b0;
          	parameter  CFG_INV      = 1'b0;
          	parameter  CFG_HASCLK   = 1'b0;
          	parameter  CFG_LAT      = 1'b0;
          	parameter  CFG_SYNC     = 1'b0;
          	parameter  CFG_ALLOW_SR = 1'b0;
          	parameter  CFG_INV_EN   = 1'b0;
          	parameter  CFG_INV_SR   = 1'b0;
		`endif
     `endif
`endif

reg         en_latch_b;
wire        en_latch;
wire        clk1;
wire        clk2;
wire        sr_ff;

wire GSR;
glbsr inst( .GSR( GSR )) ;

initial begin
    en_latch_b = 1'b0;
    @(posedge GSR);
    en_latch_b = 1'b1;
end

`ifndef CS_SW_SKEL
    assign    clk1 = (CFG_HASCLK == 1'b1)  ?  clk : 1'b1;
    assign    clk2 = (CFG_INV   == 1'b1)  ? !clk1: clk1;

    wire    clkb = ~clk2;

    wire    sri = (CFG_INV_SR == 1'b1) ? ~sr : sr ;
    wire    eni = (CFG_INV_EN == 1'b1) ? ~en : en ;

    wire    allow_sr    = CFG_ALLOW_SR ;    // actually CFG_ALLOW_SR & gbl_cfg_done & gbl_clear_b
    wire    sr_allowed    = allow_sr? sri: 1'b1 ;
    wire    sr_allowed_i  = ~sr_allowed ;
   
    reg sr_latch;

    always @(*) begin
      if (allow_sr == 0) sr_latch <= sr_allowed_i ;	 
      else if (clkb == 1) sr_latch <= sr_allowed_i ;
    end
    
    wire    sync_sr = sr_latch & clk2;

    assign  asr = (CFG_SYNC == 1'b1) ? !sync_sr : sr_allowed ;

    assign  mclkb = (CFG_LAT == 1'b1)? 1'b0 :
                                       (clk2 & en_latch);

    always @(clkb or eni or CFG_EN) begin
       if (clkb == 1) en_latch_b <= !eni;
    end
    assign en_latch = !en_latch_b | ~CFG_EN;

    assign sclk = clk2 & en_latch;


    specify

        ( clk =>  sclk ) = (0,0);


    endspecify


`endif

endmodule

module MUX2S_L (
        o,
        sel,
        i0,
        i1
);

input        i0;
input        i1;
input        sel;

output        o;

`ifndef CS_SW_SKEL
	assign         o = (sel == 1'b0) ? i0 :
                    (sel == 1'b1) ? i1 :
                    (i0  == i1  ) ? i0 :1'bx;
    specify
        ( i0 =>  o ) = (0,0);
        ( i1 =>  o ) = (0,0);
        ( sel =>  o ) = (0,0);
    endspecify

`endif

endmodule


//----------------------------------------------------------------------------
//
// module: LUT5_2
//
// description: 5 input, 2 outputs lookup-table
// config_x:      32 bit init value
// config_y:      32 bit init value
//
//----------------------------------------------------------------------------

module LUT5_2 (
         dx,
         dy,
         f4,
         f3,
         f2,
         f1,
         f0
         );

    input       f4;
    input       f3;
    input       f2;
    input       f1;
    input       f0;

    output      dx;
    output      dy;

    parameter       config_data_x = 32'h0000_0000;
    parameter       config_data_y = 32'h0000_0000;

    wire [31:0]         cfg_x = config_data_x;
    wire [31:0]         cfg_xy = config_data_y;
      
    wire [15:0]         da_x;
    wire [7:0]          db_x;
    wire [3:0]          dc_x;
    wire [1:0]          dd_x;
    wire                de_x;

    wire [15:0]         da_xy;
    wire [7:0]          db_xy;
    wire [3:0]          dc_xy;
    wire [1:0]          dd_xy;
    wire                de_xy;
    //LUT5_X
    MUX2S_L muxa0_x(.o(da_x[0]),  .sel(f0), .i1(cfg_x[1]), .i0(cfg_x[0]) );
    MUX2S_L muxa1_x(.o(da_x[1]),  .sel(f0), .i1(cfg_x[3]), .i0(cfg_x[2]) );
    MUX2S_L muxa2_x(.o(da_x[2]),  .sel(f0), .i1(cfg_x[5]), .i0(cfg_x[4]) );
    MUX2S_L muxa3_x(.o(da_x[3]),  .sel(f0), .i1(cfg_x[7]), .i0(cfg_x[6]) );
    MUX2S_L muxa4_x(.o(da_x[4]),  .sel(f0), .i1(cfg_x[9]), .i0(cfg_x[8]) );
    MUX2S_L muxa5_x(.o(da_x[5]),  .sel(f0), .i1(cfg_x[11]), .i0(cfg_x[10]) );
    MUX2S_L muxa6_x(.o(da_x[6]),  .sel(f0), .i1(cfg_x[13]), .i0(cfg_x[12]) );
    MUX2S_L muxa7_x(.o(da_x[7]),  .sel(f0), .i1(cfg_x[15]), .i0(cfg_x[14]) );
    MUX2S_L muxa8_x(.o(da_x[8]),  .sel(f0), .i1(cfg_x[17]), .i0(cfg_x[16]) );
    MUX2S_L muxa9_x(.o(da_x[9]),  .sel(f0), .i1(cfg_x[19]), .i0(cfg_x[18]) );
    MUX2S_L muxa10_x(.o(da_x[10]), .sel(f0), .i1(cfg_x[21]), .i0(cfg_x[20]) );
    MUX2S_L muxa11_x(.o(da_x[11]), .sel(f0), .i1(cfg_x[23]), .i0(cfg_x[22]) );
    MUX2S_L muxa12_x(.o(da_x[12]), .sel(f0), .i1(cfg_x[25]), .i0(cfg_x[24]) );
    MUX2S_L muxa13_x(.o(da_x[13]), .sel(f0), .i1(cfg_x[27]), .i0(cfg_x[26]) );
    MUX2S_L muxa14_x(.o(da_x[14]), .sel(f0), .i1(cfg_x[29]), .i0(cfg_x[28]) );
    MUX2S_L muxa15_x(.o(da_x[15]), .sel(f0), .i1(cfg_x[31]), .i0(cfg_x[30]) );

    MUX2S_L muxb0_x(.o(db_x[0]), .sel(f1), .i1(da_x[1]), .i0(da_x[0]) );
    MUX2S_L muxb1_x(.o(db_x[1]), .sel(f1), .i1(da_x[3]), .i0(da_x[2]) );
    MUX2S_L muxb2_x(.o(db_x[2]), .sel(f1), .i1(da_x[5]), .i0(da_x[4]) );
    MUX2S_L muxb3_x(.o(db_x[3]), .sel(f1), .i1(da_x[7]), .i0(da_x[6]) );
    MUX2S_L muxb4_x(.o(db_x[4]), .sel(f1), .i1(da_x[9]), .i0(da_x[8]) );
    MUX2S_L muxb5_x(.o(db_x[5]), .sel(f1), .i1(da_x[11]), .i0(da_x[10]) );
    MUX2S_L muxb6_x(.o(db_x[6]), .sel(f1), .i1(da_x[13]), .i0(da_x[12]) );
    MUX2S_L muxb7_x(.o(db_x[7]), .sel(f1), .i1(da_x[15]), .i0(da_x[14]) );

    MUX2S_L muxc0_x(.o(dc_x[0]), .sel(f2), .i1(db_x[1]), .i0(db_x[0]) );
    MUX2S_L muxc1_x(.o(dc_x[1]), .sel(f2), .i1(db_x[3]), .i0(db_x[2]) );
    MUX2S_L muxc2_x(.o(dc_x[2]), .sel(f2), .i1(db_x[5]), .i0(db_x[4]) );
    MUX2S_L muxc3_x(.o(dc_x[3]), .sel(f2), .i1(db_x[7]), .i0(db_x[6]) );

    MUX2S_L muxd0_x(.o(dd_x[0]), .sel(f3), .i1(dc_x[1]), .i0(dc_x[0]) );
    MUX2S_L muxd1_x(.o(dd_x[1]), .sel(f3), .i1(dc_x[3]), .i0(dc_x[2]) );

    MUX2S_L muxe0_x(.o(de_x), .sel(f4), .i1(dd_x[1]), .i0(dd_x[0]) );

    //LUT5_XY
    MUX2S_L muxa0_xy(.o(da_xy[0]),  .sel(f0), .i1(cfg_xy[1]), .i0(cfg_xy[0]) );
    MUX2S_L muxa1_xy(.o(da_xy[1]),  .sel(f0), .i1(cfg_xy[3]), .i0(cfg_xy[2]) );
    MUX2S_L muxa2_xy(.o(da_xy[2]),  .sel(f0), .i1(cfg_xy[5]), .i0(cfg_xy[4]) );
    MUX2S_L muxa3_xy(.o(da_xy[3]),  .sel(f0), .i1(cfg_xy[7]), .i0(cfg_xy[6]) );
    MUX2S_L muxa4_xy(.o(da_xy[4]),  .sel(f0), .i1(cfg_xy[9]), .i0(cfg_xy[8]) );
    MUX2S_L muxa5_xy(.o(da_xy[5]),  .sel(f0), .i1(cfg_xy[11]), .i0(cfg_xy[10]) );
    MUX2S_L muxa6_xy(.o(da_xy[6]),  .sel(f0), .i1(cfg_xy[13]), .i0(cfg_xy[12]) );
    MUX2S_L muxa7_xy(.o(da_xy[7]),  .sel(f0), .i1(cfg_xy[15]), .i0(cfg_xy[14]) );
    MUX2S_L muxa8_xy(.o(da_xy[8]),  .sel(f0), .i1(cfg_xy[17]), .i0(cfg_xy[16]) );
    MUX2S_L muxa9_xy(.o(da_xy[9]),  .sel(f0), .i1(cfg_xy[19]), .i0(cfg_xy[18]) );
    MUX2S_L muxa10_xy(.o(da_xy[10]), .sel(f0), .i1(cfg_xy[21]), .i0(cfg_xy[20]) );
    MUX2S_L muxa11_xy(.o(da_xy[11]), .sel(f0), .i1(cfg_xy[23]), .i0(cfg_xy[22]) );
    MUX2S_L muxa12_xy(.o(da_xy[12]), .sel(f0), .i1(cfg_xy[25]), .i0(cfg_xy[24]) );
    MUX2S_L muxa13_xy(.o(da_xy[13]), .sel(f0), .i1(cfg_xy[27]), .i0(cfg_xy[26]) );
    MUX2S_L muxa14_xy(.o(da_xy[14]), .sel(f0), .i1(cfg_xy[29]), .i0(cfg_xy[28]) );
    MUX2S_L muxa15_xy(.o(da_xy[15]), .sel(f0), .i1(cfg_xy[31]), .i0(cfg_xy[30]) );

    MUX2S_L muxb0_xy(.o(db_xy[0]), .sel(f1), .i1(da_xy[1]), .i0(da_xy[0]) );
    MUX2S_L muxb1_xy(.o(db_xy[1]), .sel(f1), .i1(da_xy[3]), .i0(da_xy[2]) );
    MUX2S_L muxb2_xy(.o(db_xy[2]), .sel(f1), .i1(da_xy[5]), .i0(da_xy[4]) );
    MUX2S_L muxb3_xy(.o(db_xy[3]), .sel(f1), .i1(da_xy[7]), .i0(da_xy[6]) );
    MUX2S_L muxb4_xy(.o(db_xy[4]), .sel(f1), .i1(da_xy[9]), .i0(da_xy[8]) );
    MUX2S_L muxb5_xy(.o(db_xy[5]), .sel(f1), .i1(da_xy[11]), .i0(da_xy[10]) );
    MUX2S_L muxb6_xy(.o(db_xy[6]), .sel(f1), .i1(da_xy[13]), .i0(da_xy[12]) );
    MUX2S_L muxb7_xy(.o(db_xy[7]), .sel(f1), .i1(da_xy[15]), .i0(da_xy[14]) );

    MUX2S_L muxc0_xy(.o(dc_xy[0]), .sel(f2), .i1(db_xy[1]), .i0(db_xy[0]) );
    MUX2S_L muxc1_xy(.o(dc_xy[1]), .sel(f2), .i1(db_xy[3]), .i0(db_xy[2]) );
    MUX2S_L muxc2_xy(.o(dc_xy[2]), .sel(f2), .i1(db_xy[5]), .i0(db_xy[4]) );
    MUX2S_L muxc3_xy(.o(dc_xy[3]), .sel(f2), .i1(db_xy[7]), .i0(db_xy[6]) );

    MUX2S_L muxd0_xy(.o(dd_xy[0]), .sel(f3), .i1(dc_xy[1]), .i0(dc_xy[0]) );
    MUX2S_L muxd1_xy(.o(dd_xy[1]), .sel(f3), .i1(dc_xy[3]), .i0(dc_xy[2]) );

    MUX2S_L muxe0_xy(.o(de_xy), .sel(f4), .i1(dd_xy[1]), .i0(dd_xy[0]) );

    assign        dx = de_x;
    assign        dy = de_xy;

endmodule


//----------------------------------------------------------------------------
//
// module: LUT5
//
// description: 5 input lookup-table
// config:      32 bit init value
//
//----------------------------------------------------------------------------
module LUT5 (
         dx,
         f4,
         f3,
         f2,
         f1,
         f0
`ifdef FM_HACK
        ,config_data
`endif
         );

    input       f4;
    input       f3;
    input       f2;
    input       f1;
    input       f0;

    output      dx;

`ifdef CS_FORMALPRO_HACK
    wire [31:0]         config_data;
`else
     `ifdef  SIMULATION
          reg [31:0] config_data = 32'h00000000;
     `else
        `ifdef FM_HACK
            input   [31:0]  config_data;
        `else
            parameter  config_data = 32'h00000000;
        `endif
     `endif
`endif

    wire [31:0]         cfg = config_data;
    wire [15:0]         da;
    wire [7:0]          db;
    wire [3:0]          dc;
    wire [1:0]          dd;
    wire                de;

    MUX2S_L muxa0(.o(da[0]),  .sel(f0), .i1(cfg[1]), .i0(cfg[0]) );
    MUX2S_L muxa1(.o(da[1]),  .sel(f0), .i1(cfg[3]), .i0(cfg[2]) );
    MUX2S_L muxa2(.o(da[2]),  .sel(f0), .i1(cfg[5]), .i0(cfg[4]) );
    MUX2S_L muxa3(.o(da[3]),  .sel(f0), .i1(cfg[7]), .i0(cfg[6]) );
    MUX2S_L muxa4(.o(da[4]),  .sel(f0), .i1(cfg[9]), .i0(cfg[8]) );
    MUX2S_L muxa5(.o(da[5]),  .sel(f0), .i1(cfg[11]), .i0(cfg[10]) );
    MUX2S_L muxa6(.o(da[6]),  .sel(f0), .i1(cfg[13]), .i0(cfg[12]) );
    MUX2S_L muxa7(.o(da[7]),  .sel(f0), .i1(cfg[15]), .i0(cfg[14]) );
    MUX2S_L muxa8(.o(da[8]),  .sel(f0), .i1(cfg[17]), .i0(cfg[16]) );
    MUX2S_L muxa9(.o(da[9]),  .sel(f0), .i1(cfg[19]), .i0(cfg[18]) );
    MUX2S_L muxa10(.o(da[10]), .sel(f0), .i1(cfg[21]), .i0(cfg[20]) );
    MUX2S_L muxa11(.o(da[11]), .sel(f0), .i1(cfg[23]), .i0(cfg[22]) );
    MUX2S_L muxa12(.o(da[12]), .sel(f0), .i1(cfg[25]), .i0(cfg[24]) );
    MUX2S_L muxa13(.o(da[13]), .sel(f0), .i1(cfg[27]), .i0(cfg[26]) );
    MUX2S_L muxa14(.o(da[14]), .sel(f0), .i1(cfg[29]), .i0(cfg[28]) );
    MUX2S_L muxa15(.o(da[15]), .sel(f0), .i1(cfg[31]), .i0(cfg[30]) );

    MUX2S_L muxb0(.o(db[0]), .sel(f1), .i1(da[1]), .i0(da[0]) );
    MUX2S_L muxb1(.o(db[1]), .sel(f1), .i1(da[3]), .i0(da[2]) );
    MUX2S_L muxb2(.o(db[2]), .sel(f1), .i1(da[5]), .i0(da[4]) );
    MUX2S_L muxb3(.o(db[3]), .sel(f1), .i1(da[7]), .i0(da[6]) );
    MUX2S_L muxb4(.o(db[4]), .sel(f1), .i1(da[9]), .i0(da[8]) );
    MUX2S_L muxb5(.o(db[5]), .sel(f1), .i1(da[11]), .i0(da[10]) );
    MUX2S_L muxb6(.o(db[6]), .sel(f1), .i1(da[13]), .i0(da[12]) );
    MUX2S_L muxb7(.o(db[7]), .sel(f1), .i1(da[15]), .i0(da[14]) );

    MUX2S_L muxc0(.o(dc[0]), .sel(f2), .i1(db[1]), .i0(db[0]) );
    MUX2S_L muxc1(.o(dc[1]), .sel(f2), .i1(db[3]), .i0(db[2]) );
    MUX2S_L muxc2(.o(dc[2]), .sel(f2), .i1(db[5]), .i0(db[4]) );
    MUX2S_L muxc3(.o(dc[3]), .sel(f2), .i1(db[7]), .i0(db[6]) );

    MUX2S_L muxd0(.o(dd[0]), .sel(f3), .i1(dc[1]), .i0(dc[0]) );
    MUX2S_L muxd1(.o(dd[1]), .sel(f3), .i1(dc[3]), .i0(dc[2]) );

    MUX2S_L muxe0(.o(de), .sel(f4), .i1(dd[1]), .i0(dd[0]) );

    assign      dx = de;

	specify
       
	( f0 =>  dx    ) = (0,0);
	( f1 =>  dx    ) = (0,0);
	( f2 =>  dx    ) = (0,0);
	( f3 =>  dx    ) = (0,0);
	( f4 =>  dx    ) = (0,0);
	
     endspecify

endmodule


//----------------------------------------------------------------------------
//
// module: LUT6
//
// description: 6 input lookup-table
// config:      64 bit init value
//
//----------------------------------------------------------------------------

module LUT6 (
         xy,
         f5,
         f4,
         f3,
         f2,
         f1,
         f0
         );

    input       f5;
    input       f4;
    input       f3;
    input       f2;
    input       f1;
    input       f0;

    output      xy;

    parameter       config_data = 64'h0000_0000_0000_0000;

    wire [63:0]         cfg = config_data;
    wire [31:0]         da;
    wire [15:0]         db;
    wire [7:0]          dc;
    wire [3:0]          dd;
    wire [1:0]          de;
    wire                df;

    MUX2S_L muxa0 (.o(da[0]),  .sel(f0), .i1(cfg[1]),  .i0(cfg[0]) );
    MUX2S_L muxa1 (.o(da[1]),  .sel(f0), .i1(cfg[3]),  .i0(cfg[2]) );
    MUX2S_L muxa2 (.o(da[2]),  .sel(f0), .i1(cfg[5]),  .i0(cfg[4]) );
    MUX2S_L muxa3 (.o(da[3]),  .sel(f0), .i1(cfg[7]),  .i0(cfg[6]) );
    MUX2S_L muxa4 (.o(da[4]),  .sel(f0), .i1(cfg[9]),  .i0(cfg[8]) );
    MUX2S_L muxa5 (.o(da[5]),  .sel(f0), .i1(cfg[11]), .i0(cfg[10]) );
    MUX2S_L muxa6 (.o(da[6]),  .sel(f0), .i1(cfg[13]), .i0(cfg[12]) );
    MUX2S_L muxa7 (.o(da[7]),  .sel(f0), .i1(cfg[15]), .i0(cfg[14]) );
    MUX2S_L muxa8 (.o(da[8]),  .sel(f0), .i1(cfg[17]), .i0(cfg[16]) );
    MUX2S_L muxa9 (.o(da[9]),  .sel(f0), .i1(cfg[19]), .i0(cfg[18]) );
    MUX2S_L muxa10(.o(da[10]), .sel(f0), .i1(cfg[21]), .i0(cfg[20]) );
    MUX2S_L muxa11(.o(da[11]), .sel(f0), .i1(cfg[23]), .i0(cfg[22]) );
    MUX2S_L muxa12(.o(da[12]), .sel(f0), .i1(cfg[25]), .i0(cfg[24]) );
    MUX2S_L muxa13(.o(da[13]), .sel(f0), .i1(cfg[27]), .i0(cfg[26]) );
    MUX2S_L muxa14(.o(da[14]), .sel(f0), .i1(cfg[29]), .i0(cfg[28]) );
    MUX2S_L muxa15(.o(da[15]), .sel(f0), .i1(cfg[31]), .i0(cfg[30]) );
    MUX2S_L muxa16(.o(da[16]), .sel(f0), .i1(cfg[33]), .i0(cfg[32]) );
    MUX2S_L muxa17(.o(da[17]), .sel(f0), .i1(cfg[35]), .i0(cfg[34]) );
    MUX2S_L muxa18(.o(da[18]), .sel(f0), .i1(cfg[37]), .i0(cfg[36]) );
    MUX2S_L muxa19(.o(da[19]), .sel(f0), .i1(cfg[39]), .i0(cfg[38]) );
    MUX2S_L muxa20(.o(da[20]), .sel(f0), .i1(cfg[41]), .i0(cfg[40]) );
    MUX2S_L muxa21(.o(da[21]), .sel(f0), .i1(cfg[43]), .i0(cfg[42]) );
    MUX2S_L muxa22(.o(da[22]), .sel(f0), .i1(cfg[45]), .i0(cfg[44]) );
    MUX2S_L muxa23(.o(da[23]), .sel(f0), .i1(cfg[47]), .i0(cfg[46]) );
    MUX2S_L muxa24(.o(da[24]), .sel(f0), .i1(cfg[49]), .i0(cfg[48]) );
    MUX2S_L muxa25(.o(da[25]), .sel(f0), .i1(cfg[51]), .i0(cfg[50]) );
    MUX2S_L muxa26(.o(da[26]), .sel(f0), .i1(cfg[53]), .i0(cfg[52]) );
    MUX2S_L muxa27(.o(da[27]), .sel(f0), .i1(cfg[55]), .i0(cfg[54]) );
    MUX2S_L muxa28(.o(da[28]), .sel(f0), .i1(cfg[57]), .i0(cfg[56]) );
    MUX2S_L muxa29(.o(da[29]), .sel(f0), .i1(cfg[59]), .i0(cfg[58]) );
    MUX2S_L muxa30(.o(da[30]), .sel(f0), .i1(cfg[61]), .i0(cfg[60]) );
    MUX2S_L muxa31(.o(da[31]), .sel(f0), .i1(cfg[63]), .i0(cfg[62]) );

    MUX2S_L muxb0 (.o(db[0]),  .sel(f1), .i1(da[1]),  .i0(da[0]) );
    MUX2S_L muxb1 (.o(db[1]),  .sel(f1), .i1(da[3]),  .i0(da[2]) );
    MUX2S_L muxb2 (.o(db[2]),  .sel(f1), .i1(da[5]),  .i0(da[4]) );
    MUX2S_L muxb3 (.o(db[3]),  .sel(f1), .i1(da[7]),  .i0(da[6]) );
    MUX2S_L muxb4 (.o(db[4]),  .sel(f1), .i1(da[9]),  .i0(da[8]) );
    MUX2S_L muxb5 (.o(db[5]),  .sel(f1), .i1(da[11]), .i0(da[10]) );
    MUX2S_L muxb6 (.o(db[6]),  .sel(f1), .i1(da[13]), .i0(da[12]) );
    MUX2S_L muxb7 (.o(db[7]),  .sel(f1), .i1(da[15]), .i0(da[14]) );
    MUX2S_L muxb8 (.o(db[8]),  .sel(f1), .i1(da[17]), .i0(da[16]) );
    MUX2S_L muxb9 (.o(db[9]),  .sel(f1), .i1(da[19]), .i0(da[18]) );
    MUX2S_L muxb10(.o(db[10]), .sel(f1), .i1(da[21]), .i0(da[20]) );
    MUX2S_L muxb11(.o(db[11]), .sel(f1), .i1(da[23]), .i0(da[22]) );
    MUX2S_L muxb12(.o(db[12]), .sel(f1), .i1(da[25]), .i0(da[24]) );
    MUX2S_L muxb13(.o(db[13]), .sel(f1), .i1(da[27]), .i0(da[26]) );
    MUX2S_L muxb14(.o(db[14]), .sel(f1), .i1(da[29]), .i0(da[28]) );
    MUX2S_L muxb15(.o(db[15]), .sel(f1), .i1(da[31]), .i0(da[30]) );

    MUX2S_L muxc0(.o(dc[0]), .sel(f2), .i1(db[1]),  .i0(db[0]) );
    MUX2S_L muxc1(.o(dc[1]), .sel(f2), .i1(db[3]),  .i0(db[2]) );
    MUX2S_L muxc2(.o(dc[2]), .sel(f2), .i1(db[5]),  .i0(db[4]) );
    MUX2S_L muxc3(.o(dc[3]), .sel(f2), .i1(db[7]),  .i0(db[6]) );
    MUX2S_L muxc4(.o(dc[4]), .sel(f2), .i1(db[9]),  .i0(db[8]) );
    MUX2S_L muxc5(.o(dc[5]), .sel(f2), .i1(db[11]), .i0(db[10]) );
    MUX2S_L muxc6(.o(dc[6]), .sel(f2), .i1(db[13]), .i0(db[12]) );
    MUX2S_L muxc7(.o(dc[7]), .sel(f2), .i1(db[15]), .i0(db[14]) );

    MUX2S_L muxd0(.o(dd[0]), .sel(f3), .i1(dc[1]), .i0(dc[0]) );
    MUX2S_L muxd1(.o(dd[1]), .sel(f3), .i1(dc[3]), .i0(dc[2]) );
    MUX2S_L muxd2(.o(dd[2]), .sel(f3), .i1(dc[5]), .i0(dc[4]) );
    MUX2S_L muxd3(.o(dd[3]), .sel(f3), .i1(dc[7]), .i0(dc[6]) );

    MUX2S_L muxe0(.o(de[0]), .sel(f4), .i1(dd[1]), .i0(dd[0]) );
    MUX2S_L muxe1(.o(de[1]), .sel(f4), .i1(dd[3]), .i0(dd[2]) );

    MUX2S_L muxf0(.o(df), .sel(f5), .i1(de[1]), .i0(de[0]) );

    assign      xy = df;

endmodule

//----------------------------------------------------------------------------
//
// module: MUXF7
//
// description:     2-to-1 MUXF7 
//
//----------------------------------------------------------------------------

module MUXF7 (
          out,
          sel,
          in0,
          in1
          );

   input        in0;
   input        in1;
   input        sel;

   output       out; 

   assign       out = (sel == 1'b0) ? in0 :
                      (sel == 1'b1) ? in1 : 1'bx;

endmodule


//----------------------------------------------------------------------------
//
// module: MUXF8
//
// description:     2-to-1 MUXF8
//
//----------------------------------------------------------------------------

module MUXF8 (
          out,
          sel,
          in0,
          in1
          );

   input        in0;
   input        in1;
   input        sel;

   output       out; 

   assign       out = (sel == 1'b0) ? in0 :
                      (sel == 1'b1) ? in1 : 1'bx;

endmodule

//----------------------------------------------------------------------------
module MUX2S (
    o,
    sel,
    i0,
    i1
);

input        i0;
input        i1;
input        sel;

output        o;

`ifndef CS_SW_SKEL
    assign         o = (sel == 1'b0) ? i0 :
                       (sel == 1'b1) ? i1 : 1'bx;
    specify
        ( i0 =>  o ) = (0,0);                     
        ( i1 =>  o ) = (0,0);                     
        ( sel =>  o ) = (0,0);                                       
    endspecify
`endif

endmodule


//----------------------------------------------------------------------------
//
// module: OBUFT
//
// description:     tri-state output buffer
//
//----------------------------------------------------------------------------

module OBUFT (p, i, oe);
    input   i;
    input   oe;
    output  p;
    
    bufif0 t1 (p, i, oe);
   
endmodule


//----------------------------------------------------------------------------
//
// module: OBUF
//
// description:     output buffer
//
//----------------------------------------------------------------------------

module OBUF (p, i);
    input   i;
    output  p;

    buf b1 (p, i);

endmodule
//----------------------------------------------------------------------------
//
// module: OSC
//
// description: on-chip oscillator
//
//----------------------------------------------------------------------------


module OSCV1 (
    OSC
);

output OSC;

parameter osc_pd  = 1'b0;
parameter osc_stb = 1'b0;

reg clk;

initial begin
    clk = 1'b0;
    forever #4.76 clk=~clk;
end

assign OSC = osc_pd ? 0 : ( osc_stb ? 0 : clk);

endmodule

module RBUF_GATE (
    clk,
    en,
    clk_out
);

    input clk;
    input en;
    output clk_out;
    
    reg en_latch;
    always @(*) begin
        if (clk == 0)
            en_latch <= en;
    end

    assign clk_out = clk & en_latch;

endmodule

module RBUF(in, out);
    input in;
    output out;
    
    buf (out, in);
    
endmodule

//----------------------------------------------------------------------------
//
// module: REG2CKSR
//
// description: 1 bit register or latch with "sr" to preset value
//		to use as latch, hold mclk_b=0 (master stage transparent)
//		Two clock inputs, config-mux selectable
// config:      PRESET, CLKSRSEL
//
//----------------------------------------------------------------------------
module REG2CKSR (
    qx,
    di,
    sr0,
    sr1,
    mclk0b,
    sclk0,
    mclk1b,
    sclk1
`ifdef FM_HACK
	,PRESET
	,CLKSRSEL
`endif
);

output        qx;

input         di;
input         sr0, sr1;
input         mclk0b;
input         sclk0;
input         mclk1b;
input         sclk1;


`ifdef CS_FORMALPRO_HACK
   wire        PRESET;
   wire        CLKSRSEL;
`else
     `ifdef  SIMULATION
          reg       PRESET   = 1'b0;
          reg       CLKSRSEL = 1'b0;
     `else
		`ifdef FM_HACK
	        input PRESET;
	        input CLKSRSEL;
		`else
	        parameter PRESET   = 1'b0;
	        parameter CLKSRSEL = 1'b0;
		`endif
     `endif
`endif

wire            mclkb;
wire            sclk;
   reg          qx_reg;
   reg          mout;

`ifndef CS_SW_SKEL



   assign    mclkb = (CLKSRSEL == 1'b1) ? mclk1b : mclk0b;
   assign    sclk   = (CLKSRSEL == 1'b1) ? sclk1   : sclk0;
 
   wire    a_sr = (CLKSRSEL == 1'b1) ? sr1: sr0;
   wire    rst_ =  a_sr || (PRESET == 1'b1);
   wire    set  = ~a_sr && (PRESET == 1'b1);    
  
   always @(*) begin
       if (rst_ == 1'b0) begin
           mout <= 1;
       end else if (set) begin
           mout <= 0;
       end else if (mclkb == 0) begin
           mout <= ~di;
       end else if (mclkb == 1) begin
           mout <= mout;
       end else
           mout <= 1'bx;
   end

   initial qx_reg = 1'b0;
   
   always @(*) begin
       if (rst_ == 1'b0) begin
           qx_reg <= 1;
       end else if (set) begin
           qx_reg <= 0;
       end else if (sclk == 1) begin
           qx_reg <= mout;
       end else if (sclk == 0) begin
           qx_reg <= qx_reg;
       end else
           qx_reg <= 1'bx;
   end 
   
   assign qx = ~qx_reg;

    specify
        (posedge sclk0 => ( qx  +: di) ) = (0, 0) ;
        (posedge sclk1 => ( qx  +: di) ) = (0, 0) ;
              
        $setuphold(posedge sclk, di, 0, 0);
        $recrem(a_sr, posedge sclk,  0, 0);
        
     endspecify
   

`endif

endmodule


//----------------------------------------------------------------------------
//
// module: REG
//
// description:     1-bit register 
//
//----------------------------------------------------------------------------

module REG (
    qx ,
    di ,
    sr ,
    en ,
    clk 
);

parameter   init            = 1'b0;     //register initial value, default is "1'b0"
parameter   sr_value        = 1'b0;     //output value of reset, default is "1'b0"


parameter   always_en       = 1'b0;     //0 for use external enable, 1 for always enable, default is "1'b0"
parameter   no_sr           = 1'b0;     //0 for use external set/reset, 1 for no set/reset, default is "1'b0"
parameter   latch_mode      = 1'b0;     //0 for register mode, 1 for latch mode, default is "1'b0"
parameter   sync_mode       = 1'b0;     //0 for asynchronous mode, 1 for synchronous mode, default is "1'b0"
parameter   clk_inv         = 1'b0;     //0 for clock not inverted, 1 for clock inverted, default is "1'b0"
parameter   en_inv          = 1'b0;     //0 for enable not inverted, 1 for enable inverted, default is "1'b0"
parameter   sr_inv          = 1'b0;     //0 for sr not inverted, 1 for sr inverted, default is "1'b0"

output  qx ;
input   di ;
input   sr ;
input   en ;
input   clk ;

wire clk_in;
wire reset;
wire reg_enable;
reg qx;

initial begin
    qx = init;
end

assign reg_enable    = (always_en == 1'b1) ? 1'b1 : (en_inv == 1'b0) ? en : ~en;
assign clk_in     = (clk_inv == 1'b0) ? clk : ~clk;
assign reset    = (no_sr == 1'b1) ? 1'b0 : ((sr_inv == 1'b0) ? sr : ~sr);

if(latch_mode == 1'b1) begin
    always @ (*) begin
        if(reset)  
            qx <= sr_value;
        else if(reg_enable & clk_in)  
            qx <= di;
    end
end
else begin
    if(sync_mode == 1'b0) begin
        always @(posedge clk_in or posedge reset) begin
            if(reset)
                qx <= sr_value;
            else 
                if (reg_enable) begin
                    qx <= di;
                end
        end
    end else begin
        always @(posedge clk_in) begin
            if(reset)
                qx <= sr_value;
            else 
                if (reg_enable) begin
                    qx <= di;
                end
        end
    end
end

endmodule

//----------------------------------------------------------------------------
//
// module: CFGMUX2S1
//
// description: 1 bit wide 2:1 mux
// config:      1 bit to select the input
//
//----------------------------------------------------------------------------
module CFGMUX2S1 (
    o,
    i0,
    i1
`ifdef FM_HACK
	,SEL
`endif
);

input        i0;
input        i1;
output        o;
   
`ifdef CS_FORMALPRO_HACK
    wire         SEL;
`else
     `ifdef  SIMULATION
          reg       SEL = 1'b0;
     `else
		`ifdef FM_HACK
			input	SEL;
		`else
			parameter SEL = 1'b0;
			parameter         CS_PRIM = "bool true";
		`endif
     `endif
`endif
   
`ifndef CS_SW_SKEL
    assign         o = SEL ? i1 : i0;

`endif

endmodule

// end of FIFO18K 

module SIO (
    f_id,
    clk_en,
    fclk,
    od,
    oen,
    rstn,
    setn,
    PAD
);

input            clk_en;
input            fclk;
input    [1:0]    od;
input            oen;
input            rstn;
input            setn;

output    [1:0]    f_id;

inout            PAD;

//Analog IO parameters
parameter   KEEP      = 2'b0;
parameter   NDR       = 4'b0;
parameter   NS_LV     = 2'b0;
parameter   PDR       = 4'b0;
parameter   RX_DIG_EN = 1'b0;
parameter   VPCI_EN   = 1'b0;

//Logic IO/IOC parameters
parameter   OEN_SEL      = 2'b0;    //2'b00 :  1'b1
                                    //2'b01 :  1'b0
                                    //2'b10 :  f_oen
                                    //2'b11 : ~f_oen
parameter   OUT_SEL      = 2'b0;    //2'b00 :  1'b1
                                    //2'b01 :  1'b0
                                    //2'b10 :  f_od
                                    //2'b11 : ~f_od

parameter   CLK_INV      = 1'b0;    //1'b1 : ~fclk
                                    //1'b0 : fclk
parameter   FCLK_GATE_EN = 1'b0;    //1'b0 : no clock gating
                                    //1'b1 : clock gating use clk_en
parameter   SETN_INV     = 1'b0;    //1'b1 : setn inverted
                                    //1'b0 : setn not inverted
parameter   SETN_SYNC    = 1'b0;    //1'b1 : setn sync
                                    //1'b0 : setn async
parameter   RSTN_INV     = 1'b0;    //1'b1 : rstn inverted
                                    //1'b0 : rstn not inverted
parameter   RSTN_SYNC    = 1'b0;    //1'b1 : rstn sync
                                    //1'b0 : rstn async
parameter   OEN_SETN_EN  = 1'b0;    //1'b1 : oen setn enabled
                                    //1'b0 : oen setn disabled
parameter   OD_SETN_EN   = 1'b0;    //1'b1 : od setn enabled
                                    //1'b0 : od setn disabled
parameter   ID_SETN_EN   = 1'b0;    //1'b1 : id setn enabled
                                    //1'b0 : id setn disabled
parameter   OEN_RSTN_EN  = 1'b0;    //1'b1 : oen rstn enabled
                                    //1'b0 : oen rstn disabled
parameter   OD_RSTN_EN   = 1'b0;    //1'b1 : od rstn enabled
                                    //1'b0 : od rstn disabled
parameter   ID_RSTN_EN   = 1'b0;    //1'b1 : id rstn enabled
                                    //1'b0 : id rstn disabled
parameter   FOEN_SEL    = 1'b0;     //1'b1 : registered
                                    //1'b0 : bypassed
parameter   FOUT_SEL    = 1'b0;     //1'b1 : registered
                                    //1'b0 : bypassed
parameter   FIN_SEL     = 1'b0;     //1'b1 : registered
                                    //1'b0 : bypassed
parameter   DDR_EN       = 1'b0;    //DDR enable
parameter   DDR_REG_EN   = 1'b0;    //SDR fast output: DDR_REG = 1'b0 && FOUT_SEL = 1'b1
                                    //DDR with 1 pipeline: DDR_REG = 1'b1 && FOUT_SEL = 1'b1
parameter   DDR_PREG_EN  = 1'b0;    //SDR fast input: DDR_PREG = 1'b0 && FIN_SEL = 1'b1
                                    //DDR with 1 pipeline: DDR_PREG = 1'b1 && FOUT_SEL = 1'b1
parameter   DDR_NREG_EN  = 1'b0;

parameter   is_clk_io = "false";

wire        f_od;
wire        f_oen;
wire        id;

BASIC_IO basic_io_inst0 (
    .f_od    (f_od),
    .f_oen    (f_oen),

    .id    (id),
    .PAD    (PAD)
);
defparam basic_io_inst0.CFG_KEEP      = KEEP;
defparam basic_io_inst0.CFG_NDR       = NDR;
defparam basic_io_inst0.CFG_NS_LV     = NS_LV;
defparam basic_io_inst0.CFG_PDR       = PDR;
defparam basic_io_inst0.CFG_RX_DIG_EN = RX_DIG_EN;
defparam basic_io_inst0.VPCI_EN       = VPCI_EN;

defparam basic_io_inst0.CFG_OEN_SEL   = OEN_SEL;
defparam basic_io_inst0.CFG_OUT_SEL   = OUT_SEL;

IOC_CMOS ioc_cmos_inst0 (
    .clk_en    (clk_en),
    .fclk    (fclk),
    .id    (id),
    .od    (od[1:0]),
    .oen    (oen),
    .rstn    (rstn),
    .setn    (setn),

    .f_id    (f_id[1:0]),
    .f_od    (f_od),
    .f_oen    (f_oen)
);
defparam ioc_cmos_inst0.CFG_CLK_INV      = CLK_INV;
defparam ioc_cmos_inst0.CFG_FCLK_GATE_EN = FCLK_GATE_EN;
defparam ioc_cmos_inst0.CFG_SETN_INV     = SETN_INV;
defparam ioc_cmos_inst0.CFG_SETN_SYNC    = SETN_SYNC;
defparam ioc_cmos_inst0.CFG_OEN_SETN_EN  = OEN_SETN_EN;
defparam ioc_cmos_inst0.CFG_OD_SETN_EN   = OD_SETN_EN;
defparam ioc_cmos_inst0.CFG_ID_SETN_EN   = ID_SETN_EN;
defparam ioc_cmos_inst0.CFG_RSTN_INV     = RSTN_INV;
defparam ioc_cmos_inst0.CFG_RSTN_SYNC    = RSTN_SYNC;
defparam ioc_cmos_inst0.CFG_OEN_RSTN_EN  = OEN_RSTN_EN;
defparam ioc_cmos_inst0.CFG_OD_RSTN_EN   = OD_RSTN_EN;
defparam ioc_cmos_inst0.CFG_ID_RSTN_EN   = ID_RSTN_EN;
defparam ioc_cmos_inst0.CFG_FOEN_SELN    = !FOEN_SEL;
defparam ioc_cmos_inst0.CFG_FOUT_SELN    = !FOUT_SEL;
defparam ioc_cmos_inst0.CFG_FIN_SELN     = !FIN_SEL;
defparam ioc_cmos_inst0.CFG_DDR          = DDR_EN;
defparam ioc_cmos_inst0.CFG_DDR_REG      = DDR_REG_EN;
defparam ioc_cmos_inst0.CFG_DDR_PREG     = DDR_PREG_EN;
defparam ioc_cmos_inst0.CFG_DDR_NREG     = DDR_NREG_EN;

endmodule
module BASIC_IO(
	 id
	,f_od
	,f_oen
	,PAD
`ifdef FM_HACK
	,CFG_KEEP
	,CFG_NDR
	,CFG_NS_LV
	,CFG_OEN_SEL
	,CFG_OUT_SEL
	,CFG_PDR
	,CFG_RX_DIG_EN
	,VPCI_EN
`endif
);

output		id;
input		f_od;
input		f_oen;
inout		PAD;
`ifdef FM_HACK
input	[1:0]	CFG_KEEP;
input	[3:0]	CFG_NDR;
input	[1:0]	CFG_NS_LV;
input	[1:0]	CFG_OEN_SEL;
input	[1:0]	CFG_OUT_SEL;
input	[3:0]	CFG_PDR;
input		CFG_RX_DIG_EN;
input		VPCI_EN;
`else
parameter 	CFG_KEEP         = 2'b0;
parameter 	CFG_NDR          = 4'b0;
parameter 	CFG_NS_LV        = 2'b0;
parameter 	CFG_OEN_SEL      = 2'b0;
parameter 	CFG_OUT_SEL      = 2'b0;
parameter 	CFG_PDR          = 4'b0;
parameter 	CFG_RX_DIG_EN    = 1'b0;
parameter 	VPCI_EN          = 1'b0;
`endif

io_data data_path (
	.f_oen		(f_oen),
	.f_od		(f_od),
	.rxd		(RXD),
	.CFG_OEN_SEL	(CFG_OEN_SEL[1:0]),
	.CFG_OUT_SEL	(CFG_OUT_SEL[1:0]),

	.id		(id),
	.ted		(TED),
	.txd		(TXD) 
);

CSGPIO ana_IO (
	.TXD		(TXD),
	.TED		(TED),
	.NS_LV		(CFG_NS_LV[1:0]),
	.PDR		(CFG_PDR[3:0]),
	.NDR		(CFG_NDR[3:0]),
	.KEEP		(CFG_KEEP[1:0]),
	.RX_DIG_EN	(CFG_RX_DIG_EN),
	.VPCI_EN	(VPCI_EN),

	.RXD		(RXD),
	.PAD		(PAD)
);

endmodule

module IOC_CMOS(
     clk_en
    ,oen
    ,f_oen
    ,od
    ,f_od
    ,id
    ,f_id
    ,fclk
    ,rstn
    ,setn
`ifdef FM_HACK
    ,CFG_CLK_INV
    ,CFG_DDR
    ,CFG_FOEN_SELN
    ,CFG_FOUT_SELN
    ,CFG_FIN_SELN
    ,CFG_FCLK_GATE_EN
    ,CFG_SETN_INV
    ,CFG_SETN_SYNC
    ,CFG_OEN_SETN_EN
    ,CFG_OD_SETN_EN
    ,CFG_ID_SETN_EN
    ,CFG_RSTN_INV
    ,CFG_RSTN_SYNC
    ,CFG_OEN_RSTN_EN
    ,CFG_OD_RSTN_EN
    ,CFG_ID_RSTN_EN
    ,CFG_DDR_REG
    ,CFG_DDR_PREG
    ,CFG_DDR_NREG
`endif
);

input          clk_en;
input          oen;
output         f_oen;
input  [1:0]   od;
output         f_od;
input          id;
output [1:0]   f_id;
input          fclk;
input          rstn;
input          setn;
`ifdef FM_HACK
input          CFG_CLK_INV;
input          CFG_DDR;
input          CFG_FOEN_SELN;
input          CFG_FOUT_SELN;
input          CFG_FIN_SELN;
input          CFG_FCLK_GATE_EN;
input          CFG_SETN_INV;
input          CFG_SETN_SYNC;
input          CFG_OEN_SETN_EN;
input          CFG_OD_SETN_EN;
input          CFG_ID_SETN_EN;
input          CFG_RSTN_INV;
input          CFG_RSTN_SYNC;
input          CFG_OEN_RSTN_EN;
input          CFG_OD_RSTN_EN;
input          CFG_ID_RSTN_EN;
input          CFG_DDR_REG;
input          CFG_DDR_PREG;
input          CFG_DDR_NREG;
`else
parameter   CFG_CLK_INV                =1'b0;
parameter   CFG_DDR                    =1'b0;
parameter   CFG_FOEN_SELN              =1'b0;
parameter   CFG_FOUT_SELN              =1'b0;
parameter   CFG_FIN_SELN               =1'b0;
parameter   CFG_FCLK_GATE_EN           =1'b0;
parameter   CFG_SETN_INV               =1'b0;
parameter   CFG_SETN_SYNC              =1'b0;
parameter   CFG_OEN_SETN_EN            =1'b0;
parameter   CFG_OD_SETN_EN             =1'b0;
parameter   CFG_ID_SETN_EN             =1'b0;
parameter   CFG_RSTN_INV               =1'b0;
parameter   CFG_RSTN_SYNC              =1'b0;
parameter   CFG_OEN_RSTN_EN            =1'b0;
parameter   CFG_OD_RSTN_EN             =1'b0;
parameter   CFG_ID_RSTN_EN             =1'b0;
parameter   CFG_DDR_REG                =1'b0;
parameter   CFG_DDR_PREG               =1'b0;
parameter   CFG_DDR_NREG               =1'b0;
`endif
`ifndef FM_HACK
    reg cf_rstn;
    wire GSR;
    glbsr inst( .GSR( GSR )) ;

    initial begin
        cf_rstn = 1'b0;
        @(posedge GSR);
        cf_rstn = 1'b1;
    end
`else
    wire cf_rstn = 1'b1;
`endif
wire [1:0] f_id;
wire [1:0] f_id_n;
wire rstn_inv_cfg = CFG_RSTN_INV ? ~rstn : rstn;
wire setn_inv_cfg = CFG_SETN_INV ? ~setn : setn;

//wire fclk_gate_en;
wire fclk_gate_mux;


ioc_latn_ar rstn_syn (
     .ck    (fclk_gate_mux )
    ,.d     (rstn_inv_cfg  )
    ,.q     (rstn_lat      )
    ,.rstn  (cf_rstn       )
);

ioc_latn_ar setn_syn (
     .ck    (fclk_gate_mux )
    ,.d     (setn_inv_cfg  )
    ,.q     (setn_lat      )
    ,.rstn  (cf_rstn       )
);

assign rstn_reg_mux = CFG_RSTN_SYNC ? ~fclk_gate_mux | rstn_lat : rstn_inv_cfg;
assign setn_reg_mux = CFG_SETN_SYNC ? ~fclk_gate_mux | setn_lat : setn_inv_cfg;

assign oen_rstn = CFG_OEN_RSTN_EN ? rstn_reg_mux : 1'b1;
assign od_rstn  = CFG_OD_RSTN_EN ? rstn_reg_mux : 1'b1;
assign id_rstn  = CFG_ID_RSTN_EN ? rstn_reg_mux : 1'b1;

assign t_oen_setn = CFG_OEN_SETN_EN ? setn_reg_mux : 1'b1;
assign t_od_setn  = CFG_OD_SETN_EN ? setn_reg_mux : 1'b1;
assign t_id_setn  = CFG_ID_SETN_EN ? setn_reg_mux : 1'b1;

assign oen_setn = cf_rstn & t_oen_setn;
assign od_setn  = cf_rstn & t_od_setn;
assign id_setn  = cf_rstn & t_id_setn;

assign fclk_cfginv = CFG_CLK_INV ? ~fclk : fclk;
//clk_inv u_fclk_inv ( .ck_(fclk), .ck_n(fclk_inv));
//gclk_clk_mux u_fclk_cfginv (.ck_i0(fclk), .ck_i1(fclk_inv), .sel(CFG_CLK_INV), .ck_out(fclk_cfginv));

reg fclk_gate_en;
always@(*)
begin
    if (CFG_FCLK_GATE_EN)
        fclk_gate_en <= 1'b1;
    else
    begin
        if (!fclk_cfginv)
            fclk_gate_en <= clk_en;
        else
            fclk_gate_en <= fclk_gate_en;
    end
end
assign fclk_gate_mux = fclk_gate_en & fclk_cfginv;

//assign CFG_FCLK_GATE_EN_inv = ~CFG_FCLK_GATE_EN;
//LNSNQD4BWP u0_fclk_gate (.EN(fclk_cfginv),.D(clk_en),.SDN(CFG_FCLK_GATE_EN_inv),.Q(fclk_gate_en));
//gclk_clk_and u_fclk_gate (.ck_i0(fclk_cfginv),.ck_i1(fclk_gate_en),.ck_out(fclk_gate_mux));

//ECO 20140521 beg
//assign ioc_clk_out = fclk_gate_mux;
//ECO 20140521 end

//f_oen
ioc_dff_asr u0_oen(
     .ck     ( fclk_gate_mux )
    ,.d      ( oen           )
    ,.q      ( oen_reg       )
    ,.rstn   ( oen_rstn      )
    ,.setn   ( oen_setn      )
);

assign f_oen = (CFG_FOEN_SELN == 0) ? oen_reg : oen;

//f_od
ioc_dff_asr u_od0(
     .ck     ( fclk_gate_mux )
    ,.d      ( od[0]        )
    ,.q      ( out_reg0      )
    ,.rstn   ( od_rstn       )
    ,.setn   ( od_setn       )
);

ioc_dff_asr u_od1(
     .ck     ( fclk_gate_mux )
    ,.d      ( od[1]        )
    ,.q      ( out_reg1      )
    ,.rstn   ( od_rstn       )
    ,.setn   ( od_setn       )
);
assign oddr_reg0 = CFG_DDR_REG ? out_reg0 : od[0];
assign oddr_reg1 = CFG_DDR_REG ? out_reg1 : od[1];

ioc_dff_asr u_od2(
     .ck     ( fclk_gate_mux )
    ,.d      ( oddr_reg0     )
    ,.q      ( out_reg2      )
    ,.rstn   ( od_rstn       )
    ,.setn   ( od_setn       )
);

ioc_dffn_asr u_od3(
     .ckb    ( fclk_gate_mux )
    ,.d      ( oddr_reg1     )
    ,.q      ( out_reg3      )
    ,.rstn   ( od_rstn       )
    ,.setn   ( od_setn       )
);
//assign ddr_sel = ~(fclk_gate_mux & CFG_DDR);
assign ddr_sel = fclk_gate_mux & CFG_DDR;
assign f_od_ddr = ddr_sel ? out_reg3 : out_reg2;
assign f_od = (CFG_FOUT_SELN == 0) ? f_od_ddr : od[0];
//assign f_od_fb = f_od;

//f_id
ioc_dff_asr u_id0(
     .ck     ( fclk_gate_mux )
    ,.d      ( id            )
    ,.q      ( iddr_reg0       )
    ,.rstn   ( id_rstn       )
    ,.setn   ( id_setn       )
);

ioc_dffn_asr u_id1(
     .ckb    ( fclk_gate_mux )
    ,.d      ( id            )
    ,.q      ( iddr_reg1       )
    ,.rstn   ( id_rstn       )
    ,.setn   ( id_setn       )
);

ioc_dff_asr u_id2(
     .ck     ( fclk_gate_mux )
    ,.d      ( iddr_reg0     )
    ,.q      ( in_reg0       )
    ,.rstn   ( id_rstn       )
    ,.setn   ( id_setn       )
);

ioc_dff_asr u_id3(
     .ck     ( fclk_gate_mux )
    ,.d      ( iddr_reg1     )
    ,.q      ( in_reg1       )
    ,.rstn   ( id_rstn       )
    ,.setn   ( id_setn       )
);

assign f_id0_t = CFG_DDR_PREG ? in_reg0 : iddr_reg0;
assign f_id[0] = (CFG_FIN_SELN == 0) ? f_id0_t : id;
assign f_id[1] = CFG_DDR_NREG ? in_reg1 : iddr_reg1;

endmodule

module ioc_dff_ar(
     ck
    ,d
    ,q
    ,rstn
);

input  ck;
input  d;
output q;
input  rstn;

reg q;

always@(posedge ck or negedge rstn)
begin
    if(~rstn)
        q <= `DLY 1'b0;
    else
        q <= `DLY d;
end

endmodule

module ioc_dff_asr(
     ck
    ,d
    ,q
    ,rstn
    ,setn
);

input  ck;
input  d;
output q;
input  rstn;
input  setn;

reg q;

always@(posedge ck or negedge setn or negedge rstn)
begin
    if(~rstn)
	q <= `DLY 1'b0;
    else if(~setn)
	q <= `DLY 1'b1;
    else
	q <= `DLY d;
end

endmodule

module ioc_dffn_asr(
     ckb
    ,d
    ,q
    ,rstn
    ,setn
);

input  ckb;
input  d;
output q;
input  rstn;
input  setn;

reg q;

always@(negedge ckb or negedge setn or negedge rstn)
begin
    if(~rstn)
	q <= `DLY 1'b0;
    else if(~setn)
	q <= `DLY 1'b1;
    else
	q <= `DLY d;
end

endmodule


`ifndef DLY
    `define DLY #1
`endif

module ioc_latn_ar(
     ck
    ,d
    ,q
    ,rstn
);

input  ck;
input  d;
output q;
input  rstn;

reg q;

always@(*)
begin
    if(~rstn)
        q <= `DLY 1'b0;
    else if(ck == 1'b0)
        q <= `DLY d;
end

endmodule



module CSGPIO (
     TXD
    ,TED
    ,RXD
    ,PAD
    ,NS_LV
    ,PDR
    ,NDR
    ,KEEP
    ,RX_DIG_EN
    ,VPCI_EN
);

input TXD;                    // data output to PDA
input TED;                    // data output enable
output RXD;                   // data input from PAD 
inout PAD;                    // PAD

// ------------- total 106 config bits
input [1:0] NS_LV;         // slew rate control
input [3:0] PDR;           // driving strength, P
input [3:0] NDR;           // driving strength, N
input [1:0] KEEP;          // pullup,pulldown,bus-keeper

input RX_DIG_EN;           // LVCMOS RX enable

input VPCI_EN;             // PCI diode enable
//--------------
 
supply1 VDDIO, VDDCORE;
supply0 VSSIO, VSSCORE;

    /* rx */

    wire rxe;
    
    assign rxe=~(RX_DIG_EN);
    bufif1 rx00 (RXD,PAD,RX_DIG_EN);
    nmos  NMOSrx0 (RXD, VSSIO, rxe);


    //------- driver 
//------LVCMOS  
    wire pdr0a,ndr0a,pdr0b,ndr0b,pdr0c,ndr0c,pdr1a,ndr1a,pdr1b,ndr1b,pdr1c,ndr1c;
      
    assign pdr0a=PDR[0]|PDR[1]|PDR[2]|PDR[3];
    assign ndr0a=NDR[0]|NDR[1]|NDR[2]|NDR[3];
    assign pdr0b=pdr0a&(NS_LV[0]|NS_LV[1])&(!TED);
    assign ndr0b=ndr0a&(NS_LV[0]|NS_LV[1])&(!TED);
    assign pdr0c=~( pdr0b & TXD );
    assign ndr0c=~(!ndr0b | TXD );
    
    pmos  pdriver00 (PAD, VDDIO, pdr0c);
    nmos  ndriver00 (PAD, VSSIO, ndr0c);


    
    //--------- Pullup 
    wire pullup0,pullup1,pulldn0,pulldn1;
    
    assign pullup0=~(KEEP[1]&((!KEEP[0])|(KEEP[0]&!RXD)));
    
    rpmos    PMOS0A    (PAD,        PAD0PA_out, pullup0);
    rpmos    PMOS0B    (PAD0PA_out, PAD0PB_out, pullup0);
    rpmos    PMOS0C    (PAD0PB_out, VDDIO,      pullup0);    


    //---------- Pulldown (Strength to Weak)
    assign pulldn0=(KEEP[0]&((!KEEP[1])|(KEEP[1]&RXD)));
    
    rnmos    NMOS0A    (PAD,        PAD0NA_out, pulldn0);
    rnmos    NMOS0B    (PAD0NA_out, PAD0NB_out, pulldn0);
    rnmos    NMOS0C    (PAD0NB_out, VSSIO,      pulldn0);

    
    //-------- keeper 

endmodule

module io_data(
     f_oen
    ,f_od
    ,id
    ,ted
    ,txd
    ,rxd
    ,CFG_OEN_SEL
    ,CFG_OUT_SEL
);

input          f_oen;
input          f_od;
output         id;
output         ted;
output         txd;
input          rxd;
input  [1:0]   CFG_OEN_SEL;
input  [1:0]   CFG_OUT_SEL;

assign ted = CFG_OEN_SEL == 2'b00 ?  1'b1  :
             CFG_OEN_SEL == 2'b01 ?  1'b0  :
             CFG_OEN_SEL == 2'b10 ?  f_oen :
             CFG_OEN_SEL == 2'b11 ? ~f_oen :
                                     1'bx  ;

assign txd = CFG_OUT_SEL == 2'b00 ?  1'b1  :
             CFG_OUT_SEL == 2'b01 ?  1'b0  :
             CFG_OUT_SEL == 2'b10 ?  f_od  :
             CFG_OUT_SEL == 2'b11 ? ~f_od  :
                                     1'bx  ;
assign id = rxd;

endmodule

module spi_interface(
    sclk,
    sdo,
    cson,
    sdi
        );

	input   sclk; //connect to user sclk
	input   sdi;  //connect to user sdo
	input   cson; //connect to user cson
	output  sdo;  //connect to user sdi

endmodule

//----------------------------------------------------------------------------
//
// module: SUPPLY 0
//
// description: GND Cell
//
//----------------------------------------------------------------------------
module supply0_guts (
            gnd
	     );

    output		gnd;

assign gnd = 0;
endmodule
//----------------------------------------------------------------------------
//
// module: SUPPLY 1
//
// description: vcc Cell
//
//----------------------------------------------------------------------------
module supply1_guts (
            vcc
         );

    output      vcc;

assign vcc = 1;
endmodule
module USERPORV1 (
    por
);

output  por;

`ifndef FM_HACK
    reg por;

    wire GSR;
    glbsr glbsr_inst(.GSR(GSR));
    initial begin
        por = 1'b0;
        @(posedge GSR);
        #10;
        por = 1'b1;
    end
`else
    wire por = 1'b1;
`endif

endmodule


module VCC (Y);
    output Y;

    assign Y = 1;
endmodule

module glbsr( GSR );

parameter T_GLBSR = 1;

output GSR;
reg GSR_t;

assign (weak1, weak0) GSR = GSR_t;

initial begin
    GSR_t = 1'b0;
    #(T_GLBSR)
    GSR_t = 1'b1;
end

endmodule
//==================================================================================
// Hercules Microelectronics
// Product Name: PLL_TOP
// Version     : 0.4
// Date        : 05/01/2021
// Author      : Simon su
// Description : PLL based frequency synthesizer
// 		1. Initial version, only pin
// 		2. Add function, only behavioral model
// 		3. Add dynamic phase shifting feature
// 		4. Add dynamic output frequency switching feature
// Support feature:	1. Basic frequency synthesize	o
// 			2. Deskew mode			o
// 			3. fake calibration		o
// 			4. fake lock detector		o
// 			5. fractional divider in C0	o
// 			6. fractional divider in loop	o
// 			7. programmable duty cycle	o
// 			8. cascade in C6		o
// 			9. dynamic phase shifting	o
// 			10.dynamic freqency switching	o
// 			11.sscg				x
//==================================================================================
module PLL_TOP (
//fp config shift
input FP_CF_CLK,
input FP_CF_EN,
input FP_CF_IN,
input FP_CF_SEL,
input FP_CF_UP,

// pin define
output ATEST_PLL,
output CALIB_DONE,
output CALIB_FAIL,
output CKFBBAD,
output CKINBAD,
output CO0,
output CO1,
output CO2,
output CO3,
output CO4,
output CO5,
output CO6,
output DTEST_PLL,
output FBOUT,
output PLOCK,
output PSDONE,

input FBIN,
input FP_ADD_HALF,
input FP_DUTY_FORCE,
input FP_SW,
input PDB,
input PLLCK0,
input PLLCK1,
input PSCK,
input PSDIR,
input PSEN,
input RSTPLL,
input [2:0] PSSEL,

input [6:0] FP_DIV,
input [6:0] FP_DUTY,
input [2:0] FP_SEL,

output [3:0] BAND,

input CALIB_DYN_DIR,
input CALIB_DYN_UPDATE
//////////
);

wire IBN10U = 1'b1; //power controled by regs

parameter pll_sel = "auto";
parameter input_freq = "20";
parameter RST_SEL = 2'b00; //
parameter PDB_SEL = 2'b00; //
parameter FP_CFG_EN = 1'b0;//
parameter GATE_ENB = 1'b0; //

parameter SEL_FBPATH = 1'b0;
parameter CPSEL_FN = 6'b000000;
parameter CPSEL_CR = 4'b0000;
parameter CP_AUTO_ENB = 1'b0;
parameter BK = 4'b0000;
parameter CALIB_VREF = 2'b00;
parameter CKSEL = 1'b0;
parameter LPF = 3'b000;
parameter FBDLY = 7'b0000000;
parameter PFBSEL = 3'b000;
parameter VCO_INI_SEL = 1'b0;
parameter RST_REG_ENB = 1'b0;
parameter TRIM_CAP = 3'b000;
parameter EN_LOWSPD = 1'b0;
parameter VRSEL = 2'b00;
parameter CASCADE_EN = 1'b0;
parameter FRAC_C0 = 3'b000;
parameter FRAC_FB = 3'b000;
parameter MKEN0 = 1'b0;
parameter MKEN1 = 1'b0;
parameter MKEN2 = 1'b0;
parameter MKEN3 = 1'b0;
parameter MKEN4 = 1'b0;
parameter MKEN5 = 1'b0;
parameter MKEN6 = 1'b0;
parameter BPS0 = 1'b0;
parameter BPS1 = 1'b0;
parameter BPS2 = 1'b0;
parameter BPS3 = 1'b0;
parameter BPS4 = 1'b0;
parameter BPS5 = 1'b0;
parameter BPS6 = 1'b0;
parameter DIVC0 = 7'b0000000;
parameter DIVC1 = 7'b0000000;
parameter DIVC2 = 7'b0000000;
parameter DIVC3 = 7'b0000000;
parameter DIVC4 = 7'b0000000;
parameter DIVC5 = 7'b0000000;
parameter DIVC6 = 7'b0000000;
parameter GATE_BPS0 = 1'b0;
parameter GATE_BPS1 = 1'b0;
parameter GATE_BPS2 = 1'b0;
parameter GATE_BPS3 = 1'b0;
parameter GATE_BPS4 = 1'b0;
parameter GATE_BPS5 = 1'b0;
parameter GATE_BPS6 = 1'b0;
parameter P0SEL = 3'b000;
parameter P1SEL = 3'b000;
parameter P2SEL = 3'b000;
parameter P3SEL = 3'b000;
parameter P4SEL = 3'b000;
parameter P5SEL = 3'b000;
parameter P6SEL = 3'b000;
parameter CO0DLY = 7'b0000000;
parameter CO1DLY = 7'b0000000;
parameter CO2DLY = 7'b0000000;
parameter CO3DLY = 7'b0000000;
parameter CO4DLY = 7'b0000000;
parameter CO5DLY = 7'b0000000;
parameter CO6DLY = 7'b0000000;
parameter ADD_HALF0 = 1'b0;
parameter ADD_HALF1 = 1'b0;
parameter ADD_HALF2 = 1'b0;
parameter ADD_HALF3 = 1'b0;
parameter ADD_HALF4 = 1'b0;
parameter ADD_HALF5 = 1'b0;
parameter ADD_HALF6 = 1'b0;
parameter CO0DUTY = 7'b0000000;
parameter CO1DUTY = 7'b0000000;
parameter CO2DUTY = 7'b0000000;
parameter CO3DUTY = 7'b0000000;
parameter CO4DUTY = 7'b0000000;
parameter CO5DUTY = 7'b0000000;
parameter CO6DUTY = 7'b0000000;

parameter DIVM = 7'b0000000;
parameter DIVN = 7'b0000000;

parameter EN_HOLD = 1'b0;

parameter LD_HOLD = 1'b0;
parameter LD_CLKDIV = 1'b0;
parameter LD_EN_ACC = 1'b0;
parameter LD_RT_INRNG = 1'b0;
parameter LD_RT_OUTRNG = 1'b0;
parameter LD_EN = 1'b0;
parameter LD_MUX = 1'b0;

parameter CNT_SEL = 2'b00;
parameter FORCE_LOCK = 1'b0;
parameter INRNG_SEL = 3'b000;
parameter OUTRNG_SEL = 3'b000;

parameter CALIB_DIR = 1'b0;
parameter CALIB_EN = 1'b0;
parameter CALIB_MANUAL = 4'b0000;
parameter CALIB_CYCLE = 3'b000;
parameter CALIB_EN_DYN = 1'b0;
parameter CALIB_CLKDIV = 1'b0;
parameter CALIB_STB_TIME = 3'b000;
parameter CALIB_THRESHOLD = 3'b000;

parameter SSC_ACCSEL = 2'b00;
parameter SSC_EN = 1'b0;
parameter SSC_MODE = 1'b0;
parameter SSC_TRI_PEAK = 2'b00;
parameter DIVACC = 2'b00;
parameter DIVSSC = 7'b0000000;

parameter ATEST_SEL = 2'b00;
parameter DTEST_SEL = 2'b00;

// parameters definition
parameter MAX_CLKREF_FREQ	= 1066;		// refercence clock freq: 10MHZ--1066MHz
parameter MIN_CLKREF_FREQ	= 10;		//
parameter MAX_PFD_FREQ		= 550;		// PFD input freq: 10MHZ--550MHz
parameter MIN_PFD_FREQ		= 10;		//
parameter MAX_VCO_FREQ		= 1600;		// VCO operate freq: 600MHZ--1200MHz
parameter MIN_VCO_FREQ		= 600;		//
//parameter MAX_OUTPUT_FREQ	= 1066;		// output freq: 4.69MHZ--1066MHz 
//parameter MIN_OUTPUT_FREQ	= 4.69;		//
//parameter MIN_OUTPUT_C6_FREQ	= 4.69/128;	// minimam freq for C6 in cascade mode
parameter REG_DONE_TIME		= 300;		// setup time for internal regulator. default 3000. unit: ns
parameter CALIB_TOTAL_CYCLES	= 30;		// set to 30 for fast-sim. total clkpfd cycles in calibration mode. default is 1000 in circuit, about 10us when clkpfd_freq is 100MHz
parameter LOCK_NOACC_CYCLES	= 20;		// set to 20 for fast-sim. clkpfd cycles for lock detector when LD_EN_ACC is low. default is 10000 in circuit(minimum 5000), about 100us when clkpfd_freq is 100MHz.  
parameter CAL_REF_CYCLES	= 10;		// calculate refercence clock total cycles


//fp config
reg por;
initial begin
    por = 1'b0;
    #5;
    por = 1'b1;
end
reg [128:0] cfg_dyn_dff;
reg [128:0] cfg_dyn_lat;

always@(posedge FP_CF_CLK or negedge por)
begin
    if(~por)
        cfg_dyn_dff <= 0;
    else if(FP_CF_EN)
        cfg_dyn_dff <= {cfg_dyn_dff, FP_CF_IN};
end

//assign fp_cf_out = fp_cf_en & cfg_dyn_dff[79];

always@(posedge FP_CF_UP or negedge por)
begin
    if(~por)
        cfg_dyn_lat <= 0;
    else
        cfg_dyn_lat <= cfg_dyn_dff;
end

wire FP_ADD_HALF0      = cfg_dyn_lat[73];
wire FP_ADD_HALF1      = cfg_dyn_lat[81];
wire FP_ADD_HALF2      = cfg_dyn_lat[89];
wire FP_ADD_HALF3      = cfg_dyn_lat[97];
wire FP_ADD_HALF4      = cfg_dyn_lat[105];
wire FP_ADD_HALF5      = cfg_dyn_lat[113];
wire FP_ADD_HALF6      = cfg_dyn_lat[121];
wire [6:0] FP_CO0DUTY  = cfg_dyn_lat[80:74];
wire [6:0] FP_CO1DUTY  = cfg_dyn_lat[88:82];
wire [6:0] FP_CO2DUTY  = cfg_dyn_lat[96:90];
wire [6:0] FP_CO3DUTY  = cfg_dyn_lat[104:98];
wire [6:0] FP_CO4DUTY  = cfg_dyn_lat[112:106];
wire [6:0] FP_CO5DUTY  = cfg_dyn_lat[120:114];
wire [6:0] FP_CO6DUTY  = cfg_dyn_lat[128:122];
wire [6:0] FP_DIVC0    = cfg_dyn_lat[30:24];
wire [6:0] FP_DIVC1    = cfg_dyn_lat[37:31];
wire [6:0] FP_DIVC2    = cfg_dyn_lat[44:38];
wire [6:0] FP_DIVC3    = cfg_dyn_lat[51:45];
wire [6:0] FP_DIVC4    = cfg_dyn_lat[58:52];
wire [6:0] FP_DIVC5    = cfg_dyn_lat[65:59];
wire [6:0] FP_DIVC6    = cfg_dyn_lat[72:66];
wire [6:0] FP_DIVM     = cfg_dyn_lat[16:10];
wire [6:0] FP_DIVN     = cfg_dyn_lat[6:0];
wire FP_EN_LOWSPD      = cfg_dyn_lat[17];
wire [2:0] FP_FRAC_C0  = cfg_dyn_lat[23:21];
wire [2:0] FP_FRAC_FB  = cfg_dyn_lat[9:7];
wire [2:0] FP_TRIM_CAP = cfg_dyn_lat[20:18];

//weak pulldown for FP_CF_EN port
wire fp_cfg_en = FP_CFG_EN; //(FP_CF_EN === 1'b1 || FP_CF_EN === 1'b0) ? FP_CF_EN : 1'b0;
wire fp_cfg_sel = (FP_CF_SEL === 1'b1 || FP_CF_SEL === 1'b0) ? FP_CF_SEL : 1'b0;

wire CFG_ADD_HALF0      = (fp_cfg_en & fp_cfg_sel) ? FP_ADD_HALF0 : ADD_HALF0;
wire CFG_ADD_HALF1      = (fp_cfg_en & fp_cfg_sel) ? FP_ADD_HALF1 : ADD_HALF1;
wire CFG_ADD_HALF2      = (fp_cfg_en & fp_cfg_sel) ? FP_ADD_HALF2 : ADD_HALF2;
wire CFG_ADD_HALF3      = (fp_cfg_en & fp_cfg_sel) ? FP_ADD_HALF3 : ADD_HALF3;
wire CFG_ADD_HALF4      = (fp_cfg_en & fp_cfg_sel) ? FP_ADD_HALF4 : ADD_HALF4;
wire CFG_ADD_HALF5      = (fp_cfg_en & fp_cfg_sel) ? FP_ADD_HALF5 : ADD_HALF5;
wire CFG_ADD_HALF6      = (fp_cfg_en & fp_cfg_sel) ? FP_ADD_HALF6 : ADD_HALF6;
wire [6:0] CFG_CO0DUTY  = (fp_cfg_en & fp_cfg_sel) ? FP_CO0DUTY : CO0DUTY;
wire [6:0] CFG_CO1DUTY  = (fp_cfg_en & fp_cfg_sel) ? FP_CO1DUTY : CO1DUTY;
wire [6:0] CFG_CO2DUTY  = (fp_cfg_en & fp_cfg_sel) ? FP_CO2DUTY : CO2DUTY;
wire [6:0] CFG_CO3DUTY  = (fp_cfg_en & fp_cfg_sel) ? FP_CO3DUTY : CO3DUTY;
wire [6:0] CFG_CO4DUTY  = (fp_cfg_en & fp_cfg_sel) ? FP_CO4DUTY : CO4DUTY;
wire [6:0] CFG_CO5DUTY  = (fp_cfg_en & fp_cfg_sel) ? FP_CO5DUTY : CO5DUTY;
wire [6:0] CFG_CO6DUTY  = (fp_cfg_en & fp_cfg_sel) ? FP_CO6DUTY : CO6DUTY;
wire [6:0] CFG_DIVC0    = (fp_cfg_en & fp_cfg_sel) ? FP_DIVC0 : DIVC0;
wire [6:0] CFG_DIVC1    = (fp_cfg_en & fp_cfg_sel) ? FP_DIVC1 : DIVC1;
wire [6:0] CFG_DIVC2    = (fp_cfg_en & fp_cfg_sel) ? FP_DIVC2 : DIVC2;
wire [6:0] CFG_DIVC3    = (fp_cfg_en & fp_cfg_sel) ? FP_DIVC3 : DIVC3;
wire [6:0] CFG_DIVC4    = (fp_cfg_en & fp_cfg_sel) ? FP_DIVC4 : DIVC4;
wire [6:0] CFG_DIVC5    = (fp_cfg_en & fp_cfg_sel) ? FP_DIVC5 : DIVC5;
wire [6:0] CFG_DIVC6    = (fp_cfg_en & fp_cfg_sel) ? FP_DIVC6 : DIVC6;
wire [6:0] CFG_DIVM     = (fp_cfg_en & fp_cfg_sel) ? FP_DIVM : DIVM;
wire [6:0] CFG_DIVN     = (fp_cfg_en & fp_cfg_sel) ? FP_DIVN : DIVN;
wire CFG_EN_LOWSPD      = (fp_cfg_en & fp_cfg_sel) ? FP_EN_LOWSPD : EN_LOWSPD;
wire [2:0] CFG_FRAC_C0  = (fp_cfg_en & fp_cfg_sel) ? FP_FRAC_C0 : FRAC_C0;
wire [2:0] CFG_FRAC_FB  = (fp_cfg_en & fp_cfg_sel) ? FP_FRAC_FB : FRAC_FB;
wire [2:0] CFG_TRIM_CAP = (fp_cfg_en & fp_cfg_sel) ? FP_TRIM_CAP : TRIM_CAP;

// 
wire pllck_int;
wire ckfb;
wire ck_bps;

wire pd;
wire rb;
wire pd_reg_done;
wire rb_reg_done;
wire reg_done_buf;
wire pdbrb;

reg reg_done;
wire int_vdd_ana;
wire int_vdd_dig;

wire trim_start;
wire calib_doneb;
wire calib_on;
reg calib_done_buf;
reg [3:0] band_stored;
wire calib_update_clk;

wire lkd_en;
reg lock;
real lock_cycles_all;

reg ckvco0;
reg [5:0] vco_cnt;
reg ready_ana;
reg vcoready;
wire [5:0] vco_cnt_max;

integer counter;
integer start_flag_cnt;
wire    start_flag;
real stime1;
real clkref_period_int;
real clkref_period_sum;
real clkref_period;
real clkref_percentage;
real clkref_freq;
reg clkref_exist;
real prev_stime1;
real min_clkref_period;
real max_clkref_period;
reg  error_clkref_range;
integer divn_num;
real divm_num;
real fb_frac_num;
real clkpfd_period;
real min_clkpfd_period;
real max_clkpfd_period;
real clkpfd_freq;
reg  error_clkpfd_range;
real  clkvco_period;
real  clkvco_freq;
real  min_clkvco_period;
real  max_clkvco_period;
reg   error_clkvco_range;
wire freq_range_error;

// input clock mux and fb clock mux
assign pllck_int = (CKSEL)? PLLCK1 : PLLCK0 ;
assign ckfb = (SEL_FBPATH)? FBIN : FBOUT ;
assign ck_bps = pllck_int ;

// pdctl control
assign pd = PDB_SEL == 2'b11 ? !PDB : //controlled by input sig
            PDB_SEL == 2'b10 ? 1'b0 : //controlled by reg
            PDB_SEL == 2'b01 ? 1'b0 : //always on
                               1'b1 ; //always off
assign rb = RST_SEL == 2'b11 ? !RSTPLL :// controlled by input
            RST_SEL == 2'b10 ? 1'b0    :// controlled by reg
            RST_SEL == 2'b01 ? 1'b1    :// no-reset
                               1'b0    ;// reset

// internal regulator
wire pd_bgoff;
assign pd_bgoff = pd | !IBN10U ;

initial begin
  reg_done = 1'b0;
end

always @(negedge pd_bgoff ) begin
        reg_done <= #REG_DONE_TIME !pd_bgoff ;
end

always @(posedge pd_bgoff ) begin
        reg_done <= 1'b0 ;
end

assign int_vdd_ana = !pd_bgoff ;
assign int_vdd_dig = !pd_bgoff ;

assign reg_done_buf = RST_REG_ENB | reg_done ;
assign pd_reg_done = pd || (!reg_done_buf);//!(PDB & reg_done_buf) ;
assign rb_reg_done = rb || (!reg_done_buf);//!RSTPLL & reg_done_buf ;
assign pdbrb = (!pd) & rb;//PDB & !RSTPLL ;

// calibration
assign trim_start = CALIB_EN & ready_ana & clkref_exist ;

initial begin
  band_stored = 4'd8;
  calib_done_buf = 1'b0;
end

always @(negedge rb or posedge trim_start) begin
  if(!rb) 
        calib_done_buf <= 1'b0 ;
  else
        calib_done_buf <= #(CALIB_TOTAL_CYCLES*clkpfd_period) trim_start ;
end

assign CALIB_DONE = calib_done_buf | !CALIB_EN ;
assign calib_doneb = !CALIB_DONE ;
assign CALIB_FAIL = CALIB_DONE & freq_range_error ;
assign calib_on = CALIB_EN & !CALIB_DONE ;

assign calib_update_clk = CALIB_DYN_UPDATE & CALIB_EN_DYN ;

always @(posedge calib_update_clk or negedge rb) begin
  if(!rb)			band_stored <= 4'd8;
  else if(CALIB_DYN_DIR)	band_stored <= band_stored + 4'd1;
  else				band_stored <= band_stored - 4'd1;
end

assign BAND = (CALIB_EN)? band_stored : CALIB_MANUAL ;

// lock detector
assign lkd_en = LD_EN & CALIB_DONE & rb & !freq_range_error & clkref_exist;

initial begin
  lock = 1'b0;
  lock_cycles_all = (LD_EN_ACC)? LOCK_NOACC_CYCLES*3 : LOCK_NOACC_CYCLES ;
end

always @(posedge lkd_en) begin
  lock <= #(lock_cycles_all*clkpfd_period) lkd_en ;
end

always @(negedge lkd_en) begin
  lock <= 1'b0 ;
end

assign PLOCK = (FORCE_LOCK)? 1'b1 : lock ;

// calculate reference clock period as average of CAL_REF_CYCLES cycles
initial begin
  start_flag_cnt = 0;
end

always @(negedge pdbrb or posedge pllck_int ) begin
  if (!pdbrb)
	start_flag_cnt = 1;
  else if (pdbrb && (start_flag_cnt == 1))
	start_flag_cnt = 2;
  else if (IBN10U && (start_flag_cnt == 2))
	start_flag_cnt = 3;
end

assign start_flag = (start_flag_cnt == 3);

initial begin
  counter = 0;
  clkref_period = 1000;
  clkpfd_period = 1000;
  clkvco_period = 1;
  clkref_exist = 1'b0;
end

always @ (posedge pllck_int or negedge pdbrb) begin
  if (!pdbrb)
	counter = 0;
  else
	counter = counter+1;
end

always @(posedge pllck_int or negedge pdbrb) begin
  if (!pdbrb)
	clkref_period_int = 0;
  else begin
	stime1 = $realtime;
	clkref_period_int = (stime1 - prev_stime1);
	prev_stime1 = stime1;
  end
end

always @(posedge pllck_int or negedge pdbrb) begin
  if (!pdbrb)
	clkref_period_sum = 0;
  else if ((counter > 2) && (counter < CAL_REF_CYCLES+3 ))
	clkref_period_sum = clkref_period_sum + clkref_period_int;
end

always @(posedge pllck_int or negedge pdbrb) begin
  if (!pdbrb) begin
	clkref_period <= 1000;
	clkref_exist <= 1'b0;
  end 
  else if (counter == CAL_REF_CYCLES+3) begin
	clkref_period <= (clkref_period_sum/CAL_REF_CYCLES);
	clkref_exist <= 1'b1;
  end
end

always @(*) begin
	clkref_freq = (1/clkref_period) * 1000;	//assuming the clkref freq is in MHz
end

always @(*) begin
	clkref_percentage = (clkref_period/100); //assuming the input freq is in MHz
end

// check whether clkref freq is out of range
initial begin
  max_clkref_period = (1.0/MIN_CLKREF_FREQ) * 1000;	//assuming the clkref freq is in MHz
  min_clkref_period = (1.0/MAX_CLKREF_FREQ) * 1000;	//assuming the clkref freq is in MHz
end

always @ (*) begin
  if (((clkref_period >= min_clkref_period) && (clkref_period <= max_clkref_period)) || counter <=15 ) begin
	error_clkref_range = 1'b0;
  end 
  else begin
        error_clkref_range = 1'b1;
  end
end

// input divider value and loop divider value
initial begin
  divn_num = 1;
  divm_num = 12;
  fb_frac_num = 0.0;
end

always @ (*) begin
  case (CFG_FRAC_FB)
    3'b000:	fb_frac_num = 0.000 ;
    3'b001:	fb_frac_num = 0.125 ;
    3'b010:	fb_frac_num = 0.250 ;
    3'b011:	fb_frac_num = 0.375 ;
    3'b100:	fb_frac_num = 0.500 ;
    3'b101:	fb_frac_num = 0.625 ;
    3'b110:	fb_frac_num = 0.750 ;
    3'b111:	fb_frac_num = 0.875 ;
  endcase
	divn_num = CFG_DIVN + 1 ;
	divm_num = CFG_DIVM + 1 + fb_frac_num ;
end

// check whether clkpfd freq is out of range
always @ (*) begin
  clkpfd_period = clkref_period * divn_num;
end

always @(*) begin
  clkpfd_freq = (1/clkpfd_period) * 1000;	//assuming the clkpfd freq is in MHz
end

initial begin
  max_clkpfd_period = (1.0/MIN_PFD_FREQ) * 1000;	//assuming the clkpfd freq is in MHz
  min_clkpfd_period = (1.0/MAX_PFD_FREQ) * 1000;	//assuming the clkpfd freq is in MHz
end

always @ (*) begin
  if (((clkpfd_period >= min_clkpfd_period) && (clkpfd_period <= max_clkpfd_period)) || counter <=15 ) begin
	error_clkpfd_range = 1'b0;
  end 
  else begin
        error_clkpfd_range = 1'b1;
  end
end

// check whether clkvco freq is out of range
always @ (*) begin
  clkvco_period = clkpfd_period / divm_num;
end

always @(*) begin
  clkvco_freq = (1/clkvco_period) * 1000;	//assuming the clkvco freq is in MHz
end

initial begin
  max_clkvco_period = (1.0/MIN_VCO_FREQ) * 1000;	//assuming the vco freq is in MHz
  min_clkvco_period = (1.0/MAX_VCO_FREQ) * 1000;	//assuming the vco freq is in MHz
end

always @ (*) begin
  if (( clkvco_period >= min_clkvco_period) && (clkvco_period <= max_clkvco_period) || counter <=15  ) begin
	error_clkvco_range = 1'b0;
  end
  else begin
	error_clkvco_range = 1'b1;
  end
end

// display the error information
assign freq_range_error = (error_clkref_range || error_clkpfd_range || error_clkvco_range);

always @ (posedge freq_range_error ) begin
  if (start_flag) begin
	$display (" ERROR : PLL won't work!");
	if (error_clkref_range) begin
		$display (" ERROR : error_clkref_range");
		$display (" ERROR : clkref_freq %dMHz - min_clkref_freq = %dMHz - max_clkref_freq = %dMHz", clkref_freq, MIN_CLKREF_FREQ, MAX_CLKREF_FREQ);
        end
        if (error_clkpfd_range) begin
		$display (" ERROR : error_clkpfd_range");
		$display (" ERROR : clkpfd_freq %dMHz - min_clkpfd_freq = %dMHz - max_clkpfd_freq = %dMHz", clkpfd_freq, MIN_PFD_FREQ, MAX_PFD_FREQ);
        end
        if (error_clkvco_range) begin
		$display (" ERROR : error_clkvco_range");
		$display (" ERROR : clkvco_freq %dMHz - min_clkvco_freq = %dMHz - max_clkvco_freq = %dMHz", clkvco_freq, MIN_VCO_FREQ, MAX_VCO_FREQ);
        end
  end
end

// vco outout 0
assign vco_cnt_max = (VCO_INI_SEL)? 6'd16 : 6'd32 ;
initial begin
  ckvco0 = 1'b0;
  ready_ana = 1'b0;
  vco_cnt = 6'd0;
  vcoready = 1'b0;
end

always @(posedge pd or posedge reg_done) begin
  if(pd) begin
	ckvco0 <= 1'b0;
  end 
  else begin
	forever begin
		ckvco0 <= ~ckvco0 & ~pd;
		#(clkvco_period/2);
	end
  end
end

always @(posedge ckvco0 or negedge rb_reg_done) begin
  if(!rb_reg_done) begin
	ready_ana <= 1'b0;
	vco_cnt <= 6'd0;
  end
  else if(vco_cnt == vco_cnt_max) begin
	vco_cnt <= vco_cnt_max;
	ready_ana <= 1'b1;
  end 
  else
	vco_cnt <= vco_cnt + 1'b1;
end

always @(posedge ckvco0 or negedge rb_reg_done) begin
  if(!rb_reg_done)
	vcoready <= 1'b0;
  else if(CALIB_DONE)
	vcoready <= 1'b1;
end

assign ATEST_PLL = (ATEST_SEL == 2'b00)? 1'b0 : 
		   (ATEST_SEL == 2'b01)? int_vdd_ana :
	           (ATEST_SEL == 2'b10)? int_vdd_dig : 
				         int_vdd_ana ;

assign DTEST_PLL = (DTEST_SEL == 2'b00)? 1'b0 : 
		   (DTEST_SEL == 2'b01)? lock :
	           (DTEST_SEL == 2'b10)? vcoready : 
				         calib_on ;

// dynamic phase shifting feature
reg [6:0] ps_ck_ch;
reg [6:0] ps_en_ch;
reg ps_dir_ch;
reg psdone_tmp;

initial begin
  ps_en_ch = 7'd0 ;
  ps_ck_ch = 7'd0 ;
  ps_dir_ch = 1'b0 ;
  psdone_tmp = 1'b0 ;
end

always @(negedge PSCK or negedge rb_reg_done) begin
  if(!rb_reg_done) begin
        ps_en_ch <= 7'd0;
  end
  else if(PSEN && (PSSEL == 3'd1)) ps_en_ch <= 7'b0000001 ;
  else if(PSEN && (PSSEL == 3'd2)) ps_en_ch <= 7'b0000010 ;
  else if(PSEN && (PSSEL == 3'd3)) ps_en_ch <= 7'b0000100 ;
  else if(PSEN && (PSSEL == 3'd4)) ps_en_ch <= 7'b0001000 ;
  else if(PSEN && (PSSEL == 3'd5)) ps_en_ch <= 7'b0010000 ;
  else if(PSEN && (PSSEL == 3'd6)) ps_en_ch <= 7'b0100000 ;
  else if(PSEN && (PSSEL == 3'd7)) ps_en_ch <= 7'b1000000 ;
  else	ps_en_ch <= 7'd0;
end

always @(negedge PSCK or negedge rb_reg_done) begin
  if(!rb_reg_done) begin
        ps_en_ch <= 7'd0;
  end
  else ps_dir_ch <= PSDIR;
end

always @(posedge PSCK or negedge rb_reg_done) begin
  if(!rb_reg_done) begin
	ps_ck_ch <= 7'd0;
  end 
  else if(ps_en_ch != 7'd0) begin
	psdone_tmp = 1'b1;
	ps_ck_ch <= ps_en_ch;
  end
  else begin
	psdone_tmp = 1'b0;
	ps_ck_ch <= ps_en_ch;
  end
end

assign PSDONE = psdone_tmp;

// dynamic output freqeuncy switching
wire [6:0] fp_sel_ch ;
wire [6:0] fp_sw_ch ;
integer fp_div_num ;
integer fp_duty_num ;
real fp_high_width ;

assign fp_sel_ch = (FP_SEL == 3'd1)?	7'b0000001 : 
                   (FP_SEL == 3'd2)?    7'b0000010 :
                   (FP_SEL == 3'd3)?    7'b0000100 :
                   (FP_SEL == 3'd4)?    7'b0001000 :
                   (FP_SEL == 3'd5)?    7'b0010000 :
                   (FP_SEL == 3'd6)?    7'b0100000 :
                   (FP_SEL == 3'd7)?    7'b1000000 :
					7'b0000000 ;

assign fp_sw_ch = {7{PLOCK}} & fp_sel_ch & {7{FP_SW}} ; 

always @(*) begin
  fp_div_num = FP_DIV + 1 ;
  fp_duty_num = FP_DUTY ;
  fp_high_width = (FP_DUTY_FORCE==1'b1 || fp_div_num==1)? clkvco_period* fp_div_num/2 : clkvco_period * (fp_duty_num + ((FP_ADD_HALF)? 0.5 : 0 )) ;
end

// output CFB assignment
wire cfb_en;
wire cfb_clk_core;
reg cfb_clk_core_50;
real cfb_phase;
integer cfb_dly_num;
real cfb_ph_num;
integer cfb_div_num;
real cfb_frac_num;

assign cfb_en = ready_ana ;

initial begin
  cfb_phase = 0 ;
  cfb_clk_core_50 = 1'b0 ;
end

always @ (*) begin
  case (PFBSEL)
    3'b000:	cfb_ph_num = 0.000 ;
    3'b001:	cfb_ph_num = 0.125 ;
    3'b010:	cfb_ph_num = 0.250 ;
    3'b011:	cfb_ph_num = 0.375 ;
    3'b100:	cfb_ph_num = 0.500 ;
    3'b101:	cfb_ph_num = 0.625 ;
    3'b110:	cfb_ph_num = 0.750 ;
    3'b111:	cfb_ph_num = 0.875 ;
  endcase
  case (CFG_FRAC_FB)
    3'b000:	cfb_frac_num = 0.000 ;
    3'b001:	cfb_frac_num = 0.125 ;
    3'b010:	cfb_frac_num = 0.250 ;
    3'b011:	cfb_frac_num = 0.375 ;
    3'b100:	cfb_frac_num = 0.500 ;
    3'b101:	cfb_frac_num = 0.625 ;
    3'b110:	cfb_frac_num = 0.750 ;
    3'b111:	cfb_frac_num = 0.875 ;
  endcase
	cfb_dly_num = FBDLY ;
	cfb_phase = clkvco_period * (cfb_dly_num + cfb_ph_num) ;
	cfb_div_num = CFG_DIVM + 1 ;
end

always @(posedge cfb_en or negedge rb_reg_done ) begin
  if(!rb_reg_done)
	cfb_clk_core_50 <= 1'b0 ;
  else begin
	#(cfb_phase) ;
	if (CFG_DIVM == 7'd0 && CFG_FRAC_FB != 3'd0) $display ("WARNING : Loop freq is wrong! Minimum fractional divider value is 2.000!");
	else
	forever begin
		cfb_clk_core_50 <= !cfb_clk_core_50 ;
		#((clkvco_period * (cfb_div_num + cfb_frac_num))/2) ;
	end
  end
end

assign cfb_clk_core = cfb_clk_core_50 ;

assign FBOUT = cfb_clk_core ; 

// output C0 assignment
PLL_DIV_OUT_CH0 u_PLL_CH0
(
  .RB		(rb_reg_done	),
  .MKEN		(MKEN0	    	),
  .VCOREADY	(vcoready   	),
  .LOCK		(PLOCK	    	),
  .CK_BPS	(ck_bps	    	),
  .BPS		(BPS0	    	),
  .PHASE_INI	(P0SEL	    	),
  .FRAC		(CFG_FRAC_C0    	),
  .DLYNUM	(CO0DLY	    	),
  .DIVNUM	(CFG_DIVC0		),
  .DUTYNUM	(CFG_CO0DUTY	),
  .ADD_HALF	(CFG_ADD_HALF0	),
  .GATE_ENB	(GATE_ENB	),
  .GATE_BPS	(GATE_BPS0	),
  .PSCK		(ps_ck_ch[0]	),
  .PSDIR	(ps_dir_ch	),
  .FP_SW	(fp_sw_ch[0]	),
  .CASCADE_EN	(1'b0		),
  .CLKOUT	(CO0		)
);

// output C1 assignment
PLL_DIV_OUT_CH0 u_PLL_CH1
(
  .RB		(rb_reg_done	),
  .MKEN		(MKEN1	    	),
  .VCOREADY	(vcoready   	),
  .LOCK		(PLOCK	    	),
  .CK_BPS	(ck_bps	    	),
  .BPS		(BPS1	    	),
  .PHASE_INI	(P1SEL	    	),
  .FRAC		(3'd0    	),
  .DLYNUM	(CO1DLY	    	),
  .DIVNUM	(CFG_DIVC1		),
  .DUTYNUM	(CFG_CO1DUTY	),
  .ADD_HALF	(CFG_ADD_HALF1	),
  .GATE_ENB	(GATE_ENB	),
  .GATE_BPS	(GATE_BPS1	),
  .PSCK		(ps_ck_ch[1]	),
  .PSDIR	(ps_dir_ch	),
  .FP_SW	(fp_sw_ch[1]	),
  .CASCADE_EN	(1'b0		),
  .CLKOUT	(CO1		)
);

// output C2 assignment
PLL_DIV_OUT_CH0 u_PLL_CH2
(
  .RB		(rb_reg_done	),
  .MKEN		(MKEN2	    	),
  .VCOREADY	(vcoready   	),
  .LOCK		(PLOCK	    	),
  .CK_BPS	(ck_bps	    	),
  .BPS		(BPS2	    	),
  .PHASE_INI	(P2SEL	    	),
  .FRAC		(3'd0    	),
  .DLYNUM	(CO2DLY	    	),
  .DIVNUM	(CFG_DIVC2		),
  .DUTYNUM	(CFG_CO2DUTY	),
  .ADD_HALF	(CFG_ADD_HALF2	),
  .GATE_ENB	(GATE_ENB	),
  .GATE_BPS	(GATE_BPS2	),
  .PSCK		(ps_ck_ch[2]	),
  .PSDIR	(ps_dir_ch	),
  .FP_SW	(fp_sw_ch[2]	),
  .CASCADE_EN	(1'b0		),
  .CLKOUT	(CO2		)
);

// output C3 assignment
PLL_DIV_OUT_CH0 u_PLL_CH3
(
  .RB		(rb_reg_done	),
  .MKEN		(MKEN3	    	),
  .VCOREADY	(vcoready   	),
  .LOCK		(PLOCK	    	),
  .CK_BPS	(ck_bps	    	),
  .BPS		(BPS3	    	),
  .PHASE_INI	(P3SEL	    	),
  .FRAC		(3'd0    	),
  .DLYNUM	(CO3DLY	    	),
  .DIVNUM	(CFG_DIVC3		),
  .DUTYNUM	(CFG_CO3DUTY	),
  .ADD_HALF	(CFG_ADD_HALF3	),
  .GATE_ENB	(GATE_ENB	),
  .GATE_BPS	(GATE_BPS3	),
  .PSCK		(ps_ck_ch[3]	),
  .PSDIR	(ps_dir_ch	),
  .FP_SW	(fp_sw_ch[3]	),
  .CASCADE_EN	(1'b0		),
  .CLKOUT	(CO3		)
);

// output C4 assignment
PLL_DIV_OUT_CH0 u_PLL_CH4
(
  .RB		(rb_reg_done	),
  .MKEN		(MKEN4	    	),
  .VCOREADY	(vcoready   	),
  .LOCK		(PLOCK	    	),
  .CK_BPS	(ck_bps	    	),
  .BPS		(BPS4	    	),
  .PHASE_INI	(P4SEL	    	),
  .FRAC		(3'd0    	),
  .DLYNUM	(CO4DLY	    	),
  .DIVNUM	(CFG_DIVC4		),
  .DUTYNUM	(CFG_CO4DUTY	),
  .ADD_HALF	(CFG_ADD_HALF4	),
  .GATE_ENB	(GATE_ENB	),
  .GATE_BPS	(GATE_BPS4	),
  .PSCK		(ps_ck_ch[4]	),
  .PSDIR	(ps_dir_ch	),
  .FP_SW	(fp_sw_ch[4]	),
  .CASCADE_EN	(1'b0		),
  .CLKOUT	(CO4		)
);

// output C5 assignment
PLL_DIV_OUT_CH0 u_PLL_CH5
(
  .RB		(rb_reg_done	),
  .MKEN		(MKEN5	    	),
  .VCOREADY	(vcoready   	),
  .LOCK		(PLOCK	    	),
  .CK_BPS	(ck_bps	    	),
  .BPS		(BPS5	    	),
  .PHASE_INI	(P5SEL	    	),
  .FRAC		(3'd0    	),
  .DLYNUM	(CO5DLY	    	),
  .DIVNUM	(CFG_DIVC5		),
  .DUTYNUM	(CFG_CO5DUTY	),
  .ADD_HALF	(CFG_ADD_HALF5	),
  .GATE_ENB	(GATE_ENB	),
  .GATE_BPS	(GATE_BPS5	),
  .PSCK		(ps_ck_ch[5]	),
  .PSDIR	(ps_dir_ch	),
  .FP_SW	(fp_sw_ch[5]	),
  .CASCADE_EN	(1'b0		),
  .CLKOUT	(CO5		)
);

// output C6 assignment
PLL_DIV_OUT_CH0 u_PLL_CH6
(
  .RB		(rb_reg_done	),
  .MKEN		(MKEN6	    	),
  .VCOREADY	(vcoready   	),
  .LOCK		(PLOCK	    	),
  .CK_BPS	(ck_bps	    	),
  .BPS		(BPS6	    	),
  .PHASE_INI	(P6SEL	    	),
  .FRAC		(3'd0    	),
  .DLYNUM	(CO6DLY	    	),
  .DIVNUM	(CFG_DIVC6		),
  .DUTYNUM	(CFG_CO6DUTY	),
  .ADD_HALF	(CFG_ADD_HALF6	),
  .GATE_ENB	(GATE_ENB	),
  .GATE_BPS	(GATE_BPS6	),
  .PSCK		(ps_ck_ch[6]	),
  .PSDIR	(ps_dir_ch	),
  .FP_SW	(fp_sw_ch[6]	),
  .CASCADE_EN	(CASCADE_EN	),
  .CLKOUT	(CO6		)
);


endmodule

//==================================================================================
module PLL_DIV_OUT_CH0 (
input RB,
input MKEN,
input VCOREADY,
input LOCK,
input CK_BPS,
input BPS,
input [2:0] PHASE_INI,
input [2:0] FRAC,
input [6:0] DLYNUM,
input [6:0] DIVNUM,
input [6:0] DUTYNUM,
input ADD_HALF,
input GATE_ENB,
input GATE_BPS,
input PSCK,
input PSDIR,
//input PSEN,
//input PSSEL,
input FP_SW,
//input [6:0] FP_DIVNUM,
//input [6:0] FP_DUTYNUM,
//input FP_ADD_HALF,
input CASCADE_EN,
//output PSDONE,
output CLKOUT
);

wire c0_en;
reg c0_start;
reg c0_clk_byn;
wire c0_clk_core;
reg c0_clk_core_50;
reg c0_clk_core_50tmp;
reg c0_clk_core_50d;
wire c0_clk_gate;
reg c0_clk_gate_en;
real c0_phase;
integer c0_dly_num;
real c0_ph_num;
real c0_frac_num;
integer c0_div_num;
real c0_high_width;
integer c0_duty_num;
real c0_ps_step;
reg c0_psck_sync_fall;
reg c0_psck_sync_rise;
wire c0_psck_sync;
integer c0_div_num_cfg;
integer c0_duty_num_cfg;
real c0_high_width_cfg;
reg c0_sw_sync;

assign c0_en = VCOREADY & MKEN ;

initial begin
  c0_start = 0 ;
  c0_phase = 0 ;
  c0_high_width = 0 ;
  c0_clk_byn = 1'b0 ;
  c0_clk_core_50 = 1'b0 ;
  c0_clk_core_50tmp = 1'b0 ;
  c0_clk_core_50d = 1'b0 ;
  c0_clk_gate_en = 1'b0 ;
  c0_ps_step = 0;
  c0_psck_sync_fall = 1'b0 ;
  c0_psck_sync_rise = 1'b0 ;
  c0_sw_sync = 1'b0 ;
end

wire CASCADE_EN_buf;
assign CASCADE_EN_buf = (CASCADE_EN===1'bx)? 1'b0 : CASCADE_EN ;

always @ (*) begin
  case (PHASE_INI)
    3'b000:     c0_ph_num = 0.000 ;
    3'b001:     c0_ph_num = 0.125 ;
    3'b010:     c0_ph_num = 0.250 ;
    3'b011:     c0_ph_num = 0.375 ;
    3'b100:     c0_ph_num = 0.500 ;
    3'b101:     c0_ph_num = 0.625 ;
    3'b110:     c0_ph_num = 0.750 ;
    3'b111:     c0_ph_num = 0.875 ;
  endcase
  case (FRAC)
    3'b000:     c0_frac_num = 0.000 ;
    3'b001:     c0_frac_num = 0.125 ;
    3'b010:     c0_frac_num = 0.250 ;
    3'b011:     c0_frac_num = 0.375 ;
    3'b100:     c0_frac_num = 0.500 ;
    3'b101:     c0_frac_num = 0.625 ;
    3'b110:     c0_frac_num = 0.750 ;
    3'b111:     c0_frac_num = 0.875 ;
  endcase
end

always @ (*) begin
        c0_dly_num = DLYNUM ;
        //c0_phase = (CASCADE_EN_buf)? PLL_TOP.u_PLL_CH5.c0_phase + PLL_TOP.clkvco_period * PLL_TOP.u_PLL_CH5.c0_div_num_cfg * c0_dly_num : PLL_TOP.clkvco_period * (c0_dly_num + c0_ph_num) ;
        c0_phase = (CASCADE_EN_buf)? 10 + PLL_TOP.clkvco_period * PLL_TOP.u_PLL_CH5.c0_div_num_cfg * c0_dly_num : PLL_TOP.clkvco_period * (c0_dly_num + c0_ph_num) ;
        c0_div_num_cfg = (CASCADE_EN_buf)? PLL_TOP.u_PLL_CH5.c0_div_num_cfg * (DIVNUM + 1) : DIVNUM + 1 ;
        c0_duty_num_cfg = DUTYNUM ;
        c0_high_width_cfg = (CASCADE_EN_buf)? PLL_TOP.clkvco_period * PLL_TOP.u_PLL_CH5.c0_div_num_cfg * (c0_duty_num_cfg + ((ADD_HALF)? 0.5 : 0 )) : (c0_div_num_cfg == 1)? PLL_TOP.clkvco_period*c0_div_num_cfg/2 : PLL_TOP.clkvco_period * (c0_duty_num_cfg + ((ADD_HALF)? 0.5 : 0 )) ;
        c0_div_num = c0_div_num_cfg;
        c0_duty_num = c0_duty_num_cfg ;
        c0_high_width = c0_high_width_cfg ;
end

always @(c0_en or negedge RB) begin
  if(!RB)
	c0_start <= 1'b0;
  else 
	c0_start <= #(c0_phase) c0_en;
end

always @(posedge c0_clk_byn or negedge RB) begin
  if(!RB)
        c0_sw_sync <= 1'b0;
  else if(FP_SW===1'bx)
        c0_sw_sync <= 1'b0;
  else
        c0_sw_sync <= FP_SW;
end

always @(posedge c0_sw_sync or negedge RB) begin
  if(!RB) begin
        c0_div_num = c0_div_num_cfg;
        c0_duty_num = c0_duty_num_cfg ;
        c0_high_width = c0_high_width_cfg ;
  end
  else begin
        c0_div_num = PLL_TOP.fp_div_num;
        c0_duty_num = PLL_TOP.fp_duty_num ;
        c0_high_width = PLL_TOP.fp_high_width ;
  end
end

always @(posedge c0_start or c0_clk_core_50tmp or negedge RB ) begin
  if(!RB)
        c0_clk_core_50 <= 1'b0 ;
  else if(RB===1'bx)
        c0_clk_core_50 <= 1'b0 ;
  else
        c0_clk_core_50 <= !c0_clk_core_50tmp ;
end

always @(c0_clk_core_50) begin
  if (DIVNUM == 7'd0 && FRAC != 3'd0) $display ("WARNING : Channel0 freq is wrong! Minimum fractional divider value is 2.000!");
  else
  c0_clk_core_50tmp <= #((PLL_TOP.clkvco_period * (c0_div_num + c0_frac_num))/2 + c0_ps_step) c0_clk_core_50 ;
end

always @(posedge c0_clk_core_50 or negedge RB ) begin
  if(!RB) begin
        c0_psck_sync_rise <= 1'b0;
  end
  else begin
        c0_psck_sync_rise <= PSCK;
  end
end

always @(negedge c0_clk_core_50 or negedge RB ) begin
  if(!RB) begin
        c0_psck_sync_fall <= 1'b0;
  end
  else begin
        c0_psck_sync_fall <= c0_psck_sync_rise;
  end
end

assign c0_psck_sync = c0_psck_sync_rise & !c0_psck_sync_fall ;

always @(posedge c0_psck_sync) begin
  c0_ps_step = (PSDIR)? 1*PLL_TOP.clkvco_period/64 : -1*PLL_TOP.clkvco_period/64 ;
end

always @(negedge c0_psck_sync) begin
  c0_ps_step = 0 ;
end

always @( * ) begin
  c0_clk_core_50d <= #(c0_high_width) c0_clk_core_50 ;
end

always @(posedge c0_clk_core_50) begin
        c0_clk_byn <= 1'b1;
end

always @(posedge c0_clk_core_50d) begin
        c0_clk_byn <= 1'b0;
end

assign c0_clk_core = (FRAC != 3'd0)? c0_clk_core_50 : c0_clk_byn ;

//always @( negedge c0_clk_core ) begin
//  if( !GATE_ENB && !GATE_BPS && !LOCK)
//        c0_clk_gate_en <= 1'b0 ;
//  else
//        c0_clk_gate_en <= 1'b1 ;
//end

always @( negedge c0_clk_core or negedge RB) begin
  if(!RB)
        c0_clk_gate_en <= 1'b0 ;
  else
        c0_clk_gate_en <= GATE_ENB || GATE_BPS || LOCK ;
end

assign c0_clk_gate = c0_clk_core & c0_clk_gate_en ;
assign CLKOUT = (BPS)? CK_BPS : c0_clk_gate ;

endmodule

module P0_EMB9K (doa, dob, dopa, dopb, 
             addra, addrb, clka, clkb, dia, dib, dipa, dipb, cea, ceb, regcea, regceb, 
             regsra, regsrb, wea, web);

    output [15:0] doa;
    output [15:0] dob;
    output [1:0] dopa;
    output [1:0] dopb;
    
    input cea, clka;
    input ceb, clkb;
    input regsra, regcea;
    input regsrb, regceb;

    input [12:0] addra;
    input [12:0] addrb;
    input [15:0] dia;
    input [15:0] dib;
    input [1:0] dipa;
    input [1:0] dipb;
    input [1:0] wea;
    input [1:0] web;

    parameter outreg_a = 0;
    parameter outreg_b = 0;
    parameter byte_write_enable = 0;
    parameter use_parity = 0;
    parameter width_a = 1;
    parameter width_b = 1;

    parameter clka_inv = 1'b0;
    parameter clkb_inv = 1'b0;
    parameter cea_inv  = 1'b0;
    parameter ceb_inv = 1'b0;
    parameter wea_inv = 1'b0;
    parameter web_inv = 1'b0;
    parameter regsra_inv = 1'b0;
    parameter regsrb_inv = 1'b0;
    parameter regcea_inv = 1'b0;
    parameter regceb_inv = 1'b0;
    
    parameter writemode_a = "write_first";
    parameter writemode_b = "write_first";
    parameter rammode = "sdp";  // "tdp", "sdp", "sp"
    parameter extension_mode = "power"; //"area";
    parameter reset_value = 18'b0;
    
    parameter init_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_02 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_03 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_04 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_05 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_06 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_07 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_08 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_09 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_0a = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_0b = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_0c = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_0d = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_0e = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter init_0f = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter initp_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    parameter initp_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;

    parameter init_file = "none";
    parameter is_initialized = 0;

    reg finish_error = 0;
    
    initial begin
        if(width_a != 1 && width_a != 2 && width_a != 4 && width_a != 9 && width_a != 18) begin
            $display("Error: The attribute width_a on EMB9K instance %m is set to %d, legal value for attribute width_a are 1, 2, 4, 9, 18.", width_a);
            finish_error = 1;
        end
        if(width_b != 1 && width_b != 2 && width_b != 4 && width_b != 9 && width_b != 18) begin
            $display("Error: The attribute width_b on EMB9K instance %m is set to %d, legal value for attribute width_b are 1, 2, 4, 9, 18.", width_b);
            finish_error = 1;
        end

        if (finish_error == 1)
            $finish;
    end // end initial
    
    parameter width_a_9k = width_a; 
    parameter width_b_9k = width_b;

    localparam [3:0] modea_sel = (width_a_9k == 1)  ? 4'b1111 : 
                                 (width_a_9k == 2)  ? 4'b1110 :
                                 (width_a_9k == 4)  ? 4'b1100 :
                                 (width_a_9k == 9)  ? 4'b1000 :
                                 (width_a_9k == 18) ? 4'b0000 : 4'b0000;
    localparam [3:0] modeb_sel = (width_b_9k == 1)  ? 4'b1111 : 
                                 (width_b_9k == 2)  ? 4'b1110 :
                                 (width_b_9k == 4)  ? 4'b1100 :
                                 (width_b_9k == 9)  ? 4'b1000 :
                                 (width_b_9k == 18) ? 4'b0000 : 4'b0000;
                           
    localparam [1:0] modea_wr = (writemode_a == "write_first") ? 2'b01 :
                                (writemode_a == "read_first")  ? 2'b10 :
                                (writemode_a == "no_change")   ? 2'b00 : 2'b00;
                          
    localparam [1:0] modeb_wr = (writemode_b == "write_first") ? 2'b01 :
                                (writemode_b == "read_first")  ? 2'b10 :
                                (writemode_b == "no_change")   ? 2'b00 : 2'b00;

     wire [17:0] dataina_bnk1 = (width_a_9k == 18) ? {dia[15:8], dipa[1:0], dia[7:0]}:
                                (width_a_9k == 9)  ? {dia[7:0], dipa[0], dipa[0], dia[7:0]} :
                                (width_a_9k == 4)  ? {{2{dia[3:0]}}, 2'bxx, {2{dia[3:0]}}} :
                                (width_a_9k == 2)  ? {{4{dia[1:0]}}, 2'bxx, {4{dia[1:0]}}} :
                                (width_a_9k == 1)  ? {{8{dia[0]}}, 2'bxx, {8{dia[0]}}} : 
                                                     18'bxxxxxxxxxxxxxxxxxx;
     wire [17:0] datainb_bnk1 = (width_b_9k == 18) ? {dib[15:8], dipb[1:0], dib[7:0]}:
                                (width_b_9k == 9)  ? {dib[7:0], dipb[0], dipb[0], dib[7:0]} :
                                (width_b_9k == 4)  ? {{2{dib[3:0]}}, 2'bxx, {2{dib[3:0]}}} :
                                (width_b_9k == 2)  ? {{4{dib[1:0]}}, 2'bxx, {4{dib[1:0]}}} :
                                (width_b_9k == 1)  ? {{8{dib[0]}}, 2'bxx, {8{dib[0]}}} : 
                                                     18'bxxxxxxxxxxxxxxxxxx;

    wire [17:0] dataouta_bnk1;
    wire [17:0] dataoutb_bnk1;
    
    wire [1:0] regen = (rammode == "tdp") ? {regceb, regcea} : {regcea, regcea};
    
    integer i;

    assign dopa[1:0] = dataouta_bnk1[9:8];
    assign dopb[1:0] = dataoutb_bnk1[9:8];
    assign doa = {dataouta_bnk1[17:10], dataouta_bnk1[7:0]};
    assign dob = {dataoutb_bnk1[17:10], dataoutb_bnk1[7:0]};

    // address translation
    localparam addra_lbit = (width_a_9k == 1) ? 0 : 
                            (width_a_9k == 2) ? 1 : 
                            (width_a_9k == 4) ? 2 : 
                            (width_a_9k == 9) ? 3 :
                            (width_a_9k == 18) ? 4: 0;
                       
    localparam addrb_lbit = (width_b_9k == 1) ? 0 : 
                            (width_b_9k == 2) ? 1 : 
                            (width_b_9k == 4) ? 2 : 
                            (width_b_9k == 9) ? 3 : 
                            (width_b_9k == 18) ? 4:  0;
    //translation
    wire [12:0] addra_input;
    wire [12:0] addrb_input;
    
    localparam  addr_width_a = (width_a_9k == 1) ? 13 :
                               (width_a_9k == 2) ? 12 :
                               (width_a_9k == 4) ? 11 : 
                               (width_a_9k == 9) ? 10 : 
                               (width_a_9k == 18) ? 9 : 1 ;
    localparam  addr_width_b = (width_b_9k == 1) ? 13 :
                               (width_b_9k == 2) ? 12 :
                               (width_b_9k == 4) ? 11 :
                               (width_b_9k == 9) ? 10 :
                               (width_b_9k == 18) ? 9 : 1 ;
                               
    assign addra_input[addra_lbit + addr_width_a - 1 : addra_lbit] = addra;    
    if (addra_lbit > 0)
        assign addra_input[addra_lbit - 1 : 0] = 0;
    if (addra_lbit + addr_width_a < 13)
        assign addra_input[12 : addra_lbit + addr_width_a] = 0;
        
    assign addrb_input[addrb_lbit + addr_width_b - 1 : addrb_lbit] = addrb;
    if (addrb_lbit > 0)
        assign addrb_input[addrb_lbit - 1 : 0] = 0;
    if (addrb_lbit + addr_width_b < 13)
        assign addrb_input[12 : addrb_lbit + addr_width_b] = 0; 
 
    localparam portb_ce = rammode == "sp" ? 1'b0 : 1'b1;

BRAM9K #(
      .MODEA_SEL(modea_sel),
      .MODEB_SEL(modeb_sel),
      .PORTA_PROG(8'b11110000),
      .PORTB_PROG(8'b11110000),
      .PORTA_WR_MODE(modea_wr),
      .PORTB_WR_MODE(modeb_wr),
      .PORTA_CE(1'b1),
      .PORTB_CE(portb_ce),
      .PORTA_REG_OUT(outreg_a),
      .PORTB_REG_OUT(outreg_b),

      .HASCLKA(1'b1),
      .HASCLKB(1'b1),

      .PORTA_CLKINV({clka_inv}),
      .PORTB_CLKINV({clkb_inv}),
      .PORTA_CEINV({cea_inv}),
      .PORTB_CEINV({ceb_inv}),
      .PORTA_WEINV({wea_inv}),
      .PORTB_WEINV({web_inv}),
      .PORTA_RSTINV({~regsra_inv}),//rst is high active in bram, but due to history issue, invert this sig here because it's low active in top wraper.
      .PORTB_RSTINV({~regsrb_inv}),//rst is high active in bram, but due to history issue, invert this sig here because it's low active in top wraper.
      .PORTA_ENINV({regcea_inv}),
      .PORTB_ENINV({regceb_inv}),

/////////////////////////////////////////////
      .init_file(init_file),
      .is_initialized(is_initialized),
      .init_00(init_00),
      .init_01(init_01),
      .init_02(init_02),
      .init_03(init_03),
      .init_04(init_04),
      .init_05(init_05),
      .init_06(init_06),
      .init_07(init_07),
      .init_08(init_08),
      .init_09(init_09),
      .init_0a(init_0a),
      .init_0b(init_0b),
      .init_0c(init_0c),
      .init_0d(init_0d),
      .init_0e(init_0e),
      .init_0f(init_0f),
      .initp_00(initp_00),
      .initp_01(initp_01)
    ) u0_bram (
     .cea(cea),
     .ceb(ceb),
     .reg_ena(regen[0]),
     .reg_enb(regen[1]), 
     .clka(clka),
     .clkb(clkb),
     .rst_a(regsra),
     .rst_b(regsrb),
     .wea({wea[1:0]}),
     .web({web[1:0]}),

     .di_a(dataina_bnk1),
     .di_b(datainb_bnk1),
     .aa(addra_input),
     .ab(addrb_input),

     .outa(dataouta_bnk1),
     .outb(dataoutb_bnk1)
);
endmodule // EMB9K


module BRAM9K ( 
               outa, outb, aa, ab, cea, ceb, clka, clkb,reg_ena,reg_enb,
               rst_a,rst_b, di_a, di_b, wea, web );


output [17:0]  outb;
output [17:0]  outa;
input    cea, ceb;
input [12:0]  ab;
input [12:0]  aa;
input [17:0]  di_b;
input [17:0]  di_a;
input [1:0]  wea;
input [1:0]  web;
input clka,clkb;
input rst_a,rst_b;
input reg_ena,reg_enb;

parameter MODEA_SEL = 4'b0;
parameter MODEB_SEL = 4'b0;
parameter PORTA_PROG = 8'b0;
parameter PORTB_PROG = 8'b0;
parameter PORTA_WR_MODE = 2'b0;
parameter PORTB_WR_MODE = 2'b0;
parameter PORTA_CE = 1'b0;
parameter PORTB_CE = 1'b0;
parameter PORTA_REG_OUT = 1'b0;
parameter PORTB_REG_OUT = 1'b0;
parameter HASCLKA = 1'b0;
parameter HASCLKB = 1'b0;

parameter PORTA_CEINV = 1'b0;
parameter PORTB_CEINV = 1'b0;
parameter PORTA_CLKINV = 1'b0;
parameter PORTB_CLKINV = 1'b0;
parameter PORTA_ENINV = 1'b0;
parameter PORTB_ENINV = 1'b0;
parameter PORTA_RSTINV = 1'b0;
parameter PORTB_RSTINV = 1'b0;
parameter PORTA_WEINV = 1'b0;
parameter PORTB_WEINV = 1'b0;

parameter is_initialized = 0;
parameter init_file = "";
parameter init_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_02 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_03 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_04 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_05 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_06 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_07 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_08 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_09 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0a = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0b = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0c = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0d = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0e = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0f = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter initp_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter initp_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;

wire [17:0] da ; 
reg [17:0] da_r;
wire [17:0] outa_temp;
wire [17:0] db ; 
reg [17:0] db_r;
wire [17:0] outb_temp;

wire [17:0]  emb9k_qa;
wire [17:0]  emb9k_qb;

wire [17:0] ix_a_mix;
wire [17:0] ix_b_mix;

wire [17:0] emb9k_da;
wire [17:0] emb9k_db;

wire clka_t = clka & HASCLKA;
wire clkb_t = clkb & HASCLKB;
wire w_clka       = PORTA_CLKINV  == 1'b1 ? ~clka_t            : clka_t;
wire w_clkb       = PORTB_CLKINV  == 1'b1 ? ~clkb_t            : clkb_t;
wire w_cea        = PORTA_CEINV   == 1'b1 ? ~cea               : cea;
wire w_ceb        = PORTB_CEINV   == 1'b1 ? ~ceb               : ceb;
wire [1:0] w_wea  = PORTA_WEINV   == 1'b1 ? {~wea[1], ~wea[0]} : wea;
wire [1:0] w_web  = PORTB_WEINV   == 1'b1 ? {~web[1], ~web[0]} : web;
wire w_rsta       = PORTA_RSTINV  == 1'b1 ? ~rst_a             : rst_a;
wire w_rstb       = PORTB_RSTINV  == 1'b1 ? ~rst_b             : rst_b;
wire w_reg_ena_t  = PORTA_ENINV   == 1'b1 ? ~reg_ena           : reg_ena;
wire w_reg_enb_t  = PORTB_ENINV   == 1'b1 ? ~reg_enb           : reg_enb;
wire w_reg_ena    = PORTA_REG_OUT & w_reg_ena_t;
wire w_reg_enb    = PORTB_REG_OUT & w_reg_enb_t;
wire w_emb9k_rsta     = (~w_reg_ena) & w_rsta;
wire w_emb9k_rstb     = (~w_reg_enb) & w_rstb;

assign ix_a_mix = {di_a[7:0], di_a[8], di_a[8:0]};
assign ix_b_mix = {di_b[7:0], di_b[8], di_b[8:0]};
assign emb9k_da = (MODEA_SEL == 4'b0000) ? di_a : ix_a_mix; 
assign emb9k_db = (MODEB_SEL == 4'b0000) ? di_b : ix_b_mix; 

 emb9k_wrapper #(          .init_file(init_file),
                           .is_initialized(is_initialized), 
                           .init_00(init_00),
                           .init_01(init_01),
                           .init_02(init_02),
                           .init_03(init_03),
                           .init_04(init_04),
                           .init_05(init_05),
                           .init_06(init_06),
                           .init_07(init_07),
                           .init_08(init_08),
                           .init_09(init_09),
                           .init_0a(init_0a),
                           .init_0b(init_0b),
                           .init_0c(init_0c),
                           .init_0d(init_0d),
                           .init_0e(init_0e),
                           .init_0f(init_0f),
                           .initp_00(initp_00),
                           .initp_01(initp_01),
                           .porta_ce(PORTA_CE),
                           .portb_ce(PORTB_CE),
                           .modea_sel(MODEA_SEL),
                           .modeb_sel(MODEB_SEL),
                           .porta_wr_mode(PORTA_WR_MODE),
                           .portb_wr_mode(PORTB_WR_MODE),
                           .regouta(PORTA_REG_OUT), 
                           .regoutb(PORTB_REG_OUT)) 
                        u_emb9k ( 
                          .RSTA(w_emb9k_rsta),
                          .RSTB(w_emb9k_rstb),
                          .cea(w_cea),
                          .ceb(w_ceb),
                          .wea(w_wea),
                          .web(w_web),
                          .reg_ena(w_reg_ena),
                          .reg_enb(w_reg_enb),
                          .clka(w_clka),
                          .clkb(w_clkb),
                          .por_a(1'b1),
                          .por_b(1'b1),
                          .qa(emb9k_qa), 
                          .qb(emb9k_qb),
                          .pa_sval(18'h00000), 
                          .pb_sval(18'h00000),
                          .di_a(emb9k_da),
                          .aa(aa),
                          .di_b(emb9k_db),
                          .ab (ab));


assign da = emb9k_qa;

always @ (posedge w_clka) begin
    if(w_reg_ena) begin
        if(w_rsta)
            da_r <= 0;
        else
            da_r <= da;
    end
end

assign outa_temp = (PORTA_REG_OUT) ? da_r : da;
//assign outa[17:0] = (w_cea) ? outa_temp[17:0] : 'hx;
assign outa[17:0] = outa_temp[17:0];

assign db = emb9k_qb;

always @ (posedge w_clkb) begin
    if(w_reg_enb) begin
        if(w_rstb)
            db_r <= 0;
        else
            db_r <= db;
    end
end

assign outb_temp = (PORTB_REG_OUT) ? db_r : db;
//assign outb[17:0] = w_ceb ? outb_temp[17:0] : 'hx;
assign outb[17:0] = outb_temp[17:0];

endmodule

module emb9k_wrapper ( RSTA,
                       RSTB,
                       cea,
                       ceb,
                       wea,
                       web,
                       reg_ena,
                       reg_enb,
                       clka,
                       clkb,
                       por_a,
                       por_b,
                       qa, 
                       qb,
                       pa_sval, 
                       pb_sval,
                       di_a,
                       aa,
                       di_b,
                       ab );
parameter porta_ce = 1'b0;
parameter portb_ce = 1'b0;
parameter modea_sel = 4'b0;
parameter modeb_sel = 4'b0;
parameter porta_wr_mode = 2'b0;
parameter portb_wr_mode = 2'b0;
parameter regouta = 1'b0;
parameter regoutb = 1'b0;

parameter init_file = "";
parameter is_initialized = 0;
parameter init_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_02 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_03 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_04 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_05 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_06 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_07 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_08 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_09 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0a = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0b = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0c = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0d = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0e = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0f = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter initp_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter initp_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;

input RSTA ;
input RSTB;
input cea;
input clka;
input por_a;
input [17:0] di_a;
input [17:0] pa_sval;
input [12:0] aa;
input [1:0] wea;
input reg_ena;
input reg_enb;
output [17:0] qa;

input ceb;
input clkb;
input por_b;
input [17:0] di_b;
input [17:0] pb_sval;
input [12:0] ab;
input [1:0] web;
output [17:0] qb;

wire [17:0] do_a;
wire [17:0] do_b;

    
assign qa = do_a;
assign qb = do_b;
/*
assign qa = (MODEA_SEL == 4'b0000 ) ? do_a : 
            (MODEA_SEL == 4'b1000 ) ? {9'b0,do_a[8:0]} : 
            (MODEA_SEL == 4'b1100 ) ? {14'b0,do_a[3:0]} :
            (MODEA_SEL == 4'b1110 ) ? {16'b0,do_a[1:0]} :
            (MODEA_SEL == 4'b1111 ) ? {17'b0,do_a[0]} : 18'bx;
assign qb = (MODEB_SEL == 4'b0000 ) ? do_b : 
            (MODEB_SEL == 4'b1000 ) ? {9'b0,do_b[8:0]} : 
            (MODEB_SEL == 4'b1100 ) ? {14'b0,do_b[3:0]} :
            (MODEB_SEL == 4'b1110 ) ? {16'b0,do_b[1:0]} :
            (MODEB_SEL == 4'b1111 ) ? {17'b0,do_b[0]} : 18'bx;
 */
 


emb9k_core #(.init_file(init_file),
            .is_initialized(is_initialized),
            .init_00(init_00),
            .init_01(init_01),
            .init_02(init_02),
            .init_03(init_03),
            .init_04(init_04),
            .init_05(init_05),
            .init_06(init_06),
            .init_07(init_07),
            .init_08(init_08),
            .init_09(init_09),
            .init_0a(init_0a),
            .init_0b(init_0b),
            .init_0c(init_0c),
            .init_0d(init_0d),
            .init_0e(init_0e),
            .init_0f(init_0f),
            .initp_00(initp_00),
            .initp_01(initp_01)
    ) Iemb9k (
              .ck_a(clka),
              .ad_a(aa[12:0]),
              .di_a(di_a[17:0]),
              .ce_a(cea),
              .we_a(wea),
              .reg_ena(reg_ena),
              .reg_enb(reg_enb),
              .cfg_a({porta_wr_mode[1:0],modea_sel[3:0]}),
              .do_a(do_a[17:0]),
              .ck_b(clkb),
              .ad_b(ab[12:0]),
              .di_b(di_b[17:0]),
              .ce_b(ceb),
              .we_b(web[1:0]),
              .cfg_b({portb_wr_mode[1:0],modeb_sel[3:0]}),
              .do_b(do_b[17:0]),
              .rst_a(RSTA),
              .rst_b(RSTB), 
              .papgrmdmybl(16'b0),
              .pbpgrmdmybl(16'b0),
              .por_a(por_a),
              .por_b(por_b),
              .pa_red_adr(18'b0),
              .pb_red_adr(18'b0),
              .pa_sval(pa_sval[17:0]),
              .pb_sval(pb_sval[17:0]));

endmodule

module emb9k_core(
 ck_a
,ad_a
,di_a
,ce_a
,we_a
,reg_ena
,cfg_a
,do_a
,ck_b
,ad_b
,di_b
,ce_b
,we_b
,reg_enb
,cfg_b
,do_b
,rst_a
,rst_b 
,papgrmdmybl
,pbpgrmdmybl
,por_a
,por_b
,pa_red_adr
,pb_red_adr
,pa_sval
,pb_sval
);

input       ck_a; // clk of port a, positive edge
input [12:0]ad_a; // address of port a,13bit
input [17:0]di_a; // input data of port a, 18bit 
input       ce_a; // chip enable of port a, active high
input reg_ena;
input  [1:0]we_a; // write/read enable of port a, we_a=1, write, we_a=0, read
input       rst_a; // reset of port a, active hgih, rst_a=1, do_a=pa_sval
input [17:0]pa_red_adr; // redundancy control address of port a
input [17:0]pa_sval; // initial value when reset of port a

input [5:0] cfg_a; //configuration bit of port a, [3:0] for mode selection, [5] is for readfist selection, [4] is for write first selection	
input       ck_b; // clk of port b, positive edge
input [12:0]ad_b; //address of port b,13bit
input [17:0]di_b; // input data of port b, 18bit
input       ce_b; // chip enable of port b, active high
input reg_enb;
input   [1:0]we_b; // write/read enable of port b, we_b=1, write, we_b=0, read
input   [5:0]cfg_b; //configuration bit of port b, [3:0] for mode selection, [5] is for readfist selection, [4] is for write first selection	
input        rst_b; // reset of port b, active hgih, rst_b=1, do_b=pb_sval
input  [17:0]pb_red_adr; // redundancy control address of port b
input  [17:0]pb_sval; // initial value when reset of port b

output [17:0]do_a; //output data of port a
output [17:0]do_b; //output data of port b
 
input [15:0]papgrmdmybl; // dummy bl control signal of port a
input [15:0]pbpgrmdmybl; // dummy bl control signal of port b
input por_a; // internal feedback signal of port a
input por_b; // internal feedback signal of port b

reg [17:0] do_a,do_b;
reg [17:0] do_a_comb,do_b_comb;
reg [17:0] do_a_comb_temp,do_b_comb_temp;
reg [18:0] do_a_tmp,do_b_tmp,do_b_tmp_red;
reg [17:0] di_a_reg,di_b_reg;
reg [12:0] ad_a_reg,ad_b_reg;
reg        ce_a_reg;
reg [1:0] we_a_reg;
reg        ce_b_reg;
reg [1:0] we_b_reg;
reg [18:0] tmp_a_wr,tmp_b_wr;         //add a redudant bit
reg [18:0] tmp_a_rd,tmp_b_rd;          //add a redudant bit
reg [18:0] tmp_a_rd_red,tmp_b_rd_red; //add a redudant bit
reg [18:0] mem [0:511];//add a redudant bit
reg [17:0] do_a_first_read;
reg [17:0] do_b_first_read;

parameter init_file = "";
parameter is_initialized = 0;
parameter init_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_02 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_03 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_04 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_05 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_06 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_07 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_08 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_09 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0a = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0b = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0c = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0d = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0e = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter init_0f = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter initp_00 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
parameter initp_01 = 512'h00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;

reg [17:0] mem_init [0:8191];
integer i;
integer j;
integer fh;
integer fs;
integer dwidth;
integer cnt;
reg [17:0] data;
initial // check init_file
    begin
        i = 0;
        j = 0;
        cnt = 0;
        dwidth = 0;
        fh = 0;
        fs = 0;
        //if(is_initialized == 1'b1) begin
            for(i = 0; i < 32; i = i+1) begin
                //mem[i] = {init_0f[i], init_0e[i], init_0d[i], init_0c[i], init_0b[i], init_0a[i], init_09[i], init_08[i],
                //          initp_01[i], initp_00[i],
                //          init_07[i], init_06[i], init_05[i], init_04[i], init_03[i], init_02[i], init_01[i], init_00[i]};
                mem[i    ] <=  {init_00[i*16+15 -:8 ], initp_00[i*2+1   -:2 ],  init_00[i*16+7 -:8 ]};
                mem[i+32 ] <=  {init_01[i*16+15 -:8 ], initp_00[i*2+65  -:2 ],  init_01[i*16+7 -:8 ]};
                mem[i+64 ] <=  {init_02[i*16+15 -:8 ], initp_00[i*2+129 -:2 ],  init_02[i*16+7 -:8 ]};
                mem[i+96 ] <=  {init_03[i*16+15 -:8 ], initp_00[i*2+193 -:2 ],  init_03[i*16+7 -:8 ]};
                mem[i+128 ] <= {init_04[i*16+15 -:8 ], initp_00[i*2+257 -:2 ],  init_04[i*16+7 -:8 ]};
                mem[i+160 ] <= {init_05[i*16+15 -:8 ], initp_00[i*2+321 -:2 ],  init_05[i*16+7 -:8 ]};
                mem[i+192 ] <= {init_06[i*16+15 -:8 ], initp_00[i*2+385 -:2 ],  init_06[i*16+7 -:8 ]};
                mem[i+224 ] <= {init_07[i*16+15 -:8 ], initp_00[i*2+449 -:2 ],  init_07[i*16+7 -:8 ]};
                mem[i+256 ] <= {init_08[i*16+15 -:8 ], initp_01[i*2+1   -:2 ],  init_08[i*16+7 -:8 ]};
                mem[i+288 ] <= {init_09[i*16+15 -:8 ], initp_01[i*2+65  -:2 ],  init_09[i*16+7 -:8 ]};
                mem[i+320 ] <= {init_0a[i*16+15 -:8 ], initp_01[i*2+129 -:2 ],  init_0a[i*16+7 -:8 ]};
                mem[i+352 ] <= {init_0b[i*16+15 -:8 ], initp_01[i*2+193 -:2 ],  init_0b[i*16+7 -:8 ]};
                mem[i+384 ] <= {init_0c[i*16+15 -:8 ], initp_01[i*2+257 -:2 ],  init_0c[i*16+7 -:8 ]};
                mem[i+416 ] <= {init_0d[i*16+15 -:8 ], initp_01[i*2+321 -:2 ],  init_0d[i*16+7 -:8 ]};
                mem[i+448 ] <= {init_0e[i*16+15 -:8 ], initp_01[i*2+385 -:2 ],  init_0e[i*16+7 -:8 ]};
                mem[i+480 ] <= {init_0f[i*16+15 -:8 ], initp_01[i*2+449 -:2 ],  init_0f[i*16+7 -:8 ]};
            end
        //end

        #1;
        // Initialize mem_data
        if (init_file == " " || init_file == "" || init_file == "none") begin
            //$display("emb does not need data file for memory initialization.\n");
        end else begin// Memory initialization file is used
            $display("Initialize the emb: %s.", init_file);
            fh = $fopen(init_file, "r");
            while(!$feof(fh)) begin
                if(dwidth == 0) begin //find header
                    fs = $fscanf(fh, "//Width=%d\n", dwidth);
                    if(fs != 1) begin
                        dwidth = 0;
                    end
                end else begin //read data
                    data = 0;
                    fs = $fscanf(fh, "%h\n", data);
                    if(fs == 1) begin
                        mem_init[cnt] = data;
                        cnt = cnt+1;
                        if(cnt > ((9*1024)/dwidth)) begin
                            $display("Error: Initial data (%d*%d) exceeds memory size (%d).", dwidth, cnt, 9*1024);
                        end
                    end
                end
                i = i+1;
                if(i > 9*1024) $finish;
            end
//            for(i=0; i<256; i=i+1) begin
//                $display("%d\t%0x", i, mem_init[i]);
//            end

            if(cfg_b[3:0] == 4'b1111) begin  //width = 1
                for(i=0; i<512; i=i+1) begin
                    for(j=0; j<8; j=j+1) begin
                        mem[i][j] = mem_init[i*16+j][0];
                        mem[i][j+10] = mem_init[i*16+j+8][0];
                    end
                end
            end else if(cfg_b[3:0] == 4'b1110) begin  //width = 2
                for(i=0; i<512; i=i+1) begin
                    mem[i] = {mem_init[i*8+7][1:0], mem_init[i*8+6][1:0], mem_init[i*8+5][1:0], mem_init[i*8+4][1:0],
                              2'b00, mem_init[i*8+3][1:0], mem_init[i*8+2][1:0], mem_init[i*8+1][1:0], mem_init[i*8+0][1:0]};
                end
            end else if(cfg_b[3:0] == 4'b1100) begin  //width = 4
                for(i=0; i<512; i=i+1) begin
                    mem[i] = {mem_init[i*4+3][3:0], mem_init[i*4+2][3:0], 2'b00, mem_init[i*4+1][3:0], mem_init[i*4+0][3:0]};
                end
            end else if(cfg_b[3:0] == 4'b1000) begin  //width = 9
                for(i=0; i<512; i=i+1) begin
                    mem[i] = {mem_init[i*2+1][7:0], mem_init[i*2+1][8], mem_init[i*2+0][8], mem_init[i*2+0][7:0]};
                end
            end else if(cfg_b[3:0] == 4'b0000) begin //width = 18
				for(i=0; i<512; i=i+1) begin
                    if(dwidth > 16) begin
					    //mem[i] = {mem_init[i][16:9], mem_init[i][17], mem_init[i][8], mem_init[i][7:0]};
					    mem[i] = mem_init[i];
                    end else begin
					    mem[i] = {mem_init[i][15:8], 2'b00,  mem_init[i][7:0]};
                    end
				end
			end
//            for(i=0; i<256; i=i+1) begin
//                $display("%d\t%0x", i, mem[i]);
//            end
        end
    end

wire [12:4] ad_a_reg_tmp;
wire [12:4] ad_b_reg_tmp;

//for read first
always@(negedge por_a or posedge ck_a)
begin
  if (!por_a) begin
    do_a_first_read[17:0]<=18'b0;
  end else begin
    do_a_first_read[17:0]<=  mem[ad_a[12:4]];
  end
end



always@(negedge por_b or posedge ck_b)
begin
  if (!por_b)begin
    do_b_first_read[17:0]<=18'b0;
  end else begin
    do_b_first_read[17:0]<=  mem[ad_b[12:4]];
  end
end

//interface registers
always@(posedge ck_a or negedge por_a) begin
  if (!por_a) begin
    ce_a_reg <= 0;
    ad_a_reg[12:0] <= 13'b0;
    di_a_reg[17:0] <= 18'b0;
    we_a_reg[1:0] <= 2'b0;
  end else if(rst_a) begin
    ce_a_reg <= ce_a;
    ad_a_reg[12:0] <= ad_a[12:0];
    di_a_reg[17:0] <= 18'b0;
    we_a_reg[1:0] <=2'b0;
  end else begin
    ce_a_reg <= ce_a;
    ad_a_reg[12:0] <= ad_a[12:0];
    di_a_reg[17:0] <= di_a[17:0];
    we_a_reg[1:0] <= we_a[1:0];
  end
end

always@(posedge ck_b or negedge por_b) begin
  if (!por_b) begin
    ce_b_reg <= 0;
    ad_b_reg[12:0] <= 13'b0;
    di_b_reg[17:0] <= 18'b0;
    we_b_reg[1:0] <= 2'b0;
  end else if(rst_b) begin
    ce_b_reg <=  ce_b;
    ad_b_reg[12:0]<=  ad_b[12:0];
    di_b_reg[17:0] <= 18'b0;
    we_b_reg[1:0] <= 2'b0;
  end else begin
    ce_b_reg <=  ce_b;
    ad_b_reg[12:0]<=  ad_b[12:0];
    di_b_reg[17:0] <=  di_b[17:0];
    we_b_reg[1:0]<=  we_b[1:0];
  end
end

assign ad_a_reg_tmp=ad_a_reg[12:4];
assign ad_b_reg_tmp=ad_b_reg[12:4];

//prepare write in data in port a
always@(*) begin
    tmp_a_wr = mem[ad_a_reg_tmp];
    if(cfg_a[3:0] == 4'b1111) begin  //x1 mode
        if(ad_a_reg[3:0] == 00)  tmp_a_wr[00] = di_a_reg[00];
        else if(ad_a_reg[3:0] == 01)  tmp_a_wr[01] = di_a_reg[01];
        else if(ad_a_reg[3:0] == 02)  tmp_a_wr[02] = di_a_reg[02];
        else if(ad_a_reg[3:0] == 03)  tmp_a_wr[03] = di_a_reg[03];
        else if(ad_a_reg[3:0] == 04)  tmp_a_wr[04] = di_a_reg[04];
        else if(ad_a_reg[3:0] == 05)  tmp_a_wr[05] = di_a_reg[05];
        else if(ad_a_reg[3:0] == 06)  tmp_a_wr[06] = di_a_reg[06];
        else if(ad_a_reg[3:0] == 07)  tmp_a_wr[07] = di_a_reg[07];
        else if(ad_a_reg[3:0] == 08)  tmp_a_wr[10] = di_a_reg[10];
        else if(ad_a_reg[3:0] == 09)  tmp_a_wr[11] = di_a_reg[11];
        else if(ad_a_reg[3:0] == 10)  tmp_a_wr[12] = di_a_reg[12];
        else if(ad_a_reg[3:0] == 11)  tmp_a_wr[13] = di_a_reg[13];
        else if(ad_a_reg[3:0] == 12)  tmp_a_wr[14] = di_a_reg[14];
        else if(ad_a_reg[3:0] == 13)  tmp_a_wr[15] = di_a_reg[15];
        else if(ad_a_reg[3:0] == 14)  tmp_a_wr[16] = di_a_reg[16];
        else if(ad_a_reg[3:0] == 15)  tmp_a_wr[17] = di_a_reg[17];
        tmp_a_wr[9:8] = di_a_reg[9:8];  
    end else if(cfg_a[3:0] == 4'b1110) begin // x2 mode
        if(ad_a_reg[3:1] == 0)  tmp_a_wr[01:00] = di_a_reg[01:00];
        else if(ad_a_reg[3:1] == 1)  tmp_a_wr[03:02] = di_a_reg[03:02];
        else if(ad_a_reg[3:1] == 2)  tmp_a_wr[05:04] = di_a_reg[05:04];
        else if(ad_a_reg[3:1] == 3)  tmp_a_wr[07:06] = di_a_reg[07:06];
        else if(ad_a_reg[3:1] == 4)  tmp_a_wr[11:10] = di_a_reg[11:10];
        else if(ad_a_reg[3:1] == 5)  tmp_a_wr[13:12] = di_a_reg[13:12];
        else if(ad_a_reg[3:1] == 6)  tmp_a_wr[15:14] = di_a_reg[15:14];
        else if(ad_a_reg[3:1] == 7)  tmp_a_wr[17:16] = di_a_reg[17:16];
        tmp_a_wr[9:8] = di_a_reg[9:8];
    end else if(cfg_a[3:0] == 4'b1100) begin // x4 mode
        if(ad_a_reg[3:2] == 0) tmp_a_wr[03:00] = di_a_reg[03:00];
        else if(ad_a_reg[3:2] == 1) tmp_a_wr[07:04] = di_a_reg[07:04];
        else if(ad_a_reg[3:2] == 2) tmp_a_wr[13:10] = di_a_reg[13:10];
        else if(ad_a_reg[3:2] == 3) tmp_a_wr[17:14] = di_a_reg[17:14];
        tmp_a_wr[9:8] = di_a_reg[9:8];
    end else if(cfg_a[3:0] == 4'b1000) begin // x9 mode
        if(ad_a_reg[3])  {tmp_a_wr[9],tmp_a_wr[17:10]} = {di_a_reg[9],di_a_reg[17:10]};
        else             {tmp_a_wr[8],tmp_a_wr[07:00]} = {di_a_reg[8],di_a_reg[07:00]};
    end else if(cfg_a[3:0] == 4'b0000) begin // x18 mode
        if(we_a_reg[1]) {tmp_a_wr[9],tmp_a_wr[17:10]} = {di_a_reg[9],di_a_reg[17:10]};
        if(we_a_reg[0]) {tmp_a_wr[8],tmp_a_wr[07:00]} = {di_a_reg[8],di_a_reg[07:00]};
    end

    //write
    if(ce_a_reg & (|we_a_reg[1:0])) begin
        mem[ad_a_reg_tmp]=tmp_a_wr;
    end
end


always@(*) begin
    tmp_b_wr = mem[ad_b_reg_tmp];
    if(cfg_b[3:0] == 4'b1111) begin  //x1 mode
        if(ad_b_reg[3:0] == 00)  tmp_b_wr[00] = di_b_reg[00];
        else if(ad_b_reg[3:0] == 01)  tmp_b_wr[01] = di_b_reg[01];
        else if(ad_b_reg[3:0] == 02)  tmp_b_wr[02] = di_b_reg[02];
        else if(ad_b_reg[3:0] == 03)  tmp_b_wr[03] = di_b_reg[03];
        else if(ad_b_reg[3:0] == 04)  tmp_b_wr[04] = di_b_reg[04];
        else if(ad_b_reg[3:0] == 05)  tmp_b_wr[05] = di_b_reg[05];
        else if(ad_b_reg[3:0] == 06)  tmp_b_wr[06] = di_b_reg[06];
        else if(ad_b_reg[3:0] == 07)  tmp_b_wr[07] = di_b_reg[07];
        else if(ad_b_reg[3:0] == 08)  tmp_b_wr[10] = di_b_reg[10];
        else if(ad_b_reg[3:0] == 09)  tmp_b_wr[11] = di_b_reg[11];
        else if(ad_b_reg[3:0] == 10)  tmp_b_wr[12] = di_b_reg[12];
        else if(ad_b_reg[3:0] == 11)  tmp_b_wr[13] = di_b_reg[13];
        else if(ad_b_reg[3:0] == 12)  tmp_b_wr[14] = di_b_reg[14];
        else if(ad_b_reg[3:0] == 13)  tmp_b_wr[15] = di_b_reg[15];
        else if(ad_b_reg[3:0] == 14)  tmp_b_wr[16] = di_b_reg[16];
        else if(ad_b_reg[3:0] == 15)  tmp_b_wr[17] = di_b_reg[17];
        tmp_b_wr[9:8] = di_b_reg[9:8];
    end else if(cfg_b[3:0] == 4'b1110) begin //x2 mode
        if(ad_b_reg[3:1] == 0)       tmp_b_wr[01:00] = di_b_reg[01:00];
        else if(ad_b_reg[3:1] == 1)  tmp_b_wr[03:02] = di_b_reg[03:02];
        else if(ad_b_reg[3:1] == 2)  tmp_b_wr[05:04] = di_b_reg[05:04];
        else if(ad_b_reg[3:1] == 3)  tmp_b_wr[07:06] = di_b_reg[07:06];
        else if(ad_b_reg[3:1] == 4)  tmp_b_wr[11:10] = di_b_reg[11:10];
        else if(ad_b_reg[3:1] == 5)  tmp_b_wr[13:12] = di_b_reg[13:12];
        else if(ad_b_reg[3:1] == 6)  tmp_b_wr[15:14] = di_b_reg[15:14];
        else if(ad_b_reg[3:1] == 7)  tmp_b_wr[17:16] = di_b_reg[17:16];
        tmp_b_wr[9:8] = di_b_reg[9:8];
    end else if(cfg_b[3:0] == 4'b1100) begin //x4 mode
        if(ad_b_reg[3:2] == 0)      tmp_b_wr[03:00] = di_b_reg[03:00];
        else if(ad_b_reg[3:2] == 1) tmp_b_wr[07:04] = di_b_reg[07:04];
        else if(ad_b_reg[3:2] == 2) tmp_b_wr[13:10] = di_b_reg[13:10];
        else if(ad_b_reg[3:2] == 3) tmp_b_wr[17:14] = di_b_reg[17:14];
        tmp_b_wr[9:8] = di_b_reg[9:8];
    end else if(cfg_b[3:0] == 4'b1000) begin //x9 mode
        if(ad_b_reg[3])  {tmp_b_wr[9],tmp_b_wr[17:10]} = {di_b_reg[9],di_b_reg[17:10]};
        else             {tmp_b_wr[8],tmp_b_wr[07:00]} = {di_b_reg[8],di_b_reg[07:00]};
    end else if(cfg_b[3:0] == 4'b0000) begin //x18 mode
        if(we_b_reg[1]) {tmp_b_wr[9],tmp_b_wr[17:10]} = {di_b_reg[9],di_b_reg[17:10]};
        if(we_b_reg[0]) {tmp_b_wr[8],tmp_b_wr[07:00]} = {di_b_reg[8],di_b_reg[07:00]};
    end

    //write
    if(ce_b_reg & (|we_b_reg[1:0])) begin
        mem[ad_b_reg_tmp]=tmp_b_wr;
    end
end


//////////////////////////////////////write operation  and read operation////////////////////////////////////////////////
wire pa_wrf ; // Port a in write first mode
wire pa_rdf ; // Port a in read first mode
wire pb_wrf ; // Port b in write first mode
wire pb_rdf ; // Port b in read first mode

assign pa_wrf = cfg_a[4] & !cfg_a[5] & (|we_a_reg[1:0]);
assign pa_rdf = cfg_a[5];
assign pb_wrf = cfg_b[4] & !cfg_b[5] & (|we_b_reg[1:0]);
assign pb_rdf = cfg_b[5];

always@(*)
begin
	if(ce_a_reg)
        if (rst_a)      //read first
            do_a_tmp = 0;
        else if (pa_rdf)      //read first
            do_a_tmp = do_a_first_read;
	    else if (pa_wrf) //write first
	        do_a_tmp = tmp_a_wr;
	    else if(!we_a_reg[1:0]) //read
            do_a_tmp = mem[ad_a_reg_tmp];
        else                    //no change
            do_a_tmp = do_a_tmp;
    else
        do_a_tmp = do_a_tmp;
end
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////write operation  and read operation////////////////////////////////////////////////
//write operation in port b

always@(*)
begin
	if(ce_b_reg)//read
        if (rst_b)  //read first
            do_b_tmp = 0;
        else if (pb_rdf)  //read first
            do_b_tmp = do_b_first_read;
	    else if (pb_wrf)//write through
	        do_b_tmp = tmp_b_wr;
        else if(!we_b_reg[1:0]) //read
	        do_b_tmp = mem[ad_b_reg_tmp];
        else                    //nochage
            do_b_tmp = do_b_tmp;
    else
        do_b_tmp = do_b_tmp;
    
end
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//prepare output data
always@(*) begin
    if(cfg_a[3:0] == 4'b1111) begin  
        case (ad_a_reg[3:0]) 
            4'b0000: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[0] ;
                         else        do_a_comb[0] = do_a_tmp[0] ;
                     end
            
            4'b0001: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[1] ;
                         else        do_a_comb[0] = do_a_tmp[1] ;
                     end
            
            4'b0010: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[2] ;
                         else        do_a_comb[0] = do_a_tmp[2] ;
                     end
            
            4'b0011: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[3] ;
                         else        do_a_comb[0] = do_a_tmp[3] ;
                     end
            
            4'b0100: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[4] ;
                         else        do_a_comb[0] = do_a_tmp[4] ;
                     end
            
            4'b0101: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[5] ;
                         else        do_a_comb[0] = do_a_tmp[5] ;
                     end
            
            4'b0110: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[6] ;
                         else        do_a_comb[0] = do_a_tmp[6] ;
                     end
            
            4'b0111: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[7] ;
                         else        do_a_comb[0] = do_a_tmp[7] ;
                     end
            
            4'b1000: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[10] ;
                         else        do_a_comb[0] = do_a_tmp[10] ;
                     end
            
            4'b1001: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[11] ;
                         else        do_a_comb[0] = do_a_tmp[11] ;
                     end
            
            4'b1010: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[12] ;
                         else        do_a_comb[0] = do_a_tmp[12] ;
                     end
            
            4'b1011: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[13] ;
                         else        do_a_comb[0] = do_a_tmp[13] ;
                     end
            
            4'b1100: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[14] ;
                         else        do_a_comb[0] = do_a_tmp[14] ;
                     end
            
            4'b1101: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[15] ;
                         else        do_a_comb[0] = do_a_tmp[15] ;
                     end
            
            4'b1110: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[16] ;
                         else        do_a_comb[0] = do_a_tmp[16] ;
                     end
            
            4'b1111: begin 
                         if(pa_wrf)  do_a_comb[0] = di_a_reg[17] ;
                         else        do_a_comb[0] = do_a_tmp[17] ;
                     end
        endcase 
    end else if(cfg_a[3:0] == 4'b1110) begin// 2 data width
    // by yangkun
        case (ad_a_reg[3:1]) 
           3'b000: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [1:0];
               else        do_a_comb[1:0] = do_a_tmp [1:0];
             end
           3'b001: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [3:2];
               else        do_a_comb[1:0] = do_a_tmp [3:2];
             end
           3'b010: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [5:4];
               else        do_a_comb[1:0] = do_a_tmp [5:4];
             end
           3'b011: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [7:6];
               else        do_a_comb[1:0] = do_a_tmp [7:6];
             end
           3'b100: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [11:10];
               else        do_a_comb[1:0] = do_a_tmp [11:10];
             end
           3'b101: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [13:12];
               else        do_a_comb[1:0] = do_a_tmp [13:12];
             end
           3'b110: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [15:14];
               else        do_a_comb[1:0] = do_a_tmp [15:14];
             end
           3'b111: begin  
               if(pa_wrf) do_a_comb[1:0]  = di_a_reg [17:16];
               else        do_a_comb[1:0] = do_a_tmp [17:16];
             end
        endcase
    end else if(cfg_a[3:0] == 4'b1100) begin// 4 bits data width
        if(ad_a_reg[3:2] == 0) begin  
            if(pa_wrf) begin 
                do_a_comb_temp = di_a_reg & 18'b00_0000_00xx_0000_1111;
                do_a_comb[3:0] = do_a_comb_temp[3:0];
            end else  begin 
                do_a_comb[3:0] = do_a_tmp[3:0];
            end
        end else if(ad_a_reg[3:2] == 1) begin
            if(pa_wrf)  
                do_a_comb[3:0] = di_a_reg[7:4];
            else 
            do_a_comb[3:0] = do_a_tmp[7:4] ;
        end else if(ad_a_reg[3:2] == 2) begin
            if(pa_wrf)
                do_a_comb[3:0] = di_a_reg[13:10];
            else        
                do_a_comb [3:0] = do_a_tmp[13:10];
        end else begin
            if(pa_wrf)       
                do_a_comb = di_a_reg [17:14];
            else             
                do_a_comb [3:0] = do_a_tmp [17:14] ;
        end
    end else if(cfg_a[3:0] == 4'b1000) begin
        if(~ad_a_reg[3]) begin
            if(pa_wrf) do_a_comb = di_a_reg & 18'b00_0000_0001_1111_1111;
            else       do_a_comb = do_a_tmp & 18'b00_0000_0001_1111_1111;
    	end if(ad_a_reg[3]) begin
            if(pa_wrf) begin 
                do_a_comb_temp  = (di_a_reg & 18'b11_1111_1110_0000_0000);
                do_a_comb [8]   = do_a_comb_temp[9];
                do_a_comb [7:0] = do_a_comb_temp[17:10];
            end else begin
                do_a_comb_temp  = (do_a_tmp & 18'b11_1111_1110_0000_0000);
                do_a_comb [8]   = do_a_comb_temp[9];
                do_a_comb [7:0] = do_a_comb_temp[17:10];
            end
        end
    end else if(cfg_a[3:0] == 4'b0000) begin
        if(pa_wrf) do_a_comb = di_a_reg;
        else begin 
            {do_a_comb[8],do_a_comb[7:0]}   = {do_a_tmp[8],do_a_tmp[7:0]};
            {do_a_comb[9],do_a_comb[17:10]} = {do_a_tmp[9],do_a_tmp[17:10]};
        end
    end
end

//port a 
always@(*) begin
    if (ck_a && ce_a_reg && por_a)begin
        if(!(|we_a_reg)) begin //read  20201205
            if(cfg_a[3:0] == 4'b1100 || cfg_a[3:0] == 4'b1110 || cfg_a[3:0] == 4'b1111) begin
                do_a = {do_a_comb[17:10],2'bx,do_a_comb[7:0]};
            end else begin
                do_a = do_a_comb;
            end
        end else begin//write
            if (pa_wrf) begin //write_through
                if(cfg_a[3:0] == 4'b1100 || cfg_a[3:0] == 4'b1110 || cfg_a[3:0] == 4'b1111) begin
                    do_a = {do_a_comb[17:10],2'bx,do_a_comb[7:0]};
                end else begin
	                do_a = do_a_comb;
	            end
            end if (pa_rdf) begin //read first
                if(cfg_a[3:0] == 4'b1100 || cfg_a[3:0] == 4'b1110 || cfg_a[3:0] == 4'b1111) begin
                    do_a = {do_a_comb[17:10],2'bx,do_a_comb[7:0]};
                end else begin
	                do_a = do_a_comb;
	            end
            end else begin // write no-change mode
                do_a = do_a ;//20201205
            end
        end
    end
end




 
//port b	  
always@(*) begin
    if(cfg_b[3:0] == 4'b1111) begin  
        case (ad_b_reg[3:0]) 
        4'b0000: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[0];
                  else       do_b_comb[0] = do_b_tmp[0];
                 end
        
        4'b0001: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[1];
                  else       do_b_comb[0] = do_b_tmp[1];
                 end
        4'b0010: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[2];
                  else       do_b_comb[0] = do_b_tmp[2];
                 end
        4'b0011: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[3];
                  else       do_b_comb[0] = do_b_tmp[3];
                 end
        4'b0100: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[4];
                  else       do_b_comb[0] = do_b_tmp[4];
                 end
        4'b0101: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[5];
                  else       do_b_comb[0] = do_b_tmp[5];
                 end
        4'b0110: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[6];
                  else       do_b_comb[0] = do_b_tmp[6];
                 end
        4'b0111: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[7];
                  else       do_b_comb[0] = do_b_tmp[7];
                 end
        4'b1000: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[10];
                  else       do_b_comb[0] = do_b_tmp[10];
                 end
        4'b1001: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[11];
                  else       do_b_comb[0] = do_b_tmp[11];
                 end
        4'b1010: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[12];
                  else       do_b_comb[0] = do_b_tmp[12];
                 end
        4'b1011: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[13];
                  else       do_b_comb[0] = do_b_tmp[13];
                 end
        4'b1100: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[14];
                  else       do_b_comb[0] = do_b_tmp[14];
                 end
        4'b1101: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[15];
                  else       do_b_comb[0] = do_b_tmp[15];
                 end
        4'b1110: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[16];
                  else       do_b_comb[0] = do_b_tmp[16];
                 end
        4'b1111: begin 
                  if(pb_wrf) do_b_comb[0] = di_b_reg[17];
                  else       do_b_comb[0] = do_b_tmp[17];
                 end
        endcase
    end else if(cfg_b[3:0] == 4'b1110) begin
        case (ad_b_reg[3:1]) 
        3'b000 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[1:0] ;
                    else do_b_comb[1:0] = do_b_tmp[1:0];
                 end    
        3'b001 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[3:2] ;
                    else do_b_comb[1:0] = do_b_tmp[3:2];
                 end    
        3'b010 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[5:4] ;
                    else do_b_comb[1:0] = do_b_tmp[5:4];
                 end    
        3'b011 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[7:6] ;
                    else do_b_comb[1:0] = do_b_tmp[7:6];
                 end    
        3'b100 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[11:10] ;
                    else do_b_comb[1:0] = do_b_tmp[11:10];
                 end    
        3'b101 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[13:12] ;
                    else do_b_comb[1:0] = do_b_tmp[13:12];
                 end    
        3'b110 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[15:14] ;
                    else do_b_comb[1:0] = do_b_tmp[15:14];
                 end    
        3'b111 : begin 
                    if(pb_wrf) do_b_comb[1:0] = di_b_reg[17:16] ;
                    else do_b_comb[1:0] = do_b_tmp[17:16];
                 end    
        endcase  
    end else if(cfg_b[3:0] == 4'b1100) begin// 4 bits data width
        case(ad_b_reg[3:2]) 
        2'b00 : begin 
                  if(pb_wrf) do_b_comb[3:0] = di_b_reg[3:0] ;
                  else       do_b_comb[3:0] = do_b_tmp[3:0] ;
                end
        2'b01 : begin 
                  if(pb_wrf) do_b_comb[3:0] = di_b_reg[7:4] ;
                  else       do_b_comb[3:0] = do_b_tmp[7:4] ;
                end
        2'b10 : begin 
                  if(pb_wrf) do_b_comb[3:0] = di_b_reg[13:10] ;
                  else       do_b_comb[3:0] = do_b_tmp[13:10] ;
                end
        2'b11 : begin 
                  if(pb_wrf) do_b_comb[3:0] = di_b_reg[17:14] ;
                  else       do_b_comb[3:0] = do_b_tmp[17:14] ;
                end
        endcase
    end else if(cfg_b[3:0] == 4'b1000) begin
        if(~ad_b_reg[3]) begin
            if(pb_wrf) do_b_comb[8:0] = di_b_reg[8:0];
            else       do_b_comb[8:0] = do_b_tmp[8:0];
        end else begin 
          if(pb_wrf) do_b_comb[8:0] = {di_b_reg[9],di_b_reg[17:10]};
          else       do_b_comb[8:0] = {do_b_tmp[9],do_b_tmp[17:10]};
        end
    end else if(cfg_b[3:0] == 4'b0000) begin
        if(pb_wrf) do_b_comb = di_b_reg;
        else begin
            {do_b_comb[8],do_b_comb[7:0]}   = {do_b_tmp[8],do_b_tmp[7:0]};
            {do_b_comb[9],do_b_comb[17:10]} = {do_b_tmp[9],do_b_tmp[17:10]};
        end
    end
end

always@(*) begin
    if (ck_b && ce_b_reg && por_b)begin
        if(!we_b_reg) begin //read
            if(cfg_b[3:0] == 4'b1100 || cfg_b[3:0] == 4'b1110 || cfg_b[3:0] == 4'b1111) begin
                do_b = {do_b_comb[17:10],2'bx,do_b_comb[7:0]};
            end else begin
                do_b = do_b_comb;
            end
        end else begin//write
            if ( pb_wrf) begin//write_through
                if(cfg_b[3:0] == 4'b1100 || cfg_b[3:0] == 4'b1110 || cfg_b[3:0] == 4'b1111) begin
                    do_b = {do_b_comb[17:10],2'bx,do_b_comb[7:0]};
                end else begin
                    do_b = do_b_comb;
                end
            end	else if (pb_rdf) begin//read first
                if(cfg_b[3:0] == 4'b1100 || cfg_b[3:0] == 4'b1110 || cfg_b[3:0] == 4'b1111) begin
                    do_b = {do_b_comb[17:10],2'bx,do_b_comb[7:0]};
                end else begin
                    do_b = do_b_comb;
                end
            end	else begin // write no-change mode
                do_b = do_b ;
            end
        end
    end
end	
    
endmodule


`define IO_DLY 1
module delay_block(in, sel, out);

input in;
input [4:0] sel;
output out;

wire [31:0] w;
assign w[0] = in;
assign #`IO_DLY w[1] = w[0];
assign #`IO_DLY w[2] = w[1];
assign #`IO_DLY w[3] = w[2];
assign #`IO_DLY w[4] = w[3];
assign #`IO_DLY w[5] = w[4];
assign #`IO_DLY w[6] = w[5];
assign #`IO_DLY w[7] = w[6];
assign #`IO_DLY w[8] = w[7];
assign #`IO_DLY w[9] = w[8];
assign #`IO_DLY w[10] = w[9];
assign #`IO_DLY w[11] = w[10];
assign #`IO_DLY w[12] = w[11];
assign #`IO_DLY w[13] = w[12];
assign #`IO_DLY w[14] = w[13];
assign #`IO_DLY w[15] = w[14];
assign #`IO_DLY w[16] = w[15];
assign #`IO_DLY w[17] = w[16];
assign #`IO_DLY w[18] = w[17];
assign #`IO_DLY w[19] = w[18];
assign #`IO_DLY w[20] = w[19];
assign #`IO_DLY w[21] = w[20];
assign #`IO_DLY w[22] = w[21];
assign #`IO_DLY w[23] = w[22];
assign #`IO_DLY w[24] = w[23];
assign #`IO_DLY w[25] = w[24];
assign #`IO_DLY w[26] = w[25];
assign #`IO_DLY w[27] = w[26];
assign #`IO_DLY w[28] = w[27];
assign #`IO_DLY w[29] = w[28];
assign #`IO_DLY w[30] = w[29];
assign #`IO_DLY w[31] = w[30];

assign out = w[sel];

endmodule



//--------------------------------------------
module ILOGIC_detect (
	datain,
	fclk0,
	fclk1,
	geclk0,
	geclk1,
	gsclk0,
	gsclk1,
	rst,
	setn,
	shiftin0,
	shiftin1,
	update,

	shiftout0,
	shiftout1,
	dataout,
    sout,

	//parameter
	CFG_DDR_IN_NREG,
	CFG_DDR_IN_NREG_DFF,
	CFG_DDR_IN_PREG,
	CFG_DDR_IN_PREG_DFF,
	CFG_FASTIN,
	CFG_SLAVE_IN,
	CFG_GEAR_IN
	);
output	shiftout0,shiftout1;
output	[7:0]	dataout;
output  [9:0]   sout;

input	datain;
input	fclk0;
input	fclk1;
input	geclk0;
input	geclk1;
input	gsclk0;
input	gsclk1;
input	rst;
input	setn;
input	shiftin0,shiftin1;
input	update;
input	CFG_DDR_IN_NREG;
input	CFG_DDR_IN_NREG_DFF;
input	CFG_DDR_IN_PREG;
input	CFG_DDR_IN_PREG_DFF;
input	CFG_FASTIN;
input	CFG_SLAVE_IN;
input	[7:0]	CFG_GEAR_IN;

reg do_d, do_dd;
reg de_d, de_dd;

always @(posedge fclk0 or negedge setn or posedge rst)begin
	if(!setn) de_d <= 1;
	else if(rst) de_d <= 0;
	else de_d <= datain;
end
always @(posedge fclk0 or negedge setn or posedge rst)begin
	if(!setn) de_dd <= 1;
	else if(rst) de_dd <= 0;
	else de_dd <= de_d;
end
always @(negedge fclk1 or negedge setn or posedge rst)begin
	if(!setn) do_d <= 1;
	else if(rst) do_d <= 0;
	else do_d <= datain;
end
always @(posedge fclk1 or negedge setn or posedge rst)begin
	if(!setn) do_dd <= 1;
	else if(rst) do_dd <= 0;
	else do_dd <= do_d;
end

wire    de_in, do_in;
assign de_in =	({CFG_DDR_IN_PREG_DFF,CFG_DDR_IN_PREG,CFG_FASTIN} == 3'b001) ? datain :
				({CFG_DDR_IN_PREG_DFF,CFG_DDR_IN_PREG,CFG_FASTIN} == 3'b010) ? de_d :
				({CFG_DDR_IN_PREG_DFF,CFG_DDR_IN_PREG,CFG_FASTIN} == 3'b100) ? de_dd :
				({CFG_DDR_IN_PREG_DFF,CFG_DDR_IN_PREG,CFG_FASTIN} == 3'b000) ? 1'b1 : 1'bx;
assign do_in =	({CFG_DDR_IN_NREG_DFF,CFG_DDR_IN_NREG,CFG_FASTIN} == 3'b001) ? datain :
				({CFG_DDR_IN_NREG_DFF,CFG_DDR_IN_NREG,CFG_FASTIN} == 3'b010) ? do_d :
				({CFG_DDR_IN_NREG_DFF,CFG_DDR_IN_NREG,CFG_FASTIN} == 3'b100) ? do_dd :
				({CFG_DDR_IN_NREG_DFF,CFG_DDR_IN_NREG,CFG_FASTIN} == 3'b000) ? 1'b1 : 1'bx;

reg de_pre, do_pre;
always @(posedge geclk1 or posedge rst or negedge setn) begin
	if(!setn) de_pre <= 1'b1;
    else if(rst) de_pre <= 1'b0;
	else de_pre <= de_in;
end
always @(posedge geclk0 or posedge rst or negedge setn) begin
	if(!setn) do_pre <= 1'b1;
    else if(rst) do_pre <= 1'b0;
	else do_pre <= do_in;
end

reg	q7,q5,q3,q1;
reg	q6,q4,q2,q0;
wire	slv1,slv0;
assign slv1 = CFG_SLAVE_IN ? shiftin1 : do_pre; 
assign slv0 = CFG_SLAVE_IN ? shiftin0 : de_pre;
always @(posedge geclk1 or negedge setn or posedge rst)begin
	if(!setn) q7 <= 1'b1;
    else if(rst) q7 <= 1'b0;
	else q7 <= slv1;
end
always @(posedge geclk0 or negedge setn or posedge rst)begin
	if(!setn) q6 <= 1'b1;
    else if(rst) q6 <= 1'b0;
	else q6 <= slv0;
end
always @(posedge geclk1 or negedge setn or posedge rst)begin
	if(!setn) begin
        q5 <= 1'b1;
        q3 <= 1'b1;
        q1 <= 1'b1;
    end else if(rst) begin
        q5 <= 1'b0;
        q3 <= 1'b0;
        q1 <= 1'b0;
	end else begin
        q5 <= q7;
        q3 <= q5;
        q1 <= q3;
    end
end
always @(posedge geclk0 or negedge setn or posedge rst)begin
	if(!setn) begin
        q4 <= 1'b1;
        q2 <= 1'b1;
        q0 <= 1'b1;
    end else if(rst) begin
        q4 <= 1'b0;
        q2 <= 1'b0;
        q0 <= 1'b0;
	end else begin
        q4 <= q6;
        q2 <= q4;
        q0 <= q2;
    end
end

assign shiftout1 = q1;
assign shiftout0 = q0;

reg	q7s,q5s,q3s,q1s;
reg	q6s,q4s,q2s,q0s;

always @(posedge geclk1 or negedge setn or posedge rst)begin
	if(!setn) begin
        q7s <= 1'b1;
        q5s <= 1'b1;
        q3s <= 1'b1;
        q1s <= 1'b1;
    end else if(rst) begin 
        q7s <= 1'b0;
        q5s <= 1'b0;
        q3s <= 1'b0;
        q1s <= 1'b0;
	end else if(update) begin
        q7s <= q7;
        q5s <= q5;
        q3s <= q3;
        q1s <= q1;
    end
end

always @(posedge geclk0 or negedge setn or posedge rst)begin
	if(!setn) begin
        q6s <= 1'b1;
        q4s <= 1'b1;
        q2s <= 1'b1;
        q0s <= 1'b1;
    end else if (rst) begin
        q6s <= 1'b0;
        q4s <= 1'b0;
        q2s <= 1'b0;
        q0s <= 1'b0;
	end else if(update) begin
        q6s <= q6;
        q4s <= q4;
        q2s <= q2;
        q0s <= q0;
    end
end

reg p0, p2, p4, p6;
reg p1, p3, p5, p7;
always @(posedge gsclk1 or negedge setn or posedge rst)begin
	if(!setn) begin
        p7 <= 1'b1;
        p5 <= 1'b1;
        p3 <= 1'b1;
        p1 <= 1'b1;
    end else if(rst) begin
        p7 <= 1'b0;
        p5 <= 1'b0;
        p3 <= 1'b0;
        p1 <= 1'b0;
	end else begin
        p7 <= q7s;
        p5 <= q5s;
        p3 <= q3s;
        p1 <= q1s;
    end
end
always @(posedge gsclk0 or negedge setn or posedge rst)begin
	if(!setn) begin
        p6 <= 1'b1;
        p4 <= 1'b1;
        p2 <= 1'b1;
        p0 <= 1'b1;
    end else if (rst) begin
        p6 <= 1'b0;
        p4 <= 1'b0;
        p2 <= 1'b0;
        p0 <= 1'b0;
	end else begin
        p6 <= q6s;
        p4 <= q4s;
        p2 <= q2s;
        p0 <= q0s;
    end
end

assign dataout[7] =	CFG_GEAR_IN[7] & p7;
assign dataout[6] =	CFG_GEAR_IN[6] & p6;
assign dataout[5] =	CFG_GEAR_IN[5] & p5;
assign dataout[4] =	CFG_GEAR_IN[4] & p4;
assign dataout[3] =	CFG_GEAR_IN[3] & p3;
assign dataout[2] =	CFG_GEAR_IN[2] & p2;
assign dataout[1] = CFG_GEAR_IN[1] ? p1 : do_in;
assign dataout[0] = CFG_GEAR_IN[0] ? p0 : de_in;

assign sout = {do_in, de_in, do_pre, de_pre, q7, q6, q5, q4, q3, q2};

endmodule

// Library - P2_IO, Cell - IO_HP_LVPECL, View - schematic
// LAST TIME SAVED: Feb 26 14:48:06 2022
// NETLIST TIME: Feb 26 17:09:50 2022
//ref: /home_hw/hme_center/hw/p2_center/circuit/verilog/array_block_cut_long/IO_HP_LVPECL.v

module IO_HP_LVPECL (
        /* basic IO function */ 
        PAD0, PAD1, 
        ODT_0, ODT_1, TED0, TED1, TXD0, TXD1,
        LP_RXD0, LP_RXD1, RXD0, RXD1,

        /* rx_lp control */
        fp_rx_lp_en,

        /* lvds_tx control */
        fp_lvds_tx_en,

        /* term control */
        fp_term_diff_en,

        /* input/output delay control */
        fp_idly_sel_0, fp_idly_sel_1, fp_idly_update_0, fp_idly_update_1,
        fp_odly_sel_0, fp_odly_sel_1, fp_odly_update_0, fp_odly_update_1,

        /* drive strength dynamic calibration control */
        fp_ndr_cal_0, fp_ndr_cal_1, fp_pdr_cal_0, fp_pdr_cal_1,
        //fp_pndr_rstn0, fp_pndr_rstn1, fp_tpud_rstn0,fp_tpud_rstn1,
       fp_pndr_update0, fp_pndr_update1,

        /* TP dynamic calibration control */
        fp_tpu_cal_0, fp_tpu_cal_1, fp_tpd_cal_0, fp_tpd_cal_1,
          fp_tpud_update0, fp_tpud_update1,

        /* TD dynamic control */
        fp_td_cal, 
        //fp_td_rstn, 
        fp_td_update
    );

inout  PAD0, PAD1; 
input  ODT_0, ODT_1, TED0, TED1, TXD0, TXD1;
output LP_RXD0, LP_RXD1, RXD0, RXD1;

/** outside RX_LP/LVDS_TX/TERM_DIFF dynamic control logic **/
//dyn control ports from xbar (move here from ../io_guts_lvds)
//config paras (move here from ../io_guts_lvds)
input fp_lvds_tx_en;
parameter CFG_DYN_LVDS_TX_EN = 1'b0;
parameter CFG_LVDS_TX_EN = 1'b0;

input fp_rx_lp_en;
parameter CFG_DYN_LP0 = 1'b0;
parameter CFG_RX_LP_EN_0 = 1'b0;
parameter CFG_DYN_LP1 = 1'b0;
parameter CFG_RX_LP_EN_1= 1'b0;

input fp_term_diff_en;
parameter CFG_DYN_TERM_EN = 1'b0;
parameter CFG_TERM_DIFF_EN = 1'b0;

/** outside INPUT/OUT Delay dynamic ctrol logic **/
//dyn control ports from xbar (move here from ../io_guts_lvds)
input [4:0] fp_idly_sel_0;
input [4:0] fp_idly_sel_1;
input fp_idly_update_0;
input fp_idly_update_1;
input [4:0] fp_odly_sel_0;
input [4:0] fp_odly_sel_1;
input fp_odly_update_0;
input fp_odly_update_1;
//config paras (move here from ../io_guts_lvds)
parameter CFG_IDLC0_BYPASS = 1'b0;
parameter CFG_IDLC0_DEL_SEL = 5'b00000;
parameter CFG_IDLC0_MODE = 1'b0;
parameter CFG_IDLC1_BYPASS = 1'b0;
parameter CFG_IDLC1_DEL_SEL = 5'b00000;
parameter CFG_IDLC1_MODE = 1'b0;
parameter CFG_ODLC0_BYPASS = 1'b0;
parameter CFG_ODLC0_DEL_SEL = 5'b00000;
parameter CFG_ODLC0_MODE = 1'b0;
parameter CFG_ODLC1_BYPASS = 1'b0;
parameter CFG_ODLC1_DEL_SEL = 5'b00000;
parameter CFG_ODLC1_MODE = 1'b0;

/* self logics */
//dyn ports
input [6:0]  fp_ndr_cal_0;
input [6:0]  fp_ndr_cal_1;
input [6:0]  fp_pdr_cal_0;
input [6:0]  fp_pdr_cal_1;
//input fp_pndr_rstn0;
//input fp_pndr_rstn1;
input fp_pndr_update0;
input fp_pndr_update1;
parameter CFG_NDR_0 = 7'b0000000;
parameter CFG_NDR_1 = 7'b0000000;
parameter CFG_PDR_0 = 7'b0000000;
parameter CFG_PDR_1 = 7'b0000000;
parameter CFG_PNDR_CAL_EN0 = 1'b0;
parameter CFG_PNDR_CAL_EN1 = 1'b0;
parameter CFG_PNDR_UD_BYP0 = 1'b0;
parameter CFG_PNDR_UD_BYP1 = 1'b0;

input [6:0]  fp_tpu_cal_0;
input [6:0]  fp_tpu_cal_1;
input [6:0]  fp_tpd_cal_0;
input [6:0]  fp_tpd_cal_1;
//input fp_tpud_rstn0;
//input fp_tpud_rstn1;
input fp_tpud_update0;
input fp_tpud_update1;
parameter CFG_TPD_0 = 7'b0000000;
parameter CFG_TPD_1 = 7'b0000000;
parameter CFG_TPU_0 = 7'b0000000;
parameter CFG_TPU_1 = 7'b0000000;
parameter CFG_TPD_CAL_EN0 = 1'b0;
parameter CFG_TPD_CAL_EN1 = 1'b0;
parameter CFG_TPU_CAL_EN0 = 1'b0;
parameter CFG_TPU_CAL_EN1 = 1'b0;
parameter CFG_TPUD_UD_BYP0 = 1'b0;
parameter CFG_TPUD_UD_BYP1 = 1'b0;

input [4:0]  fp_td_cal;
//input fp_td_rstn;
input fp_td_update;
parameter CFG_TD = 5'b00000;
parameter CFG_TD_CAL_EN = 1'b0;
parameter CFG_TD_UD_BYP = 1'b0;

//paras 
parameter CFG_CALRX_EN_0 = 1'b0;  //no use in p0
parameter CFG_CALRX_EN_1 = 1'b0;  //no use in p0
parameter CFG_RX_DIFF_EN = 1'b0;
parameter CFG_RX_DIG_EN_0 = 1'b0;
parameter CFG_RX_DIG_EN_1 = 1'b0;
parameter CFG_RX_SINGLE_EN_0 = 1'b0;
parameter CFG_RX_SINGLE_EN_1 = 1'b0;
parameter CFG_ST0 = 1'b0;
parameter CFG_ST1 = 1'b0;
parameter CFG_VREF_EN = 1'b0; //VRef en
//LVPECL support
parameter CFG_LVPECL_TX_EN = 1'b0;
parameter CFG_LVPECL_TERM_EN = 1'b0;
parameter CFG_LVPECL_TUNE_EN = 1'b0;
//common mode feadback
parameter CFG_CMFB_TX_EN = 1'b0;
//
parameter CFG_SMIT_TUNE_EN = 1'b0;
//support 0.9
parameter CFG_VCM0P9_EN = 1'b0;

parameter CFG_TX_POST_EN0 = 1'b0; //no use in p0
parameter CFG_TX_POST_EN1 = 1'b0; //no use in p0
parameter CFG_TX_PRE_EN0 = 1'b0;  //no use in p0
parameter CFG_TX_PRE_EN1 = 1'b0;  //no use in p0
parameter CFG_TX_POST0 = 3'b000;  //no use in p0
parameter CFG_TX_POST1 = 3'b000;  //no use in p0
parameter CFG_TX_PRE0 = 3'b000;   //no use in p0
parameter CFG_TX_PRE1 = 3'b000;   //no use in p0

parameter CFG_LDR = 2'b00;
parameter CFG_KEEP_0 = 2'b00;
parameter CFG_KEEP_1 = 2'b00;
parameter CFG_NS_LV_0 = 2'b00;
parameter CFG_NS_LV_1 = 2'b00;

/** X25 shared config bits (in io seam) **/
//TBD: move these paras to seam (DBUF/RBUF) ?
parameter CFG_INTREF_VREF_EN = 1'b0;
parameter CFG_INTREF_VREF_SEL = 4'b0000;
parameter CFG_ATEST_EN = 1'b0;
parameter CFG_ATEST_SEL = 4'b0000;
parameter CFG_IO_BIAS_EN = 1'b0;

/* dynamic config pdr/ndr */
wire [6:0] ndr_mux_0 = CFG_PNDR_CAL_EN0 == 1'b1 ? fp_ndr_cal_0 : CFG_NDR_0;
wire [6:0] pdr_mux_0 = CFG_PNDR_CAL_EN0 == 1'b1 ? fp_pdr_cal_0 : CFG_PDR_0;
wire [6:0] ndr_mux_1 = CFG_PNDR_CAL_EN1 == 1'b1 ? fp_ndr_cal_1 : CFG_NDR_1;
wire [6:0] pdr_mux_1 = CFG_PNDR_CAL_EN1 == 1'b1 ? fp_pdr_cal_1 : CFG_PDR_1;
reg [6:0] ndr_reg_0 = 7'bxxxxxxx;
reg [6:0] pdr_reg_0 = 7'bxxxxxxx;
always @ (posedge fp_pndr_update0) begin
    pdr_reg_0 <= pdr_mux_0;
    ndr_reg_0 <= ndr_mux_0;
end
reg [6:0] ndr_reg_1 = 7'bxxxxxxx;
reg [6:0] pdr_reg_1 = 7'bxxxxxxx;
always @ (posedge fp_pndr_update1) begin
    pdr_reg_1 <= pdr_mux_1;
    ndr_reg_1 <= ndr_mux_1;
end
wire [6:0] ndr_0 = CFG_PNDR_UD_BYP0 == 1'b1 ? ndr_mux_0 : ndr_reg_0;
wire [6:0] pdr_0 = CFG_PNDR_UD_BYP0 == 1'b1 ? pdr_mux_0 : pdr_reg_0;
wire [6:0] ndr_1 = CFG_PNDR_UD_BYP1 == 1'b1 ? ndr_mux_1 : ndr_reg_1;
wire [6:0] pdr_1 = CFG_PNDR_UD_BYP1 == 1'b1 ? pdr_mux_1 : pdr_reg_1;

/* dynamic config tpu/tpd */
wire [6:0] tpd_mux_0 = CFG_TPD_CAL_EN0 == 1'b1 ? fp_tpd_cal_0 : CFG_TPD_0;
wire [6:0] tpu_mux_0 = CFG_TPU_CAL_EN0 == 1'b1 ? fp_tpu_cal_0 : CFG_TPU_0;
wire [6:0] tpd_mux_1 = CFG_TPD_CAL_EN1 == 1'b1 ? fp_tpd_cal_1 : CFG_TPD_1;
wire [6:0] tpu_mux_1 = CFG_TPU_CAL_EN1 == 1'b1 ? fp_tpu_cal_1 : CFG_TPU_1;
reg [6:0] tpd_reg_0 = 7'bxxxxxxx;
reg [6:0] tpu_reg_0 = 7'bxxxxxxx;
always @ (posedge fp_tpud_update0) begin
    tpd_reg_0 <= tpd_mux_0;
    tpu_reg_0 <= tpu_mux_0;
end
reg [6:0] tpd_reg_1 = 7'bxxxxxxx;
reg [6:0] tpu_reg_1 = 7'bxxxxxxx;
always @ (posedge fp_tpud_update1) begin
    tpd_reg_1 <= tpd_mux_1;
    tpu_reg_1 <= tpu_mux_1;
end
wire [6:0] tpd_0 = CFG_TPUD_UD_BYP0 == 1'b1 ? tpd_mux_0 : tpd_reg_0;
wire [6:0] tpu_0 = CFG_TPUD_UD_BYP0 == 1'b1 ? tpu_mux_0 : tpu_reg_0;
wire [6:0] tpd_1 = CFG_TPUD_UD_BYP1 == 1'b1 ? tpd_mux_1 : tpd_reg_1;
wire [6:0] tpu_1 = CFG_TPUD_UD_BYP1 == 1'b1 ? tpu_mux_1 : tpu_reg_1;


/* dynamic config td */
wire [4:0] td_mux = CFG_TD_CAL_EN == 1'b1 ? fp_td_cal : CFG_TD;
reg [4:0] td_reg = 5'bxxxxx;
always @ (fp_td_update)
    td_reg <= td_mux;
wire td = CFG_TD_UD_BYP == 1'b1 ? td_mux : td_reg;

/* dynamic lvds_tx_en */
wire lvds_tx_en = CFG_DYN_LVDS_TX_EN == 1'b1 ? fp_lvds_tx_en : CFG_LVDS_TX_EN;

/* dynamic rx_lp en */
wire rx_lp_en_0 = CFG_DYN_LP0 == 1'b1 ? fp_rx_lp_en : CFG_RX_LP_EN_0;
wire rx_lp_en_1 = CFG_DYN_LP1 == 1'b1 ? fp_rx_lp_en : CFG_RX_LP_EN_1;

/* dynamic term_diff en */
wire term_diff_en = CFG_DYN_TERM_EN == 1'b1 ? fp_term_diff_en : CFG_TERM_DIFF_EN;

/* dynamic delay config */
reg [4:0] idly_reg_0 = 5'bxxxxx;
always @ (posedge fp_idly_update_0)
    idly_reg_0 <= fp_idly_sel_0;
reg [4:0] odly_reg_0 = 5'bxxxxx;
always @ (posedge fp_odly_update_0)
    odly_reg_0 <= fp_odly_sel_0;
reg [4:0] idly_reg_1 = 5'bxxxxx;
always @ (posedge fp_idly_update_1)
    idly_reg_1 <= fp_idly_sel_1;
reg [4:0] odly_reg_1 = 5'bxxxxx;
always @ (posedge fp_odly_update_1)
    odly_reg_1 <= fp_odly_sel_1;

wire [4:0] idly_mux0 = CFG_IDLC0_MODE == 1'b1 ? idly_reg_0 : CFG_IDLC0_DEL_SEL;
wire [4:0] odly_mux0 = CFG_ODLC0_MODE == 1'b1 ? odly_reg_0 : CFG_ODLC0_DEL_SEL;
wire [4:0] idly_mux1 = CFG_IDLC1_MODE == 1'b1 ? idly_reg_1 : CFG_IDLC1_DEL_SEL;
wire [4:0] odly_mux1 = CFG_ODLC1_MODE == 1'b1 ? odly_reg_1 : CFG_ODLC1_DEL_SEL;

wire  rxd0_dly, rxd1_dly, txd0_dly, txd1_dly;
reg   rxd0_in, rxd1_in;
wire  txd0_out, txd1_out;
delay_block Irx0(.in(rxd0_in), .sel(idly_mux0), .out(rxd0_dly));
delay_block Itx0(.in(TXD0), .sel(odly_mux0), .out(txd0_dly));
delay_block Irx1(.in(rxd1_in), .sel(idly_mux1), .out(rxd1_dly));
delay_block Itx1(.in(TXD1), .sel(odly_mux1), .out(txd1_dly));

assign RXD0 = CFG_IDLC0_BYPASS == 1'b1 ? rxd0_in : rxd0_dly;
assign RXD1 = CFG_IDLC1_BYPASS == 1'b1 ? rxd1_in : rxd1_dly;
assign txd0_out = CFG_ODLC0_BYPASS == 1'b1 ? TXD0 : txd0_dly;
assign txd1_out = CFG_ODLC1_BYPASS == 1'b1 ? TXD1 : txd1_dly;

///////////////////////////////////////////////////////////////
// IO logics
///////////////////////////////////////////////////////////////

//paramter check
//wire [2:0] rxen_sel0 = {CFG_RX_DIFF_EN, CFG_RX_SINGLE_EN_0, CFG_RX_DIG_EN_0};
//wire [2:0] rxen_sel1 = {CFG_RX_DIFF_EN, CFG_RX_SINGLE_EN_1, CFG_RX_DIG_EN_1};
wire [2:0] rxen_sel0;
wire [2:0] rxen_sel1;
assign rxen_sel0[0] = CFG_RX_DIG_EN_0;
assign rxen_sel0[1] = CFG_RX_SINGLE_EN_0;
assign rxen_sel0[2] = CFG_RX_DIFF_EN;
assign rxen_sel1[0] = CFG_RX_DIG_EN_1;
assign rxen_sel1[1] = CFG_RX_SINGLE_EN_1;
assign rxen_sel1[2] = CFG_RX_DIFF_EN;

assign LP_RXD0 = rx_lp_en_0 == 1'b1 ? rxd0_in : 1'b0;
assign LP_RXD1 = rx_lp_en_1 == 1'b1 ? rxd1_in : 1'b0;

reg   PAD0_reg, PAD1_reg;         //if use always, output must be "reg"
wire   un_define0, un_define1;   //input is "x" or "z"

wire rx_en0 = rxen_sel0 == 3'b100 ? (1'b1 & CFG_VREF_EN) :
              rxen_sel0 == 3'b010 ? (1'b1 & CFG_VREF_EN) :
              rxen_sel0 == 3'b001 ? rx_lp_en_0 : 1'b0;
wire rx_en1 = rxen_sel1 == 3'b100 ? (1'b1 & CFG_VREF_EN) :
              rxen_sel1 == 3'b010 ? (1'b1 & CFG_VREF_EN) :
              rxen_sel1 == 3'b001 ? rx_lp_en_1 : 1'b0;

wire pndr_en0 = (ndr_0 > 0 & pdr_0 > 0);
wire pndr_en1 = (ndr_1 > 0 & pdr_1 > 0);

wire odt_ctrl_0 = ODT_0 === 1'b1 ? 1'b1 : 1'b0;
wire odt_ctrl_1 = ODT_1 === 1'b1 ? 1'b1 : 1'b0;

wire cmos_tx_en0   = (CFG_NS_LV_0 != 2'b00) && (~TED0) && pndr_en0 &&
                     (~(lvds_tx_en | CFG_LVPECL_TX_EN | CFG_CMFB_TX_EN | odt_ctrl_0));
wire cmos_tx_en1   = (CFG_NS_LV_1 != 2'b00) && (~TED1) && pndr_en1 &&
                     (~(lvds_tx_en | CFG_LVPECL_TX_EN | CFG_CMFB_TX_EN | odt_ctrl_1));
wire ddr_tx_en0    = (CFG_NS_LV_0 != 2'b00 && CFG_VREF_EN == 1'b1) && (~TED0) && pndr_en0 && odt_ctrl_0 && 
                     (~(lvds_tx_en | CFG_LVPECL_TX_EN | CFG_CMFB_TX_EN));
wire ddr_tx_en1    = (CFG_NS_LV_1 != 2'b00 && CFG_VREF_EN == 1'b1) && (~TED1) && pndr_en1 && odt_ctrl_1 &&
                     (~(lvds_tx_en | CFG_LVPECL_TX_EN | CFG_CMFB_TX_EN));
wire lvds_tx_en0   = (CFG_VREF_EN == 1'b1) && (TED0) && pndr_en0 &&
                     (lvds_tx_en & CFG_CMFB_TX_EN) && (~(CFG_LVPECL_TX_EN | odt_ctrl_0));
wire lvds_tx_en1   = (CFG_VREF_EN == 1'b1) && (TED1) && pndr_en1 &&
                     (lvds_tx_en & CFG_CMFB_TX_EN) & (~(CFG_LVPECL_TX_EN | odt_ctrl_1));
wire lvpecl_tx_en0 = (TED0) && pndr_en0 &&
                     (lvds_tx_en & CFG_LVPECL_TX_EN) & (~(CFG_CMFB_TX_EN | odt_ctrl_0));
wire lvpecl_tx_en1 = (TED1) && pndr_en1 &&
                     (lvds_tx_en & CFG_LVPECL_TX_EN) & (~(CFG_CMFB_TX_EN | odt_ctrl_1));

assign un_define0 = (&CFG_KEEP_0) & (&ndr_0) & (&pdr_0) & (&tpd_0) & (&tpu_0) & (&td);
assign un_define1 = (&CFG_KEEP_1) & (&ndr_1) & (&pdr_1) & (&tpd_1) & (&tpu_1) & (&td);

always @(*) begin
    if(un_define0 == 1'bx || un_define0 == 1'bz) //unconnected input
        PAD0_reg  =  1'bz;
    else if(CFG_KEEP_0 == 2'b01) //pull down
        PAD0_reg  =  1'b0;
    else if(CFG_KEEP_0 == 2'b10) //pull up
        PAD0_reg  =  1'b1;
    else if(CFG_KEEP_0 == 2'b11) //keep
        PAD0_reg  =  PAD0_reg;
    else if(cmos_tx_en0 || ddr_tx_en0)   //normal output
        PAD0_reg  =  txd0_out;
    else if(lvds_tx_en0 || lvpecl_tx_en0)   //lvds
        PAD0_reg  =  txd0_out;
    else
        PAD0_reg  =  1'bz;
end
always @(*) begin
    if(un_define1 == 1'bx || un_define1 == 1'bz) //unconnected input
        PAD1_reg  =  1'bz;
    else if(CFG_KEEP_1 == 2'b01) //pull down
        PAD1_reg  =  1'b0;
    else if(CFG_KEEP_1 == 2'b10) //pull up
        PAD1_reg  =  1'b1;
    else if(CFG_KEEP_1 == 2'b11) //keep
        PAD1_reg  =  PAD1_reg;
    else if(cmos_tx_en1 || ddr_tx_en1)   //normal output
        PAD1_reg  =  txd1_out;
    else if(lvds_tx_en1 || lvpecl_tx_en1)   //lvds
        PAD1_reg  =  ~txd0_out;
    else
        PAD1_reg  =  1'bz;
end

assign PAD0  = PAD0_reg;
always @(*) begin
    if(un_define0 == 1'bx || un_define0 == 1'bz) //unconnected input
        rxd0_in  =  1'bz;
    else if(rx_en0)   //normal input
        rxd0_in =  PAD0;
    else 
        rxd0_in  =  1'b0;
end
assign PAD1  = PAD1_reg;
always @(*) begin
    if(un_define1 == 1'bx || un_define1 == 1'bz) //unconnected input
        rxd1_in  =  1'bz;
    else if(rx_en1)   //normal input
        rxd1_in =  PAD1;
    else 
        rxd1_in  =  1'b0;
end

endmodule

// Library - P2_IOC, Cell - IPATH, View - schematic
// LAST TIME SAVED: Feb 11 13:19:03 2022
// NETLIST TIME: Feb 26 17:09:54 2022
`timescale 1ns / 1ps 

module IPATH ( dataout, CFG_DDR_IN_NREG, CFG_DDR_IN_NREG_DFF,
     CFG_DDR_IN_PREG, CFG_DDR_IN_PREG_DFF, CFG_FASTIN_0, CFG_FASTIN_1,
     datain, fclk0, fclk1, rst, set_ );


input  CFG_DDR_IN_NREG, CFG_DDR_IN_NREG_DFF, CFG_DDR_IN_PREG,
     CFG_DDR_IN_PREG_DFF, CFG_FASTIN_0, CFG_FASTIN_1, 
     datain, fclk0, fclk1, rst, set_;

output [1:0]  dataout;

reg [1:0] even;
reg [1:0] odd;

always @ (posedge fclk0 or posedge rst or negedge set_) begin
    if(!set_)
        even <= 2'b11;
    else if(rst)
        even <= 2'b00;
    else
        even <= {even[0], datain};
end

always @ (negedge fclk1 or posedge rst or negedge set_) begin
    if(!set_)
        odd[0] <= 1'b1;
    else if(rst)
        odd[0] <= 1'b0;
    else
        odd[0] <= datain;
end
always @ (posedge fclk1 or posedge rst or negedge set_) begin
    if(!set_)
        odd[1] <= 1'b1;
    else if(rst)
        odd[1] <= 1'b0;
    else
        odd[1] <= odd[0];
end

wire [2:0] sel0 = {CFG_DDR_IN_PREG_DFF, CFG_DDR_IN_PREG, CFG_FASTIN_0};
wire [2:0] sel1 = {CFG_DDR_IN_NREG_DFF, CFG_DDR_IN_NREG, CFG_FASTIN_1};

assign dataout[0] = sel0 == 3'b001 ? datain  :
                    sel0 == 3'b010 ? even[0] :
                    sel0 == 3'b100 ? even[1] :
                    sel0 == 3'b000 ? 1'b1    :
                    1'bx;

assign dataout[1] = sel1 == 3'b001 ? datain :
                    sel1 == 3'b010 ? odd[0] :
                    sel1 == 3'b100 ? odd[1] :
                    sel1 == 3'b000 ? 1'b1    :
                    1'bx;

endmodule

// ref: /home_hw/hme_center/hw/p2_center/circuit/verilog/array_block_cut_long/key_detec_top.v

module key_detec_top (det_enb_out, det_out, rst0_o, rst1_o,
      det_clk, fp_det_en, rst0, rst1, data
     );

output  det_enb_out, det_out, rst0_o, rst1_o;

input  det_clk, fp_det_en, rst0, rst1;
input [15:0]  data;

parameter cfg_key_word = 16'b0000_0000_0000_0000;
parameter cfg_detect_mode = 16'b0000_0000_0000_0000;
parameter cfg_detect_cont = 1'b0;
parameter cfg_detect_en = 1'b0;

/* ** outside clock gating logic ** */
//move ../lbuf.clkgate_block logic to here
parameter CFG_FCLK_DET_EN = 1'b0; //CFG_FCLK_DET, to key_detect_top.detect_clk

//wire w_det_enb;
////in update_block_cnt_det block, I68.di (detect_rstb) = !(!(det_enb | det_out) & cfg_detect_en)
////To clarify the logic, change the logic to 
//// (det_enb | !cfg_detect_en) | det_out
//// and move "det_enb | !cfg_detect"_en to here
//assign det_enb = w_det_enb | !cfg_detect_en;

wire detect_clk = CFG_FCLK_DET_EN & det_clk;
wire detect_en = cfg_detect_en & fp_det_en;

reg [1:0] en_dly = 0;
always @ (posedge detect_clk) begin
    en_dly <= {en_dly[0], detect_en};
end

wire [15:0] comp;
assign comp[0]  = ~cfg_detect_mode[0]  | (cfg_key_word[0]  ==  data[0]);
assign comp[1]  = ~cfg_detect_mode[1]  | (cfg_key_word[1]  ==  data[1]);
assign comp[2]  = ~cfg_detect_mode[2]  | (cfg_key_word[2]  ==  data[2]);
assign comp[3]  = ~cfg_detect_mode[3]  | (cfg_key_word[3]  ==  data[3]);
assign comp[4]  = ~cfg_detect_mode[4]  | (cfg_key_word[4]  ==  data[4]);
assign comp[5]  = ~cfg_detect_mode[5]  | (cfg_key_word[5]  ==  data[5]);
assign comp[6]  = ~cfg_detect_mode[6]  | (cfg_key_word[6]  ==  data[6]);
assign comp[7]  = ~cfg_detect_mode[7]  | (cfg_key_word[7]  ==  data[7]);
assign comp[8]  = ~cfg_detect_mode[8]  | (cfg_key_word[8]  ==  data[8]);
assign comp[9]  = ~cfg_detect_mode[9]  | (cfg_key_word[9]  ==  data[9]);
assign comp[10] = ~cfg_detect_mode[10] | (cfg_key_word[10] ==  data[10]);
assign comp[11] = ~cfg_detect_mode[11] | (cfg_key_word[11] ==  data[11]);
assign comp[12] = ~cfg_detect_mode[12] | (cfg_key_word[12] ==  data[12]);
assign comp[13] = ~cfg_detect_mode[13] | (cfg_key_word[13] ==  data[13]);
assign comp[14] = ~cfg_detect_mode[14] | (cfg_key_word[14] ==  data[14]);
assign comp[15] = ~cfg_detect_mode[15] | (cfg_key_word[15] ==  data[15]);

assign cmpr = (&comp) & en_dly[1];

wire detect = (~en_dly[1] & en_dly[0]);

reg match;
always @ (posedge detect_clk or posedge detect) begin
    if(detect)
        match <= 0;
    else if(~match || cfg_detect_cont)
        match <= cmpr;
    else
        match <= match;
end

assign rst0_o = detect | rst0;
assign rst1_o = detect | rst1;
assign det_enb_out = ~en_dly[1];
assign det_out = match;

endmodule

// Library - P2_IOC, Cell - odt_block, View - schematic
// LAST TIME SAVED: Sep 17 11:28:20 2021
// NETLIST TIME: Feb 26 17:09:47 2022
`timescale 1ns / 1ps 

module odt_block ( odt, CFG_ODT_EN, CFG_ODT_PHASE, fclk, phy_odt_ctrl );

output  odt;
input fclk, phy_odt_ctrl;

input CFG_ODT_EN;
input [3:0]  CFG_ODT_PHASE;

reg [3:0] odt_dff = 0;
always @ (posedge fclk) begin
    odt_dff <= {odt_dff[2:0], phy_odt_ctrl};
end

wire [3:0] sel;
assign sel = {CFG_ODT_PHASE[3], CFG_ODT_PHASE[2], CFG_ODT_PHASE[1], CFG_ODT_PHASE[0]};
wire odt_phase = sel == 4'b0000 ? 1'b0 :
                 sel == 4'b0001 ? odt_dff[0] :
                 sel == 4'b0010 ? odt_dff[1] :
                 sel == 4'b0100 ? odt_dff[2] :
                 sel == 4'b1000 ? odt_dff[3] : 1'bx;
assign odt = CFG_ODT_EN == 1'b1 ? phy_odt_ctrl : odt_phase ;

endmodule

// Library - P2_IOC, Cell - oen_block, View - schematic
// LAST TIME SAVED: Jan 11 18:58:30 2022
// NETLIST TIME: Feb 26 17:09:47 2022
`timescale 1ns / 1ps 

module oen_block ( f_oen, shiftout,
            CFG_DDR_OUT_REG, CFG_GEAR_OUT, CFG_OEN_INV, CFG_OEN_SEL,
            fclk, geclk, gsclk, rst, set_, oen, shiftin, update);

output  f_oen, shiftout;

input  CFG_DDR_OUT_REG, CFG_GEAR_OUT, CFG_OEN_INV;
input [3:0]  CFG_OEN_SEL;
input fclk, geclk, gsclk, rst, set_, shiftin, update;
input [3:0]  oen;

//iob_cmux_inv Iinvmux_3_ ( .out(oen_mux[3]), .VDDD(VDDD), .VSSD(VSSD),
//     .in(oen[3]), .inv(CFG_OEN_INV));
//iob_cmux_inv Iinvmux_2_ ( .out(oen_mux[2]), .VDDD(VDDD), .VSSD(VSSD),
//     .in(oen[2]), .inv(CFG_OEN_INV));
//iob_cmux_inv Iinvmux_1_ ( .out(oen_mux[1]), .VDDD(VDDD), .VSSD(VSSD),
//     .in(oen[1]), .inv(CFG_OEN_INV));
//iob_cmux_inv Iinvmux_0_ ( .out(oen_mux[0]), .VDDD(VDDD), .VSSD(VSSD),
//     .in(oen[0]), .inv(CFG_OEN_INV));

wire [3:0] oen_mux;
assign oen_mux[0] = CFG_OEN_INV == 1'b1 ? ~oen[0] : oen[0];
assign oen_mux[1] = CFG_OEN_INV == 1'b1 ? ~oen[1] : oen[1];
assign oen_mux[2] = CFG_OEN_INV == 1'b1 ? ~oen[2] : oen[2];
assign oen_mux[3] = CFG_OEN_INV == 1'b1 ? ~oen[3] : oen[3];

//ologic_half Ioe ( .qs0(oen_dff), .VDDD(VDDD), .VSSD(VSSD),
//     .di(oen_mux), .geclk(geclk), .gsclk(gsclk), .rst(rst),
//     .set_(set_), .shiftin(shiftin), .update(update),
//     .update_(update_));

reg [3:0] oen_gsclk = 0;
reg [3:0] oen_geclk = 0;
wire oen_dff;
always @ (posedge gsclk or posedge rst or negedge set_) begin
    if(~set_)
        oen_gsclk <= 4'b1111;
    else if(rst)
        oen_gsclk <= 4'b0000;
    else
        oen_gsclk <= oen_mux;
end
always @ (posedge geclk or posedge rst or negedge set_) begin
    if(~set_) begin
        oen_geclk <= 4'b1111;
    end else if(rst) begin
        oen_geclk <= 4'b0000;
    end else begin
        oen_geclk[3] <= update ? oen_gsclk[3] : shiftin;
        oen_geclk[2] <= update ? oen_gsclk[2] : oen_geclk[3];
        oen_geclk[1] <= update ? oen_gsclk[1] : oen_geclk[2];
        oen_geclk[0] <= update ? oen_gsclk[0] : oen_geclk[1];
    end
end
assign oen_dff = oen_geclk[0];

wire oen_m, oen_m2, oen_m4;
reg oen_m1 = 0, oen_m3 = 0;
//iob_mux2x1 Iemux0 ( .out(oen_m), .VDDD(VDDD), .VSSD(VSSD),
//     .in0(oen_mux[0]), .in1(oen_dff), .sel(CFG_GEAR_OUT));
assign oen_m = CFG_GEAR_OUT == 1'b1 ? oen_dff : oen_mux[0];

//iob_dffrs Iedfff0 ( .qx(oen_m1), .VDDD(VDDD), .VSSD(VSSD), .ck(fclk),
//     .di(oen_m), .rst(rst), .set_(set_));
//iob_dffrs Iedff1 ( .qx(oen_m3), .VDDD(VDDD), .VSSD(VSSD), .ck(fclk),
//     .di(oen_m2), .rst(rst), .set_(set_));
always @ (posedge fclk or posedge rst or negedge set_) begin
    if(~set_) begin
        oen_m1 <= 1'b1;
        oen_m3 <= 1'b1;
    end else if (rst) begin
        oen_m1 <= 1'b0;
        oen_m3 <= 1'b0;
    end else begin
        oen_m1 <= oen_m;
        oen_m3 <= oen_m2;
    end
end

//iob_mux2x1 Iemux1 ( .out(oen_m2), .VDDD(VDDD), .VSSD(VSSD),
//     .in0(oen_m), .in1(oen_m1), .sel(CFG_DDR_OUT_REG));
assign oen_m2 = CFG_DDR_OUT_REG == 1'b1 ? oen_m1 : oen_m;
//iob_nand2x1 I1 ( .Y(net68), .A(oen_m2), .B(oen_m3), .VDDD(VDDD),
//     .VSSD(VSSD));
//iob_invx2 I2 ( .Y(oen_mx4), .A(net68), .VDDD(VDDD), .VSSD(VSSD));
assign oen_m4 = oen_m2 & oen_m3;

//iob_invx1_hvt I6_3_ ( .Y(cfg_oen_selb[3]), .A(CFG_OEN_SEL[3]),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_invx1_hvt I6_2_ ( .Y(cfg_oen_selb[2]), .A(CFG_OEN_SEL[2]),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_invx1_hvt I6_1_ ( .Y(cfg_oen_selb[1]), .A(CFG_OEN_SEL[1]),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_invx1_hvt I6_0_ ( .Y(cfg_oen_selb[0]), .A(CFG_OEN_SEL[0]),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_tiel I4 ( .ZN(tielow), .VDDD(VDDD), .VSSD(VSSD));
//
//iob_mux4 I0 ( .Y(net66), .I0(tielow), .I1(oen_mx4), .I2(oen_m2),
//     .I3(oen_m3), .S0(CFG_OEN_SEL[0]), .S0B(cfg_oen_selb[0]),
//     .S1(CFG_OEN_SEL[1]), .S1B(cfg_oen_selb[1]), .S2(CFG_OEN_SEL[2]),
//     .S2B(cfg_oen_selb[2]), .S3(CFG_OEN_SEL[3]), .S3B(cfg_oen_selb[3]),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_nand2x2 I3 ( .Y(f_oen), .A(net66), .B(usermode), .VDDD(VDDD),
//     .VSSD(VSSD));
wire [3:0] sel = {CFG_OEN_SEL[3], CFG_OEN_SEL[2], CFG_OEN_SEL[1], CFG_OEN_SEL[0]};
assign f_oen = sel == 4'b0000 ? 1'b1   :
               sel == 4'b0001 ? 1'b0   :
               sel == 4'b0010 ? oen_m4 :
               sel == 4'b0100 ? oen_m2 :
               sel == 4'b1000 ? oen_m3 : 1'bx;
//iob_invx1 I8 ( .Y(net67), .A(oen_dff), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx4 I9 ( .Y(shiftout), .A(net67), .VDDD(VDDD), .VSSD(VSSD));
assign shiftout = oen_dff;
endmodule

// Library - P2_IOC, Cell - oen_path, View - schematic
// LAST TIME SAVED: Jun 15 11:18:08 2021
// NETLIST TIME: Feb 26 17:09:54 2022
`timescale 1ns / 1ps 

module oen_path ( f_oen, CFG_DDR_OUT_REG, CFG_OEN_INV, CFG_OEN_SEL,
     fclk, oen, rst, set_);

output  f_oen;

input  CFG_DDR_OUT_REG, CFG_OEN_INV, fclk, oen, rst, set_;

input [3:0]  CFG_OEN_SEL;

wire oen0, oen2, oen4;
assign oen0 = CFG_OEN_INV == 1'b1 ? ~oen : oen;
reg oen1, oen3;

always @ (posedge fclk or posedge rst or negedge set_) begin
    if(~set_)
        oen1 <= 1'b1;
    else if(rst)
        oen1 <= 1'b0;
    else
        oen1 <= oen0;
end

assign oen2 = CFG_DDR_OUT_REG == 1'b0 ? oen1 : oen0;

always @ (posedge fclk or posedge rst or negedge set_) begin
    if(~set_)
        oen3 <= 1'b1;
    else if(rst)
        oen3 <= 1'b0;
    else
        oen3 <= oen2;
end

assign oen4 = oen2 & oen3;

assign f_oen = CFG_OEN_SEL == 4'b0000 ? 1'b1 :
               CFG_OEN_SEL == 4'b0001 ? 1'b0 :
               CFG_OEN_SEL == 4'b0010 ? oen4 :
               CFG_OEN_SEL == 4'b0100 ? oen2 :
               CFG_OEN_SEL == 4'b1000 ? oen3 : 1'bx;

endmodule


//-------------------------------------------------------
module OLOGIC (
	//input
	fclk0,
	fclk1,
	geclk0,
	geclk1,
	gsclk0,
	gsclk1,
	rst,
	setn,
	shiftin0,
	shiftin1,
	update,
	datain,
	//PARAMETER
	CFG_DDR_OUT,
	CFG_DDR_OUT_REG,
	CFG_FCLK_INV,
	CFG_FOUT_SEL,
	CFG_GEAR_OUT,
    //output
	dataout,
	shiftout0,
	shiftout1
	);
output	dataout;
output	shiftout0,shiftout1;

input	fclk0;
input	fclk1;
input	geclk0;
input	geclk1;
input	gsclk0;
input	gsclk1;
input	rst;
input	setn;
input	shiftin0,shiftin1;
input	update;
input	[7:0]	datain;

input	CFG_DDR_OUT_REG;
input	CFG_FOUT_SEL;
input	CFG_GEAR_OUT;
input	CFG_DDR_OUT;
input	CFG_FCLK_INV;

reg	qs7,qs6,qs5,qs4,qs3,qs2,qs1,qs0;
always @(posedge gsclk1 or negedge setn or posedge rst)begin
	if(~setn) begin
        qs7 <= 1'b1;
        qs5 <= 1'b1;
        qs3 <= 1'b1;
        qs1 <= 1'b1;
    end else if (rst) begin
        qs7 <= 1'b0;
        qs5 <= 1'b0;
        qs3 <= 1'b0;
        qs1 <= 1'b0;
	end else begin
        qs7 <= datain[7];
        qs5 <= datain[5];
        qs3 <= datain[3];
        qs1 <= datain[1];
    end
end
always @(posedge gsclk0 or negedge setn or posedge rst)begin
	if(~setn) begin
        qs6 <= 1'b1;
        qs4 <= 1'b1;
        qs2 <= 1'b1;
        qs0 <= 1'b1;
    end else if (rst) begin
        qs6 <= 1'b0;
        qs4 <= 1'b0;
        qs2 <= 1'b0;
        qs0 <= 1'b0;
	end else begin
        qs6 <= datain[6];
        qs4 <= datain[4];
        qs2 <= datain[2];
        qs0 <= datain[0];
    end
end

reg s0, s1, s2, s3, s4, s5, s6, s7;
always @(posedge geclk1 or negedge setn or posedge rst) begin
    if(~setn) begin
        s7 <= 1'b1;
        s5 <= 1'b1;
        s3 <= 1'b1;
        s1 <= 1'b1;
    end else if (rst) begin
        s7 <= 1'b0;
        s5 <= 1'b0;
        s3 <= 1'b0;
        s1 <= 1'b0;
    end else if (update) begin
        s7 <= qs7;
        s5 <= qs5;
        s3 <= qs3;
        s1 <= qs1;
    end else begin
        s7 <= shiftin1;
        s5 <= s7;
        s3 <= s5;
        s1 <= s3;
    end
end
always @(posedge geclk0 or negedge setn or posedge rst) begin
    if(~setn) begin
        s6 <= 1'b1;
        s4 <= 1'b1;
        s2 <= 1'b1;
        s0 <= 1'b1;
    end else if (rst) begin
        s6 <= 1'b0;
        s4 <= 1'b0;
        s2 <= 1'b0;
        s0 <= 1'b0;
    end else if (update) begin
        s6 <= qs6;
        s4 <= qs4;
        s2 <= qs2;
        s0 <= qs0;
    end else begin
        s6 <= shiftin0;
        s4 <= s6;
        s2 <= s4;
        s0 <= s2;
    end
end

assign shiftout0 = s0;
assign shiftout1 = s1;

wire	dp_0,dn_0,dp_2,dn_2;
reg		dp_1,dn_1,dp_3,dn_3;

assign dp_0 =	CFG_GEAR_OUT ? s0 : datain[0];
assign dn_0 =	CFG_GEAR_OUT ? s1 : datain[1];

always @(posedge fclk0 or posedge rst or negedge setn)begin
	if(!setn) dp_1 <= 1;
	else if(rst) dp_1 <= 0;
	else dp_1 <= dp_0;
end
always @(posedge fclk1 or posedge rst or negedge setn)begin
	if(!setn) dn_1 <= 1;
	else if(rst) dn_1 <= 0;
	else dn_1 <= dn_0;
end


assign dp_2 = CFG_DDR_OUT_REG ? dp_1 : dp_0;
assign dn_2 = CFG_DDR_OUT_REG ? dn_1 : dn_0;

always @(posedge fclk0 or posedge rst or negedge setn)begin
	if(!setn) dp_3 <= 1;
	else if(rst) dp_3 <= 0;
	else dp_3 <= dp_2;
end
always @(negedge fclk1 or posedge rst or negedge setn)begin
	if(!setn) dn_3 <= 1;
	else if(rst) dn_3 <= 0;
	else dn_3 <= dn_2;
end

wire	dp_out;
wire	pn_sel;
wire	pn_sel_finv;
assign pn_sel = !(CFG_DDR_OUT & fclk0);
assign pn_sel_finv = !(pn_sel & CFG_FCLK_INV);
assign dp_out =	(pn_sel_finv == 0) ? dp_3 :
				(pn_sel_finv == 1) ? dn_3 : 1'bx;

assign dataout = ~CFG_FOUT_SEL ? dp_out : datain[0];

endmodule

// Library - P2_IOC, Cell - OPATH, View - schematic
// LAST TIME SAVED: Jul 15 18:15:03 2021
// NETLIST TIME: Feb 26 17:09:54 2022
`timescale 1ns / 1ps 

module OPATH ( dataout, CFG_DDR_OUT, CFG_DDR_OUT_REG, CFG_FCLK_INV,
     CFG_FOUT_SEL, datain, fclk0, fclk1, rst, set_ );

output  dataout;

input  CFG_DDR_OUT, CFG_DDR_OUT_REG, CFG_FCLK_INV, CFG_FOUT_SEL,
     fclk0, fclk1, rst, set_;

input [1:0]  datain;

reg dp0, dp1;
reg dn0, dn1;

always @ (posedge fclk0 or posedge rst or negedge set_) begin
    if(~set_) 
        dp0 <= 1'b1;
    else if(rst)
        dp0 <= 1'b0;
    else
        dp0 <= datain[0];
end
always @ (posedge fclk0 or posedge rst or negedge set_) begin
    if(~set_) 
        dp1 <= 1'b1;
    else if(rst)
        dp1 <= 1'b0;
    else
        dp1 <= CFG_DDR_OUT_REG == 1'b0 ? dp0 : datain[0];
end

always @ (posedge fclk1 or posedge rst or negedge set_) begin
    if(~set_) 
        dn0 <= 1'b1;
    else if(rst)
        dn0 <= 1'b0;
    else
        dn0 <= datain[1];
end
always @ (negedge fclk1 or posedge rst or negedge set_) begin
    if(~set_) 
        dn1 <= 1'b1;
    else if(rst)
        dn1 <= 1'b0;
    else
        dn1 <= CFG_DDR_OUT_REG == 1'b0 ? dn0 : datain[1];
end

wire pnsel = ~( (~(CFG_DDR_OUT & fclk0)) & CFG_FCLK_INV );
wire ddro = pnsel ? dn1 : dp1;
assign dataout = CFG_FOUT_SEL == 1'b1 ? datain[0] : ddro;

endmodule

// Library - P2_IOC, Cell - update_block_cnt_det, View - schematic
// LAST TIME SAVED: May 14 14:44:14 2022
// NETLIST TIME: May 16 22:57:26 2022

module update_block_cnt_det ( sysclk_adjust_retval, update, update_b,
     align_rst, align_user, 
     det_enb, det_out, geclk, gsclk);

output  sysclk_adjust_retval, update, update_b;

input  align_rst, align_user, det_enb,
     det_out, geclk, gsclk; 

parameter CFG_UP_SEL = 7'b000_0000;

parameter CFG_GSCLK_UPI_EN = 1'b0; //to update_block_cnt_det.gsclk
parameter CFG_GECLK_UPI_EN = 1'b0; //to update_block_cnt_det.geclk

parameter CFG_RST_EN = 1'b0; //from: lbuf1.CFG_RST_EN_I[1]

wire w_geclk, w_gsclk, w_align_rst;
assign w_geclk = CFG_GECLK_UPI_EN == 1'b1 ? geclk : 1'b0;
assign w_gsclk = CFG_GSCLK_UPI_EN == 1'b1 ? gsclk : 1'b0;
assign w_align_rst = CFG_RST_EN == 1'b1 ? align_rst : 1'b0;

//wire #1 usermode = 1'b1;
wire usermode;
glbsr u_gsr(.GSR(usermode));

// Buses in the design


//iob_invx2 I34 ( .Y(net077), .A(geclk), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx16 I35 ( .Y(geclk_buf), .A(net077), .VDDD(VDDD), .VSSD(VSSD));
wire geclk_buf = w_geclk;

//iob_invx1 I36 ( .Y(net071), .A(usermode), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx4 I37 ( .Y(usermode_d), .A(net071), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx4 I38 ( .Y(usermode_b), .A(usermode_d), .VDDD(VDDD), .VSSD(VSSD));

wire usermode_b = ~usermode;

//iob_invx0 I45 ( net081, align_rst, VDDD, VSSD);
//iob_nand2x1 I39 ( .Y(net069), .A(net081), .B(usermode), .VDDD(VDDD),
//     .VSSD(VSSD));
//iob_invx2 I41 ( .Y(rst), .A(rst_), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx2 I40 ( .Y(rst_), .A(net069), .VDDD(VDDD), .VSSD(VSSD));
wire rst = ~(usermode & (~w_align_rst));

/*
iob_ndffr Ialign2 ( .qx(align2), .VDDD(VDDD), .VSSD(VSSD),
     .ck(geclk_buf), .di(align1), .rst(usermode_b));
iob_ndffr Ialign1 ( .qx(align1), .VDDD(VDDD), .VSSD(VSSD),
     .ck(geclk_buf), .di(align0), .rst(usermode_b));
iob_ndffr Ialign0 ( .qx(align0), .VDDD(VDDD), .VSSD(VSSD),
     .ck(geclk_buf), .di(align_user), .rst(usermode_b));
*/
reg align2, align1, align0;
always @ (negedge geclk_buf or posedge usermode_b) begin
    if(usermode_b) begin
        align2 <= 1'b0;
        align1 <= 1'b0;
        align0 <= 1'b0;
    end else begin
        align2 <= align1;
        align1 <= align0;
        align0 <= align_user;
    end
end

//iob_invx1 I8 ( .Y(net58), .A(align2), .VDDD(VDDD), .VSSD(VSSD));
//iob_nand2x1 I9 ( .Y(net57), .A(align1), .B(net58), .VDDD(VDDD),
//     .VSSD(VSSD));
//iob_invx2 I13 ( .Y(net56), .A(net57), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx16 I14 ( .Y(en), .A(net56), .VDDD(VDDD), .VSSD(VSSD));
wire en = ~(align1 & (~align2));

//iob_ndffrs I68 ( .qx(detect_rstb), .VDDD(VDDD), .VSSD(VSSD),
//     .ck(geclk_buf), .di(net0106), .rst(usermode_b), .set_(tiehi));
//iob_nand2x2_hvt I66 ( .Y(net0106), .A(net095), .B(cfg_detect_en),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_nor2x1_hvt I67 ( .Y(net095), .A(det_enb), .B(det_out), .VDDD(VDDD),
//     .VSSD(VSSD));
//wire net095 = ~(det_enb | det_out);
//wire net0106 = ~(net095 & cfg_detect_en);
wire net0106 = det_enb | det_out;

reg detect_rstb;
always @ (negedge geclk_buf or posedge usermode_b) begin
    if(usermode_b)
        detect_rstb <= 1'b0;
    else
        detect_rstb <= net0106;
end

/*
iob_ndffr I61 ( .qx(net070), .VDDD(VDDD), .VSSD(VSSD), .ck(geclk_buf),
     .di(cnt_rst), .rst(rst));
iob_nand2x2 I70 ( .Y(net55), .A(net070), .B(detect_rstb), .VDDD(VDDD),
     .VSSD(VSSD));
iob_invx4 I31 ( .Y(net54), .A(net55), .VDDD(VDDD), .VSSD(VSSD));
iob_s2d Iobuf ( .out(update), .outb(update_), .VDDD(VDDD), .VSSD(VSSD),
     .in(net54), .tiehi(tiehi), .tielow(tielow));
*/
reg net070;
wire cnt_rst;
always @ (negedge geclk_buf or posedge rst) begin
    if(rst)
        net070 <= 1'b0;
    else
        net070 <= cnt_rst;
end
//wire net55 = ~(net070 & detect_rstb);
//wire net54 = ~net55;
assign update = net070 & detect_rstb;
assign update_b = ~update;

//iob_invx4 I4 ( .Y(sysclk_adjust_retval), .A(net59), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx1 I44 ( .Y(net59), .A(net60), .VDDD(VDDD), .VSSD(VSSD));
//iob_ndffr It1 ( .qx(net60), .VDDD(VDDD), .VSSD(VSSD), .ck(gsclk),
//     .di(net61), .rst(usermode_b));
//iob_ndffr It0 ( .qx(net61), .VDDD(VDDD), .VSSD(VSSD), .ck(geclk_buf),
//     .di(update), .rst(usermode_b));
reg net60, net61;
always @ (negedge geclk_buf or posedge usermode_b) begin
    if(usermode_b)
        net61 <= 1'b0;
    else
        net61 <= update;
end
always @ (negedge w_gsclk or posedge usermode_b) begin
    if(usermode_b)
        net60 <= 1'b0;
    else
        net60 <= net61;
end
assign sysclk_adjust_retval = net60;


//iob_tiel Itl ( .ZN(tielow), .VDDD(VDDD), .VSSD(VSSD));
//iob_tieh Ith ( .Z(tiehi), .VDDD(VDDD), .VSSD(VSSD));

//iob_nand2x4 I59 ( .Y(cnt_rst), .A(net073), .B(detect_rstb),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_nand3x2 I58 ( .Y(net073), .A(net063), .B(net072), .C(net094),
//     .VDDD(VDDD), .VSSD(VSSD));
wire net063, net072, net094;
assign cnt_rst = ~(detect_rstb & ~(net063 & net072 & net094));

/*
iob_nor2x1 I52 ( .Y(net062), .A(Q[0]), .B(cnt_rst), .VDDD(VDDD),
     .VSSD(VSSD));
iob_nor2x1 I54 ( .Y(net060), .A(net079), .B(cnt_rst), .VDDD(VDDD),
     .VSSD(VSSD));
iob_nor2x1 I53 ( .Y(net061), .A(net082), .B(cnt_rst), .VDDD(VDDD),
     .VSSD(VSSD));

iob_ndffre Icnt_2_ ( .q(Q[2]), .VDDD(VDDD), .VSSD(VSSD),
     .clk(geclk_buf), .d(net060), .en(en), .rst(rst));
iob_ndffre Icnt_1_ ( .q(Q[1]), .VDDD(VDDD), .VSSD(VSSD),
     .clk(geclk_buf), .d(net061), .en(en), .rst(rst));
iob_ndffre Icnt_0_ ( .q(Q[0]), .VDDD(VDDD), .VSSD(VSSD),
     .clk(geclk_buf), .d(net062), .en(en), .rst(rst));
*/
wire net079, net082, net096;
wire net062, net061, net060;
reg [2:0] Q;
assign net062 = ~(Q[0] | cnt_rst);
assign net060 = ~(net079 | cnt_rst);
assign net061 = ~(net082 | cnt_rst);
always @ (negedge geclk_buf or posedge rst) begin
    if(rst)
        Q <= 3'b000;
    else if(en)
        Q <= {net060, net061, net062};
    else
        Q <= Q;
end
 
/*
iob_xnr2 I62 ( .ZN(net082), .A1(Q[1]), .A2(Q[0]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I63 ( .ZN(net079), .A1(Q[2]), .A2(net096), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I57 ( .ZN(net094), .A1(Q[2]), .A2(CFG_UP_SEL[2]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I55 ( .ZN(net063), .A1(Q[0]), .A2(CFG_UP_SEL[0]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I56 ( .ZN(net072), .A1(Q[1]), .A2(CFG_UP_SEL[1]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_and2x2 I64 ( .Y(net096), .A(Q[0]), .B(Q[1]), .VDDD(VDDD),
     .VSSD(VSSD));
*/
assign net063 = ~(Q[0] ^ CFG_UP_SEL[0]);
assign net072 = ~(Q[1] ^ CFG_UP_SEL[1]);
assign net094 = ~(Q[2] ^ CFG_UP_SEL[2]);

assign net082 = ~(Q[1] ^ Q[0]);
assign net079 = ~(Q[2] ^ net096);
assign net096 = Q[0] & Q[1];

endmodule
// Library - P2_IOC, Cell - update_block_cnt_det, View - schematic
// LAST TIME SAVED: Sep  8 18:31:29 2021
// NETLIST TIME: Feb 26 17:09:51 2022
// ref: /home_hw/hme_center/hw/p2_center/circuit/verilog/array_block_cut_long/update_block_cnt.v

module update_block_cnt(
     align_rst, align_user, geclk, gsclk,
     sysclk_adjust_retval, update, update_b
    );


input align_rst, align_user, geclk, gsclk;
output sysclk_adjust_retval, update, update_b;

parameter CFG_UP_SEL = 7'b0000000;

/* ** outside clock gating logic ** */
//move ../lbuf.clkgate_block logic to here
parameter CFG_GSCLK_UPO_EN = 1'b0; //to update_block_cnt.gsclk
parameter CFG_GECLK_UPO_EN = 1'b0; //to update_block_cnt.geclk

parameter CFG_RST_EN = 1'b0; //from: lbuf1.CFG_RST_EN_O[2]

wire w_geclk, w_gsclk, w_rst;
assign w_geclk = CFG_GECLK_UPO_EN == 1'b1 ? geclk : 1'b0;
assign w_gsclk = CFG_GSCLK_UPO_EN == 1'b1 ? gsclk : 1'b0;
assign w_rst = CFG_RST_EN == 1'b1 ? align_rst : 1'b0;

//wire #1 usermode = 1'b1;
wire usermode;
glbsr u_gsr(.GSR(usermode));

// Buses in the design


//iob_invx2 I34 ( .Y(net077), .A(geclk), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx16 I35 ( .Y(geclk_buf), .A(net077), .VDDD(VDDD), .VSSD(VSSD));
wire geclk_buf = w_geclk;

//iob_invx1 I36 ( .Y(net071), .A(usermode), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx4 I37 ( .Y(usermode_d), .A(net071), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx4 I38 ( .Y(usermode_b), .A(usermode_d), .VDDD(VDDD), .VSSD(VSSD));

wire usermode_b = ~usermode;

//iob_invx0 I45 ( net081, align_rst, VDDD, VSSD);
//iob_nand2x1 I39 ( .Y(net069), .A(net081), .B(usermode), .VDDD(VDDD),
//     .VSSD(VSSD));
//iob_invx2 I41 ( .Y(rst), .A(rst_), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx2 I40 ( .Y(rst_), .A(net069), .VDDD(VDDD), .VSSD(VSSD));
wire rst = ~(usermode & (~w_rst));

/*
iob_ndffr Ialign2 ( .qx(align2), .VDDD(VDDD), .VSSD(VSSD),
     .ck(geclk_buf), .di(align1), .rst(usermode_b));
iob_ndffr Ialign1 ( .qx(align1), .VDDD(VDDD), .VSSD(VSSD),
     .ck(geclk_buf), .di(align0), .rst(usermode_b));
iob_ndffr Ialign0 ( .qx(align0), .VDDD(VDDD), .VSSD(VSSD),
     .ck(geclk_buf), .di(align_user), .rst(usermode_b));
*/
reg align2, align1, align0;
always @ (negedge geclk_buf or posedge usermode_b) begin
    if(usermode_b) begin
        align2 <= 0;
        align1 <= 0;
        align0 <= 0;
    end else begin
        align2 <= align1;
        align1 <= align0;
        align0 <= align_user;
    end
end

//iob_invx1 I8 ( .Y(net58), .A(align2), .VDDD(VDDD), .VSSD(VSSD));
//iob_nand2x1 I9 ( .Y(net57), .A(align1), .B(net58), .VDDD(VDDD),
//     .VSSD(VSSD));
//iob_invx2 I13 ( .Y(net56), .A(net57), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx16 I14 ( .Y(en), .A(net56), .VDDD(VDDD), .VSSD(VSSD));
wire en = ~(align1 & (~align2));

//iob_ndffrs I68 ( .qx(detect_rstb), .VDDD(VDDD), .VSSD(VSSD),
//     .ck(geclk_buf), .di(net0106), .rst(usermode_b), .set_(tiehi));
//iob_nand2x2_hvt I66 ( .Y(net0106), .A(net095), .B(cfg_detect_en),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_nor2x1_hvt I67 ( .Y(net095), .A(det_enb), .B(det_out), .VDDD(VDDD),
//     .VSSD(VSSD));
//wire net095 = ~(det_enb | det_out);
//wire net0106 = ~(net095 & cfg_detect_en);
wire net0106 = 1'b1;//det_enb | det_out;

reg detect_rstb;
always @ (negedge geclk_buf or posedge usermode_b) begin
    if(usermode_b)
        detect_rstb <= 1'b0;
    else
        detect_rstb <= net0106;
end

/*
iob_ndffr I61 ( .qx(net070), .VDDD(VDDD), .VSSD(VSSD), .ck(geclk_buf),
     .di(cnt_rst), .rst(rst));
iob_nand2x2 I70 ( .Y(net55), .A(net070), .B(detect_rstb), .VDDD(VDDD),
     .VSSD(VSSD));
iob_invx4 I31 ( .Y(net54), .A(net55), .VDDD(VDDD), .VSSD(VSSD));
iob_s2d Iobuf ( .out(update), .outb(update_), .VDDD(VDDD), .VSSD(VSSD),
     .in(net54), .tiehi(tiehi), .tielow(tielow));
*/
reg net070;
wire cnt_rst;
always @ (negedge geclk_buf or posedge rst) begin
    if(rst)
        net070 <= 1'b0;
    else
        net070 <= cnt_rst;
end
//wire net55 = ~(net070 & detect_rstb);
//wire net54 = ~net55;
assign update = net070 & detect_rstb;
assign update_b = ~update;

//iob_invx4 I4 ( .Y(sysclk_adjust_retval), .A(net59), .VDDD(VDDD), .VSSD(VSSD));
//iob_invx1 I44 ( .Y(net59), .A(net60), .VDDD(VDDD), .VSSD(VSSD));
//iob_ndffr It1 ( .qx(net60), .VDDD(VDDD), .VSSD(VSSD), .ck(gsclk),
//     .di(net61), .rst(usermode_b));
//iob_ndffr It0 ( .qx(net61), .VDDD(VDDD), .VSSD(VSSD), .ck(geclk_buf),
//     .di(update), .rst(usermode_b));
reg net60, net61;
always @ (negedge geclk_buf or posedge usermode_b) begin
    if(usermode_b)
        net61 <= 1'b0;
    else
        net61 <= update;
end
always @ (negedge w_gsclk or posedge usermode_b) begin
    if(usermode_b)
        net60 <= 1'b0;
    else
        net60 <= net61;
end
assign sysclk_adjust_retval = net60;


//iob_tiel Itl ( .ZN(tielow), .VDDD(VDDD), .VSSD(VSSD));
//iob_tieh Ith ( .Z(tiehi), .VDDD(VDDD), .VSSD(VSSD));

//iob_nand2x4 I59 ( .Y(cnt_rst), .A(net073), .B(detect_rstb),
//     .VDDD(VDDD), .VSSD(VSSD));
//iob_nand3x2 I58 ( .Y(net073), .A(net063), .B(net072), .C(net094),
//     .VDDD(VDDD), .VSSD(VSSD));
wire net063, net072, net094;
assign cnt_rst = ~(detect_rstb & ~(net063 & net072 & net094));

/*
iob_nor2x1 I52 ( .Y(net062), .A(Q[0]), .B(cnt_rst), .VDDD(VDDD),
     .VSSD(VSSD));
iob_nor2x1 I54 ( .Y(net060), .A(net079), .B(cnt_rst), .VDDD(VDDD),
     .VSSD(VSSD));
iob_nor2x1 I53 ( .Y(net061), .A(net082), .B(cnt_rst), .VDDD(VDDD),
     .VSSD(VSSD));

iob_ndffre Icnt_2_ ( .q(Q[2]), .VDDD(VDDD), .VSSD(VSSD),
     .clk(geclk_buf), .d(net060), .en(en), .rst(rst));
iob_ndffre Icnt_1_ ( .q(Q[1]), .VDDD(VDDD), .VSSD(VSSD),
     .clk(geclk_buf), .d(net061), .en(en), .rst(rst));
iob_ndffre Icnt_0_ ( .q(Q[0]), .VDDD(VDDD), .VSSD(VSSD),
     .clk(geclk_buf), .d(net062), .en(en), .rst(rst));
*/
wire net079, net082, net096;
wire net062, net061, net060;
reg [2:0] Q;
assign net062 = ~(Q[0] | cnt_rst);
assign net060 = ~(net079 | cnt_rst);
assign net061 = ~(net082 | cnt_rst);
always @ (negedge geclk_buf or posedge rst) begin
    if(rst)
        Q <= 3'b000;
    else if(en)
        Q <= {net060, net061, net062};
    else
        Q <= Q;
end
 
/*
iob_xnr2 I62 ( .ZN(net082), .A1(Q[1]), .A2(Q[0]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I63 ( .ZN(net079), .A1(Q[2]), .A2(net096), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I57 ( .ZN(net094), .A1(Q[2]), .A2(CFG_UP_SEL[2]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I55 ( .ZN(net063), .A1(Q[0]), .A2(CFG_UP_SEL[0]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_xnr2 I56 ( .ZN(net072), .A1(Q[1]), .A2(CFG_UP_SEL[1]), .VDDD(VDDD),
     .VSSD(VSSD));
iob_and2x2 I64 ( .Y(net096), .A(Q[0]), .B(Q[1]), .VDDD(VDDD),
     .VSSD(VSSD));
*/
assign net063 = ~(Q[0] ^ CFG_UP_SEL[0]);
assign net072 = ~(Q[1] ^ CFG_UP_SEL[1]);
assign net094 = ~(Q[2] ^ CFG_UP_SEL[2]);

assign net082 = ~(Q[1] ^ Q[0]);
assign net079 = ~(Q[2] ^ net096);
assign net096 = Q[0] & Q[1];


endmodule

//*************************************************
//company:   capital-micro
//author:   hongyu.wang
//date:      20140526
//function:   `DSP
//*************************************************
//$Log: .v,v $


`ifndef DLY
`define DLY #0.1
`endif


module dsp_clkgate(
                      test_mode    ,
                      clk          ,
                      ce_pra       ,
                      ce_mult_u    ,
                      ce_mult_l    ,
                      ce_poa       ,
                      ce_comp      ,
                      ce_ufc       ,
                      ce_ibuf      ,
                      clk_pra      ,
                      clk_mult_u   ,
                      clk_mult_l   ,
                      clk_poa      ,
                      clk_comp     ,
                      clk_ufc      ,
                      clk_ibuf      
                  );
input   test_mode    ;
input   clk          ;
input   ce_pra       ;
input   ce_mult_u    ;
input   ce_mult_l    ;
input   ce_poa       ;
input   ce_comp      ;
input   ce_ufc       ;
input   ce_ibuf      ;

output  clk_pra      ;
output  clk_mult_u   ;
output  clk_mult_l   ;
output  clk_poa      ;
output  clk_comp     ;
output  clk_ufc      ;
output  clk_ibuf     ;


gclk_clk_gate U_gclk_clk_gate_pra (
                .test_mode       (test_mode  ),       
                .clk_gate_in     (clk        ),         
                .gate_en         (ce_pra     ),     
                .clk_gate_out    (clk_pra    )      
);


//----------------------------------------------
gclk_clk_gate U_gclk_clk_gate_mult_u (
                .test_mode       (test_mode     ),       
                .clk_gate_in     (clk           ),         
                .gate_en         (ce_mult_u     ),     
                .clk_gate_out    (clk_mult_u    )      
);



//----------------------------------------------
gclk_clk_gate U_gclk_clk_gate_mult_l (
                .test_mode       (test_mode     ),       
                .clk_gate_in     (clk           ),         
                .gate_en         (ce_mult_l     ),     
                .clk_gate_out    (clk_mult_l    )      
);


//----------------------------------------------
gclk_clk_gate U_gclk_clk_gate_poa (
                .test_mode       (test_mode     ),       
                .clk_gate_in     (clk           ),         
                .gate_en         (ce_poa     ),     
                .clk_gate_out    (clk_poa    )      
);


//----------------------------------------------
gclk_clk_gate U_gclk_clk_gate_comp (
                .test_mode       (test_mode     ),       
                .clk_gate_in     (clk           ),         
                .gate_en         (ce_comp     ),     
                .clk_gate_out    (clk_comp    )      
);


//----------------------------------------------
gclk_clk_gate U_gclk_clk_gate_ufc (
                .test_mode       (test_mode     ),       
                .clk_gate_in     (clk           ),         
                .gate_en         (ce_ufc     ),     
                .clk_gate_out    (clk_ufc    )      
);


//----------------------------------------------
gclk_clk_gate U_gclk_clk_gate_ibuf (
                .test_mode       (test_mode     ),       
                .clk_gate_in     (clk           ),         
                .gate_en         (ce_ibuf     ),     
                .clk_gate_out    (clk_ibuf    )      
);

endmodule   
//*************************************************
//company:   capital-micro
//author:   hongyu.wang
//date:      20140526
//function:   `DSP
//*************************************************
//$Log: .v,v $


`ifndef DLY
`define DLY #0.1
`endif


module dsp_compunit(
                       clk_comp        ,                          
                       rst_n      ,                            
                       R_pre      ,                            
                       U_buf      ,  
                       Z0H_U      ,
                       Z1H_U      ,
                       Z0H_L      ,
                       Z1H_L      ,
                       FRCTRL     ,                    
                       cfg_vldbit ,                                 
                       cfg_match  ,                                
                       cfg_imatch , 
                       cfg_poa_rout,                                
                       MATCH      ,                            
                       UNDERFLOW  ,                                
                       iMATCH     ,    
                       OVERFLOW   ,
                       Z01_18      
                                          );

//clock and reset
input           clk_comp;
input           rst_n;   //sys reset

input  [55:0]   R_pre;  
input  [55:0]   U_buf; 
input  [1:0]    Z0H_U;               // 2 10*10 mode high 18 bit [19:18] out
input  [1:0]    Z1H_U;               // 2 10*10 mode high 18 bit [19:18] out
input  [1:0]    Z0H_L;               // 2 10*10 mode high 18 bit [19:18] out
input  [1:0]    Z1H_L;               // 2 10*10 mode high 18 bit [19:18] out
input  [1:0]    FRCTRL ;             // 2   I Postadder Select final result source 
input  [55:0]   cfg_vldbit;
input           cfg_match;
input           cfg_imatch;
input  [4:0]    cfg_poa_rout ;       // 5   I Input   cfgmem,postadder R out control

output          MATCH;
output          UNDERFLOW;
output          iMATCH;
output          OVERFLOW;
output [3:0]    Z01_18;
wire   [55:0]   w56_comp_r;
wire   [55:0]   w56_comp_u;
wire   [55:0]   w56_comp_ub;

wire   [3:0]    match_0_pre;
reg             match_0;
reg             match_1;
reg             imatch_0;
reg             imatch_1;

wire match_pre0;
wire match_pre1;
wire match_pre2;
wire underflow_pre0;
wire underflow_pre1;
wire underflow_pre2;
wire overflow_pre0;
wire overflow_pre1;
wire overflow_pre2;
wire   [3:0]   z01_18_pre0;
wire   [3:0]   z01_18_pre1;

DSPCELL_AND2 #(56) U_DSPCELL_AND2_COMP_R ( //assign o = i0 & i1;
                        .i0  (cfg_vldbit  ),
                        .i1  (R_pre       ),
                        .o   (w56_comp_r  ) 
);
DSPCELL_AND2 #(56) U_DSPCELL_AND2_COMP_U ( //assign o = i0 & i1;
                        .i0  (cfg_vldbit  ),
                        .i1  (U_buf       ),
                        .o   (w56_comp_u  ) 
);
DSPCELL_AND2 #(56) U_DSPCELL_AND2_COMP_UB ( //assign o = i0 & i1;
                        .i0  (cfg_vldbit  ),
                        .i1  (~U_buf       ),
                        .o   (w56_comp_ub  ) 
);
assign match_0_pre[0] = (w56_comp_r[13:0]  == w56_comp_u[13:0])  ? 1'b1 : 1'b0;
assign match_0_pre[1] = (w56_comp_r[27:14] == w56_comp_u[27:14]) ? 1'b1 : 1'b0;
assign match_0_pre[2] = (w56_comp_r[41:28] == w56_comp_u[41:28]) ? 1'b1 : 1'b0;
assign match_0_pre[3] = (w56_comp_r[55:42] == w56_comp_u[55:42]) ? 1'b1 : 1'b0;
always @(posedge clk_comp or negedge rst_n) begin
   if(rst_n == 0) begin
       match_0 <= `DLY 0;
   end
   else if(match_0_pre[0]&match_0_pre[1]&match_0_pre[2]&match_0_pre[3]) begin
       match_0 <= `DLY 1;
   end
   else begin
       match_0 <= `DLY 0;
   end
end
always @(posedge clk_comp or negedge rst_n) begin
   if(rst_n == 0) begin
       imatch_0 <= `DLY 0;
   end
   else if(w56_comp_r == w56_comp_ub) begin
       imatch_0 <= `DLY 1;
   end
   else begin
       imatch_0 <= `DLY 0;
   end
end
always @(posedge clk_comp or negedge rst_n) begin
   if(rst_n == 0) begin
       match_1  <= `DLY 0;
       imatch_1 <= `DLY 0;
   end
   else begin
       match_1  <= `DLY match_0 ;
       imatch_1 <= `DLY imatch_0;
   end
end

DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_COMP_M(
                .i0   (match_1     ),
                .i1   (match_0     ),
                .sel  (cfg_match   ),
                .o    (match_pre0       )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z0_19_L0(
                .i0   (match_pre0     ),
                .i1   (Z0H_L[1]     ),  //H3M bug: this is 10*10 result bit 19, not bit 18, H3M work around in system software. P2 bug fix: this is 10*10 result bit 19
                .sel  (cfg_poa_rout[2]   ),
                .o    (match_pre1       )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z0_19_L1(
                .i0   (match_pre1     ),
                .i1   (1'b0     ),
                .sel  (FRCTRL[1]   ),
                .o    (match_pre2       )
);
DSPCELL_SELBUF #(1) U_DSPCELL_SELBUF_10x10_Z0_19_L(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_comp           ),       
                     .rst_n(rst_n         ),       
                     .i    (match_pre2     ),     
                     .sel  (cfg_poa_rout[3] ),       
                     .o    (MATCH     )    
);

DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_COMP_iM(
                .i0   (imatch_1     ),
                .i1   (imatch_0     ),
                .sel  (cfg_imatch   ),
                .o    (imatch_pre0       )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z1_19_L0(
                .i0   (imatch_pre0     ),
                .i1   (Z1H_L[1]     ),  //H3M bug: this is 10*10 result bit 19, not bit 18, H3M work around in system software. P2 bug fix: this is 10*10 result bit 19
                .sel  (cfg_poa_rout[2]   ),
                .o    (imatch_pre1       )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z1_19_L1(
                .i0   (imatch_pre1     ),
                .i1   (1'b0     ),
                .sel  (FRCTRL[1]   ),
                .o    (imatch_pre2       )
);
DSPCELL_SELBUF #(1) U_DSPCELL_SELBUF_10x10_Z1_19_L(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_comp           ),       
                     .rst_n(rst_n         ),       
                     .i    (imatch_pre2     ),     
                     .sel  (cfg_poa_rout[3] ),       
                     .o    (iMATCH     )    
);

DSPCELL_AND3 #(1) U_DSPCELL_AND3_COMP_UDFL (
                .i0 (~match_0   ),
                .i1 (~imatch_0  ),
                .i2 (match_1    ),
                .o  (underflow_pre0  ) 
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z0_19_U0(
                .i0   (underflow_pre0     ),
                .i1   (Z0H_U[1]     ),  //H3M bug: this is 10*10 result bit 19, not bit 18, H3M work around in system software. P2 bug fix: this is 10*10 result bit 19
                .sel  (cfg_poa_rout[2]   ),
                .o    (underflow_pre1       )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z0_19_U1(
                .i0   (underflow_pre1     ),
                .i1   (1'b0     ),
                .sel  (FRCTRL[1]   ),
                .o    (underflow_pre2       )
);
DSPCELL_SELBUF #(1) U_DSPCELL_SELBUF_10x10_Z0_19_U(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_comp           ),       
                     .rst_n(rst_n         ),       
                     .i    (underflow_pre2     ),     
                     .sel  (cfg_poa_rout[3] ),       
                     .o    (UNDERFLOW     )    
);

DSPCELL_AND3 #(1) U_DSPCELL_AND3_COMP_OVFL (
                .i0 (~match_0   ),
                .i1 (~imatch_0  ),
                .i2 (imatch_1   ),
                .o  (overflow_pre0   ) 
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z1_19_U0(
                .i0   (overflow_pre0     ),
                .i1   (Z1H_U[1]     ),  //H3M bug: this is 10*10 result bit 19, not bit 18, H3M work around in system software. P2 bug fix: this is 10*10 result bit 19
                .sel  (cfg_poa_rout[2]   ),
                .o    (overflow_pre1       )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_10x10_Z1_19_U1(
                .i0   (overflow_pre1     ),
                .i1   (1'b0     ),
                .sel  (FRCTRL[1]   ),
                .o    (overflow_pre2       )
);
DSPCELL_SELBUF #(1) U_DSPCELL_SELBUF_10x10_Z1_19_U(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_comp           ),       
                     .rst_n(rst_n         ),       
                     .i    (overflow_pre2     ),     
                     .sel  (cfg_poa_rout[3] ),       
                     .o    (OVERFLOW     )    
);

//Z1/0_U/L[18]
DSPCELL_MUX2 #(4) U_DSPCELL_MUX2_10x10_Z01_18_U0(
                .i0   ({Z1H_U[0],Z0H_U[0],Z1H_L[0],Z0H_L[0]}     ),  //H3M bug: this is 10*10 result bit 18, not bit 19, H3M work around in system software. P2 bug fix: this is 10*10 result bit 18
                .i1   (match_0_pre[3:0]     ),
                .sel  (cfg_poa_rout[4]   ),
                .o    (z01_18_pre0       )
);
DSPCELL_MUX2 #(4) U_DSPCELL_MUX2_10x10_Z01_18_U1(
                .i0   (z01_18_pre0     ),
                .i1   ({4{1'b0}}     ),
                .sel  (FRCTRL[1]   ),
                .o    (z01_18_pre1       )
);
DSPCELL_SELBUF #(4) U_DSPCELL_SELBUF_10x10_Z01_18_U(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_comp       ),
                     .rst_n(rst_n          ),
                     .i    (z01_18_pre1     ),
                     .sel  (cfg_poa_rout[0]),
                     .o    (Z01_18         )
);

endmodule   
//*************************************************
//company:   capital-micro
//author:   hongyu.wang
//date:      20140526
//function:   `DSP
//*************************************************
//$Log: .v,v $


`ifndef DLY
`define DLY #0.1
`endif


module dsp_input_buf(
                clk_ibuf,
                rst_n,   //sys reset
                cfg_PAZCTRL   ,    //             
                cfg_PARCTRL   ,    //             
                cfg_PASUBCTRL ,    //               
                cfg_BLCTRL    ,    //            
                cfg_BUCTRL    ,    //            
                cfg_PASCTRL   ,    //             
                cfg_ALUCTRL   ,    //   I Input Control the functionality of ALU operation 
                cfg_FCCTRL    ,    //   I Input Select Feedback and Cascaded data for UFC MUX 
                cfg_UFCCTRL   ,    //   I Input Select U and FC MUX data for post stage operation
                cfg_SUBCTRL   ,    //   I Input Select and Control to perform Subtraction in post stage operation 
                cfg_MCTRL     ,    //   I Input Select and Control the result of lower and upper multipliers and source for MR_U MUX 
                cfg_CCTRL     ,    //   I Input Select the source for rounding method, and perform R_CARRY_CAS_IN and user defined CARRY logic 
                cfg_FRCTRL    ,    //   I Input Select final result source 
                cfg_RNDM      ,    //   I Input Rounding method from user logic: 0 Rounding to Zero, 1:Rounding to infinity 
                iPAZCTRL   ,    //             
                iPARCTRL   ,    //             
                iPASUBCTRL ,    //               
                iBLCTRL    ,    //            
                iBUCTRL    ,    //            
                iPASCTRL   ,    //             
                iALUCTRL   ,    //   I Input Control the functionality of ALU operation 
                iFCCTRL    ,    //   I Input Select Feedback and Cascaded data for UFC MUX 
                iUFCCTRL   ,    //   I Input Select U and FC MUX data for post stage operation
                iSUBCTRL   ,    //   I Input Select and Control to perform Subtraction in post stage operation 
                iMCTRL     ,    //   I Input Select and Control the result of lower and upper multipliers and source for MR_U MUX 
                iCCTRL     ,    //   I Input Select the source for rounding method, and perform R_CARRY_CAS_IN and user defined CARRY logic 
                iFRCTRL    ,    //   I Input Select final result source 
                iRNDM      ,    //   I Input Rounding method from user logic: 0 Rounding to Zero, 1:Rounding to infinity 
                
                PAZCTRL   ,    //             
                PARCTRL   ,    //             
                PASUBCTRL ,    //               
                BLCTRL    ,    //            
                BUCTRL    ,    //            
                PASCTRL   ,    //             
                ALUCTRL   ,    //   I Input Control the functionality of ALU operation 
                FCCTRL    ,    //   I Input Select Feedback and Cascaded data for UFC MUX 
                UFCCTRL   ,    //   I Input Select U and FC MUX data for post stage operation
                SUBCTRL   ,    //   I Input Select and Control to perform Subtraction in post stage operation 
                MCTRL     ,    //   I Input Select and Control the result of lower and upper multipliers and source for MR_U MUX 
                CCTRL     ,    //   I Input Select the source for rounding method, and perform R_CARRY_CAS_IN and user defined CARRY logic 
                FRCTRL    ,    //   I Input Select final result source 
                RNDM           //   I Input Rounding method from user logic: 0 Rounding to Zero, 1:Rounding to infinity 



                     
                  );

 

//clock and reset
input              clk_ibuf;
input              rst_n;   //sys reset
                            //iPACTRL   24  I Input Select and Control the functionality of the Pre-Adder and Selector
input              cfg_PAZCTRL   ;    //             
input              cfg_PARCTRL   ;    //             
input              cfg_PASUBCTRL ;    //               
input              cfg_BLCTRL    ;    //            
input              cfg_BUCTRL    ;    //            
input              cfg_PASCTRL   ;    //             
input              cfg_ALUCTRL   ;    //   I Input Control the functionality of ALU operation 
input              cfg_FCCTRL    ;    //   I Input Select Feedback and Cascaded data for UFC MUX 
input              cfg_UFCCTRL   ;    //   I Input Select U and FC MUX data for post stage operation
input              cfg_SUBCTRL   ;    //   I Input Select and Control to perform Subtraction in post stage operation 
input              cfg_MCTRL     ;    //   I Input Select and Control the result of lower and upper multipliers and source for MR_U MUX 
input              cfg_CCTRL     ;    //   I Input Select the source for rounding method, and perform R_CARRY_CAS_IN and user defined CARRY logic 
input              cfg_FRCTRL    ;    //   I Input Select final result source 
input              cfg_RNDM      ;    //   I Input Rounding method from user logic: 0 Rounding to Zero, 1:Rounding to infinity 

input      [7:0 ]  iPAZCTRL   ;    //             
input      [5:0 ]  iPARCTRL   ;    //             
input      [1:0 ]  iPASUBCTRL ;    //               
input              iBLCTRL    ;    //            
input              iBUCTRL    ;    //            
input      [7:0 ]  iPASCTRL   ;    //             
input      [2:0 ]  iALUCTRL   ;    //   I Input Control the functionality of ALU operation 
input      [2:0 ]  iFCCTRL    ;    //   I Input Select Feedback and Cascaded data for UFC MUX 
input              iUFCCTRL   ;    //   I Input Select U and FC MUX data for post stage operation
input      [2:0 ]  iSUBCTRL   ;    //   I Input Select and Control to perform Subtraction in post stage operation 
input      [7:0 ]  iMCTRL     ;    //   I Input Select and Control the result of lower and upper multipliers and source for MR_U MUX 
input      [2:0 ]  iCCTRL     ;    //   I Input Select the source for rounding method, and perform R_CARRY_CAS_IN and user defined CARRY logic 
input      [1:0 ]  iFRCTRL    ;    //   I Input Select final result source 
input              iRNDM      ;    //   I Input Rounding method from user logic: 0 Rounding to Zero, 1:Rounding to infinity 

output     [7:0 ]  PAZCTRL   ;    //             
output     [5:0 ]  PARCTRL   ;    //             
output     [1:0 ]  PASUBCTRL ;    //               
output             BLCTRL    ;    //            
output             BUCTRL    ;    //            
output     [7:0 ]  PASCTRL   ;    //             
output     [2:0 ]  ALUCTRL   ;    //   I Input Control the functionality of ALU operation 
output     [2:0 ]  FCCTRL    ;    //   I Input Select Feedback and Cascaded data for UFC MUX 
output             UFCCTRL   ;    //   I Input Select U and FC MUX data for post stage operation
output     [2:0 ]  SUBCTRL   ;    //   I Input Select and Control to perform Subtraction in post stage operation 
output     [7:0 ]  MCTRL     ;    //   I Input Select and Control the result of lower and upper multipliers and source for MR_U MUX 
output     [2:0 ]  CCTRL     ;    //   I Input Select the source for rounding method, and perform R_CARRY_CAS_IN and user defined CARRY logic 
output     [1:0 ]  FRCTRL    ;    //   I Input Select final result source 
output             RNDM      ;    //   I Input Rounding method from user logic: 0 Rounding to Zero, 1:Rounding to infinity 




DSPCELL_SELBUF #(8)  U_DSPCELL_SELBUF_IBUF0 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iPAZCTRL         ),
                     .sel      (cfg_PAZCTRL      ),
                     .o        (PAZCTRL          ) 
);
DSPCELL_SELBUF #(6)  U_DSPCELL_SELBUF_IBUF1 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iPARCTRL         ),
                     .sel      (cfg_PARCTRL      ),
                     .o        (PARCTRL          ) 
);

DSPCELL_SELBUF #(2)  U_DSPCELL_SELBUF_IBUF2 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iPASUBCTRL         ),
                     .sel      (cfg_PASUBCTRL      ),
                     .o        (PASUBCTRL          ) 
);
DSPCELL_SELBUF #(1)  U_DSPCELL_SELBUF_IBUF3 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iBLCTRL         ),
                     .sel      (cfg_BLCTRL      ),
                     .o        (BLCTRL          ) 
);
DSPCELL_SELBUF #(1)  U_DSPCELL_SELBUF_IBUF4 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iBUCTRL         ),
                     .sel      (cfg_BUCTRL      ),
                     .o        (BUCTRL          ) 
);
DSPCELL_SELBUF #(8)  U_DSPCELL_SELBUF_IBUF5 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iPASCTRL         ),
                     .sel      (cfg_PASCTRL      ),
                     .o        (PASCTRL          ) 
);
DSPCELL_SELBUF #(3)  U_DSPCELL_SELBUF_IBUF6 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iALUCTRL         ),
                     .sel      (cfg_ALUCTRL      ),
                     .o        (ALUCTRL          ) 
);
DSPCELL_SELBUF #(3)  U_DSPCELL_SELBUF_IBUF7 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iFCCTRL         ),
                     .sel      (cfg_FCCTRL      ),
                     .o        (FCCTRL          ) 
);
DSPCELL_SELBUF #(1)  U_DSPCELL_SELBUF_IBUF8 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iUFCCTRL         ),
                     .sel      (cfg_UFCCTRL      ),
                     .o        (UFCCTRL          ) 
);
DSPCELL_SELBUF #(3)  U_DSPCELL_SELBUF_IBUF9 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iSUBCTRL         ),
                     .sel      (cfg_SUBCTRL      ),
                     .o        (SUBCTRL          ) 
);
DSPCELL_SELBUF #(8)  U_DSPCELL_SELBUF_IBUF10 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iMCTRL         ),
                     .sel      (cfg_MCTRL      ),
                     .o        (MCTRL          ) 
);
DSPCELL_SELBUF #(3)  U_DSPCELL_SELBUF_IBUF11 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iCCTRL         ),
                     .sel      (cfg_CCTRL      ),
                     .o        (CCTRL          ) 
);
DSPCELL_SELBUF #(2)  U_DSPCELL_SELBUF_IBUF12 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iFRCTRL         ),
                     .sel      (cfg_FRCTRL      ),
                     .o        (FRCTRL          ) 
);
DSPCELL_SELBUF #(1)  U_DSPCELL_SELBUF_IBUF13 (  
                     .clk      (clk_ibuf              ),
                     .rst_n    (rst_n            ),  
                     .i        (iRNDM         ),
                     .sel      (cfg_RNDM      ),
                     .o        (RNDM          ) 
);


endmodule   
//*************************************************
//author:    hywang
//date:      20140522
//function:   1)DSPCELL common cell
//            2)
//*************************************************
//$Log: .v,v $

`ifdef DLY_DSP
`else
`define DLY_DSP #0.1
`endif

`ifdef DLY
`else
`define DLY #0.1
`endif




//==========================================================
module DSPCELL_AND2 ( //assign o = i0 & i1;
      i0,
      i1,
      o  
);
parameter WIDTH = 1;
input [WIDTH-1:0]  i0;
input [WIDTH-1:0]  i1;
output[WIDTH-1:0]  o;

	assign o = i0 & i1;
endmodule
//==========================================================

module DSPCELL_AND3 (
    i0,
    i1,
    i2,
    o  
);
parameter WIDTH = 1;
input  [WIDTH-1:0] i0;
input  [WIDTH-1:0] i1;
input  [WIDTH-1:0] i2;
output [WIDTH-1:0] o;
	assign o = i0 & i1 & i2;
endmodule
//==========================================================
module DSPCELL_MUX2(  //assign o = sel ? i1 : i0;
                      i0,        
                      i1,     
                      sel,    
                      o         
);
parameter WIDTH = 1;
input [WIDTH-1:0] i0;
input [WIDTH-1:0] i1;
input             sel;
output[WIDTH-1:0] o;

//	assign o = sel ? i1 : i0;
genvar i; 
generate for(i=0; i<WIDTH; i=i+1) begin:gmux2
MUX2S U_gclk_clk_mux(
      .i0      (i0[i]  ),
      .i1      (i1[i]  ),
      .sel        (sel     ),
      .o     (o[i] )
);
end
endgenerate
endmodule
//==========================================================
//==========================================================
module DSPCELL_MUX4(  
                      i0,        
                      i1,     
                      i2,     
                      i3,     
                      sel,    
                      o         
);
parameter WIDTH = 1;
input [WIDTH-1:0] i0;
input [WIDTH-1:0] i1;
input [WIDTH-1:0] i2;
input [WIDTH-1:0] i3;
input [1:0]       sel;
output[WIDTH-1:0] o;

wire  [WIDTH-1:0] o_0;
wire  [WIDTH-1:0] o_1;

DSPCELL_MUX2 #(WIDTH) U_DSPCELL_MUX2_0(
                .i0   (i0     ),  
                .i1   (i1     ),  
                .sel  (sel[0] ),   
                .o    (o_0    )  
);
DSPCELL_MUX2 #(WIDTH) U_DSPCELL_MUX2_1(
                .i0   (i2     ),  
                .i1   (i3     ),  
                .sel  (sel[0] ),   
                .o    (o_1    )  
);
DSPCELL_MUX2 #(WIDTH) U_DSPCELL_MUX2_2(
                .i0   (o_0     ),  
                .i1   (o_1     ),  
                .sel  (sel[1]  ),   
                .o    (o       )  
);

endmodule
//==========================================================
module DSPCELL_MUX8(  
                      i0,        
                      i1,     
                      i2,     
                      i3,     
                      i4,     
                      i5,     
                      i6,     
                      i7,     
                      sel,    
                      o         
);
parameter WIDTH = 1;
input [WIDTH-1:0] i0;
input [WIDTH-1:0] i1;
input [WIDTH-1:0] i2;
input [WIDTH-1:0] i3;
input [WIDTH-1:0] i4;
input [WIDTH-1:0] i5;
input [WIDTH-1:0] i6;
input [WIDTH-1:0] i7;
input [2:0]       sel;
output[WIDTH-1:0] o;

wire  [WIDTH-1:0] o_0;
wire  [WIDTH-1:0] o_1;

DSPCELL_MUX4 #(WIDTH) U_DSPCELL_MUX4_0(
                .i0   (i0     ),  
                .i1   (i1     ),  
                .i2   (i2     ),  
                .i3   (i3     ),  
                .sel  (sel[1:0] ),   
                .o    (o_0    )  
);
DSPCELL_MUX4 #(WIDTH) U_DSPCELL_MUX4_1(
                .i0   (i4     ),  
                .i1   (i5     ),  
                .i2   (i6     ),  
                .i3   (i7     ),  
                .sel  (sel[1:0] ),   
                .o    (o_1    )  
);
DSPCELL_MUX2 #(WIDTH) U_DSPCELL_MUX2_2(
                .i0   (o_0     ),  
                .i1   (o_1     ),  
                .sel  (sel[2]  ),   
                .o    (o       )  
);

endmodule
//==========================================================



module DSPCELL_SELBUF(  //assign o = sel == 0 ? o_buf :  i
                      clk,        
                      rst_n,
                      i,     
                      sel,    
                      o         
);
parameter WIDTH = 1;
input             clk;
input             rst_n;
input [WIDTH-1:0] i;
input             sel;
output[WIDTH-1:0] o;
reg   [WIDTH-1:0] o_buf;
always @(posedge clk or negedge rst_n) begin
    if(rst_n == 0) begin
        o_buf <= 0;
    end
    else begin
        o_buf <= i;
    end
end
assign o = sel == 0 ? o_buf :  i;

endmodule
//==========================================================
module DSPCELL_FULLADD(  //s = i0 + i1
                      i0,        
                      i1,     
                      s,    
                      ci,         
                      co         
);
parameter WIDTH = 1;
input [WIDTH-1:0] i0;
input [WIDTH-1:0] i1;
input             ci;
output[WIDTH-1:0] s;
output            co;
wire  [WIDTH:0]   s_temp;
assign  `DLY_DSP s_temp = i0 + i1 + ci;
assign s = s_temp[WIDTH-1:0];
assign co = s_temp[WIDTH];

endmodule

//==========================================================
module DSPCELL_ADD(  //s = i0 + i1
                      i0,        
                      i1,     
                      s,    
                      c         
);
parameter WIDTH = 1;
input [WIDTH-1:0] i0;
input [WIDTH-1:0] i1;
output[WIDTH-1:0] s;
output            c;
wire  [WIDTH:0]   s_temp;
assign  `DLY_DSP s_temp = i0 + i1;
assign s = s_temp[WIDTH-1:0];
assign c = s_temp[WIDTH];

endmodule
//==========================================================
module DSPCELL_BIT_ADD(  
                      i0,        
                      i1,     
                      i2,     
                      s,
                      c    
);
parameter WIDTH = 1;

input   [WIDTH-1:0]   i0;
input   [WIDTH-1:0]   i1;
input   [WIDTH-1:0]   i2;
output  [WIDTH-1:0]   s;
output  [WIDTH-1:0]   c;
wire    [WIDTH*2-1:0] temp;
generate
genvar i;
for(i =0;i<WIDTH;i = i + 1) 
begin: adder
   assign temp[i*2+1:i*2] = i0[i] + i1[i] + i2[i];
   assign `DLY_DSP s[i] = temp[i*2];
   assign `DLY_DSP c[i] = temp[i*2+1];
end
endgenerate
endmodule


//==========================================================
module DSPCELL_SUB(  // s = i0 - i1 
                      i0,        
                      i1,     
                      s,    
                      c         
);
parameter WIDTH = 1;
input [WIDTH-1:0] i0;
input [WIDTH-1:0] i1;
output[WIDTH-1:0] s;
output            c;
wire  [WIDTH:0]   s_temp;
assign s_temp = i0 - i1;
assign `DLY_DSP s = s_temp[WIDTH-1:0];
assign `DLY_DSP c = s_temp[WIDTH];
endmodule
//==========================================================
module DSPCELL_MULT(  // s = i0 * i1 
                      i0,        
                      i1,     
                      o    
);
parameter WIDTH = 1;

input   [WIDTH-1:0]   i0;
input   [WIDTH-1:0]   i1;
output  [WIDTH*2-1:0] o;
wire signed  [WIDTH-1:0]    i0;
wire signed  [WIDTH-1:0]    i1;
wire signed  [WIDTH*2-1:0]  o;

assign `DLY_DSP o = i0 * i1;
endmodule
//==========================================================

// Following are added by dwli
//==========================================================
module DSPCELL_BUF(
                      i,        
                      o         
);
parameter WIDTH = 1;
input [WIDTH-1:0] i;
output[WIDTH-1:0] o;

	assign o = i;
endmodule
//==========================================================
module DSPCELL_MUX10(
    o,
    i0,
    i1,
    i2,
    i3,
    i4,
    i5,
    i6,
    i7,
    i8,
    i9,
    sel
);
parameter WIDTH = 1;
   
input [WIDTH-1:0] i0;
input [WIDTH-1:0] i1;
input [WIDTH-1:0] i2;
input [WIDTH-1:0] i3;
input [WIDTH-1:0] i4;
input [WIDTH-1:0] i5;
input [WIDTH-1:0] i6;
input [WIDTH-1:0] i7;
input [WIDTH-1:0] i8;
input [WIDTH-1:0] i9;
input [3:0] sel;

output [WIDTH-1:0] o;
wire  [WIDTH-1:0] o_0;
wire  [WIDTH-1:0] o_1;

DSPCELL_MUX8 #(WIDTH) U_DSPCELL_MUX8_0(  
                .i0         (i0     ),
                .i1         (i1     ),
                .i2         (i2     ),
                .i3         (i3     ),
                .i4         (i4     ),
                .i5         (i5     ),
                .i6         (i6     ),
                .i7         (i7     ),
                .sel        (sel[2:0]),
                .o          (o_0     )
);


DSPCELL_MUX2 #(WIDTH) U_DSPCELL_MUX2_0(
                .i0   (i8      ),  
                .i1   (i9      ),  
                .sel  (sel[0]  ),   
                .o    (o_1     )  
);

DSPCELL_MUX2 #(WIDTH) U_DSPCELL_MUX2_1(
                .i0   (o_0     ),  
                .i1   (o_1     ),  
                .sel  (sel[3]  ),   
                .o    (o       )  
);
endmodule
//==========================================================
module DSPCELL_TIEL(
    gnd
);

output gnd;

    assign gnd = 1'b0;
endmodule
//==========================================================
module DSPCELL_TIEH(
    vcc
);

output vcc;

    assign vcc = 1'b1;
endmodule

//==========================================================
// posedge triger and async reset , add del cell for cfgchain
module DSPCELL_DFF (
       clk,
       rst_n,
       d,
       q
);

input      clk ;
input      rst_n ;
input      d   ;
output     q   ;

reg  q;
always @ ( posedge clk or negedge rst_n ) begin
    if ( !rst_n ) begin
        q   <=  `DLY 1'b0 ;
    end
    else begin
        q   <=  `DLY  d ;
    end
end

endmodule

module DNT_rclk_buf(ck_in, ck_out);
parameter w = 1;
input  [w-1:0] ck_in;
output [w-1:0] ck_out;

    assign ck_out = ck_in;

endmodule

module DSPCELL_DECODE_4TO10(  // s = 
                      a,
                      s
);
input [3:0] a;
output[9:0] s;

assign s = (a == 4'h0) ? 10'b0000000001 :
           (a == 4'h1) ? 10'b0000000010 : 
           (a == 4'h2) ? 10'b0000000100 : 
           (a == 4'h3) ? 10'b0000001000 : 
           (a == 4'h4) ? 10'b0000010000 : 
           (a == 4'h5) ? 10'b0000100000 : 
           (a == 4'h6) ? 10'b0001000000 : 
           (a == 4'h7) ? 10'b0010000000 : 
           (a == 4'h8) ? 10'b0100000000 : 
           (a == 4'h9) ? 10'b1000000000 : 10'b0000000001 ; 

endmodule

module DSPCELL_MUX10_1 ( o, i0, i1, i2, i3, i4, i5, i6, i7, i8, i9,
        sel ,cfg_hasclk);
  output [0:0] o;
  input [0:0] i0;
  input [0:0] i1;
  input [0:0] i2;
  input [0:0] i3;
  input [0:0] i4;
  input [0:0] i5;
  input [0:0] i6;
  input [0:0] i7;
  input [0:0] i8;
  input [0:0] i9;
  input [9:0] sel;
  input cfg_hasclk ;

wire w = (sel == 10'b0000000001) ? i0 :
           (sel == 10'b0000000010) ? i1 :
           (sel == 10'b0000000100) ? i2 :
           (sel == 10'b0000001000) ? i3 :
           (sel == 10'b0000010000) ? i4 :
           (sel == 10'b0000100000) ? i5 :
           (sel == 10'b0001000000) ? i6 :
           (sel == 10'b0010000000) ? i7 :
           (sel == 10'b0100000000) ? i8 :
           (sel == 10'b1000000000) ? i9 : 1'bz;
assign o = cfg_hasclk & w;

endmodule

//*************************************************
//company:   capital-micro
//author:   hongyu.wang
//date:      20140526
//function:   `DSP
//*************************************************
//$Log: .v,v $


`ifndef DLY
`define DLY #0.1
`endif


module dsp_preadder(
                      clk_pra          ,                     
                      rst_n        ,
                      A_L          ,                     
                      C_L          ,                     
                      D_L          , 
                      F_L          ,                     
                      A_U          ,                     
                      C_U          ,                     
                      D_U          ,  
                      F_U          ,                    
                      X_U_SIGNED   ,
                      X_L_SIGNED   ,
                      Y_U_SIGNED   ,
                      Y_L_SIGNED   ,                                       
                      A_L_CAS_IN   ,                            
                      C_L_CAS_IN   ,                            
                      A_U_CAS_IN   ,                            
                      C_U_CAS_IN   ,                            
                      A_L_CAS_OUT  ,                             
                      C_L_CAS_OUT  ,                             
                      A_U_CAS_OUT  ,                             
                      C_U_CAS_OUT  ,                             
                      MA_U         ,   
                      MB_U         ,   
                      MB_L         ,   
                      MA_L         ,   
                      MD_U         ,
                      MD_L         ,
                      MF_U         ,
                      MF_L         ,
                      MX_U_SIGNED  ,
                      MX_L_SIGNED  ,
                      MY_U_SIGNED  ,
                      MY_L_SIGNED  ,
                      PAZCTRL      ,    
                      PARCTRL      ,    
                      PASUBCTRL    ,      
                      BLCTRL       ,                        
                      BUCTRL       ,                        
                      PASCTRL      ,    
                      cfg_pra_au   ,                            
                      cfg_pra_cu   ,                            
                      cfg_pra_du   ,                            
                      cfg_pra_dl   ,  
                      cfg_pra_fu   ,
                      cfg_pra_fl   ,                          
                      cfg_pra_al   ,                            
                      cfg_pra_cl   ,                            
                      cfg_pra_ou   ,                                                           
                      cfg_pra_ol                                
                  );

 

//clock and reset
input           clk_pra;
input           rst_n;   //sys reset

input  [17:0]   A_L;            //Input Input of the lower multiplier 
input  [17:0]   C_L;            //Input Input to the Pre-Adder and can be used as another input to the lower multiplier 
input  [17:0]   D_L;            //Input Another input to the Pre-Adder 
input  [17:0]   F_L;            //Input for 9*9 mode
input  [17:0]   A_U;            //Input Input of the upper multiplier 
input  [17:0]   C_U;            //Input Input to the Pre-Adder and can be used as another input to the upper multiplier 
input  [17:0]   D_U;            //Input Another input to the Pre-Adder 
input  [17:0]   F_U;            //Input for 9*9 mode

input  [3:0]    X_U_SIGNED;    //Input for 10*10 mode
input  [3:0]    X_L_SIGNED;    //Input for 10*10 mode
input  [3:0]    Y_U_SIGNED;    //Input for 10*10 mode
input  [3:0]    Y_L_SIGNED;    //Input for 10*10 mode
        
input  [17:0]   A_L_CAS_IN;     //Input Cascaded data input from A_L_CAS_OUT of previous DSP slice 
input  [17:0]   C_L_CAS_IN;     //Input Cascaded data input from C_L_CAS_OUT of previous DSP slice 
input  [17:0]   A_U_CAS_IN;     //Input Cascaded data input from A_U_CAS_OUT of previous DSP slice 
input  [17:0]   C_U_CAS_IN;     //Input Cascaded data input from C_U_CAS_OUT of previous DSP slice 
        
output [17:0]   A_L_CAS_OUT;    //Output  Cascaded data output to A_L_CAS_IN of next DSP slice 
output [17:0]   C_L_CAS_OUT;    //Output  Cascaded data output to C_L_CAS_IN of next DSP slice 
output [17:0]   A_U_CAS_OUT;    //Output  Cascaded data output to A_U_CAS_IN of next DSP slice 
output [17:0]   C_U_CAS_OUT;    //Output  Cascaded data output to C_U_CAS_IN of next DSP slice 

output [17:0]   MA_U;   
output [17:0]   MB_U;   
output [17:0]   MB_L;   
output [17:0]   MA_L;
   
output [17:0]   MD_U;     
output [17:0]   MD_L;  

output [17:0]   MF_U;     
output [17:0]   MF_L; 

output [3:0]    MX_U_SIGNED;    //Output for 10*10 mode
output [3:0]    MX_L_SIGNED;    //Output for 10*10 mode
output [3:0]    MY_U_SIGNED;    //Output for 10*10 mode
output [3:0]    MY_L_SIGNED;    //Output for 10*10 mode        
                                //Select and Control the functionality of the Pre-Adder and Selector
input  [7:0]    PAZCTRL; 
input  [5:0]    PARCTRL; 
input  [1:0]    PASUBCTRL; 
input           BLCTRL; 
input           BUCTRL; 
input  [7:0]    PASCTRL;

input  [2:0]    cfg_pra_au;     //3 I Input cfg mempreadder AU control
input  [2:0]    cfg_pra_cu;     //3 I Input cfg mempreadder CU control
input           cfg_pra_du;     //1 I Input cfg mempreadder DU control
input           cfg_pra_dl;     //1 I Input cfg mempreadder DL control
input           cfg_pra_fu;     //1 I Input cfg 9*9 mode FU control
input           cfg_pra_fl;     //1 I Input cfg 9*9 mode FU control
input  [2:0]    cfg_pra_al;     //3 I Input cfg mempreadder AL control
input  [2:0]    cfg_pra_cl;     //3 I Input cfg mempreadder CL control
input           cfg_pra_ou;     //1 I Input cfg mempreadder OU control
input           cfg_pra_ol;     //1 I Input cfg mempreadder OL control

wire   [17:0]   w18_au0;
wire   [17:0]   w18_au1;
wire   [17:0]   w18_au2;
wire   [17:0]   w18_au3;
wire   [17:0]   w18_cu0;
wire   [17:0]   w18_cu1;
wire   [17:0]   w18_cu2;
wire   [17:0]   w18_cu3;
wire   [17:0]   w18_du2;
wire   [17:0]   w18_du3;
wire   [17:0]   w18_dl2;
wire   [17:0]   w18_dl3;
wire   [17:0]   w18_fu2;
wire   [17:0]   w18_fu3;
wire   [17:0]   w18_fl2;
wire   [17:0]   w18_fl3;
wire   [17:0]   w18_cl0;
wire   [17:0]   w18_cl1;
wire   [17:0]   w18_cl2;
wire   [17:0]   w18_cl3;
wire   [17:0]   w18_al0;
wire   [17:0]   w18_al1;
wire   [17:0]   w18_al2;
wire   [17:0]   w18_al3;

wire   [1:0]    w2_ext_au2;
wire   [1:0]    w2_ext_cu2;
wire   [1:0]    w2_ext_du2;
wire   [1:0]    w2_ext_fu2;
wire   [1:0]    w2_ext_al2;
wire   [1:0]    w2_ext_cl2;
wire   [1:0]    w2_ext_dl2;
wire   [1:0]    w2_ext_fl2;

wire   [17:0]   D_U_CAS_OUT;    //Output  Cascaded data output to A_U_CAS_IN of next DSP slice 
wire   [17:0]   D_L_CAS_OUT;    //Output  Cascaded data output to C_U_CAS_IN of next DSP slice 
wire   [17:0]   A_U_BP;
wire   [17:0]   B_U;
wire   [17:0]   A_L_BP;
wire   [17:0]   B_L;
wire   [17:0]   w18_bu_0;
wire   [17:0]   w18_bu_1;
wire   [17:0]   w18_bu_2;
wire   [17:0]   w18_bu_3;
wire   [17:0]   w18_bl_0;
wire   [17:0]   w18_bl_1;
wire   [17:0]   w18_bl_2;
wire   [17:0]   w18_bl_3;
wire   [17:0]   C_U_CAS_OUT_t;
wire   [17:0]   C_L_CAS_OUT_t;

//======================================      
//AU 5
DSPCELL_MUX2 #(18) U_PAMUX_AU0(
                          .i0  (A_L_CAS_OUT   ),     
                          .i1  (A_U_CAS_IN    ),     
                          .sel (cfg_pra_au[0] ),      
                          .o   (w18_au0         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_AU1(
                          .i0  (w18_au0         ),     
                          .i1  (A_U           ),     
                          .sel (cfg_pra_au[1] ),      
                          .o   (w18_au1         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_AU2(
                          .i1  (18'h0           ),     
                          .i0  (w18_au1         ),     
                          .sel (PAZCTRL[5]      ),      
                          .o   (w18_au2         )  
);

//DSPCELL_AND2 #(18) U_PAAND2_AU2(
//                          .i0  (PAZCTRL[5]    ),
//                          .i1  (w18_au1         ),
//                          .o   (w18_au2         )
//);

DSPCELL_SELBUF #(18) U_PCELL_SELBUF_AU3(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_au2         ),  
                          .sel (cfg_pra_au[2] ),   
                          .o   (w18_au3         ) 
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_AU4(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_au3         ),  
                          .sel (PARCTRL[5]    ),   
                          .o   (A_U_CAS_OUT   ) 
);

//AU EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_AU2(
                          .i1  (2'h0            ),     
                          .i0  ({X_U_SIGNED[1],X_U_SIGNED[0]}      ),     
                          .sel (PAZCTRL[5]      ),      
                          .o   (w2_ext_au2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_AU3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_au2      ),  
                          .sel (cfg_pra_au[2]   ),   
                          .o   ({MX_U_SIGNED[1],MX_U_SIGNED[0]}     ) 
);

//CU 4
DSPCELL_MUX2 #(18) U_PAMUX_CU0(
                          .i0  (C_L_CAS_OUT   ),     
                          .i1  (C_U_CAS_IN    ),     
                          .sel (cfg_pra_cu[0] ),      
                          .o   (w18_cu0         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_CU1(
                          .i0  (w18_cu0         ),     
                          .i1  (C_U           ),     
                          .sel (cfg_pra_cu[1] ),      
                          .o   (w18_cu1         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_CU2(
                          .i1  (18'h0           ),     
                          .i0  (w18_cu1         ),     
                          .sel (PAZCTRL[4]      ),      
                          .o   (w18_cu2         )  
);

//
//DSPCELL_AND2 #(18) U_PAAND2_CU2(
//                          .i0  (PAZCTRL[4]    ),
//                          .i1  (w18_cu1         ),
//                          .o   (w18_cu2         )
//);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_CU3(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_cu2         ),  
                          .sel (cfg_pra_cu[2] ),   
                          .o   (w18_cu3         ) 
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_CU4(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_cu3         ),  
                          .sel (PARCTRL[4]    ),   
                          .o   (C_U_CAS_OUT   ) 
);
DSPCELL_MUX2 #(18)  U_PAMUX_BU0(
                          .i0  (C_U_CAS_OUT   ), 
                          .i1  (~C_U_CAS_OUT  ),
                          .sel (PASUBCTRL[1]  ),      
                          .o   (C_U_CAS_OUT_t )  
);

//CU EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_CU2(
                          .i1  (2'h0            ),     
                          .i0  ({Y_U_SIGNED[1],Y_U_SIGNED[0]}      ),     
                          .sel (PAZCTRL[4]      ),      
                          .o   (w2_ext_cu2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_CU3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_cu2      ),  
                          .sel (cfg_pra_cu[2]   ),   
                          .o   ({MY_U_SIGNED[1],MY_U_SIGNED[0]}     ) 
);

//DU 3
DSPCELL_MUX2 #(18) U_PAMUX_DU2(
                          .i1  (18'h0           ),     
                          .i0  (D_U             ),     
                          .sel (PAZCTRL[3]      ),      
                          .o   (w18_du2         )  
);
//DSPCELL_AND2 #(18) U_PAAND2_DU2(
//                          .i0  (PAZCTRL[3]    ),
//                          .i1  (D_U           ),
//                          .o   (w18_du2         )
//);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_DU3(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_du2         ),  
                          .sel (cfg_pra_du    ),   
                          .o   (w18_du3         ) 
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_DU4(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_du3         ),  
                          .sel (PARCTRL[3]    ),   
                          .o   (D_U_CAS_OUT   ) 
);
assign MD_U = D_U_CAS_OUT;

//DU EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_DU2(
                          .i1  (2'h0            ),     
                          .i0  ({X_U_SIGNED[3],X_U_SIGNED[2]}      ),     
                          .sel (PAZCTRL[3]      ),      
                          .o   (w2_ext_du2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_DU3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_du2      ),  
                          .sel (cfg_pra_du   ),   
                          .o   ({MX_U_SIGNED[3],MX_U_SIGNED[2]}     ) 
);

//FU 2
DSPCELL_MUX2 #(18) U_PAMUX_FU2(
                          .i1  (18'h0           ),     
                          .i0  (F_U             ),     
                          .sel (PAZCTRL[7]      ),      
                          .o   (w18_fu2         )  
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_FU3(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_fu2         ),  
                          .sel (cfg_pra_fu    ),   
                          .o   (MF_U         ) 
);
//DSPCELL_SELBUF #(18) U_PCELL_SELBUF_FU4(
//                          .clk (clk_pra           ),  
//                          .rst_n(rst_n         ),       
//                          .i   (w18_fu3         ),  
//                          .sel (PARCTRL[7]    ),   
//                          .o   (MF_U   ) 
//);
    
//FU EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_FU2(
                          .i1  (2'h0            ),     
                          .i0  ({Y_U_SIGNED[3],Y_U_SIGNED[2]}      ),     
                          .sel (PAZCTRL[7]      ),      
                          .o   (w2_ext_fu2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_FU3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_fu2      ),  
                          .sel (cfg_pra_fu   ),   
                          .o   ({MY_U_SIGNED[3],MY_U_SIGNED[2]}     ) 
);

//DL 2
DSPCELL_MUX2 #(18) U_PAMUX_DL2(
                          .i1  (18'h0           ),     
                          .i0  (D_L             ),     
                          .sel (PAZCTRL[2]      ),      
                          .o   (w18_dl2         )  
);

//DSPCELL_AND2 #(18) U_PAAND2_DL2(
//                          .i0  (PAZCTRL[2]    ),
//                          .i1  (D_L           ),
//                          .o   (w18_dl2         )
//);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_DL3(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_dl2         ),  
                          .sel (cfg_pra_dl    ),   
                          .o   (w18_dl3         ) 
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_DL4(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_dl3         ),  
                          .sel (PARCTRL[2]    ),   
                          .o   (D_L_CAS_OUT   ) 
);
assign MD_L = D_L_CAS_OUT;

//DL EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_DL2(
                          .i1  (2'h0            ),     
                          .i0  ({X_L_SIGNED[3],X_L_SIGNED[2]}      ),     
                          .sel (PAZCTRL[2]      ),      
                          .o   (w2_ext_dl2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_DL3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_dl2      ),  
                          .sel (cfg_pra_dl   ),   
                          .o   ({MX_L_SIGNED[3],MX_L_SIGNED[2]}     ) 
);

//FL 2
DSPCELL_MUX2 #(18) U_PAMUX_FL2(
                          .i1  (18'h0           ),     
                          .i0  (F_L             ),     
                          .sel (PAZCTRL[6]      ),      
                          .o   (w18_fl2         )  
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_FL3(
                          .clk (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i   (w18_fl2         ),  
                          .sel (cfg_pra_fl    ),   
                          .o   (MF_L         ) 
);
//DSPCELL_SELBUF #(18) U_PCELL_SELBUF_FL4(
//                          .clk (clk_pra           ),  
//                          .rst_n(rst_n         ),       
//                          .i   (w18_fl3         ),  
//                          .sel (PARCTRL[6]    ),   
//                          .o   (MF_L   ) 
//);

//FL EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_FL2(
                          .i1  (2'h0            ),     
                          .i0  ({Y_L_SIGNED[3],Y_L_SIGNED[2]}      ),     
                          .sel (PAZCTRL[6]      ),      
                          .o   (w2_ext_fl2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_FL3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_fl2      ),  
                          .sel (cfg_pra_fl   ),   
                          .o   ({MY_L_SIGNED[3],MY_L_SIGNED[2]}     ) 
);

//CL 1
DSPCELL_MUX2 #(18) U_PAMUX_CL0(
                          .i0  (C_L_CAS_IN    ),     
                          .i1  (C_U_CAS_IN    ),     
                          .sel (cfg_pra_cl[0] ),      
                          .o   (w18_cl0         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_CL1(
                          .i0  (w18_cl0         ),     
                          .i1  (C_L           ),     
                          .sel (cfg_pra_cl[1] ),      
                          .o   (w18_cl1         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_CL2(
                          .i1  (18'h0           ),     
                          .i0  (w18_cl1         ),     
                          .sel (PAZCTRL[1]      ),      
                          .o   (w18_cl2         )  
);


//DSPCELL_AND2 #(18) U_PAAND2_CL2(
//                          .i0  (PAZCTRL[1]    ),
//                          .i1  (w18_cl1         ),
//                          .o   (w18_cl2         )
//);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_CL3(
                          .clk  (clk_pra            ),  
                          .rst_n(rst_n          ),       
                          .i    (w18_cl2        ),  
                          .sel  (cfg_pra_cl[2]  ),   
                          .o    (w18_cl3        ) 
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_CL4(
                          .clk  (clk_pra            ),  
                          .rst_n(rst_n          ),       
                          .i    (w18_cl3        ),  
                          .sel  (PARCTRL[1]     ),   
                          .o    (C_L_CAS_OUT    ) 
);
DSPCELL_MUX2 #(18)  U_PAMUX_BL0(
                          .i0  (C_L_CAS_OUT   ), 
                          .i1  (~C_L_CAS_OUT  ),
                          .sel (PASUBCTRL[0]  ),      
                          .o   (C_L_CAS_OUT_t )  
);

//CL EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_CL2(
                          .i1  (2'h0            ),     
                          .i0  ({Y_L_SIGNED[1],Y_L_SIGNED[0]}      ),     
                          .sel (PAZCTRL[1]      ),      
                          .o   (w2_ext_cl2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_CL3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_cl2      ),  
                          .sel (cfg_pra_cl[2]   ),   
                          .o   ({MY_L_SIGNED[1],MY_L_SIGNED[0]}     ) 
);

//AL 0
DSPCELL_MUX2 #(18) U_PAMUX_AL0(
                          .i0  (A_L_CAS_IN      ),     
                          .i1  (A_U_CAS_IN      ),     
                          .sel (cfg_pra_al[0]   ),      
                          .o   (w18_al0         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_AL1(
                          .i0  (w18_al0         ),     
                          .i1  (A_L             ),     
                          .sel (cfg_pra_al[1]   ),      
                          .o   (w18_al1         )  
);
DSPCELL_MUX2 #(18) U_PAMUX_AL2(
                          .i1  (18'h0           ),     
                          .i0  (w18_al1         ),     
                          .sel (PAZCTRL[0]      ),      
                          .o   (w18_al2         )  
);

//DSPCELL_AND2 #(18) U_PAAND2_AL2(
//                          .i0  (PAZCTRL[1]      ),
//                          .i1  (w18_al1         ),
//                          .o   (w18_al2         )
//);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_AL3(
                          .clk    (clk_pra           ),  
                          .rst_n  (rst_n         ),       
                          .i      (w18_al2       ),  
                          .sel    (cfg_pra_al[2] ),   
                          .o      (w18_al3       ) 
);
DSPCELL_SELBUF #(18) U_PCELL_SELBUF_AL4(
                          .clk    (clk_pra           ),  
                          .rst_n  (rst_n         ),       
                          .i      (w18_al3       ),  
                          .sel    (PARCTRL[0]    ),   
                          .o      (A_L_CAS_OUT   ) 
);

//AL EXT
DSPCELL_MUX2 #(2) U_PAMUX_EXT_AL2(
                          .i1  (2'h0            ),     
                          .i0  ({X_L_SIGNED[1],X_L_SIGNED[0]}      ),     
                          .sel (PAZCTRL[0]      ),      
                          .o   (w2_ext_al2      )  
);
DSPCELL_SELBUF #(2) U_PCELL_SELBUF_EXT_AL3(
                          .clk (clk_pra         ),  
                          .rst_n(rst_n          ),       
                          .i   (w2_ext_al2      ),  
                          .sel (cfg_pra_al[2]   ),   
                          .o   ({MX_L_SIGNED[1],MX_L_SIGNED[0]}     ) 
);

assign A_U_BP = A_U_CAS_OUT;
assign A_L_BP = A_L_CAS_OUT;
//BU
//DSPCELL_MUX2 #(18)  U_PAMUX_BU0(
//                          .i0  (D_U_CAS_OUT   ), 
//                          .i1  (~D_U_CAS_OUT  ),
//                          .sel (PASUBCTRL[1]  ),      
//                          .o   (w18_bu_0      )  
//);
assign w18_bu_0 = D_U_CAS_OUT ;
DSPCELL_FULLADD #(18)   U_PCELL_FULLADD_BU1  (  //s = i0 + i1
                     .i0   (C_U_CAS_OUT_t),
                     .i1   (w18_bu_0     ),
                     .s    (w18_bu_1     ),
                     .ci   (PASUBCTRL[1] ),
                     .co   (             )  
);
DSPCELL_SELBUF #(18)  U_PCELL_SELBUF_BU2(
                          .clk  (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i    (w18_bu_1      ),  
                          .sel  (cfg_pra_ou    ),   
                          .o    (w18_bu_2      ) 
);
DSPCELL_MUX2 #(18)  U_PAMUX_BU3(
                          .i0  (C_U_CAS_OUT ),     
                          .i1  (w18_bu_2      ),     
                          .sel (BUCTRL        ),      
                          .o   (B_U           )  
);
//BL
//DSPCELL_MUX2 #(18)  U_PAMUX_BL0(
//                          .i0  (D_L_CAS_OUT   ), 
//                          .i1  (~D_L_CAS_OUT  ),
//                          .sel (PASUBCTRL[0]  ),      
//                          .o   (w18_bl_0      )  
//);
assign w18_bl_0 = D_L_CAS_OUT ;
DSPCELL_FULLADD #(18)   U_PCELL_FULLADD_BL1  (  //s = i0 + i1
                     .i0   (C_L_CAS_OUT_t),
                     .i1   (w18_bl_0     ),
                     .s    (w18_bl_1     ),
                     .ci   (PASUBCTRL[0] ),
                     .co   (             )  
);
DSPCELL_SELBUF #(18)  U_PCELL_SELBUF_BL2(
                          .clk  (clk_pra           ),  
                          .rst_n(rst_n         ),       
                          .i    (w18_bl_1      ),  
                          .sel  (cfg_pra_ol    ),   
                          .o    (w18_bl_2      ) 
);
DSPCELL_MUX2 #(18)  U_PAMUX_BL4(
                          .i0  (C_L_CAS_OUT ),     
                          .i1  (w18_bl_2      ),     
                          .sel (BLCTRL        ),      
                          .o   (B_L           )  
);
//BL
//PCELL_ADD #(18)   U_PCELL_ADD_BL0  (  //s = i0 + i1
//                     .i0   (C_L_CAS_OUT  ),
//                     .i1   (D_L_CAS_OUT  ),
//                     .s    (w18_bl_0     ),
//                     .c    (             )  
//);
//DSPCELL_SUB #(18) U_DSPCELL_SUB_BL1  (  // s = i0 - i1
//                     .i0   (C_L_CAS_OUT  ),
//                     .i1   (D_L_CAS_OUT  ),
//                     .s    (w18_bl_1     ),
//                     .c    (             ) 
//);
//DSPCELL_MUX2 #(18)  U_PAMUX_BL2(
//                          .i0  (w18_bl_0      ), //add 
//                          .i1  (w18_bl_1      ), //sub  
//                          .sel (PASUBCTRL[0]  ),      
//                          .o   (w18_bl_2      )  
//);
//PCELL_SELBUF #(18)  U_PCELL_SELBUF_BL3(
//                          .clk  (clk           ),  
//                          .rst_n(rst_n         ),       
//                          .i    (w18_bl_2      ),  
//                          .sel  (cfg_pra_ol    ),   
//                          .o    (w18_bl_3      ) 
//);
//DSPCELL_MUX2 #(18)  U_PAMUX_BL4(
//                          .i0  (C_U_CAS_OUT   ),     
//                          .i1  (w18_bl_3      ),     
//                          .sel (BLCTRL        ),
//                          .o   (B_L           )  
//);
//MA_U
DSPCELL_MUX4 #(18) U_DSPCELL_MUX4_MA_U(
                     .i3   (A_U_BP       ),  
                     .i2   (B_U          ),  
                     .i1   (A_L_BP       ),  
                     .i0   (B_L          ),  
                     .sel  (PASCTRL[7:6] ),   
                     .o    (MA_U         ) 
);
//MB_U
DSPCELL_MUX4 #(18) U_DSPCELL_MUX4_MB_U(
                     .i3   (A_U_BP       ),  
                     .i2   (B_U          ),  
                     .i1   (A_L_BP       ),  
                     .i0   (B_L          ),  
                     .sel  (PASCTRL[5:4] ),   
                     .o    (MB_U         ) 
);
//MB_L
DSPCELL_MUX4 #(18) U_DSPCELL_MUX4_MB_L(
                     .i3   (A_U_BP       ),  
                     .i2   (B_U          ),  
                     .i1   (A_L_BP       ),  
                     .i0   (B_L          ),  
                     .sel  (PASCTRL[3:2] ),   
                     .o    (MB_L         ) 
);
//MA_L
DSPCELL_MUX4 #(18) U_DSPCELL_MUX4_MA_L(
                     .i3   (A_U_BP       ),  
                     .i2   (B_U          ),  
                     .i1   (A_L_BP       ),  
                     .i0   (B_L          ),  
                     .sel  (PASCTRL[1:0] ),   
                     .o    (MA_L         ) 
);

endmodule
//*************************************************
//company:   capital-micro
//author:   hongyu.wang
//date:      20140526
//function:   `DSP
//*************************************************
//$Log: .v,v $


`ifndef DLY
`define DLY #0.1
`endif


module dsp_ufcmux(
                   clk_ufc          ,    
                   rst_n        ,
                   R_CAS_IN     ,           
                   R            ,    
                   U            ,    
                   UFC          ,      
                   U_BUF        ,        
                   FCCTRL       ,       
                   UFCCTRL      ,        
                   cfg_ufcmux               
                                     );
//clock and reset
input           clk_ufc;
input           rst_n;   //sys reset

input  [55:0]   R_CAS_IN;  
input  [55:0]   R;  
input  [55:0]   U;  
output [55:0]   UFC;  
output [55:0]   U_BUF;  

input  [2:0]    FCCTRL;
input           UFCCTRL;
input           cfg_ufcmux;

reg    [55:0]   R_CAS_IN_buf;
wire   [55:0]   w56_ufcmux0;

always @(posedge clk_ufc or negedge rst_n ) begin
    if(rst_n == 0) begin
        R_CAS_IN_buf <= `DLY 56'h0;
    end
    else begin
        R_CAS_IN_buf <= `DLY R_CAS_IN;
    end
end
//DSPCELL_MUX8 #(56) U_PCELL_MUX8_UFCMUX0(
//                     .i0  ( R                                   ), 
//                     .i1  ( 56'h0                               ), 
//                     .i2  ( {{17{R[55]}},R[55:17]}                ), 
//                     .i3  ( {{34{R[55]}},R[55:34]}                ), 
//                     .i4  ( R_CAS_IN                            ), 
//                     .i5  ( {{34{R_CAS_IN[55]}},R_CAS_IN[55:34]}  ), 
//                     .i6  ( R_CAS_IN_buf                        ), 
//                     .i7  ( {{17{R_CAS_IN[55]}},R_CAS_IN[55:17]}  ), 
//                     .sel ( FCCTRL                              ),  
//                     .o   ( w56_ufcmux0                         )  
//);
DSPCELL_MUX8 #(56) U_PCELL_MUX8_UFCMUX0(
                     .i0  ( 56'h0                                   ), 
                     .i1  ( R                               ), 
                     .i2  ( {{17{R[55]}},R[55:17]}                ), 
                     .i3  ( {{34{R[55]}},R[55:34]}                ), 
                     .i4  ( R_CAS_IN                            ), 
                     .i5  ( {{34{R_CAS_IN[55]}},R_CAS_IN[55:34]}  ), 
                     .i6  ( R_CAS_IN_buf                        ), 
                     .i7  ( {{17{R_CAS_IN[55]}},R_CAS_IN[55:17]}  ), 
                     .sel ( FCCTRL                              ),  
                     .o   ( w56_ufcmux0                         )  
);

DSPCELL_SELBUF #(56) U_PCELL_SELBUF_UFCMUX1(  //assign o = sel == 0 ? o_buf :  i
                     .clk ( clk_ufc         ),    
                     .rst_n ( rst_n),    
                     .i   ( U           ),  
                     .sel ( cfg_ufcmux  ),    
                     .o   ( U_BUF       )   
);
//DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_UFCMUX2(
//                .i0   (U_BUF           ),
//                .i1   (w56_ufcmux0     ),
//                .sel  (UFCCTRL         ),
//                .o    (UFC             )
//);
DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_UFCMUX2(
                .i0   (w56_ufcmux0     ),
                .i1   (U_BUF           ),
                .sel  (UFCCTRL         ),
                .o    (UFC             )
);

endmodule   

// VPERL: GENERATED_BEG

module dsp_core (
	alcasout,
	aucasout,
	clcasout,
	cucasout,
	imatch,
	match,
	overflow,
	r,
	r_ext,
	rcarrycasout,
	rcasout,
	recarrycasout,
	underflow,
	z01_18,
	al,
	alcasin,
	au,
	aucasin,
	cecomp,
	ceibuf,
	cemultl,
	cemultu,
	cepoa,
	cepra,
	ceufc,
	cfg_aluctrl,
	cfg_blctrl,
	cfg_buctrl,
	cfg_cctrl,
	cfg_fcctrl,
	cfg_frctrl,
	cfg_imatch,
	cfg_mat_clr,
	cfg_match,
	cfg_mctrl,
	cfg_mult_l,
	cfg_mult_u,
	cfg_parctrl,
	cfg_pasctrl,
	cfg_pasubctrl,
	cfg_pazctrl,
	cfg_poa_ceyin,
	cfg_poa_ceyout,
	cfg_poa_cyout,
	cfg_poa_rout,
	cfg_poa_rout_ext,
	cfg_pra_al,
	cfg_pra_au,
	cfg_pra_cl,
	cfg_pra_cu,
	cfg_pra_dl,
	cfg_pra_du,
	cfg_pra_fl,
	cfg_pra_fu,
	cfg_pra_ol,
	cfg_pra_ou,
	cfg_rndm,
	cfg_subctrl,
	cfg_ufcctrl,
	cfg_ufcmux,
	cfg_vldbit,
	cl,
	clcasin,
	clk,
	cu,
	cucasin,
	dl,
	du,
	fl,
	fu,
	ialuctrl,
	iblctrl,
	ibuctrl,
	icctrl,
	ifcctrl,
	ifrctrl,
	imctrl,
	iparctrl,
	ipasctrl,
	ipasubctrl,
	ipazctrl,
	irndm,
	isubctrl,
	iufcctrl,
	rcarrycasin,
	rcasin,
	recarrycasin,
	rstn,
	u,
	x_l_signed,
	x_u_signed,
	y_l_signed,
	y_u_signed 
);

output	[17:0]	alcasout;
output	[17:0]	aucasout;
output	[17:0]	clcasout;
output	[17:0]	cucasout;
output		imatch;
output		match;
output		overflow;
output	[71:0]	r;
output	[79:0]	r_ext;
output		rcarrycasout;
output	[55:0]	rcasout;
output		recarrycasout;
output		underflow;
output	[3:0]	z01_18;
input	[17:0]	al;
input	[17:0]	alcasin;
input	[17:0]	au;
input	[17:0]	aucasin;
input		cecomp;
input		ceibuf;
input		cemultl;
input		cemultu;
input		cepoa;
input		cepra;
input		ceufc;
input		cfg_aluctrl;
input		cfg_blctrl;
input		cfg_buctrl;
input		cfg_cctrl;
input		cfg_fcctrl;
input		cfg_frctrl;
input		cfg_imatch;
input		cfg_mat_clr;
input		cfg_match;
input		cfg_mctrl;
input	[8:0]	cfg_mult_l;
input	[8:0]	cfg_mult_u;
input		cfg_parctrl;
input		cfg_pasctrl;
input		cfg_pasubctrl;
input		cfg_pazctrl;
input		cfg_poa_ceyin;
input		cfg_poa_ceyout;
input		cfg_poa_cyout;
input	[4:0]	cfg_poa_rout;
input	[3:0]	cfg_poa_rout_ext;
input	[2:0]	cfg_pra_al;
input	[2:0]	cfg_pra_au;
input	[2:0]	cfg_pra_cl;
input	[2:0]	cfg_pra_cu;
input		cfg_pra_dl;
input		cfg_pra_du;
input		cfg_pra_fl;
input		cfg_pra_fu;
input		cfg_pra_ol;
input		cfg_pra_ou;
input		cfg_rndm;
input		cfg_subctrl;
input		cfg_ufcctrl;
input		cfg_ufcmux;
input	[55:0]	cfg_vldbit;
input	[17:0]	cl;
input	[17:0]	clcasin;
input		clk;
input	[17:0]	cu;
input	[17:0]	cucasin;
input	[17:0]	dl;
input	[17:0]	du;
input	[17:0]	fl;
input	[17:0]	fu;
input	[2:0]	ialuctrl;
input		iblctrl;
input		ibuctrl;
input	[2:0]	icctrl;
input	[2:0]	ifcctrl;
input	[1:0]	ifrctrl;
input	[7:0]	imctrl;
input	[5:0]	iparctrl;
input	[7:0]	ipasctrl;
input	[1:0]	ipasubctrl;
input	[7:0]	ipazctrl;
input		irndm;
input	[2:0]	isubctrl;
input		iufcctrl;
input		rcarrycasin;
input	[55:0]	rcasin;
input		recarrycasin;
input		rstn;
input	[55:0]	u;
input	[3:0]	x_l_signed;
input	[3:0]	x_u_signed;
input	[3:0]	y_l_signed;
input	[3:0]	y_u_signed;

wire	[2:0]	aluctrl;
wire		blctrl;
wire		buctrl;
wire	[2:0]	cctrl;
wire		clk_comp;
wire		clk_ibuf;
wire		clk_mult_l;
wire		clk_mult_u;
wire		clk_poa;
wire		clk_pra;
wire		clk_ufc;
wire	[2:0]	fcctrl;
wire	[1:0]	frctrl;
wire	[17:0]	mal;
wire	[17:0]	mau;
wire	[17:0]	mbl;
wire	[17:0]	mbu;
wire	[7:0]	mctrl;
wire	[17:0]	mdl;
wire	[17:0]	mdu;
wire	[17:0]	mfl;
wire	[17:0]	mfu;
wire	[55:0]	mrl;
wire	[55:0]	mru;
wire	[3:0]	mx_l_signed;
wire	[3:0]	mx_u_signed;
wire	[3:0]	my_l_signed;
wire	[3:0]	my_u_signed;
wire	[5:0]	parctrl;
wire	[7:0]	pasctrl;
wire	[1:0]	pasubctrl;
wire	[7:0]	pazctrl;
wire		rndm;
wire	[55:0]	rpre;
wire		smode;
wire	[2:0]	subctrl;
wire	[55:0]	ubuf;
wire	[55:0]	ufc;
wire		ufcctrl;
wire	[19:0]	z1_l;
wire	[19:0]	z1_u;
wire	[1:0]	z1h_l;
wire	[1:0]	z1h_u;
wire	[19:0]	z0_l;
wire	[19:0]	z0_u;
wire	[1:0]	z0h_l;
wire	[1:0]	z0h_u;

dsp_clkgate U_dsp_clkgate (
	.test_mode	(smode),
	.clk		(clk),
	.ce_pra		(cepra),
	.ce_mult_u	(cemultu),
	.ce_mult_l	(cemultl),
	.ce_poa		(cepoa),
	.ce_comp	(cecomp),
	.ce_ufc		(ceufc),
	.ce_ibuf	(ceibuf),

	.clk_pra	(clk_pra),
	.clk_mult_u	(clk_mult_u),
	.clk_mult_l	(clk_mult_l),
	.clk_poa	(clk_poa),
	.clk_comp	(clk_comp),
	.clk_ufc	(clk_ufc),
	.clk_ibuf	(clk_ibuf) 
);
assign smode = 1'b0;

dsp_compunit U_dsp_compunit (
	.clk_comp	(clk_comp),
	.rst_n		(rstn),
	.R_pre		(rpre[55:0]),
	.U_buf		(ubuf[55:0]),
	.Z0H_U		(z0h_u[1:0]),
	.Z1H_U		(z1h_u[1:0]),
	.Z0H_L		(z0h_l[1:0]),
	.Z1H_L		(z1h_l[1:0]),
	.FRCTRL		(frctrl[1:0]),
	.cfg_vldbit	(cfg_vldbit[55:0]),
	.cfg_match	(cfg_match),
	.cfg_imatch	(cfg_imatch),
	.cfg_poa_rout	(cfg_poa_rout[4:0]),

	.MATCH		(match),
	.UNDERFLOW	(underflow),
	.iMATCH		(imatch),
	.OVERFLOW	(overflow),
	.Z01_18		(z01_18[3:0]) 
);
assign z0h_u = z0_u[19:18];
assign z1h_u = z1_u[19:18];
assign z0h_l = z0_l[19:18];
assign z1h_l = z1_l[19:18];

dsp_input_buf U_dsp_input_buf (
	.clk_ibuf	(clk_ibuf),
	.rst_n		(rstn),
	.cfg_PAZCTRL	(cfg_pazctrl),
	.cfg_PARCTRL	(cfg_parctrl),
	.cfg_PASUBCTRL	(cfg_pasubctrl),
	.cfg_BLCTRL	(cfg_blctrl),
	.cfg_BUCTRL	(cfg_buctrl),
	.cfg_PASCTRL	(cfg_pasctrl),
	.cfg_ALUCTRL	(cfg_aluctrl),
	.cfg_FCCTRL	(cfg_fcctrl),
	.cfg_UFCCTRL	(cfg_ufcctrl),
	.cfg_SUBCTRL	(cfg_subctrl),
	.cfg_MCTRL	(cfg_mctrl),
	.cfg_CCTRL	(cfg_cctrl),
	.cfg_FRCTRL	(cfg_frctrl),
	.cfg_RNDM	(cfg_rndm),
	.iPAZCTRL	(ipazctrl[7:0]),
	.iPARCTRL	(iparctrl[5:0]),
	.iPASUBCTRL	(ipasubctrl[1:0]),
	.iBLCTRL	(iblctrl),
	.iBUCTRL	(ibuctrl),
	.iPASCTRL	(ipasctrl[7:0]),
	.iALUCTRL	(ialuctrl[2:0]),
	.iFCCTRL	(ifcctrl[2:0]),
	.iUFCCTRL	(iufcctrl),
	.iSUBCTRL	(isubctrl[2:0]),
	.iMCTRL		(imctrl[7:0]),
	.iCCTRL		(icctrl[2:0]),
	.iFRCTRL	(ifrctrl[1:0]),
	.iRNDM		(irndm),

	.PAZCTRL	(pazctrl[7:0]),
	.PARCTRL	(parctrl[5:0]),
	.PASUBCTRL	(pasubctrl[1:0]),
	.BLCTRL		(blctrl),
	.BUCTRL		(buctrl),
	.PASCTRL	(pasctrl[7:0]),
	.ALUCTRL	(aluctrl[2:0]),
	.FCCTRL		(fcctrl[2:0]),
	.UFCCTRL	(ufcctrl),
	.SUBCTRL	(subctrl[2:0]),
	.MCTRL		(mctrl[7:0]),
	.CCTRL		(cctrl[2:0]),
	.FRCTRL		(frctrl[1:0]),
	.RNDM		(rndm) 
);

dsp_multblock U_dsp_multblock (
	.clk_mult_u		(clk_mult_u),
	.clk_mult_l		(clk_mult_l),
	.rst_n			(rstn),
	.MA_U			(mau[17:0]),
	.MB_U			(mbu[17:0]),
	.MD_U			(mdu[17:0]),
	.MF_U			(mfu[17:0]),
	.MA_L			(mal[17:0]),
	.MB_L			(mbl[17:0]),
	.MD_L			(mdl[17:0]),
	.MF_L			(mfl[17:0]),
	.MX_U_SIGNED		(mx_u_signed[3:0]),
	.MX_L_SIGNED		(mx_l_signed[3:0]),
	.MY_U_SIGNED		(my_u_signed[3:0]),
	.MY_L_SIGNED		(my_l_signed[3:0]),
	.U_buf			(ubuf[55:0]),
	.R			(r[55:0]),
	.cfg_mult_u		(cfg_mult_u[8:0]),
	.cfg_mult_l		(cfg_mult_l[8:0]),
	.cfg_poa_rout_ext	(cfg_poa_rout_ext[3:0]),
	.MCTRL			(mctrl[7:0]),

	.MR_U			(mru[55:0]),
	.MR_L			(mrl[55:0]),
	.R_EXT			(r_ext[79:0]),
	.Z0_U			(z0_u[19:0]),
	.Z1_U			(z1_u[19:0]),
	.Z0_L			(z0_l[19:0]),
	.Z1_L			(z1_l[19:0]) 
);

dsp_postadder U_dsp_postadder (
	.clk_poa		(clk_poa),
	.rst_n			(rstn),
	.MR_U			(mru[55:0]),
	.MR_L			(mrl[55:0]),
	.UFC			(ufc[55:0]),
	.Z0_U			(z0_u[17:0]),
	.Z1_U			(z1_u[17:0]),
	.Z0_L			(z0_l[17:0]),
	.Z1_L			(z1_l[17:0]),
	.R_CARRY_CAS_IN		(rcarrycasin),
	.R_ECARRY_CAS_IN	(recarrycasin),
	.MATCH			(match),
	.RNDM			(rndm),
	.SUBCTRL		(subctrl[2:0]),
	.ALUCTRL		(aluctrl[2:0]),
	.FRCTRL			(frctrl[1:0]),
	.CCTRL			(cctrl[2:0]),
	.cfg_mat_clr		(cfg_mat_clr),
	.cfg_poa_cyout		(cfg_poa_cyout),
	.cfg_poa_ceyout		(cfg_poa_ceyout),
	.cfg_poa_ceyin		(cfg_poa_ceyin),
	.cfg_poa_rout		(cfg_poa_rout[2:0]),

	.R_CARRY_CAS_OUT	(rcarrycasout),
	.R_ECARRY_CAS_OUT	(recarrycasout),
	.R_pre			(rpre[55:0]),
	.R			(r[71:0]),
	.R_CAS_OUT		(rcasout[55:0]) 
);

dsp_preadder U_dsp_preadder (
	.clk_pra	(clk_pra),
	.rst_n		(rstn),
	.A_L		(al[17:0]),
	.C_L		(cl[17:0]),
	.D_L		(dl[17:0]),
	.F_L		(fl[17:0]),
	.A_U		(au[17:0]),
	.C_U		(cu[17:0]),
	.D_U		(du[17:0]),
	.F_U		(fu[17:0]),
	.X_U_SIGNED	(x_u_signed[3:0]),
	.X_L_SIGNED	(x_l_signed[3:0]),
	.Y_U_SIGNED	(y_u_signed[3:0]),
	.Y_L_SIGNED	(y_l_signed[3:0]),
	.A_L_CAS_IN	(alcasin[17:0]),
	.C_L_CAS_IN	(clcasin[17:0]),
	.A_U_CAS_IN	(aucasin[17:0]),
	.C_U_CAS_IN	(cucasin[17:0]),
	.PAZCTRL	(pazctrl[7:0]),
	.PARCTRL	(parctrl[5:0]),
	.PASUBCTRL	(pasubctrl[1:0]),
	.BLCTRL		(blctrl),
	.BUCTRL		(buctrl),
	.PASCTRL	(pasctrl[7:0]),
	.cfg_pra_au	(cfg_pra_au[2:0]),
	.cfg_pra_cu	(cfg_pra_cu[2:0]),
	.cfg_pra_du	(cfg_pra_du),
	.cfg_pra_dl	(cfg_pra_dl),
	.cfg_pra_fu	(cfg_pra_fu),
	.cfg_pra_fl	(cfg_pra_fl),
	.cfg_pra_al	(cfg_pra_al[2:0]),
	.cfg_pra_cl	(cfg_pra_cl[2:0]),
	.cfg_pra_ou	(cfg_pra_ou),
	.cfg_pra_ol	(cfg_pra_ol),

	.A_L_CAS_OUT	(alcasout[17:0]),
	.C_L_CAS_OUT	(clcasout[17:0]),
	.A_U_CAS_OUT	(aucasout[17:0]),
	.C_U_CAS_OUT	(cucasout[17:0]),
	.MA_U		(mau[17:0]),
	.MB_U		(mbu[17:0]),
	.MB_L		(mbl[17:0]),
	.MA_L		(mal[17:0]),
	.MD_U		(mdu[17:0]),
	.MD_L		(mdl[17:0]),
	.MF_U		(mfu[17:0]),
	.MF_L		(mfl[17:0]),
	.MX_U_SIGNED	(mx_u_signed[3:0]),
	.MX_L_SIGNED	(mx_l_signed[3:0]),
	.MY_U_SIGNED	(my_u_signed[3:0]),
	.MY_L_SIGNED	(my_l_signed[3:0]) 
);

dsp_ufcmux U_dsp_ufcmux (
	.clk_ufc	(clk_ufc),
	.rst_n		(rstn),
	.R_CAS_IN	(rcasin[55:0]),
	.R		(r[55:0]),
	.U		(u[55:0]),
	.FCCTRL		(fcctrl[2:0]),
	.UFCCTRL	(ufcctrl),
	.cfg_ufcmux	(cfg_ufcmux),

	.UFC		(ufc[55:0]),
	.U_BUF		(ubuf[55:0]) 
);

endmodule

// VPERL: GENERATED_END

//*************************************************
//company:   capital-micro
//author:   hongyu.wang
//date:      20140526
//function:   `DSP
//*************************************************
//$Log: .v,v $


`ifndef DLY
`define DLY #0.1
`endif


module dsp_multblock(
                         clk_mult_u, 
                         clk_mult_l, 
                         rst_n,
                         MA_U ,   
                         MB_U , 
                         MD_U ,  
                         MF_U ,
                         MA_L ,   
                         MB_L ,   
                         MD_L ,
                         MF_L ,
                         MX_U_SIGNED ,
                         MX_L_SIGNED ,
                         MY_U_SIGNED ,
                         MY_L_SIGNED ,
                         U_buf,  
                         R,  
                         MR_U ,   
                         MR_L ,  
                         R_EXT,   
                         Z0_U ,
                         Z1_U ,
                         Z0_L ,
                         Z1_L ,
                         cfg_mult_u,
                         cfg_mult_l,
                         cfg_poa_rout_ext,
                         MCTRL         
                                     );
 

//clock and reset
input           clk_mult_u;
input           clk_mult_l;
input           rst_n;   //sys reset

input  [17:0]   MA_U;  
input  [17:0]   MB_U;  
input  [17:0]   MD_U;
input  [17:0]   MF_U;
input  [17:0]   MA_L;  
input  [17:0]   MB_L; 
input  [17:0]   MD_L; 
input  [17:0]   MF_L; 
input  [3:0]    MX_U_SIGNED;    //Input for 10*10 mode  
input  [3:0]    MX_L_SIGNED;    //Input for 10*10 mode  
input  [3:0]    MY_U_SIGNED;    //Input for 10*10 mode  
input  [3:0]    MY_L_SIGNED;    //Input for 10*10 mode  
input  [55:0]   U_buf;  
input  [55:0]   R;  

output [55:0]   MR_U;  
output [55:0]   MR_L;  
output [79:0]   R_EXT;
output [19:0]   Z0_U;           //output to postadder and compuit
output [19:0]   Z1_U;           //output to postadder and compuit
output [19:0]   Z0_L;           //output to postadder and compuit
output [19:0]   Z1_L;           //output to postadder and compuit


input  [8:0]   cfg_mult_u;
input  [8:0]   cfg_mult_l;
input  [3:0]    cfg_poa_rout_ext;
input  [7:0]    MCTRL;

wire   [35:0]   w36_mabu;
wire   [35:0]   w36_mabu1;
wire   [35:0]   w36_mabu2;
wire   [35:0]   w36_mabu3;
wire   [52:0]   w53_mabu4;

wire   [35:0]   w36_mabl;
wire   [35:0]   w36_mabl1;
wire   [35:0]   w36_mabl2;
wire   [35:0]   w36_mabl3;

wire   [55:0]   w56_mcat;
wire   [55:0]   MR_L_t;

wire   [9:0]    X0_U, X1_U, X2_U, X3_U;
wire   [9:0]    X0_L, X1_L, X2_L, X3_L;  
wire   [9:0]    Y0_U, Y1_U, Y2_U, Y3_U;
wire   [9:0]    Y0_L, Y1_L, Y2_L, Y3_L;    
wire   [19:0]   Z0_U;
wire   [19:0]   Z1_U;
wire   [19:0]   Z2_U;
wire   [19:0]   Z3_U; 
wire   [19:0]   Z0_L;
wire   [19:0]   Z1_L;
wire   [19:0]   Z2_L;
wire   [19:0]   Z3_L;  
wire   [26:0]   w27_z1_u;
wire   [26:0]   w27_z3_u;
wire   [26:0]   w27_z1_l;
wire   [26:0]   w27_z3_l;
wire   [26:0]   w27_z01_adds_u;
wire   [26:0]   w27_z23_adds_u;
wire   [35:0]   w36_z01_u;
wire   [28:0]   w29_z23_u0;
wire   [28:0]   w29_z23_u1;
wire   [26:0]   w27_z01_adds_l;
wire   [26:0]   w27_z23_adds_l;
wire   [35:0]   w36_z01_l;
wire   [28:0]   w29_z23_l0;
wire   [28:0]   w29_z23_l1;
wire   [19:0]   w20_18x9_bit19_0_u0;
wire   [19:0]   w20_18x9_bit19_0_u1;
wire   [19:0]   w20_18x9_bit39_20_u0;
wire   [19:0]   w20_18x9_bit39_20_u1;
wire   [19:0]   w20_18x9_bit19_0_l0;
wire   [19:0]   w20_18x9_bit19_0_l1;
wire   [19:0]   w20_18x9_bit39_20_l0;
wire   [19:0]   w20_18x9_bit39_20_l1;
wire            X1_U_9_MUX;
wire            X1_L_9_MUX;
wire            Y1_U_9_MUX;
wire            Y1_L_9_MUX;
wire            Y2_U_9_MUX;
wire            Y2_L_9_MUX;
wire            Y3_U_9_MUX;
wire            Y3_L_9_MUX;

//DSPCELL_MULT #(18) U_DSPCELL_MULT_MABU0(
//                     .i0 (MA_U      ),
//                     .i1 (MB_U      ),
//                     .o  (w36_mabu  ) 
//);

DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X0_9_U0(
                .i0   (1'b0 	 ),
                .i1   (MX_U_SIGNED[0] ),
                .sel  (cfg_mult_u[2]),
                .o    (X0_U[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y0_9_U0(
                .i0   (1'b0 	 ),
                .i1   (MY_U_SIGNED[0] ),
                .sel  (cfg_mult_u[2]),
                .o    (Y0_U[9] )
);
assign X0_U[8:0] = MA_U[8:0];
assign Y0_U[8:0] = MB_U[8:0];
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABU0(
                     .i0 (X0_U      ),
                     .i1 (Y0_U      ),
                     .o  (Z0_U      ) 
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X1_9_U0(
                .i0   (MA_U[17] 	 ),
                .i1   (MX_U_SIGNED[1] ),
                .sel  (cfg_mult_u[2]),
                .o    (X1_U[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y1_9_U0(
                .i0   (MB_U[17] 	 ),
                .i1   (MY_U_SIGNED[1] ),
                .sel  (cfg_mult_u[2]),
                .o    (Y1_U[9] )
);
assign X1_U[8:0] = MA_U[17:9];
assign Y1_U[8:0] = MB_U[17:9];
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABU1(
                     .i0 (X1_U      ),
                     .i1 (Y1_U      ),
                     .o  (Z1_U      ) 
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X2_9_U0(
                .i0   (1'b0 	 ),
                .i1   (MX_U_SIGNED[2] ),
                .sel  (cfg_mult_u[2]),
                .o    (X2_U[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y2_9_U0(
                .i0   (MB_U[17] 	 ),
                .i1   (MY_U_SIGNED[2] ),
                .sel  (cfg_mult_u[2]),
                .o    (Y2_U[9] )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_X2_U0(
                .i0   (MA_U[8:0]  ),
                .i1   (MD_U[8:0] ),
                .sel  (cfg_mult_u[2]          ),
                .o    (X2_U[8:0]              )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_Y2_U0(
                .i0   (MB_U[17:9] 	 ),
                .i1   (MF_U[8:0] ),
                .sel  (cfg_mult_u[2]  ),
                .o    (Y2_U[8:0] )
);
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABU2(
                     .i0 (X2_U      ),
                     .i1 (Y2_U      ),
                     .o  (Z2_U      ) 
);
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_U0(
                .i0   (Z2_U[19:0]  ),
                .i1   (w27_z23_adds_u[19:0] ),
                .sel  (cfg_mult_u[8]            ),
                .o    (w20_18x9_bit19_0_u0                )
); 
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_U2(
                .i1   ({20{1'b0}}  ),
                .i0   (w20_18x9_bit19_0_u0[19:0] ),
                .sel  (MCTRL[7]                  ),
                .o    (w20_18x9_bit19_0_u1                )
);                               
DSPCELL_SELBUF #(20) U_DSPCELL_SELBUF_Z2_U0(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_u           ),       
                     .rst_n(rst_n         ),       
                     .i    (w20_18x9_bit19_0_u1      ),     
                     .sel  (cfg_poa_rout_ext[2] ),       
                     .o    (R_EXT[59:40]     )      
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X3_9_U0(
                .i0   (1'b0 	 ),
                .i1   (MX_U_SIGNED[3] ),
                .sel  (cfg_mult_u[2]),
                .o    (X3_U[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y3_9_U0(
                .i0   (MA_U[17] 	 ),
                .i1   (MY_U_SIGNED[3] ),
                .sel  (cfg_mult_u[2]),
                .o    (Y3_U[9] )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_X3_U0(
                .i0   (MB_U[8:0]  ),
                .i1   (MD_U[17:9] ),
                .sel  (cfg_mult_u[2]            ),
                .o    (X3_U[8:0]                )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_Y3_U0(
                .i0   (MA_U[17:9] 	     ),
                .i1   (MF_U[17:9] ),
                .sel  (cfg_mult_u[2]  ),
                .o    (Y3_U[8:0] )
);
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABU3(
                     .i0 (X3_U      ),
                     .i1 (Y3_U      ),
                     .o  (Z3_U  ) 
);
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_U1(
                .i0   (Z3_U[19:0]  ),
                .i1   ({{13{w27_z23_adds_u[26]}},w27_z23_adds_u[26:20]} ),
                .sel  (cfg_mult_u[8]            ),
                .o    (w20_18x9_bit39_20_u0                )
);
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_U3(
                .i1   ({20{1'b0}}  ),
                .i0   (w20_18x9_bit39_20_u0[19:0] ),
                .sel  (MCTRL[7]            ),
                .o    (w20_18x9_bit39_20_u1                )
);
DSPCELL_SELBUF #(20) U_DSPCELL_SELBUF_Z3_U0(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_u           ),       
                     .rst_n(rst_n         ),       
                     .i    (w20_18x9_bit39_20_u1      ),     
                     .sel  (cfg_poa_rout_ext[3] ),       
                     .o    (R_EXT[79:60]     )      
);
DSPCELL_MUX2 #(27) U_DSPCELL_MUX2_Z1_U0(
                .i0   ({{7{Z1_U[19]}},Z1_U[19:0]} 	     ),
                .i1   ({Z1_U[17:0],{9{1'b0}}} ),
                .sel  (cfg_mult_u[4]  ),
                .o    (w27_z1_u )
);
DSPCELL_ADD #(27)   U_DSPCELL_ADD27_Z01_U0  (  //s = i0 + i1
                     .i0   ({{7{Z0_U[19]}},Z0_U[19:0]}     ),
                     .i1   (w27_z1_u     ),
                     .s    (w27_z01_adds_u     ),
                     .c   (       )  
);
DSPCELL_MUX2 #(36) U_DSPCELL_MUX2_Z01_U0(
                .i0   ({Z1_U[17:0],Z0_U[17:0]} 	     ),
                .i1   ({{9{w27_z01_adds_u[26]}},w27_z01_adds_u} ),
                .sel  (cfg_mult_u[3]  ),
                .o    (w36_z01_u )
);
DSPCELL_MUX2 #(27) U_DSPCELL_MUX2_Z3_U0(
                .i0   ({{7{Z3_U[19]}},Z3_U[19:0]} 	     ),
                .i1   ({Z3_U[17:0],{9{1'b0}}} ),
                .sel  (cfg_mult_u[7]  ),
                .o    (w27_z3_u )
);
DSPCELL_ADD #(27)   U_DSPCELL_ADD27_Z23_U0  (  //s = i0 + i1
                     .i0   ({{7{Z2_U[19]}},Z2_U[19:0]}     ),
                     .i1   (w27_z3_u     ),
                     .s    (w27_z23_adds_u     ),
                     .c    (       )  
);
DSPCELL_MUX2 #(29) U_DSPCELL_MUX2_Z23_U0(
                .i0   ({w27_z23_adds_u[19:0],{9{1'b0}}}            ),
                .i1   ({{2{w27_z23_adds_u[26]}},w27_z23_adds_u} ),
                .sel  (cfg_mult_u[5]                   ),
                .o    (w29_z23_u0                         )
);
DSPCELL_MUX2 #(29) U_DSPCELL_MUX2_Z23_U1(
                .i0   (w29_z23_u0            ),
                .i1   ({29{1'b0}}  ),
                .sel  (cfg_mult_u[6]                   ),
                .o    (w29_z23_u1                         )
);
DSPCELL_ADD #(36)   U_DSPCELL_ADD36_Z0123_U0  (  //s = i0 + i1
                     .i0   (w36_z01_u     ),
                     .i1   ({{7{w29_z23_u1[28]}},w29_z23_u1}     ),
                     .s    (w36_mabu     ),
                     .c   (       )  
);


DSPCELL_SELBUF #(36) U_DSPCELL_SELBUF_U1(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_u           ),       
                     .rst_n(rst_n         ),       
                     .i    (w36_mabu      ),     
                     .sel  (cfg_mult_u[0] ),       
                     .o    (w36_mabu1     )      
);
DSPCELL_SELBUF #(36) U_DSPCELL_SELBUF_U2(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_u           ),       
                     .rst_n(rst_n         ),       
                     .i    (w36_mabu1     ),     
                     .sel  (cfg_mult_u[1] ),       
                     .o    (w36_mabu2     )    
);
//DSPCELL_MUX2 #(36) U_DSPCELL_MUX2_U3(
//                .i0   (w36_mabu2 ),
//                .i1   (36'h0     ),
//                .sel  (MCTRL[1]  ),
//                .o    (w36_mabu3 )
//);
DSPCELL_MUX2 #(36) U_DSPCELL_MUX2_U3(
                .i0   (36'h0 	 ),
                .i1   (w36_mabu2 ),
                .sel  (MCTRL[1]  ),
                .o    (w36_mabu3 )
);
//DSPCELL_MUX2 #(53) U_DSPCELL_MUX2_U4(
//                .i0   ({w36_mabu3,17'h0}            ),
//                .i1   ({{17{w36_mabu3[35]}},w36_mabu3} ),
//                .sel  (MCTRL[2]                     ),
//                .o    (w53_mabu4                    )
//);
//DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_U5(
//                .i0   ({{3{w53_mabu4[52]}},w53_mabu4}      ),
//                .i1   (U_buf                            ),
//                .sel  (MCTRL[3]                     ),
//                .o    (MR_U                         )
//);
DSPCELL_MUX4 #(56) U_DSPCELL_MUX4_U5(
                .i0   ({{3{w36_mabu3[35]}},w36_mabu3,17'h0}            ),
                .i1   ({{20{w36_mabu3[35]}},w36_mabu3} ),
                .i2   ({{4{w36_mabu3[35]}},w36_mabu3,16'h0}            ),
                .i3   (U_buf                            ),
                .sel  (MCTRL[3:2]                   ),
                .o    (MR_U                         )
);



//=============================================
//DSPCELL_MULT #(18) U_DSPCELL_MULT_MABL(
//                     .i0 (MA_L      ),
//                     .i1 (MB_L      ),
//                     .o  (w36_mabl  ) 
//);

DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X0_9_L0(
                .i0   (1'b0 	 ),
                .i1   (MX_L_SIGNED[0] ),
                .sel  (cfg_mult_l[2]),
                .o    (X0_L[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y0_9_L0(
                .i0   (1'b0 	 ),
                .i1   (MY_L_SIGNED[0] ),
                .sel  (cfg_mult_l[2]),
                .o    (Y0_L[9] )
);
assign X0_L[8:0] = MA_L[8:0];
assign Y0_L[8:0] = MB_L[8:0];
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABL0(
                     .i0 (X0_L      ),
                     .i1 (Y0_L      ),
                     .o  (Z0_L      ) 
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X1_9_L0(
                .i0   (MA_L[17] 	 ),
                .i1   (MX_L_SIGNED[1] ),
                .sel  (cfg_mult_l[2]),
                .o    (X1_L[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y1_9_L0(
                .i0   (MB_L[17] 	 ),
                .i1   (MY_L_SIGNED[1] ),
                .sel  (cfg_mult_l[2]),
                .o    (Y1_L[9] )
);
assign X1_L[8:0] = MA_L[17:9];
assign Y1_L[8:0] = MB_L[17:9];
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABL1(
                     .i0 (X1_L      ),
                     .i1 (Y1_L      ),
                     .o  (Z1_L      ) 
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X2_9_L0(
                .i0   (1'b0 	 ),
                .i1   (MX_L_SIGNED[2] ),
                .sel  (cfg_mult_l[2]),
                .o    (X2_L[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y2_9_L0(
                .i0   (MB_L[17] 	 ),
                .i1   (MY_L_SIGNED[2] ),
                .sel  (cfg_mult_l[2]),
                .o    (Y2_L[9] )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_X2_L0(
                .i0   (MA_L[8:0]  ),
                .i1   (MD_L[8:0] ),
                .sel  (cfg_mult_l[2]          ),
                .o    (X2_L[8:0]              )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_Y2_L0(
                .i0   (MB_L[17:9] 	 ),
                .i1   (MF_L[8:0] ),
                .sel  (cfg_mult_l[2]  ),
                .o    (Y2_L[8:0] )
);
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABL2(
                     .i0 (X2_L      ),
                     .i1 (Y2_L      ),
                     .o  (Z2_L      ) 
);
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_L0(
                .i0   (Z2_L[19:0]  ),
                .i1   (w27_z23_adds_l[19:0] ),
                .sel  (cfg_mult_l[8]            ),
                .o    (w20_18x9_bit19_0_l0                )
);
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_L2(
                .i1   ({20{1'b0}}  ),
                .i0   (w20_18x9_bit19_0_l0[19:0] ),
                .sel  (MCTRL[6]                  ),
                .o    (w20_18x9_bit19_0_l1                )
);
DSPCELL_SELBUF #(20) U_DSPCELL_SELBUF_Z2_L0(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_l           ),       
                     .rst_n(rst_n         ),       
                     .i    (w20_18x9_bit19_0_l1      ),     
                     .sel  (cfg_poa_rout_ext[0] ),       
                     .o    (R_EXT[19:0]     )      
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_X3_9_L0(
                .i0   (1'b0 	 ),
                .i1   (MX_L_SIGNED[3] ),
                .sel  (cfg_mult_l[2]),
                .o    (X3_L[9] )
);
DSPCELL_MUX2 #(1) U_DSPCELL_MUX2_Y3_9_L0(
                .i0   (MA_L[17] 	 ),
                .i1   (MY_L_SIGNED[3] ),
                .sel  (cfg_mult_l[2]),
                .o    (Y3_L[9] )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_X3_L0(
                .i0   (MB_L[8:0]  ),
                .i1   (MD_L[17:9] ),
                .sel  (cfg_mult_l[2]            ),
                .o    (X3_L[8:0]                )
);
DSPCELL_MUX2 #(9) U_DSPCELL_MUX2_Y3_L0(
                .i0   (MA_L[17:9] 	     ),
                .i1   (MF_L[17:9] ),
                .sel  (cfg_mult_l[2]  ),
                .o    (Y3_L[8:0] )
);
DSPCELL_MULT #(10) U_DSPCELL_MULT_MABL3(
                     .i0 (X3_L      ),
                     .i1 (Y3_L      ),
                     .o  (Z3_L  ) 
);
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_L1(
                .i0   (Z3_L[19:0]  ),
                .i1   ({{13{w27_z23_adds_l[26]}},w27_z23_adds_l[26:20]} ),
                .sel  (cfg_mult_l[8]            ),
                .o    (w20_18x9_bit39_20_l0                )
);
DSPCELL_MUX2 #(20) U_DSPCELL_MUX2_ADDZ23_L3(
                .i1   ({20{1'b0}}  ),
                .i0   (w20_18x9_bit39_20_l0[19:0] ),
                .sel  (MCTRL[6]                   ),
                .o    (w20_18x9_bit39_20_l1                )
);
DSPCELL_SELBUF #(20) U_DSPCELL_SELBUF_Z3_L0(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_l           ),       
                     .rst_n(rst_n         ),       
                     .i    (w20_18x9_bit39_20_l1      ),     
                     .sel  (cfg_poa_rout_ext[1] ),       
                     .o    (R_EXT[39:20]     )      
);
DSPCELL_MUX2 #(27) U_DSPCELL_MUX2_Z1_L0(
                .i0   ({{7{Z1_L[19]}},Z1_L[19:0]} 	     ),
                .i1   ({Z1_L[17:0],{9{1'b0}}} ),
                .sel  (cfg_mult_l[4]  ),
                .o    (w27_z1_l )
);
DSPCELL_ADD #(27)   U_DSPCELL_ADD27_Z01_L0  (  //s = i0 + i1
                     .i0   ({{7{Z0_L[19]}},Z0_L[19:0]}     ),
                     .i1   (w27_z1_l     ),
                     .s    (w27_z01_adds_l     ),
                     .c    (       )  
);
DSPCELL_MUX2 #(36) U_DSPCELL_MUX2_Z01_L0(
                .i0   ({Z1_L[17:0],Z0_L[17:0]} 	     ),
                .i1   ({{9{w27_z01_adds_l[26]}},w27_z01_adds_l} ),
                .sel  (cfg_mult_l[3]  ),
                .o    (w36_z01_l )
);             
DSPCELL_MUX2 #(27) U_DSPCELL_MUX2_Z3_L0(
                .i0   ({{7{Z3_L[19]}},Z3_L[19:0]} 	     ),
                .i1   ({Z3_L[17:0],{9{1'b0}}} ),
                .sel  (cfg_mult_l[7]  ),
                .o    (w27_z3_l )
);
DSPCELL_ADD #(27)   U_DSPCELL_ADD27_Z23_L0  (  //s = i0 + i1
                     .i0   ({{7{Z2_L[19]}},Z2_L[19:0]}     ),
                     .i1   (w27_z3_l     ),
                     .s    (w27_z23_adds_l     ),
                     .c   (       )  
);
DSPCELL_MUX2 #(29) U_DSPCELL_MUX2_Z23_L0(
                .i0   ({w27_z23_adds_l[19:0],{9{1'b0}}}            ),
                .i1   ({{2{w27_z23_adds_l[26]}},w27_z23_adds_l} ),
                .sel  (cfg_mult_l[5]                   ),
                .o    (w29_z23_l0                         )
);
DSPCELL_MUX2 #(29) U_DSPCELL_MUX2_Z23_L1(
                .i0   (w29_z23_l0           ),
                .i1   ({29{1'b0}} ),
                .sel  (cfg_mult_l[6]                   ),
                .o    (w29_z23_l1                         )
);
DSPCELL_ADD #(36)   U_DSPCELL_ADD36_Z0123_L0  (  //s = i0 + i1
                     .i0   (w36_z01_l     ),
                     .i1   ({{7{w29_z23_l1[28]}},w29_z23_l1}     ),
                     .s    (w36_mabl     ),
                     .c   (       )  
);

DSPCELL_SELBUF #(36) U_DSPCELL_SELBUF_L1(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_l           ),       
                     .rst_n(rst_n         ),
                     .i    (w36_mabl      ),     
                     .sel  (cfg_mult_l[0] ),       
                     .o    (w36_mabl1     )      
);
DSPCELL_SELBUF #(36) U_DSPCELL_SELBUF_L2(  //assign o = sel == 0 ? o_buf :  i
                     .clk  (clk_mult_l           ),       
                     .rst_n(rst_n         ),
                     .i    (w36_mabl1     ),     
                     .sel  (cfg_mult_l[1] ),       
                     .o    (w36_mabl2     )    
);
//DSPCELL_MUX2 #(36) U_DSPCELL_MUX2_L3(
//                .i0   (w36_mabl2 ),
//                .i1   (36'h0     ),
//                .sel  (MCTRL[0]  ),
//                .o    (w36_mabl3 )
//);
DSPCELL_MUX2 #(36) U_DSPCELL_MUX2_L3(
                .i0   (36'h0 	 ),
                .i1   (w36_mabl2 ),
                .sel  (MCTRL[0]  ),
                .o    (w36_mabl3 )
);
DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_L4(
                .i0   ({{20{w36_mabl3[35]}},w36_mabl3} ),
                .i1   (w56_mcat    ),
                .sel  (MCTRL[4]                     ),
                .o    (MR_L_t                         )
);
DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_L5(
                .i0   (MR_L_t),
                .i1   (R[55:0]),
                .sel  (MCTRL[5]                     ),
                .o    (MR_L                         )
);


assign  w56_mcat = {MA_U[1:0],MB_U[17:0],MB_L[17:0],MA_L[17:0]};



endmodule
//*************************************************
//company:   capital-micro
//author:   hongyu.wang
//date:      20140526
//function:   `DSP
//*************************************************
//$Log: .v,v $


`ifndef DLY
`define DLY #0.1
`endif


module dsp_postadder(
                        clk_poa             ,                  
                        rst_n           ,                    
                        MR_U            ,                   
                        MR_L            ,                   
                        UFC             ,  
                        Z0_U            ,
                        Z1_U            ,
                        Z0_L            ,
                        Z1_L            ,     
                        R_CARRY_CAS_IN  ,                              
                        R_CARRY_CAS_OUT ,                              
                        R_ECARRY_CAS_IN  ,                              
                        R_ECARRY_CAS_OUT ,                              
                        R_pre           ,                    
                        R               ,                    
                        R_CAS_OUT       ,
                        MATCH           ,
                        RNDM            ,                      
                        SUBCTRL         ,                      
                        ALUCTRL         ,                      
                        FRCTRL          ,                      
                        CCTRL           ,                      
                        cfg_mat_clr     ,
                        cfg_poa_cyout   ,                            
                        cfg_poa_ceyout  ,                            
                        cfg_poa_ceyin   ,                            
                        cfg_poa_rout                                 
                  );

 

//clock and reset
input           clk_poa;
input           rst_n;   //sys reset

input    [55:0]  MR_U;               // 56  I MultBlock MultBlock calculate result
input    [55:0]  MR_L;               // 56  I MultBlock MultBlock calculate result
input    [55:0]  UFC ;               // 56  I UFCMux  UFC Mux select result
input    [17:0]  Z0_U;               // 18 10*10 mode low 18 bit [17:0] out
input    [17:0]  Z1_U;               // 18 10*10 mode low 18 bit [17:0] out
input    [17:0]  Z0_L;               // 18 10*10 mode low 18 bit [17:0] out
input    [17:0]  Z1_L;               // 18 10*10 mode low 18 bit [17:0] out
input            R_CARRY_CAS_IN ;    // 1   I Input Carry cascade in
output           R_CARRY_CAS_OUT;    // 1   O Output  Carry cascade out
input            R_ECARRY_CAS_IN ;    // 1   I Input Carry cascade in
output           R_ECARRY_CAS_OUT;    // 1   O Output  Carry cascade out
output   [55:0]  R_pre;              // 56  O R output before buffer and mux, to CompUnit
output   [71:0]  R    ;              // 56  O MAC calculate Output
output   [55:0]  R_CAS_OUT    ;      // 
input            MATCH;
input            RNDM   ;            // 1   I Input Round Control
input    [2:0]   SUBCTRL;            // 3   I Postadder Select and Control to perform Subtraction in post stage operation 
input    [2:0]   ALUCTRL;            // 2   I Postadder Control the functionality of ALU operation 
input    [1:0]   FRCTRL ;            // 2   I Postadder Select final result source 
input    [2:0]   CCTRL  ;            // 3   I Postadder Select the source for rounding method, and perform R_CARRY_CAS_IN and user defined CARRY logic 
        
input            cfg_mat_clr     ;
input            cfg_poa_cyout;      // 1   I Input   cfgmem,postadder carry out control
input           cfg_poa_ceyout;    
input            cfg_poa_ceyin;    
input    [2:0]   cfg_poa_rout ;      // 3   I Input   cfgmem,postadder R out control
        
//======================================      
reg         R_S_55_reg;
wire        w1_poa0;
wire        w1_poa1;
wire [56:0] w57_poa2;
wire [55:0] w56_poa3;
wire [71:0] w56_poa4_pre;
wire [71:0] w56_poa4;
reg  [71:0] w56_poa4_reg;
wire [55:0] w56_poa5;
wire        carry;
wire [55:0] A;
wire [55:0] B;
wire [55:0] U;
wire [55:0] S;
wire [55:0] C;
wire [55:0] R_S;
wire [55:0] MR_U_temp; 

always @(posedge clk_poa or negedge rst_n) begin
    if(rst_n == 0) begin
         R_S_55_reg <= `DLY 1'b0;
    end
    else begin
         R_S_55_reg <= `DLY R_S[55];
    end
end
DSPCELL_MUX4 #(1) U_PostAdder_Mux_PO0(
                          .i0  (1'h0         ),     
                          .i1  (R_S_55_reg    ),     
                          .i2  (MR_L[55]      ),     
                          .i3  (MR_U[55]      ),     
                          .sel (CCTRL[2:1]    ),      
                          .o   (w1_poa0       )  
);
//DSPCELL_MUX2 #(1) U_PostAdder_Mux_PO1(
//                          .i0  (R_CARRY_CAS_IN),
//                          .i1  (RNDM          ),     
//                          .sel (CCTRL[0]    ),      
//                          .o   (w1_poa1       )  
//);
DSPCELL_MUX2 #(1) U_PostAdder_Mux_PO1(
                          .i0  (RNDM		),
                          .i1  (R_CARRY_CAS_IN 	),     
                          .sel (CCTRL[0]    ),      
                          .o   (w1_poa1       )  
);
assign carry = w1_poa0 ^ w1_poa1;
//input            R_ECARRY_CAS_IN ;    // 1   I Input Carry cascade in
//output           R_ECARRY_CAS_OUT;    // 1   O Output  Carry cascade out
//wire      [55:0] MR_U_temp; 
DSPCELL_ADD #(57) U_DSPCELL_ADD_PO2(  //s = i0 + i1
                          .i0  ({1'b0,S[55:0]}    ), 
                          .i1  ({C[55:0],carry}   ), 
                          .s   (w57_poa2          ),
                          .c   (R_ECARRY_CAS_OUT_t  ) 
);
DSPCELL_SELBUF U_DSPCELL_SELBUF_PO3E (  //assign o = sel == 0 ? o_buf :  i
                          .clk       (clk_poa            ),
                          .rst_n     (rst_n          ),  
                          .i         (R_ECARRY_CAS_OUT_t   ),
                          .sel       (cfg_poa_ceyout  ),
                          .o         (R_ECARRY_CAS_OUT) 
);
assign R_S = w57_poa2[55:0];
DSPCELL_SELBUF U_DSPCELL_SELBUF_PO3(  //assign o = sel == 0 ? o_buf :  i
                          .clk       (clk_poa            ),
                          .rst_n     (rst_n          ),  
                          .i         (w57_poa2[56]    ),
                          .sel       (cfg_poa_cyout  ),
                          .o         (R_CARRY_CAS_OUT) 
);

//######################
DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_PO4E(
                          .i0  (MR_U                         ),
                          .i1  ({54'h0,R_ECARRY_CAS_IN,1'b0} ),     
                          .sel (cfg_poa_ceyin                ),        
                          .o   (MR_U_temp                    )  
);

DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_PO4(
                          .i0  (MR_U_temp   ),
                          .i1  (~MR_U_temp  ),     
                          .sel (SUBCTRL[2]  ),      
                          .o   (A           )  
);
DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_PO5(
                          .i0  (MR_L        ),
                          .i1  (~MR_L       ),     
                          .sel (SUBCTRL[1]  ),      
                          .o   (B           )  
);
DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_PO6(
                          .i0  (UFC         ),
                          .i1  (~UFC        ),     
                          .sel (SUBCTRL[0]  ),      
                          .o   (U           )  
);
DSPCELL_BIT_ADD #(56) U_DSPCELL_BIT_ADD_PO7 (
                          .i0  (A  ),
                          .i1  (B  ),
                          .i2  (U  ),
                          .s   (S  ),
                          .c   (C  )  
);
DSPCELL_MUX4 #(56) U_DSPCELL_MUX4_PO8(
                          .i0  (R_S         ),
                          .i1  (S           ),     
                          .i2  (C           ),     
                          .i3  (56'h0       ),     
                          .sel (ALUCTRL[1:0]),      
                          .o   (w56_poa3    )  
);
wire [55:0] w56_poa3_mux;
wire        w1_poa3_sel;
assign      w1_poa3_sel = MATCH && cfg_mat_clr;
DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_PO9E(
                          .i0  (w56_poa3    ),
                          .i1  (56'h0       ),     
                          .sel (w1_poa3_sel ),      
                          .o   (w56_poa3_mux )  
);

DSPCELL_MUX2 #(56) U_DSPCELL_MUX2_PO9(
                          .i0  (w56_poa3_mux    ),
                          .i1  (~w56_poa3_mux   ),     
                          .sel (ALUCTRL[2]  ),      
                          .o   (R_pre       )  
);
//DSPCELL_SELBUF #(56)  U_DSPCELL_SELBUF_PO10 (  //assign o = sel == 0 ? o_buf :  i
//                     .clk      (clk_poa              ),
//                     .rst_n    (rst_n            ),  
//                     .i        (R_pre            ),
//                     .sel      (cfg_poa_rout[0]  ),
//                     .o        (w56_poa4         ) 
//);
//DSPCELL_SELBUF #(56)  U_DSPCELL_SELBUF_PO11 (  //assign o = sel == 0 ? o_buf :  i
//                     .clk      (clk_poa                    ),
//                     .rst_n    (rst_n                  ),  
//                     .i        ({MR_U[27:0],MR_L[27:0]} ),
//                     .sel      (cfg_poa_rout[1]  ),
//                     .o        (w56_poa5         ) 
//);
//DSPCELL_MUX4 #(56) U_DSPCELL_MUX4_PO12(
//                          .i0  (R_pre         ),
//                          .i1  (MR_L          ),     
//                          .i2  ({MR_U[27:0],MR_L[27:0]}  ),     
//                          .i3  (56'h0         ),     
//                          .sel (FRCTRL[1:0]   ), 
//                          .o   (w56_poa4      )  
//);
DSPCELL_MUX2 #(72) U_DSPCELL_MUX2_PRE_PO10(
                          .i0  ({MR_U[35:0],MR_L[35:0]}  ),
                          .i1  ({Z1_U[17:0],Z0_U[17:0],Z1_L[17:0],Z0_L[17:0]}),     
                          .sel (cfg_poa_rout[2]   ), 
                          .o   (w56_poa4_pre      )  
);
DSPCELL_MUX2 #(72) U_DSPCELL_MUX2_PO10(
                          .i0  ({{16{R_pre[55]}},R_pre}  ),
                          .i1  (w56_poa4_pre             ),     
                          .sel (FRCTRL[0]   ), 
                          .o   (w56_poa4      )  
);
//DSPCELL_SELBUF #(56)  U_DSPCELL_SELBUF_PO10 (  //assign o = sel == 0 ? o_buf :  i
//                     .clk      (clk_poa              ),
//                     .rst_n    (rst_n            ),  
//                     .i        (w56_poa4 ),
//                     .sel      (cfg_poa_rout[0]  ),
//                     .o        (R         ) 
//);
always @(posedge clk_poa or negedge rst_n) begin
    if(rst_n == 0) begin
         w56_poa4_reg <= 56'b0;
    end
    else begin
         w56_poa4_reg <= w56_poa4;
    end
end
DSPCELL_MUX2 #(72)  U_DSPCELL_MUX2_PO11 (
                     .i0       	(w56_poa4_reg 	),
		     .i1	(w56_poa4	),
                     .sel      	(cfg_poa_rout[0]),
                     .o        	(R         ) 
);
//DSPCELL_SELBUF #(56)  U_DSPCELL_SELBUF_PO11 (  //assign o = sel == 0 ? o_buf :  i
//                     .clk      (clk_poa                    ),
//                     .rst_n    (rst_n                  ),  
//                     .i        (w56_poa4 ),
//                     .sel      (cfg_poa_rout[1]  ),
//                     .o        (R_CAS_OUT         ) 
//);
DSPCELL_MUX2 #(56)  U_DSPCELL_MUX2_PO12 (
                     .i0        (w56_poa4_reg[55:0] ),
		     .i1	(w56_poa4[55:0]	    ),
                     .sel      	(cfg_poa_rout[1]    ),
                     .o        	(R_CAS_OUT         ) 
);


endmodule   

module DSP56V4 (
	alcasout,
	aucasout,
	clcasout,
	cucasout,
	imatch,
	match,
	overflow,
	r,
	r_ext,
	rcarrycasout,
	rcasout,
	recarrycasout,
	underflow,
	z01_18,
	al,
	alcasin,
	au,
	aucasin,
	rstn,
	cecomp,
	ceibuf,
	cemultl,
	cemultu,
	cepoa,
	cepra,
	ceufc,
	cl,
	clcasin,
	cu,
	cucasin,
	dl,
	du,
	fl,
	fu,
	ialuctrl,
	iblctrl,
	ibuctrl,
	icctrl,
	ifcctrl,
	ifrctrl,
	imctrl,
	iparctrl,
	ipasctrl,
	ipasubctrl,
	ipazctrl,
	irndm,
	isubctrl,
	iufcctrl,
	rcarrycasin,
	rcasin,
	clk,
	recarrycasin,
	u,
	x_l_signed,
	x_u_signed,
	y_l_signed,
	y_u_signed 
);

output	[17:0]	alcasout;
output	[17:0]	aucasout;
output	[17:0]	clcasout;
output	[17:0]	cucasout;
output		imatch;
output		match;
output		overflow;
output	[71:0]	r;
output	[79:0]	r_ext;
output		rcarrycasout;
output	[55:0]	rcasout;
output		recarrycasout;
output		underflow;
output	[3:0]	z01_18;
input	[17:0]	al;
input	[17:0]	alcasin;
input	[17:0]	au;
input	[17:0]	aucasin;
input		rstn;
input		cecomp;
input		ceibuf;
input		cemultl;
input		cemultu;
input		cepoa;
input		cepra;
input		ceufc;
/*
*/
parameter cfg_aluctrl  = 1'b0;
parameter cfg_blctrl = 1'b0;
parameter cfg_buctrl = 1'b0;
parameter cfg_cctrl = 1'b0;
parameter cfg_fcctrl = 1'b0;
parameter cfg_frctrl = 1'b0;
parameter cfg_imatch = 1'b0;
parameter cfg_mat_clr = 1'b0;
parameter cfg_match = 1'b0;
parameter cfg_mctrl = 1'b0;
parameter cfg_mult_l = 9'b0;
parameter cfg_mult_u = 9'b0;
parameter cfg_parctrl = 1'b0;
parameter cfg_pasctrl = 1'b0;
parameter cfg_pasubctrl = 1'b0;
parameter cfg_pazctrl = 1'b0;
parameter cfg_poa_ceyin = 1'b0;
parameter cfg_poa_ceyout = 1'b0;
parameter cfg_poa_cyout = 1'b0;
parameter cfg_poa_rout = 5'b0;
parameter cfg_poa_rout_ext = 4'b0;
parameter cfg_pra_al=3'b0;
parameter cfg_pra_au=3'b0;
parameter cfg_pra_cl=3'b0;
parameter cfg_pra_cu=3'b0;
parameter cfg_pra_dl = 1'b0;
parameter cfg_pra_du = 1'b0;
parameter cfg_pra_fl = 1'b0;
parameter cfg_pra_fu = 1'b0;
parameter cfg_pra_ol = 1'b0;
parameter cfg_pra_ou = 1'b0;
parameter cfg_rndm = 1'b0;
parameter cfg_subctrl = 1'b0;
parameter cfg_ufcctrl = 1'b0;
parameter cfg_ufcmux = 1'b0;
parameter cfg_vldbit = 56'b0;
parameter cfg_hasclk = 1'b0;

input	[17:0]	cl;
input	[17:0]	clcasin;
input	[17:0]	cu;
input	[17:0]	cucasin;
input	[17:0]	dl;
input	[17:0]	du;
input	[17:0]	fl;
input	[17:0]	fu;
input	[2:0]	ialuctrl;
input		iblctrl;
input		ibuctrl;
input	[2:0]	icctrl;
input	[2:0]	ifcctrl;
input	[1:0]	ifrctrl;
input	[7:0]	imctrl;
input	[5:0]	iparctrl;
input	[7:0]	ipasctrl;
input	[1:0]	ipasubctrl;
input	[7:0]	ipazctrl;
input		irndm;
input	[2:0]	isubctrl;
input		iufcctrl;
input		rcarrycasin;
input	[55:0]	rcasin;
input     	clk;
input		recarrycasin;
input	[55:0]	u;
input	[3:0]	x_l_signed;
input	[3:0]	x_u_signed;
input	[3:0]	y_l_signed;
input	[3:0]	y_u_signed;

wire		gnd;
wire		hasclk;
wire		rstn_t;
wire		vcc;
wire        clk_g = clk & cfg_hasclk;

dsp_core dsp_core (
	.al			(al[17:0]),
	.alcasin		(alcasin[17:0]),
	.au			(au[17:0]),
	.aucasin		(aucasin[17:0]),
	.cecomp			(cecomp),
	.ceibuf			(ceibuf),
	.cemultl		(cemultl),
	.cemultu		(cemultu),
	.cepoa			(cepoa),
	.cepra			(cepra),
	.ceufc			(ceufc),
	.cfg_aluctrl		(cfg_aluctrl),
	.cfg_blctrl		(cfg_blctrl),
	.cfg_buctrl		(cfg_buctrl),
	.cfg_cctrl		(cfg_cctrl),
	.cfg_fcctrl		(cfg_fcctrl),
	.cfg_frctrl		(cfg_frctrl),
	.cfg_imatch		(cfg_imatch),
	.cfg_mat_clr		(cfg_mat_clr),
	.cfg_match		(cfg_match),
	.cfg_mctrl		(cfg_mctrl),
	.cfg_mult_l		(cfg_mult_l[8:0]),
	.cfg_mult_u		(cfg_mult_u[8:0]),
	.cfg_parctrl		(cfg_parctrl),
	.cfg_pasctrl		(cfg_pasctrl),
	.cfg_pasubctrl		(cfg_pasubctrl),
	.cfg_pazctrl		(cfg_pazctrl),
	.cfg_poa_ceyin		(cfg_poa_ceyin),
	.cfg_poa_ceyout		(cfg_poa_ceyout),
	.cfg_poa_cyout		(cfg_poa_cyout),
	.cfg_poa_rout		(cfg_poa_rout[4:0]),
	.cfg_poa_rout_ext	(cfg_poa_rout_ext[3:0]),
	.cfg_pra_al		(cfg_pra_al[2:0]),
	.cfg_pra_au		(cfg_pra_au[2:0]),
	.cfg_pra_cl		(cfg_pra_cl[2:0]),
	.cfg_pra_cu		(cfg_pra_cu[2:0]),
	.cfg_pra_dl		(cfg_pra_dl),
	.cfg_pra_du		(cfg_pra_du),
	.cfg_pra_fl		(cfg_pra_fl),
	.cfg_pra_fu		(cfg_pra_fu),
	.cfg_pra_ol		(cfg_pra_ol),
	.cfg_pra_ou		(cfg_pra_ou),
	.cfg_rndm		(cfg_rndm),
	.cfg_subctrl		(cfg_subctrl),
	.cfg_ufcctrl		(cfg_ufcctrl),
	.cfg_ufcmux		(cfg_ufcmux),
	.cfg_vldbit		(cfg_vldbit[55:0]),
	.cl			(cl[17:0]),
	.clcasin		(clcasin[17:0]),
	.clk			(clk_g),
	.cu			(cu[17:0]),
	.cucasin		(cucasin[17:0]),
	.dl			(dl[17:0]),
	.du			(du[17:0]),
	.fl			(fl[17:0]),
	.fu			(fu[17:0]),
	.ialuctrl		(ialuctrl[2:0]),
	.iblctrl		(iblctrl),
	.ibuctrl		(ibuctrl),
	.icctrl			(icctrl[2:0]),
	.ifcctrl		(ifcctrl[2:0]),
	.ifrctrl		(ifrctrl[1:0]),
	.imctrl			(imctrl[7:0]),
	.iparctrl		(iparctrl[5:0]),
	.ipasctrl		(ipasctrl[7:0]),
	.ipasubctrl		(ipasubctrl[1:0]),
	.ipazctrl		(ipazctrl[7:0]),
	.irndm			(irndm),
	.isubctrl		(isubctrl[2:0]),
	.iufcctrl		(iufcctrl),
	.rcarrycasin		(rcarrycasin),
	.rcasin			(rcasin[55:0]),
	.recarrycasin		(recarrycasin),
	.rstn			(rstn),
	.u			(u[55:0]),
	.x_l_signed		(x_l_signed[3:0]),
	.x_u_signed		(x_u_signed[3:0]),
	.y_l_signed		(y_l_signed[3:0]),
	.y_u_signed		(y_u_signed[3:0]),

	.alcasout		(alcasout[17:0]),
	.aucasout		(aucasout[17:0]),
	.clcasout		(clcasout[17:0]),
	.cucasout		(cucasout[17:0]),
	.imatch			(imatch),
	.match			(match),
	.overflow		(overflow),
	.r			(r[71:0]),
	.r_ext			(r_ext[79:0]),
	.rcarrycasout		(rcarrycasout),
	.rcasout		(rcasout[55:0]),
	.recarrycasout		(recarrycasout),
	.underflow		(underflow),
	.z01_18			(z01_18[3:0]) 
);

endmodule

module ram4kx32 ( clk,clken,ceb,web,bl, adr, wdat,dout); 
input clk;
input clken;
input ceb;
input web;
input [3:0] bl;
input [11:0] adr;
input [31:0] wdat;
output [31:0] dout;

parameter SLP = 1'b0;
parameter SD = 1'b0;

parameter SRAM_SEL = 1'b0; //software flow only
parameter init_file = "";

reg [31:0] mem [4*1024-1 : 0];
reg [31:0] mem_init[4*1024-1:0];
reg [31:0] dout;

wire clkg = clk & clken;

integer i, j;
initial begin
    i = 0;

    // Initialize mem_data
    if (init_file == " " || init_file == "" || init_file == "none") begin
            //$display("VMB has no data file for memory initialization.\n");
    end else begin// Memory initialization file is used
        $display("Initialize the sram: %s.", init_file);
        $readmemh(init_file, mem_init);
        #1;
//      for(i=0; i<32*1024; i=i+1) begin
//          $display("%d\t%0x", i, mem_init[i]);
//      end

        if(!SLP && !SD) begin  //
            for(i=0; i<4*1024; i=i+1) begin
                mem[i] = mem_init[i];
            end
		end

//        for(i=0; i<1024; i=i+1) begin
//            $display("%d\t%08x, %08x", i, mem_a[i], mem_b[i]);
//        end
    end //if
end  //inital

always @ (posedge clkg) begin
    if (!ceb && !SLP && !SD) begin
        if (!web) begin //write
            //for (j = 0 ; j < 4 ; j = j + 1) begin 
            //    if (bl[j] == 1'b0) 
            //        mem[adr][j*8+7:j*8] <= wdat[j*8+7:j*8] ;
            //end
            if(bl[0] == 1'b0)
                mem[adr][7:0] <= wdat[7:0];
            if(bl[1] == 1'b0)
                mem[adr][15:8] <= wdat[15:8];
            if(bl[2] == 1'b0)
                mem[adr][23:16] <= wdat[23:16];
            if(bl[3] == 1'b0)
                mem[adr][31:24] <= wdat[31:24];
            
        end else begin //read
            dout <= mem[adr];
        end
    end else begin
        dout <= 32'hxxxx_xxxx;
    end
end
endmodule


module PDIV_TOP ( CLKOUT, CLKIN, CLR );

output  CLKOUT;

input CLKIN, CLR;


parameter CFG_PDIV_EN = 1'b0;
parameter CFG_PDIV_BPS = 1'b0; 

parameter CFG_DIVNUM = 3'b000;
parameter CFG_DLYNUM = 3'b000;

//function description 
/* ------------
CLR: 1 rst;
DIV: 000: div1
     001: div2
     010: div3
     011: div4
     100: div5
     101: div6
     110: div7
     111: div8
------------ */
wire CKOUT1;
reg CLKG;
reg [3:0] DIVSOR;
wire CLKDLY;

assign CLKOUT = CFG_PDIV_EN == 1'b0 ? 1'b0:
               CFG_PDIV_BPS == 1'b1 ? CLKIN : CLKDLY;
reg SYNC;
wire UP = DIVSOR == CFG_DIVNUM;

wire ZERO = ~(|DIVSOR);
always @ (CLKIN)
    SYNC <= ZERO;

reg CLRS = 0;
always @ (negedge CLKIN)
    CLRS <= CLR;

always @ (CLKIN or posedge CLRS) begin
    if(CLRS)
        DIVSOR <= 0;
    else if(UP)
        DIVSOR <= 0;
    else
        DIVSOR <= DIVSOR+1;
end

always @ (posedge SYNC or posedge CLRS) begin
    if(CLRS)
        CLKG <= 1'b0;
    else
        CLKG <= ~CLKG;
end

assign CKOUT1 = CFG_DIVNUM == 3'b000 ? CLKIN : CLKG;

wire [7:0] clks;
assign clks[0]    = CKOUT1;
assign #1 clks[1] = CKOUT1;
assign #2 clks[2] = CKOUT1;
assign #3 clks[3] = CKOUT1;
assign #4 clks[4] = CKOUT1;
assign #5 clks[5] = CKOUT1;
assign #6 clks[6] = CKOUT1;
assign #7 clks[7] = CKOUT1;

assign CLKDLY = CFG_DLYNUM == 3'b000 ? clks[0] :
                CFG_DLYNUM == 3'b001 ? clks[1] :
                CFG_DLYNUM == 3'b010 ? clks[2] :
                CFG_DLYNUM == 3'b011 ? clks[3] :
                CFG_DLYNUM == 3'b100 ? clks[4] :
                CFG_DLYNUM == 3'b101 ? clks[5] :
                CFG_DLYNUM == 3'b110 ? clks[6] :
                CFG_DLYNUM == 3'b111 ? clks[7] : 1'bx;

endmodule


module RCOSC_TOP (RC_OSC);
output RC_OSC;

reg osc;
initial begin
    osc = 1'b0;
    forever #6.25 osc = ~osc;
end
assign RC_OSC = osc;

endmodule

module dss (
             dss_ckref,
             dss_dlock,
             dss_dtest,
             dss_of,
             dss_uf
);
input dss_ckref;
output dss_dlock;
output dss_dtest;
output dss_of;
output dss_uf;

endmodule 
module P2_GPIO_HR (RXD, PAD, TED, TXD);
input TED, TXD;
output  RXD;
inout  PAD;

parameter CFG_25_EN = 1'b0;//bank shared 
parameter CFG_MSC = 1'b0;  //bank shared
parameter CFG_NS_LV = 1'b0;
parameter CFG_RX_DIG_EN = 1'b0;
parameter CFG_RX_EN12 = 1'b0;
parameter CFG_RX_EN15 = 1'b0;
parameter CFG_RX_EN33 = 1'b0;
parameter CFG_RX_ST_EN = 1'b0;

parameter CFG_NDR = 4'b0000;
parameter CFG_PDR = 5'b00000;
parameter CFG_KEEP = 2'b00;
parameter CFG_RSE = 4'b0000;

reg   PAD_reg;         //if use always, output must be "reg"
reg   RXD;
wire   un_define;   //input is "x" or "z"
wire rx_en = CFG_RX_DIG_EN & ( CFG_MSC | CFG_RX_EN12 | CFG_RX_EN15 | CFG_RX_EN33 | CFG_RX_ST_EN );
wire tx_en = (~TED) & (CFG_PDR > 0 && CFG_NDR > 0);
assign un_define = rx_en & (&CFG_KEEP) & (&CFG_PDR) & (&CFG_NDR);

always @(*) begin
    if(un_define == 1'bx || un_define == 1'bz) //unconnected input
        PAD_reg  =  1'bz;
    else if(tx_en)   //normal output
        PAD_reg  =  TXD;
    else if(CFG_KEEP == 2'b01) //pull down
        PAD_reg  =  0;
    else if(CFG_KEEP == 2'b10) //pull up
        PAD_reg  =  1;
    else if(CFG_KEEP == 2'b11) //keep
        PAD_reg  =  PAD_reg;
    else
        PAD_reg  =  1'bz;
end

assign PAD  = PAD_reg;
always @(*) begin
    if(un_define == 1'bx || un_define == 1'bz) //unconnected input
        RXD  =  1'bz;
    else if(rx_en == 1)   //normal input
        RXD  =  PAD;
    else 
        RXD  =  1'b0;
end

endmodule

//ref: /home_hw/hme_center/hw/p2_center/circuit/verilog/array_block_cut_long/GPIO_HP.v

module P2_GPIO_HP ( 
    RXD, PAD, 
    TXD, TED,
    ODT,
    fp_pndr_update, fp_tpud_update,
    fp_ndr_cal, fp_tpu_cal, fp_tpd_cal, fp_pdr_cal
    );

//basi function
output  RXD;
inout  PAD;
input  TXD, TED;

input ODT;
//TXD_MAIN, TXD_POST, TXD_PRE <= TXD

//dynamic calibration
//input PNDR_RSTN, PNDR_UPDATE, TPUD_RSTN, TPUD_UPDATE;
//RSTN <-- glb_clear_b

input fp_pndr_update, fp_tpud_update;
input [6:0]  fp_ndr_cal;
input [6:0]  fp_tpu_cal;
input [6:0]  fp_tpd_cal;
input [6:0]  fp_pdr_cal;

//paras
parameter /*[1:0]*/  CFG_KEEP = 2'b00;
parameter /*[1:0]*/  CFG_NS_LV = 2'b00;
parameter CFG_RX_DIG_EN = 1'b0;
parameter CFG_RX_SINGLE_EN = 1'b0;
parameter CFG_ST = 1'b0;

parameter /*[6:0]*/  CFG_NDR = 7'b0000000;
parameter /*[6:0]*/  CFG_PDR = 7'b0000000;
parameter CFG_PNDR_CAL_EN = 1'b0;
parameter CFG_PNDR_UD_BYP = 1'b0;

parameter /*[6:0]*/  CFG_TPD = 7'b0000000;
parameter /*[6:0]*/  CFG_TPU = 7'b0000000;
parameter CFG_TPD_CAL_EN = 1'b0;
parameter CFG_TPU_CAL_EN = 1'b0;
parameter CFG_TPUD_UD_BYP = 1'b0;

parameter /*[2:0]*/ CFG_TX_POST = 3'b000;
parameter /*[2:0]*/ CFG_TX_PRE = 3'b000;
parameter CFG_TX_POST_EN = 1'b0;
parameter CFG_TX_PRE_EN = 1'b0;

parameter CFG_SMIT_TUNE_EN = 1'b0;

parameter CFG_VREF_EN = 1'b0;

/* dynamic config pdr/ndr */
wire [6:0] ndr_mux = CFG_PNDR_CAL_EN == 1'b1 ? fp_ndr_cal : CFG_NDR;
wire [6:0] pdr_mux = CFG_PNDR_CAL_EN == 1'b1 ? fp_pdr_cal : CFG_PDR;
reg [6:0] ndr_reg = 7'bxxxxxxx;
reg [6:0] pdr_reg = 7'bxxxxxxx;
always @ (posedge fp_pndr_update) begin
    pdr_reg <= pdr_mux;
    ndr_reg <= ndr_mux;
end
wire [6:0] ndr = CFG_PNDR_UD_BYP == 1'b1 ? ndr_mux : ndr_reg;
wire [6:0] pdr = CFG_PNDR_UD_BYP == 1'b1 ? pdr_mux : pdr_reg;

/* dynamic config tpu/tpd */
wire [6:0] tpd_mux = CFG_TPD_CAL_EN == 1'b1 ? fp_tpd_cal : CFG_TPD;
wire [6:0] tpu_mux = CFG_TPU_CAL_EN == 1'b1 ? fp_tpu_cal : CFG_TPU;
reg [6:0] tpd_reg = 7'bxxxxxxx;
reg [6:0] tpu_reg = 7'bxxxxxxx;
always @ (posedge fp_tpud_update) begin
    tpd_reg <= tpd_mux;
    tpu_reg <= tpu_mux;
end
wire [6:0] tpd = CFG_TPUD_UD_BYP == 1'b1 ? tpd_mux : tpd_reg;
wire [6:0] tpu = CFG_TPUD_UD_BYP == 1'b1 ? tpu_mux : tpu_reg;

//paramter check
wire [1:0] rxen_sel = {CFG_RX_SINGLE_EN, CFG_RX_DIG_EN};


reg   PAD_reg;         //if use always, output must be "reg"
wire   un_define;   //input is "x" or "z"

wire rx_en = rxen_sel == 2'b10 ? (ODT === 1'b1 ? CFG_VREF_EN: 1'b1) :
             rxen_sel == 2'b01 ? 1'b1: 1'b0;

wire pndr_en = (ndr > 0 & pdr > 0);

wire tx_en   = (~TED) && pndr_en && (ODT === 1'b1 ? CFG_VREF_EN : 1'b1);

assign un_define = (&CFG_KEEP) & (&ndr) & (&pdr) & (&tpd) & (&tpu);

always @(*) begin
    if(un_define == 1'bx ) //unconnected input
        PAD_reg  =  1'bz;
    else if(CFG_KEEP == 2'b01) //pull down
        PAD_reg  =  1'b0;
    else if(CFG_KEEP == 2'b10) //pull up
        PAD_reg  =  1'b1;
    else if(CFG_KEEP == 2'b11) //keep
        PAD_reg  =  PAD_reg;
    else if(tx_en)   //normal output
        PAD_reg  =  TXD;
    else
        PAD_reg  =  1'bz;
end

assign PAD  = PAD_reg;
reg   rxd_in;
always @(*) begin
    if(un_define == 1'bx) //unconnected input
        rxd_in  =  1'bz;
    else if(rx_en)   //normal input
        rxd_in =  PAD;
    else 
        rxd_in  =  1'b0;
end
assign RXD = rxd_in;

endmodule

module P2_IOC_GPIO_HP (oen_out, q, rxd_dr, txd_out, odt_out,
     d, fclk_il, fclk_ol, cken_il, cken_ol, fp_odt_ctrl,
    oen, rst_il, rst_ol, rxd_in, setn_il, setn_ol );

output oen_out, rxd_dr, txd_out, odt_out;
output [1:0]  q;

input [1:0]  d;
input oen, rxd_in;
input fp_odt_ctrl;
input fclk_il, fclk_ol, cken_il, cken_ol, rst_il, rst_ol, setn_il, setn_ol;

parameter CFG_OEN_SEL = 4'b0000;
parameter CFG_CK_PAD_EN = 1'b0;
parameter CFG_DDR_IN_NREG = 1'b0;
parameter CFG_DDR_IN_NREG_DFF = 1'b0;
parameter CFG_DDR_IN_PREG = 1'b0;
parameter CFG_DDR_IN_PREG_DFF = 1'b0;
parameter CFG_DDR_OUT = 1'b0;
parameter CFG_DDR_OUT_REG = 1'b0;
parameter CFG_FASTIN_0 = 1'b0;
parameter CFG_FASTIN_1 = 1'b0;
parameter CFG_FCLK_INV = 1'b0;
parameter CFG_FOUT_SEL = 1'b0;
parameter CFG_OEN_DDR_OUT_REG = 1'b0;
parameter CFG_OEN_INV = 1'b0;
parameter CFG_OFDBK = 1'b0;
parameter CFG_OUT_INV = 1'b0;

//**** from IOC_LBUF_GPIO.ckgate_block_GPIO
parameter CFG_FCLK0_I_EN = 1'b0;
parameter CFG_FCLK0_O_EN = 1'b0;
parameter CFG_FCLK1_I_EN = 1'b0;
parameter CFG_FCLK1_O_EN = 1'b0;
parameter CFG_FCLK_OEN_EN = 1'b0;
parameter CFG_FCLK_SR_I_EN = 1'b0;
parameter CFG_FCLK_SR_O_EN = 1'b0;

//**** from IOC_LBUF_GPIO.CLK_MUX_GPIO
parameter CFG_SET_INV_I = 1'b0;
parameter CFG_SET_EN_I = 2'b00; //[1] not use
parameter CFG_SET_SYN_I = 1'b0;
parameter CFG_SET_INV_O = 1'b0;
parameter CFG_SET_EN_O = 2'b00; //[1] to control oe
parameter CFG_SET_SYN_O = 1'b0;

parameter CFG_RST_INV_I = 1'b0;
parameter CFG_RST_EN_I = 2'b00; //[1] not use
parameter CFG_RST_SYN_I = 1'b0;
parameter CFG_RST_INV_O = 1'b0;
parameter CFG_RST_EN_O = 2'b00; //[1] to control oe
parameter CFG_RST_SYN_O = 1'b0;

parameter CFG_EN0_I = 1'b0;
parameter CFG_EN1_I = 1'b0;
parameter CFG_CKEN_INV_I = 1'b0;
parameter CFG_EN0_O = 1'b0;
parameter CFG_EN1_O = 1'b0;
parameter CFG_CKEN_INV_O = 1'b0;

//0618 new circuit 
parameter CFG_FCLK0_ODT_EN = 1'b0;
parameter CFG_ODT_PHASE = 4'b0;
parameter CFG_ODT_EN = 1'b0;

/* clock gatings */
wire fclk0_il, fclk0_ol, fclk1_il, fclk1_ol, fclk_oen;
wire fclk_sr_il, fclk_sr_ol, rst_oen, setn_oen;

wire cken_il_i, cken_ol_i, cken_il_g, cken_ol_g;
assign cken_il_i = CFG_CKEN_INV_I == 1'b1 ? cken_il : ~cken_il;
assign cken_ol_i = CFG_CKEN_INV_O == 1'b1 ? cken_ol : ~cken_ol;
assign cken_il_g = ((~(CFG_EN0_I & cken_il_i)) | CFG_EN1_I );
assign cken_ol_g = ((~(CFG_EN0_O & cken_ol_i)) | CFG_EN1_O );

reg cken_il_lat, cken_ol_lat;
always @ (cken_il_g or fclk_il)
    cken_il_lat <= cken_il_g;
always @ (cken_ol_g or fclk_ol)
    cken_ol_lat <= cken_ol_g;

wire fclk_il_g, fclk_ol_g;
assign fclk_il_g = fclk_il & cken_il_lat;
assign fclk_ol_g = fclk_ol & cken_ol_lat;

assign fclk0_il   = fclk_il_g & CFG_FCLK0_I_EN;
assign fclk1_il   = fclk_il_g & CFG_FCLK1_I_EN;
assign fclk0_ol   = fclk_ol_g & CFG_FCLK0_O_EN;
assign fclk1_ol   = fclk_ol_g & CFG_FCLK1_O_EN;
assign fclk_oen   = fclk_ol_g & CFG_FCLK_OEN_EN;
assign fclk_sr_il = fclk_il_g & CFG_FCLK_SR_I_EN;
assign fclk_sr_ol = fclk_ol_g & CFG_FCLK_SR_O_EN;
wire w_fclk_odt  = CFG_FCLK0_ODT_EN & fclk_il_g;

/* set/reset controls */
wire rst_il_i, rst_ol_i, setn_il_i, setn_ol_i;
wire rst_il_s, rst_ol_s, setn_il_s, setn_ol_s;
reg rst_il_r, rst_ol_r, setn_il_r, setn_ol_r;
wire rst_il_g, rst_ol_g, rst_oen_g;
wire setn_il_g, setn_ol_g, setn_oen_g;

assign rst_il_i  = CFG_RST_INV_I == 1'b1 ? ~rst_il  : rst_il;
assign rst_ol_i  = CFG_RST_INV_O == 1'b1 ? ~rst_ol  : rst_ol;
assign setn_il_i = CFG_SET_INV_I == 1'b1 ? ~setn_il : setn_il;
assign setn_ol_i = CFG_SET_INV_O == 1'b1 ? ~setn_ol : setn_ol;

always @ (posedge fclk_sr_il) begin
    rst_il_r <= rst_il_i;
    setn_il_r <= setn_il_i;
end
always @ (posedge fclk_sr_ol) begin
    rst_ol_r <= rst_ol_i;
    setn_ol_r <= setn_ol_i;
end

assign rst_il_s  = CFG_RST_SYN_I == 1'b1 ? rst_il_r  : rst_il_i;
assign rst_ol_s  = CFG_RST_SYN_O == 1'b1 ? rst_ol_r  : rst_ol_i;
assign setn_il_s = CFG_SET_SYN_I == 1'b1 ? setn_il_r : setn_il_i;
assign setn_ol_s = CFG_SET_SYN_O == 1'b1 ? setn_ol_r : setn_ol_i;

assign rst_il_g   = CFG_RST_EN_I[0] & rst_il_s;
assign rst_ol_g   = CFG_RST_EN_O[0] & rst_ol_s;
assign rst_oen_g  = CFG_RST_EN_O[1] & rst_ol_s;
assign setn_il_g  = CFG_SET_EN_I[0] & setn_il_s;
assign setn_ol_g  = CFG_SET_EN_O[0] & setn_ol_s;
assign setn_oen_g = CFG_SET_EN_O[1] & setn_ol_s;

/* input/output logics */
wire txd_ol, rxd_mux;

assign rxd_mux = CFG_OFDBK == 1'b1 ? txd_ol : rxd_in;

odt_block Iodt ( .odt(odt_out),
     .CFG_ODT_EN(CFG_ODT_EN), .CFG_ODT_PHASE(CFG_ODT_PHASE), 
     .fclk(w_fclk_odt), .phy_odt_ctrl(fp_odt_ctrl));

IPATH Iilg ( .dataout(q),
     .CFG_DDR_IN_NREG(CFG_DDR_IN_NREG),
     .CFG_DDR_IN_NREG_DFF(CFG_DDR_IN_NREG_DFF),
     .CFG_DDR_IN_PREG(CFG_DDR_IN_PREG),
     .CFG_DDR_IN_PREG_DFF(CFG_DDR_IN_PREG_DFF),
     .CFG_FASTIN_0(CFG_FASTIN_1), .CFG_FASTIN_1(CFG_FASTIN_0),
     .datain(rxd_mux),
     .fclk0(fclk0_il), .fclk1(fclk1_il),
     .rst(rst_il_g), .set_(~setn_il_g));

OPATH Iolg ( .dataout(txd_ol),
     .CFG_DDR_OUT(CFG_DDR_OUT), .CFG_DDR_OUT_REG(CFG_DDR_OUT_REG),
     .CFG_FCLK_INV(CFG_FCLK_INV), .CFG_FOUT_SEL(CFG_FOUT_SEL),
     .datain(d),
     .fclk0(fclk0_ol), .fclk1(fclk1_ol),
     .rst(rst_ol_g), .set_(~setn_ol_g));

oen_path Ioen ( .f_oen(oen_out),
     .CFG_DDR_OUT_REG(CFG_OEN_DDR_OUT_REG),
     .CFG_OEN_INV(CFG_OEN_INV), .CFG_OEN_SEL(CFG_OEN_SEL),
     .fclk(fclk_oen), .oen(oen), .rst(rst_oen_g),
     .set_(~setn_oen_g));

assign rxd_dr = CFG_CK_PAD_EN & rxd_in;
assign txd_out = CFG_OUT_INV == 1'b1 ? ~txd_ol : txd_ol;

endmodule

module P2_IOC_GPIO (oen_out, q, rxd_dr, txd_out, 
     d, fclk_il, fclk_ol, cken_il, cken_ol, 
    oen, rst_il, rst_ol, rxd_in, setn_il, setn_ol );

output oen_out, rxd_dr, txd_out;
output [1:0]  q;

input [1:0]  d;
input oen, rxd_in;
input fclk_il, fclk_ol, cken_il, cken_ol, rst_il, rst_ol, setn_il, setn_ol;

parameter CFG_OEN_SEL = 4'b0000;
parameter CFG_CK_PAD_EN = 1'b0;
parameter CFG_DDR_IN_NREG = 1'b0;
parameter CFG_DDR_IN_NREG_DFF = 1'b0;
parameter CFG_DDR_IN_PREG = 1'b0;
parameter CFG_DDR_IN_PREG_DFF = 1'b0;
parameter CFG_DDR_OUT = 1'b0;
parameter CFG_DDR_OUT_REG = 1'b0;
parameter CFG_FASTIN_0 = 1'b0;
parameter CFG_FASTIN_1 = 1'b0;
parameter CFG_FCLK_INV = 1'b0;
parameter CFG_FOUT_SEL = 1'b0;
parameter CFG_OEN_DDR_OUT_REG = 1'b0;
parameter CFG_OEN_INV = 1'b0;
parameter CFG_OFDBK = 1'b0;
parameter CFG_OUT_INV = 1'b0;

//**** from IOC_LBUF_GPIO.ckgate_block_GPIO
parameter CFG_FCLK0_I_EN = 1'b0;
parameter CFG_FCLK0_O_EN = 1'b0;
parameter CFG_FCLK1_I_EN = 1'b0;
parameter CFG_FCLK1_O_EN = 1'b0;
parameter CFG_FCLK_OEN_EN = 1'b0;
parameter CFG_FCLK_SR_I_EN = 1'b0;
parameter CFG_FCLK_SR_O_EN = 1'b0;

//**** from IOC_LBUF_GPIO.CLK_MUX_GPIO
parameter CFG_SET_INV_I = 1'b0;
parameter CFG_SET_EN_I = 2'b00; //[1] not use
parameter CFG_SET_SYN_I = 1'b0;
parameter CFG_SET_INV_O = 1'b0;
parameter CFG_SET_EN_O = 2'b00; //[1] to control oe
parameter CFG_SET_SYN_O = 1'b0;

parameter CFG_RST_INV_I = 1'b0;
parameter CFG_RST_EN_I = 2'b00; //[1] not use
parameter CFG_RST_SYN_I = 1'b0;
parameter CFG_RST_INV_O = 1'b0;
parameter CFG_RST_EN_O = 2'b00; //[1] to control oe
parameter CFG_RST_SYN_O = 1'b0;

parameter CFG_EN0_I = 1'b0;
parameter CFG_EN1_I = 1'b0;
parameter CFG_CKEN_INV_I = 1'b0;
parameter CFG_EN0_O = 1'b0;
parameter CFG_EN1_O = 1'b0;
parameter CFG_CKEN_INV_O = 1'b0;

/* clock gatings */
wire fclk0_il, fclk0_ol, fclk1_il, fclk1_ol, fclk_oen;
wire fclk_sr_il, fclk_sr_ol, rst_oen, setn_oen;

wire cken_il_i, cken_ol_i, cken_il_g, cken_ol_g;
assign cken_il_i = CFG_CKEN_INV_I == 1'b1 ? cken_il : ~cken_il;
assign cken_ol_i = CFG_CKEN_INV_O == 1'b1 ? cken_ol : ~cken_ol;
assign cken_il_g = ((~(CFG_EN0_I & cken_il_i)) | CFG_EN1_I );
assign cken_ol_g = ((~(CFG_EN0_O & cken_ol_i)) | CFG_EN1_O );

reg cken_il_lat, cken_ol_lat;
always @ (cken_il_g or fclk_il)
    cken_il_lat <= cken_il_g;
always @ (cken_ol_g or fclk_ol)
    cken_ol_lat <= cken_ol_g;

wire fclk_il_g, fclk_ol_g;
assign fclk_il_g = fclk_il & cken_il_lat;
assign fclk_ol_g = fclk_ol & cken_ol_lat;

assign fclk0_il   = fclk_il_g & CFG_FCLK0_I_EN;
assign fclk1_il   = fclk_il_g & CFG_FCLK1_I_EN;
assign fclk0_ol   = fclk_ol_g & CFG_FCLK0_O_EN;
assign fclk1_ol   = fclk_ol_g & CFG_FCLK1_O_EN;
assign fclk_oen   = fclk_ol_g & CFG_FCLK_OEN_EN;
assign fclk_sr_il = fclk_il_g & CFG_FCLK_SR_I_EN;
assign fclk_sr_ol = fclk_ol_g & CFG_FCLK_SR_O_EN;

/* set/reset controls */
wire rst_il_i, rst_ol_i, setn_il_i, setn_ol_i;
wire rst_il_s, rst_ol_s, setn_il_s, setn_ol_s;
reg rst_il_r, rst_ol_r, setn_il_r, setn_ol_r;
wire rst_il_g, rst_ol_g, rst_oen_g;
wire setn_il_g, setn_ol_g, setn_oen_g;

assign rst_il_i  = CFG_RST_INV_I == 1'b1 ? ~rst_il  : rst_il;
assign rst_ol_i  = CFG_RST_INV_O == 1'b1 ? ~rst_ol  : rst_ol;
assign setn_il_i = CFG_SET_INV_I == 1'b1 ? ~setn_il : setn_il;
assign setn_ol_i = CFG_SET_INV_O == 1'b1 ? ~setn_ol : setn_ol;

always @ (posedge fclk_sr_il) begin
    rst_il_r <= rst_il_i;
    setn_il_r <= setn_il_i;
end
always @ (posedge fclk_sr_ol) begin
    rst_ol_r <= rst_ol_i;
    setn_ol_r <= setn_ol_i;
end

assign rst_il_s  = CFG_RST_SYN_I == 1'b1 ? rst_il_r  : rst_il_i;
assign rst_ol_s  = CFG_RST_SYN_O == 1'b1 ? rst_ol_r  : rst_ol_i;
assign setn_il_s = CFG_SET_SYN_I == 1'b1 ? setn_il_r : setn_il_i;
assign setn_ol_s = CFG_SET_SYN_O == 1'b1 ? setn_ol_r : setn_ol_i;

assign rst_il_g   = CFG_RST_EN_I[0] & rst_il_s;
assign rst_ol_g   = CFG_RST_EN_O[0] & rst_ol_s;
assign rst_oen_g  = CFG_RST_EN_O[1] & rst_ol_s;
assign setn_il_g  = CFG_SET_EN_I[0] & setn_il_s;
assign setn_ol_g  = CFG_SET_EN_O[0] & setn_ol_s;
assign setn_oen_g = CFG_SET_EN_O[1] & setn_ol_s;

/* input/output logics */
wire txd_ol, rxd_mux;

assign rxd_mux = CFG_OFDBK == 1'b1 ? txd_ol : rxd_in;

IPATH Iilg ( .dataout(q),
     .CFG_DDR_IN_NREG(CFG_DDR_IN_NREG),
     .CFG_DDR_IN_NREG_DFF(CFG_DDR_IN_NREG_DFF),
     .CFG_DDR_IN_PREG(CFG_DDR_IN_PREG),
     .CFG_DDR_IN_PREG_DFF(CFG_DDR_IN_PREG_DFF),
     .CFG_FASTIN_0(CFG_FASTIN_1), .CFG_FASTIN_1(CFG_FASTIN_0),
     .datain(rxd_mux),
     .fclk0(fclk0_il), .fclk1(fclk1_il),
     .rst(rst_il_g), .set_(~setn_il_g));

OPATH Iolg ( .dataout(txd_ol),
     .CFG_DDR_OUT(CFG_DDR_OUT), .CFG_DDR_OUT_REG(CFG_DDR_OUT_REG),
     .CFG_FCLK_INV(CFG_FCLK_INV), .CFG_FOUT_SEL(CFG_FOUT_SEL),
     .datain(d),
     .fclk0(fclk0_ol), .fclk1(fclk1_ol),
     .rst(rst_ol_g), .set_(~setn_ol_g));

oen_path Ioen ( .f_oen(oen_out),
     .CFG_DDR_OUT_REG(CFG_OEN_DDR_OUT_REG),
     .CFG_OEN_INV(CFG_OEN_INV), .CFG_OEN_SEL(CFG_OEN_SEL),
     .fclk(fclk_oen), .oen(oen), .rst(rst_oen_g),
     .set_(~setn_oen_g));

assign rxd_dr = CFG_CK_PAD_EN & rxd_in;
assign txd_out = CFG_OUT_INV == 1'b1 ? ~txd_ol : txd_ol;

endmodule

// ref: /home_hw/hme_center/hw/p2_center/circuit/verilog/array_block_cut_long/IOC.v

module P2_IOC_GPIO_elvds (
    /** connect to/from fp **/
    fclk_i, fclk_o, gsclk_o,
    rst_i, rst_o, setn_i, setn_o,
    cken_i, cken_o, oen, 
    d, q, rxd_dr,

    /** internal connection **/
    //pads
    oen_out, txd_out, rxd_in, txd_inv, txd_out_inv,
    //update
    update, update_,
    //cascade
    shiftin0, shiftin1,
    shiftout0, shiftout1
    );
//
input fclk_i, fclk_o, gsclk_o;
input rst_i, rst_o, setn_i, setn_o, cken_i, cken_o;
input [7:0]  d;
input oen;
output [7:0]  q;
output rxd_dr;

//////////////////////////////////////
input rxd_in, txd_inv;
output oen_out, txd_out, txd_out_inv;

//////////////////////////////////////
input shiftin0, shiftin1;
output shiftout0, shiftout1;

//////////////////////////////////////
input update, update_;

/* ** outside clock gating logic ** */
//en
parameter CFG_CKEN_EN0_I = 1'b0;
parameter CFG_CKEN_EN1_I = 1'b0;
parameter CFG_CKEN_INV_I = 1'b0;
parameter CFG_CKEN_EN0_O = 1'b0;
parameter CFG_CKEN_EN1_O = 1'b0;
parameter CFG_CKEN_INV_O = 1'b0;
wire cken_il_i, cken_ol_i, cken_il_g, cken_ol_g;
assign cken_il_i = CFG_CKEN_INV_I == 1'b1 ? ~cken_i : cken_i;
assign cken_ol_i = CFG_CKEN_INV_O == 1'b1 ? ~cken_o : cken_o;
assign cken_il_g = ( (~(CFG_CKEN_EN0_I & cken_il_i)) | CFG_CKEN_EN1_I );
assign cken_ol_g = ( (~(CFG_CKEN_EN0_O & cken_ol_i)) | CFG_CKEN_EN1_O );
reg cken_il_lat, cken_ol_lat;
always @ (cken_il_g or fclk_i)
    cken_il_lat <= cken_il_g;
always @ (cken_ol_g or fclk_o)
    cken_ol_lat <= cken_ol_g;

wire fclk_il_g  = fclk_i  & cken_il_lat;
wire gsclk_ol_g = gsclk_o & cken_ol_lat;
wire fclk_ol_g  = fclk_o  & cken_ol_lat;

//move ../lbuf.clkgate_block logic to here
parameter CFG_GSCLK0_O_EN = 1'b0;
parameter CFG_GSCLK1_O_EN = 1'b0;
wire w_gsclk0_ol = CFG_GSCLK0_O_EN & gsclk_ol_g;
wire w_gsclk1_ol = CFG_GSCLK1_O_EN & gsclk_ol_g;

parameter CFG_FCLK0_I_EN = 1'b0;
wire w_fclk0_il  = CFG_FCLK0_I_EN    & fclk_il_g;

parameter CFG_FCLK1_I_EN = 1'b0;
parameter CFG_FCLK_SR_I_EN = 1'b0; //control i_rst
wire w_fclk1_il  = CFG_FCLK1_I_EN  & fclk_il_g;
wire w_sr_clk_il = CFG_FCLK_SR_I_EN   & fclk_il_g;

parameter CFG_FCLK0_O_EN = 1'b0;
parameter CFG_GECLK0_O_EN = 1'b0;
parameter CFG_FCLK_SR_O_EN = 1'b0; //control o_rst
wire w_fclk0_ol  = CFG_FCLK0_O_EN  & fclk_ol_g;
wire w_geclk0_ol = CFG_GECLK0_O_EN & fclk_ol_g;
wire w_sr_clk_ol = CFG_FCLK_SR_O_EN   & fclk_ol_g;

parameter CFG_FCLK1_O_EN = 1'b0;
parameter CFG_GECLK1_O_EN = 1'b0;
parameter CFG_FCLK_OEN_EN = 1'b0;
wire w_fclk1_ol  = CFG_FCLK1_O_EN   & fclk_ol_g;
wire w_geclk1_ol = CFG_GECLK1_O_EN  & fclk_ol_g;
wire w_fclk_oen  = CFG_FCLK_OEN_EN  & fclk_ol_g;

/* set/reset controls */
//move ../lbuf.CLK_MUX_V2 logic to here
parameter CFG_SET_INV_I = 1'b0;
//parameter CFG_SET_EN_I = 3'b000; [0]:setn_il
parameter CFG_SET_EN_I = 2'b00;

parameter CFG_SET_SYN_I = 1'b0;
parameter CFG_SET_INV_O = 1'b0;
//parameter CFG_SET_EN_O = 3'b000; [0]:setn_ol, [1]:setn_oen
parameter CFG_SET_EN_O = 2'b00;

parameter CFG_SET_SYN_O = 1'b0;
parameter CFG_RST_INV_I = 1'b0;
//parameter CFG_RST_EN_I = 3'b000; [0]:rst_il
parameter CFG_RST_EN_I = 2'b00;
parameter CFG_RST_SYN_I = 1'b0;
parameter CFG_RST_INV_O = 1'b0;
//parameter CFG_RST_EN_O = 3'b000; [0]:rst_ol, [1]:rst_oen
parameter CFG_RST_EN_O = 2'b00;
parameter CFG_RST_SYN_O = 1'b0;

wire rst_il_i, rst_ol_i, setn_il_i, setn_ol_i;
wire rst_il_s, rst_ol_s, setn_il_s, setn_ol_s;
reg rst_il_r, rst_ol_r, setn_il_r, setn_ol_r;
wire rst_il_g, rst_ol_g;
wire setn_il_g, setn_ol_g;

assign rst_il_i  = CFG_RST_INV_I == 1'b1 ? ~rst_i  : rst_i;
assign rst_ol_i  = CFG_RST_INV_O == 1'b1 ? ~rst_o  : rst_o;
assign setn_il_i = CFG_SET_INV_I == 1'b1 ? ~setn_i : setn_i;
assign setn_ol_i = CFG_SET_INV_O == 1'b1 ? ~setn_o : setn_o;

always @ (posedge w_sr_clk_il) begin
    rst_il_r <= rst_il_i;
    setn_il_r <= setn_il_i;
end
always @ (posedge w_sr_clk_ol) begin
    rst_ol_r <= rst_ol_i;
    setn_ol_r <= setn_ol_i;
end

assign rst_il_s  = CFG_RST_SYN_I == 1'b1 ? rst_il_r  : rst_il_i;
assign rst_ol_s  = CFG_RST_SYN_O == 1'b1 ? rst_ol_r  : rst_ol_i;
assign setn_il_s = CFG_SET_SYN_I == 1'b1 ? setn_il_r : setn_il_i;
assign setn_ol_s = CFG_SET_SYN_O == 1'b1 ? setn_ol_r : setn_ol_i;

assign rst_il_g   = CFG_RST_EN_I   & rst_il_s;
assign rst_ol_g   = CFG_RST_EN_O[0]   & rst_ol_s;
assign rst_oen_g  = CFG_RST_EN_O[1] & rst_ol_s;
assign setn_il_g  = ~(CFG_SET_EN_I   & setn_il_s);
assign setn_ol_g  = ~(CFG_SET_EN_O[0]   & setn_ol_s);
assign setn_oen_g = ~(CFG_SET_EN_O[1] & setn_ol_s);


/* ** IOC paras ** */
parameter CFG_CK_PAD_EN = 1'b0;
parameter CFG_DDR_IN_NREG = 1'b0;
parameter CFG_DDR_IN_NREG_DFF = 1'b0;
parameter CFG_DDR_IN_PREG = 1'b0;
parameter CFG_DDR_IN_PREG_DFF = 1'b0;
parameter CFG_DDR_OUT = 1'b0;
parameter CFG_DDR_OUT_REG = 1'b0;
parameter CFG_FASTIN_0 = 1'b0;
parameter CFG_FASTIN_1 = 1'b0;
parameter CFG_FCLK_INV = 1'b0;
parameter CFG_GEAR_OUT = 1'b0;
parameter CFG_OEN_DDR_OUT_REG = 1'b0;
parameter CFG_OEN_INV = 1'b0;
parameter CFG_OFDBK = 1'b0;
parameter CFG_OUT_INV = 1'b0;

parameter CFG_GEAR_IN = 8'b00000000;
parameter CFG_OEN_SEL = 4'b0000;

parameter CFG_FOUT_SEL = 1'b0;

parameter CFG_LVDS_TXINV_SEL = 1'b0;
parameter CFG_IDLC_BYPASS = 1'b0;
parameter CFG_ODLC_BYPASS = 1'b0;
parameter CFG_IDLC_DEL_SEL = 5'b00000;
parameter CFG_ODLC_DEL_SEL = 5'b00000;

oen_path Ioen ( .f_oen(oen_out), 
                .CFG_DDR_OUT_REG(CFG_OEN_DDR_OUT_REG), 
                .CFG_OEN_INV(CFG_OEN_INV),
                .CFG_OEN_SEL(CFG_OEN_SEL),
                .fclk(w_fclk_oen),
                .oen(oen),
                .rst(rst_oen_g),
                .set_(setn_oen_g)
            );

wire txd_ol, rxd_mux;
OLOGIC Iolg ( .dataout(txd_ol), .shiftout0(shiftout0), .shiftout1(shiftout1),
     .CFG_DDR_OUT(CFG_DDR_OUT),
     .CFG_DDR_OUT_REG(CFG_DDR_OUT_REG),
     .CFG_FCLK_INV(CFG_FCLK_INV),
     .CFG_FOUT_SEL(CFG_FOUT_SEL),
     .CFG_GEAR_OUT(CFG_GEAR_OUT),
     .datain(d), .fclk0(w_fclk0_ol),
     .fclk1(w_fclk1_ol), .geclk0(w_geclk0_ol), .geclk1(w_geclk1_ol),
     .gsclk0(w_gsclk0_ol), .gsclk1(w_gsclk1_ol), .rst(rst_ol_g),
     .setn(setn_ol_g), .shiftin0(shiftin0), .shiftin1(shiftin1),
     .update(update));

wire rxd_dly, rxd;
delay_block Irx(.in(rxd_in), .sel(CFG_IDLC_DEL_SEL), .out(rxd_dly));
assign rxd = CFG_IDLC_BYPASS == 1'b1 ? rxd_in : rxd_dly;

assign rxd_mux = CFG_OFDBK == 1'b1 ? txd_ol : rxd;
IPATH Iilg( .dataout(q),
            .CFG_DDR_IN_NREG(CFG_DDR_IN_NREG),
            .CFG_DDR_IN_NREG_DFF(CFG_DDR_IN_NREG_DFF),
            .CFG_DDR_IN_PREG(CFG_DDR_IN_PREG),
            .CFG_DDR_IN_PREG_DFF(CFG_DDR_IN_PREG_DFF),
            .CFG_FASTIN_0(CFG_FASTIN_0),
            .CFG_FASTIN_1(CFG_FASTIN_1),
            .datain(rxd_mux),
            .fclk0(w_fclk0_il),
            .fclk1(w_fclk1_il),
            .rst(rst_il_g),
            .set_(setn_il_g)
        );

assign rxd_dr = CFG_CK_PAD_EN & rxd;

wire txd_dly, txd_dly_o, txd;
assign txd = CFG_OUT_INV == 1'b1 ? ~txd_ol : txd_ol;
delay_block Itx(.in(txd), .sel(CFG_ODLC_DEL_SEL), .out(txd_dly_o));
assign txd_dly = CFG_ODLC_BYPASS == 1'b1 ? txd : txd_dly_o;
assign txd_out = CFG_LVDS_TXINV_SEL == 1'b1 ? txd_inv : txd_dly;
assign txd_out_inv = ~txd_dly;

endmodule

// ref: /home_hw/hme_center/hw/p2_center/circuit/verilog/array_block_cut_long/IOC.v

module P2_IOC_LVDS (
    /** connect to/from fp **/
    fclk_il, fclk_ol, gsclk_il, gsclk_ol,
    rst_il, rst_ol, set_il, set_ol,
    cken_il, cken_ol, oen, 
    phy_odt_ctrl,
    fout_dyn_ctrl,
    d, q, rxd_dr,

    /** internal connection **/
    //pads
    odt_out, oen_out, txd_out, rxd_in,
    //update & pattern detect
    update_b_il, update_b_ol, update_il, update_ol, sout,
    //cascade
    shiftin0_il, shiftin0_ol, shiftin1_il, shiftin1_ol, shiftin_oen,
    shiftout0_il, shiftout0_ol, shiftout1_il, shiftout1_ol, shiftout_oen
    );
//
input fclk_il, fclk_ol, gsclk_il, gsclk_ol;
input rst_il, rst_ol, set_il, set_ol, cken_il, cken_ol;
input phy_odt_ctrl; 
input fout_dyn_ctrl;
input [7:0]  d;
input [3:0]  oen;
output [7:0]  q;
output rxd_dr;

//////////////////////////////////////
input rxd_in;
output odt_out, oen_out, txd_out;

//////////////////////////////////////
input shiftin0_il, shiftin0_ol, shiftin1_il, shiftin1_ol, shiftin_oen;
output shiftout0_il, shiftout0_ol, shiftout1_il, shiftout1_ol, shiftout_oen;

//////////////////////////////////////
input update_b_il, update_b_ol, update_il, update_ol;
output [9:0]  sout;

/* ** outside clock gating logic ** */
//en
parameter CFG_CKEN_EN0_I = 1'b0;
parameter CFG_CKEN_EN1_I = 1'b0;
parameter CFG_CKEN_INV_I = 1'b0;
parameter CFG_CKEN_EN0_O = 1'b0;
parameter CFG_CKEN_EN1_O = 1'b0;
parameter CFG_CKEN_INV_O = 1'b0;
wire cken_il_i, cken_ol_i, cken_il_g, cken_ol_g;
assign cken_il_i = CFG_CKEN_INV_I == 1'b1 ? ~cken_il : cken_il;
assign cken_ol_i = CFG_CKEN_INV_O == 1'b1 ? ~cken_ol : cken_ol;
assign cken_il_g = ( (~(CFG_CKEN_EN0_I & cken_il_i)) | CFG_CKEN_EN1_I );
assign cken_ol_g = ( (~(CFG_CKEN_EN0_O & cken_ol_i)) | CFG_CKEN_EN1_O );
reg cken_il_lat, cken_ol_lat;
always @ (cken_il_g or fclk_il)
    cken_il_lat <= cken_il_g;
always @ (cken_ol_g or fclk_ol)
    cken_ol_lat <= cken_ol_g;

wire gsclk_il_g = gsclk_il & cken_il_lat;
wire fclk_il_g  = fclk_il  & cken_il_lat;
wire gsclk_ol_g = gsclk_ol & cken_ol_lat;
wire fclk_ol_g  = fclk_ol  & cken_ol_lat;

//move ../lbuf.clkgate_block logic to here
parameter CFG_GSCLK0_I_EN = 1'b0;
parameter CFG_GSCLK1_I_EN = 1'b0;
//parameter CFG_GSCLK_IDLC_EN = 1'b0; //no use
//parameter CFG_GSCLK_UPI_EN = 1'b0; //to update_block_cnt_det.gsclk
wire w_gsclk0_il = CFG_GSCLK0_I_EN & gsclk_il_g;
wire w_gsclk1_il = CFG_GSCLK1_I_EN & gsclk_il_g;

parameter CFG_GSCLK0_O_EN = 1'b0;
parameter CFG_GSCLK1_O_EN = 1'b0;
//parameter CFG_GSCLK_ODLC_EN = 1'b0; //no use
parameter CFG_GSCLK_OEN_EN = 1'b0;
//parameter CFG_GSCLK_UPO_EN = 1'b0; //to update_block_cnt.gsclk
wire w_gsclk0_ol = CFG_GSCLK0_O_EN & gsclk_ol_g;
wire w_gsclk1_ol = CFG_GSCLK1_O_EN & gsclk_ol_g;
wire w_gsclk_oen = CFG_GSCLK_OEN_EN & gsclk_ol_g;

parameter CFG_FCLK0_I_EN = 1'b0;
parameter CFG_GECLK0_I_EN = 1'b0;
//parameter CFG_GECLK_UPI_EN = 1'b0; //to update_block_cnt_det.geclk
parameter CFG_FCLK_ODT_I_EN = 1'b0;
wire w_fclk0_il  = CFG_FCLK0_I_EN    & fclk_il_g;
wire w_geclk0_il = CFG_GECLK0_I_EN   & fclk_il_g;
wire w_fclk_odt  = CFG_FCLK_ODT_I_EN & fclk_il_g;

parameter CFG_FCLK1_I_EN = 1'b0;
parameter CFG_GECLK1_I_EN = 1'b0;
parameter CFG_FCLK_SR_I = 1'b0; //control i_rst
//parameter CFG_FCLK_DET = 1'b0; //to key_detect_top.detect_clk
wire w_fclk1_il  = CFG_FCLK1_I_EN  & fclk_il_g;
wire w_geclk1_il = CFG_GECLK1_I_EN & fclk_il_g;
wire w_sr_clk_il = CFG_FCLK_SR_I   & fclk_il_g;

parameter CFG_FCLK0_O_EN = 1'b0;
parameter CFG_GECLK0_O_EN = 1'b0;
//parameter CFG_GECLK_UPO_EN = 1'b0; //to update_block_cnt.geclk
parameter CFG_FCLK_SR_O = 1'b0; //control o_rst
wire w_fclk0_ol  = CFG_FCLK0_O_EN  & fclk_ol_g;
wire w_geclk0_ol = CFG_GECLK0_O_EN & fclk_ol_g;
wire w_sr_clk_ol = CFG_FCLK_SR_O   & fclk_ol_g;

parameter CFG_FCLK1_O_EN = 1'b0;
parameter CFG_GECLK1_O_EN = 1'b0;
parameter CFG_FCLK_OEN_EN = 1'b0;
parameter CFG_GECLK_OEN_EN = 1'b0;
wire w_fclk1_ol  = CFG_FCLK1_O_EN   & fclk_ol_g;
wire w_geclk1_ol = CFG_GECLK1_O_EN  & fclk_ol_g;
wire w_fclk_oen  = CFG_FCLK_OEN_EN  & fclk_ol_g;
wire w_geclk_oen = CFG_GECLK_OEN_EN & fclk_ol_g;

/* set/reset controls */
//move ../lbuf.CLK_MUX_V2 logic to here
parameter CFG_SET_INV_I = 1'b0;
//parameter CFG_SET_EN_I = 3'b000; [0]:setn_il
parameter CFG_SET_EN_I = 1'b0;

parameter CFG_SET_SYN_I = 1'b0;
parameter CFG_SET_INV_O = 1'b0;
//parameter CFG_SET_EN_O = 3'b000; [0]:setn_ol, [1]:setn_oen
parameter CFG_SET_EN_O = 1'b0;
parameter CFG_SET_EN_OEN = 1'b0;

parameter CFG_SET_SYN_O = 1'b0;
parameter CFG_RST_INV_I = 1'b0;
//parameter CFG_RST_EN_I = 3'b000; [0]:rst_il
parameter CFG_RST_EN_I = 1'b0;
parameter CFG_RST_SYN_I = 1'b0;
parameter CFG_RST_INV_O = 1'b0;
//parameter CFG_RST_EN_O = 3'b000; [0]:rst_ol, [1]:rst_oen
parameter CFG_RST_EN_O = 1'b0;
parameter CFG_RST_EN_OEN = 1'b0;
parameter CFG_RST_SYN_O = 1'b0;

wire rst_il_i, rst_ol_i, setn_il_i, setn_ol_i;
wire rst_il_s, rst_ol_s, setn_il_s, setn_ol_s;
reg rst_il_r, rst_ol_r, setn_il_r, setn_ol_r;
wire rst_il_g, rst_ol_g, rst_oen_g;
wire setn_il_g, setn_ol_g, setn_oen_g;

assign rst_il_i  = CFG_RST_INV_I == 1'b1 ? ~rst_il  : rst_il;
assign rst_ol_i  = CFG_RST_INV_O == 1'b1 ? ~rst_ol  : rst_ol;
assign setn_il_i = CFG_SET_INV_I == 1'b1 ? ~set_il : set_il;
assign setn_ol_i = CFG_SET_INV_O == 1'b1 ? ~set_ol : set_ol;

always @ (posedge w_sr_clk_il) begin
    rst_il_r <= rst_il_i;
    setn_il_r <= setn_il_i;
end
always @ (posedge w_sr_clk_ol) begin
    rst_ol_r <= rst_ol_i;
    setn_ol_r <= setn_ol_i;
end

assign rst_il_s  = CFG_RST_SYN_I == 1'b1 ? rst_il_r  : rst_il_i;
assign rst_ol_s  = CFG_RST_SYN_O == 1'b1 ? rst_ol_r  : rst_ol_i;
assign setn_il_s = CFG_SET_SYN_I == 1'b1 ? setn_il_r : setn_il_i;
assign setn_ol_s = CFG_SET_SYN_O == 1'b1 ? setn_ol_r : setn_ol_i;

assign rst_il_g   = CFG_RST_EN_I   & rst_il_s;
assign rst_ol_g   = CFG_RST_EN_O   & rst_ol_s;
assign rst_oen_g  = CFG_RST_EN_OEN & rst_ol_s;
assign setn_il_g  = ~(CFG_SET_EN_I   & setn_il_s);
assign setn_ol_g  = ~(CFG_SET_EN_O   & setn_ol_s);
assign setn_oen_g = ~(CFG_SET_EN_OEN & setn_ol_s);


/* ** IOC paras ** */
parameter CFG_CK_PAD_EN = 1'b0;
parameter CFG_DDR_IN_NREG = 1'b0;
parameter CFG_DDR_IN_NREG_DFF = 1'b0;
parameter CFG_DDR_IN_PREG = 1'b0;
parameter CFG_DDR_IN_PREG_DFF = 1'b0;
parameter CFG_DDR_OUT = 1'b0;
parameter CFG_DDR_OUT_REG = 1'b0;
parameter CFG_FASTIN = 1'b0;
parameter CFG_FCLK_INV = 1'b0;
parameter CFG_FCLK_OUT_EN = 1'b0;
parameter CFG_GEAR_OUT = 1'b0;
parameter CFG_ODT_EN = 1'b0;
parameter CFG_OEN_DDR_OUT_REG = 1'b0;
parameter CFG_OEN_GEAR_OUT = 1'b0;
parameter CFG_OEN_INV = 1'b0;
parameter CFG_OFDBK = 1'b0;
parameter CFG_OUT_INV = 1'b0;
parameter CFG_SLAVE_IN = 1'b0;

parameter CFG_GEAR_IN = 8'b00000000;
parameter CFG_OEN_SEL = 4'b0000;
parameter CFG_ODT_PHASE = 4'b0000;

parameter CFG_FOUT_SEL = 1'b0;
parameter CFG_DYN_FOUT = 1'b0;

wire fout_sel = CFG_DYN_FOUT == 1'b1 ? fout_dyn_ctrl : CFG_FOUT_SEL;

odt_block Iodt ( .odt(odt_out),
     .CFG_ODT_EN(CFG_ODT_EN), .CFG_ODT_PHASE(CFG_ODT_PHASE), 
     .fclk(w_fclk_odt), .phy_odt_ctrl(phy_odt_ctrl));

oen_block Ioen ( .f_oen(oen_out), .shiftout(shiftout_oen),
     .CFG_DDR_OUT_REG(CFG_OEN_DDR_OUT_REG),
     .CFG_GEAR_OUT(CFG_OEN_GEAR_OUT), .CFG_OEN_INV(CFG_OEN_INV),
     .CFG_OEN_SEL(CFG_OEN_SEL), 
     .fclk(w_fclk_oen), .geclk(w_geclk_oen), .gsclk(w_gsclk_oen), .oen(oen),
     .rst(rst_oen_g), .set_(setn_oen_g), .shiftin(shiftin_oen),
     .update(update_ol));

wire txd_ol, rxd_mux;
OLOGIC Iolg ( .dataout(txd_ol), .shiftout0(shiftout0_ol), .shiftout1(shiftout1_ol),
     .CFG_DDR_OUT(CFG_DDR_OUT),
     .CFG_DDR_OUT_REG(CFG_DDR_OUT_REG),
     .CFG_FCLK_INV(CFG_FCLK_INV),
     .CFG_FOUT_SEL(fout_sel),
     .CFG_GEAR_OUT(CFG_GEAR_OUT),
     .datain(d), .fclk0(w_fclk0_ol),
     .fclk1(w_fclk1_ol), .geclk0(w_geclk0_ol), .geclk1(w_geclk1_ol),
     .gsclk0(w_gsclk0_ol), .gsclk1(w_gsclk1_ol), .rst(rst_ol_g),
     .setn(setn_ol_g), .shiftin0(shiftin0_ol), .shiftin1(shiftin1_ol),
     .update(update_ol));

assign rxd_mux = CFG_OFDBK == 1'b1 ? txd_ol : rxd_in;
ILOGIC_detect Iilg ( .dataout(q), .sout(sout), .shiftout0(shiftout0_il), .shiftout1(shiftout1_il),
     .CFG_DDR_IN_NREG(CFG_DDR_IN_NREG),
     .CFG_DDR_IN_NREG_DFF(CFG_DDR_IN_NREG_DFF),
     .CFG_DDR_IN_PREG(CFG_DDR_IN_PREG),
     .CFG_DDR_IN_PREG_DFF(CFG_DDR_IN_PREG_DFF),
     .CFG_FASTIN(CFG_FASTIN),
     .CFG_GEAR_IN(CFG_GEAR_IN),
     .CFG_SLAVE_IN(CFG_SLAVE_IN),
     .datain(rxd_mux), .fclk0(w_fclk0_il), .fclk1(w_fclk1_il),
     .geclk0(w_geclk0_il), .geclk1(w_geclk1_il), .gsclk0(w_gsclk0_il),
     .gsclk1(w_gsclk1_il), .rst(rst_il_g), .setn(setn_il_g),
     .shiftin0(shiftin0_il), .shiftin1(shiftin1_il),
     .update(update_il));

assign rxd_dr = CFG_CK_PAD_EN & rxd_in;
assign txd_out = CFG_FCLK_OUT_EN == 1'b1 ? w_fclk0_ol :
                  CFG_OUT_INV == 1'b1 ? ~txd_ol : txd_ol;

endmodule

module P2_IOC_FUN (
                     f_in, in,  clk_i_en, fclk_i, rstn_i, setn_i,
                     f_oen, f_out, f_out_fb, oen, out,clk_o_en, fclk_o,
                     rstn_o, setn_o );

input          in;
output [1:0]   f_in;
input          clk_i_en;
input          fclk_i;
input          rstn_i;
input          setn_i;
input          oen;
output         f_oen;
input  [1:0]   out;
output         f_out;
output         f_out_fb;
input          clk_o_en;
input          fclk_o;
input          rstn_o;
input          setn_o;

parameter CFG_I_FIN_SEL = 1'b0;
parameter CFG_I_FCLK_GATE_EN = 1'b0;
parameter CFG_I_SETN_INV = 1'b0;
parameter CFG_I_SETN_SYNC = 1'b0;
parameter CFG_I_ID_SETN_EN = 1'b0;
parameter CFG_I_RSTN_INV = 1'b0;
parameter CFG_I_RSTN_SYNC = 1'b0;
parameter CFG_I_ID_RSTN_EN = 1'b0;
parameter CFG_O_NC = 1'b0;
parameter CFG_O_DDR = 1'b0;
parameter CFG_O_FOEN_SEL = 1'b0;
parameter CFG_O_FOUT_SEL = 1'b0;
parameter CFG_O_FCLK_INV = 1'b0;
parameter CFG_O_FCLK_GATE_EN = 1'b0;
parameter CFG_O_SETN_INV = 1'b0;
parameter CFG_O_SETN_SYNC = 1'b0;
parameter CFG_O_OEN_SETN_EN = 1'b0;
parameter CFG_O_OD_SETN_EN = 1'b0;
parameter CFG_O_RSTN_INV = 1'b0;
parameter CFG_O_RSTN_SYNC = 1'b0;
parameter CFG_O_OEN_RSTN_EN = 1'b0;
parameter CFG_O_OD_RSTN_EN = 1'b0;

parameter CFG_O_OUT_SEL = 2'b00;
parameter CFG_O_OEN_SEL = 2'b00;

P2_IOC_FUN_IN u_in (
    .f_in(f_in),
    .in(in),
//    .cf_rstn(),
    .clk_en(clk_i_en),
    .fclk(fclk_i),
    .rstn(rstn_i),
    .setn(setn_i),
    .CFG_FIN_SEL(CFG_I_FIN_SEL),
    .CFG_FCLK_GATE_EN(CFG_I_FCLK_GATE_EN),
    .CFG_SETN_INV(CFG_I_SETN_INV),
    .CFG_SETN_SYNC(CFG_I_SETN_SYNC),
    .CFG_ID_SETN_EN(CFG_I_ID_SETN_EN),
    .CFG_RSTN_INV(CFG_I_RSTN_INV),
    .CFG_RSTN_SYNC(CFG_I_RSTN_SYNC),
    .CFG_ID_RSTN_EN(CFG_I_ID_RSTN_EN)
    );

wire w_oen, w_out;
P2_IOC_FUN_OUT u_out(
    .f_oen(w_oen),
    .f_out(w_out),
    .f_out_fb(f_out_fb),
    // Inputs
    .oen(oen),
    .out(out),
//    .cf_rstn(),
    .clk_en(clk_o_en),
    .fclk(fclk_o),
    .rstn(rstn_o),
    .setn(setn_o),
    .CFG_NC(CFG_O_NC),
    .CFG_DDR(CFG_O_DDR),
    .CFG_FOEN_SEL(CFG_O_FOEN_SEL),
    .CFG_FOUT_SEL(CFG_O_FOUT_SEL),
    .CFG_FCLK_INV(CFG_O_FCLK_INV),
    .CFG_FCLK_GATE_EN(CFG_O_FCLK_GATE_EN),
    .CFG_SETN_INV(CFG_O_SETN_INV),
    .CFG_SETN_SYNC(CFG_O_SETN_SYNC),
    .CFG_OEN_SETN_EN(CFG_O_OEN_SETN_EN),
    .CFG_OD_SETN_EN(CFG_O_OD_SETN_EN),
    .CFG_RSTN_INV(CFG_O_RSTN_INV),
    .CFG_RSTN_SYNC(CFG_O_RSTN_SYNC),
    .CFG_OEN_RSTN_EN(CFG_O_OEN_RSTN_EN),
    .CFG_OD_RSTN_EN(CFG_O_OD_RSTN_EN)
    );


wire f_oen = CFG_O_OEN_SEL == 2'b00 ? 1'b1 :
             CFG_O_OEN_SEL == 2'b01 ? 1'b0 :
             CFG_O_OEN_SEL == 2'b10 ? w_oen  :
             CFG_O_OEN_SEL == 2'b11 ? ~w_oen : 1'bx;
wire f_out = CFG_O_OUT_SEL == 2'b00 ? 1'b1 :
             CFG_O_OUT_SEL == 2'b01 ? 1'b0 :
             CFG_O_OUT_SEL == 2'b10 ? w_out  :
             CFG_O_OUT_SEL == 2'b11 ? ~w_out : 1'bx;

endmodule
 

module P2_IOC_FUN_IN (/*AUTOARG*/
   // Outputs
   f_in,
   // Inputs
   in, clk_en, fclk, rstn, setn, CFG_FIN_SEL,
   CFG_FCLK_GATE_EN, CFG_SETN_INV, CFG_SETN_SYNC, CFG_ID_SETN_EN,
   CFG_RSTN_INV, CFG_RSTN_SYNC, CFG_ID_RSTN_EN
   );

input          in;
output [1:0]   f_in;

input          clk_en;
input          fclk;
input          rstn;
input          setn;

input          CFG_FIN_SEL;
input          CFG_FCLK_GATE_EN;
input          CFG_SETN_INV;
input          CFG_SETN_SYNC;
input          CFG_ID_SETN_EN;
input          CFG_RSTN_INV;
input          CFG_RSTN_SYNC;
input          CFG_ID_RSTN_EN;


reg rstn_reg = 0;
reg setn_reg = 0;
reg in_reg0 = 1;
reg in_reg1 = 1;

//==fclk gate===============================================================================
wire fclk_GT;

gclk_clk_gate fclk_gate ( .clk_gate_in(fclk), .gate_en(clk_en), .test_mode(1'b0), .clk_gate_out(fclk_GT));

wire fclk_gate_mux = CFG_FCLK_GATE_EN ? fclk : fclk_GT;    //0=gated


//==rstn/setn configuration===============================================================================
wire rstn_inv_cfg = CFG_RSTN_INV ? ~rstn : rstn;
wire setn_inv_cfg = CFG_SETN_INV ? ~setn : setn;

always @ (posedge fclk_gate_mux) begin
    rstn_reg <= rstn_inv_cfg;
    setn_reg <= setn_inv_cfg;
end

//ioc_dff_ar rstn_syn (
//     .ck    (fclk_gate_mux ),
//     .d     (rstn_inv_cfg  ),
//     .q     (rstn_reg      ),
//     .rstn  (cf_rstn       )
//);
//
//ioc_dff_ar setn_syn (
//     .ck    (fclk_gate_mux ),
//     .d     (setn_inv_cfg  ),
//     .q     (setn_reg      ),
//     .rstn  (cf_rstn       )
//);

wire   rstn_reg_mux = CFG_RSTN_SYNC   ? (rstn_inv_cfg || rstn_reg) : rstn_inv_cfg;
wire   setn_reg_mux = CFG_SETN_SYNC   ? (setn_inv_cfg || setn_reg) : setn_inv_cfg;

wire   id_rstn      = CFG_ID_RSTN_EN  ? rstn_reg_mux : 1'b1;
                    
wire   t_id_setn    = CFG_ID_SETN_EN  ? setn_reg_mux : 1'b1;
wire   id_setn      = t_id_setn;


//==iopad => array, input path===============================================================================

always @ (posedge fclk_gate_mux or negedge id_rstn or negedge id_setn) begin
    if(~id_rstn) begin
        in_reg0 <= 1'b0;
    end else if(~id_setn) begin
        in_reg0 <= 1'b1;
    end else begin
        in_reg0 <= in;
    end
end
        
always @ (negedge fclk_gate_mux or negedge id_rstn or negedge id_setn) begin
    if(~id_rstn) begin
        in_reg1 <= 1'b0;
    end else if(~id_setn) begin
        in_reg1 <= 1'b1;
    end else begin
        in_reg1 <= in;
    end
end

// f_in[1:0]
//ioc_dff_asr u_id0(                
//    .ck     ( fclk_gate_mux ),
//    .d      ( in            ),
//    .q      ( in_reg0       ),
//    .rstn   ( id_rstn       ),
//    .setn   ( id_setn       )
//);
//
//ioc_dffn_asr u_id1(
//    .ckb    ( fclk_gate_mux ),
//    .d      ( in            ),
//    .q      ( in_reg1       ),
//    .rstn   ( id_rstn       ),
//    .setn   ( id_setn       )
//);

assign f_in[0] = ~CFG_FIN_SEL ? in_reg0 : in;
assign f_in[1] = in_reg1;

endmodule


module P2_IOC_FUN_OUT(/*AUTOARG*/
   // Outputs
   f_oen, f_out, f_out_fb,
   // Inputs
   oen, out, cf_rstn, clk_en, fclk, rstn, setn, CFG_NC, CFG_DDR,
   CFG_FOEN_SEL, CFG_FOUT_SEL, CFG_FCLK_INV, CFG_FCLK_GATE_EN,
   CFG_SETN_INV, CFG_SETN_SYNC, CFG_OEN_SETN_EN, CFG_OD_SETN_EN,
   CFG_RSTN_INV, CFG_RSTN_SYNC, CFG_OEN_RSTN_EN, CFG_OD_RSTN_EN
   );

input          oen;
output         f_oen;

input  [1:0]   out;
output         f_out;
output         f_out_fb;

input          cf_rstn;
input          clk_en;
input          fclk;
input          rstn;
input          setn;

input          CFG_NC;
input          CFG_DDR;
input          CFG_FOEN_SEL;
input          CFG_FOUT_SEL;
input          CFG_FCLK_INV;
input          CFG_FCLK_GATE_EN;
input          CFG_SETN_INV;
input          CFG_SETN_SYNC;
input          CFG_OEN_SETN_EN;
input          CFG_OD_SETN_EN;
input          CFG_RSTN_INV;
input          CFG_RSTN_SYNC;
input          CFG_OEN_RSTN_EN;
input          CFG_OD_RSTN_EN;

reg rstn_reg = 0;
reg setn_reg = 0;
reg oen_reg = 1;
reg oen_regn = 1;
reg out_reg0 = 1;
reg out_reg1 = 1;


//==fclk gate===============================================================================
wire fclk_GT;

gclk_clk_gate fclk_gate ( .clk_gate_in(fclk), .gate_en(clk_en), .test_mode(1'b0), .clk_gate_out(fclk_GT));

wire fclk_gate_mux = CFG_FCLK_GATE_EN ? fclk : fclk_GT;    //0=gated


//==rstn/setn configuration===============================================================================

wire rstn_inv_cfg = CFG_RSTN_INV ? ~rstn : rstn;
wire setn_inv_cfg = CFG_SETN_INV ? ~setn : setn;

always @ (posedge fclk_gate_mux) begin
    rstn_reg <= rstn_inv_cfg;
    setn_reg <= setn_inv_cfg;
end

//ioc_dff_ar rstn_syn (
//     .ck    (fclk_gate_mux ),
//     .d     (rstn_inv_cfg  ),
//     .q     (rstn_reg      ),
//     .rstn  (cf_rstn       )
//);
//
//ioc_dff_ar setn_syn (
//     .ck    (fclk_gate_mux ),
//     .d     (setn_inv_cfg  ),
//     .q     (setn_reg      ),
//     .rstn  (cf_rstn       )
//);

wire   rstn_reg_mux = CFG_RSTN_SYNC   ? (rstn_inv_cfg || rstn_reg) : rstn_inv_cfg;
wire   setn_reg_mux = CFG_SETN_SYNC   ? (setn_inv_cfg || setn_reg) : setn_inv_cfg;

wire   oen_rstn     = CFG_OEN_RSTN_EN ? rstn_reg_mux : 1'b1;
wire   od_rstn      = CFG_OD_RSTN_EN  ? rstn_reg_mux : 1'b1;
                    
wire   t_oen_setn   = CFG_OEN_SETN_EN ? setn_reg_mux : 1'b1;
wire   t_od_setn    = CFG_OD_SETN_EN  ? setn_reg_mux : 1'b1;
                    
wire   oen_setn     = cf_rstn & t_oen_setn;
wire   od_setn      = cf_rstn & t_od_setn;


//==array => iopad, output path===============================================================================
 
//f_oen
always @ (posedge fclk_gate_mux or negedge oen_setn or negedge oen_rstn) begin
    if(~oen_rstn)
        oen_reg <= 1'b0;
    else if(~oen_setn)
        oen_reg <= 1'b1;
    else
        oen_reg <= oen;
end
always @ (negedge fclk_gate_mux or negedge oen_setn or negedge oen_rstn) begin
    if(~oen_rstn)
        oen_regn <= 1'b0;
    else if(~oen_setn)
        oen_regn <= 1'b1;
    else
        oen_regn <= oen;
end

//ioc_dff_asr u0_oen(
//    .ck     ( fclk_gate_mux     ),
//    .d      ( oen               ),
//    .q      ( oen_reg           ),
//    .rstn   ( oen_rstn          ),
//    .setn   ( oen_setn          )
//);
//
//ioc_dffn_asr u1_oen(
//    .ckb    ( fclk_gate_mux     ),
//    .d      ( oen               ),
//    .q      ( oen_regn          ),
//    .rstn   ( oen_rstn          ),
//    .setn   ( oen_setn          )
//);


wire   f_oen_sel =  CFG_NC       ? oen_regn : oen_reg;
wire   f_oen     = ~CFG_FOEN_SEL ? f_oen_sel : oen;


//==array => iopad, output path===============================================================================

//f_out/f_out_fb
always @ (posedge fclk_gate_mux or negedge od_setn or negedge od_rstn) begin
    if(~od_rstn)
        out_reg0 <= 1'b0;
    else if(~od_setn)
        out_reg0 <= 1'b1;
    else
        out_reg0 <= out[0];
end
always @ (negedge fclk_gate_mux or negedge od_setn or negedge od_rstn) begin
    if(~od_rstn)
        out_reg1 <= 1'b0;
    else if(~od_setn)
        out_reg1 <= 1'b1;
    else
        out_reg1 <= out[1];
end

//ioc_dff_asr u_od0(
//    .ck     ( fclk_gate_mux ),
//    .d      ( out[0]        ),
//    .q      ( out_reg0      ),
//    .rstn   ( od_rstn       ),
//    .setn   ( od_setn       )
//);
//
//ioc_dffn_asr u_od1(
//    .ckb    ( fclk_gate_mux ),
//    .d      ( out[1]        ),
//    .q      ( out_reg1      ),
//    .rstn   ( od_rstn       ),
//    .setn   ( od_setn       )
//);


//assign ddr_sel = ~(fclk_gate_mux & CFG_DDR);
wire   ddr_sel   = ~((~(fclk_gate_mux && CFG_DDR)) & CFG_FCLK_INV);
wire   f_out_ddr = ddr_sel ? out_reg1 : out_reg0;
wire   f_out     = ~CFG_FOUT_SEL ? f_out_ddr : out[0];
wire   f_out_fb  = f_out;

endmodule

module PVTS_TOP ( 
               PVTS_EN, 
               PVTS_DONE, 
               PVTS_DOUT, 
               PVTS_PS_MODE,
               PVTS_VS_MODE,
               PVTS_TRIM, 
               PVTS_DTEST 
     );

input  PVTS_EN,PVTS_PS_MODE,PVTS_VS_MODE;
input [4:0]  PVTS_TRIM; 
output [10:0]  PVTS_DOUT;      
output PVTS_DONE,PVTS_DTEST;

parameter SEL_CKR = 1'b0;
parameter SEL_ATEST = 2'b0;
parameter SEL_DTEST = 2'b0;
parameter SEL_CKDIV = 2'b0;
parameter MANUAL_EN = 1'b0;
parameter MANUAL_CODE = 4'b0;
parameter SEL_IREF = 3'b0;
parameter SEL_VIN = 2'b0;
parameter SEL_VREG = 2'b0;

endmodule 

// VPERL: GENERATED_BEG

module soc_top (
	m3soc_clk_o,
	m3soc_clken,
	m3soc_rstn,
	rcosc_clk,
	u_m3soc_dma_ack_o,
	u_m3soc_fp_RXEV,
	u_m3soc_fp_TRACECLKIN,
	u_m3soc_fp_clk,
	u_m3soc_fp_intr,
	u_m3soc_fp_rst_n,
	u_m3soc_fp_sram1_rdat,
	u_m3soc_fp_sram0_rdat,
	u_m3soc_haddr_m1_fp,
	u_m3soc_hburst_m1_fp,
	u_m3soc_hlock_m1_fp,
	u_m3soc_hprot_m1_fp,
	u_m3soc_hrdata_s1_fp,
	u_m3soc_hready_m1_fp,
	u_m3soc_hready_resp_s1_fp,
	u_m3soc_hresp_s1_fp,
	u_m3soc_hsel_m1_fp,
	u_m3soc_hsize_m1_fp,
	u_m3soc_htrans_m1_fp,
	u_m3soc_hwdata_m1_fp,
	u_m3soc_hwrite_m1_fp,
	u_m3soc_ic2_clk_in_a,
	u_m3soc_ic2_data_in_a_i,
	u_m3soc_ic1_clk_in_a,
	u_m3soc_ic1_data_in_a_i,
	u_m3soc_ic0_clk_in_a,
	u_m3soc_ic0_data_in_a_i,
	u_m3soc_prdata_fp,
	u_m3soc_pready_fp,
	u_m3soc_pslverr_fp,
	u_m3soc_spi2_clk_in_i,
	u_m3soc_spi2_cs_n_in_i,
	u_m3soc_spi2_hold_n_in_i,
	u_m3soc_spi2_miso_in_i,
	u_m3soc_spi2_mosi_in_i,
	u_m3soc_spi2_wp_n_in_i,
	u_m3soc_spi1_clk_in_i,
	u_m3soc_spi1_cs_n_in_i,
	u_m3soc_spi1_hold_n_in_i,
	u_m3soc_spi1_miso_in_i,
	u_m3soc_spi1_mosi_in_i,
	u_m3soc_spi1_wp_n_in_i,
	u_m3soc_spi0_clk_in_i,
	u_m3soc_spi0_cs_n_in_i,
	u_m3soc_spi0_hold_n_in_i,
	u_m3soc_spi0_miso_in_i,
	u_m3soc_spi0_mosi_in_i,
	u_m3soc_spi0_wp_n_in_i,
	u_m3soc_sram2_rdat,
	u_m3soc_uart2_cts_n_i,
	u_m3soc_uart2_dcd_n_i,
	u_m3soc_uart2_dsr_n_i,
	u_m3soc_uart2_ri_n_i,
	u_m3soc_uart2_sin_i,
	u_m3soc_uart2_sir_in_i,
	u_m3soc_uart1_cts_n_i,
	u_m3soc_uart1_dcd_n_i,
	u_m3soc_uart1_dsr_n_i,
	u_m3soc_uart1_ri_n_i,
	u_m3soc_uart1_sin_i,
	u_m3soc_uart1_sir_in_i,
	u_m3soc_uart0_cts_n_i,
	u_m3soc_uart0_dcd_n_i,
	u_m3soc_uart0_dsr_n_i,
	u_m3soc_uart0_ri_n_i,
	u_m3soc_uart0_sin_i,
	u_m3soc_uart0_sir_in_i,
	efuse_out,
	fp_sram1_clk,
	fp_sram0_clk,
	glb_clear_b_d16_fp,
	hclk_fp_brg_gate,
	m3cpr_m3soc_clk_en,
	m3cpr_m3soc_sft_rstn,
	m3cpr_sram10_clk_en,
	m3cpr_sram00_clk_en,
	u_m3soc_dma_req_i,
	u_m3soc_dma_single_i,
	u_m3soc_fp_SWV,
	u_m3soc_fp_TRACECLK,
	u_m3soc_fp_TRACEDATA,
	u_m3soc_fp_TXEV,
	u_m3soc_fp_sram1_addr,
	u_m3soc_fp_sram1_bl,
	u_m3soc_fp_sram1_ce,
	u_m3soc_fp_sram1_wdat,
	u_m3soc_fp_sram1_wr,
	u_m3soc_fp_sram0_addr,
	u_m3soc_fp_sram0_bl,
	u_m3soc_fp_sram0_ce,
	u_m3soc_fp_sram0_wdat,
	u_m3soc_fp_sram0_wr,
	u_m3soc_haddr_s1_fp,
	u_m3soc_hburst_s1_fp,
	u_m3soc_hrdata_m1_fp,
	u_m3soc_hready_resp_m1_fp,
	u_m3soc_hresp_m1_fp,
	u_m3soc_hsize_s1_fp,
	u_m3soc_htrans_s1_fp,
	u_m3soc_hwdata_s1_fp,
	u_m3soc_hwrite_s1_fp,
	u_m3soc_ic2_clk_oe,
	u_m3soc_ic2_current_src_en_o,
	u_m3soc_ic2_data_oe_o,
	u_m3soc_ic2_en_o,
	u_m3soc_ic1_clk_oe,
	u_m3soc_ic1_current_src_en_o,
	u_m3soc_ic1_data_oe_o,
	u_m3soc_ic1_en_o,
	u_m3soc_ic0_clk_oe,
	u_m3soc_ic0_current_src_en_o,
	u_m3soc_ic0_data_oe_o,
	u_m3soc_ic0_en_o,
	u_m3soc_pclk_fp,
	u_m3soc_penable_fp,
	u_m3soc_presetn_fp,
	u_m3soc_psel_fp,
	u_m3soc_pwdata_fp,
	u_m3soc_pwrite_fp,
	u_m3soc_soc_intr,
	u_m3soc_spi2_clk_oe_o,
	u_m3soc_spi2_clk_out_o,
	u_m3soc_spi2_cs_n_oe_o,
	u_m3soc_spi2_cs_n_out_o,
	u_m3soc_spi2_hold_n_oe_o,
	u_m3soc_spi2_hold_n_out_o,
	u_m3soc_spi2_miso_oe_o,
	u_m3soc_spi2_miso_out_o,
	u_m3soc_spi2_mosi_oe_o,
	u_m3soc_spi2_mosi_out_o,
	u_m3soc_spi2_wp_n_oe_o,
	u_m3soc_spi2_wp_n_out_o,
	u_m3soc_spi1_clk_oe_o,
	u_m3soc_spi1_clk_out_o,
	u_m3soc_spi1_cs_n_oe_o,
	u_m3soc_spi1_cs_n_out_o,
	u_m3soc_spi1_hold_n_oe_o,
	u_m3soc_spi1_hold_n_out_o,
	u_m3soc_spi1_miso_oe_o,
	u_m3soc_spi1_miso_out_o,
	u_m3soc_spi1_mosi_oe_o,
	u_m3soc_spi1_mosi_out_o,
	u_m3soc_spi1_wp_n_oe_o,
	u_m3soc_spi1_wp_n_out_o,
	u_m3soc_spi0_clk_oe_o,
	u_m3soc_spi0_clk_out_o,
	u_m3soc_spi0_cs_n_oe_o,
	u_m3soc_spi0_cs_n_out_o,
	u_m3soc_spi0_hold_n_oe_o,
	u_m3soc_spi0_hold_n_out_o,
	u_m3soc_spi0_miso_oe_o,
	u_m3soc_spi0_miso_out_o,
	u_m3soc_spi0_mosi_oe_o,
	u_m3soc_spi0_mosi_out_o,
	u_m3soc_spi0_wp_n_oe_o,
	u_m3soc_spi0_wp_n_out_o,
	u_m3soc_sram2_addr,
	u_m3soc_sram2_bl,
	u_m3soc_sram2_ce,
	u_m3soc_sram2_clk,
	u_m3soc_sram2_wdat,
	u_m3soc_sram2_wr,
	u_m3soc_uart2_baudout_n_o,
	u_m3soc_uart2_dtr_n_o,
	u_m3soc_uart2_out2_n_o,
	u_m3soc_uart2_out1_n_o,
	u_m3soc_uart2_rts_n_o,
	u_m3soc_uart2_sir_out_n_o,
	u_m3soc_uart2_sout_o,
	u_m3soc_uart1_baudout_n_o,
	u_m3soc_uart1_dtr_n_o,
	u_m3soc_uart1_out2_n_o,
	u_m3soc_uart1_out1_n_o,
	u_m3soc_uart1_rts_n_o,
	u_m3soc_uart1_sir_out_n_o,
	u_m3soc_uart1_sout_o,
	u_m3soc_uart0_baudout_n_o,
	u_m3soc_uart0_dtr_n_o,
	u_m3soc_uart0_out2_n_o,
	u_m3soc_uart0_out1_n_o,
	u_m3soc_uart0_rts_n_o,
	u_m3soc_uart0_sir_out_n_o,
	u_m3soc_uart0_sout_o,
    u_m3soc_paddr_fp,
    u_m3soc_gpio1_ext_porta_i,
    u_m3soc_gpio0_ext_porta_i,
    u_m3soc_gpio1_porta_ddr_o,
    u_m3soc_gpio1_porta_dr_o,
    u_m3soc_gpio0_porta_ddr_o,
    u_m3soc_gpio0_porta_dr_o,
    PVTS_EN, 
    PVTS_DONE, 
    PVTS_DOUT, 
    PVTS_PS_MODE,
    PVTS_VS_MODE,
    PVTS_TRIM, 
    PVTS_DTEST 
);

input		m3soc_clk_o;
input		m3soc_clken;
input		m3soc_rstn;
input		rcosc_clk;
input	[19:0]	u_m3soc_dma_ack_o;
input		u_m3soc_fp_RXEV;
input		u_m3soc_fp_TRACECLKIN;
input		u_m3soc_fp_clk;
input	[16:0]	u_m3soc_fp_intr;
input		u_m3soc_fp_rst_n;
input	[31:0]	u_m3soc_fp_sram1_rdat;
input	[31:0]	u_m3soc_fp_sram0_rdat;
input	[31:0]	u_m3soc_haddr_m1_fp;
input	[2:0]	u_m3soc_hburst_m1_fp;
input		u_m3soc_hlock_m1_fp;
input	[3:0]	u_m3soc_hprot_m1_fp;
input	[31:0]	u_m3soc_hrdata_s1_fp;
input		u_m3soc_hready_m1_fp;
input		u_m3soc_hready_resp_s1_fp;
input	[1:0]	u_m3soc_hresp_s1_fp;
input		u_m3soc_hsel_m1_fp;
input	[2:0]	u_m3soc_hsize_m1_fp;
input	[1:0]	u_m3soc_htrans_m1_fp;
input	[31:0]	u_m3soc_hwdata_m1_fp;
input		u_m3soc_hwrite_m1_fp;
input		u_m3soc_ic2_clk_in_a;
input		u_m3soc_ic2_data_in_a_i;
input		u_m3soc_ic1_clk_in_a;
input		u_m3soc_ic1_data_in_a_i;
input		u_m3soc_ic0_clk_in_a;
input		u_m3soc_ic0_data_in_a_i;
input	[31:0]	u_m3soc_prdata_fp;
input		u_m3soc_pready_fp;
input		u_m3soc_pslverr_fp;
input		u_m3soc_spi2_clk_in_i;
input		u_m3soc_spi2_cs_n_in_i;
input		u_m3soc_spi2_hold_n_in_i;
input		u_m3soc_spi2_miso_in_i;
input		u_m3soc_spi2_mosi_in_i;
input		u_m3soc_spi2_wp_n_in_i;
input		u_m3soc_spi1_clk_in_i;
input		u_m3soc_spi1_cs_n_in_i;
input		u_m3soc_spi1_hold_n_in_i;
input		u_m3soc_spi1_miso_in_i;
input		u_m3soc_spi1_mosi_in_i;
input		u_m3soc_spi1_wp_n_in_i;
input		u_m3soc_spi0_clk_in_i;
input		u_m3soc_spi0_cs_n_in_i;
input		u_m3soc_spi0_hold_n_in_i;
input		u_m3soc_spi0_miso_in_i;
input		u_m3soc_spi0_mosi_in_i;
input		u_m3soc_spi0_wp_n_in_i;
input	[31:0]	u_m3soc_sram2_rdat;
input		u_m3soc_uart2_cts_n_i;
input		u_m3soc_uart2_dcd_n_i;
input		u_m3soc_uart2_dsr_n_i;
input		u_m3soc_uart2_ri_n_i;
input		u_m3soc_uart2_sin_i;
input		u_m3soc_uart2_sir_in_i;
input		u_m3soc_uart1_cts_n_i;
input		u_m3soc_uart1_dcd_n_i;
input		u_m3soc_uart1_dsr_n_i;
input		u_m3soc_uart1_ri_n_i;
input		u_m3soc_uart1_sin_i;
input		u_m3soc_uart1_sir_in_i;
input		u_m3soc_uart0_cts_n_i;
input		u_m3soc_uart0_dcd_n_i;
input		u_m3soc_uart0_dsr_n_i;
input		u_m3soc_uart0_ri_n_i;
input		u_m3soc_uart0_sin_i;
input		u_m3soc_uart0_sir_in_i;
output	[31:0]	efuse_out;
output		fp_sram1_clk;
output		fp_sram0_clk;
output		glb_clear_b_d16_fp;
output		hclk_fp_brg_gate;
output		m3cpr_m3soc_clk_en;
output		m3cpr_m3soc_sft_rstn;
output		m3cpr_sram10_clk_en;
output		m3cpr_sram00_clk_en;
output	[19:0]	u_m3soc_dma_req_i;
output	[11:0]	u_m3soc_dma_single_i;
output		u_m3soc_fp_SWV;
output		u_m3soc_fp_TRACECLK;
output	[3:0]	u_m3soc_fp_TRACEDATA;
output		u_m3soc_fp_TXEV;
output	[21:0]	u_m3soc_fp_sram1_addr;
output	[3:0]	u_m3soc_fp_sram1_bl;
output		u_m3soc_fp_sram1_ce;
output	[31:0]	u_m3soc_fp_sram1_wdat;
output		u_m3soc_fp_sram1_wr;
output	[21:0]	u_m3soc_fp_sram0_addr;
output	[3:0]	u_m3soc_fp_sram0_bl;
output		u_m3soc_fp_sram0_ce;
output	[31:0]	u_m3soc_fp_sram0_wdat;
output		u_m3soc_fp_sram0_wr;
output	[31:0]	u_m3soc_haddr_s1_fp;
output	[2:0]	u_m3soc_hburst_s1_fp;
output	[31:0]	u_m3soc_hrdata_m1_fp;
output		u_m3soc_hready_resp_m1_fp;
output	[1:0]	u_m3soc_hresp_m1_fp;
output	[2:0]	u_m3soc_hsize_s1_fp;
output	[1:0]	u_m3soc_htrans_s1_fp;
output	[31:0]	u_m3soc_hwdata_s1_fp;
output		u_m3soc_hwrite_s1_fp;
output		u_m3soc_ic2_clk_oe;
output		u_m3soc_ic2_current_src_en_o;
output		u_m3soc_ic2_data_oe_o;
output		u_m3soc_ic2_en_o;
output		u_m3soc_ic1_clk_oe;
output		u_m3soc_ic1_current_src_en_o;
output		u_m3soc_ic1_data_oe_o;
output		u_m3soc_ic1_en_o;
output		u_m3soc_ic0_clk_oe;
output		u_m3soc_ic0_current_src_en_o;
output		u_m3soc_ic0_data_oe_o;
output		u_m3soc_ic0_en_o;
output		u_m3soc_pclk_fp;
output		u_m3soc_penable_fp;
output		u_m3soc_presetn_fp;
output	[15:0]	u_m3soc_psel_fp;
output	[31:0]	u_m3soc_pwdata_fp;
output		u_m3soc_pwrite_fp;
output	[16:0]	u_m3soc_soc_intr;
output		u_m3soc_spi2_clk_oe_o;
output		u_m3soc_spi2_clk_out_o;
output		u_m3soc_spi2_cs_n_oe_o;
output		u_m3soc_spi2_cs_n_out_o;
output		u_m3soc_spi2_hold_n_oe_o;
output		u_m3soc_spi2_hold_n_out_o;
output		u_m3soc_spi2_miso_oe_o;
output		u_m3soc_spi2_miso_out_o;
output		u_m3soc_spi2_mosi_oe_o;
output		u_m3soc_spi2_mosi_out_o;
output		u_m3soc_spi2_wp_n_oe_o;
output		u_m3soc_spi2_wp_n_out_o;
output		u_m3soc_spi1_clk_oe_o;
output		u_m3soc_spi1_clk_out_o;
output		u_m3soc_spi1_cs_n_oe_o;
output		u_m3soc_spi1_cs_n_out_o;
output		u_m3soc_spi1_hold_n_oe_o;
output		u_m3soc_spi1_hold_n_out_o;
output		u_m3soc_spi1_miso_oe_o;
output		u_m3soc_spi1_miso_out_o;
output		u_m3soc_spi1_mosi_oe_o;
output		u_m3soc_spi1_mosi_out_o;
output		u_m3soc_spi1_wp_n_oe_o;
output		u_m3soc_spi1_wp_n_out_o;
output		u_m3soc_spi0_clk_oe_o;
output		u_m3soc_spi0_clk_out_o;
output		u_m3soc_spi0_cs_n_oe_o;
output		u_m3soc_spi0_cs_n_out_o;
output		u_m3soc_spi0_hold_n_oe_o;
output		u_m3soc_spi0_hold_n_out_o;
output		u_m3soc_spi0_miso_oe_o;
output		u_m3soc_spi0_miso_out_o;
output		u_m3soc_spi0_mosi_oe_o;
output		u_m3soc_spi0_mosi_out_o;
output		u_m3soc_spi0_wp_n_oe_o;
output		u_m3soc_spi0_wp_n_out_o;
output	[23:0]	u_m3soc_sram2_addr;
output	[3:0]	u_m3soc_sram2_bl;
output		u_m3soc_sram2_ce;
output		u_m3soc_sram2_clk;
output	[31:0]	u_m3soc_sram2_wdat;
output		u_m3soc_sram2_wr;
output		u_m3soc_uart2_baudout_n_o;
output		u_m3soc_uart2_dtr_n_o;
output		u_m3soc_uart2_out2_n_o;
output		u_m3soc_uart2_out1_n_o;
output		u_m3soc_uart2_rts_n_o;
output		u_m3soc_uart2_sir_out_n_o;
output		u_m3soc_uart2_sout_o;
output		u_m3soc_uart1_baudout_n_o;
output		u_m3soc_uart1_dtr_n_o;
output		u_m3soc_uart1_out2_n_o;
output		u_m3soc_uart1_out1_n_o;
output		u_m3soc_uart1_rts_n_o;
output		u_m3soc_uart1_sir_out_n_o;
output		u_m3soc_uart1_sout_o;
output		u_m3soc_uart0_baudout_n_o;
output		u_m3soc_uart0_dtr_n_o;
output		u_m3soc_uart0_out2_n_o;
output		u_m3soc_uart0_out1_n_o;
output		u_m3soc_uart0_rts_n_o;
output		u_m3soc_uart0_sir_out_n_o;
output		u_m3soc_uart0_sout_o;
output [18:0] u_m3soc_paddr_fp;
input [31:0]	u_m3soc_gpio1_ext_porta_i;
input [31:0]	u_m3soc_gpio0_ext_porta_i;
output [31:0]	u_m3soc_gpio1_porta_ddr_o;
output [31:0]	u_m3soc_gpio1_porta_dr_o;
output [31:0]	u_m3soc_gpio0_porta_ddr_o;
output [31:0]	u_m3soc_gpio0_porta_dr_o;
output  PVTS_EN,PVTS_PS_MODE,PVTS_VS_MODE;
output [4:0]  PVTS_TRIM; 
input [10:0]  PVTS_DOUT;      
input PVTS_DONE,PVTS_DTEST;

parameter DEBUG_EN = 1'b0;
parameter SRAM_LOC = 2'b0;
parameter fp_interface_en = 21'b0;
parameter program_file = "";

endmodule

