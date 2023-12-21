module PLLE2_ADV_HME(
    CLKFBIN,
    CLKIN1,
    PWRDWN,
    RST,
    CLKFBOUT,
    CLKOUT0,
	CLKOUT1,
	CLKOUT2,
    CLKOUT3,
	CLKOUT4,
	CLKOUT5,
	LOCKED
);

input CLKFBIN;
input CLKIN1;
input PWRDWN;
input RST;
output CLKFBOUT;
output CLKOUT0;
output CLKOUT1;
output CLKOUT2;
output CLKOUT3;
output CLKOUT4;
output CLKOUT5;
output LOCKED;

parameter CLKFBOUT_MULT = 0;
parameter CLKIN1_PERIOD = 0;
parameter CLKOUT0_DIVIDE = 0;
parameter CLKOUT0_PHASE = 0;
parameter CLKOUT1_DIVIDE = 0;
parameter CLKOUT1_PHASE = 0;
parameter CLKOUT2_DIVIDE = 0;
parameter CLKOUT2_PHASE = 0;
parameter CLKOUT3_DIVIDE = 0;
parameter CLKOUT3_PHASE = 0;
parameter CLKOUT4_DIVIDE = 0;
parameter CLKOUT4_PHASE = 0;
parameter DIVCLK_DIVIDE = 0;
parameter REF_JITTER1 = 0;
parameter STARTUP_WAIT = 0;

pll_v1 pll_v1_inst(
    //.pwrdown( ~PWRDWN ),
	.pll_rst( RST ),
    .clkin0 ( CLKIN1 ),
    .locked ( LOCKED ),
    .clkout0( CLKOUT0 ),
    .clkout1( CLKOUT1 ),
    .clkout2( CLKOUT2 ),
    .clkout3( CLKOUT3 ),
	.clkout4( CLKOUT4 ),
	.clkout5( CLKOUT5 )
);


endmodule
