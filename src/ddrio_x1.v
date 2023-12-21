module ddrio_x1(
input 	wire geclk_ol    ,
input 	wire gsclk_ol    ,
input 	wire geclk_il    ,
input 	wire gsclk_il    ,
input 	wire rst	     ,
input 	wire cken	     ,
input   wire align_il    ,
input 	wire align_ol    ,
input 	wire phy_odt_ctrl_0,
input 	wire [3:0] t_0	 ,
input 	wire [1:0] d_0 	 ,
output 	wire [1:0] q_0 	 ,
inout  	wire dq_0 
);

wire        geclk_ol_buf_o          ;
wire        geclk_il_buf_o          ;
wire        internal_update_ol_d0   ;
wire        internal_update_b_ol_d0 ;
wire        internal_update_il_d0   ;
wire        internal_update_b_il_d0 ;
wire        ODT_0				    ;

wire [9:0] internal_sout0_0;
wire [9:0] internal_sout1_0;
wire internal_shiftin0_il_0, internal_shiftin0_ol_0;
wire internal_shiftin1_il_0, internal_shiftin1_ol_0;
wire internal_shiftin_oen_0, internal_shiftout_oen_0;
wire internal_shiftout0_il_0, internal_shiftout0_ol_0;
wire internal_shiftout1_il_0, internal_shiftout1_ol_0;
wire [15:0] sout_data_0    		    ;
wire [15:0] internal_sout_data_0    ;

assign internal_sout_data_0 = {internal_sout1_0[0], internal_sout1_0[1], internal_sout1_0[2], internal_sout1_0[3], internal_sout1_0[4], internal_sout1_0[5],
                                internal_shiftout0_il_0, internal_shiftout1_il_0,
                                internal_sout0_0[0], internal_sout0_0[1], internal_sout0_0[2], internal_sout0_0[3], internal_sout0_0[4], internal_sout0_0[5], internal_sout0_0[6], internal_sout0_0[7]};
assign sout_data_0[15:8]    = internal_sout_data_0[15:8];

`ifdef RTL

assign dq = t ? q : 1'bz;
assign q = dq;

`else
wire id_0	;
wire f_oen_0;
wire f_od_0 ;
//wire id_1	;
//wire f_oen_1;
//wire f_od_1 ;

assign geclk_ol_buf_o = geclk_ol;
assign geclk_il_buf_o = geclk_il;

P0_IOC_GPIO_HP ioc_cmos_inst0  (
	. cken_il     ( cken),
	. cken_ol     ( cken),
	. d 	      ( {d_0[0],d_0[1]} ),
	. fclk_il     ( geclk_il_buf_o),
	. fclk_ol     ( geclk_ol_buf_o),
	. fp_odt_ctrl ( ),
	. odt_out     ( ODT_0),
	. oen 	      ( t_0),
	. oen_out     ( f_oen_0  ),
	. q 	      ( {q_0[0],q_0[1]}),
	. rst_il      ( rst),
	. rst_ol      ( rst),
	. rxd_dr      ( ),
	. rxd_in      ( id_0  ),
	. setn_il     ( ),
	. setn_ol     ( ),
	. txd_out     ( f_od_0  )
);
defparam ioc_cmos_inst0 .CFG_EN0_O = 1'b0;
defparam ioc_cmos_inst0 .CFG_RST_SYN_I = 1'b0;
defparam ioc_cmos_inst0 .CFG_EN1_O = 1'b0;
defparam ioc_cmos_inst0 .CFG_RST_INV_O = 1'b0;
defparam ioc_cmos_inst0 .CFG_RST_EN_I = 2'h0;
defparam ioc_cmos_inst0 .CFG_FCLK1_I_EN = 1'b1;
defparam ioc_cmos_inst0 .CFG_DDR_IN_NREG = 1'b0;
defparam ioc_cmos_inst0 .CFG_SET_EN_O = 2'h0;
defparam ioc_cmos_inst0 .CFG_FCLK_INV = 1'b1;
defparam ioc_cmos_inst0 .CFG_SET_SYN_I = 1'b0;
defparam ioc_cmos_inst0 .CFG_OEN_SEL = 4'b0100;
defparam ioc_cmos_inst0 .CFG_CKEN_INV_I = 1'b0;
defparam ioc_cmos_inst0 .CFG_DDR_IN_PREG = 1;
defparam ioc_cmos_inst0 .CFG_SET_INV_O = 1'b0;
defparam ioc_cmos_inst0 .CFG_OEN_INV = 1'b0;
defparam ioc_cmos_inst0 .CFG_FCLK_SR_I_EN = 1'b0;
defparam ioc_cmos_inst0 .CFG_DDR_OUT_REG = 1'b0;
defparam ioc_cmos_inst0 .CFG_OEN_DDR_OUT_REG = 1'b1;
defparam ioc_cmos_inst0 .CFG_RST_SYN_O = 1'b0;
defparam ioc_cmos_inst0 .CFG_FCLK0_ODT_EN = 1'b0;
defparam ioc_cmos_inst0 .CFG_RST_EN_O = 2'h0;
defparam ioc_cmos_inst0 .CFG_ODT_EN = 1'b1;
defparam ioc_cmos_inst0 .CFG_FOUT_SEL = 1'b0;
defparam ioc_cmos_inst0 .CFG_FCLK1_O_EN = 1'b1;
defparam ioc_cmos_inst0 .CFG_DDR_OUT = 1;
defparam ioc_cmos_inst0 .CFG_FCLK0_I_EN = 1'b1;
defparam ioc_cmos_inst0 .CFG_DDR_IN_NREG_DFF = 1;
defparam ioc_cmos_inst0 .CFG_SET_SYN_O = 1'b0;
defparam ioc_cmos_inst0 .CFG_FCLK_OEN_EN = 1'b1;
defparam ioc_cmos_inst0 .CFG_CKEN_INV_O = 1'b0;
defparam ioc_cmos_inst0 .CFG_EN0_I = 1'b0;
defparam ioc_cmos_inst0 .CFG_OUT_INV = 1'b0;
defparam ioc_cmos_inst0 .CFG_FCLK_SR_O_EN = 1'b0;
defparam ioc_cmos_inst0 .CFG_FASTIN_0 = 1'b0;
defparam ioc_cmos_inst0 .CFG_CK_PAD_EN = 1'b0;
defparam ioc_cmos_inst0 .CFG_DDR_IN_PREG_DFF = 0;
defparam ioc_cmos_inst0 .CFG_EN1_I = 1'b0;
defparam ioc_cmos_inst0 .CFG_RST_INV_I = 1'b0;
defparam ioc_cmos_inst0 .CFG_FASTIN_1 = 1'b0;
defparam ioc_cmos_inst0 .CFG_SET_EN_I = 2'h0;
defparam ioc_cmos_inst0 .CFG_OFDBK = 1'b0;
defparam ioc_cmos_inst0 .CFG_ODT_PHASE = 4'h0;
defparam ioc_cmos_inst0 .CFG_SET_INV_I = 1'b0;
defparam ioc_cmos_inst0 .CFG_FCLK0_O_EN = 1'b1;


P0_GPIO_HP basic_io_inst0 (
	.ODT			( ODT_0		),
	.PAD			( dq_0		),
	.RXD			( id_0		),
	.TED			( f_oen_0	),
	.TXD			( f_od_0	),
	.fp_ndr_cal		(			),
	.fp_pdr_cal		(			),
	.fp_pndr_update (			),
	.fp_tpd_cal		(			),
	.fp_tpu_cal		(			),
	.fp_tpud_update (			)
);
defparam basic_io_inst0.CFG_TX_POST = 3'h0;
defparam basic_io_inst0.CFG_TPU = 7'h0;
defparam basic_io_inst0.CFG_TPU_CAL_EN = 1'b0;
defparam basic_io_inst0.CFG_NS_LV = 3;
defparam basic_io_inst0.CFG_VREF_EN = 1'b0;
defparam basic_io_inst0.CFG_KEEP = 0;
defparam basic_io_inst0.CFG_RX_SINGLE_EN = 1'b0;
defparam basic_io_inst0.CFG_PNDR_UD_BYP = 1'b1;
defparam basic_io_inst0.CFG_RX_DIG_EN = 1'b1;
defparam basic_io_inst0.CFG_TPD_CAL_EN = 1'b0;
defparam basic_io_inst0.CFG_TX_PRE = 3'h0;
defparam basic_io_inst0.CFG_SMIT_TUNE_EN = 1'b0;
defparam basic_io_inst0.CFG_TX_POST_EN = 1'b0;
defparam basic_io_inst0.CFG_TX_PRE_EN = 1'b0;
defparam basic_io_inst0.CFG_NDR = 7'b1111100;
defparam basic_io_inst0.CFG_PNDR_CAL_EN = 1'b0;
defparam basic_io_inst0.CFG_PDR = 7'b1001101;
defparam basic_io_inst0.CFG_TPUD_UD_BYP = 1'b1;
defparam basic_io_inst0.CFG_ST = 1'b0;
defparam basic_io_inst0.CFG_TPD = 7'b0101101;

`endif

endmodule