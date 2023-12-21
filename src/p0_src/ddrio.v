module ddrio(
input 	wire geclk_ol,
input 	wire gsclk_ol,
input 	wire geclk_il,
input 	wire gsclk_il,
input 	wire rst,
input 	wire align_0_il,
input 	wire t1,
input 	wire t2,
input 	wire t3,
input 	wire t4,
input 	wire d1,
input 	wire d2,
input 	wire d3,
input 	wire d4,
input 	wire d5,
input 	wire d6,
input 	wire d7,
input 	wire d8,
output 	wire q1,
output 	wire q2,
output 	wire q3,
output 	wire q4,
output 	wire q5,
output 	wire q6,
output 	wire q7,
output 	wire q8,
inout  	wire dq
);
wire geclk_ol_buf_o;
wire geclk_il_buf_o;
wire internal_update_ol_d0;
wire internal_update_b_ol_d0;
wire internal_update_il_d0;
wire internal_update_b_il_d0;






`ifdef RTL

assign dq = t ? q : 1'bz;
assign q = dq;

`else
wire id;
wire f_oen;
wire f_od;
/*
IOBUF_HME u_geclk_ol(
        .i(geclk_ol),
        .o(geclk_ol_buf_o)
);
IOBUF_HME u_geclk_il(
        .i(geclk_il),
        .o(geclk_il_buf_o)
);*/
assign geclk_ol_buf_o = geclk_ol;
assign geclk_il_buf_o = geclk_il;

update_block_cnt #(
        .CFG_UP_SEL (7'b0000011),
        .CFG_GSCLK_UPO_EN (1'b1),
        .CFG_GECLK_UPO_EN (1'b1),
        .CFG_RST_EN (1)
)
u_data_ol_update_d0 (
        .update (internal_update_ol_d0),
        .update_b (internal_update_b_ol_d0),
        .geclk (geclk_ol_buf_o),
        .gsclk (gsclk_ol),
        .align_rst (rst),
        .align_user (1'b0),
        .sysclk_adjust_retval ()
);

update_block_cnt_det #(
        .CFG_UP_SEL (7'b0000011),
        .CFG_GSCLK_UPI_EN (1'b1),
        .CFG_GECLK_UPI_EN (1'b1),
        .CFG_RST_EN (1'b1)
)
u_data_il_update_d0 (
        .update (internal_update_il_d0),
        .update_b (internal_update_b_il_d0),
        .det_out (),
        .det_enb (),
        .geclk (geclk_il_buf_o),
        .gsclk (gsclk_il),
        .align_rst (rst),
        .align_user (align_0_il),
        .sysclk_adjust_retval ()
);

P0_IOC_LVDS #(
		//Output Attributes
        .CFG_CKEN_EN0_O (1'b0),
        .CFG_CKEN_EN1_O (1'b0),
        .CFG_CKEN_INV_O (1'b0),
        .CFG_FCLK0_O_EN (1'b1),
        .CFG_FCLK1_O_EN (1'b1),
        .CFG_SET_EN_O (1'b1),
        .CFG_RST_EN_O (1'b1),
//        .CFG_DDR_IN_NREG (1'b0),
//        .CFG_DDR_IN_PREG (1'b0),
//        .CFG_FASTIN (1'b0),
        .CFG_GECLK1_O_EN (1'b1),
        .CFG_GECLK0_O_EN (1'b1),
        .CFG_GSCLK0_O_EN (1'b1),
        .CFG_GSCLK1_O_EN (1'b1),
        .CFG_GEAR_OUT (1'b1),
        .CFG_DDR_OUT_REG (1'b0),
        .CFG_FOUT_SEL (1'b0),
//        .CFG_CK_PAD_EN (1'b0),
        .CFG_DDR_OUT (1'b1),
        .CFG_OEN_SEL (4'b0100),
        .CFG_FCLK_INV (1'b1),
		//Input Attributes
		.CFG_CKEN_EN0_I (1'b0),
        .CFG_CKEN_EN1_I (1'b0),
        .CFG_CKEN_INV_I (1'b0),
        .CFG_FCLK0_I_EN (1'b1),
        .CFG_FCLK1_I_EN (1'b1),
        .CFG_SET_EN_I (1'b1),
        .CFG_RST_EN_I (1'b1),
        .CFG_DDR_IN_NREG (1'b0),
        .CFG_DDR_IN_PREG (1'b0),
        .CFG_DDR_IN_NREG_DFF (1'b1),
        .CFG_DDR_IN_PREG_DFF (1'b1),
        .CFG_FASTIN (1'b0),
        .CFG_GECLK1_I_EN (1'b1),
        .CFG_GECLK0_I_EN (1'b1),
        .CFG_GSCLK0_I_EN (1'b1),
        .CFG_GSCLK1_I_EN (1'b1),
        .CFG_GEAR_IN (8'b11111111),
        .CFG_SLAVE_IN (1'b0),
        .CFG_CK_PAD_EN (1'b0)
)
u_data_p  (
	. cken_il ( 1'b1 ),
	. cken_ol ( 1'b1 ),
	. d ( {d7,d8,d5,d6,d3,d4,d1,d2} ),
	. fclk_il ( geclk_il_buf_o ),
	. fclk_ol ( geclk_ol_buf_o ),
	. fout_dyn_ctrl ( ),
	. gsclk_il ( gsclk_il ),
	. gsclk_ol ( gsclk_ol ),
	. odt_out ( ),
	. oen ( { t4,t3,t2,t1 } ),
	. oen_out ( f_oen  ),
	. phy_odt_ctrl (1'b0 ),
	. q ( {q8,q7,q6,q5,q4,q3,q2,q1} ),
	. rst_il ( rst ),
	. rst_ol ( rst ),
	. rxd_dr ( ),
	. rxd_in ( id  ),
	. set_il ( 1'b0 ),
	. set_ol ( 1'b0 ),
	. shiftin0_il ( ),
	. shiftin0_ol ( ),
	. shiftin1_il ( ),
	. shiftin1_ol ( ),
	. shiftin_oen ( ),
	. shiftout0_il ( ),
	. shiftout0_ol ( ),
	. shiftout1_il ( ),
	. shiftout1_ol ( ),
	. shiftout_oen ( ),
	. sout ( ),
	. txd_out ( f_od  ),
	. update_b_il ( internal_update_b_il_d0 ),
	. update_b_ol ( internal_update_b_ol_d0 ),
	. update_il ( internal_update_il_d0 ),
	. update_ol ( internal_update_ol_d0 )
);

IO_HP_LVPECL cvlvds_inst0  (
	. LP_RXD0 ( ),
	. LP_RXD1 ( ),
	. ODT_0 ( ),
	. ODT_1 ( ),
	. PAD0 ( dq ),
	. PAD1 (  ),
	. RXD0 ( id  ),
	. RXD1 ( ),
	. TED0 ( f_oen  ),
	. TED1 ( ),
	. TXD0 ( f_od  ),
	. TXD1 ( ),
	. fp_idly_sel_0 ( )
,
	. fp_idly_sel_1 ( )
,
	. fp_idly_update_0 ( ),
	. fp_idly_update_1 ( ),
	. fp_lvds_tx_en ( ),
	. fp_ndr_cal_0 ( )
,
	. fp_ndr_cal_1 ( )
,
	. fp_odly_sel_0 ( )
,
	. fp_odly_sel_1 ( )
,
	. fp_odly_update_0 ( ),
	. fp_odly_update_1 ( ),
	. fp_pdr_cal_0 ( )
,
	. fp_pdr_cal_1 ( )
,
	. fp_pndr_update0 ( ),
	. fp_pndr_update1 ( ),
	. fp_rx_lp_en ( ),
	. fp_td_cal ( )
,
	. fp_td_update ( ),
	. fp_term_diff_en ( ),
	. fp_tpd_cal_0 ( )
,
	. fp_tpd_cal_1 ( )
,
	. fp_tpu_cal_0 ( )
,
	. fp_tpu_cal_1 ( )
,
	. fp_tpud_update0 ( ),
	. fp_tpud_update1 ( )
);
defparam cvlvds_inst0 .CFG_TX_POST0 = 3'h0;
defparam cvlvds_inst0 .CFG_TD_CAL_EN = 1'b0;
defparam cvlvds_inst0 .CFG_TX_POST1 = 3'h0;
defparam cvlvds_inst0 .CFG_CALRX_EN_0 = 1'b0;
defparam cvlvds_inst0 .CFG_CALRX_EN_1 = 1'b0;
defparam cvlvds_inst0 .CFG_IDLC1_BYPASS = 1'b0;
defparam cvlvds_inst0 .CFG_LVPECL_TX_EN = 1'b0;
defparam cvlvds_inst0 .CFG_IDLC0_BYPASS = 1'b1;
defparam cvlvds_inst0 .CFG_CMFB_TX_EN = 1'b0;
defparam cvlvds_inst0 .CFG_PNDR_UD_BYP0 = 1'b1;
defparam cvlvds_inst0 .CFG_RX_DIG_EN_0 = 1;
defparam cvlvds_inst0 .CFG_ODLC1_MODE = 1'b0;
defparam cvlvds_inst0 .CFG_PNDR_UD_BYP1 = 1'b0;
defparam cvlvds_inst0 .CFG_RX_DIG_EN_1 = 1'b0;
defparam cvlvds_inst0 .CFG_IDLC0_DEL_SEL = 5'h0;
defparam cvlvds_inst0 .CFG_NDR_0 = 15;
defparam cvlvds_inst0 .CFG_TX_PRE_EN0 = 1'b0;
defparam cvlvds_inst0 .CFG_DYN_TERM_EN = 1'b0;
defparam cvlvds_inst0 .CFG_ATEST_SEL = 4'h0;
defparam cvlvds_inst0 .CFG_IDLC1_DEL_SEL = 5'h0;
defparam cvlvds_inst0 .CFG_NDR_1 = 7'h1b;
defparam cvlvds_inst0 .CFG_TX_PRE_EN1 = 1'b0;
defparam cvlvds_inst0 .CFG_PDR_0 = 15;
defparam cvlvds_inst0 .CFG_PDR_1 = 7'h1c;
defparam cvlvds_inst0 .CFG_TPD_0 = 7'h0;
defparam cvlvds_inst0 .CFG_LVPECL_TERM_EN = 1'b0;
defparam cvlvds_inst0 .CFG_TPD_1 = 7'h0;
defparam cvlvds_inst0 .CFG_TPD_CAL_EN0 = 1'b0;
defparam cvlvds_inst0 .CFG_IDLC1_MODE = 1'b0;
defparam cvlvds_inst0 .CFG_ODLC0_DEL_SEL = 5'h0;
defparam cvlvds_inst0 .CFG_TPD_CAL_EN1 = 1'b0;
defparam cvlvds_inst0 .CFG_ODLC1_DEL_SEL = 5'h0;
defparam cvlvds_inst0 .CFG_TX_POST_EN0 = 1'b0;
defparam cvlvds_inst0 .CFG_ATEST_EN = 1'b0;
defparam cvlvds_inst0 .CFG_LVPECL_TUNE_EN = 1'b0;
defparam cvlvds_inst0 .CFG_TX_POST_EN1 = 1'b0;
defparam cvlvds_inst0 .CFG_KEEP_0 = 0;
defparam cvlvds_inst0 .CFG_ODLC1_BYPASS = 1'b0;
defparam cvlvds_inst0 .CFG_RX_SINGLE_EN_0 = 1'b0;
defparam cvlvds_inst0 .CFG_KEEP_1 = 2'h0;
defparam cvlvds_inst0 .CFG_PNDR_CAL_EN0 = 1'b0;
defparam cvlvds_inst0 .CFG_RX_SINGLE_EN_1 = 1'b0;
defparam cvlvds_inst0 .CFG_PNDR_CAL_EN1 = 1'b0;
defparam cvlvds_inst0 .CFG_TPUD_UD_BYP0 = 1'b0;
defparam cvlvds_inst0 .CFG_ST0 = 1'b0;
defparam cvlvds_inst0 .CFG_TPUD_UD_BYP1 = 1'b0;
defparam cvlvds_inst0 .CFG_ST1 = 1'b0;
defparam cvlvds_inst0 .CFG_DYN_LVDS_TX_EN = 1'b0;
defparam cvlvds_inst0 .CFG_IO_BIAS_EN = 1'b1;
defparam cvlvds_inst0 .CFG_VCM0P9_EN = 1'b0;
defparam cvlvds_inst0 .CFG_TX_PRE0 = 3'h0;
defparam cvlvds_inst0 .CFG_TX_PRE1 = 3'h0;
defparam cvlvds_inst0 .CFG_ODLC0_BYPASS = 1'b1;
defparam cvlvds_inst0 .CFG_ODLC0_MODE = 1'b0;
defparam cvlvds_inst0 .CFG_TPU_CAL_EN0 = 1'b0;
defparam cvlvds_inst0 .CFG_TERM_DIFF_EN = 1'b0;
defparam cvlvds_inst0 .CFG_TD_UD_BYP = 1'b0;
defparam cvlvds_inst0 .CFG_TPU_CAL_EN1 = 1'b0;
defparam cvlvds_inst0 .CFG_RX_LP_EN_0 = 1;
defparam cvlvds_inst0 .CFG_RX_LP_EN_1 = 1'b0;
defparam cvlvds_inst0 .CFG_IDLC0_MODE = 1'b0;
defparam cvlvds_inst0 .CFG_VREF_EN = 1'b0;
defparam cvlvds_inst0 .CFG_LVDS_TX_EN = 1'b0;
defparam cvlvds_inst0 .CFG_TPU_0 = 7'h0;
defparam cvlvds_inst0 .CFG_TD = 5'h0;
defparam cvlvds_inst0 .CFG_TPU_1 = 7'h0;
defparam cvlvds_inst0 .CFG_NS_LV_0 = 3;
defparam cvlvds_inst0 .CFG_SMIT_TUNE_EN = 1'b0;
defparam cvlvds_inst0 .CFG_LDR = 2'h0;
defparam cvlvds_inst0 .CFG_NS_LV_1 = 2'h0;
defparam cvlvds_inst0 .CFG_INTREF_VREF_EN = 1'b0;
defparam cvlvds_inst0 .CFG_RX_DIFF_EN = 1'b0;
defparam cvlvds_inst0 .CFG_INTREF_VREF_SEL = 4'h0;
defparam cvlvds_inst0 .CFG_DYN_LP0 = 1'b0;
defparam cvlvds_inst0 .CFG_DYN_LP1 = 1'b0;

`endif

endmodule