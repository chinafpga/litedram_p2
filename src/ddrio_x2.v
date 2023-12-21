module ddrio_x2(
input 	wire geclk_ol    ,
input 	wire gsclk_ol    ,
input 	wire geclk_il    ,
input 	wire gsclk_il    ,
input 	wire align_rst_ol,
input 	wire rst	     ,
input 	wire cken	     ,
input   wire align_il    ,
input 	wire align_ol    ,
input 	wire phy_odt_ctrl_0,
input 	wire phy_odt_ctrl_1,
input 	wire [3:0] t_0	 ,
input 	wire [7:0] d_0 	 ,
output 	wire [7:0] q_0 	 ,
input 	wire [3:0] t_1	 ,
input 	wire [7:0] d_1 	 ,
output 	wire [7:0] q_1 	 ,
inout  	wire dq_0		 , 
inout  	wire dq_1 
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

reg align_rst_o;
reg align_rst_i;
reg align_ol_d1;

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
wire id_1	;
wire f_oen_1;
wire f_od_1 ;
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

//always@(posedge geclk_ol_buf_o)
always@(posedge geclk_ol_buf_o or posedge align_rst_ol)
begin
	if (align_rst_ol) begin
		align_rst_o <= 1'b1;
		align_rst_i <= 1'b1;
	end else begin
		align_rst_o <= 1'b0;
		align_rst_i <= 1'b0;
	end
end

always@(posedge geclk_ol_buf_o or posedge align_rst_ol)
begin
	if (align_rst_ol)
		align_ol_d1 <= 1'b1;
	else
		align_ol_d1 <= align_ol;
end

update_block_cnt #(
    .CFG_UP_SEL 	  	  (7'b0000011),
    .CFG_GSCLK_UPO_EN 	  (1'b1		 ),
    .CFG_GECLK_UPO_EN 	  (1'b1		 ),
    .CFG_RST_EN		  	  (1'b1		 )
)

u_data_ol_update_d0 (
    .update               (internal_update_ol_d0  ),
    .update_b             (internal_update_b_ol_d0),
    .geclk 	              (geclk_ol_buf_o		  ),
    .gsclk                (gsclk_ol				  ),
    .align_rst            (align_rst_o			  ),
    //.align_rst            (rst		  ),
    .align_user           (align_ol_d1				),
    .sysclk_adjust_retval (						  )
);

update_block_cnt_det #(
    .CFG_UP_SEL 	  	  (7'b0000011),
    .CFG_GSCLK_UPI_EN 	  (1'b1		 ),
    .CFG_GECLK_UPI_EN 	  (1'b1		 ),
    .CFG_RST_EN 	  	  (1'b1		 )
)
u_data_il_update_d0 (
    .update     		  (internal_update_il_d0  ),
    .update_b   		  (internal_update_b_il_d0),
    .det_out    		  (internal_det_out_0     ),
    .det_enb    		  (internal_det_enb_0     ),
    .geclk      		  (geclk_il_buf_o         ),
    .gsclk      		  (gsclk_il		          ),
    .align_rst            (align_rst_o			  ),
    //.align_rst  		  (rst     		          ),
    .align_user 		  (align_il		          ),
    .sysclk_adjust_retval (					      )
);

key_detec_top #(
    .CFG_FCLK_DET_EN      (1'b1       ),
    .cfg_detect_cont      (1'b0       ),
    .cfg_detect_en        (1'b0       ),
    .cfg_key_word         (8'b00000000),
    .cfg_detect_mode      (16'h00ff   )
)
u_idet_0 (
    .det_clk      		  (geclk_il_buf_o       ),
    .fp_det_en    		  (1'b0                 ),
    .rst0         		  (align_rst_o                  ),
    .rst1         		  (align_rst_o                  ),
    .det_out      		  (internal_det_out_0   ),
    .rst0_o       		  (internal_det_rst0_o_0),
    .rst1_o       		  (internal_det_rst1_o_0),
    .data         		  (sout_data_0          ),
    .det_enb_out  		  (internal_det_enb_0   )
);

P2_IOC_LVDS #(
	//Output Attributes
    .CFG_CKEN_EN0_O       (1'b1		  ),
    .CFG_CKEN_EN1_O       (1'b0		  ),
    .CFG_CKEN_INV_O       (1'b0		  ),
    .CFG_FCLK0_O_EN       (1'b1		  ),
    .CFG_FCLK1_O_EN       (1'b1		  ),
    .CFG_SET_EN_O         (1'b1		  ),
    .CFG_RST_EN_O         (1'b1		  ),
//	.CFG_DDR_IN_NREG      (1'b0		  ),
//	.CFG_DDR_IN_PREG      (1'b0		  ),
//	.CFG_FASTIN           (1'b0		  ),
    .CFG_GECLK1_O_EN      (1'b1		  ),
    .CFG_GECLK0_O_EN      (1'b1		  ),
    .CFG_GSCLK0_O_EN      (1'b1		  ),
    .CFG_GSCLK1_O_EN      (1'b1		  ),
    .CFG_GEAR_OUT         (1'b1		  ),
    .CFG_DDR_OUT_REG      (1'b0		  ),
    .CFG_FOUT_SEL         (1'b0		  ),
//	.CFG_CK_PAD_EN        (1'b0		  ),
    .CFG_DDR_OUT          (1'b1		  ),
    .CFG_OEN_SEL          (4'b0100	  ),
    .CFG_FCLK_INV         (1'b1		  ),
	//Input Attributes
	.CFG_CKEN_EN0_I       (1'b0		  ),
    .CFG_CKEN_EN1_I       (1'b0		  ),
    .CFG_CKEN_INV_I       (1'b0		  ),
    .CFG_FCLK0_I_EN       (1'b1		  ),
    .CFG_FCLK1_I_EN       (1'b1		  ),
    .CFG_SET_EN_I         (1'b1		  ),
    .CFG_RST_EN_I         (1'b1		  ),
    .CFG_DDR_IN_NREG      (1'b1		  ),
    .CFG_DDR_IN_PREG      (1'b1		  ),
    .CFG_DDR_IN_NREG_DFF  (1'b0		  ),
    .CFG_DDR_IN_PREG_DFF  (1'b0		  ),
    .CFG_FASTIN           (1'b0		  ),
    .CFG_GECLK1_I_EN      (1'b1		  ),
    .CFG_GECLK0_I_EN      (1'b1		  ),
    .CFG_GSCLK0_I_EN      (1'b1		  ),
    .CFG_GSCLK1_I_EN      (1'b1		  ),
    .CFG_GEAR_IN          (8'b11111111),
    .CFG_SLAVE_IN         (1'b0		  ),
    .CFG_CK_PAD_EN        (1'b0		  ),
//	.CFG_FCLK0_ODT_EN     (1'b1		  ),
	.CFG_ODT_PHASE        (4'b0000 	  ),
	.CFG_ODT_EN           (1'b1		  )
)
u_data_p  (
	.cken_il 		( cken 						),
	.cken_ol 		( cken 						),
	.d 	  		( {d_0[6],d_0[7],d_0[4],d_0[5],d_0[2],d_0[3],d_0[0],d_0[1]} ),
	.fclk_il 		( geclk_il_buf_o 			),
	.fclk_ol 		( geclk_ol_buf_o 			),
	.fout_dyn_ctrl ( 							),
	.gsclk_il 		( gsclk_il 					),
	.gsclk_ol 		( gsclk_ol 					),
	.odt_out  		( ODT_0 					),
	.oen 			( t_0				   	    ),
	.oen_out 		( f_oen_0				    ),
	.phy_odt_ctrl 	(phy_odt_ctrl_0 			),
	.q 		   	( {q_0[0],q_0[1],q_0[2],q_0[3],q_0[4],q_0[5],q_0[6],q_0[7]} ),
	.rst_il       	( align_rst_o 						),
	.rst_ol       	( align_rst_o 						),
	.rxd_dr       	( 							),
	.rxd_in       	( id_0    					),
	.set_il       	( 1'b0  					),
	.set_ol       	( 1'b0  					),
	.shiftin0_il    ( internal_shiftin0_il_0    ),
    .shiftin0_ol    ( internal_shiftin0_ol_0    ),
    .shiftin1_il    ( internal_shiftin1_il_0    ),
    .shiftin1_ol    ( internal_shiftin1_ol_0    ),
    .shiftin_oen    ( internal_shiftin_oen_0    ),
    .shiftout0_il   ( internal_shiftout0_il_0   ),
    .shiftout0_ol   ( internal_shiftout0_ol_0   ),
    .shiftout1_il   ( internal_shiftout1_il_0   ),
    .shiftout1_ol   ( internal_shiftout1_ol_0   ),
    .shiftout_oen   ( internal_shiftout_oen_0   ),
	.sout 		   	( internal_sout0_0 			),
	.txd_out	    ( f_od_0  					),
	.update_b_il   ( internal_update_b_il_d0 	),
	.update_b_ol   ( internal_update_b_ol_d0 	),
	.update_il	    ( internal_update_il_d0 	),
	.update_ol     ( internal_update_ol_d0	 	)
);

P2_IOC_LVDS #(
	//Output Attributes
    .CFG_CKEN_EN0_O       (1'b1		  ),
    .CFG_CKEN_EN1_O       (1'b0		  ),
    .CFG_CKEN_INV_O       (1'b0		  ),
    .CFG_FCLK0_O_EN       (1'b1		  ),
    .CFG_FCLK1_O_EN       (1'b1		  ),
    .CFG_SET_EN_O         (1'b1		  ),
    .CFG_RST_EN_O         (1'b1		  ),
//	.CFG_DDR_IN_NREG      (1'b0		  ),
//	.CFG_DDR_IN_PREG      (1'b0		  ),
//	.CFG_FASTIN           (1'b0		  ),
    .CFG_GECLK1_O_EN      (1'b1		  ),
    .CFG_GECLK0_O_EN      (1'b1		  ),
    .CFG_GSCLK0_O_EN      (1'b1		  ),
    .CFG_GSCLK1_O_EN      (1'b1		  ),
    .CFG_GEAR_OUT         (1'b1		  ),
    .CFG_DDR_OUT_REG      (1'b0		  ),
    .CFG_FOUT_SEL         (1'b0		  ),
//	.CFG_CK_PAD_EN        (1'b0		  ),
    .CFG_DDR_OUT          (1'b1		  ),
    .CFG_OEN_SEL          (4'b0100	  ),
    .CFG_FCLK_INV         (1'b1		  ),
	//Input Attributes
	.CFG_CKEN_EN0_I       (1'b0		  ),
    .CFG_CKEN_EN1_I       (1'b0		  ),
    .CFG_CKEN_INV_I       (1'b0		  ),
    .CFG_FCLK0_I_EN       (1'b1		  ),
    .CFG_FCLK1_I_EN       (1'b1		  ),
    .CFG_SET_EN_I         (1'b1		  ),
    .CFG_RST_EN_I         (1'b1		  ),
    .CFG_DDR_IN_NREG      (1'b1		  ),
    .CFG_DDR_IN_PREG      (1'b1		  ),
    .CFG_DDR_IN_NREG_DFF  (1'b0		  ),
    .CFG_DDR_IN_PREG_DFF  (1'b0		  ),
    .CFG_FASTIN           (1'b0		  ),
    .CFG_GECLK1_I_EN      (1'b1		  ),
    .CFG_GECLK0_I_EN      (1'b1		  ),
    .CFG_GSCLK0_I_EN      (1'b1		  ),
    .CFG_GSCLK1_I_EN      (1'b1		  ),
    .CFG_GEAR_IN          (8'b11111111),
    .CFG_SLAVE_IN         (1'b0		  ),
    .CFG_CK_PAD_EN        (1'b0		  ),
//	.CFG_FCLK0_ODT_EN     (1'b1		  ),
	.CFG_ODT_PHASE        (4'b0000 	  ),
	.CFG_ODT_EN           (1'b1		  )
)
u_data_n  (
	.cken_il 		( cken 						),
	.cken_ol 		( cken 						),
	.d 	  		    ( {d_1[6],d_1[7],d_1[4],d_1[5],d_1[2],d_1[3],d_1[0],d_1[1]} ),
	.fclk_il 		( geclk_il_buf_o 			),
	.fclk_ol 		( geclk_ol_buf_o 			),
	.fout_dyn_ctrl  ( 							),
	.gsclk_il 		( gsclk_il 					),
	.gsclk_ol 		( gsclk_ol 					),
	.odt_out  		( ODT_1 					),
	.oen 			( t_1				   	    ),
	.oen_out 		( f_oen_1				    ),
	.phy_odt_ctrl 	(phy_odt_ctrl_1 			),
	.q 		   	    ( {q_1[0],q_1[1],q_1[2],q_1[3],q_1[4],q_1[5],q_1[6],q_1[7]} ),
	.rst_il       	( align_rst_o 						),
	.rst_ol       	( align_rst_o 						),
	.rxd_dr       	( 							),
	.rxd_in       	( id_1    					),
	.set_il       	( 1'b0  					),
	.set_ol       	( 1'b0  					),
	.shiftin0_il    ( internal_shiftout0_il_0   ),
    .shiftin0_ol    ( internal_shiftout0_ol_0   ),
    .shiftin1_il    ( internal_shiftout1_il_0   ),
    .shiftin1_ol    ( internal_shiftout1_ol_0   ),
    .shiftin_oen    ( internal_shiftout_oen_0   ),
    .shiftout0_il   ( internal_shiftin0_il_0    ),
    .shiftout0_ol   ( internal_shiftin0_ol_0    ),
    .shiftout1_il   ( internal_shiftin1_il_0    ),
    .shiftout1_ol   ( internal_shiftin1_ol_0    ),
    .shiftout_oen   ( internal_shiftin_oen_0    ),
	.sout 		   	( internal_sout1_0 			),
	.txd_out	    ( f_od_1  					),
	.update_b_il    ( internal_update_b_il_d0 	),
	.update_b_ol    ( internal_update_b_ol_d0 	),
	.update_il	    ( internal_update_il_d0 	),
	.update_ol      ( internal_update_ol_d0	 	)
);

IO_HP_LVPECL cvlvds_inst0  (
	.LP_RXD0          (		),
	.LP_RXD1          (		),
	.ODT_0            ( ODT_0	),
	.ODT_1            ( ODT_1	),
	.PAD0             ( dq_0	),
	.PAD1             ( dq_1	),
	.RXD0             ( id_0		),
	.RXD1             ( id_1	),
	.TED0             ( f_oen_0	),
	.TED1             ( f_oen_1	),
	.TXD0             ( f_od_0	),
	.TXD1             ( f_od_1	),
	.fp_idly_sel_0    (		),
	.fp_idly_sel_1    (		),
	.fp_idly_update_0 (		),
	.fp_idly_update_1 (		),
	.fp_lvds_tx_en    (		),
	.fp_ndr_cal_0     (		),
	.fp_ndr_cal_1     (		),
	.fp_odly_sel_0    (		),
	.fp_odly_sel_1    (		),
	.fp_odly_update_0 (		),
	.fp_odly_update_1 (		),
	.fp_pdr_cal_0     (		),
	.fp_pdr_cal_1     (		),
	.fp_pndr_update0  (		),
	.fp_pndr_update1  (		),
	.fp_rx_lp_en      (		),
	.fp_td_cal        (		),
	.fp_td_update     (		),
	.fp_term_diff_en  (		),
	.fp_tpd_cal_0     (		),
	.fp_tpd_cal_1     (		),
	.fp_tpu_cal_0     (		),
	.fp_tpu_cal_1     (		),
	.fp_tpud_update0  (		),
	.fp_tpud_update1  (		)
);
defparam cvlvds_inst0.CFG_TX_POST0        = 3'h0	   ;
defparam cvlvds_inst0.CFG_TD_CAL_EN       = 1'b0	   ;
defparam cvlvds_inst0.CFG_TX_POST1        = 3'h0	   ;
defparam cvlvds_inst0.CFG_CALRX_EN_0      = 1'b0	   ;
defparam cvlvds_inst0.CFG_CALRX_EN_1      = 1'b0	   ;
defparam cvlvds_inst0.CFG_IDLC1_BYPASS    = 1'b1	   ;
defparam cvlvds_inst0.CFG_LVPECL_TX_EN    = 1'b0	   ;
defparam cvlvds_inst0.CFG_IDLC0_BYPASS    = 1'b1	   ;
defparam cvlvds_inst0.CFG_CMFB_TX_EN      = 1'b0	   ;
defparam cvlvds_inst0.CFG_PNDR_UD_BYP0    = 1'b1	   ;
defparam cvlvds_inst0.CFG_RX_DIG_EN_0     = 1'b0   	   ;
defparam cvlvds_inst0.CFG_ODLC1_MODE      = 1'b0	   ;
defparam cvlvds_inst0.CFG_PNDR_UD_BYP1    = 1'b1	   ;
defparam cvlvds_inst0.CFG_RX_DIG_EN_1     = 1'b0   	   ;
defparam cvlvds_inst0.CFG_IDLC0_DEL_SEL   = 5'h0	   ;
defparam cvlvds_inst0.CFG_NDR_0 	   	   = 7'b1111100;
defparam cvlvds_inst0.CFG_TX_PRE_EN0  	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_DYN_TERM_EN 	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_ATEST_SEL   	   = 4'h0	   ;
defparam cvlvds_inst0.CFG_IDLC1_DEL_SEL   = 5'h0	   ;
defparam cvlvds_inst0.CFG_NDR_1 		   = 7'b1111100;
defparam cvlvds_inst0.CFG_TX_PRE_EN1      = 1'b0      ;
defparam cvlvds_inst0.CFG_PDR_0 		   = 7'b1001101;
defparam cvlvds_inst0.CFG_PDR_1 		   = 7'b1001101;
defparam cvlvds_inst0.CFG_TPD_0 		   = 7'b0101101;
defparam cvlvds_inst0.CFG_LVPECL_TERM_EN  = 1'b0	   ;
defparam cvlvds_inst0.CFG_TPD_1		       = 7'b0101101;
defparam cvlvds_inst0.CFG_TPD_CAL_EN0 	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_IDLC1_MODE  	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_ODLC0_DEL_SEL   = 5'h0	   ;
defparam cvlvds_inst0.CFG_TPD_CAL_EN1 	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_ODLC1_DEL_SEL   = 5'h0	   ;
defparam cvlvds_inst0.CFG_TX_POST_EN0 	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_ATEST_EN 	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_LVPECL_TUNE_EN  = 1'b0	   ;
defparam cvlvds_inst0.CFG_TX_POST_EN1     = 1'b0	   ;
defparam cvlvds_inst0.CFG_KEEP_0 		   = 0   	   ;
defparam cvlvds_inst0.CFG_ODLC1_BYPASS    = 1'b0	   ;
defparam cvlvds_inst0.CFG_RX_SINGLE_EN_0  = 1'b1	   ;
defparam cvlvds_inst0.CFG_KEEP_1 		   = 0   	   ;
defparam cvlvds_inst0.CFG_PNDR_CAL_EN0    = 1'b0	   ;
defparam cvlvds_inst0.CFG_RX_SINGLE_EN_1  = 1'b1	   ;
defparam cvlvds_inst0.CFG_PNDR_CAL_EN1    = 1'b0	   ;
defparam cvlvds_inst0.CFG_TPUD_UD_BYP0    = 1'b1	   ;
defparam cvlvds_inst0.CFG_ST0 			   = 1'b0	   ;
defparam cvlvds_inst0.CFG_TPUD_UD_BYP1	   = 1'b1	   ;
defparam cvlvds_inst0.CFG_ST1 			   = 1'b0	   ;
defparam cvlvds_inst0.CFG_DYN_LVDS_TX_EN  = 1'b0	   ;
defparam cvlvds_inst0.CFG_IO_BIAS_EN  	   = 1'b1	   ;
defparam cvlvds_inst0.CFG_VCM0P9_EN   	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_TX_PRE0     	   = 3'h0	   ;
defparam cvlvds_inst0.CFG_TX_PRE1     	   = 3'h0	   ;
defparam cvlvds_inst0.CFG_ODLC0_BYPASS	   = 1'b1	   ;
defparam cvlvds_inst0.CFG_ODLC0_MODE  	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_TPU_CAL_EN0 	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_TERM_DIFF_EN	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_TD_UD_BYP   	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_TPU_CAL_EN1 	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_RX_LP_EN_0  	   = 1'b0   	   ; //DDR need 0 ?
defparam cvlvds_inst0.CFG_RX_LP_EN_1  	   = 1'b0  	   ;
defparam cvlvds_inst0.CFG_IDLC0_MODE  	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_VREF_EN     	   = 1'b1	   ; //DDR need 1 ?
defparam cvlvds_inst0.CFG_LVDS_TX_EN  	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_TPU_0   		   = 7'b0101111;
defparam cvlvds_inst0.CFG_TD      		   = 5'h0	   ;
defparam cvlvds_inst0.CFG_TPU_1   		   = 7'b0101111;
defparam cvlvds_inst0.CFG_NS_LV_0 		   = 3   	   ;
defparam cvlvds_inst0.CFG_SMIT_TUNE_EN    = 1'b0	   ;
defparam cvlvds_inst0.CFG_LDR 		       = 2'h1	   ;
defparam cvlvds_inst0.CFG_NS_LV_1 		   = 3  	   ;
defparam cvlvds_inst0.CFG_INTREF_VREF_EN  = 1'b0	   ; //DDR need 1 ?
defparam cvlvds_inst0.CFG_RX_DIFF_EN	   = 1'b0	   ;
defparam cvlvds_inst0.CFG_INTREF_VREF_SEL = 4'h0	   ; //DDR need ?
defparam cvlvds_inst0.CFG_DYN_LP0 		   = 1'b0	   ;
defparam cvlvds_inst0.CFG_DYN_LP1 		   = 1'b0	   ;

`endif

endmodule