onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/clk100
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/cpu_reset
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/eth_ref_clk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/serial_tx
add wave -noupdate -radix ascii /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_tx_sink_payload_data
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/serial_rx
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_a
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_ba
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_ras_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_cas_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_we_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_dm
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_oe_delay_tappeddelayline_tappeddelayline1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip02
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip03
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_dq
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_dqs_p
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_dqs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_clk_p
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_clk_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_cke
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_odt
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_reset_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/test
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/user_led0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/user_led1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/user_led2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/user_led3
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/sys4x_clk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/sys_clk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_i_delayed0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_rst_storage
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/sys_rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip03
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_i_delayed0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_i_nodelay0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_dq
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_t0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_o_nodelay0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_i_nodelay0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_address
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_bank
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_cas_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_ras_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_we_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_cke
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_odt
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_reset_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_act_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_wrdata_mask
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_rddata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_address
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_bank
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_cas_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_ras_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_we_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_cke
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_odt
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_reset_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_act_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_wrdata_mask
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_rddata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_address
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_bank
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_cas_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_ras_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_we_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_cke
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_odt
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_reset_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_act_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_wrdata_mask
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_rddata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_address
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_bank
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_cas_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_ras_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_we_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_cke
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_odt
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_reset_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_act_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_wrdata_mask
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_rddata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/clk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/reset
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/externalResetVector
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/timerInterrupt
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/softwareInterrupt
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/externalInterruptArray
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_CYC
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_STB
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_ACK
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_WE
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_ADR
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_DAT_MISO
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_DAT_MOSI
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_SEL
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_ERR
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_CTI
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/iBusWishbone_BTE
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_CYC
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_STB
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_ACK
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_WE
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_ADR
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_DAT_MISO
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_DAT_MOSI
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_SEL
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_ERR
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_CTI
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/VexRiscv/dBusWishbone_BTE
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/shiftout0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/shiftout1
add wave -noupdate -expand /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/dataout
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/sout
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/datain
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/fclk0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/fclk1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/geclk0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/geclk1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/gsclk0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/gsclk1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/setn
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/shiftin0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/shiftin1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/update
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/CFG_DDR_IN_NREG
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/CFG_DDR_IN_NREG_DFF
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/CFG_DDR_IN_PREG
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/CFG_DDR_IN_PREG_DFF
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/CFG_FASTIN
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/CFG_SLAVE_IN
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/CFG_GEAR_IN
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/do_d
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/do_dd
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/de_d
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/de_dd
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/de_in
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/do_in
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/de_pre
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/do_pre
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q7
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q5
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q3
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q6
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q4
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/slv1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/slv0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q7s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q5s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q3s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q1s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q6s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q4s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q2s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/q0s
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p4
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p6
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p3
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p5
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/u_data_p/Iilg/p7
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/geclk_ol
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/gsclk_ol
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/geclk_il
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/gsclk_il
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/align_rst_ol
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/cken
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/align_il
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/align_ol
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/phy_odt_ctrl_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/phy_odt_ctrl_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/t_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/d_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/q_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/t_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/d_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/q_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/dq_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/dq_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/geclk_ol_buf_o
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/geclk_il_buf_o
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_update_ol_d0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_update_b_ol_d0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_update_il_d0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_update_b_il_d0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/ODT_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_sout0_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_sout1_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftin0_il_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftin0_ol_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftin1_il_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftin1_ol_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftin_oen_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftout_oen_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftout0_il_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftout0_ol_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftout1_il_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_shiftout1_ol_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/sout_data_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_sout_data_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/id_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/f_oen_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/f_od_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/id_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/f_oen_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/f_od_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_det_out_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_det_enb_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_det_rst0_o_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/internal_det_rst1_o_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_29/ODT_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_phaseinjector0_rddata_status
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_phaseinjector1_rddata_status
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_phaseinjector2_rddata_status
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_phaseinjector3_rddata_status
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p0_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p0_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p1_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p1_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p2_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p2_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p3_rddata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p3_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_port_rdata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_port_rdata_payload_data
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_interface_dat_r
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_interface_ack
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_rddata_valid
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_rddata_en_tappeddelayline7
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_rddata_en_tappeddelayline8
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_wlevel_en_storage
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dq_oe
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip03
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip13
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip21
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip31
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip41
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip51
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip61
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip71
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip81
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip91
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip101
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip111
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip121
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip131
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip141
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_bitslip151
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dataout
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/shiftout0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/shiftout1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/fclk0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/fclk1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/geclk0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/geclk1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/gsclk0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/gsclk1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/setn
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/shiftin0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/shiftin1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/update
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/datain
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/CFG_DDR_OUT_REG
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/CFG_FOUT_SEL
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/CFG_GEAR_OUT
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/CFG_DDR_OUT
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/CFG_FCLK_INV
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs7
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs6
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs5
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs4
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs3
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/qs0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s3
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s4
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s5
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s6
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/s7
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dp_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dn_0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dp_2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dn_2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dp_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dn_1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dp_3
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dn_3
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/dp_out
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/pn_sel
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_n/Iolg/pn_sel_finv
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/CFG_UP_SEL
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/CFG_GSCLK_UPO_EN
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/CFG_GECLK_UPO_EN
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/CFG_RST_EN
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/align_rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/align_user
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/geclk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/gsclk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/sysclk_adjust_retval
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/update
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/update_b
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/w_geclk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/w_gsclk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/w_rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/usermode
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/geclk_buf
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/usermode_b
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/align2
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/align1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/align0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net0106
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/detect_rstb
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net070
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/cnt_rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net60
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net61
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net063
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net072
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net094
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net079
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net082
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net096
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net062
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net061
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/net060
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_18/u_data_ol_update_d0/Q
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_ext_dfi_p0_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_ext_dfi_p1_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_ext_dfi_p2_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_ext_dfi_p3_wrdata_en
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_slave_p0_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_slave_p1_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_slave_p2_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_slave_p3_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p0_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p1_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p2_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_basesoc_sdram_csr_dfi_p3_wrdata
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi0_wrdata0_r
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi1_wrdata0_r
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi2_wrdata0_r
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi3_wrdata0_r
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi0_wrdata0_re
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi1_wrdata0_re
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi2_wrdata0_re
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/builder_csr_bankarray_csrbank3_dfii_pi3_wrdata0_re
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p3_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p2_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p1_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/main_a7ddrphy_dfi_p0_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/dataout
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/CFG_DDR_OUT
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/CFG_DDR_OUT_REG
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/CFG_FCLK_INV
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/CFG_FOUT_SEL
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/fclk0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/fclk1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/rst
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/set_
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/datain
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/dp0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/dp1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/dn0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/dn1
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/pnsel
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/OSERDESE2_2/ioc_cmos_inst0/Iolg/ddro
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {1719906835 ps} 0} {{Cursor 2} {8847976416 ps} 0} {{Cursor 3} {724694 ps} 0}
quietly wave cursor active 3
configure wave -namecolwidth 528
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {88391177 ps}
