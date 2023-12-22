onerror {resume}
quietly virtual signal -install {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k } { /sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[7:0]} tx_uart_byte
quietly virtual signal -install /sim_tb_top/u_ip_top/ddr_inst { (context /sim_tb_top/u_ip_top/ddr_inst )&{\VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[31]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[30]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[29]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[28]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[27]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[26]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[25]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[24]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[23]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[22]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[21]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[20]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[19]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[18]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[17]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[16]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[15]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[14]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[13]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[12]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[11]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[10]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[9]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[8]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[7]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[6]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[5]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[4]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[3]|qx_net , \VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[2]|qx_net }} VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address
quietly WaveActivateNextPane {} 0
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/clk100
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/cpu_reset
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/eth_ref_clk
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/serial_tx
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/serial_rx
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_a
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_ba
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_ras_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_cas_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_we_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_cs_n
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_dm
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
add wave -noupdate /sim_tb_top/u_ip_top/ddr_inst/ddram_dq
add wave -noupdate -radix ascii -childformat {{{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[7]} -radix ascii} {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[6]} -radix ascii} {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[5]} -radix ascii} {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[4]} -radix ascii} {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[3]} -radix ascii} {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[2]} -radix ascii} {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[1]} -radix ascii} {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte[0]} -radix ascii}} -subitemconfig {{/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[7]} {-radix ascii} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[6]} {-radix ascii} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[5]} {-radix ascii} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[4]} {-radix ascii} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[3]} {-radix ascii} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[2]} {-radix ascii} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[1]} {-radix ascii} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa[0]} {-radix ascii}} {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /tx_uart_byte}
add wave -noupdate {/sim_tb_top/u_ip_top/ddr_inst/\storage_d0|u0_bram9k /outa}
add wave -noupdate -expand /sim_tb_top/u_ip_top/ddr_inst/VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address
add wave -noupdate {/sim_tb_top/u_ip_top/ddr_inst/\VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[2] /clk_in}
add wave -noupdate {/sim_tb_top/u_ip_top/ddr_inst/\VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[2] /reset}
add wave -noupdate {/sim_tb_top/u_ip_top/ddr_inst/\VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[2] /reg_enable}
add wave -noupdate {/sim_tb_top/u_ip_top/ddr_inst/\VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[2] /qx}
add wave -noupdate {/sim_tb_top/u_ip_top/ddr_inst/\VexRiscv_dataCache_1_io_mem_cmd_s2mPipe_rData_address_reg[2] /di}
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {3472560218 ps} 0} {{Cursor 2} {6563781783 ps} 0} {{Cursor 3} {8632073 ps} 0}
quietly wave cursor active 3
configure wave -namecolwidth 709
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
WaveRestoreZoom {8346604 ps} {8917542 ps}
