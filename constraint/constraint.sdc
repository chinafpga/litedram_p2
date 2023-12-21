create_clock -period 10 -name {clk} [get_ports {clk}]
create_clock -period 5 -name {PLLE2_ADV_pll_v1_inst_pll_u0/CO0} [get_pins {PLLE2_ADV_pll_v1_inst_pll_u0/CO0}]
create_clock -period 10 -name {PLLE2_ADV_pll_v1_inst_pll_u0/CO1} [get_pins {PLLE2_ADV_pll_v1_inst_pll_u0/CO1}]
create_clock -period 2.5 -name {PLLE2_ADV_pll_v1_inst_pll_u0/CO2} [get_pins {PLLE2_ADV_pll_v1_inst_pll_u0/CO2}]
create_clock -period 2.5 -name {PLLE2_ADV_pll_v1_inst_pll_u0/CO3} [get_pins {PLLE2_ADV_pll_v1_inst_pll_u0/CO3}]
create_clock -period 3 -name {auto_0_f_id[0]_clk} [get_pins {io_clk_inst/f_id[0]}]
create_clock -period 3 -name {auto_0_out_clk} [get_pins {debugware_v2_1_u_gbuf_update/out}]
create_clock -period 3 -name {auto_1_out_clk} [get_pins {debugware_v2_1_u_tap_u_gbuf_tck/out}]
create_clock -period 3 -name {auto_0_out_clk} [get_pins {u_debugware_v2_2_u_gbuf_update/out}]
create_clock -period 3 -name {auto_1_out_clk} [get_pins {u_debugware_v2_2_u_tap_u_gbuf_tck/out}]

create_clock -period 3 -name {auto_0_clk100_clk} [get_pins {clk100}] -comment {auto_created}

create_clock -period 3 -name {auto_0_jtag_fp_update_clk} [get_pins {debugware_v2_2_inst_u_tap_genblk1_u_jtag/jtag_fp_update}] -comment {auto_created}
create_clock -period 3 -name {auto_1_jtag_fp_drck_clk} [get_pins {debugware_v2_2_inst_u_tap_genblk1_u_jtag/jtag_fp_drck}] -comment {auto_created}
