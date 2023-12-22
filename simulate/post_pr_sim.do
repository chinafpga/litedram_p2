transcript on
if {[file exists post_timing_work]} {
    vdel -lib post_timing_work -all
}
vlib post_timing_work
vmap -c work post_timing_work
vlog -64 -incr -work work "../src/sim_lib/primitive_cfg.v"
#vlog -vlog01compat -work work ../outputs/p2_sim.v
vlog -64 -incr -sv -work work ../src/imports/ddr3_model.sv
vlog -64 -incr -work work  "../outputs/digilent_arty.arv" "../src/imports/example_top.v" "../src/imports/sim_tb_top.v" "../src/imports/wiredly.v"
vsim -voptargs=+acc work.sim_tb_top -sdftyp /sim_tb_top/u_ip_top/ddr_inst=../outputs/digilent_arty_router.sdf
#vsim -voptargs=+acc work.sim_tb_top
do post_syn_wave.do
run 1us