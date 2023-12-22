transcript on
if {[file exists rtl_work]} {
    vdel -lib rtl_work -all
}
vlib rtl_work
vmap -c work rtl_work
vlog -incr -mfcu -vlog01compat -work work "../src/sim_lib/p2_sim.v"
vlog -incr -mfcu -sv -work work ../src/imports/ddr3_model.sv
vlog -incr -mfcu -vlog01compat -work work  "../src/imports/VexRiscv.v" "../src/p0_src/ddrio_dqs.v" "../src/p0_src/pll_v1.v" "../src/p0_src/PLLE2_ADV_HME.v" "../src/p0_src/FDCE_HME.v" "../src/p0_src/FDPE_HME.v" "../src/imports/digilent_arty.v" "../src/sdrio_x1.v" "../src/ddrio_x2.v" "../src/p0_src/ologic_aligner.v" "../src/imports/example_top.v" "../src/imports/sim_tb_top.v" "../src/imports/wiredly.v" +initreg+0 +define+SIM
vopt +acc=np -suppress 10016 -L work -work work work.sim_tb_top -o sim_tb_top_opt
#vsim -voptargs=+acc=np work.sim_tb_top 
vsim -lib work sim_tb_top_opt
do wave.do
run 1us