transcript on
if {[file exists rtl_work]} {
    vdel -lib rtl_work -all
}
vlib rtl_work
vmap -c work rtl_work
vlog -vlog01compat -work work D:/soft_installer/fpga/agate/2023-10-09-win64-rel-1906/data/lib/p2_sim.v
vlog -sv -work work ../src/imports/ddr3_model.sv
vlog -vlog01compat -work work  "../src/imports/VexRiscv.v" "../src/p0_src/ddrio_dqs.v" "../src/p0_src/pll_v1.v" "../src/p0_src/PLLE2_ADV_HME.v" "../src/p0_src/FDCE_HME.v" "../src/p0_src/FDPE_HME.v" "../src/imports/digilent_arty.v" "../src/sdrio_x1.v" "../src/ddrio_x2.v" "../src/p0_src/ologic_aligner.v" "../src/imports/example_top.v" "../src/imports/sim_tb_top.v" "../src/imports/wiredly.v" +initreg+0 +define+SIM
vsim -voptargs=+acc work.sim_tb_top 
do wave.do
run 1us