transcript on
if {[file exists post_syn_work]} {
    vdel -lib post_syn_work -all
}
vlib post_syn_work
vmap -c work post_syn_work
vlog -64 -incr +initmem+0 -timescale 1ns/1ns -work work D:/soft_installer/fpga/agate/2023-10-09-win64-rel-1906/data/lib/p2_sim.v
vlog -64 -incr -sv -work work ../src/imports/ddr3_model.sv
vlog -64 -incr -work work  "../outputs/digilent_arty.amv" "../src/imports/example_top.v" "../src/imports/sim_tb_top.v" "../src/imports/wiredly.v" +initreg+0
vsim -voptargs=+acc work.sim_tb_top 
do wave.do
run 1us