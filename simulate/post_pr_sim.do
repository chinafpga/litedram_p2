transcript on
if {[file exists post_timing_work]} {
    vdel -lib post_timing_work -all
}
vlib post_timing_work
vmap -c work post_timing_work
vlog -64 -incr +initmem+0+initreg+0 -work work "D:/soft_installer/fpga/agate/2023-10-09-win64-rel-1906/data/catalog/P2/primitive_cfg.v"
#vlog -vlog01compat -work work D:/soft_installer/fpga/agate/2023-10-09-win64-rel-1906/data/lib/p2_sim.v
vlog -64 -incr -sv -work work ../src/imports/ddr3_model.sv
vlog -64 -incr -work work  "D:/work/fpga/cme/demo/P0/wuyuxin/litedram_p0/outputs/digilent_arty.arv" "D:/work/fpga/cme/demo/P0/wuyuxin/litedram_p0/src/imports/example_top.v" "D:/work/fpga/cme/demo/P0/wuyuxin/litedram_p0/src/imports/sim_tb_top.v" "D:/work/fpga/cme/demo/P0/wuyuxin/litedram_p0/src/imports/wiredly.v" +initreg+0
#vsim -voptargs=+acc work.sim_tb_top -sdftyp /sim_tb_top/u_ip_top/ddr_inst=../outputs/digilent_arty_router.sdf
vsim -voptargs=+acc work.sim_tb_top
do wave.do
run 1us