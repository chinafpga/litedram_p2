C:\\questasim64_2021.1\\win64\\vlib questa_lib/work
C:\\questasim64_2021.1\\win64\\vlib questa_lib/msim

C:\\questasim64_2021.1\\win64\\vlib questa_lib/msim/xil_defaultlib

C:\\questasim64_2021.1\\win64\\vmap xil_defaultlib questa_lib/msim/xil_defaultlib

C:\\questasim64_2021.1\\win64\\vlog  -incr -work xil_defaultlib  "+incdir+../src/imports/" \
"../src/imports/VexRiscv.v" \
"../src/imports/digilent_arty.v" \
"../src/imports/example_top.v" \
"../src/imports/wiredly.v" \
+define+SIM \

C:\\questasim64_2021.1\\win64\\vlog  -incr -sv -work xil_defaultlib  "+incdir+../src/imports/" \
"../src/imports/ddr3_model.sv" \

C:\\questasim64_2021.1\\win64\\vlog  -incr -work xil_defaultlib  "+incdir+../src/imports/" \
"../src/imports/sim_tb_top.v" \

# compile glbl module
C:\\questasim64_2021.1\\win64\\vlog -work xil_defaultlib "glbl.v"

C:\\questasim64_2021.1\\win64\\vopt +acc=npr -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -work xil_defaultlib xil_defaultlib.sim_tb_top xil_defaultlib.glbl -o sim_tb_top_opt

vsim -lib xil_defaultlib sim_tb_top_opt

set NumericStdNoWarnings 1
set StdArithNoWarnings 1

do {sim_tb_top_wave.do}

view wave
view structure
view signals

do {sim_tb_top.udo}

run 10us