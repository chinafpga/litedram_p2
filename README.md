# litedram_p2
P2 RTL simulation flow
1. Change line 20 : "set bin_path=D:\\APP\\questasim\\win64" in "questa\compile.bat" file to your local modelsim or questasim install directory;Change "set bin_path=D:\\APP\\questasim\\win64" in "questa\elaborate.bat" and  "questa\simulate.bat" files to your local modelsim or questasim install directory in the same way.
2. Enter "questa" sub-directory and Double click "compile.bat".
3. Enter "questa" sub-directory and Double click "elaborate.bat".
4. Enter "questa" sub-directory and Double click "simulate.bat".
5. P2 rtl simulation could run successfully now!
6. Simulation pass wave as following：
   ![1703131889798](https://github.com/chinafpga/litedram_p2/assets/522003/f369f022-d456-4d3c-853d-559a2420c1f3)

7. To speedup simulation, change litex c code is necessary. Changed c files is as following:
   litex/soc/software/bios/main.c
   litex/soc/software/libbase/memtest.c
   litex/soc/software/liblitedram/sdram.c

   Above 3 change files is in software 
