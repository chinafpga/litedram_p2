//*****************************************************************************

// (c) Copyright 2009 - 2013 Xilinx, Inc. All rights reserved.

//

// This file contains confidential and proprietary information

// of Xilinx, Inc. and is protected under U.S. and

// international copyright and other intellectual property

// laws.

//

// DISCLAIMER

// This disclaimer is not a license and does not grant any

// rights to the materials distributed herewith. Except as

// otherwise provided in a valid license issued to you by

// Xilinx, and to the maximum extent permitted by applicable

// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND

// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES

// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING

// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-

// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and

// (2) Xilinx shall not be liable (whether in contract or tort,

// including negligence, or under any other theory of

// liability) for any loss or damage of any kind or nature

// related to, arising under or in connection with these

// materials, including for any direct, or any indirect,

// special, incidental, or consequential loss or damage

// (including loss of data, profits, goodwill, or any type of

// loss or damage suffered as a result of any action brought

// by a third party) even if such damage or loss was

// reasonably foreseeable or Xilinx had been advised of the

// possibility of the same.

//

// CRITICAL APPLICATIONS

// Xilinx products are not designed or intended to be fail-

// safe, or for use in any application requiring fail-safe

// performance, such as life-support or safety devices or

// systems, Class III medical devices, nuclear facilities,

// applications related to the deployment of airbags, or any

// other applications that could lead to death, personal

// injury, or severe property or environmental damage

// (individually and collectively, "Critical

// Applications"). Customer assumes the sole risk and

// liability of any use of Xilinx products in Critical

// Applications, subject only to applicable laws and

// regulations governing limitations on product liability.

//

// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS

// PART OF THIS FILE AT ALL TIMES.

//

//*****************************************************************************

//   ____  ____

//  /   /\/   /

// /___/  \  /    Vendor             : Xilinx

// \   \   \/     Version            : 4.2

//  \   \         Application        : MIG

//  /   /         Filename           : example_top.v

// /___/   /\     Date Last Modified : $Date: 2011/06/02 08:35:03 $

// \   \  /  \    Date Created       : Tue Sept 21 2010

//  \___\/\___\

//

// Device           : 7 Series

// Design Name      : DDR3 SDRAM

// Purpose          :

//   Top-level  module. This module serves as an example,

//   and allows the user to synthesize a self-contained design,

//   which they can be used to test their hardware.

//   In addition to the memory controller, the module instantiates:

//     1. Synthesizable testbench - used to model user's backend logic

//        and generate different traffic patterns

// Reference        :

// Revision History :

//*****************************************************************************



//`define SKIP_CALIB

`timescale 1ns/1ps

`default_nettype wire

module example_top #

  (



   //***************************************************************************

   // Traffic Gen related parameters

   //***************************************************************************

   parameter BEGIN_ADDRESS         = 32'h00000000,

   parameter END_ADDRESS           = 32'h00ffffff,

   parameter PRBS_EADDR_MASK_POS   = 32'hff000000,

   parameter ENFORCE_RD_WR         = 0,

   parameter ENFORCE_RD_WR_CMD     = 8'h11,

   parameter ENFORCE_RD_WR_PATTERN = 3'b000,

	parameter C_EN_WRAP_TRANS       = 1,
   //parameter C_EN_WRAP_TRANS       = 0,

   parameter C_AXI_NBURST_TEST     = 0,



   //***************************************************************************

   // The following parameters refer to width of various ports

   //***************************************************************************

   parameter CK_WIDTH              = 1,

                                     // # of CK/CK# outputs to memory.

   parameter nCS_PER_RANK          = 1,

                                     // # of unique CS outputs per rank for phy

   parameter CKE_WIDTH             = 1,

                                     // # of CKE outputs to memory.

   parameter DM_WIDTH              = 2,

                                     // # of DM (data mask)

   parameter ODT_WIDTH             = 1,

                                     // # of ODT outputs to memory.

   parameter BANK_WIDTH            = 3,

                                     // # of memory Bank Address bits.

   parameter COL_WIDTH             = 10,

                                     // # of memory Column Address bits.

   parameter CS_WIDTH              = 1,

                                     // # of unique CS outputs to memory.

   parameter DQ_WIDTH              = 16,

                                     // # of DQ (data)

   parameter DQS_WIDTH             = 2,

   parameter DQS_CNT_WIDTH         = 1,

                                     // = ceil(log2(DQS_WIDTH))

   parameter DRAM_WIDTH            = 8,

                                     // # of DQ per DQS

   parameter ECC                   = "OFF",

   parameter ECC_TEST              = "OFF",

   //parameter nBANK_MACHS           = 4,

   parameter nBANK_MACHS           = 8,

   parameter RANKS                 = 1,

                                     // # of Ranks.

   parameter ROW_WIDTH             = 14,

                                     // # of memory Row Address bits.

   parameter ADDR_WIDTH            = 28,

                                     // # = RANK_WIDTH + BANK_WIDTH

                                     //     + ROW_WIDTH + COL_WIDTH;

                                     // Chip Select is always tied to low for

                                     // single rank devices



   //***************************************************************************

   // The following parameters are mode register settings

   //***************************************************************************

   parameter BURST_MODE            = "8",

                                     // DDR3 SDRAM:

                                     // Burst Length (Mode Register 0).

                                     // # = "8", "4", "OTF".

                                     // DDR2 SDRAM:

                                     // Burst Length (Mode Register).

                                     // # = "8", "4".



   

   //***************************************************************************

   // The following parameters are multiplier and divisor factors for PLLE2.

   // Based on the selected design frequency these parameters vary.

   //***************************************************************************

   parameter CLKIN_PERIOD          = 5000,

                                     // Input Clock Period

   parameter CLKFBOUT_MULT         = 4,

                                     // write PLL VCO multiplier

   parameter DIVCLK_DIVIDE         = 1,

                                     // write PLL VCO divisor

   parameter CLKOUT0_PHASE         = 0.0,

                                     // Phase for PLL output clock (CLKOUT0)

   parameter CLKOUT0_DIVIDE        = 1,

                                     // VCO output divisor for PLL output clock (CLKOUT0)

   parameter CLKOUT1_DIVIDE        = 2,

                                     // VCO output divisor for PLL output clock (CLKOUT1)

   parameter CLKOUT2_DIVIDE        = 32,

                                     // VCO output divisor for PLL output clock (CLKOUT2)

   parameter CLKOUT3_DIVIDE        = 8,

                                     // VCO output divisor for PLL output clock (CLKOUT3)

   parameter MMCM_VCO              = 800,

                                     // Max Freq (MHz) of MMCM VCO

   parameter MMCM_MULT_F           = 8,

                                     // write MMCM VCO multiplier

   parameter MMCM_DIVCLK_DIVIDE    = 1,

                                     // write MMCM VCO divisor



   //***************************************************************************

   // Simulation parameters

   //***************************************************************************

   parameter SIMULATION            = "FALSE",

                                     // Should be TRUE during design simulations and

                                     // FALSE during implementations



   //***************************************************************************

   // IODELAY and PHY related parameters

   //***************************************************************************

   parameter TCQ                   = 100,

   

   parameter DRAM_TYPE             = "DDR3",



   

   //***************************************************************************

   // System clock frequency parameters

   //***************************************************************************

   parameter nCK_PER_CLK           = 4,

                                     // # of memory CKs per fabric CLK



   

   //***************************************************************************

   // AXI4 Shim parameters

   //***************************************************************************

   parameter C_S_AXI_ID_WIDTH              = 4,

                                             // Width of all master and slave ID signals.

                                             // # = >= 1.

   parameter C_S_AXI_ADDR_WIDTH            = 28,

                                             // Width of S_AXI_AWADDR, S_AXI_ARADDR, M_AXI_AWADDR and

                                             // M_AXI_ARADDR for all SI/MI slots.

                                             // # = 32.

   parameter C_S_AXI_DATA_WIDTH            = 128,

                                             // Width of WDATA and RDATA on SI slot.

                                             // Must be <= APP_DATA_WIDTH.

                                             // # = 32, 64, 128, 256.

   parameter C_S_AXI_SUPPORTS_NARROW_BURST = 0,

                                             // Indicates whether to instatiate upsizer

                                             // Range: 0, 1

      



   //***************************************************************************

   // Debug parameters

   //***************************************************************************

   parameter DEBUG_PORT            = "OFF",

                                     // # = "ON" Enable debug signals/controls.

                                     //   = "OFF" Disable debug signals/controls.

      

   parameter RST_ACT_LOW           = 1

                                     // =1 for active low reset,

                                     // =0 for active high.

   )

  (



   // Inouts

   inout [15:0]                         ddr3_dq,

   inout [1:0]                        ddr3_dqs_n,

   inout [1:0]                        ddr3_dqs_p,



   // Outputs

   output [13:0]                       ddr3_addr,

   output [2:0]                      ddr3_ba,

   output                                       ddr3_ras_n,

   output                                       ddr3_cas_n,

   output                                       ddr3_we_n,

   output                                       ddr3_reset_n,

   output [0:0]                        ddr3_ck_p,

   output [0:0]                        ddr3_ck_n,

   output [0:0]                       ddr3_cke,

   

   output [0:0]           ddr3_cs_n,

   

   output [1:0]                        ddr3_dm,

   

   output [0:0]                       ddr3_odt,

   



   // Inputs

   

   // Single-ended system clock

   input                                        sys_clk_i,

   



   output                                       tg_compare_error,

   output                                       init_calib_complete,

   

      



   // System reset - Default polarity of sys_rst pin is Active Low.

   // System reset polarity will change based on the option 

   // selected in GUI.

   input                                        sys_rst

   );



function integer clogb2 (input integer size);

    begin

      size = size - 1;

      for (clogb2=1; size>1; clogb2=clogb2+1)

        size = size >> 1;

    end

  endfunction // clogb2



  function integer STR_TO_INT;

    input [7:0] in;

    begin

      if(in == "8")

        STR_TO_INT = 8;

      else if(in == "4")

        STR_TO_INT = 4;

      else

        STR_TO_INT = 0;

    end

  endfunction





  localparam DATA_WIDTH            = 16;

  localparam RANK_WIDTH = clogb2(RANKS);

  localparam PAYLOAD_WIDTH         = (ECC_TEST == "OFF") ? DATA_WIDTH : DQ_WIDTH;

  localparam BURST_LENGTH          = STR_TO_INT(BURST_MODE);

  localparam APP_DATA_WIDTH        = 2 * nCK_PER_CLK * PAYLOAD_WIDTH;

  localparam APP_MASK_WIDTH        = APP_DATA_WIDTH / 8;



  //***************************************************************************

  // Traffic Gen related parameters (derived)

  //***************************************************************************

  localparam  TG_ADDR_WIDTH = ((CS_WIDTH == 1) ? 0 : RANK_WIDTH)

                                 + BANK_WIDTH + ROW_WIDTH + COL_WIDTH;

  localparam MASK_SIZE             = DATA_WIDTH/8;

  localparam DBG_WR_STS_WIDTH      = 40;

  localparam DBG_RD_STS_WIDTH      = 40;

      



  // Wire declarations

      

  wire                              clk;

  wire                              rst;

  wire                              mmcm_locked;

  reg                               aresetn;

  wire                              app_sr_active;

  wire                              app_ref_ack;

  wire                              app_zq_ack;

  wire                              app_rd_data_valid;

  wire [APP_DATA_WIDTH-1:0]         app_rd_data;



  wire                              mem_pattern_init_done;



  wire                              cmd_err;

  wire                              data_msmatch_err;

  wire                              write_err;

  wire                              read_err;

  wire                              test_cmptd;

  wire                              write_cmptd;

  wire                              read_cmptd;

  wire                              cmptd_one_wr_rd;



  // Slave Interface Write Address Ports

  wire [C_S_AXI_ID_WIDTH-1:0]       s_axi_awid;

  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr;

  wire [7:0]                        s_axi_awlen;

  wire [2:0]                        s_axi_awsize;

  wire [1:0]                        s_axi_awburst;

  wire [0:0]                        s_axi_awlock;

  wire [3:0]                        s_axi_awcache;

  wire [2:0]                        s_axi_awprot;

  wire                              s_axi_awvalid;

  wire                              s_axi_awready;

   // Slave Interface Write Data Ports

  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata;

  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]   s_axi_wstrb;

  wire                              s_axi_wlast;

  wire                              s_axi_wvalid;

  wire                              s_axi_wready;

   // Slave Interface Write Response Ports

  wire                              s_axi_bready;

  wire [C_S_AXI_ID_WIDTH-1:0]       s_axi_bid;

  wire [1:0]                        s_axi_bresp;

  wire                              s_axi_bvalid;

   // Slave Interface Read Address Ports

  wire [C_S_AXI_ID_WIDTH-1:0]       s_axi_arid;

  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr;

  wire [7:0]                        s_axi_arlen;

  wire [2:0]                        s_axi_arsize;

  wire [1:0]                        s_axi_arburst;

  wire [0:0]                        s_axi_arlock;

  wire [3:0]                        s_axi_arcache;

  wire [2:0]                        s_axi_arprot;

  wire                              s_axi_arvalid;

  wire                              s_axi_arready;

   // Slave Interface Read Data Ports

  wire                              s_axi_rready;

  wire [C_S_AXI_ID_WIDTH-1:0]       s_axi_rid;

  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata;

  wire [1:0]                        s_axi_rresp;

  wire                              s_axi_rlast;

  wire                              s_axi_rvalid;



  wire                              cmp_data_valid;

  wire [C_S_AXI_DATA_WIDTH-1:0]      cmp_data;     // Compare data

  wire [C_S_AXI_DATA_WIDTH-1:0]      rdata_cmp;      // Read data



  wire                              dbg_wr_sts_vld;

  wire [DBG_WR_STS_WIDTH-1:0]       dbg_wr_sts;

  wire                              dbg_rd_sts_vld;

  wire [DBG_RD_STS_WIDTH-1:0]       dbg_rd_sts;

  wire [11:0]                           device_temp;

  

`ifdef SKIP_CALIB

  // skip calibration wires

  wire                          calib_tap_req;

  reg                           calib_tap_load;

  reg [6:0]                     calib_tap_addr;

  reg [7:0]                     calib_tap_val;

  reg                           calib_tap_load_done;

`endif

      

  



//***************************************************************************

// Start of User Design top instance

//***************************************************************************

// The User design is instantiated below. The memory interface ports are

// connected to the top-level and the application interface ports are

// connected to the traffic generator module. This provides a reference

// for connecting the memory controller to system.

//***************************************************************************




wire init_done;
wire init_error;
wire user_clk;
wire user_rst;

//***************************************************************************

// The traffic generation module instantiated below drives traffic (patterns)

// on the application interface of the memory controller

//***************************************************************************
wire uart_tx;
wire uart_rx;

assign uart_rx= 1'b0;

digilent_arty ddr_inst(
    .serial_tx                  (uart_tx),
    .serial_rx                  (uart_rx),
    .clk100                   (sys_clk_i),
    .cpu_reset                (sys_rst), //cpu_reset and sys_rst: low level active
    //.pll_locked               (),
    .ddram_a                  (ddr3_addr),
    .ddram_ba                 (ddr3_ba),
    .ddram_ras_n              (ddr3_ras_n),
    .ddram_cas_n              (ddr3_cas_n),
    .ddram_we_n               (ddr3_we_n),
    .ddram_cs_n               (ddr3_cs_n),
    .ddram_dm                 (ddr3_dm),
    .ddram_dq                 (ddr3_dq),
    .ddram_dqs_p              (ddr3_dqs_p),
    .ddram_dqs_n              (ddr3_dqs_n),
    .ddram_clk_p              (ddr3_ck_p),
    .ddram_clk_n              (ddr3_ck_n),
    .ddram_cke                (ddr3_cke),
    .ddram_odt                (ddr3_odt),
    .ddram_reset_n            (ddr3_reset_n),
    .user_led0                (init_done),
    .user_led1                (init_error),
    .user_led2                (user_clk),
    .user_led3                (user_rst)/*,
    .user_port_wishbone_0_adr     (axi2wb_adr  ),
    .user_port_wishbone_0_dat_w   (axi2wb_dat_w),
    .user_port_wishbone_0_dat_r   (axi2wb_dat_r),
    .user_port_wishbone_0_sel     (axi2wb_sel  ),
    .user_port_wishbone_0_cyc     (axi2wb_cyc  ),
    .user_port_wishbone_0_stb     (axi2wb_stb  ),
    .user_port_wishbone_0_ack     (axi2wb_ack  ),
    .user_port_wishbone_0_we      (axi2wb_we   ),
    .user_port_wishbone_0_err     (axi2wb_err  )*/
);

endmodule




