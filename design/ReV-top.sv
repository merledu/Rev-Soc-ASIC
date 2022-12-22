//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.06.2022 11:22:01
// Design Name: 
// Module Name: ReV-top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Western Digital Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
`default_nettype	wire

`include "common_defines.vh"
module Rev_top ( input bit clk_in1 , input wire rst,input wire uart_rx_i, output pin0,pin1,pin2,pin3,pin4,output rst_led,programmed, output wire pwm_led,//output tx_o,input rx_i,//output rst_led,clk_led, output pin0,pin1,//output pwm_en,pwm_led,////*_l,input porst_l,
//output logic [31:0]                     trace_rv_i_insn_ip,
//output  logic TIMER_irq_o,
//output logic [31:0]                     trace_rv_i_address_ip,
//output logic                            trace_rv_i_valid_ip,
//output logic                            trace_rv_i_exception_ip,
//output logic [4:0]                      trace_rv_i_ecause_ip,
//output logic                            trace_rv_i_interrupt_ip,
//output logic [31:0]                     trace_rv_i_tval_ip
  output logic TIMER_irq_o,tx_o,input wire baud_change,in0,in1,in2,in3,in4);

    logic                            nmi_int;
    logic [31:0]                     trace_rv_i_insn_ip;
    //output  logic TIMER_irq_o,
    logic [31:0]                     trace_rv_i_address_ip;
    logic                            trace_rv_i_valid_ip;
    logic                            trace_rv_i_exception_ip;
    logic [4:0]                      trace_rv_i_ecause_ip;
    logic                            trace_rv_i_interrupt_ip;
    logic [31:0]                     trace_rv_i_tval_ip;
//    logic clk_temp;
    logic rx_i,clk_led,inst_fetch_led;
    
    logic reset;
    assign programmed = reset;

//    BUFIO buffer (.I,
//                  .O,
//                  .T);
//    logic TIMER_irq_o;
    logic core_clk;
    assign rst_led=rst;
    int                         cycleCnt;
    assign clk_led = cycleCnt > 4000000;

    logic        [31:0]         reset_vector;
    logic        [31:0]         nmi_vector;
    logic        [31:1]         jtag_id;
    
    assign reset_vector = `RV_RESET_VEC;
    assign nmi_vector = 32'hee000000;
    assign jtag_id={4'd1,16'd0,11'h45};

    logic        [31:0]         ic_haddr        ;
    logic rst_l;//Quick Access
    assign rst_l = ~rst;
    logic        [2:0]          ic_hburst       ;
    logic                       ic_hmastlock    ;
    logic        [3:0]          ic_hprot        ;
    logic        [2:0]          ic_hsize        ;
    logic        [1:0]          ic_htrans       ;
    logic                       ic_hwrite       ;
    logic        [63:0]         ic_hrdata       ;
    logic                       ic_hready       ;
    logic                       ic_hresp        ;

    logic        [31:0]         lsu_haddr      ;
    logic        [2:0]          lsu_hburst      ;
    logic                       lsu_hmastlock   ;
    logic        [3:0]          lsu_hprot       ;
    logic        [2:0]          lsu_hsize       ;
    logic        [1:0]          lsu_htrans      ;
    logic                       lsu_hwrite      ;
    logic        [63:0]         lsu_hrdata      ;
    logic        [63:0]         lsu_hwdata      ;
    logic                       lsu_hready      ;
    logic                       lsu_hresp        ;

    logic        [31:0]         sb_haddr        ;
    logic        [2:0]          sb_hburst       ;
    logic                       sb_hmastlock    ;
     logic       rx_dv_i;
     logic [7:0] rx_byte_i;
     
     logic we;
     logic [13:0] addr;
     logic [63:0] data;
//     logic uart_rx_i;
     
    prog_uart_controller controller(
          .clk_i       (core_clk),
          .rst_ni      (rst_l),
          .rx_dv_i     (rx_dv_i),
          .rx_byte_i   (rx_byte_i),
          .we_o        (we),
          .addr_o      (addr),
          .wdata_o     (data),
          .reset_o     (reset)
      );
    
      uart_receiver programmer (
       .i_Clock       (core_clk),
       .rst_ni        (rst_l),
       .i_Rx_Serial   (uart_rx_i),
       .CLKS_PER_BIT   ({10'd0,3'b101,(~baud_change),baud_change}),
       .o_Rx_DV       (rx_dv_i),
       .o_Rx_Byte     (rx_byte_i)
      );

//    function int get_iccm_bank(input[31:0] addr,  output int bank_idx);
//    `ifdef RV_DCCM_NUM_BANKS_2
//        bank_idx = int'(addr[`RV_DCCM_BITS-1:3]);
//        return int'( addr[2]);
//    `elsif RV_ICCM_NUM_BANKS_4
//        bank_idx = int'(addr[`RV_ICCM_BITS-1:4]);
//        return int'(addr[3:2]);
//    `elsif RV_ICCM_NUM_BANKS_8
//        bank_idx = int'(addr[`RV_ICCM_BITS-1:5]);
//        return int'( addr[4:2]);
//    `elsif RV_ICCM_NUM_BANKS_16
//        bank_idx = int'(addr[`RV_ICCM_BITS-1:6]);
//        return int'( addr[5:2]);
//    `endif
//    endfunction
    
//    function[6:0] riscv_ecc32(input[31:0] data);
//    reg[6:0] synd;
//    synd[0] = ^(data & 32'h56aa_ad5b);
//    synd[1] = ^(data & 32'h9b33_366d);
//    synd[2] = ^(data & 32'he3c3_c78e);
//    synd[3] = ^(data & 32'h03fc_07f0);
//    synd[4] = ^(data & 32'h03ff_f800);
//    synd[5] = ^(data & 32'hfc00_0000);
//    synd[6] = ^{data, synd[5:0]};
//    return synd;
//    endfunction
    
//    function slam_iccm_ram( input[31:0] addr, input[38:0] data);
//    int bank, idx;
    
//    bank = get_iccm_bank(addr, idx);
//    `ifdef RV_ICCM_ENABLE
//    case(bank) // {
//      0: `IRAM(0)[idx] = data;
//      1: `IRAM(1)[idx] = data;
//     `ifdef RV_ICCM_NUM_BANKS_4
//      2: `IRAM(2)[idx] = data;
//      3: `IRAM(3)[idx] = data;
//     `endif
//     `ifdef RV_ICCM_NUM_BANKS_8
//      2: `IRAM(2)[idx] = data;
//      3: `IRAM(3)[idx] = data;
//      4: `IRAM(4)[idx] = data;
//      5: `IRAM(5)[idx] = data;
//      6: `IRAM(6)[idx] = data;
//      7: `IRAM(7)[idx] = data;
//     `endif
    
//     `ifdef RV_ICCM_NUM_BANKS_16
//      2: `IRAM(2)[idx] = data;
//      3: `IRAM(3)[idx] = data;
//      4: `IRAM(4)[idx] = data;
//      5: `IRAM(5)[idx] = data;
//      6: `IRAM(6)[idx] = data;
//      7: `IRAM(7)[idx] = data;
//      8: `IRAM(8)[idx] = data;
//      9: `IRAM(9)[idx] = data;
//      10: `IRAM(10)[idx] = data;
//      11: `IRAM(11)[idx] = data;
//      12: `IRAM(12)[idx] = data;
//      13: `IRAM(13)[idx] = data;
//      14: `IRAM(14)[idx] = data;
//      15: `IRAM(15)[idx] = data;
//     `endif
//    endcase // }
//    `endif
//    endfunction
    logic        [3:0]          sb_hprot        ;
    logic        [2:0]          sb_hsize        ;
    logic        [1:0]          sb_htrans       ;
    logic                       sb_hwrite       ;

    logic        [63:0]         sb_hrdata       ;
    logic        [63:0]         sb_hwdata       ;
    logic                       sb_hready       ;
    logic                       sb_hresp        ;

    logic                       o_debug_mode_status;
  //  assign led_on1 = trace_rv_i_valid_ip; // Use for FPGA

    logic                       jtag_tdo;
    logic                       o_cpu_halt_ack;
    logic                       o_cpu_halt_status;
    logic                       o_cpu_run_ack;

/*
mailbox_write=0;
     dma_hrdata =0      ;
     dma_hwdata =0      ;
     dma_hready =0     ;
     dma_hresp  =0      ;
 
     mailbox_data_val=0;
     wb_valid=0;
     wb_dest=0;
     wb_data=0;*/

    logic                       mailbox_write;
    logic        [63:0]         dma_hrdata;
    logic        [63:0]         dma_hwdata;
    logic                       dma_hready ;
    logic                       dma_hresp  ;

    logic                       mpc_debug_halt_req;
    logic                       mpc_debug_run_req;
    logic                       mpc_reset_run_req;
    logic                       mpc_debug_halt_ack;
    logic                       mpc_debug_run_ack;
    logic                       debug_brkpt_status;

    logic                       mailbox_data_val;

    wire                        dma_hready_out;
    int                         commit_count;

    logic                       wb_valid;
    logic [4:0]                 wb_dest;
    logic [31:0]                wb_data;
    logic fetch_led;
    


//`ifdef RV_BUILD_AXI4
   //-------------------------- LSU AXI signals--------------------------
   // AXI Write Channels
    wire                        lsu_axi_awvalid;
    wire                        lsu_axi_awready;
    wire [`RV_LSU_BUS_TAG-1:0]  lsu_axi_awid;
    wire [31:0]                 lsu_axi_awaddr;
    wire [3:0]                  lsu_axi_awregion;
    wire [7:0]                  lsu_axi_awlen;
    wire [2:0]                  lsu_axi_awsize;
    wire [1:0]                  lsu_axi_awburst;
    wire                        lsu_axi_awlock;
    wire [3:0]                  lsu_axi_awcache;
    wire [2:0]                  lsu_axi_awprot;
    wire [3:0]                  lsu_axi_awqos;

    wire                        lsu_axi_wvalid;
    wire                        lsu_axi_wready;
    wire [63:0]                 lsu_axi_wdata;
    wire [7:0]                  lsu_axi_wstrb;
    wire                        lsu_axi_wlast;

    wire                        lsu_axi_bvalid;
    wire                        lsu_axi_bready;
    wire [1:0]                  lsu_axi_bresp;
    wire [`RV_LSU_BUS_TAG-1:0]  lsu_axi_bid;
//    assign lsu_axi_bid = 0;

    // AXI Read Channels
    wire                        lsu_axi_arvalid;
    wire                        lsu_axi_arready;
    wire [`RV_LSU_BUS_TAG-1:0]  lsu_axi_arid;
    wire [31:0]                 lsu_axi_araddr;
    wire [3:0]                  lsu_axi_arregion;
    wire [7:0]                  lsu_axi_arlen;
    wire [2:0]                  lsu_axi_arsize;
    wire [1:0]                  lsu_axi_arburst;
    wire                        lsu_axi_arlock;
    wire [3:0]                  lsu_axi_arcache;
    wire [2:0]                  lsu_axi_arprot;
    wire [3:0]                  lsu_axi_arqos;

    wire                        lsu_axi_rvalid;
    wire                        lsu_axi_rready;
    wire [`RV_LSU_BUS_TAG-1:0]  lsu_axi_rid;
//    assign lsu_axi_rid = 0;
    wire [63:0]                 lsu_axi_rdata;
    wire [1:0]                  lsu_axi_rresp;
    wire                        lsu_axi_rlast;
    
        parameter int unsigned AXI_ADDR_WIDTH    = 64;
       parameter int unsigned AXI_DATA_WIDTH    = 64;
       
          wire                        lmem_req_o;
           wire                        lmem_we_o;
           wire [AXI_ADDR_WIDTH-1:0]   lmem_addr_o;
           wire [AXI_DATA_WIDTH/8-1:0] lmem_be_o;
           wire [AXI_DATA_WIDTH-1:0]   lmem_data_o;
           wire [AXI_DATA_WIDTH-1:0]   lmem_data_i;

    //-------------------------- IFU AXI signals--------------------------
    // AXI Write Channels
    wire                        ifu_axi_awvalid;
    wire                        ifu_axi_awready;
    wire [`RV_IFU_BUS_TAG-1:0]  ifu_axi_awid;
    wire [31:0]                 ifu_axi_awaddr;
    wire [3:0]                  ifu_axi_awregion;
    wire [7:0]                  ifu_axi_awlen;
    wire [2:0]                  ifu_axi_awsize;
    wire [1:0]                  ifu_axi_awburst;
    wire                        ifu_axi_awlock;
    wire [3:0]                  ifu_axi_awcache;
    wire [2:0]                  ifu_axi_awprot;
    wire [3:0]                  ifu_axi_awqos;

    wire                        ifu_axi_wvalid;
    wire                        ifu_axi_wready;
    wire [63:0]                 ifu_axi_wdata;
    wire [7:0]                  ifu_axi_wstrb;
    wire                        ifu_axi_wlast;

    wire                        ifu_axi_bvalid;
    wire                        ifu_axi_bready;
    wire [1:0]                  ifu_axi_bresp;
    wire [`RV_IFU_BUS_TAG-1:0]  ifu_axi_bid;

    // AXI Read Channels
    wire                        ifu_axi_arvalid;
    wire                        ifu_axi_arready;
    wire [`RV_IFU_BUS_TAG-1:0]  ifu_axi_arid;
    wire [31:0]                 ifu_axi_araddr;
    wire [3:0]                  ifu_axi_arregion;
    wire [7:0]                  ifu_axi_arlen;
    wire [2:0]                  ifu_axi_arsize;
    wire [1:0]                  ifu_axi_arburst;
    wire                        ifu_axi_arlock;
    wire [3:0]                  ifu_axi_arcache;
    wire [2:0]                  ifu_axi_arprot;
    wire [3:0]                  ifu_axi_arqos;

    wire                        ifu_axi_rvalid;
    wire                        ifu_axi_rready;
    wire [`RV_IFU_BUS_TAG-1:0]  ifu_axi_rid;
//    assign ifu_axi_rid = 0;
    wire [63:0]                 ifu_axi_rdata;
    wire [1:0]                  ifu_axi_rresp;
    wire                        ifu_axi_rlast;
    
       wire                        imem_req_o;
        wire                        imem_we_o;
        wire [AXI_ADDR_WIDTH-1:0]   imem_addr_o;
        wire [AXI_DATA_WIDTH/8-1:0] imem_be_o;
        wire [AXI_DATA_WIDTH-1:0]   imem_data_o;
        wire [AXI_DATA_WIDTH-1:0]   imem_data_i;

    //-------------------------- SB AXI signals--------------------------
    // AXI Write Channels
    wire                        sb_axi_awvalid;
    wire                        sb_axi_awready;
    wire [`RV_SB_BUS_TAG-1:0]   sb_axi_awid;
    wire [31:0]                 sb_axi_awaddr;
    wire [3:0]                  sb_axi_awregion;
    wire [7:0]                  sb_axi_awlen;
    wire [2:0]                  sb_axi_awsize;
    wire [1:0]                  sb_axi_awburst;
    wire                        sb_axi_awlock;
    wire [3:0]                  sb_axi_awcache;
    wire [2:0]                  sb_axi_awprot;
    wire [3:0]                  sb_axi_awqos;

    wire                        sb_axi_wvalid;
    wire                        sb_axi_wready;
    wire [63:0]                 sb_axi_wdata;
    wire [7:0]                  sb_axi_wstrb;
    wire                        sb_axi_wlast;

    wire                        sb_axi_bvalid;
    wire                        sb_axi_bready;
    wire [1:0]                  sb_axi_bresp;
    wire [`RV_SB_BUS_TAG-1:0]   sb_axi_bid;

    // AXI Read Channels
    wire                        sb_axi_arvalid;
    wire                        sb_axi_arready;
    wire [`RV_SB_BUS_TAG-1:0]   sb_axi_arid;
    wire [31:0]                 sb_axi_araddr;
    wire [3:0]                  sb_axi_arregion;
    wire [7:0]                  sb_axi_arlen;
    wire [2:0]                  sb_axi_arsize;
    wire [1:0]                  sb_axi_arburst;
    wire                        sb_axi_arlock;
    wire [3:0]                  sb_axi_arcache;
    wire [2:0]                  sb_axi_arprot;
    wire [3:0]                  sb_axi_arqos;

    wire                        sb_axi_rvalid;
    wire                        sb_axi_rready;
    wire [`RV_SB_BUS_TAG-1:0]   sb_axi_rid;
    wire [63:0]                 sb_axi_rdata;
    wire [1:0]                  sb_axi_rresp;
    wire                        sb_axi_rlast;

   //-------------------------- DMA AXI signals--------------------------
   // AXI Write Channels
    wire                        dma_axi_awvalid;
    wire                        dma_axi_awready;
    wire [`RV_DMA_BUS_TAG-1:0]  dma_axi_awid;
    wire [31:0]                 dma_axi_awaddr;
    wire [2:0]                  dma_axi_awsize;
    wire [2:0]                  dma_axi_awprot;
    wire [7:0]                  dma_axi_awlen;
    wire [1:0]                  dma_axi_awburst;


    wire                        dma_axi_wvalid;
    wire                        dma_axi_wready;
    wire [63:0]                 dma_axi_wdata;
    wire [7:0]                  dma_axi_wstrb;
    wire                        dma_axi_wlast;

    wire                        dma_axi_bvalid;
    wire                        dma_axi_bready;
    wire [1:0]                  dma_axi_bresp;
    wire [`RV_DMA_BUS_TAG-1:0]  dma_axi_bid;

    // AXI Read Channels
    wire                        dma_axi_arvalid;
    wire                        dma_axi_arready;
    wire [`RV_DMA_BUS_TAG-1:0]  dma_axi_arid;
    wire [31:0]                 dma_axi_araddr;
    wire [2:0]                  dma_axi_arsize;
    wire [2:0]                  dma_axi_arprot;
    wire [7:0]                  dma_axi_arlen;
    wire [1:0]                  dma_axi_arburst;

    wire                        dma_axi_rvalid;
    wire                        dma_axi_rready;
    wire [`RV_DMA_BUS_TAG-1:0]  dma_axi_rid;
    wire [63:0]                 dma_axi_rdata;
    wire [1:0]                  dma_axi_rresp;
    wire                        dma_axi_rlast;

    wire                        lmem_axi_arvalid;
    wire                        lmem_axi_arready;

    wire                        lmem_axi_rvalid;
    wire [`RV_LSU_BUS_TAG-1:0]  lmem_axi_rid;
    wire [1:0]                  lmem_axi_rresp;
    wire [63:0]                 lmem_axi_rdata;
    wire                        lmem_axi_rlast;
    wire                        lmem_axi_rready;

    wire                        lmem_axi_awvalid;
    wire                        lmem_axi_awready;

    wire                        lmem_axi_wvalid;
    wire                        lmem_axi_wready;

    wire [1:0]                  lmem_axi_bresp;
    wire                        lmem_axi_bvalid;
    wire [`RV_LSU_BUS_TAG-1:0]  lmem_axi_bid;
    wire                        lmem_axi_bready;

//`endif
//    wire[63:0] WriteData;
//    string                      abi_reg[32]; // ABI register names

//`define DEC rvtop.swerv.dec

   // assign mailbox_write = lmem.mailbox_write;
//    assign WriteData = lmem.WriteData;
//    assign mailbox_data_val = WriteData[7:0] > 8'h5 && WriteData[7:0] < 8'h7f;

//    parameter MAX_CYCLES = 2_000_000;

//    integer fd, tp, el;

    always @(negedge core_clk) begin
        cycleCnt <= cycleCnt+1;
        // Test timeout monitor
    end

logic porst_l;
    assign porst_l = 0;
//    assign porst_l = ~porst; // Use for FPGA
// Comment the below lines    before next comment to enable / disable simulation mode
    logic clk_temp;
    assign core_clk = clk_in1;
//    clk_wiz_0 clk_gen0   // Use for FPGA 
//     (
//      // Clock out ports
//     .clk_out1(clk_temp),
//     // Clock in ports
//     .clk_in1(clk_in1)
//     );
     
//     always @(posedge clk_temp)
//     core_clk <= ~core_clk; 
/// Ends here
     
    always @(posedge core_clk or negedge rst_l)
     if (!rst_l) begin
     fetch_led <= 1'b0;
     nmi_int <= 0; 
     end
     else if (ifu_axi_rdata == 64'h55528293595552B7)
     fetch_led <= 1'b1;
     else 
     fetch_led <= fetch_led;
     logic /*pin0,pin1,*/pwm_en;
     
     assign inst_fetch_led = fetch_led;
     logic interrupt;
     assign rx_i = (programmed)? uart_rx_i:0;
   assign interrupt = TIMER_irq_o;
   //=========================================================================-
   // RTL instance
   //=========================================================================-
el2_swerv_wrapper rvtop (
    .rst_l                  ( rst_l         ),
    .dbg_rst_l              ( porst_l       ),
    .clk                    ( core_clk      ),
    .rst_vec                ( reset_vector[31:1]),
    .nmi_int                ( nmi_int       ),
    .nmi_vec                ( nmi_vector[31:1]),
    .jtag_id                ( jtag_id[31:1]),
    .pin0                   (pin0           ),
    .pin1                   (pin1           ),
    .pin2                   (pin2           ),
    .pin3                   (pin3           ),
    .pin4                   (pin4           ),
    .in0                    (in0            ),
    .in1                    (in1            ),
    .in2                    (in2            ),
    .in3                    (in3            ),
    .in4                    (in4            ),
    .TIMER_irq_o            (TIMER_irq_o),
    .pwm_led(pwm_led), 
    .pwm_en(pwm_en),
    .reset(reset),
    .rx_i(rx_i),
//    .tx_o(tx_o),
    .tx_o(tx_o),

`ifdef RV_BUILD_AHB_LITE
    .haddr                  ( ic_haddr      ),
    .hburst                 ( ic_hburst     ),
    .hmastlock              ( ic_hmastlock  ),
    .hprot                  ( ic_hprot      ),
    .hsize                  ( ic_hsize      ),
    .htrans                 ( ic_htrans     ),
    .hwrite                 ( ic_hwrite     ),

    .hrdata                 ( ic_hrdata[63:0]),
    .hready                 ( ic_hready     ),
    .hresp                  ( ic_hresp      ),

    //---------------------------------------------------------------
    // Debug AHB Master
    //---------------------------------------------------------------
    .sb_haddr               ( sb_haddr      ),
    .sb_hburst              ( sb_hburst     ),
    .sb_hmastlock           ( sb_hmastlock  ),
    .sb_hprot               ( sb_hprot      ),
    .sb_hsize               ( sb_hsize      ),
    .sb_htrans              ( sb_htrans     ),
    .sb_hwrite              ( sb_hwrite     ),
    .sb_hwdata              ( sb_hwdata     ),

    .sb_hrdata              ( sb_hrdata     ),
    .sb_hready              ( sb_hready     ),
    .sb_hresp               ( sb_hresp      ),

    //---------------------------------------------------------------
    // LSU AHB Master
    //---------------------------------------------------------------
    .lsu_haddr              ( lsu_haddr       ),
    .lsu_hburst             ( lsu_hburst      ),
    .lsu_hmastlock          ( lsu_hmastlock   ),
    .lsu_hprot              ( lsu_hprot       ),
    .lsu_hsize              ( lsu_hsize       ),
    .lsu_htrans             ( lsu_htrans      ),
    .lsu_hwrite             ( lsu_hwrite      ),
    .lsu_hwdata             ( lsu_hwdata      ),

    .lsu_hrdata             ( lsu_hrdata[63:0]),
    .lsu_hready             ( lsu_hready      ),
    .lsu_hresp              ( lsu_hresp       ),

    //---------------------------------------------------------------
    // DMA Slave
    //---------------------------------------------------------------
    .dma_haddr              ( '0 ),
    .dma_hburst             ( '0 ),
    .dma_hmastlock          ( '0 ),
    .dma_hprot              ( '0 ),
    .dma_hsize              ( '0 ),
    .dma_htrans             ( '0 ),
    .dma_hwrite             ( '0 ),
    .dma_hwdata             ( '0 ),

    .dma_hrdata             ( dma_hrdata    ),
    .dma_hresp              ( dma_hresp     ),
    .dma_hsel               ( 1'b1            ),
    .dma_hreadyin           ( dma_hready_out  ),
    .dma_hreadyout          ( dma_hready_out  ),
`endif
`ifdef RV_BUILD_AXI4
    //-------------------------- LSU AXI signals--------------------------
    // AXI Write Channels
    .lsu_axi_awvalid_o        (lsu_axi_awvalid),
    .lsu_axi_awready_o        (lsu_axi_awready),
    .lsu_axi_awid_o           (lsu_axi_awid),
    .lsu_axi_awaddr_o         (lsu_axi_awaddr),
    .lsu_axi_awregion_o       (lsu_axi_awregion),
    .lsu_axi_awlen_o          (lsu_axi_awlen),
    .lsu_axi_awsize_o         (lsu_axi_awsize),
    .lsu_axi_awburst_o        (lsu_axi_awburst),
    .lsu_axi_awlock_o         (lsu_axi_awlock),
    .lsu_axi_awcache_o        (lsu_axi_awcache),
    .lsu_axi_awprot_o         (lsu_axi_awprot),
    .lsu_axi_awqos_o          (lsu_axi_awqos),

    .lsu_axi_wvalid_o         (lsu_axi_wvalid),
    .lsu_axi_wready_o         (lsu_axi_wready),
    .lsu_axi_wdata_o          (lsu_axi_wdata),
    .lsu_axi_wstrb_o          (lsu_axi_wstrb),
    .lsu_axi_wlast_o          (lsu_axi_wlast),

    .lsu_axi_bvalid_o         (lsu_axi_bvalid),
    .lsu_axi_bready_o         (lsu_axi_bready),
    .lsu_axi_bresp_o          (lsu_axi_bresp),
    .lsu_axi_bid_o            (lsu_axi_bid),


    .lsu_axi_arvalid_o        (lsu_axi_arvalid),
    .lsu_axi_arready_o        (lsu_axi_arready),
    .lsu_axi_arid_o           (lsu_axi_arid),
    .lsu_axi_araddr_o         (lsu_axi_araddr),
    .lsu_axi_arregion_o       (lsu_axi_arregion),
    .lsu_axi_arlen_o          (lsu_axi_arlen),
    .lsu_axi_arsize_o         (lsu_axi_arsize),
    .lsu_axi_arburst_o        (lsu_axi_arburst),
    .lsu_axi_arlock_o         (lsu_axi_arlock),
    .lsu_axi_arcache_o        (lsu_axi_arcache),
    .lsu_axi_arprot_o         (lsu_axi_arprot),
    .lsu_axi_arqos_o          (lsu_axi_arqos),

    .lsu_axi_rvalid_o         (lsu_axi_rvalid),
    .lsu_axi_rready_o         (lsu_axi_rready),
    .lsu_axi_rid_o            (lsu_axi_rid),
    .lsu_axi_rdata_o          (lsu_axi_rdata),
    .lsu_axi_rresp_o          (lsu_axi_rresp),
    .lsu_axi_rlast_o          (lsu_axi_rlast),
    //-------------------------- IFU AXI signals--------------------------
    // AXI Write Channels
    .ifu_axi_awvalid        (ifu_axi_awvalid),
    .ifu_axi_awready        (ifu_axi_awready),
    .ifu_axi_awid           (ifu_axi_awid),
    .ifu_axi_awaddr         (ifu_axi_awaddr),
    .ifu_axi_awregion       (ifu_axi_awregion),
    .ifu_axi_awlen          (ifu_axi_awlen),
    .ifu_axi_awsize         (ifu_axi_awsize),
    .ifu_axi_awburst        (ifu_axi_awburst),
    .ifu_axi_awlock         (ifu_axi_awlock),
    .ifu_axi_awcache        (ifu_axi_awcache),
    .ifu_axi_awprot         (ifu_axi_awprot),
    .ifu_axi_awqos          (ifu_axi_awqos),

    .ifu_axi_wvalid         (ifu_axi_wvalid),
    .ifu_axi_wready         (ifu_axi_wready),
    .ifu_axi_wdata          (ifu_axi_wdata),
    .ifu_axi_wstrb          (ifu_axi_wstrb),
    .ifu_axi_wlast          (ifu_axi_wlast),

    .ifu_axi_bvalid         (ifu_axi_bvalid),
    .ifu_axi_bready         (ifu_axi_bready),
    .ifu_axi_bresp          (ifu_axi_bresp),
    .ifu_axi_bid            (ifu_axi_bid),

    .ifu_axi_arvalid        (ifu_axi_arvalid),
    .ifu_axi_arready        (ifu_axi_arready),
    .ifu_axi_arid           (ifu_axi_arid),
    .ifu_axi_araddr         (ifu_axi_araddr),
    .ifu_axi_arregion       (ifu_axi_arregion),
    .ifu_axi_arlen          (ifu_axi_arlen),
    .ifu_axi_arsize         (ifu_axi_arsize),
    .ifu_axi_arburst        (ifu_axi_arburst),
    .ifu_axi_arlock         (ifu_axi_arlock),
    .ifu_axi_arcache        (ifu_axi_arcache),
    .ifu_axi_arprot         (ifu_axi_arprot),
    .ifu_axi_arqos          (ifu_axi_arqos),

    .ifu_axi_rvalid         (ifu_axi_rvalid),
    .ifu_axi_rready         (ifu_axi_rready),
    .ifu_axi_rid            (ifu_axi_rid),
    .ifu_axi_rdata          (ifu_axi_rdata),
    .ifu_axi_rresp          (ifu_axi_rresp),
    .ifu_axi_rlast          (ifu_axi_rlast),

    //-------------------------- SB AXI signals--------------------------
    // AXI Write Channels
    .sb_axi_awvalid         (sb_axi_awvalid),
    .sb_axi_awready         (sb_axi_awready),
    .sb_axi_awid            (sb_axi_awid),
    .sb_axi_awaddr          (sb_axi_awaddr),
    .sb_axi_awregion        (sb_axi_awregion),
    .sb_axi_awlen           (sb_axi_awlen),
    .sb_axi_awsize          (sb_axi_awsize),
    .sb_axi_awburst         (sb_axi_awburst),
    .sb_axi_awlock          (sb_axi_awlock),
    .sb_axi_awcache         (sb_axi_awcache),
    .sb_axi_awprot          (sb_axi_awprot),
    .sb_axi_awqos           (sb_axi_awqos),

    .sb_axi_wvalid          (sb_axi_wvalid),
    .sb_axi_wready          (sb_axi_wready),
    .sb_axi_wdata           (sb_axi_wdata),
    .sb_axi_wstrb           (sb_axi_wstrb),
    .sb_axi_wlast           (sb_axi_wlast),

    .sb_axi_bvalid          (sb_axi_bvalid),
    .sb_axi_bready          (sb_axi_bready),
    .sb_axi_bresp           (sb_axi_bresp),
    .sb_axi_bid             (sb_axi_bid),


    .sb_axi_arvalid         (sb_axi_arvalid),
    .sb_axi_arready         (sb_axi_arready),
    .sb_axi_arid            (sb_axi_arid),
    .sb_axi_araddr          (sb_axi_araddr),
    .sb_axi_arregion        (sb_axi_arregion),
    .sb_axi_arlen           (sb_axi_arlen),
    .sb_axi_arsize          (sb_axi_arsize),
    .sb_axi_arburst         (sb_axi_arburst),
    .sb_axi_arlock          (sb_axi_arlock),
    .sb_axi_arcache         (sb_axi_arcache),
    .sb_axi_arprot          (sb_axi_arprot),
    .sb_axi_arqos           (sb_axi_arqos),

    .sb_axi_rvalid          (sb_axi_rvalid),
    .sb_axi_rready          (sb_axi_rready),
    .sb_axi_rid             (sb_axi_rid),
    .sb_axi_rdata           (sb_axi_rdata),
    .sb_axi_rresp           (sb_axi_rresp),
    .sb_axi_rlast           (sb_axi_rlast),

    //-------------------------- DMA AXI signals--------------------------
    // AXI Write Channels
    .dma_axi_awvalid        (dma_axi_awvalid),
    .dma_axi_awready        (dma_axi_awready),
    .dma_axi_awid           ('0),
    .dma_axi_awaddr         (lsu_axi_awaddr),
    .dma_axi_awsize         (lsu_axi_awsize),
    .dma_axi_awprot         (lsu_axi_awprot),
    .dma_axi_awlen          (lsu_axi_awlen),
    .dma_axi_awburst        (lsu_axi_awburst),


    .dma_axi_wvalid         (dma_axi_wvalid),
    .dma_axi_wready         (dma_axi_wready),
    .dma_axi_wdata          (lsu_axi_wdata),
    .dma_axi_wstrb          (lsu_axi_wstrb),
    .dma_axi_wlast          (lsu_axi_wlast),

    .dma_axi_bvalid         (dma_axi_bvalid),
    .dma_axi_bready         (dma_axi_bready),
    .dma_axi_bresp          (dma_axi_bresp),
    .dma_axi_bid            (),


    .dma_axi_arvalid        (dma_axi_arvalid),
    .dma_axi_arready        (dma_axi_arready),
    .dma_axi_arid           ('0),
    .dma_axi_araddr         (lsu_axi_araddr),
    .dma_axi_arsize         (lsu_axi_arsize),
    .dma_axi_arprot         (lsu_axi_arprot),
    .dma_axi_arlen          (lsu_axi_arlen),
    .dma_axi_arburst        (lsu_axi_arburst),

    .dma_axi_rvalid         (dma_axi_rvalid),
    .dma_axi_rready         (dma_axi_rready),
    .dma_axi_rid            (),
    .dma_axi_rdata          (dma_axi_rdata),
    .dma_axi_rresp          (dma_axi_rresp),
    .dma_axi_rlast          (dma_axi_rlast),
`endif
    .timer_int              ( 1'b0     ),
    .extintsrc_req          ({30'd0,interrupt,interrupt}),
    .lsu_bus_clk_en         ( 1'b1  ),// Clock ratio b/w cpu core clk & AHB master interface
    .ifu_bus_clk_en         ( 1'b1  ),// Clock ratio b/w cpu core clk & AHB master interface
    .dbg_bus_clk_en         ( 1'b1  ),// Clock ratio b/w cpu core clk & AHB Debug master interface
    .dma_bus_clk_en         ( 1'b1  ),// Clock ratio b/w cpu core clk & AHB slave interface

    .trace_rv_i_insn_ip     (trace_rv_i_insn_ip),
    .trace_rv_i_address_ip  (trace_rv_i_address_ip),
    .trace_rv_i_valid_ip    (trace_rv_i_valid_ip),
    .trace_rv_i_exception_ip(trace_rv_i_exception_ip),
    .trace_rv_i_ecause_ip   (trace_rv_i_ecause_ip),
    .trace_rv_i_interrupt_ip(trace_rv_i_interrupt_ip),
    .trace_rv_i_tval_ip     (trace_rv_i_tval_ip),

    .jtag_tck               ( 1'b0  ),
    .jtag_tms               ( 1'b0  ),
    .jtag_tdi               ( 1'b0  ),
    .jtag_trst_n            ( 1'b0  ),
    .jtag_tdo               ( jtag_tdo ),

    .mpc_debug_halt_ack     ( mpc_debug_halt_ack),
    .mpc_debug_halt_req     ( 1'b0),
    .mpc_debug_run_ack      ( mpc_debug_run_ack),
    .mpc_debug_run_req      ( 1'b1),
    .mpc_reset_run_req      ( 1'b1),             // Start running after reset
     .debug_brkpt_status    (debug_brkpt_status),

    .i_cpu_halt_req         ( 1'b0  ),    // Async halt req to CPU
    .o_cpu_halt_ack         ( o_cpu_halt_ack ),    // core response to halt
    .o_cpu_halt_status      ( o_cpu_halt_status ), // 1'b1 indicates core is halted
    .i_cpu_run_req          ( 1'b0  ),     // Async restart req to CPU
    .o_debug_mode_status    (o_debug_mode_status),
    .o_cpu_run_ack          ( o_cpu_run_ack ),     // Core response to run req

    .dec_tlu_perfcnt0       (),
    .dec_tlu_perfcnt1       (),
    .dec_tlu_perfcnt2       (),
    .dec_tlu_perfcnt3       (),

// remove mems DFT pins for opensource
    .dccm_ext_in_pkt        ('0),
    .iccm_ext_in_pkt        ('0),
    .ic_data_ext_in_pkt     ('0),
    .ic_tag_ext_in_pkt      ('0),

    .soft_int               ('0),
    .core_id                ('0),
    .scan_mode              ( 1'b0 ),         // To enable scan mode
    .mbist_mode             ( 1'b0 )        // to enable mbist

);


   //=========================================================================-
   // AHB I$ instance
   //=========================================================================-
`ifdef RV_BUILD_AHB_LITE

ahb_sif imem (
     // Inputs
     .HWDATA(64'h0),
     .HCLK(core_clk),
     .HSEL(1'b1),
     .HPROT(ic_hprot),
     .HWRITE(ic_hwrite),
     .HTRANS(ic_htrans),
     .HSIZE(ic_hsize),
     .HREADY(ic_hready),
     .HRESETn(rst_l),
     .HADDR(ic_haddr),
     .HBURST(ic_hburst),

     // Outputs
     .HREADYOUT(ic_hready),
     .HRESP(ic_hresp),
     .HRDATA(ic_hrdata[63:0])
);


ahb_sif lmem (
     // Inputs
     .HWDATA(lsu_hwdata),
     .HCLK(core_clk),
     .HSEL(1'b1),
     .HPROT(lsu_hprot),
     .HWRITE(lsu_hwrite),
     .HTRANS(lsu_htrans),
     .HSIZE(lsu_hsize),
     .HREADY(lsu_hready),
     .HRESETn(rst_l),
     .HADDR(lsu_haddr),
     .HBURST(lsu_hburst),

     // Outputs
     .HREADYOUT(lsu_hready),
     .HRESP(lsu_hresp),
     .HRDATA(lsu_hrdata[63:0])
);

`endif
`ifdef RV_BUILD_AXI4
//axi_slv #(.TAGW(`RV_IFU_BUS_TAG)) imem(
//    .aclk(core_clk),
//    .rst_l(rst_l),
//    .arvalid(ifu_axi_arvalid),
//    .arready(ifu_axi_arready),
//    .araddr(ifu_axi_araddr),
//    .arid(ifu_axi_arid),
//    .arlen(ifu_axi_arlen),
//    .arburst(ifu_axi_arburst),
//    .arsize(ifu_axi_arsize),

//    .rvalid(ifu_axi_rvalid),
//    .rready(ifu_axi_rready),
//    .rdata(ifu_axi_rdata),
//    .rresp(ifu_axi_rresp),
//    .rid(ifu_axi_rid),
//    .rlast(ifu_axi_rlast),

//    .awvalid(1'b0),
//    .awready(),
//    .awaddr('0),
//    .awid('0),
//    .awlen('0),
//    .awburst('0),
//    .awsize('0),

//    .wdata('0),
//    .wstrb('0),
//    .wvalid(1'b0),
//    .wready(),

//    .bvalid(),
//    .bready(1'b0),
//    .bresp(),
//    .bid()
//);

////defparam lmem.TAGW =`RV_LSU_BUS_TAG;

//axi_slv #(.TAGW(`RV_LSU_BUS_TAG)) lmem(
////axi_slv  lmem(
//    .aclk(core_clk),
//    .rst_l(rst_l),
//    .arvalid(lmem_axi_arvalid),
//    .arready(lmem_axi_arready),
//    .araddr(lsu_axi_araddr),
//    .arid(lsu_axi_arid),
//    .arlen(lsu_axi_arlen),
//    .arburst(lsu_axi_arburst),text
//    .arsize(lsu_axi_arsize),

//    .rvalid(lmem_axi_rvalid),
//    .rready(lmem_axi_rready),
//    .rdata(lmem_axi_rdata),
//    .rresp(lmem_axi_rresp),
//    .rid(lmem_axi_rid),
//    .rlast(lmem_axi_rlast),

//    .awvalid(lmem_axi_awvalid),
//    .awready(lmem_axi_awready),
//    .awaddr(lsu_axi_awaddr),
//    .awid(lsu_axi_awid),
//    .awlen(lsu_axi_awlen),
//    .awburst(lsu_axi_awburst),
//    .awsize(lsu_axi_awsize),

//    .wdata(lsu_axi_wdata),
//    .wstrb(lsu_axi_wstrb),
//    .wvalid(lmem_axi_wvalid),
//    .wready(lmem_axi_wready),

//    .bvalid(lmem_axi_bvalid),
//    .bready(lmem_axi_bready),
//    .bresp(lmem_axi_bresp),
//    .bid(lmem_axi_bid)
//);

logic [31:0] ifu_axi_araddr_t;
//logic zero_it=0;
//always @(posedge core_clk) begin
//if(ifu_axi_araddr == 32'h8xxxxxxx || ifu_axi_araddr == 32'h8xxxxxxx )begin
//zero_it <= 0;
//end
//else
//zero_it <= 1;
//end

//assign ifu_axi_araddr_t = (ifu_axi_araddr == 32'h8xxxxxxx)? 32'hFFFFFFFF:ifu_axi_araddr;

//**********Master Logic******************
//always_comb begin
//if (ifu_axi_araddr == 32'h80000000 || ifu_axi_araddr == 32'h80000008 || ifu_axi_araddr == 32'h80000010 )
//ifu_axi_araddr_t = 32'hFFFFFFFF;
//else
//ifu_axi_araddr_t = ifu_axi_araddr;
//end


parameter C_AXI_DATA_WIDTH	= 64;// Width of the AXI R&W data
	parameter C_AXI_ADDR_WIDTH	=  32;	// AXI Address width (log wordsize)
	parameter C_AXI_ID_WIDTH	=   3;
	parameter DW			=  64;	// Wishbone data width
	parameter AW			=  32;	// Wishbone address width (log wordsize)
	parameter [C_AXI_ID_WIDTH-1:0] AXI_WRITE_ID = 1'b0;
	parameter [C_AXI_ID_WIDTH-1:0] AXI_READ_ID  = 1'b1;
	//
	// OPT_LITTLE_ENDIAN controls which word has the greatest address
	// when the bus size is adjusted.  If OPT_LITTLE_ENDIAN is true,
	// the biggest address is in the most significant word(s), otherwise
	// the least significant word(s).  This parameter is ignored if
	// C_AXI_DATA_WIDTH == DW.
	parameter [0:0]			OPT_LITTLE_ENDIAN = 1'b1;
	parameter LGFIFO		=   6;
logic        awvalid;
logic        awready;
logic [2:0]  awid;
logic [31:0] awaddr;
logic	[7:0]	 awlen;	// Write Burst Length
logic	[2:0]	 awsize;	// Write Burst size
logic	[1:0]	 awburst;	// Write Burst type
logic	[0:0]	 awlock;	// Write lock type
logic	[3:0]	 awcache;	// Write Cache type
logic	[2:0]	 awprot;	// Write Protection type
logic	[3:0]	 awqos;	// Write Quality of Svc

// AXI write data channel signals
logic	                         wvalid;	// Write valid
logic	                         wready;  // Write data ready
logic	[C_AXI_DATA_WIDTH-1:0]	 wdata;	// Write data
logic	[C_AXI_DATA_WIDTH/8-1:0] wstrb;	// Write strobes
logic			                     wlast;	// Last write transaction

// AXI write response channel signals
logic                          bvalid;  // Write reponse valid
logic	                         bready;  // Response ready
logic [C_AXI_ID_WIDTH-1:0]	   bid;// Response ID
logic [1:0]		                 bresp;	// Write response

bus_converter simpletoAxi (
	// {{{
	.i_clk          (core_clk),	// System clock
	.i_reset        (~rst_l),// Reset signal,drives AXI rst

	// AXI write address channel signals
  .o_axi_awvalid  (awvalid),	// Write address valid
	.i_axi_awready  (awready), // Slave is ready to accept
	.o_axi_awid     (awid   ),	// Write ID
	.o_axi_awaddr   (awaddr ),	// Write address
	.o_axi_awlen    (awlen  ),	// Write Burst Length
	.o_axi_awsize   (awsize ),	// Write Burst size
	.o_axi_awburst  (awburst),	// Write Burst type
	.o_axi_awlock   (awlock ),	// Write lock type
	.o_axi_awcache  (awcache),	// Write Cache type
	.o_axi_awprot   (awprot ),	// Write Protection type
	.o_axi_awqos    (awqos  ),	// Write Quality of Svc

// AXI write data channel signals
	.o_axi_wvalid   (wvalid),	// Write valid
	.i_axi_wready   (wready),  // Write data ready
	.o_axi_wdata    (wdata ),	// Write data
	.o_axi_wstrb    (wstrb ),	// Write strobes
	.o_axi_wlast    (wlast ),	// Last write transaction

// AXI write response channel signals
	.i_axi_bvalid   (bvalid),  // Write reponse valid
	.o_axi_bready   (bready),  // Response ready
	.i_axi_bid      (bid   ),	// Response ID
	.i_axi_bresp    (bresp ),	// Write response

// AXI read address channel signals
	.o_axi_arvalid  (),	// Read address valid
	.i_axi_arready  (1'd1),	// Read address ready
	.o_axi_arid     (),	// Read ID
	.o_axi_araddr   (),	// Read address
	.o_axi_arlen    (),	// Read Burst Length
	.o_axi_arsize   (),	// Read Burst size
	.o_axi_arburst  (),	// Read Burst type
	.o_axi_arlock   (),	// Read lock type
	.o_axi_arcache  (),	// Read Cache type
	.o_axi_arprot   (),	// Read Protection type
	.o_axi_arqos    (),	// Read Protection type

// AXI read data channel signals
	.i_axi_rvalid   (1'd1),  // Read reponse valid
  .o_axi_rready   (),  // Read Response ready
	.i_axi_rid      (3'd0),     // Response ID
	.i_axi_rdata    (64'd32),    // Read data
  .i_axi_rresp    (1'd1),   // Read response
	.i_axi_rlast    (1'd1),    // Read last

	// We'll share the clock and the reset
	.i_wb_cyc       (core_clk),
  .i_wb_stb       (1'b1),
	.i_wb_we        (we),
	.i_wb_addr      ({18'd0,addr}),
	.i_wb_data      (data),
	.i_wb_sel       (8'hFF),
  .o_wb_stall     (),
  .o_wb_ack       (),
  .o_wb_data      (),
  .o_wb_err       ()
	// }}}
);
//axi_slv #(.TAGW(`RV_IFU_BUS_TAG)) imem(
logic rst_l_s;
always@(posedge core_clk)
rst_l_s <= rst_l;

////axi_slv imem(
//    .s_aclk(core_clk),
//    .s_aresetn(rst_l_s),
//    .s_axi_arvalid(ifu_axi_arvalid),
//    .s_axi_arready(ifu_axi_arready),
//    .s_axi_araddr(ifu_axi_araddr),
//    .s_axi_arid(ifu_axi_arid),
//    .s_axi_arlen(ifu_axi_arlen),
//    .s_axi_arburst(ifu_axi_arburst),
//    .s_axi_arsize(ifu_axi_arsize),

//    .s_axi_rvalid(ifu_axi_rvalid),
//    .s_axi_rready(ifu_axi_rready),
////    .s_axi_rdata(ifu_axi_rdata_t),
//    .s_axi_rdata(ifu_axi_rdata),
//    .s_axi_rresp(ifu_axi_rresp),
//    .s_axi_rid(ifu_axi_rid),
//    .s_axi_rlast(ifu_axi_rlast),
    
//    .s_axi_awvalid(awvalid),
//    .s_axi_awready(awready),
//    .s_axi_awaddr(awaddr),
//    .s_axi_awid(awid),
//    .s_axi_awlen(awlen),
//    .s_axi_awburst(awburst),
//    .s_axi_awsize(awsize),
//    .s_axi_wlast(wlast),

//    .s_axi_wdata(wdata),
//    .s_axi_wstrb(wstrb),
//    .s_axi_wvalid(wvalid),
//    .s_axi_wready(wready),

//    .s_axi_bvalid(bvalid),
//    .s_axi_bready(bready),
//    .s_axi_bresp(bresp),
//    .s_axi_bid(bid)
//);
axi2mem #(
  .ID_WIDTH(`RV_LSU_BUS_TAG),
  .AXI_ADDR_WIDTH(64),
  .AXI_DATA_WIDTH(64),
  .AXI_USER_WIDTH(0)
) u_axi2memimem(
.clk_i                   (core_clk),
.rst_ni                  (~rst_l_s),
.slave_ar_id             (ifu_axi_arid),
.slave_ar_addr           (ifu_axi_araddr),
.slave_ar_len            (ifu_axi_arlen),
.slave_ar_size           (ifu_axi_arsize),
.slave_ar_burst          (ifu_axi_arburst),
.slave_ar_valid          (ifu_axi_arvalid),
.slave_ar_ready          (ifu_axi_arready),   
.slave_r_id              (ifu_axi_rid),
.slave_r_data            (ifu_axi_rdata),
//.slave_r_data           (ifu_axi_rdata_t),
.slave_r_resp            (ifu_axi_rresp),
.slave_r_last            (ifu_axi_rlast),
.slave_r_valid           (ifu_axi_rvalid),
.slave_r_ready           (ifu_axi_rready),   

.slave_w_data             (wdata),
.slave_w_strb             (wstrb),
.slave_w_last             (wlast),
.slave_w_valid            (wvalid),
.slave_w_ready            (wready),

.slave_aw_id                (awid),
.slave_aw_addr              (awaddr),
.slave_aw_len               (awlen),
.slave_aw_size              (awsize),
.slave_aw_burst             (awburst),
.slave_aw_valid             (awvalid),
.slave_aw_ready             (awready),

.slave_b_id                  (bid),
.slave_b_resp               (bresp),
.slave_b_valid              (bvalid),
.slave_b_ready              (bready),

 .req_o                    (imem_req_o),
 .we_o                     (imem_we_o),
 .addr_o                   (imem_addr_o[10:0]),
 .be_o                     (imem_be_o),
 .data_o                   (imem_data_o),
 .data_i                   (imem_data_i)
);
sky130_sram_1kbyte_1rw1r_32x256_8 imem (
 .clk0(core_clk),
 .csb0(~imem_req_o),
 .web0(~imem_we_o),
 .wmask0(imem_be_o[3:0]),
 .addr0(imem_addr_o[10:3]),
 .din0(imem_data_o[31:0]),
 .dout0(imem_data_i[31:0]),
 .clk1('b0),
 .csb1('b0),
 .addr1('b0),
 .dout1()
);

sky130_sram_1kbyte_1rw1r_32x256_8 imem1 (
 .clk0(core_clk),
 .csb0(~imem_req_o),
 .web0(~imem_we_o),
 .wmask0(imem_be_o[7:4]),
 .addr0(imem_addr_o[10:3]),
 .din0(imem_data_o[63:32]),
 .dout0(imem_data_i[63:32]),
 .clk1('b0),
 .csb1('b0),
 .addr1('b0),
 .dout1()
);

//defparam lmem.TAGW =`RV_LSU_BUS_TAG;

//axi_slv_l #(.TAGW(`RV_LSU_BUS_TAG)) lmem(
//axi_slv lmem(
////axi_slv  lmem(
//    .s_aclk(core_clk),
//    .s_aresetn(rst_l_s),
//    .s_axi_arvalid(lmem_axi_arvalid),
//    .s_axi_arready(lmem_axi_arready),
//    .s_axi_araddr(lsu_axi_araddr),
//    .s_axi_arid(lsu_axi_arid),
//    .s_axi_arlen(lsu_axi_arlen),
//    .s_axi_arburst(lsu_axi_arburst),
//    .s_axi_arsize(lsu_axi_arsize),

//    .s_axi_rvalid(lmem_axi_rvalid),
//    .s_axi_rready(lmem_axi_rready),
//    .s_axi_rdata(lmem_axi_rdata),
//    .s_axi_rresp(lmem_axi_rresp),
//    .s_axi_rid(lmem_axi_rid),
//    .s_axi_rlast(lmem_axi_rlast),

//    .s_axi_awvalid(lmem_axi_awvalid),
//    .s_axi_awready(lmem_axi_awready),
//    .s_axi_awaddr(lsu_axi_awaddr),
//    .s_axi_awid(lsu_axi_awid),
//    .s_axi_awlen(lsu_axi_awlen),
//    .s_axi_awburst(lsu_axi_awburst),
//    .s_axi_awsize(lsu_axi_awsize),

//    .s_axi_wdata(lsu_axi_wdata),
//    .s_axi_wstrb(lsu_axi_wstrb),
//    .s_axi_wvalid(lmem_axi_wvalid),
//    .s_axi_wready(lmem_axi_wready),

//    .s_axi_bvalid(lmem_axi_bvalid),
//    .s_axi_bready(lmem_axi_bready),
//    .s_axi_bresp(lmem_axi_bresp),
//    .s_axi_bid(lmem_axi_bid)
//);
axi2mem #(
        .ID_WIDTH(`RV_LSU_BUS_TAG),
        .AXI_ADDR_WIDTH(64),
        .AXI_DATA_WIDTH(64),
        .AXI_USER_WIDTH(0)
 ) u_axi2meml(
    //---------------------AXI-SRAM-----------------------------------
    .clk_i                   (core_clk),
    .rst_ni                  (~rst_l_s),
    .slave_aw_id     (lsu_axi_awid),
    .slave_aw_addr   (lsu_axi_awaddr),
    .slave_aw_len   (lsu_axi_awlen),
    .slave_aw_size  (lsu_axi_awsize),
    .slave_aw_burst (lsu_axi_awburst),
    .slave_aw_valid (lmem_axi_awvalid),
    .slave_aw_ready (lmem_axi_awready),
    .slave_ar_id    (lsu_axi_arid),
    .slave_ar_addr  (lsu_axi_araddr),
    .slave_ar_len   (lsu_axi_arlen),
    .slave_ar_size  (lsu_axi_arsize),
    .slave_ar_burst (lsu_axi_arburst),
    .slave_ar_valid  (lmem_axi_arvalid),
    .slave_ar_ready  (lmem_axi_arready),
    .slave_w_data   (lsu_axi_wdata),
    .slave_w_strb   (lsu_axi_wstrb),
    .slave_w_last   (),
    .slave_w_valid  (lmem_axi_wvalid),
    .slave_w_ready  (lmem_axi_wready),
    .slave_b_id     (lmem_axi_bid),
    .slave_b_resp   (lmem_axi_bresp),
    .slave_b_valid  (lmem_axi_bvalid),
    .slave_b_ready  (lmem_axi_bready),
    .slave_r_id     (lmem_axi_rid),
    .slave_r_data    (lmem_axi_rdata),
    .slave_r_resp   (lmem_axi_rresp),
    .slave_r_last   (lmem_axi_rlast),
    .slave_r_valid   (lmem_axi_rvalid),
    .slave_r_ready  (lmem_axi_rready),

    .req_o                    (lmem_req_o),
    .we_o                     (lmem_we_o),
    .addr_o                   (lmem_addr_o[10:0]),
    .be_o                     (lmem_be_o),
    .data_o                   (lmem_data_o),
    .data_i                   (lmem_data_i)
    );


 sky130_sram_1kbyte_1rw1r_32x256_8 lmem (
    .clk0(core_clk),
    .csb0(~lmem_req_o),
    .web0(~lmem_we_o),
    .wmask0(lmem_be_o[3:0]),
    .addr0(lmem_addr_o[10:3]),
    .din0(lmem_data_o[31:0]),
    .dout0(lmem_data_i[31:0]),
    .clk1(0),
    .csb1(0),
    .addr1(0),
    .dout1()
  );
sky130_sram_1kbyte_1rw1r_32x256_8 lmem1 (
      .clk0(core_clk),
      .csb0(~lmem_req_o),
      .web0(~lmem_we_o),
      .wmask0(lmem_be_o[7:4]),
      .addr0(lmem_addr_o[10:3]),
      .din0(lmem_data_o[63:32]),
      .dout0(lmem_data_i[63:32]),
      .clk1(0),
      .csb1(0),
      .addr1(0),
      .dout1()
    );
axi_lsu_dma_bridge # (`RV_LSU_BUS_TAG,`RV_LSU_BUS_TAG ) bridge(
    .clk(core_clk),
    .reset_l(rst_l),

    .m_arvalid(lsu_axi_arvalid),
    .m_arid(lsu_axi_arid),
    .m_araddr(lsu_axi_araddr),
    .m_arready(lsu_axi_arready),

    .m_rvalid(lsu_axi_rvalid),
    .m_rready(lsu_axi_rready),
    .m_rdata(lsu_axi_rdata),
    .m_rid(lsu_axi_rid),
    .m_rresp(lsu_axi_rresp),
    .m_rlast(lsu_axi_rlast),

    .m_awvalid(lsu_axi_awvalid),
    .m_awid(lsu_axi_awid),
    .m_awaddr(lsu_axi_awaddr),
    .m_awready(lsu_axi_awready),

    .m_wvalid(lsu_axi_wvalid),
    .m_wready(lsu_axi_wready),

    .m_bresp(lsu_axi_bresp),
    .m_bvalid(lsu_axi_bvalid),
    .m_bid(lsu_axi_bid),
    .m_bready(lsu_axi_bready),

    .s0_arvalid(lmem_axi_arvalid),
    .s0_arready(lmem_axi_arready),

    .s0_rvalid(lmem_axi_rvalid),
    .s0_rid(lmem_axi_rid),
    .s0_rresp(lmem_axi_rresp),
    .s0_rdata(lmem_axi_rdata),
    .s0_rlast(lmem_axi_rlast),
    .s0_rready(lmem_axi_rready),

    .s0_awvalid(lmem_axi_awvalid),
    .s0_awready(lmem_axi_awready),

    .s0_wvalid(lmem_axi_wvalid),
    .s0_wready(lmem_axi_wready),

    .s0_bresp(lmem_axi_bresp),
    .s0_bvalid(lmem_axi_bvalid),
    .s0_bid(lmem_axi_bid),
    .s0_bready(lmem_axi_bready),


    .s1_arvalid(dma_axi_arvalid),
    .s1_arready(dma_axi_arready),

    .s1_rvalid(dma_axi_rvalid),
    .s1_rresp(dma_axi_rresp),
    .s1_rdata(dma_axi_rdata),
    .s1_rlast(dma_axi_rlast),
    .s1_rready(dma_axi_rready),

    .s1_awvalid(dma_axi_awvalid),
    .s1_awready(dma_axi_awready),

    .s1_wvalid(dma_axi_wvalid),
    .s1_wready(dma_axi_wready),

    .s1_bresp(dma_axi_bresp),
    .s1_bvalid(dma_axi_bvalid),
    .s1_bready(dma_axi_bready)
);


`endif


endmodule
