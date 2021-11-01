module axi_interconnect
import el2_pkg::*;
#(
`include "el2_param.vh"
 )
  (


      //-------------------------- LSU AXI signals--------------------------
   // AXI Write Channels
   output  logic                            lsu_axi_awready,
   output  logic                            lsu_axi_wready,
   output  logic                            lsu_axi_bvalid,
   output  logic [1:0]                      lsu_axi_bresp,
   output  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_bid,


   input logic                            lsu_axi_awvalid,
   input logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_awid,
   input logic [31:0]                     lsu_axi_awaddr,
   input logic [3:0]                      lsu_axi_awregion,
   input logic [7:0]                      lsu_axi_awlen,
   input logic [2:0]                      lsu_axi_awsize,
   input logic [1:0]                      lsu_axi_awburst,
   input logic                            lsu_axi_awlock,
   input logic [3:0]                      lsu_axi_awcache,
   input logic [2:0]                      lsu_axi_awprot,
   input logic [3:0]                      lsu_axi_awqos,
   input logic                            lsu_axi_wvalid,
   input logic [63:0]                     lsu_axi_wdata,
   input logic [7:0]                      lsu_axi_wstrb,
   input logic                            lsu_axi_wlast,
   input logic                            lsu_axi_bready,
//////////////////////////////////////////////////////////////
   // AXI Read Channels
   output  logic                            lsu_axi_arready,
   output  logic                            lsu_axi_rvalid,
   output  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_rid,
   output  logic [63:0]                     lsu_axi_rdata,
   output  logic [1:0]                      lsu_axi_rresp,
   output  logic                            lsu_axi_rlast,
//////////////////////////////////////////////////////////////
   output logic                            lsu_axi_arvalid,
   output logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_arid,
   output logic [31:0]                     lsu_axi_araddr,
   output logic [3:0]                      lsu_axi_arregion,
   output logic [7:0]                      lsu_axi_arlen,
   output logic [2:0]                      lsu_axi_arsize,
   output logic [1:0]                      lsu_axi_arburst,
   output logic                            lsu_axi_arlock,
   output logic [3:0]                      lsu_axi_arcache,
   output logic [2:0]                      lsu_axi_arprot,
   output logic [3:0]                      lsu_axi_arqos,
   output logic                            lsu_axi_rready,

   //-------------------------- IFU AXI signals--------------------------
   // AXI Write Channels
   output  logic                            ifu_axi_bvalid,
   output  logic                            ifu_axi_awready,
   output  logic                            ifu_axi_wready,
   output  logic [1:0]                      ifu_axi_bresp,
   output  logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_bid,


/////////////////////////////////////////////////////////////
   input logic                            ifu_axi_awvalid,
   input logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_awid,
   input logic [31:0]                     ifu_axi_awaddr,
   input logic [3:0]                      ifu_axi_awregion,
   input logic [7:0]                      ifu_axi_awlen,
   input logic [2:0]                      ifu_axi_awsize,
   input logic [1:0]                      ifu_axi_awburst,
   input logic                            ifu_axi_awlock,
   input logic [3:0]                      ifu_axi_awcache,
   input logic [2:0]                      ifu_axi_awprot,
   input logic [3:0]                      ifu_axi_awqos,
   input logic                            ifu_axi_wvalid,
   input logic [63:0]                     ifu_axi_wdata,
   input logic [7:0]                      ifu_axi_wstrb,
   input logic                            ifu_axi_wlast,
   input logic                            ifu_axi_bready,
   // AXI Read Channels
  ///////////////////////////////////////////////////////////
  
   output  logic                            ifu_axi_arready,
   output  logic                            ifu_axi_rvalid,
   output  logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_rid,
   output  logic [63:0]                     ifu_axi_rdata,
   output  logic [1:0]                      ifu_axi_rresp,
   output  logic                            ifu_axi_rlast,

///////////////////////////////////////////////////////////////
   output logic                            ifu_axi_arvalid,
   output logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_arid,
   output logic [31:0]                     ifu_axi_araddr,
   output logic [3:0]                      ifu_axi_arregion,
   output logic [7:0]                      ifu_axi_arlen,
   output logic [2:0]                      ifu_axi_arsize,
   output logic [1:0]                      ifu_axi_arburst,
   output logic                            ifu_axi_arlock,
   output logic [3:0]                      ifu_axi_arcache,
   output logic [2:0]                      ifu_axi_arprot,
   output logic [3:0]                      ifu_axi_arqos,
   output logic                            ifu_axi_rready,
   //-------------------------- SB AXI signals--------------------------
   // AXI Write Channels
   output  logic                            sb_axi_awready,
   output  logic                            sb_axi_wready,
   output  logic                            sb_axi_bvalid,
   output  logic [1:0]                      sb_axi_bresp,
   output  logic [pt.SB_BUS_TAG-1:0]        sb_axi_bid,


//////////////////////////////////////////////////////////////////////////
   input logic                            sb_axi_awvalid,
   input logic [pt.SB_BUS_TAG-1:0]        sb_axi_awid,
   input logic [31:0]                     sb_axi_awaddr,
   input logic [3:0]                      sb_axi_awregion,
   input logic [7:0]                      sb_axi_awlen,
   input logic [2:0]                      sb_axi_awsize,
   input logic [1:0]                      sb_axi_awburst,
   input logic                            sb_axi_awlock,
   input logic [3:0]                      sb_axi_awcache,
   input logic [2:0]                      sb_axi_awprot,
   input logic [3:0]                      sb_axi_awqos,
   input logic                            sb_axi_wvalid,
   input logic [63:0]                     sb_axi_wdata,
   input logic [7:0]                      sb_axi_wstrb,
   input logic                            sb_axi_wlast,
   input logic                            sb_axi_bready,
   // AXI Read Channels

   //////////////////////////////////////////////////////////////
   output  logic                            sb_axi_arready,
   output  logic [pt.SB_BUS_TAG-1:0]        sb_axi_rid,
   output  logic [63:0]                     sb_axi_rdata,
   output  logic [1:0]                      sb_axi_rresp,
   output  logic                            sb_axi_rlast,
   output  logic                            sb_axi_rvalid,
   
   
/////////////////////////////////////////////////////////////////
   input logic                            sb_axi_arvalid,
   input logic [pt.SB_BUS_TAG-1:0]        sb_axi_arid,
   input logic [31:0]                     sb_axi_araddr,
   input logic [3:0]                      sb_axi_arregion,
   input logic [7:0]                      sb_axi_arlen,
   input logic [2:0]                      sb_axi_arsize,
   input logic [1:0]                      sb_axi_arburst,
   input logic                            sb_axi_arlock,
   input logic [3:0]                      sb_axi_arcache,
   input logic [2:0]                      sb_axi_arprot,
   input logic [3:0]                      sb_axi_arqos,
   input logic                            sb_axi_rready,











    //-------------------------- LSU AXI signals--------------------------
   
   
   
   
   input  logic                            lsu_axi_arready_o,
   input  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_rid_o,
   input  logic [63:0]                     lsu_axi_rdata_o,
   input  logic [1:0]                      lsu_axi_rresp_o,
   input  logic                            lsu_axi_rlast_o,
   input  logic                            lsu_axi_rvalid_o,
   input  logic [1:0]                      lsu_axi_bresp_o,
   input  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_bid_o,
   input  logic                            lsu_axi_awready_o,
   input  logic                            lsu_axi_wready_o,
   input  logic                            lsu_axi_bvalid_o,

   output logic                            lsu_axi_rready_o,
   output logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_arid_o,
   output logic [31:0]                     lsu_axi_araddr_o,
   output logic [3:0]                      lsu_axi_arregion_o,
   output logic [7:0]                      lsu_axi_arlen_o,
   output logic [2:0]                      lsu_axi_arsize_o,
   output logic [1:0]                      lsu_axi_arburst_o,
   output logic                            lsu_axi_arlock_o,
   output logic [3:0]                      lsu_axi_arcache_o,
   output logic [2:0]                      lsu_axi_arprot_o,
   output logic [3:0]                      lsu_axi_arqos_o,
   output logic                            lsu_axi_arvalid_o,
   output logic                            lsu_axi_awvalid_o,
   output logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_awid_o,
   output logic [31:0]                     lsu_axi_awaddr_o,
   output logic [3:0]                      lsu_axi_awregion_o,
   output logic [7:0]                      lsu_axi_awlen_o,
   output logic [2:0]                      lsu_axi_awsize_o,
   output logic [1:0]                      lsu_axi_awburst_o,
   output logic                            lsu_axi_awlock_o,
   output logic [3:0]                      lsu_axi_awcache_o,
   output logic [2:0]                      lsu_axi_awprot_o,
   output logic [3:0]                      lsu_axi_awqos_o,
   output logic                            lsu_axi_wvalid_o,
   output logic [63:0]                     lsu_axi_wdata_o,
   output logic [7:0]                      lsu_axi_wstrb_o,
   output logic                            lsu_axi_wlast_o,
   output logic                            lsu_axi_bready_o,
   
   

   //-------------------------- IFU AXI signals--------------------------
   
   input  logic                            ifu_axi_awready_o,
   input  logic [1:0]                      ifu_axi_bresp_o,
   input  logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_bid_o,
   input  logic                            ifu_axi_wready_o,
   input  logic                            ifu_axi_bvalid_o,
   input  logic                            ifu_axi_arready_o,
   input  logic                            ifu_axi_rvalid_o,
   input  logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_rid_o,
   input  logic [63:0]                     ifu_axi_rdata_o,
   input  logic [1:0]                      ifu_axi_rresp_o,
   input  logic                            ifu_axi_rlast_o,
   output logic                            ifu_axi_bready_o,
   output logic                            ifu_axi_awvalid_o,
   output logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_awid_o,
   output logic [31:0]                     ifu_axi_awaddr_o,
   output logic [3:0]                      ifu_axi_awregion_o,
   output logic [7:0]                      ifu_axi_awlen_o,
   output logic [2:0]                      ifu_axi_awsize_o,
   output logic [1:0]                      ifu_axi_awburst_o,
   output logic                            ifu_axi_awlock_o,
   output logic [3:0]                      ifu_axi_awcache_o,
   output logic [2:0]                      ifu_axi_awprot_o,
   output logic                            ifu_axi_wlast_o,
   output logic [63:0]                     ifu_axi_wdata_o,
   output logic [7:0]                      ifu_axi_wstrb_o,
   output logic [3:0]                      ifu_axi_awqos_o,
   output logic                            ifu_axi_wvalid_o,
   output logic                            ifu_axi_arvalid_o,
   output logic [pt.IFU_BUS_TAG-1:0]       ifu_axi_arid_o,
   output logic [31:0]                     ifu_axi_araddr_o,
   output logic [3:0]                      ifu_axi_arregion_o,
   output logic [7:0]                      ifu_axi_arlen_o,
   output logic [2:0]                      ifu_axi_arsize_o,
   output logic [1:0]                      ifu_axi_arburst_o,
   output logic                            ifu_axi_arlock_o,
   output logic [3:0]                      ifu_axi_arcache_o,
   output logic [2:0]                      ifu_axi_arprot_o,
   output logic [3:0]                      ifu_axi_arqos_o,
   output logic                            ifu_axi_rready_o,
   
   

   

   
   
   
   

   
   

   //-------------------------- SB AXI signals--------------------------

   
   
   
   input  logic                            sb_axi_awready_o,
   input  logic                            sb_axi_bvalid_o,
   input  logic [1:0]                      sb_axi_bresp_o,
   input  logic [pt.SB_BUS_TAG-1:0]        sb_axi_bid_o,
   input  logic                            sb_axi_wready_o,
   input  logic                            sb_axi_rvalid_o,
   input  logic                            sb_axi_arready_o,
   input  logic [pt.SB_BUS_TAG-1:0]        sb_axi_rid_o,
   input  logic [63:0]                     sb_axi_rdata_o,
   input  logic [1:0]                      sb_axi_rresp_o,
   input  logic                            sb_axi_rlast_o,
   
   
   output logic                            sb_axi_bready_o,
   output logic                            sb_axi_awvalid_o,
   output logic [pt.SB_BUS_TAG-1:0]        sb_axi_awid_o,
   output logic [31:0]                     sb_axi_awaddr_o,
   output logic [3:0]                      sb_axi_awregion_o,
   output logic [7:0]                      sb_axi_awlen_o,
   output logic [2:0]                      sb_axi_awsize_o,
   output logic [1:0]                      sb_axi_awburst_o,
   output logic                            sb_axi_awlock_o,
   output logic [3:0]                      sb_axi_awcache_o,
   output logic [2:0]                      sb_axi_awprot_o,
   output logic [3:0]                      sb_axi_awqos_o,
   output logic                            sb_axi_wvalid_o,
   output logic [63:0]                     sb_axi_wdata_o,
   output logic [7:0]                      sb_axi_wstrb_o,
   output logic                            sb_axi_wlast_o,
   output logic                            sb_axi_arvalid_o,
   output logic [pt.SB_BUS_TAG-1:0]        sb_axi_arid_o,
   output logic [31:0]                     sb_axi_araddr_o,
   output logic [3:0]                      sb_axi_arregion_o,
   output logic [7:0]                      sb_axi_arlen_o,
   output logic [2:0]                      sb_axi_arsize_o,
   output logic [1:0]                      sb_axi_arburst_o,
   output logic                            sb_axi_arlock_o,
   output logic [3:0]                      sb_axi_arcache_o,
   output logic [2:0]                      sb_axi_arprot_o,
   output logic [3:0]                      sb_axi_arqos_o,
   output logic                            sb_axi_rready_o,



   //-------------------------- LSU AXI signals--------------------------
   
   
   
   
   input  logic                            lsu_axi_arready_B_o,
   input  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_rid_B_o,
   input  logic [63:0]                     lsu_axi_rdata_B_o,
   input  logic [1:0]                      lsu_axi_rresp_B_o,
   input  logic                            lsu_axi_rlast_B_o,
   input  logic                            lsu_axi_rvalid_B_o,
   input  logic [1:0]                      lsu_axi_bresp_B_o,
   input  logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_bid_B_o,
   input  logic                            lsu_axi_awready_B_o,
   input  logic                            lsu_axi_wready_B_o,
   input  logic                            lsu_axi_bvalid_B_o,

   output logic                            lsu_axi_rready_B_o,
   output logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_arid_B_o,
   output logic [31:0]                     lsu_axi_araddr_B_o,
   output logic [3:0]                      lsu_axi_arregion_B_o,
   output logic [7:0]                      lsu_axi_arlen_B_o,
   output logic [2:0]                      lsu_axi_arsize_B_o,
   output logic [1:0]                      lsu_axi_arburst_B_o,
   output logic                            lsu_axi_arlock_B_o,
   output logic [3:0]                      lsu_axi_arcache_B_o,
   output logic [2:0]                      lsu_axi_arprot_B_o,
   output logic [3:0]                      lsu_axi_arqos_B_o,
   output logic                            lsu_axi_arvalid_B_o,
   output logic                            lsu_axi_awvalid_B_o,
   output logic [pt.LSU_BUS_TAG-1:0]       lsu_axi_awid_B_o,
   output logic [31:0]                     lsu_axi_awaddr_B_o,
   output logic [3:0]                      lsu_axi_awregion_B_o,
   output logic [7:0]                      lsu_axi_awlen_B_o,
   output logic [2:0]                      lsu_axi_awsize_B_o,
   output logic [1:0]                      lsu_axi_awburst_B_o,
   output logic                            lsu_axi_awlock_B_o,
   output logic [3:0]                      lsu_axi_awcache_B_o,
   output logic [2:0]                      lsu_axi_awprot_B_o,
   output logic [3:0]                      lsu_axi_awqos_B_o,
   output logic                            lsu_axi_wvalid_B_o,
   output logic [63:0]                     lsu_axi_wdata_B_o,
   output logic [7:0]                      lsu_axi_wstrb_B_o,
   output logic                            lsu_axi_wlast_B_o,
   output logic                            lsu_axi_bready_B_o
   

);
 
 localparam EXTERNAL_MEM_RANGE_low  =  32'h7fffffff;
 localparam EXTERNAL_MEM_RANGE_high =  32'hffffffff; 
 localparam BRIDGE_RANGE_low        =  32'h20000000;
 localparam BRIDGE_RANGE_high       =  32'h80000000;





always_comb begin
    
         
           ifu_axi_arready = ifu_axi_arready_o;    
           ifu_axi_rid     = ifu_axi_rid_o;    
           ifu_axi_rdata   = ifu_axi_rdata_o;   
           ifu_axi_rresp   = ifu_axi_rresp_o;   
           ifu_axi_rlast   = ifu_axi_rlast_o;   
           ifu_axi_rvalid  = ifu_axi_rvalid_o;   
           ifu_axi_bresp   = ifu_axi_bresp_o;   
           ifu_axi_bid     = ifu_axi_bid_o;    
           ifu_axi_awready = ifu_axi_awready_o;   
           ifu_axi_wready  = ifu_axi_wready_o;   
           ifu_axi_bvalid  = ifu_axi_bvalid_o;   
           
           ifu_axi_rready_o     = ifu_axi_rready   ;
           ifu_axi_arid_o       = ifu_axi_arid     ;
           ifu_axi_araddr_o     = ifu_axi_araddr   ;
           ifu_axi_arregion_o   = ifu_axi_arregion ;
           ifu_axi_arlen_o      = ifu_axi_arlen    ;
           ifu_axi_arsize_o     = ifu_axi_arsize   ;
           ifu_axi_arburst_o    = ifu_axi_arburst  ;
           ifu_axi_arlock_o     = ifu_axi_arlock   ;
           ifu_axi_arcache_o    = ifu_axi_arcache  ;
           ifu_axi_arprot_o     = ifu_axi_arprot   ;
           ifu_axi_arqos_o      = ifu_axi_arqos    ;                      
           ifu_axi_arvalid_o    = ifu_axi_arvalid  ;
           ifu_axi_awvalid_o    = ifu_axi_awvalid  ;
           ifu_axi_awid_o       = ifu_axi_awid     ;
           ifu_axi_awaddr_o     = ifu_axi_awaddr   ;
           ifu_axi_awregion_o   = ifu_axi_awregion ;
           ifu_axi_awlen_o      = ifu_axi_awlen    ;
           ifu_axi_awsize_o     = ifu_axi_awsize   ;
           ifu_axi_awburst_o    = ifu_axi_awburst  ;
           ifu_axi_awlock_o     = ifu_axi_awlock   ;
           ifu_axi_awcache_o    = ifu_axi_awcache  ;
           ifu_axi_awprot_o     = ifu_axi_awprot   ;
           ifu_axi_awqos_o      = ifu_axi_awqos    ;
           ifu_axi_wvalid_o     = ifu_axi_wvalid   ;
           ifu_axi_wdata_o      = ifu_axi_wdata    ;
           ifu_axi_wstrb_o      = ifu_axi_wstrb    ;
           ifu_axi_wlast_o      = ifu_axi_wlast    ;
           ifu_axi_bready_o     = ifu_axi_bready   ;
           
           
           
           
           
           sb_axi_arready = sb_axi_arready_o;    
           sb_axi_rid   = sb_axi_rid_o;    
           sb_axi_rdata   = sb_axi_rdata_o;   
           sb_axi_rresp   = sb_axi_rresp_o;   
           sb_axi_rlast   = sb_axi_rlast_o;   
           sb_axi_rvalid  = sb_axi_rvalid_o;   
           sb_axi_bresp   = sb_axi_bresp_o;   
           sb_axi_bid   = sb_axi_bid_o;    
           sb_axi_awready = sb_axi_awready_o;   
           sb_axi_wready  = sb_axi_wready_o;   
           sb_axi_bvalid  = sb_axi_bvalid_o;   
           
           sb_axi_rready_o     = sb_axi_rready   ;
           sb_axi_arid_o       = sb_axi_arid     ;
           sb_axi_araddr_o     = sb_axi_araddr   ;
           sb_axi_arregion_o   = sb_axi_arregion ;
           sb_axi_arlen_o      = sb_axi_arlen    ;
           sb_axi_arsize_o     = sb_axi_arsize   ;
           sb_axi_arburst_o    = sb_axi_arburst  ;
           sb_axi_arlock_o     = sb_axi_arlock   ;
           sb_axi_arcache_o    = sb_axi_arcache  ;
           sb_axi_arprot_o     = sb_axi_arprot   ;
           sb_axi_arqos_o      = sb_axi_arqos    ;                      
           sb_axi_arvalid_o    = sb_axi_arvalid  ;
           sb_axi_awvalid_o    = sb_axi_awvalid  ;
           sb_axi_awid_o       = sb_axi_awid     ;
           sb_axi_awaddr_o     = sb_axi_awaddr   ;
           sb_axi_awregion_o   = sb_axi_awregion ;
           sb_axi_awlen_o      = sb_axi_awlen    ;
           sb_axi_awsize_o     = sb_axi_awsize   ;
           sb_axi_awburst_o    = sb_axi_awburst  ;
           sb_axi_awlock_o     = sb_axi_awlock   ;
           sb_axi_awcache_o    = sb_axi_awcache  ;
           sb_axi_awprot_o     = sb_axi_awprot   ;
           sb_axi_awqos_o      = sb_axi_awqos    ;
           sb_axi_wvalid_o     = sb_axi_wvalid   ;
           sb_axi_wdata_o      = sb_axi_wdata    ;
           sb_axi_wstrb_o      = sb_axi_wstrb    ;
           sb_axi_wlast_o      = sb_axi_wlast    ;
           sb_axi_bready_o     = sb_axi_bready   ;
    
    if ( ((lsu_axi_araddr > EXTERNAL_MEM_RANGE_low) && (lsu_axi_araddr < EXTERNAL_MEM_RANGE_high))/* ||  ((lsu_axi_awaddr > EXTERNAL_MEM_RANGE_low) && (lsu_axi_awaddr <= EXTERNAL_MEM_RANGE_high)) */ ||   ((ifu_axi_araddr > EXTERNAL_MEM_RANGE_low) && (ifu_axi_araddr < EXTERNAL_MEM_RANGE_high))  || /* ((ifu_axi_awaddr > EXTERNAL_MEM_RANGE_low) && (ifu_axi_awaddr <= EXTERNAL_MEM_RANGE_high)) || */  ((sb_axi_araddr > EXTERNAL_MEM_RANGE_low) && (sb_axi_araddr < EXTERNAL_MEM_RANGE_high)) /*||  ((sb_axi_awaddr > EXTERNAL_MEM_RANGE_low) && (sb_axi_awaddr <= EXTERNAL_MEM_RANGE_high) )*/   ) begin    
           
           lsu_axi_arready = lsu_axi_arready_o;    
           lsu_axi_rid   = lsu_axi_rid_o;    
           lsu_axi_rdata   = lsu_axi_rdata_o;   
           lsu_axi_rresp   = lsu_axi_rresp_o;   
           lsu_axi_rlast   = lsu_axi_rlast_o;   
           lsu_axi_rvalid  = lsu_axi_rvalid_o;   
           lsu_axi_bresp   = lsu_axi_bresp_o;   
           lsu_axi_bid   = lsu_axi_bid_o;    
           lsu_axi_awready = lsu_axi_awready_o;   
           lsu_axi_wready  = lsu_axi_wready_o;   
           lsu_axi_bvalid  = lsu_axi_bvalid_o;   
           
           lsu_axi_rready_o     = lsu_axi_rready   ;
           lsu_axi_arid_o       = lsu_axi_arid     ;
           lsu_axi_araddr_o     = lsu_axi_araddr   ;
           lsu_axi_arregion_o   = lsu_axi_arregion ;
           lsu_axi_arlen_o      = lsu_axi_arlen    ;
           lsu_axi_arsize_o     = lsu_axi_arsize   ;
           lsu_axi_arburst_o    = lsu_axi_arburst  ;
           lsu_axi_arlock_o     = lsu_axi_arlock   ;
           lsu_axi_arcache_o    = lsu_axi_arcache  ;
           lsu_axi_arprot_o     = lsu_axi_arprot   ;
           lsu_axi_arqos_o      = lsu_axi_arqos    ;                      
           lsu_axi_arvalid_o    = lsu_axi_arvalid  ;
           lsu_axi_awvalid_o    = lsu_axi_awvalid  ;
           lsu_axi_awid_o       = lsu_axi_awid     ;
           lsu_axi_awaddr_o     = lsu_axi_awaddr   ;
           lsu_axi_awregion_o   = lsu_axi_awregion ;
           lsu_axi_awlen_o      = lsu_axi_awlen    ;
           lsu_axi_awsize_o     = lsu_axi_awsize   ;
           lsu_axi_awburst_o    = lsu_axi_awburst  ;
           lsu_axi_awlock_o     = lsu_axi_awlock   ;
           lsu_axi_awcache_o    = lsu_axi_awcache  ;
           lsu_axi_awprot_o     = lsu_axi_awprot   ;
           lsu_axi_awqos_o      = lsu_axi_awqos    ;
           lsu_axi_wvalid_o     = lsu_axi_wvalid   ;
           lsu_axi_wdata_o      = lsu_axi_wdata    ;
           lsu_axi_wstrb_o      = lsu_axi_wstrb    ;
           lsu_axi_wlast_o      = lsu_axi_wlast    ;
           lsu_axi_bready_o     = lsu_axi_bready   ;     
           
        
    end
    
    

    

    if  ((lsu_axi_araddr > BRIDGE_RANGE_low) && (lsu_axi_araddr < BRIDGE_RANGE_high)) begin 
          
          lsu_axi_arready = lsu_axi_arready_B_o;    
           lsu_axi_rid   = lsu_axi_rid_B_o;    
           lsu_axi_rdata   = lsu_axi_rdata_B_o;   
           lsu_axi_rresp   = lsu_axi_rresp_B_o;   
           lsu_axi_rlast   = lsu_axi_rlast_B_o;   
           lsu_axi_rvalid  = lsu_axi_rvalid_B_o;   
           lsu_axi_bresp   = lsu_axi_bresp_B_o;   
           lsu_axi_bid   = lsu_axi_bid_B_o;    
           lsu_axi_awready = lsu_axi_awready_B_o;   
           lsu_axi_wready  = lsu_axi_wready_B_o;   
           lsu_axi_bvalid  = lsu_axi_bvalid_B_o;   
           
           lsu_axi_rready_B_o     = lsu_axi_rready   ;
           lsu_axi_arid_B_o       = lsu_axi_arid     ;
           lsu_axi_araddr_B_o     = lsu_axi_araddr   ;
           lsu_axi_arregion_B_o   = lsu_axi_arregion ;
           lsu_axi_arlen_B_o      = lsu_axi_arlen    ;
           lsu_axi_arsize_B_o     = lsu_axi_arsize   ;
           lsu_axi_arburst_B_o    = lsu_axi_arburst  ;
           lsu_axi_arlock_B_o     = lsu_axi_arlock   ;
           lsu_axi_arcache_B_o    = lsu_axi_arcache  ;
           lsu_axi_arprot_B_o     = lsu_axi_arprot   ;
           lsu_axi_arqos_B_o      = lsu_axi_arqos    ;                      
           lsu_axi_arvalid_B_o    = lsu_axi_arvalid  ;
           lsu_axi_awvalid_B_o    = lsu_axi_awvalid  ;
           lsu_axi_awid_B_o       = lsu_axi_awid     ;
           lsu_axi_awaddr_B_o     = lsu_axi_awaddr   ;
           lsu_axi_awregion_B_o   = lsu_axi_awregion ;
           lsu_axi_awlen_B_o      = lsu_axi_awlen    ;
           lsu_axi_awsize_B_o     = lsu_axi_awsize   ;
           lsu_axi_awburst_B_o    = lsu_axi_awburst  ;
           lsu_axi_awlock_B_o     = lsu_axi_awlock   ;
           lsu_axi_awcache_B_o    = lsu_axi_awcache  ;
           lsu_axi_awprot_B_o     = lsu_axi_awprot   ;
           lsu_axi_awqos_B_o      = lsu_axi_awqos    ;
           lsu_axi_wvalid_B_o     = lsu_axi_wvalid   ;
           lsu_axi_wdata_B_o      = lsu_axi_wdata    ;
           lsu_axi_wstrb_B_o      = lsu_axi_wstrb    ;
           lsu_axi_wlast_B_o      = lsu_axi_wlast    ;
           lsu_axi_bready_B_o     = lsu_axi_bready   ;     
           
        

 end

end

    
endmodule
