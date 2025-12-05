`timescale 1 ns / 1 ps

module i2c #
(
    // Parameters of Axi Interface
    parameter integer C_AXI_DATA_WIDTH = 32,
    parameter integer C_AXI_ADDR_WIDTH = 5
)
(
    // AXI slave interface (exposed on the IP)
    input  wire                             axi_aclk,
    input  wire                             axi_aresetn,
    input  wire [C_AXI_ADDR_WIDTH-1 : 0]    axi_awaddr,
    input  wire [2 : 0]                     axi_awprot,
    input  wire                             axi_awvalid,
    output wire                             axi_awready,
    input  wire [C_AXI_DATA_WIDTH-1 : 0]    axi_wdata,
    input  wire [(C_AXI_DATA_WIDTH/8)-1 : 0] axi_wstrb,
    input  wire                             axi_wvalid,
    output wire                             axi_wready,
    output wire [1 : 0]                     axi_bresp,
    output wire                             axi_bvalid,
    input  wire                             axi_bready,
    input  wire [C_AXI_ADDR_WIDTH-1 : 0]    axi_araddr,
    input  wire [2 : 0]                     axi_arprot,
    input  wire                             axi_arvalid,
    output wire                             axi_arready,
    output wire [C_AXI_DATA_WIDTH-1 : 0]    axi_rdata,
    output wire [1 : 0]                     axi_rresp,
    output wire                             axi_rvalid,
    input  wire                             axi_rready,
    // User Ports
    output wire       ack_error,
    output wire       busy,
    output wire       sda_out,
    output wire       scl,
    output wire       test_out,
    output wire [3:0] state,
    output wire [4:0] phase,
    input  wire       sda_in,
    output wire [3:0] LED_TX_INDEX,
    output wire [3:0] LED_RX_INDEX
);

    // Instantiate the AXI implementation (the register + FIFO logic)
    i2c_slave_lite_v1_0_AXI # (
        .C_S_AXI_DATA_WIDTH(C_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_AXI_ADDR_WIDTH)
    ) i2c_slave_lite_v1_0_AXI_inst (
        .S_AXI_ACLK   (axi_aclk),
        .S_AXI_ARESETN(axi_aresetn),
        .S_AXI_AWADDR (axi_awaddr),
        .S_AXI_AWPROT (axi_awprot),
        .S_AXI_AWVALID(axi_awvalid),
        .S_AXI_AWREADY(axi_awready),
        .S_AXI_WDATA  (axi_wdata),
        .S_AXI_WSTRB  (axi_wstrb),
        .S_AXI_WVALID (axi_wvalid),
        .S_AXI_WREADY (axi_wready),
        .S_AXI_BRESP  (axi_bresp),
        .S_AXI_BVALID (axi_bvalid),
        .S_AXI_BREADY (axi_bready),
        .S_AXI_ARADDR (axi_araddr),
        .S_AXI_ARPROT (axi_arprot),
        .S_AXI_ARVALID(axi_arvalid),
        .S_AXI_ARREADY(axi_arready),
        .S_AXI_RDATA  (axi_rdata),
        .S_AXI_RRESP  (axi_rresp),
        .S_AXI_RVALID (axi_rvalid),
        .S_AXI_RREADY (axi_rready),
        // User Ports
        .ack_error(ack_error),
        .busy(busy),
        .sda_out(sda_out),
        .scl(scl),
        .test_out(test_out),
        .state(state),
        .phase(phase),
        .sda_in(sda_in),
        .LED_TX_INDEX (LED_TX_INDEX),
        .LED_RX_INDEX (LED_RX_INDEX)
    );
endmodule