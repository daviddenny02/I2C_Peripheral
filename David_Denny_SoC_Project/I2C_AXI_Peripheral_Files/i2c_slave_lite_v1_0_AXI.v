// David Denny, 1001915603
// I2C AXI Peripheral
// Based on Losh documentation and the auto-generated AXI file from Vivado

// i2c_slave_lite_v1_0_AXI.sv
`timescale 1ns/1ps
module i2c_slave_lite_v1_0_AXI #
(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 5
)
(
    // AXI4-Lite interface
    input  wire S_AXI_ACLK,
    input  wire S_AXI_ARESETN,
    // Write address channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire [2:0]  S_AXI_AWPROT,
    input  wire        S_AXI_AWVALID,
    output wire        S_AXI_AWREADY,
    // Write data channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire        S_AXI_WVALID,
    output wire        S_AXI_WREADY,
    // Write response
    output wire [1:0]  S_AXI_BRESP,
    output wire        S_AXI_BVALID,
    input  wire        S_AXI_BREADY,
    // Read address channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire [2:0]  S_AXI_ARPROT,
    input  wire        S_AXI_ARVALID,
    output wire        S_AXI_ARREADY,
    // Read data channel
    output wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output wire [1:0]  S_AXI_RRESP,
    output wire        S_AXI_RVALID,
    input  wire        S_AXI_RREADY,
    // Debug LED outputs
    
    // I2C I/O
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

// AXI4-lite signals
reg axi_awready_reg;
reg axi_wready_reg;
reg [1:0] axi_bresp_reg;
reg axi_bvalid_reg;
reg axi_arready_reg;
reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata_reg;
reg [1:0] axi_rresp_reg;
reg axi_rvalid_reg;

assign S_AXI_AWREADY = axi_awready_reg;
assign S_AXI_WREADY  = axi_wready_reg;
assign S_AXI_BRESP   = axi_bresp_reg;
assign S_AXI_BVALID  = axi_bvalid_reg;
assign S_AXI_ARREADY = axi_arready_reg;
assign S_AXI_RDATA   = axi_rdata_reg;
assign S_AXI_RRESP   = axi_rresp_reg;
assign S_AXI_RVALID  = axi_rvalid_reg;

// Address decoding constants
localparam integer ADDR_LSB = (C_S_AXI_DATA_WIDTH/32) + 1;
localparam integer OPT_MEM_ADDR_BITS = 2;

// Slave registers (8 32-bit registers, of which only 5 are used)
// Register map
// ofs  fn
//   0  ADDRESS
//   4  REGISTER
//   8  DATA
//  12  STATUS
//  16  CONTROL
//  20  [UNUSED]
//  24  [UNUSED]
//  28  [UNUSED]
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg0; // ADDRESS
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg1; // REGISTER
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg2; // DATA
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg3; // STATUS
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg4; // CONTROL
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg5;
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg6;
reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg7;

integer byte_index;

// Capture friendly aliases
wire axi_clk    = S_AXI_ACLK;
wire axi_resetn = S_AXI_ARESETN;
wire [31:0] axi_awaddr_wire = S_AXI_AWADDR;
wire axi_awvalid = S_AXI_AWVALID;
wire axi_wvalid  = S_AXI_WVALID;
wire [3:0] axi_wstrb = S_AXI_WSTRB;
wire axi_bready  = S_AXI_BREADY;
wire [31:0] axi_araddr_wire = S_AXI_ARADDR;
wire axi_arvalid = S_AXI_ARVALID;
wire axi_rready  = S_AXI_RREADY;

// Clock signals
wire clk   = axi_clk;
wire reset = ~axi_resetn; // Hardware Controlled by Active-High Reset

// TX FIFO interface signals
reg  [7:0] tx_write_data;
logic      tx_wr_request; // Was PB on FSM
wire [7:0] tx_rd_data;
logic      tx_rd_request;
wire       tx_empty;
wire       tx_full;
wire       tx_overflow;
wire [3:0] tx_wr_index;
wire [3:0] tx_rd_index;
reg        tx_clear_overflow_request; 

// RX FIFO interface signals
reg  [7:0] rx_write_data;
reg        rx_wr_request;
wire [7:0] rx_rd_data;
reg        rx_rd_request;
wire       rx_empty;
wire       rx_full;
wire       rx_overflow;
wire [3:0] rx_wr_index;
wire [3:0] rx_rd_index;
reg        rx_clear_overflow_request = 1'b0;

// Edge detectors for rd_request and wr_request
wire tx_wr_pulse;
wire rx_rd_pulse;

// Edge Detector for TX Writes
edge_detector tx_wr_edge_inst (
    .clk(axi_clk),
    .reset(~axi_resetn),
    .signal(tx_wr_request),
    .edge_out(tx_wr_pulse)
);

// Edge Detector for RX Reads
edge_detector rx_rd_edge_inst (
    .clk(axi_clk),
    .reset(~axi_resetn),
    .signal(rx_rd_request),
    .edge_out(rx_rd_pulse)
);

// TX FIFO Instantiation
fifo tx_fifo_inst
(
    .clk(clk),
    .reset(reset),
    .wr_data(tx_write_data),    // From Linux
    .wr_request(tx_wr_pulse),   // From Linux
    .rd_data(tx_rd_data),       // From FSM
    .rd_request(tx_rd_request), // From FSM
    .empty(tx_empty),
    .full(tx_full),
    .overflow(tx_overflow),
    .clear_overflow_request(tx_clear_overflow_request),
    .wr_index(tx_wr_index),
    .rd_index(tx_rd_index)
);

// RX FIFO Instantiation
fifo rx_fifo_inst
(
    .clk(clk),
    .reset(reset),
    .wr_data(rx_write_data),    // From FSM
    .wr_request(rx_wr_request), // From FSM
    .rd_data(rx_rd_data),       // From Linux
    .rd_request(rx_rd_pulse),   // From Linux
    .empty(rx_empty),
    .full(rx_full),
    .overflow(rx_overflow),
    .clear_overflow_request(rx_clear_overflow_request),
    .wr_index(rx_wr_index),
    .rd_index(rx_rd_index)
);

// I2C FSM Signals
logic       fsm_ack_error;
logic       fsm_busy;
logic       fsm_sda;
logic       fsm_scl;
logic       fsm_test_out;
logic [3:0] fsm_state;
logic [4:0] fsm_phase;
logic       fsm_sda_in; // GPIO Input from Top

// I2C Signals from Internal Registers
logic [6:0] fsm_addr;
logic [7:0] fsm_register;
logic [3:0] fsm_byte_count;
logic       fsm_read_write;
logic       fsm_use_register;
logic       fsm_use_repeated_start;
logic       fsm_start;
logic       fsm_ignore_ack;

// Drive I/O from the peripheral
reg fsm_test_out_enable;
assign ack_error    = fsm_ack_error; 
assign busy         = fsm_busy;
assign sda_out      = fsm_sda;
assign scl          = fsm_scl;
assign test_out     = (fsm_test_out_enable ? fsm_test_out : 1'b0); // Enable test_out based on CONTROL signal
assign state        = fsm_state;
assign phase        = fsm_phase;
assign fsm_sda_in   = sda_in;
assign LED_TX_INDEX = tx_wr_index;
assign LED_RX_INDEX = rx_rd_index;


// I2C FSM Instantiation
i2c_axi_fsm i2c_axi_fsm_inst
(
    .clk(clk),                                   // 100MHz Clock Input from AXI
    .reset(reset),                               // Reset Input from AXI
    .addr(fsm_addr),                             // FSM Address Input from AXI
    .register(fsm_register),                     // FSM Register Input from AXI
    .byte_count(fsm_byte_count),                 // FSM Byte Count Input from AXI
    .read_write(fsm_read_write),                 // FSM R/W Input from AXI
    .use_register(fsm_use_register),             // FSM Use Register Input from AXI
    .use_repeated_start(fsm_use_repeated_start), // FSM Repeated Start Input from AXI
    .start(fsm_start),                           // FSM Start Input from AXI
    .ack_error(fsm_ack_error),                   // FSM ACK Error from FSM                          OUT
    .busy(fsm_busy),                             // FSM Busy Output for ILA                         OUT
    .sda(fsm_sda),                               // SDA Line Output from FSM,              GPIO[17] OUT
    .scl(fsm_scl),                               // SCL Line Output from FSM,              GPIO[19] OUT
    .test_out(fsm_test_out),                     // 200kHz Test Out Clock Output from FSM, GPIO[18] OUT
    .tx_rd_data(tx_rd_data),                     // TX FIFO Read Data Input from AXI
    .tx_rd_request(tx_rd_request),               // TX FIFO Read Request Output from FSM
    .rx_wr_data(rx_write_data),                  // RX FIFO Write Data Output from FSM
    .rx_wr_request(rx_wr_request),               // RX FIFO Write Request Output from FSM
    .ignore_ack(fsm_ignore_ack),                 // Ignore ACK Input from CONTROL Register
    .state_out(fsm_state),                       // FSM State Output for ILA                        OUT
    .phase_out(fsm_phase),                       // FSM Phase Output for ILA                        OUT
    .sda_in(fsm_sda_in)                          // GPIO Input from the Top-Level, copy of SDA      IN
);

reg sticky_tx_overflow;
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) sticky_tx_overflow <= 1'b0;
    else if (tx_clear_overflow_request) sticky_tx_overflow <= 1'b0;
    else sticky_tx_overflow <= sticky_tx_overflow | tx_overflow;
end

reg sticky_rx_overflow;
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) sticky_rx_overflow <= 1'b0;
    else if (rx_clear_overflow_request) sticky_rx_overflow <= 1'b0;
    else sticky_rx_overflow <= sticky_rx_overflow | rx_overflow;
end

reg sticky_fsm_ack_error;

// Assert address ready handshake (axi_awready) 
// - after address is valid (axi_awvalid)
// - after data is valid (axi_wvalid)
// - while configured to receive a write (aw_en)
// De-assert ready (axi_awready)
// - after write response channel ready handshake received (axi_bready)
// - after this module sends write response channel valid (axi_bvalid) 
wire wr_add_data_valid = axi_awvalid && axi_wvalid;

reg aw_en;
reg [C_S_AXI_ADDR_WIDTH-1:0] waddr;

always_ff @(posedge axi_clk) begin
    if (!axi_resetn) begin
        axi_awready_reg <= 1'b0;
        aw_en <= 1'b1;
    end else begin
        if (wr_add_data_valid && ~axi_awready_reg && aw_en) begin
            axi_awready_reg <= 1'b1;
            aw_en <= 1'b0;
        end else if (axi_bready && axi_bvalid_reg) begin
            aw_en <= 1'b1;
            axi_awready_reg <= 1'b0;
        end else begin
            axi_awready_reg <= 1'b0;
        end
    end
end

// Capture the write address (axi_awaddr) in the first clock (~axi_awready)
// - after write address is valid (axi_awvalid)
// - after write data is valid (axi_wvalid)
// - while configured to receive a write (aw_en)
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) waddr <= {C_S_AXI_ADDR_WIDTH{1'b0}};
    else if (wr_add_data_valid && ~axi_awready_reg && aw_en) waddr <= axi_awaddr_wire;
end

// Output write data ready handshake (axi_wready) generation for one clock
// - after address is valid (axi_awvalid)
// - after data is valid (axi_wvalid)
// - while configured to receive a write (aw_en)
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) axi_wready_reg <= 1'b0;
    else axi_wready_reg <= (wr_add_data_valid && ~axi_wready_reg && aw_en);
end

// Single-cycle write event
wire write_accepted = wr_add_data_valid && axi_awready_reg && axi_wready_reg;

// Write data to internal registers
// - after address is valid (axi_awvalid)
// - after write data is valid (axi_wvalid)
// - after this module asserts ready for address handshake (axi_awready)
// - after this module asserts ready for data handshake (axi_wready)
// write correct bytes in 32-bit word based on byte enables (axi_wstrb)
// int_clear_request write is only active for one clock
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) begin
        slv_reg0 <= 32'h0;
        slv_reg1 <= 32'h0;
        slv_reg2 <= 32'h0;
        slv_reg3 <= 32'h0;
        slv_reg4 <= 32'h0;
        slv_reg5 <= 32'h0;
        slv_reg6 <= 32'h0;
        slv_reg7 <= 32'h0;
        tx_wr_request <= 1'b0;
        tx_write_data <= 8'h00;
        tx_clear_overflow_request <= 1'b0;
        sticky_fsm_ack_error <= 1'b0;
    end else begin
        tx_wr_request <= 1'b0;
        fsm_start <= 1'b0; // Clear FSM start condition
        tx_clear_overflow_request <= 1'b0;
        rx_clear_overflow_request <= 1'b0;
        sticky_fsm_ack_error <= sticky_fsm_ack_error | fsm_ack_error;

        if (write_accepted) begin
            case (waddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                3'h0: begin // ADDRESS
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg0[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                            
                    fsm_addr <= S_AXI_WDATA[6:0];
                end

                3'h1: begin // REGISTER
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg1[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                            
                    fsm_register <= S_AXI_WDATA[7:0];
                end

                3'h2: begin // DATA
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg2[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];

                    tx_write_data <= S_AXI_WDATA[7:0];
                    tx_wr_request <= 1'b1;
                end

                3'h3: begin // STATUS
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg3[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];

                    if (S_AXI_WDATA[0]) rx_clear_overflow_request <= 1'b1;
                    if (S_AXI_WDATA[3]) tx_clear_overflow_request <= 1'b1;
                    if (S_AXI_WDATA[6]) sticky_fsm_ack_error <= 1'b0;
                end

                3'h4: begin // CONTROL
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg4[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                            
                    fsm_read_write         <= S_AXI_WDATA[0];
                    fsm_byte_count         <= S_AXI_WDATA[4:1];
                    fsm_use_register       <= S_AXI_WDATA[5];
                    fsm_use_repeated_start <= S_AXI_WDATA[6];
                    fsm_start              <= S_AXI_WDATA[7];
                    fsm_test_out_enable    <= S_AXI_WDATA[8];
                    fsm_ignore_ack         <= S_AXI_WDATA[24]; // Ignore ACK Custom Signal
                end

                3'h5: begin // UNUSED
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg5[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end

                3'h6: begin // UNUSED
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg6[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end

                3'h7: begin // UNUSED
                    for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                        if (axi_wstrb[byte_index])
                            slv_reg7[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end

                default: ;
            endcase
        end
    end
end

// Send write response (axi_bvalid, axi_bresp)
// - after address is valid (axi_awvalid)
// - after write data is valid (axi_wvalid)
// - after this module asserts ready for address handshake (axi_awready)
// - after this module asserts ready for data handshake (axi_wready)
// Clear write response valid (axi_bvalid) after one clock
wire wr_add_data_ready = axi_awready_reg && axi_wready_reg;
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) begin
        axi_bvalid_reg <= 1'b0;
        axi_bresp_reg  <= 2'b0;
    end else begin
        if (wr_add_data_valid && wr_add_data_ready && ~axi_bvalid_reg) begin
            axi_bvalid_reg <= 1'b1;
            axi_bresp_reg  <= 2'b0;
        end else if (S_AXI_BREADY && axi_bvalid_reg) begin
            axi_bvalid_reg <= 1'b0;
        end
    end
end

// In the first clock (~axi_arready) that the read address is valid
// - capture the address (axi_araddr)
// - output ready (axi_arready) for one clock
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) begin
        axi_arready_reg <= 1'b0;
    end else begin
        if (axi_arvalid && ~axi_arready_reg) begin
            axi_arready_reg <= 1'b1;
        end else axi_arready_reg <= 1'b0;
    end
end

reg [C_S_AXI_ADDR_WIDTH-1:0] raddr;
always_ff @(posedge axi_clk) begin
    if (!axi_resetn) raddr <= {C_S_AXI_ADDR_WIDTH{1'b0}};
    else if (axi_arvalid && ~axi_arready_reg) raddr <= axi_araddr_wire;
end

// Update register read data
// - after this module receives a valid address (axi_arvalid)
// - after this module asserts ready for address handshake (axi_arready)
// - before the module asserts the data is valid (~axi_rvalid)
//   (don't change the data while asserting read data is valid)
wire rd_valid = axi_arvalid && axi_arready_reg && ~axi_rvalid_reg;

always_ff @(posedge axi_clk) begin
    if (!axi_resetn) begin
        axi_rdata_reg <= {C_S_AXI_DATA_WIDTH{1'b0}};
        rx_rd_request <= 1'b0;
        axi_rvalid_reg <= 1'b0;
        axi_rresp_reg <= 2'b0;
    end else begin
        // Default clear
        rx_rd_request <= 1'b0;

        if (rd_valid) begin
            case (raddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                3'h0: axi_rdata_reg <= slv_reg0; // ADDRESS
                3'h1: axi_rdata_reg <= slv_reg1; // REGISTER
                3'h2: begin // DATA
                        axi_rdata_reg <= {24'h0, rx_rd_data}; // Reads from DATA Register are from the RX FIFO
                        rx_rd_request <= 1'b1;                // Reads from DATA Register are from the RX FIFO
                      end
                3'h3: begin // STATUS
                        // bit 0   = RX Overflow
                        // bit 1   = RX Full
                        // bit 2   = RX Empty
                        // bit 3   = TX Overflow
                        // bit 4   = TX Full
                        // bit 5   = TX Empty
                        // [11:8]  = TX_RD_INDEX
                        // [15:12] = TX_WR_INDEX,
                        // [19:16] = RX_RD_INDEX,
                        // [23:20] = RX_WR_INDEX
                        axi_rdata_reg <= 32'h00000000;
                        axi_rdata_reg[0]     <= sticky_rx_overflow;
                        axi_rdata_reg[1]     <= rx_full;
                        axi_rdata_reg[2]     <= rx_empty;
                        axi_rdata_reg[3]     <= sticky_tx_overflow;
                        axi_rdata_reg[4]     <= tx_full;
                        axi_rdata_reg[5]     <= tx_empty;
                        axi_rdata_reg[6]     <= sticky_fsm_ack_error;
                        axi_rdata_reg[7]     <= fsm_busy;
                        axi_rdata_reg[11:8]  <= tx_rd_index;
                        axi_rdata_reg[15:12] <= tx_wr_index;
                        axi_rdata_reg[19:16] <= rx_rd_index;
                        axi_rdata_reg[23:20] <= rx_wr_index;
                end
                3'h4: axi_rdata_reg <= slv_reg4; // CONTROL
                3'h5: axi_rdata_reg <= slv_reg5; // UNUSED
                3'h6: axi_rdata_reg <= slv_reg6; // UNUSED
                3'h7: axi_rdata_reg <= slv_reg7; // UNUSED
                default: axi_rdata_reg <= 32'h0;
            endcase

            // Assert data is valid for reading (axi_rvalid)
            // - after address is valid (axi_arvalid)
            // - after this module asserts ready for address handshake (axi_arready)
            // De-assert data valid (axi_rvalid) 
            // - after master ready handshake is received (axi_rready)
            axi_rvalid_reg <= 1'b1;
            axi_rresp_reg  <= 2'b0;
        end else if (axi_rvalid_reg && axi_rready) begin
            axi_rvalid_reg <= 1'b0;
        end
    end
end

always_ff @(posedge axi_clk) begin
    if (!axi_resetn) axi_rresp_reg <= 2'b0;
end

// Assign outputs
assign S_AXI_RRESP = axi_rresp_reg;
assign S_AXI_RDATA = axi_rdata_reg;

// Assign write response signals
assign S_AXI_BRESP  = axi_bresp_reg;
assign S_AXI_BVALID = axi_bvalid_reg;

endmodule