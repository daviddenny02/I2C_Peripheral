// David Denny, 1001915603
// I2C FSM Testbench

module FSM (
    input  CLK100,
    output [9:0] LED,
    output [2:0] RGB0,
    output [2:0] RGB1,
    output [3:0] SS_ANODE,
    output [7:0] SS_CATHODE,
    input [11:0] SW,
    input [3:0] PB,
    inout  tri [23:0] GPIO,
    output [3:0] SERVO,
    output PDM_SPEAKER,
    input PDM_MIC_DATA,
    output PDM_MIC_CLK,
    output ESP32_UART1_TXD,
    input ESP32_UART1_RXD,
    output IMU_SCLK,
    output IMU_SDI,
    input IMU_SDO_AG,
    input IMU_SDO_M,
    output IMU_CS_AG,
    output IMU_CS_M,
    input IMU_DRDY_M,
    input IMU_INT1_AG,
    input IMU_INT_M,
    output IMU_DEN_AG
);

    // Tie-offs (unchanged)
    assign RGB0 = 3'b000;
    assign RGB1 = 3'b000;
    assign SS_ANODE = 4'b0111;
    assign SS_CATHODE = 8'b11111111;
    assign SERVO = 4'b0000;
    assign PDM_SPEAKER = 1'b0;
    assign PDM_MIC_CLK = 1'b0;
    assign ESP32_UART1_TXD = 1'b0;
    assign IMU_SCLK = 1'b0;
    assign IMU_SDI = 1'b0;
    assign IMU_CS_AG = 1'b1;
    assign IMU_CS_M  = 1'b1;
    assign IMU_DEN_AG = 1'b0;

    // Buffered clock for internal logic/ILA
    wire clk;
    (* DONT_TOUCH = "TRUE", keep = "true" *) BUFG bufg_inst(.I(CLK100), .O(clk));

    // ------------------ PB synchronizers & rising-edge detectors -----------------------
    // handle input metastability safely
    reg pb; // Perform a Read/Write
    reg pre_pb;
    always_ff @ (posedge(clk))
    begin
        pre_pb <= PB[0];
        pb <= pre_pb;
    end
    // handle input metastability safely
    reg wr_request_pb; // Write a byte to the TX FIFO
    reg pre_wr_request_pb;
    always_ff @ (posedge(clk))
    begin
        pre_wr_request_pb <= PB[1];
        wr_request_pb <= pre_wr_request_pb;
    end
    // handle input metastability safely
    reg rd_request_pb; // Read a byte from the TX FIFO -- UNUSED!
    reg pre_rd_request_pb;
    always_ff @ (posedge(clk))
    begin
        pre_rd_request_pb <= PB[2];
        rd_request_pb <= pre_rd_request_pb;
    end
    // handle input metastability safely
    reg reset; // Reset everything related to the FSM
    reg pre_reset;
    always_ff @ (posedge(clk))
    begin
        pre_reset <= PB[3];
        reset <= pre_reset;
    end
    
    // --------------------- TX FIFO Variables -----------------------------
    wire [7:0] tx_write_data = 8'h66;
    wire [7:0] tx_rd_data;
    wire       tx_rd_request;
    wire       tx_empty;
    wire       tx_full;
    reg        tx_overflow;
    wire       tx_clear_overflow_request = 1'b0;
    reg  [3:0] tx_wr_index;
    reg  [3:0] tx_rd_index;
    
    // --------------------- RX FIFO Variables -----------------------------
    wire [7:0] rx_write_data;
    wire       rx_wr_request;
    wire [7:0] rx_rd_data;
    wire       rx_rd_request;
    wire       rx_empty;
    wire       rx_full;
    reg        rx_overflow;
    wire       rx_clear_overflow_request = 1'b0;
    reg  [3:0] rx_wr_index;
    reg  [3:0] rx_rd_index;
    
    // Edge Detector for TX Writes
    //   Given how the Control Signal this represents is a W1C bit,
    //   this edge should not be required when communicating over the
    //   AXI bus
    edge_detector tx_wr_edge_inst (
        .clk(clk),
        .reset(reset),
        .signal(wr_request_pb),
        .edge_out(tx_fifo_wr_pulse)
    );

    // --------------------- TX FIFO Instantiation -----------------------------
    fifo tx_fifo_inst
    (
        .clk(clk),
        .reset(reset),
        .wr_data(tx_write_data),
        .wr_request(tx_fifo_wr_pulse),
        .rd_data(tx_rd_data),
        .rd_request(tx_rd_request),
        .empty(tx_empty),
        .full(tx_full),
        .overflow(tx_overflow),
        .clear_overflow_request(tx_clear_overflow_request),
        .wr_index(tx_wr_index),
        .rd_index(tx_rd_index)
    );
    
    // --------------------- RX FIFO Instantiation -----------------------------
    fifo rx_fifo_inst
    (
        .clk(clk),
        .reset(reset),
        .wr_data(rx_write_data),
        .wr_request(rx_wr_request),
        .rd_data(rx_rd_data),
        .rd_request(rx_rd_request),
        .empty(rx_empty),
        .full(rx_full),
        .overflow(rx_overflow),
        .clear_overflow_request(rx_clear_overflow_request),
        .wr_index(rx_wr_index),
        .rd_index(rx_rd_index)
    );

    // ------------------ Shared I2C bus nets (tri-state signals) ------------------
    tri sda_bus;
    tri scl_bus;

    // ------------- Hard-coded control inputs for the i2c_fsm instantiation ----------------
    localparam logic [6:0]  HARD_ADDR = 7'h20;       // 7-bit device address
    localparam logic [7:0]  HARD_REG  = 8'h09;       // Register address used for complex reads/writes
    localparam logic [3:0]  HARD_BYTE_COUNT = 4'd0;  // 0 single byte, >0 = multiple bytes
    localparam logic        HARD_READ_WRITE = 1'b0;  // 0 = Write, 1 = Read
    localparam logic        HARD_USE_REGISTER = 1'b1;// Complex Reads/Writes
    localparam logic        HARD_USE_REPSTART = 1'b0;// Repeated Start Flags
    localparam logic        HARD_IGNORE_ACK = 1'b1;  // Ignore ACK Signal

    logic fsm_ack_error;
    logic fsm_busy;
    logic fsm_sda;
    logic fsm_scl;
    logic fsm_test_out;
    logic [3:0] fsm_state;
    logic [4:0] fsm_phase;
    logic fsm_sda_in;
    
    // SDA line used for acks
    //   For stability purposes, the SDA line will be interpreted as just an output
    //   within the FSM itself, which requires a distinct input to the FSM based on
    //   a copy of the raw SDA/GPIO line:
    assign fsm_sda_in = GPIO[17];

    i2c_fsm i2c_inst
    (
        .clk(clk),
        .reset(reset),
        .addr(HARD_ADDR),
        .register(HARD_REG),
        .byte_count(HARD_BYTE_COUNT),
        .read_write(HARD_READ_WRITE),
        .use_register(HARD_USE_REGISTER),
        .use_repeated_start(HARD_USE_REPSTART),
        .start(pb),
        .ack_error(fsm_ack_error),
        .busy(fsm_busy),
        .sda(fsm_sda),
        .scl(fsm_scl),
        .test_out(fsm_test_out),
        .tx_rd_data(tx_rd_data),
        .tx_rd_request(tx_rd_request),
        .rx_wr_data(rx_write_data),
        .rx_wr_request(rx_wr_request),
        .ignore_ack(HARD_IGNORE_ACK),
        .state_out(fsm_state),
        .phase_out(fsm_phase),
        .sda_in(fsm_sda_in)
    );

    // Map FSM outputs onto tri busses.
    //   i2c_fsm SDA/SCL are logic outputs. For open-drain behaviour we typically
    //   want the FSM to drive 0 or 'z'.
    assign sda_bus = fsm_sda;
    assign scl_bus = fsm_scl;

    // Export test_out to the GPIO (200kHz tick)
    assign GPIO[18] = fsm_test_out;

    // Connect SDA / SCL to the external pads (tri-state)
    assign GPIO[17] = sda_bus;
    assign GPIO[19] = scl_bus;

    // ------------------ ILA probes -------------------
    ila_1 ila_inst (
        .clk(clk),
        .probe0(fsm_sda),      // SDA bus
        .probe1(fsm_scl),      // SCL bus
        .probe2(fsm_test_out), // 200kHz tick from FSM
        .probe3(PB[0]),        // PB0 raw
        .probe4(PB[1]),        // PB1 raw
        .probe5(fsm_busy),     // FSM Busy
        .probe6(fsm_state),    // FSM state
        .probe7(fsm_phase)     // FSM phase
    );

    // LED outputs used for debugging/demonstration purposes
    assign LED[3:0] = tx_wr_index; // Reflects the number of bytes transmitted using the FSM
    assign LED[7:4] = rx_rd_index; // Reflects the number of bytes read using the FSM
    assign LED[8]   = tx_empty;    // Reflects the empty flag of the TX FIFO
    assign LED[9]   = rx_empty;    // Reflects the empty flag of the RX FIFO

endmodule