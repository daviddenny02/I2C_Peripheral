// David Denny, 1001915603
// I2C Project

// i2c_system_top.sv
`timescale 1 ns / 1 ps

module i2c_system_top (
    input  CLK100,
    output [9:0] LED,       // RGB1, RGB0, LED 9..0 placed from left to right
    output [2:0] RGB0,      
    output [2:0] RGB1,
    output [3:0] SS_ANODE,   // Anodes 3..0 placed from left to right
    output [7:0] SS_CATHODE, // Bit order: DP, G, F, E, D, C, B, A
    input [11:0] SW,         // SWs 11..0 placed from left to right
    input [3:0] PB,          // PBs 3..0 placed from left to right
    inout [23:0] GPIO,       // PMODA-C 1P, 1N, ... 3P, 3N order
    output [3:0] SERVO,      // Servo outputs
    output PDM_SPEAKER,      // PDM signals for mic and speaker
    input  PDM_MIC_DATA,      
    output PDM_MIC_CLK,
    output ESP32_UART1_TXD,  // WiFi/Bluetooth serial interface 1
    input  ESP32_UART1_RXD,
    output IMU_SCLK,         // IMU spi clk
    output IMU_SDI,          // IMU spi data input
    input  IMU_SDO_AG,       // IMU spi data output (accel/gyro)
    input  IMU_SDO_M,        // IMU spi data output (mag)
    output IMU_CS_AG,        // IMU cs (accel/gyro) 
    output IMU_CS_M,         // IMU cs (mag)
    input  IMU_DRDY_M,       // IMU data ready (mag)
    input  IMU_INT1_AG,      // IMU interrupt (accel/gyro)
    input  IMU_INT_M,        // IMU interrupt (mag)
    output IMU_DEN_AG,       // IMU data enable (accel/gyro)

    inout [14:0] DDR_addr,
    inout [2:0]  DDR_ba,
    inout        DDR_cas_n,
    inout        DDR_ck_n,
    inout        DDR_ck_p,
    inout        DDR_cke,
    inout        DDR_cs_n,
    inout [3:0]  DDR_dm,
    inout [31:0] DDR_dq,
    inout [3:0]  DDR_dqs_n,
    inout [3:0]  DDR_dqs_p,
    inout        DDR_odt,
    inout        DDR_ras_n,
    inout        DDR_reset_n,
    inout        DDR_we_n,
    inout        FIXED_IO_ddr_vrn,
    inout        FIXED_IO_ddr_vrp,
    inout [53:0] FIXED_IO_mio,
    inout        FIXED_IO_ps_clk,
    inout        FIXED_IO_ps_porb,
    inout        FIXED_IO_ps_srstb
    );

    // Terminate all of the unused outputs or i/o's
    // assign LED = 10'b0000000000;
    assign RGB0 = 3'b000;
    assign RGB1 = 3'b000;
    // assign SS_ANODE = 4'b0000;
    // assign SS_CATHODE = 8'b11111111;GPIO
    assign GPIO = 24'bzzzzzzzzzzzzzzzzzzzzzzzz;
    assign SERVO = 4'b0000;
    assign PDM_SPEAKER = 1'b0;
    assign PDM_MIC_CLK = 1'b0;
    assign ESP32_UART1_TXD = 1'b0;
    assign IMU_SCLK = 1'b0;
    assign IMU_SDI = 1'b0;
    assign IMU_CS_AG = 1'b1;
    assign IMU_CS_M = 1'b1;
    assign IMU_DEN_AG = 1'b0;
    
    // display I (I2C) on left seven segment display
    assign SS_ANODE = 4'b0111;
    assign SS_CATHODE = 8'b11111001;
    
    // Buffered clock for internal logic/ILA
    wire clk;
    (* DONT_TOUCH = "TRUE", keep = "true" *) BUFG bufg_inst(.I(CLK100), .O(clk));
    
        // ---------------------------------------------------------------------
    // Internal wires for signals exported from the system_wrapper instance
    // ---------------------------------------------------------------------
    wire [3:0] led_rx_index;
    wire [3:0] led_tx_index;

    // wires driven by the system instance (AXI peripheral)
    wire       ack_error;
    wire       busy;
    wire [4:0] phase;
    wire       scl;
    wire       sda_out;   // FSM requests: 1 -> release, 0 -> pull low
    wire       test_out;
    wire [3:0] state;
    wire       sda_in;    // pad readback, will be driven from GPIO[17]

    system_wrapper system (
        .DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
        .LED_RX_INDEX(led_rx_index),
        .LED_TX_INDEX(led_tx_index),
        .ack_error(ack_error),
        .busy(busy),
        .phase(phase),
        .scl(scl),
        .sda_in(sda_in),
        .sda_out(sda_out),
        .state(state),
        .test_out(test_out)
    );

    // Build the full 24-bit GPIO vector in one expression so there are no
    // conflicting assignments. Bits not used by I2C are left as 'z'.
    //
    // Map:
    //   GPIO[19] <= SCL  (open-drain: 0 => pull low, 1 => release/z)
    //   GPIO[18] <= TEST_OUT (open-drain)
    //   GPIO[17] <= SDA  (open-drain)
    // the rest are 'z'
    //
    // vector layout: { GPIO[23], ..., GPIO[0] }
    assign GPIO = {
        4'bzzzz,                         // GPIO[23:20]
        (scl      ? 1'bz : 1'b0),       // GPIO[19]
        (test_out ? 1'bz : 1'b0),       // GPIO[18]
        (sda_out  ? 1'bz : 1'b0),       // GPIO[17]
        17'bzzzzzzzzzzzzzzzzz           // GPIO[16:0]
    };

    // Read-back of SDA from the pad
    assign sda_in = GPIO[17];

    // Route exposed outputs from I2C IP Block to LEDs (keep as you had it)
    assign LED[3:0] = led_tx_index;
    assign LED[7:4] = led_rx_index;
    assign LED[8]   = busy;
    assign LED[9]   = ack_error;
    
    ila_0 ila_0_inst (
        .clk(clk), // input wire clk
        .probe0(ack_error), // input wire [0:0]  probe0  
        .probe1(busy),      // input wire [0:0]  probe1 
        .probe2(sda_out),   // input wire [0:0]  probe2 
        .probe3(sda_in),    // input wire [0:0]  probe3 
        .probe4(scl),       // input wire [0:0]  probe4 
        .probe5(test_out),  // input wire [0:0]  probe5 
        .probe6(state)      // input wire [3:0]  probe6
    );

endmodule
