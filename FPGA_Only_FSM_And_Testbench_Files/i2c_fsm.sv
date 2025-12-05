// David Denny, 1001915603
// I2C FSM

module i2c_axi_fsm
(
    input  logic        clk,
    input  logic        reset,
    input  reg [6:0]    addr,
    input  reg [7:0]    register,
    
    // Control Register  Signals
    input  reg [3:0]    byte_count,
    input  reg          read_write,
    input  reg          use_register,
    input  reg          use_repeated_start,
    input  reg          start,
    
    // Status Register Signals
    output reg          ack_error,
    output reg          busy,
    
    // Main FSM Outputs
    output  logic       sda,
    output  logic       scl,
    output  logic       test_out, // debug 200kHz Clock, Technically part of Control Register
    
    // FIFO Signals
    input  logic [7:0]  tx_rd_data,
    output logic        tx_rd_request,
    output logic [7:0]  rx_wr_data,
    output logic        rx_wr_request,
    
    // Miscellaneous Signals
    input  logic        ignore_ack,
    output logic [3:0]  state_out,  // Exposes curr_state for ILA probing
    output logic [4:0]  phase_out,  // Exposes phase for ILA probing
    input  logic        sda_in
);

    logic tick;
    divide_by_500 div500
    (
        .clk(clk), 
        .out(tick)
    );
    assign test_out = tick;
    
    // Edge Detector for TX FIFO Reads
    reg tx_rd_request_signal;
    edge_detector rd_edge_inst (
        .clk(clk),
        .reset(reset),
        .signal(tx_rd_request_signal),
        .edge_out(tx_rd_request)
    );
    
    // Edge Detector for RX FIFO Writes
    reg rx_wr_request_signal;
    edge_detector wr_edge_inst (
        .clk(clk),
        .reset(reset),
        .signal(rx_wr_request_signal),
        .edge_out(rx_wr_request)
    );
    
    // 10-State FSM
    typedef enum logic [3:0] {
        ST_IDLE                     = 4'd0,
        ST_START                    = 4'd1,
        ST_WR_AND_COMPLEX_RD_ADDR   = 4'd2,
        ST_WR_AND_COMPLEX_RD_REG    = 4'd3,
        ST_WR_DATA                  = 4'd4,
        ST_STOP                     = 4'd5,
        ST_REPEATED_START_STOP      = 4'd6,
        ST_REPEATED_START_START     = 4'd7,
        ST_RD_ADDR                  = 4'd8,
        ST_RD_DATA                  = 4'd9
    } state;
    
    // Ack logic, Force correct acks if we are ignoring acks from device
    wire ack;
    assign ack = ignore_ack ? 1'b0 : sda_in;
    reg ack_latch;
    
    // FSM Sequential Block
    //   This block is based on the 200kHz tick
    //   This is the only block that updates curr_state
    state curr_state;
    reg start_cond;
    reg [4:0] phase;
    reg [1:0] start_bits;
    reg [1:0] stop_bits;
    reg [3:0] byte_count_internal;
    reg [0:0] repeated_start_bit;
    always_ff @(posedge clk) begin
    
        if (reset) begin
            // Reset all sequential state here
            curr_state <= ST_IDLE;
            byte_count_internal <= byte_count;
            start_cond <= 1'b0;
            phase <= 1'b0;
            start_bits <= 1'b0;
            stop_bits  <= 1'b0;
            ack_error  <= 1'b0;
            repeated_start_bit <= 1'b0;
            tx_rd_request_signal <= 1'b0;
            rx_wr_request_signal <= 1'b0;
            ack_latch <= 1'b1;
            sda <= 1'b1;         // SDA idle high
            scl <= 1'b1;         // SCL idle high
            busy <= 1'b0;
        end else if (start) begin
            start_cond <= 1'b1;
            byte_count_internal <= byte_count;
            phase <= 1'b0;
            start_bits <= 1'b0;
            stop_bits  <= 1'b0;
            ack_error  <= 1'b0;
        end else begin 
            // Default pulses cleared each clock (tx_rd_req/rx_wr_req are single-tick pulses)
            if (tick) begin
                case (curr_state)
                    // =========================================================================================
                    // Default state, constantly reading control register to detect a start
                    ST_IDLE: begin
                        if (start_cond) begin
                            curr_state <= ST_START; // Start if a start from control register is detected
                            byte_count_internal <= byte_count;
                            start_bits <= 1'b0;
                            stop_bits  <= 1'b0;
                            repeated_start_bit <= 1'b0;
                            busy <= 1'b1;
                            scl <=  1'b1;
                            sda <=  1'b1;
                            phase <= 1'b0;
                            ack_error  <= 1'b0;
                        end else begin
                            curr_state <= ST_IDLE;  // Else default to idle
                            byte_count_internal <= byte_count;
                            start_bits <= 1'b0;
                            stop_bits  <= 1'b0;
                            repeated_start_bit <= 1'b0;
                            busy <= 1'b0;
                            phase <= 1'b0;
                            ack_error  <= 1'b0;
                        end
                    end

                    // =========================================================================================
                    // Start state, from here we determine if we are to branch to one of two states:
                    //   If we are performing a Simple/Complex Write, or Complex Read, we go to ST_WR_AND_COMPLEX_RD_ADDR
                    //     This is determined if use_register (Complex RD/WR or Simple Write) or ~read_write (Writing if crtl_rw == 0)
                    //   If we are performing a Simple Read, we go to ST_RD_ADDR
                    //     This is determined if ~use_register (Non-Complex) and read_write (Reading if read_write == 1)
                    // After this stage, we need to clear the start bit whenever it is safe to do so (when clearing doesn't conflict with other FSM logic)
                    ST_START: begin
                        // Stagger the SDA and SCL lines for 3 ticks for metastability purposes
                        //   We need to make sure we are safe to perform a start condition successfully
                        // Perform start condition and start the SCL behavior correctly
                        case (start_bits)
                            2'b00: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                start_bits <= start_bits + 1'b1;
                            end
                            2'b01: begin
                                scl <= 1'b1;
                                sda <= 1'b0;
                                start_bits <= start_bits + 1'b1;
                            end
                            2'b10: begin
                                if (start_cond & (use_register | ~read_write)) begin
                                    curr_state <= ST_WR_AND_COMPLEX_RD_ADDR; // Start if a start from control register is received
                                    scl <= 1'b0;
                                    sda <= addr[6]; // Begin spitting out ADDR[6]
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                end else if (start_cond & (~use_register & read_write)) begin
                                    curr_state <= ST_RD_ADDR; // Start if a start from control register is received
                                    scl <= 1'b0;
                                    sda <= addr[6]; // Begin spitting out ADDR[6]
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                end else begin
                                    curr_state <= ST_IDLE;  // Default
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                end
                            end
                            default: begin
                                curr_state <= ST_IDLE;
                            end
                       endcase
                    end

                    // =========================================================================================
                    // Address state for Writes and Complex Reads, from here we transmit the 7 address bits and single R/W bit
                    // Afterwards, if successful, we determine if we are to branch to one of two states:
                    //   If we are performing a Complex Read or Write, we go to ST_WR_AND_COMPLEX_RD_REG
                    //     This is determined if use_register and ACK = 0
                    //   If we are performing a Simple Write, we go to ST_WR_DATA
                    //     This is determined if ~use_register and ACK = 0
                    ST_WR_AND_COMPLEX_RD_ADDR: begin
                    // Transmit 7 address bits and the R/W bit over the SDA line
                        //   Keep track of this transmission with a counter
                        // If, after this transmission, the next bit (ACK) is 1, we revert back to ST_IDLE
                        // If, after this transmission, the next bit (ACK) is 0, we go to ST_WR_AND_COMPLEX_RD_REG or ST_WR_DATA
                        case (phase)
                            5'd0: begin
                                scl <= 1'b1;
                                sda <= addr[6];
                                phase <= phase + 1'b1;
                            end
                            5'd1: begin
                                scl <= 1'b0;
                                sda <= addr[5];
                                phase <= phase + 1'b1;
                            end
                            5'd2: begin
                                scl <= 1'b1;
                                sda <= addr[5];
                                phase <= phase + 1'b1;
                            end
                            5'd3: begin
                                scl <= 1'b0;
                                sda <= addr[4];
                                phase <= phase + 1'b1;
                            end
                            5'd4: begin
                                scl <= 1'b1;
                                sda <= addr[4];
                                phase <= phase + 1'b1;
                            end
                            5'd5: begin
                                scl <= 1'b0;
                                sda <= addr[3];
                                phase <= phase + 1'b1;
                            end
                            5'd6: begin
                                scl <= 1'b1;
                                sda <= addr[3];
                                phase <= phase + 1'b1;
                            end
                            5'd7: begin
                                scl <= 1'b0;
                                sda <= addr[2];
                                phase <= phase + 1'b1;
                            end
                            5'd8: begin
                                scl <= 1'b1;
                                sda <= addr[2];
                                phase <= phase + 1'b1;
                            end
                            5'd9: begin
                                scl <= 1'b0;
                                sda <= addr[1];
                                phase <= phase + 1'b1;
                            end
                            5'd10: begin
                                scl <= 1'b1;
                                sda <= addr[1];
                                phase <= phase + 1'b1;
                            end
                            5'd11: begin
                                scl <= 1'b0;
                                sda <= addr[0];
                                phase <= phase + 1'b1;
                            end
                            5'd12: begin
                                scl <= 1'b1;
                                sda <= addr[0];
                                phase <= phase + 1'b1;
                            end
                            5'd13: begin
                                scl <= 1'b0;
                                sda <= read_write;
                                phase <= phase + 1'b1;
                            end
                            5'd14: begin
                                scl <= 1'b1;
                                sda <= read_write;
                                phase <= phase + 1'b1;
                            end
                            5'd15: begin
                                scl <= 1'b0;
                                // Beginning of ACK. Don't sample ACK here
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd16: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                ack_latch <= ack; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd17: begin
                                if (use_register & ~ack_latch) begin
                                    curr_state <= ST_WR_AND_COMPLEX_RD_REG;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= register[7];
                                end else if (~use_register & ~ack_latch) begin
                                    curr_state <= ST_WR_DATA;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= tx_rd_data[7];
                                end else begin // ACK = 1
                                    curr_state <= ST_IDLE;  // Default
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    busy <= 1'b0;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    ack_error  <= 1'b1;
                                end
                            end
                        endcase
                    end

                    // =========================================================================================
                    // Register state for Complex Writes and Reads, from here we transmit the 8 register bits
                    // Afterwards, if successful, we determine if we are to branch to one of three states:
                    //   If we are performing a Complex Write, we go to ST_WR_DATA
                    //     This is determined if ~read_write and ACK = 0
                    //   If we are performing a Complex Read with a Repeated Start, we go to ST_REPEATED_START_START
                    //     This is determined if read_write, use_repeated_start, and ACK = 0
                    //   If we are performing a Complex Read without a Repeated Start, we go to ST_REPEATED_START_STOP
                    //     This is determined if read_write, ~use_repeated_start, and ACK = 0
                    ST_WR_AND_COMPLEX_RD_REG: begin
                        case (phase)
                            5'd0: begin
                                scl <= 1'b1;
                                sda <= register[7];
                                phase <= phase + 1'b1;
                            end
                            5'd1: begin
                                scl <= 1'b0;
                                sda <= register[6];
                                phase <= phase + 1'b1;
                            end
                            5'd2: begin
                                scl <= 1'b1;
                                sda <= register[6];
                                phase <= phase + 1'b1;
                            end
                            5'd3: begin
                                scl <= 1'b0;
                                sda <= register[5];
                                phase <= phase + 1'b1;
                            end
                            5'd4: begin
                                scl <= 1'b1;
                                sda <= register[5];
                                phase <= phase + 1'b1;
                            end
                            5'd5: begin
                                scl <= 1'b0;
                                sda <= register[4];
                                phase <= phase + 1'b1;
                            end
                            5'd6: begin
                                scl <= 1'b1;
                                sda <= register[4];
                                phase <= phase + 1'b1;
                            end
                            5'd7: begin
                                scl <= 1'b0;
                                sda <= register[3];
                                phase <= phase + 1'b1;
                            end
                            5'd8: begin
                                scl <= 1'b1;
                                sda <= register[3];
                                phase <= phase + 1'b1;
                            end
                            5'd9: begin
                                scl <= 1'b0;
                                sda <= register[2];
                                phase <= phase + 1'b1;
                            end
                            5'd10: begin
                                scl <= 1'b1;
                                sda <= register[2];
                                phase <= phase + 1'b1;
                            end
                            5'd11: begin
                                scl <= 1'b0;
                                sda <= register[1];
                                phase <= phase + 1'b1;
                            end
                            5'd12: begin
                                scl <= 1'b1;
                                sda <= register[1];
                                phase <= phase + 1'b1;
                            end
                            5'd13: begin
                                scl <= 1'b0;
                                sda <= register[0];
                                phase <= phase + 1'b1;
                            end
                            5'd14: begin
                                scl <= 1'b1;
                                sda <= register[0];
                                phase <= phase + 1'b1;
                            end
                            5'd15: begin
                                scl <= 1'b0;
                                // Beginning of ACK. Don't sample ACK here
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd16: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                ack_latch <= ack; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd17: begin
                                if (~read_write & ~ack_latch) begin
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    curr_state <= ST_WR_DATA;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= tx_rd_data[7]; // Send the MSB of the data byte in advance
                                end else if (read_write & use_repeated_start & ~ack_latch) begin // ST_REPEATED_START_START
                                    curr_state <= ST_REPEATED_START_START;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= use_repeated_start; // Send the MSB of the address byte in advance
                                end else if (read_write & ~use_repeated_start & ~ack_latch) begin // ST_REPEATED_START_STOP
                                    curr_state <= ST_REPEATED_START_STOP;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= use_repeated_start; // Send the MSB of the address byte in advance
                                end else begin // ACK = 1
                                    curr_state <= ST_IDLE;  // Default
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    busy <= 1'b0;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    ack_error  <= 1'b1;
                                end
                            end
                        endcase
                    end
                    
                    // =========================================================================================
                    // Simple and Complex Write Data, from here we transmit 8 data bits, repeatedly so if byte_count > 1
                    // Afterwards, if successful, we determine if we are to branch to one of two states:
                    //   If we are writing multiple bytes, we set up a separate counter and decrement from that counter every time we send a byte, and send bytes until byte_count == 0, then continue
                    //     If byte_count != 0 and the requisite ACK = 0, we go back to ST_WR_DATA to send the next byte
                    //     If byte_count == 0 and the requisite ACK = 0, we go to ST_STOP to complete the transmission
                    ST_WR_DATA: begin
                        case (phase)
                            5'd0: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[7];
                                phase <= phase + 1'b1;
                            end
                            5'd1: begin
                                scl <= 1'b0;
                                sda <= tx_rd_data[6];
                                phase <= phase + 1'b1;
                            end
                            5'd2: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[6];
                                phase <= phase + 1'b1;
                            end
                            5'd3: begin
                                scl <= 1'b0;
                                sda <= tx_rd_data[5];
                                phase <= phase + 1'b1;
                            end
                            5'd4: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[5];
                                phase <= phase + 1'b1;
                            end
                            5'd5: begin
                                scl <= 1'b0;
                                sda <= tx_rd_data[4];
                                phase <= phase + 1'b1;
                            end
                            5'd6: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[4];
                                phase <= phase + 1'b1;
                            end
                            5'd7: begin
                                scl <= 1'b0;
                                sda <= tx_rd_data[3];
                                phase <= phase + 1'b1;
                            end
                            5'd8: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[3];
                                phase <= phase + 1'b1;
                            end
                            5'd9: begin
                                scl <= 1'b0;
                                sda <= tx_rd_data[2];
                                phase <= phase + 1'b1;
                            end
                            5'd10: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[2];
                                phase <= phase + 1'b1;
                            end
                            5'd11: begin
                                scl <= 1'b0;
                                sda <= tx_rd_data[1];
                                phase <= phase + 1'b1;
                            end
                            5'd12: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[1];
                                phase <= phase + 1'b1;
                            end
                            5'd13: begin
                                scl <= 1'b0;
                                sda <= tx_rd_data[0];
                                phase <= phase + 1'b1;
                            end
                            5'd14: begin
                                scl <= 1'b1;
                                sda <= tx_rd_data[0];
                                phase <= phase + 1'b1;
                            end
                            5'd15: begin
                                scl <= 1'b0;
                                // Beginning of ACK. Don't sample ACK here
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd16: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                ack_latch <= ack; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                                tx_rd_request_signal <= 1'b1;
                            end
                            5'd17: begin
                                if ((byte_count_internal != 0) & ~ack_latch) begin // Sending multiple bytes
                                    curr_state <= ST_WR_DATA;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    byte_count_internal <= byte_count_internal - 1'b1;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= tx_rd_data[7];
                                    tx_rd_request_signal <= 1'b0;
                                end else if ((byte_count_internal == 0) & ~ack_latch) begin // Stop sending bytes
                                    curr_state <= ST_STOP;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= 1'b0;
                                    tx_rd_request_signal <= 1'b0;
                                end else begin // ACK = 1
                                    curr_state <= ST_IDLE;  // Default
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    busy <= 1'b0;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    tx_rd_request_signal <= 1'b0;
                                    ack_error  <= 1'b1;
                                end
                            end
                        endcase
                    end

                    // =========================================================================================
                    // No Repeated Start Condition for Complex Reads, from here we determine Repeated Start Logic
                    //   Repeated Start is used for a master to keep control of the slave device, and prevent
                    //   other master devices from controlling it
                    // Go to ST_REPEATED_START_STOP
                    ST_REPEATED_START_START: begin
                        curr_state <= ST_REPEATED_START_STOP;
                        ack_latch  <= 1'b1; // Clear ACK for next byte
                        start_cond <= 1'b0;
                        start_bits <= 1'b0;
                        stop_bits  <= 1'b0;
                        repeated_start_bit <= 1'b0;
                        phase <= 1'b0;
                        scl <= 1'b1;
                        sda <= use_repeated_start;
                    end
                    
                    // =========================================================================================
                    // Repeated Start Condition for Complex Reads, from here we determine Repeated Start Logic
                    //   Repeated Start is used for a master to keep control of the slave device, and prevent
                    //   other master devices from controlling it
                    // Go to ST_RD_ADDR
                    ST_REPEATED_START_STOP: begin
                        if(use_repeated_start) begin
                            curr_state <= ST_RD_ADDR;
                            ack_latch  <= 1'b1; // Clear ACK for next byte
                            start_cond <= 1'b0;
                            start_bits <= 1'b0;
                            stop_bits  <= 1'b0;
                            repeated_start_bit <= 1'b0;
                            phase <= 1'b0;
                            scl <= 1'b0;
                            sda <= addr[6];
                        end else begin
                            if(repeated_start_bit) begin
                                curr_state <= ST_RD_ADDR;
                                ack_latch  <= 1'b1; // Clear ACK for next byte
                                start_cond <= 1'b0;
                                start_bits <= 1'b0;
                                stop_bits  <= 1'b0;
                                repeated_start_bit <= 1'b0;
                                phase <= 1'b0;
                                scl <= 1'b0;
                                sda <= addr[6];
                            end else begin
                                curr_state <= ST_REPEATED_START_STOP;
                                scl <= 1'b1;
                                sda <= use_repeated_start;
                                repeated_start_bit <= repeated_start_bit + 1;
                            end
                        end
                    end

                    // =========================================================================================
                    // Simple Read Address state, from here we transmit the 7 address bits and single R/W bit
                    // Afterwards, if successful, we determine if we are to branch to one of two states:
                    //   If we are performing a Complex Read or Write, we go to ST_RD_DATA
                    //     This is determined if ACK = 0 after transmitting the 8 total bits
                    // If we reach this state from a Complex Read, I believe we go straight to ST_RD_DATA instead of transmitting another 8 total bits
                    //   This is determined if use_register or use_repeated_start
                    ST_RD_ADDR: begin
                    // If we reach this state from a Complex Read, I believe we go straight to ST_RD_DATA
                    // Transmit 7 address bits and the R/W bit over the SDA line
                    //   Keep track of this transmission with a counter
                    // If, after this transmission, the next bit (ACK) is 1, we revert back to ST_IDLE
                    // If, after this transmission, the next bit (ACK) is 0, we go to ST_RD_DATA
                        case (phase)
                            5'd0: begin
                                scl <= 1'b1;
                                sda <= addr[6];
                                phase <= phase + 1'b1;
                            end
                            5'd1: begin
                                scl <= 1'b0;
                                sda <= addr[5];
                                phase <= phase + 1'b1;
                            end
                            5'd2: begin
                                scl <= 1'b1;
                                sda <= addr[5];
                                phase <= phase + 1'b1;
                            end
                            5'd3: begin
                                scl <= 1'b0;
                                sda <= addr[4];
                                phase <= phase + 1'b1;
                            end
                            5'd4: begin
                                scl <= 1'b1;
                                sda <= addr[4];
                                phase <= phase + 1'b1;
                            end
                            5'd5: begin
                                scl <= 1'b0;
                                sda <= addr[3];
                                phase <= phase + 1'b1;
                            end
                            5'd6: begin
                                scl <= 1'b1;
                                sda <= addr[3];
                                phase <= phase + 1'b1;
                            end
                            5'd7: begin
                                scl <= 1'b0;
                                sda <= addr[2];
                                phase <= phase + 1'b1;
                            end
                            5'd8: begin
                                scl <= 1'b1;
                                sda <= addr[2];
                                phase <= phase + 1'b1;
                            end
                            5'd9: begin
                                scl <= 1'b0;
                                sda <= addr[1];
                                phase <= phase + 1'b1;
                            end
                            5'd10: begin
                                scl <= 1'b1;
                                sda <= addr[1];
                                phase <= phase + 1'b1;
                            end
                            5'd11: begin
                                scl <= 1'b0;
                                sda <= addr[0];
                                phase <= phase + 1'b1;
                            end
                            5'd12: begin
                                scl <= 1'b1;
                                sda <= addr[0];
                                phase <= phase + 1'b1;
                            end
                            5'd13: begin
                                scl <= 1'b0;
                                sda <= read_write;
                                phase <= phase + 1'b1;
                            end
                            5'd14: begin
                                scl <= 1'b1;
                                sda <= read_write;
                                phase <= phase + 1'b1;
                            end
                            5'd15: begin
                                scl <= 1'b0;
                                // Beginning of ACK. Don't sample ACK here
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd16: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                ack_latch <= ack; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd17: begin
                                if (~ack_latch) begin // Simple Read, ACK is 0 after transmitting the read address
                                    curr_state <= ST_RD_DATA;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= 1'b1;
                                end else if (use_register | use_repeated_start) begin // Complex Read
                                    curr_state <= ST_RD_DATA;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= 1'b1;
                                end else begin // ACK = 1
                                    curr_state <= ST_IDLE;  // Default
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    busy <= 1'b0;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    ack_error  <= 1'b1;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                end
                            end
                        endcase
                    end

                    // =========================================================================================
                    // Simple and Complex Read Data, from here we receive 8 data bits, repeatedly so if byte_count > 1
                    // Afterwards, if successful, we determine if we are to branch to one of two states:
                    //   If we are reading multiple bytes, we set up a separate counter and decrement from that counter every time we read a byte, and read bytes until byte_count == 0, then continue
                    //     If byte_count != 0 and the requisite ACK = 0, we go back to ST_RD_DATA to read the next byte
                    //     If byte_count == 0 and the requisite ACK = 0, we go to ST_STOP to complete the transmission
                    ST_RD_DATA: begin
                    
                        case (phase)
                            5'd0: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[7] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd1: begin
                                scl <= 1'b0;
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd2: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[6] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd3: begin
                                scl <= 1'b0;
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd4: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[5] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd5: begin
                                scl <= 1'b0;
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd6: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[4] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd7: begin
                                scl <= 1'b0;
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd8: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[3] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd9: begin
                                scl <= 1'b0;
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd10: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[2] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd11: begin
                                scl <= 1'b0;
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd12: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[1] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd13: begin
                                scl <= 1'b0;
                                sda <= 1'b1;
                                phase <= phase + 1'b1;
                            end
                            5'd14: begin
                                scl <= 1'b1;
                                sda <= 1'b1;
                                rx_wr_data[0] <= sda_in; // Sample on posedge of SCL
                                phase <= phase + 1'b1;
                            end
                            5'd15: begin
                                scl <= 1'b0;
                                // Beginning of ACK
                                if (byte_count_internal != 0) begin // Send ACK here, ACK = 0 if byte_count < 1, ACK = 1 if done receiving
                                    sda <= 1'b0;
                                end else begin // Send ACK here, ACK = 0 if byte_count < 1, ACK = 1 if done receiving
                                    sda <= 1'b1;
                                end
                                phase <= phase + 1'b1;
                            end
                            5'd16: begin
                                scl <= 1'b1;
                                if (byte_count_internal != 0) begin // Send ACK here, ACK = 0 if byte_count < 1, ACK = 1 if done receiving
                                    sda <= 1'b0;
                                end else begin // Send ACK here, ACK = 0 if byte_count < 1, ACK = 1 if done receiving
                                    sda <= 1'b1;
                                end
                                phase <= phase + 1'b1;
                                rx_wr_request_signal <= 1'b1;
                            end
                            5'd17: begin
                                if (byte_count_internal != 0) begin // Receiving multiple bytes
                                    curr_state <= ST_RD_DATA;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    byte_count_internal <= byte_count_internal - 1'b1;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= 1'b1;
                                    rx_wr_request_signal <= 1'b0;
                                end else if (byte_count_internal == 0) begin // Stop Receiving bytes
                                    curr_state <= ST_STOP;
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    scl <= 1'b0;
                                    sda <= 1'b0;
                                    rx_wr_request_signal <= 1'b0;
                                end else begin // Should be unreachable, but revert to Idle if encountered
                                    curr_state <= ST_IDLE;  // Default
                                    ack_latch  <= 1'b1; // Clear ACK for next byte
                                    busy <= 1'b0;
                                    start_cond <= 1'b0;
                                    start_bits <= 1'b0;
                                    stop_bits  <= 1'b0;
                                    ack_error  <= 1'b1;
                                    repeated_start_bit <= 1'b0;
                                    phase <= 1'b0;
                                    rx_wr_request_signal <= 1'b0;
                                end
                            end
                        endcase
                    end

                    // =========================================================================================
                    // Complete the transmission by sending the requisite Stop Condition
                    // Revert back to Idle after sending the Stop Condition
                    ST_STOP: begin
                        case (stop_bits)
                            2'b00: begin
                                scl <= 1'b1;
                                sda <= 1'b0;
                                stop_bits <= stop_bits + 1'b1;
                            end
                            2'b01: begin
                                curr_state <= ST_IDLE; // Start if a start from control register is received
                                ack_latch  <= 1'b1; // Clear ACK for next byte
                                scl <= 1'b1;
                                sda <= 1'b1; // Final transmission bit
                                start_cond <= 1'b0;
                                start_bits <= 1'b0;
                                stop_bits  <= 1'b0;
                                ack_error  <= 1'b0;
                                repeated_start_bit <= 1'b0;
                                phase <= 1'b0;
                                busy <= 1'b0;
                            end
                            default: begin
                                curr_state <= ST_IDLE;
                                ack_latch  <= 1'b1; // Clear ACK for next byte
                                start_cond <= 1'b0;
                                start_bits <= 1'b0;
                                stop_bits  <= 1'b0;
                                ack_error  <= 1'b0;
                                repeated_start_bit <= 1'b0;
                                phase <= 1'b0;
                                busy <= 1'b0;
                            end
                       endcase
                    end
                    
                    // =========================================================================================
                    default: begin
                        // If anything needed
                        curr_state <= ST_IDLE;
                        ack_latch  <= 1'b1; // Clear ACK for next byte
                        start_cond <= 1'b0;
                        start_bits <= 1'b0;
                        stop_bits  <= 1'b0;
                        repeated_start_bit <= 1'b0;
                        phase <= 1'b0;
                        busy <= 1'b0;
                    end
                endcase
            end // tick end
        end // reset end
    end // CLK100 end
    
    assign state_out = curr_state;
    assign phase_out = phase;

endmodule

// Clock divider
module divide_by_500(
    input clk,
    output reg out);

    reg [26:0] count;

    always_ff @ (posedge(clk))
    begin
        if (count < 27'd500)
        begin
           count <= count + 1;
           out <= 0;
        end
        else
        begin
            count <= 0;
            out <= 1;
        end
    end
endmodule