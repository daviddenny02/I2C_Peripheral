// SoC Lab 3
// David Denny, 1001915603

module fifo (
    input            clk,
    input            reset,                 // active-high reset
    input      [7:0] wr_data,
    input            wr_request,
    output     [7:0] rd_data,               // synchronous read data
    input            rd_request,
    output           empty,
    output           full,
    output reg       overflow,
    input            clear_overflow_request,
    output reg [3:0] wr_index,
    output reg [3:0] rd_index
);

    // 15-deep, 8-bit-wide FIFO
    reg [7:0] fifo [0:14];
    
    // Combinational Logic Assignments
    assign empty = (rd_index == wr_index);
    assign full  = ((wr_index + 4'd1) == rd_index);
    // Prevent reading from fifo[15]
    assign rd_data = fifo[(rd_index == 4'd15) ? 4'd0 : rd_index];
    
    always_ff @(posedge clk) begin
        // Handle Reset Request
        if (reset) begin
            wr_index <= 4'd0;
            rd_index <= 4'd0;
            overflow <= 1'b0;
        end else begin
            // Handle Clear Overflow Request
            if (clear_overflow_request) overflow <= 1'b0;
    
            // Handle Write Request
            if (wr_request) begin
                // Handle Overflow Detection
                if (full) begin
                    overflow <= 1'b1;
                // Perform the Write
                end else begin
                    // Prevent writing to fifo[15]
                    fifo[(wr_index == 4'd15) ? 4'd0 : wr_index] <= wr_data;
                    wr_index <= wr_index + 4'd1;
                end
            end
    
            // Handle Read Request
            //   This also prevents reads if Overflow is set
            //     MAY NEED TO REMOVE THIS IF IT CAUSES ERRORS!
            if (rd_request & ~empty & ~overflow) begin
                // Perform the Read
                rd_index <= rd_index + 4'd1;
            end
        end
    end
    

endmodule
