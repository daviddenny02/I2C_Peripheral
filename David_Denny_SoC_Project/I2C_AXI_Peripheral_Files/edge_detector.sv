// SoC Lab 3
// David Denny, 1001915603

module edge_detector (
    input  clk,
    input  reset,
    input  signal,
    output edge_out   // 1-cycle pulse on rising edge
);

    reg previous_signal;  // Previous signal to compare Current Signal against

    always_ff @(posedge clk) begin
        if (reset) begin
            previous_signal <= 1'b0;
        end else begin
            previous_signal <= signal;
        end
    end

    assign edge_out = signal & ~previous_signal;

endmodule
