module ologic_aligner(
input 	wire gsclk_ol,
input 	wire rst,
output 	reg align_ol,
output  reg align_ol_ready_n
);
parameter  bitslip_bits = 4'h3;
localparam cnt_val = bitslip_bits*2;

reg [4:0]  cnt;

always @(posedge gsclk_ol or posedge rst) begin
	if (rst) begin
		cnt <= 5'b00000;
	end
	else begin
		if (cnt == cnt_val) begin
			cnt <= cnt;
		end
		else begin
			cnt <= cnt + 1;
		end
	end
end

always @(posedge gsclk_ol or posedge rst) begin
	if (rst) begin
		align_ol <= 1'b1;
		align_ol_ready_n <= 1'b1;
	end
	else begin
		if (cnt < cnt_val) begin
			align_ol <= ~align_ol;
			align_ol_ready_n <= 1'b1;
		end
		else begin
			align_ol_ready_n <= 1'b0;
		end
	end
end

endmodule