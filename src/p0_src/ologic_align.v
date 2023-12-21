module ologic_align(
input 	wire gsclk_ol,
input 	wire rst,
output 	reg align_ol
);

reg [1:0] cnt;

always @(posedge gsclk_ol) begin
	if (rst) begin
		cnt <= 2'b00;
	end
	else begin
		if (cnt == 2'b11) begin
			cnt <= cnt;
		end
		else begin
			cnt <= cnt + 1;
		end
	end
end

always @(posedge gsclk_ol) begin
	if (rst) begin
		align_ol <= 1'b0;
	end
	else begin
		if (cnt <= 2'b11) begin
			align_ol <= ~align_ol;
		end
	end
end

endmodule