/*******
FDCE_HME 0821  WUYUXIN DEBUG
*******/
`timescale  1 ps / 1 ps
module FDCE_HME (
  input wire D,      // 数据输入
  input wire C,      // 时钟输入
  input wire CE,     // 时钟使能输入
  input wire CLR,    // 异步清零输入
  output wire Q      // 输出
);

  reg Q_reg;         // 寄存器用于存储输出值

  always @(posedge C or posedge CLR) begin
    if (CLR) begin
      Q_reg <= 1'b0;  // 异步清零
    end else if (CE) begin
      Q_reg <= D;     // 在时钟边沿存储数据
    end
  end

  assign Q = Q_reg;   // 输出

endmodule
