module FDPE_HME #(
parameter INIT=1'd1
)
(
  input wire D,      // 数据输入
  input wire C,      // 时钟输入
  input wire CE,     // 时钟使能输入
  input wire PRE,    // 异步预置输入
  output wire Q      // 输出
);
  
  reg Q_reg;         // 寄存器用于存储输出值

  always @(posedge C or posedge PRE) begin
    if (PRE) begin
      Q_reg <= 1'b1;  // 异步预置
    end else if (CE) begin
      Q_reg <= D;     // 在时钟边沿存储数据
    end
  end

  assign Q = Q_reg;   // 输出

endmodule
