下面是图片中的 **32 位 AXI 接口信号一览表**，已整理为 Markdown 表格，便于复制到 Markdown、Word 或 LaTeX 中。

# 表 8.4  32 位 AXI 接口信号一览

## AXI 时钟与复位信号

| 信号       | 位宽 | 方向    | 功能           | 备注 |
| -------- | -- | ----- | ------------ | -- |
| aclk*    | 1  | input | AXI 时钟       |    |
| aresetn* | 1  | input | AXI 复位，低电平有效 |    |

---

## 读请求通道（AR，Address Read）

| 信号       | 位宽 | 方向           | 功能            | 备注            |
| -------- | -- | ------------ | ------------- | ------------- |
| arid*    | 4  | master→slave | 读请求的 ID 号     | 取指置为 0；取数置为 1 |
| araddr*  | 32 | master→slave | 读请求地址         |               |
| arlen    | 8  | master→slave | 读请求长度（数据传输拍数） | 固定为 0         |
| arsize*  | 3  | master→slave | 每拍传输字节数       |               |
| arburst  | 2  | master→slave | 传输类型          | 固定为 `2'b01`   |
| arlock   | 2  | master→slave | 原子锁控制         | 固定为 0         |
| arcache  | 4  | master→slave | Cache 属性      | 固定为 0         |
| arprot   | 3  | master→slave | 保护属性          | 固定为 0         |
| arvalid* | 1  | master→slave | 读地址有效握手信号     |               |
| arready* | 1  | slave→master | 从设备准备好接收读地址   |               |

---

## 读响应通道（R，Read Data）

| 信号      | 位宽 | 方向           | 功能                  | 备注            |
| ------- | -- | ------------ | ------------------- | ------------- |
| rid*    | 4  | slave→master | 返回读请求 ID，应与 arid 一致 | 0 对应取指；1 对应取数 |
| rdata*  | 32 | slave→master | 返回读数据               |               |
| rresp   | 2  | slave→master | 读响应状态               | 可忽略           |
| rlast   | 1  | slave→master | 最后一拍数据标志            | 可忽略           |
| rvalid* | 1  | slave→master | 读数据有效               |               |
| rready* | 1  | master→slave | 主设备准备好接收数据          |               |

---

## 写请求地址通道（AW，Address Write）

| 信号       | 位宽 | 方向           | 功能            | 备注          |
| -------- | -- | ------------ | ------------- | ----------- |
| awid*    | 4  | master→slave | 写请求 ID        | 固定为 1       |
| awaddr*  | 32 | master→slave | 写请求地址         |             |
| awlen    | 8  | master→slave | 写请求长度（数据传输拍数） | 固定为 0       |
| awsize*  | 3  | master→slave | 每拍传输字节数       |             |
| awburst  | 2  | master→slave | 传输类型          | 固定为 `2'b01` |
| awlock   | 2  | master→slave | 原子锁控制         | 固定为 0       |
| awcache  | 4  | master→slave | Cache 属性      | 固定为 0       |
| awprot   | 3  | master→slave | 保护属性          | 固定为 0       |
| awvalid* | 1  | master→slave | 写地址有效         |             |
| awready* | 1  | slave→master | 从设备准备好接收写地址   |             |

---

## 写数据通道（W，Write Data）

| 信号      | 位宽 | 方向           | 功能          | 备注    |
| ------- | -- | ------------ | ----------- | ----- |
| wid*    | 4  | master→slave | 写请求 ID      | 固定为 1 |
| wdata*  | 32 | master→slave | 写数据         |       |
| wstrb*  | 4  | master→slave | 字节选通信号      |       |
| wlast   | 1  | master→slave | 最后一拍数据标志    | 固定为 1 |
| wvalid* | 1  | master→slave | 写数据有效       |       |
| wready* | 1  | slave→master | 从设备准备好接收写数据 |       |

---

## 写响应通道（B，Write Response）

| 信号      | 位宽 | 方向           | 功能                      | 备注  |
| ------- | -- | ------------ | ----------------------- | --- |
| bid*    | 4  | slave→master | 返回写请求 ID，应与 awid、wid 一致 | 可忽略 |
| bresp   | 2  | slave→master | 写响应状态                   | 可忽略 |
| bvalid* | 1  | slave→master | 写响应有效                   |     |
| bready* | 1  | master→slave | 主设备准备好接收写响应             |     |

---

### 从这份表可以看出，这本《CPU设计实战（LoongArch版）》采用的是**简化版 AXI4 接口**，主要特点有：

* **不支持 Burst**：`arlen = awlen = 0`，即每次只传输一个数据拍（Single Transfer）。
* **Burst 类型固定**：`arburst = awburst = 2'b01`（INCR）。
* **Cache、Protection、Lock 等属性全部固定**，没有使用高级特性。
* **ID 仅用于区分取指和取数**：

  * `arid = 0`：取指（Instruction Fetch）
  * `arid = 1`：数据访问（Load）
  * `awid = wid = 1`：Store
* 仍然保留了 AXI 的五个独立通道（AR、R、AW、W、B）以及 `VALID/READY` 握手机制，因此接口兼容标准 AXI4，但功能上只使用了最基本的子集。

对于你后续接 **Vivado MIG DDR3 控制器**，这份表中的接口形式就是需要重点理解的 AXI Master 侧信号。真正实现时，如果使用 MIG 的 AXI 接口，还会根据配置增加数据宽度（如 64/128 bit）和支持 Burst 等功能。
