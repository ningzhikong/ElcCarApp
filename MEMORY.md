# 代码审查与修改记录

日期：2026-10-07

## 修改记录

### MPU6050 I²C 设备地址

- 文件：`Core/Inc/mpu6050.h`
- 修改：将 `MPU6050_ADDR` 从 `0x68` 改为 `(0x68U << 1)`，并修正地址注释。
- 理由：STM32 HAL 的 `HAL_I2C_Mem_Read` 和 `HAL_I2C_Mem_Write` 的 `DevAddress` 参数使用左移一位后的地址。之前直接传入 7 位地址，MPU6050 的读写请求地址错误，导致初始化、校准和陀螺仪读取无法正常工作。AD0 接地时器件的 7 位地址是 `0x68`，对应 HAL 参数 `0xD0`。

## 验证

- 静态检查确认 MPU6050 的寄存器读写均引用同一个 `MPU6050_ADDR` 宏，因此该修正确实覆盖初始化、寄存器配置和数据读取。
- 未重新运行 Keil 构建或硬件验证；此前项目目录中的构建日志不能证明此次修改后的构建结果。

### 视觉识别结果在新一轮启动时残留

- 文件：`Core/Src/fsm.c`
- 修改：保留并明确蓝色为默认路线；每次从待机进入视觉识别时将颜色重置为 `COLOR_BLUE`，同时清空闪烁状态并把圈数恢复为 1。
- 理由：蓝色是用户指定的默认路线，不属于错误。开始新一轮识别时重置为蓝色可避免沿用上轮的红色结果；未收到新识别数据时会按默认蓝色路线运行。

### 停车后未清空路线边沿检测状态

- 文件：`Core/Src/fsm.c`
- 修改：进入待机时清零峰值维持脉冲、峰值锁定时间、当前/上次峰值状态和转向 PID 历史值。
- 理由：这些状态会跨越停车保留。下一次启动时，首轮传感器采样可能把上次运行的峰值状态当作新的路线边沿，造成错误计数或转向微分突变。

### UART 中断与主循环共享变量缺少 `volatile`

- 文件：`Core/Src/fsm.c`、`Core/Inc/fsm.h`
- 修改：对由 UART 接收中断更新、同时由主循环读取的系统状态、路线圈数/颜色，以及中断和主循环共享的闪烁计数与接收索引增加 `volatile`。
- 理由：这些值会在异步中断中变化，声明为 `volatile` 可避免编译器把读取缓存为旧值。

### OLED 重绘前未清空显存

- 文件：`Core/Src/main.c`
- 修改：在绘制新速度字符串前调用 `OLED_NewFrame()`。
- 理由：旧显存未清除时，较短的新字符串不会覆盖旧字符串末尾，屏幕可能残留前一帧的字符。

## 后续静态核对

- 已确认新视觉会话会清除旧结果，待机入口会清除路线边沿和转向 PID 历史状态；中断共享变量的定义与头文件声明已保持一致。
- 已确认 OLED 的主循环刷新路径在绘制前清空显存。
- 未重新运行 Keil 构建或硬件验证。

## 2026-10-07 检验结果（对照代码逐项验证）

### ✅ 1. MPU6050 I²C 设备地址 — 正确
- 代码确认为 `(0x68U << 1)` = `0xD0`
- STM32 HAL 文档 (stm32f1xx_hal_i2c.c:2674) 明确写明：*"The device 7 bits address value in datasheet must be shifted to the left before calling the interface"*
- 修改符合 HAL 规范，无问题

### ✅ 2. 视觉识别结果重置 — 正确
- 进入 SYS_STATE_VISION 时重置 `g_vision_color = COLOR_BLUE`、`g_target_laps = 1`、闪烁计数清零
- 新一轮不会沿用上轮红色结果

### ✅ 3. 停车后清空状态 — 正确
- `Enter_Standby_State()` 清除了 `peak_maintain_pulse`、`is_on_peak`/`was_on_peak`、`pid_turn` 历史值等
- 下次启动不会误判上次残留的边沿

### ✅ 4. UART 共享变量 volatile — 正确（基本完整）
- `g_system_state`、`g_target_laps`、`g_vision_color`、`blink_total_count`、`blink_cur_count`、`uart_rx_idx`、`speed_L`、`speed_R` 均已加 `volatile`
- 头文件与源文件声明一致
- **发现遗漏：** `g_current_laps` (fsm.c:18) 在主循环中被 OLED 显示函数异步读取，建议加 `volatile`

### ✅ 5. OLED 重绘前清空显存 — 正确
- main.c:135 100ms 刷新路径调用了 `OLED_NewFrame()`
- fsm.c:479 `FSM_OLED_Update()` 也调用了 `OLED_NewFrame()`

### 遗留风险（未改动）
- 主循环每 100 ms 同步发送完整 OLED 帧；I²C1 配置为 100 kHz，单帧约 1 KB，发送会占用主循环几十毫秒。与此同时 10 ms 控制周期也由该主循环调度，可能出现控制周期延迟，且 MPU6050 积分仍固定按 10 ms 计算。需要把显示传输与控制调度解耦，或优化为仅发送变化区域；本次没有擅自改动控制架构。
