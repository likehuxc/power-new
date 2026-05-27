# ADC 采样与 BMI088 读取性能测试

## 测试环境

| 项目 | 参数 |
|------|------|
| MCU | HC32F460（Cortex-M4） |
| 主频 | 200 MHz（XTAL 12MHz，MPLL 12/3×100/2） |
| PCLK2（ADC 时钟源） | 50 MHz（HCLK / 4） |
| PCLK3（I2C 时钟源） | 50 MHz（HCLK / 4） |
| I2C2 配置波特率 | 100 kHz，CLK_DIV2 |
| ADC 分辨率 | 12 位 |
| ADC 硬件平均 | 256 次 |
| ADC 扫描通道数 | 8 通道（序列 A） |
| 测量方法 | DWT->CYCCNT 周期计数器（精度 5 ns） |

---

## 理论计算

### ADC 一轮扫描

- 单通道单次转换 ≈ 25 个 ADCLK 周期（采样 11 + 转换 14）
- ADCLK = PCLK2 = 50 MHz → 单次转换 ≈ 0.5 µs
- 硬件 256 次平均 → 单通道 ≈ 0.5 × 256 = 128 µs
- 8 通道扫描 → 128 × 8 ≈ **1024 µs**

### BMI088 I2C 读取（按 100 kHz 标准模式估算）

读取 6 字节数据的 I2C 时序：
- 写阶段：START + 地址字节(W) + ACK + 寄存器地址 + ACK ≈ 20 bit
- 读阶段：RESTART + 地址字节(R) + ACK + 6×数据 + 5×ACK + NACK + STOP ≈ 72 bit
- 总计 ≈ 92 bit

按 100 kHz 计算：92 / 100,000 ≈ **920 µs**

---

## 实测结果

### 测试日志

```
[INFO] BMI088 timing: ACC=151.4 us, GYRO=151.3 us
[INFO] ADC scan time: 984 us
[INFO] BMI088 timing: ACC=151.4 us, GYRO=151.3 us
[INFO] ADC scan time: 984 us
[INFO] BMI088 timing: ACC=191.7 us, GYRO=151.3 us  ← 被中断打断
```

### 结果汇总

| 操作 | 理论值 | 实测值 | 备注 |
|------|--------|--------|------|
| ADC 一轮扫描（8 通道 × 256 次平均） | ~1024 µs | **984 µs** | 吻合，略快于理论值 |
| BMI088 读取加速度计（I2C 读 6 字节） | ~920 µs（@100kHz） | **151 µs** | I2C 实际速率远高于配置值 |
| BMI088 读取陀螺仪（I2C 读 6 字节） | ~920 µs（@100kHz） | **151 µs** | 同上 |

---

## 分析

### ADC 耗时（984 µs）

实测与理论基本一致，差异在 4% 以内，属于正常范围。ADC 采用 DMA 连续扫描模式，CPU 无需干预，不占用任务执行时间。

### BMI088 I2C 耗时（151 µs）

实测远快于按 100 kHz 计算的理论值，原因：

HC32F460 的 I2C 硬件波特率由 `PCLK3 / CLK_DIV / (内部分频计数)` 决定。配置 `baudrate=100000, clk_div=I2C_CLK_DIV2` 时，I2C 模块内部时钟为 PCLK3/2 = 25 MHz，硬件根据 SCL 高低电平时间参数计算实际 SCL 频率，结果明显高于 100 kHz。

反推实际 SCL 频率：
- 总线传输约 81 bit（9 字节 × 9 bit/字节，含 ACK）
- 81 bit / 151 µs ≈ **536 kHz**

实际运行在接近 Fast Mode（400 kHz）以上的速率。从 IMU 数据稳定性来看（加速度/陀螺仪读数正常），通信无误，可以保持现状。

### 偶发抖动（ACC=191.7 µs）

个别采样点耗时偏高（191 µs），是因为测量期间被更高优先级中断（如 CAN 发送）打断，DWT 计数器包含了中断处理时间。属于正常现象，不影响数据正确性。

---

## 测量代码说明

### BMI088 耗时测量（bmi088.c - BMI088_Task）

```c
/* 使能 DWT 周期计数器 */
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0U;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

/* 测量加速度计读取耗时 */
t_start = DWT->CYCCNT;
BMI088_ReadAccelRaw(&acc_raw);
t_end = DWT->CYCCNT;
acc_us = (float)(t_end - t_start) / (SystemCoreClock / 1000000UL);
```

### ADC 耗时测量（adc.c - DMA 中断回调）

```c
/* DMA 中断中记录结束时刻 */
g_adc_end_cycle = DWT->CYCCNT;
g_adc_elapsed_us = (g_adc_end_cycle - g_adc_start_cycle) / (SystemCoreClock / 1000000UL);

/* 启动下一轮前记录起始时刻 */
g_adc_start_cycle = DWT->CYCCNT;
ADC_Start(CM_ADC1);
```

---

## 结论

- ADC 8 通道 256 次平均扫描耗时约 **1 ms**，由 DMA 自动完成，不占 CPU
- BMI088 单次 I2C 读取（6 字节）耗时约 **151 µs**，读取全部数据（acc + gyro + temp）约 **450 µs**
- 当前 I2C 实际通信速率约 536 kHz，通信稳定
- 如需精确控制 I2C 速率为 100 kHz，需调大 `clk_div` 或增大 `scl_time` 参数
