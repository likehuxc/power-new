# HC32 硬件 I2C 调试踩坑记录

## 概述

本文档记录了在 HC32F4xx 平台上使用硬件 I2C（轮询模式）驱动 EEPROM (GT24C32B) 和 IMU (BMI088) 过程中遇到的问题及解决方案。

---

## 问题一：I2C_Start() 始终返回超时

### 现象

调用 `I2C_Start()` 后永远返回 `LL_ERR_TIMEOUT`，总线无法发起 START 信号。

### 根因

`I2C_Start()` 内部会先检查 BUSY 标志是否为 RESET（总线空闲），然后才发起 START。但通过寄存器观察发现 **CR1 寄存器的 PE 位（bit 0）= 0**，即 I2C 外设根本没有使能。

在 `i2c_write()` / `i2c_read()` 函数开头执行了软复位：

```c
I2C_SWResetCmd(I2C_UNIT, ENABLE);
I2C_SWResetCmd(I2C_UNIT, DISABLE);
```

软复位会清除 CR1 中的 PE 位以及其他配置，但复位后没有重新使能外设，导致后续所有操作都在外设关闭状态下执行。

### 解决方案

在软复位后立即重新使能 I2C 外设：

```c
I2C_SWResetCmd(I2C_UNIT, ENABLE);
I2C_SWResetCmd(I2C_UNIT, DISABLE);
I2C_Cmd(I2C_UNIT, ENABLE);  // 必须重新使能
```

### 寄存器证据

| 寄存器 | 修复前 | 修复后 |
|--------|--------|--------|
| CR1    | 0x00000140 (PE=0) | 0x00000041 (PE=1) |

---

## 问题二：EEPROM 写入后 ACK 轮询始终失败（全速执行失败，单步正常）

### 现象

- EEPROM 写入操作本身成功（`i2c_write` 返回 OK）
- 但随后的 `eeprom_wait_ready()` 始终超时返回错误
- 单步调试时一切正常，全速执行必定失败

### 根因

EEPROM 写入后进入内部编程周期（最长 5ms），期间不响应 I2C（返回 NACK）。驱动通过 `i2c_check_ack()` 反复发送地址来轮询设备是否就绪。

问题出在 `i2c_check_ack()` 函数**没有做软复位**：

1. 第一次调用 → EEPROM 返回 NACK → SR 寄存器中 NACKF=1，BUSY 标志残留
2. `I2C_Stop()` 发了停止条件，但 HC32 I2C 外设**不会自动清除 NACKF 等状态标志**
3. 第二次调用 → `I2C_Start()` 检查 BUSY 标志 → 发现仍为 SET → 直接返回超时
4. 后续所有轮询都失败，50 次后返回超时错误

单步调试正常的原因：人为引入的延时足够长，EEPROM 编程周期已完成，第一次轮询就能成功。

### 解决方案

在 `i2c_check_ack()` 每次调用时先软复位 + 重新使能，清除残留状态：

```c
int32_t i2c_check_ack(i2c_handle_t *hi2c, uint16_t dev_addr)
{
    const i2c_config_t *cfg = hi2c->cfg;
    int32_t ret;

    /* 软复位清除残留状态，确保总线空闲 */
    I2C_SWResetCmd(cfg->unit, ENABLE);
    I2C_SWResetCmd(cfg->unit, DISABLE);
    I2C_BusWaitCmd(cfg->unit, ENABLE);
    I2C_Cmd(cfg->unit, ENABLE);

    ret = I2C_Start(cfg->unit, cfg->timeout);
    if (ret == LL_OK) {
        ret = I2C_TransAddr(cfg->unit, dev_addr, I2C_DIR_TX, cfg->timeout);
        if (ret != LL_OK) {
            ret = LL_ERR;  // NACK，设备未就绪
        }
    }
    (void)I2C_Stop(cfg->unit, cfg->timeout);
    return ret;
}
```

### 关键认知

> HC32 I2C 外设在收到 NACK 后，即使发送了 STOP 信号，BUSY 和 NACKF 标志也不会自动清除。必须通过软复位来归零状态机。

---

## 问题三：BMI088 陀螺仪写寄存器 NACK（读正常，写失败）

### 现象

- BMI088 陀螺仪软复位后，读 chip_id（0x0F）成功
- 紧接着写 GYRO_RANGE 寄存器时，从机返回 NACK
- 加速度计的读写操作一切正常
- 延长软复位后的等待时间（从 35ms 到 350ms）无效

### 寄存器证据

调试时观察 I2C2 外设寄存器：

| 寄存器/标志 | 值 | 含义 |
|-------------|---|------|
| SR.ACKRF | 1 | 地址阶段收到了 ACK（从机在线） |
| SR.NACKF | 1 | 数据阶段收到了 NACK（写被拒绝） |
| SR.BUSY | 1 | 总线忙 |
| DRR | 0x0F | 上次读到的数据 = 陀螺仪 chip_id（读操作成功） |
| **CR4.BUSWAIT** | **0** | **总线等待功能未使能！** |

关键发现：**BUSWAIT = 0**，但 `i2c_init()` 中明确配置了 `I2C_BusWaitCmd(ENABLE)`。

### 根因分析

#### BUSWAIT 是什么

`I2C_BusWaitCmd(ENABLE)` 使能后，I2C 硬件会**持续监测 SCL/SDA 引脚的物理电平**，只要总线被占用（任一线为低）就保持 BUSY=1。这让 `I2C_Start()` 中的等待逻辑能真正反映总线的物理状态。

#### I2C_Start() 的内部逻辑

```c
// hc32_ll_i2c.c
int32_t I2C_Start(CM_I2C_TypeDef *I2Cx, uint32_t u32Timeout)
{
    int32_t i32Ret;

    // 第一步：等待 BUSY 标志变为 RESET（总线空闲）
    i32Ret = I2C_WaitStatus(I2Cx, I2C_FLAG_BUSY, RESET, u32Timeout);

    if (LL_OK == i32Ret) {
        // 第二步：发起 START 信号
        I2C_GenerateStart(I2Cx);
        // 第三步：等待 START 条件确认
        i32Ret = I2C_WaitStatus(I2Cx, (I2C_FLAG_BUSY | I2C_FLAG_START), SET, u32Timeout);
    }

    return i32Ret;
}
```

#### BUSWAIT 对 BUSY 标志的影响

| BUSWAIT 状态 | 软复位后 BUSY 的行为 | I2C_Start() 第一步的结果 |
|-------------|---------------------|------------------------|
| **= 1（使能）** | 硬件重新检测引脚电平，如果总线还被占用则 BUSY 重新置 1 | 真正等到总线物理释放才通过 |
| **= 0（关闭）** | 软复位已将 BUSY 清零，不再跟踪实际引脚状态 | 发现 BUSY=RESET，**立刻通过**（假象！） |

#### 出问题的完整时序

```
gyro_read_reg() 完成
  └─ I2C_Stop() → SCL/SDA 开始释放（需要几个 SCL 周期）

gyro_write_reg() 立刻开始（间隔仅几微秒）
  └─ I2C_SWResetCmd() → BUSY 被清零，BUSWAIT 被清零
  └─ I2C_Cmd(ENABLE)  → PE 恢复，但 BUSWAIT 仍为 0
  └─ I2C_Start()
       └─ 检查 BUSY → 已被软复位清零 = RESET → "总线空闲"（假的！）
       └─ 发 START → 但实际 SCL/SDA 可能还没完全释放
       └─ 从机看到异常时序 → NACK
```

#### 为什么加速度计没问题

加速度计初始化时，每次写操作之间都有 `vTaskDelay(1ms)` 或 `DDL_DelayMS(50ms)` 的长延时。即使 BUSWAIT=0，这些延时远超总线释放时间，所以碰不到问题。

陀螺仪初始化时，读 chip_id 后紧接着写 GYRO_RANGE，两次操作间隔极短（微秒级），没有 BUSWAIT 保护就撞上了总线还没释放的窗口。

### 解决方案

在软复位后补上 `I2C_BusWaitCmd(ENABLE)`：

```c
// 修复后的标准软复位恢复序列
I2C_SWResetCmd(cfg->unit, ENABLE);
I2C_SWResetCmd(cfg->unit, DISABLE);
I2C_BusWaitCmd(cfg->unit, ENABLE);   // 恢复总线等待功能！
I2C_Cmd(cfg->unit, ENABLE);
```

修复后的时序：

```
gyro_read_reg() 完成
  └─ I2C_Stop() → SCL/SDA 开始释放

gyro_write_reg() 立刻开始
  └─ I2C_SWResetCmd() → BUSY 被清零
  └─ I2C_BusWaitCmd(ENABLE) → 硬件重新监测引脚电平
  └─ I2C_Cmd(ENABLE)
  └─ I2C_Start()
       └─ 检查 BUSY → 硬件检测到 SCL/SDA 还是低 → BUSY=SET → 等待
       └─ ... 几微秒后引脚释放 → BUSY=RESET → 通过
       └─ 发 START → 时序正确 → ACK ✓
```

---

## 总结：HC32 I2C 软复位的完整影响

### 软复位会清除的内容

| 项目 | 说明 |
|------|------|
| CR1.PE | 外设使能位 → 必须恢复 |
| CR4.BUSWAIT | 总线等待功能 → 必须恢复 |
| SR 所有标志 | BUSY/NACKF/ACKRF 等 → 这正是我们想清除的 |
| 内部状态机 | 回到 IDLE → 这正是我们想要的 |

### 软复位不会清除的内容

| 项目 | 说明 |
|------|------|
| CCR（时钟配置） | 波特率、分频等保留 |
| SLR（建立/保持时间） | 时序参数保留 |
| FLTR（滤波配置） | 数字滤波保留 |

### 标准软复位恢复序列（所有 I2C 操作前使用）

```c
I2C_SWResetCmd(unit, ENABLE);       // 1. 触发软复位
I2C_SWResetCmd(unit, DISABLE);      // 2. 释放复位
I2C_BusWaitCmd(unit, ENABLE);       // 3. 恢复总线等待（关键！）
I2C_Cmd(unit, ENABLE);              // 4. 重新使能外设
```

### 调试技巧

遇到 I2C 通信异常时的检查顺序：

1. **CR1.PE = 1？** → 外设是否使能
2. **CR4.BUSWAIT = 1？** → 总线等待是否使能
3. **SR.NACKF？** → 是否有残留的 NACK 标志
4. **SR.BUSY？** → 总线是否卡在忙状态
5. **用逻辑分析仪确认** → NACK 发生在地址阶段还是数据阶段

---

## 附：相关文件

| 文件 | 说明 |
|------|------|
| `communication/i2c/i2c.c` | 硬件 I2C 主机驱动（轮询模式，多实例，含总线恢复） |
| `communication/i2c/i2c.h` | I2C 驱动接口（配置结构体 + 句柄） |
| `applications/eeprom.c` | EEPROM 读写驱动（基于 I2C1） |
| `applications/bmi088.c` | BMI088 IMU 驱动（基于 I2C2） |
| `libraries/hc32_ll_driver/src/hc32_ll_i2c.c` | HC32 LL 库 I2C 底层实现 |
