# I2C 驱动中断化改造方案（FreeRTOS）

> 目标：把 `communication/i2c/i2c.c` 当前的**轮询忙等**实现改造为**中断驱动 + FreeRTOS 信号量阻塞**模型，让 EEPROM、BMI088 这类 I2C 设备在 RTOS 下不再独占 CPU；保持公共 API 不变，应用层无需大改。

---

## 0. 适用范围与基本信息

| 项 | 值 |
|---|---|
| 目标 MCU | HC32F460（LQFP100，DDL Rev3.3.0） |
| OS | FreeRTOS V202012.00（已在用） |
| 改造文件 | `communication/i2c/i2c.h`、`communication/i2c/i2c.c`（新增中断版） |
| 受影响应用 | `applications/eeprom/eeprom.c`、`applications/imu/bmi088.c` |
| 启动/调度 | `main.c` → `Board_Init` → `App_Init` → `App_StartTasks` → `vTaskStartScheduler` |
| 当前波特率 | 100 kHz（I2C1=EEPROM, I2C2=BMI088） |

---

## 1. 可行性分析

### 1.1 软件层

| 维度 | 现状 | 中断化后 | 结论 |
|------|------|----------|------|
| API 签名 | `i2c_init/i2c_write/i2c_read/i2c_check_ack/i2c_recover` | 完全保留 | ✅ 应用零改动 |
| 调用上下文 | 任务上下文（已是） | 任务上下文 + 等信号量 | ✅ 与 FreeRTOS 天然契合 |
| 总线恢复 | GPIO bit-banging `i2c_recover` | 保留（与中断无关） | ✅ |
| 多实例 | I2C1/I2C2 通过 `i2c_handle_t` 区分 | 每实例独立状态机/信号量/互斥量 | ✅ |
| FreeRTOS 兼容性 | `configMAX_SYSCALL_INTERRUPT_PRIORITY = 1` | I2C IRQ 优先级取 ≥5（数值小=高），满足 `≥1` 即可调用 `FromISR` | ✅ |

### 1.2 硬件层

HC32F460 每个 I2C 单元有 **4 个独立中断源**：

| 中断 | 含义 | 触发时机 |
|------|------|----------|
| EEI | Event/Error | START、STOP、NACK、ARLO、TIMEOUT |
| RXI | RX Full | 接收寄存器满 |
| TXI | TX Empty | 发送寄存器空 |
| TEI | TX Complete | 字节完整送上总线 + 收到 ACK/NACK |

每个中断源通过 `INTC_IrqSignIn` 绑定到任意 `INT000~INT127_IRQn` 槽位，**资源充足**。

### 1.3 当前 IRQ 资源占用（已确认）

| IRQn | 当前使用 |
|------|----------|
| INT000~INT004 | UART1 DMA 链路 |
| INT005~INT009 | UART4 DMA 链路 |
| INT007 | SPI DMA RX（**与 UART4_TX_CPLT 同号，已存在冲突，本次不涉及**） |
| INT010 | CAN |
| **INT011~INT018** | **空闲 → 分配给 I2C1/I2C2** |

### 1.4 难点与对策

| 难点 | 对策 |
|------|------|
| 读时序含 **Repeated START**（`S→Addr+W→MemAddr→Sr→Addr+R→Data→P`），比官方 `i2c_master_int` 示例（无寄存器地址）复杂 | 引入 `phase` 字段：`PHASE_TX_REG`、`PHASE_RESTART`、`PHASE_RX_DATA`，TEI 完成寄存器地址后触发 `I2C_GenerateStart` 等待新一轮 START 中断 |
| 写流程含 **寄存器地址 + 数据** 两段 TX | 用「双缓冲」TX 视图：`tx0(=reg_addr)` + `tx1(=data)`，TXI 顺序取 |
| 异常退出路径（NACK / ARLO / TIMEOUT）必须释放信号量 | EEI 三个分支统一进入 `i2c_finalize_(err)`，无条件 `xSemaphoreGiveFromISR` |
| `i2c_check_ack` 仅发地址不发数据，需要单字节流转 | 引入 `OP_CHECK_ACK`，地址 ACK/NACK 后立即 STOP |
| EEPROM `ACK Polling` 频率高（每 1ms 一次），不能有 RAM 抖动 | 复用全局 `i2c_handle_t.state`，禁止 `malloc`；超时由信号量 `xSemaphoreTake` 的 timeout 控制 |

**结论：方案可行，工作量集中在 `i2c.c` 一个文件的重写；其他文件 ≤ 5 行改动。**

---

## 2. 整体架构

### 2.1 分层

```
┌─────────────────────────────────────┐
│ Application（不变）                  │
│  eeprom.c / bmi088.c                │
└──────────────┬──────────────────────┘
               │ i2c_write/read/check_ack/recover
┌──────────────▼──────────────────────┐
│ I2C HAL（本次改造）                  │
│  - 阻塞 API（互斥 + 信号量）         │
│  - 状态机（OP × PHASE）              │
│  - 4 个 ISR 回调                     │
└──────────────┬──────────────────────┘
               │ I2C_WriteData / I2C_ReadData / I2C_IntCmd ...
┌──────────────▼──────────────────────┐
│ HC32 DDL（不变）hc32_ll_i2c.c        │
└─────────────────────────────────────┘
```

### 2.2 同步机制

每个 I2C 实例（句柄）含：

- `bus_mtx`：`SemaphoreHandle_t`（Mutex），同一实例多任务串行
- `done_sem`：`SemaphoreHandle_t`（二值信号量），任务在 ISR 完成时被唤醒
- `state`：内部状态机字段（仅 ISR 与 API 内部读写，受互斥锁保护）
- `err`：ISR 写入、API 读取的错误码

```
任务上下文（持有 bus_mtx）：
   填 state.op / 填 buf / Take(done_sem, timeout)  ────┐
                                                       │
中断上下文（EEI/RXI/TXI/TEI）：                          │
   推进 state.phase / 操作硬件 → 完成时 GiveFromISR ←──┘
```

### 2.3 状态机

```c
typedef enum {
    OP_NONE        = 0,
    OP_WRITE,        /* S Addr+W [reg_addr][data] P              */
    OP_READ,         /* S Addr+W [reg_addr] Sr Addr+R [data] P   */
    OP_CHECK_ACK,    /* S Addr+W P                               */
} i2c_op_t;

typedef enum {
    PHASE_IDLE         = 0,
    PHASE_WAIT_START,    /* 已 GenerateStart，等 EEI 的 STARTF       */
    PHASE_TX_ADDR_W,     /* 已写入 Addr+W，等 TEI                    */
    PHASE_TX_REG,        /* 通过 TXI 推送 reg_addr                   */
    PHASE_TX_DATA,       /* 通过 TXI 推送 data                       */
    PHASE_TX_LAST,       /* 最后一字节已入寄存器，等 TEI 触发 STOP/Sr */
    PHASE_WAIT_RESTART,  /* 已 GenerateStart（第二次），等 STARTF      */
    PHASE_TX_ADDR_R,     /* 已写入 Addr+R，等 TEI                    */
    PHASE_RX_DATA,       /* 通过 RXI 接收                            */
    PHASE_WAIT_STOP,     /* 已 GenerateStop，等 STOPF                */
    PHASE_DONE,
} i2c_phase_t;
```

### 2.4 主要时序

#### 2.4.1 写：`i2c_write(... reg_addr_len=2, len=N)`

```
任务         EEI           TEI           TXI
 │           │             │             │
 │ Start ──► │             │             │
 │           │ STARTF      │             │
 │           │  写 Addr+W  │             │
 │           │  开 TEI     │             │
 │           │             │ 地址ACK     │
 │           │             │  写 reg[0]  │
 │           │             │  开 TXI     │
 │           │             │             │ TX空 写 reg[1]
 │           │             │             │ TX空 写 data[0]
 │           │             │             │  ...
 │           │             │             │ TX空 写 data[N-1]
 │           │             │             │ 关 TXI 开 TEI
 │           │             │ 末字节完成  │
 │           │             │  GenStop    │
 │           │             │  开 STOP    │
 │           │ STOPF       │             │
 │           │  Give 信号量│             │
 │ ◄─ Take ──┘             │             │
 │           │             │             │
```

#### 2.4.2 读：`i2c_read(... reg_addr_len=2, len=N)`

```
任务   EEI         TEI               TXI          RXI
 │     │           │                 │            │
 │ S ─►│           │                 │            │
 │     │STARTF     │                 │            │
 │     │ 写Addr+W  │                 │            │
 │     │ 开TEI     │                 │            │
 │     │           │ 地址ACK         │            │
 │     │           │  写reg[0] 开TXI │            │
 │     │           │                 │ 写reg[1]   │
 │     │           │                 │ 关TXI开TEI │
 │     │           │ reg末字节完成   │            │
 │     │           │  GenStart(Sr)   │            │
 │     │STARTF(Sr) │                 │            │
 │     │ 写Addr+R  │                 │            │
 │     │ 若len==1  │                 │            │
 │     │  AckCfg   │                 │            │
 │     │  (NACK)   │                 │            │
 │     │ 开RXI     │                 │            │
 │     │           │ 地址ACK         │            │
 │     │           │ (TEI禁用即可)   │            │
 │     │           │                 │            │ 收到字节
 │     │           │                 │            │  BufWrite
 │     │           │                 │            │ ...
 │     │           │                 │            │ index==len-2:
 │     │           │                 │            │  AckCfg(NACK)
 │     │           │                 │            │ index==len-1:
 │     │           │                 │            │  GenStop
 │     │           │                 │            │  关RXI 开STOP
 │     │STOPF      │                 │            │
 │     │ Give      │                 │            │
 │ ◄───┘           │                 │            │
```

> 注：读 1 字节的特例（在 START 阶段就提前 `I2C_AckConfig(NACK)`），与官方 `i2c_master_int` 示例一致。

#### 2.4.3 异常退出

```
任意阶段
   ↓
NACK / ARLO 触发 EEI
   ↓
关所有数据中断 → GenerateStop → 标记 err = LL_ERR
   ↓
EEI 的 STOPF 分支统一执行：disable I2C → state=IDLE → Give(done_sem)
```

---

## 3. IRQ 资源与优先级分配

### 3.1 IRQn 槽位

| 实例 | 中断 | IRQn | 中断源（INTC） | 优先级 |
|------|------|------|----------------|--------|
| I2C1 | EEI | `INT011_IRQn` | `INT_SRC_I2C1_EEI` | **5** |
| I2C1 | RXI | `INT012_IRQn` | `INT_SRC_I2C1_RXI` | 6 |
| I2C1 | TXI | `INT013_IRQn` | `INT_SRC_I2C1_TXI` | 6 |
| I2C1 | TEI | `INT014_IRQn` | `INT_SRC_I2C1_TEI` | 6 |
| I2C2 | EEI | `INT015_IRQn` | `INT_SRC_I2C2_EEI` | **5** |
| I2C2 | RXI | `INT016_IRQn` | `INT_SRC_I2C2_RXI` | 6 |
| I2C2 | TXI | `INT017_IRQn` | `INT_SRC_I2C2_TXI` | 6 |
| I2C2 | TEI | `INT018_IRQn` | `INT_SRC_I2C2_TEI` | 6 |

### 3.2 优先级约束（来自 `FreeRTOSConfig.h`）

```c
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 1   // 数值越小优先级越高
configLIBRARY_LOWEST_INTERRUPT_PRIORITY      = 0xF
configPRIO_BITS = 4
NVIC_SetPriorityGrouping(3)  // 全部 4 bit 为抢占优先级
```

- I2C 回调里要调用 `xSemaphoreGiveFromISR`，**抢占优先级数值必须 ≥ 1**（数值小=优先级高）
- I2C **EEI 必须比 RXI/TXI/TEI 优先级更高**（更小），否则可能错过 START/STOP/NACK 时序，按 5 < 6 已满足
- 与现有 UART/CAN（默认优先级，一般为 0xF 或 DDL_IRQ_PRIO_DEFAULT=15）共存无问题

---

## 4. 文件清单

### 4.1 新增/修改一览

| 文件 | 操作 | 说明 |
|------|------|------|
| `communication/i2c/i2c.h` | **小改** | 句柄新增 `done_sem`、`bus_mtx`、`state`；API 不变 |
| `communication/i2c/i2c.c` | **重写** | 全量替换为中断版（保留 `i2c_recover` 实现） |
| `applications/eeprom/eeprom.c` | **不改** | API 兼容；`vTaskDelay` 已经存在 |
| `applications/imu/bmi088.c` | **不改** | 同上 |
| `applications/app.c` | **不改** | 不动 |
| `board/board.c` | 视情况 | 如果 I2C 初始化挪到 `Board_Init`，否则保持 `Eeprom_Task`/`BMI088_Task` 里调 `i2cX_init()` |

### 4.2 头文件包含关系（不变）

```
eeprom.c / bmi088.c
  └── i2c.h
        └── hc32_ll.h（DDL）
i2c.c（新增）
  ├── i2c.h
  ├── hc32_ll.h
  ├── FreeRTOS.h / semphr.h
```

---

## 5. 详细设计：`i2c.h`

```c
#ifndef __I2C_H__
#define __I2C_H__

#include "hc32_ll.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ---------- 公共错误码（保持向后兼容） ---------- */
/* 直接复用 LL_OK / LL_ERR / LL_ERR_TIMEOUT / LL_ERR_BUSY */

/* ---------- 实例硬件配置 ---------- */
typedef struct {
    CM_I2C_TypeDef *unit;
    uint32_t        fcg;
    uint8_t         scl_port;
    uint16_t        scl_pin;
    uint8_t         sda_port;
    uint16_t        sda_pin;
    uint8_t         scl_func;
    uint8_t         sda_func;
    uint32_t        baudrate;
    uint32_t        clk_div;
    uint32_t        scl_time;
    uint32_t        timeout_ms;     /* 改：原 timeout 计数 → 毫秒数（信号量超时） */

    /* 中断资源 */
    IRQn_Type       irqn_eei;
    IRQn_Type       irqn_rxi;
    IRQn_Type       irqn_txi;
    IRQn_Type       irqn_tei;
    en_int_src_t    src_eei;
    en_int_src_t    src_rxi;
    en_int_src_t    src_txi;
    en_int_src_t    src_tei;
    uint8_t         irq_prio_eei;   /* 例如 5 */
    uint8_t         irq_prio_data;  /* 例如 6 */
} i2c_config_t;

/* ---------- 操作/阶段（内部用，外部不感知） ---------- */
typedef enum {
    I2C_OP_NONE = 0,
    I2C_OP_WRITE,
    I2C_OP_READ,
    I2C_OP_CHECK_ACK,
} i2c_op_t;

typedef enum {
    I2C_PHASE_IDLE = 0,
    I2C_PHASE_WAIT_START,
    I2C_PHASE_TX_ADDR_W,
    I2C_PHASE_TX_REG,
    I2C_PHASE_TX_DATA,
    I2C_PHASE_TX_LAST,
    I2C_PHASE_WAIT_RESTART,
    I2C_PHASE_TX_ADDR_R,
    I2C_PHASE_RX_DATA,
    I2C_PHASE_WAIT_STOP,
} i2c_phase_t;

/* ---------- 实例句柄 ---------- */
typedef struct {
    const i2c_config_t *cfg;

    /* 同步对象 */
    SemaphoreHandle_t   bus_mtx;       /* 总线互斥 */
    SemaphoreHandle_t   done_sem;      /* 传输完成信号量（二值） */
    StaticSemaphore_t   bus_mtx_buf;   /* 可选：静态分配 */
    StaticSemaphore_t   done_sem_buf;

    /* 运行时状态（仅 ISR 与 API 内部访问） */
    volatile i2c_op_t    op;
    volatile i2c_phase_t phase;
    volatile int32_t     err;

    uint16_t        dev_addr;        /* 7-bit */
    /* TX 双段视图：先 tx0（寄存器地址），再 tx1（数据） */
    const uint8_t  *tx0;  uint32_t tx0_len;  volatile uint32_t tx0_idx;
    const uint8_t  *tx1;  uint32_t tx1_len;  volatile uint32_t tx1_idx;
    /* RX */
    uint8_t        *rx;   uint32_t rx_len;   volatile uint32_t rx_idx;
} i2c_handle_t;

/* ---------- 公共 API（保持原签名） ---------- */
int32_t i2c_init(i2c_handle_t *hi2c, const i2c_config_t *cfg);

int32_t i2c_write(i2c_handle_t *hi2c, uint16_t dev_addr, uint16_t reg_addr,
                  uint8_t reg_addr_len, const uint8_t *buf, uint32_t len);

int32_t i2c_read (i2c_handle_t *hi2c, uint16_t dev_addr, uint16_t reg_addr,
                  uint8_t reg_addr_len, uint8_t *buf, uint32_t len);

int32_t i2c_check_ack(i2c_handle_t *hi2c, uint16_t dev_addr);

int32_t i2c_recover(i2c_handle_t *hi2c);

/* ---------- 预定义实例 ---------- */
extern i2c_handle_t g_hi2c1;
extern i2c_handle_t g_hi2c2;
int32_t i2c1_init(void);
int32_t i2c2_init(void);

#endif
```

> 注意 `timeout` 字段语义变化：原来是 DDL 内部循环计数，新版改为 **毫秒**，传给 `xSemaphoreTake`。预定义配置里取 `100`（保守值，单次传输 256 字节 @ 100kHz 约 ~25ms）。

---

## 6. 详细设计：`i2c.c`

### 6.1 预定义配置

```c
static const i2c_config_t s_i2c1_cfg = {
    .unit          = CM_I2C1,
    .fcg           = FCG1_PERIPH_I2C1,
    .scl_port      = GPIO_PORT_A, .scl_pin = GPIO_PIN_11,
    .sda_port      = GPIO_PORT_A, .sda_pin = GPIO_PIN_10,
    .scl_func      = GPIO_FUNC_49, .sda_func = GPIO_FUNC_48,
    .baudrate      = 100000UL,
    .clk_div       = I2C_CLK_DIV2,
    .scl_time      = 3UL,
    .timeout_ms    = 100U,
    .irqn_eei      = INT011_IRQn, .src_eei = INT_SRC_I2C1_EEI, .irq_prio_eei = 5U,
    .irqn_rxi      = INT012_IRQn, .src_rxi = INT_SRC_I2C1_RXI,
    .irqn_txi      = INT013_IRQn, .src_txi = INT_SRC_I2C1_TXI,
    .irqn_tei      = INT014_IRQn, .src_tei = INT_SRC_I2C1_TEI,
    .irq_prio_data = 6U,
};

static const i2c_config_t s_i2c2_cfg = {
    .unit          = CM_I2C2,
    .fcg           = FCG1_PERIPH_I2C2,
    .scl_port      = GPIO_PORT_A, .scl_pin = GPIO_PIN_09,
    .sda_port      = GPIO_PORT_A, .sda_pin = GPIO_PIN_08,
    .scl_func      = GPIO_FUNC_51, .sda_func = GPIO_FUNC_50,
    .baudrate      = 100000UL,
    .clk_div       = I2C_CLK_DIV2,
    .scl_time      = 3UL,
    .timeout_ms    = 100U,
    .irqn_eei      = INT015_IRQn, .src_eei = INT_SRC_I2C2_EEI, .irq_prio_eei = 5U,
    .irqn_rxi      = INT016_IRQn, .src_rxi = INT_SRC_I2C2_RXI,
    .irqn_txi      = INT017_IRQn, .src_txi = INT_SRC_I2C2_TXI,
    .irqn_tei      = INT018_IRQn, .src_tei = INT_SRC_I2C2_TEI,
    .irq_prio_data = 6U,
};

i2c_handle_t g_hi2c1;
i2c_handle_t g_hi2c2;
```

### 6.2 初始化

```c
static i2c_handle_t *s_irq_handle_i2c1; /* ISR 调度 */
static i2c_handle_t *s_irq_handle_i2c2;

static void i2c_eei_isr_i2c1(void); static void i2c_rxi_isr_i2c1(void);
static void i2c_txi_isr_i2c1(void); static void i2c_tei_isr_i2c1(void);
static void i2c_eei_isr_i2c2(void); static void i2c_rxi_isr_i2c2(void);
static void i2c_txi_isr_i2c2(void); static void i2c_tei_isr_i2c2(void);

static void register_irq(IRQn_Type irqn, en_int_src_t src, func_ptr_t cb, uint8_t prio)
{
    stc_irq_signin_config_t c;
    c.enIRQn = irqn; c.enIntSrc = src; c.pfnCallback = cb;
    (void)INTC_IrqSignIn(&c);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, prio);
    NVIC_EnableIRQ(irqn);
}

int32_t i2c_init(i2c_handle_t *h, const i2c_config_t *cfg)
{
    stc_gpio_init_t  gpio;
    stc_i2c_init_t   i2c;
    float32_t        ferr;
    int32_t          ret;

    h->cfg   = cfg;
    h->op    = I2C_OP_NONE;
    h->phase = I2C_PHASE_IDLE;
    h->err   = LL_OK;

    /* 1. GPIO */
    (void)GPIO_StructInit(&gpio);
    (void)GPIO_Init(cfg->scl_port, cfg->scl_pin, &gpio);
    (void)GPIO_Init(cfg->sda_port, cfg->sda_pin, &gpio);
    GPIO_SetFunc(cfg->scl_port, cfg->scl_pin, cfg->scl_func);
    GPIO_SetFunc(cfg->sda_port, cfg->sda_pin, cfg->sda_func);

    /* 2. 时钟 */
    FCG_Fcg1PeriphClockCmd(cfg->fcg, ENABLE);

    /* 3. 外设 */
    (void)I2C_DeInit(cfg->unit);
    (void)I2C_StructInit(&i2c);
    i2c.u32ClockDiv = cfg->clk_div;
    i2c.u32Baudrate = cfg->baudrate;
    i2c.u32SclTime  = cfg->scl_time;
    ret = I2C_Init(cfg->unit, &i2c, &ferr);
    if (ret != LL_OK) return ret;
    I2C_BusWaitCmd(cfg->unit, ENABLE);

    /* 4. 同步对象（首次初始化时创建） */
    if (h->bus_mtx == NULL) {
        h->bus_mtx  = xSemaphoreCreateMutexStatic(&h->bus_mtx_buf);
        h->done_sem = xSemaphoreCreateBinaryStatic(&h->done_sem_buf);
        configASSERT(h->bus_mtx && h->done_sem);
    }

    /* 5. ISR 调度槽位 */
    if (cfg->unit == CM_I2C1) s_irq_handle_i2c1 = h;
    else if (cfg->unit == CM_I2C2) s_irq_handle_i2c2 = h;

    /* 6. 中断注册（每实例 4 个）*/
    if (cfg->unit == CM_I2C1) {
        register_irq(cfg->irqn_eei, cfg->src_eei, &i2c_eei_isr_i2c1, cfg->irq_prio_eei);
        register_irq(cfg->irqn_rxi, cfg->src_rxi, &i2c_rxi_isr_i2c1, cfg->irq_prio_data);
        register_irq(cfg->irqn_txi, cfg->src_txi, &i2c_txi_isr_i2c1, cfg->irq_prio_data);
        register_irq(cfg->irqn_tei, cfg->src_tei, &i2c_tei_isr_i2c1, cfg->irq_prio_data);
    } else {
        register_irq(cfg->irqn_eei, cfg->src_eei, &i2c_eei_isr_i2c2, cfg->irq_prio_eei);
        register_irq(cfg->irqn_rxi, cfg->src_rxi, &i2c_rxi_isr_i2c2, cfg->irq_prio_data);
        register_irq(cfg->irqn_txi, cfg->src_txi, &i2c_txi_isr_i2c2, cfg->irq_prio_data);
        register_irq(cfg->irqn_tei, cfg->src_tei, &i2c_tei_isr_i2c2, cfg->irq_prio_data);
    }

    return LL_OK;
}
```

### 6.3 通用启动/结束辅助

```c
static void i2c_start_transfer(i2c_handle_t *h)
{
    /* 与官方 i2c_master_int 一致：开启 I2C → 打开 START 中断 → 软复位 → GenerateStart */
    I2C_Cmd(h->cfg->unit, ENABLE);
    I2C_IntCmd(h->cfg->unit, I2C_INT_START, ENABLE);
    I2C_SWResetCmd(h->cfg->unit, ENABLE);
    I2C_SWResetCmd(h->cfg->unit, DISABLE);
    h->phase = I2C_PHASE_WAIT_START;
    I2C_GenerateStart(h->cfg->unit);
}

static void i2c_finalize_isr(i2c_handle_t *h, int32_t err, BaseType_t *hpw)
{
    I2C_IntCmd(h->cfg->unit,
        I2C_INT_START | I2C_INT_STOP | I2C_INT_NACK |
        I2C_INT_TX_EMPTY | I2C_INT_TX_CPLT | I2C_INT_RX_FULL, DISABLE);
    I2C_ClearStatus(h->cfg->unit, I2C_FLAG_CLR_ALL);
    I2C_Cmd(h->cfg->unit, DISABLE);
    I2C_AckConfig(h->cfg->unit, I2C_ACK);
    h->err   = err;
    h->phase = I2C_PHASE_IDLE;
    h->op    = I2C_OP_NONE;
    (void)xSemaphoreGiveFromISR(h->done_sem, hpw);
}

static inline bool tx_has_more(const i2c_handle_t *h)
{
    return (h->tx0_idx < h->tx0_len) || (h->tx1_idx < h->tx1_len);
}

static inline uint32_t tx_remaining(const i2c_handle_t *h)
{
    return (h->tx0_len - h->tx0_idx) + (h->tx1_len - h->tx1_idx);
}

static inline uint8_t tx_pop(i2c_handle_t *h)
{
    if (h->tx0_idx < h->tx0_len) return h->tx0[h->tx0_idx++];
    return h->tx1[h->tx1_idx++];
}
```

### 6.4 ISR 回调（4 个，按 I2C 单元区分）

```c
static void i2c_on_eei(i2c_handle_t *h, BaseType_t *hpw)
{
    CM_I2C_TypeDef *u = h->cfg->unit;

    /* ----- STARTF：刚发出 S 或 Sr ----- */
    if (I2C_GetStatus(u, I2C_FLAG_START) == SET) {
        I2C_ClearStatus(u, I2C_FLAG_CLR_START | I2C_FLAG_CLR_NACK);
        I2C_IntCmd(u, I2C_INT_STOP | I2C_INT_NACK, ENABLE);

        if (h->phase == I2C_PHASE_WAIT_START) {
            /* 首次 S：先发 Addr+W（写、读、check_ack 都走这里） */
            I2C_IntCmd(u, I2C_INT_TX_CPLT, ENABLE);
            I2C_WriteData(u, (uint8_t)((h->dev_addr << 1) | I2C_DIR_TX));
            h->phase = I2C_PHASE_TX_ADDR_W;
        } else if (h->phase == I2C_PHASE_WAIT_RESTART) {
            /* Sr：发 Addr+R */
            if (h->rx_len == 1U) I2C_AckConfig(u, I2C_NACK);
            I2C_IntCmd(u, I2C_INT_RX_FULL, ENABLE);
            I2C_WriteData(u, (uint8_t)((h->dev_addr << 1) | I2C_DIR_RX));
            h->phase = I2C_PHASE_TX_ADDR_R;
        }
        return;
    }

    /* ----- NACK：从机不响应 ----- */
    if (I2C_GetStatus(u, I2C_FLAG_NACKF) == SET) {
        I2C_ClearStatus(u, I2C_FLAG_CLR_NACK);
        I2C_IntCmd(u, I2C_INT_TX_EMPTY | I2C_INT_RX_FULL | I2C_INT_TX_CPLT | I2C_INT_NACK, DISABLE);
        I2C_GenerateStop(u);
        h->err   = LL_ERR;
        h->phase = I2C_PHASE_WAIT_STOP;
        return;
    }

    /* ----- STOPF：传输结束 ----- */
    if (I2C_GetStatus(u, I2C_FLAG_STOP) == SET) {
        i2c_finalize_isr(h, h->err, hpw);
    }
}

static void i2c_on_tei(i2c_handle_t *h, BaseType_t *hpw)
{
    CM_I2C_TypeDef *u = h->cfg->unit;
    I2C_IntCmd(u, I2C_INT_TX_CPLT, DISABLE);

    switch (h->phase) {

    case I2C_PHASE_TX_ADDR_W: {
        /* 地址 ACK 收到 → 发首字节（reg_addr 或 data） */
        if (h->op == I2C_OP_CHECK_ACK) {
            /* 只为探测，立即 STOP */
            I2C_IntCmd(u, I2C_INT_STOP, ENABLE);
            I2C_GenerateStop(u);
            h->phase = I2C_PHASE_WAIT_STOP;
            return;
        }
        if (!tx_has_more(h)) {
            /* 写 0 字节（理论上不会发生）→ STOP */
            I2C_IntCmd(u, I2C_INT_STOP, ENABLE);
            I2C_GenerateStop(u);
            h->phase = I2C_PHASE_WAIT_STOP;
            return;
        }
        I2C_IntCmd(u, I2C_INT_TX_EMPTY, ENABLE);
        I2C_WriteData(u, tx_pop(h));
        h->phase = (h->tx0_idx <= h->tx0_len) ? I2C_PHASE_TX_REG : I2C_PHASE_TX_DATA;
        return;
    }

    case I2C_PHASE_TX_LAST: {
        /* 最后一字节已经上总线 */
        if (h->op == I2C_OP_READ) {
            /* 寄存器地址写完 → 触发 Repeated START */
            h->phase = I2C_PHASE_WAIT_RESTART;
            I2C_IntCmd(u, I2C_INT_START, ENABLE);
            I2C_GenerateStart(u);
        } else {
            /* WRITE 完成 → STOP */
            I2C_IntCmd(u, I2C_INT_STOP, ENABLE);
            I2C_GenerateStop(u);
            h->phase = I2C_PHASE_WAIT_STOP;
        }
        return;
    }

    case I2C_PHASE_TX_ADDR_R:
        /* 读地址 ACK，RXI 已开启，TEI 用完即关 */
        return;

    default:
        return;
    }
}

static void i2c_on_txi(i2c_handle_t *h, BaseType_t *hpw)
{
    CM_I2C_TypeDef *u = h->cfg->unit;

    if (tx_has_more(h)) {
        I2C_WriteData(u, tx_pop(h));
        /* 若 op==READ 且写完了 tx0（reg_addr），停在最后一个 reg byte → 切到等 TEI */
        if (h->op == I2C_OP_READ && h->tx0_idx == h->tx0_len && h->tx1_len == 0U) {
            /* 已经把最后一个 reg byte 灌进去 */
            I2C_IntCmd(u, I2C_INT_TX_EMPTY, DISABLE);
            I2C_IntCmd(u, I2C_INT_TX_CPLT, ENABLE);
            h->phase = I2C_PHASE_TX_LAST;
        }
        return;
    }
    /* 没有更多字节要发了 → 关 TXI，开 TEI 等末字节发完 */
    I2C_IntCmd(u, I2C_INT_TX_EMPTY, DISABLE);
    I2C_IntCmd(u, I2C_INT_TX_CPLT, ENABLE);
    h->phase = I2C_PHASE_TX_LAST;
}

static void i2c_on_rxi(i2c_handle_t *h, BaseType_t *hpw)
{
    CM_I2C_TypeDef *u = h->cfg->unit;

    /* 倒数第 2 字节：下一字节回 NACK */
    if (h->rx_len >= 2U && h->rx_idx == h->rx_len - 2U) {
        I2C_AckConfig(u, I2C_NACK);
    }
    /* 最后一字节：先 STOP 再读 */
    if (h->rx_idx == h->rx_len - 1U) {
        I2C_IntCmd(u, I2C_INT_STOP, ENABLE);
        I2C_IntCmd(u, I2C_INT_RX_FULL, DISABLE);
        I2C_GenerateStop(u);
        I2C_AckConfig(u, I2C_ACK);
        h->phase = I2C_PHASE_WAIT_STOP;
    }
    h->rx[h->rx_idx++] = I2C_ReadData(u);
}
```

#### 6.4.1 中断入口（薄壳）

```c
static void i2c_eei_isr_i2c1(void) { BaseType_t hpw = pdFALSE; i2c_on_eei(s_irq_handle_i2c1, &hpw); portYIELD_FROM_ISR(hpw); }
static void i2c_rxi_isr_i2c1(void) { BaseType_t hpw = pdFALSE; i2c_on_rxi(s_irq_handle_i2c1, &hpw); portYIELD_FROM_ISR(hpw); }
static void i2c_txi_isr_i2c1(void) { BaseType_t hpw = pdFALSE; i2c_on_txi(s_irq_handle_i2c1, &hpw); portYIELD_FROM_ISR(hpw); }
static void i2c_tei_isr_i2c1(void) { BaseType_t hpw = pdFALSE; i2c_on_tei(s_irq_handle_i2c1, &hpw); portYIELD_FROM_ISR(hpw); }
/* I2C2 同理，把 s_irq_handle_i2c2 传入 */
```

### 6.5 阻塞 API

```c
static int32_t i2c_xfer_blocking(i2c_handle_t *h)
{
    /* 1. 进入临界区前：清空残留信号量 */
    (void)xSemaphoreTake(h->done_sem, 0);

    /* 2. 启动硬件 */
    i2c_start_transfer(h);

    /* 3. 等完成 */
    if (xSemaphoreTake(h->done_sem, pdMS_TO_TICKS(h->cfg->timeout_ms)) != pdTRUE) {
        /* 超时：强制清干净 */
        portENTER_CRITICAL();
        I2C_IntCmd(h->cfg->unit, I2C_INT_ALL, DISABLE);
        I2C_Cmd(h->cfg->unit, DISABLE);
        h->phase = I2C_PHASE_IDLE;
        h->op    = I2C_OP_NONE;
        portEXIT_CRITICAL();
        return LL_ERR_TIMEOUT;
    }
    return h->err;
}

int32_t i2c_write(i2c_handle_t *h, uint16_t dev_addr, uint16_t reg_addr,
                  uint8_t reg_addr_len, const uint8_t *buf, uint32_t len)
{
    static uint8_t reg_buf_static[2];   /* 仅同实例使用，受 bus_mtx 保护 */
    int32_t ret;

    if ((buf == NULL && len > 0U) || (reg_addr_len > 2U)) return LL_ERR_INVD_PARAM;
    xSemaphoreTake(h->bus_mtx, portMAX_DELAY);

    if (reg_addr_len == 2U) {
        reg_buf_static[0] = (uint8_t)(reg_addr >> 8);
        reg_buf_static[1] = (uint8_t)(reg_addr & 0xFFU);
    } else {
        reg_buf_static[0] = (uint8_t)(reg_addr & 0xFFU);
    }

    h->op        = I2C_OP_WRITE;
    h->err       = LL_OK;
    h->dev_addr  = dev_addr & 0x7FU;
    h->tx0       = reg_buf_static; h->tx0_len = reg_addr_len; h->tx0_idx = 0;
    h->tx1       = buf;            h->tx1_len = len;          h->tx1_idx = 0;
    h->rx        = NULL;           h->rx_len  = 0;            h->rx_idx  = 0;

    ret = i2c_xfer_blocking(h);

    xSemaphoreGive(h->bus_mtx);
    return ret;
}

int32_t i2c_read(i2c_handle_t *h, uint16_t dev_addr, uint16_t reg_addr,
                 uint8_t reg_addr_len, uint8_t *buf, uint32_t len)
{
    static uint8_t reg_buf_static[2];
    int32_t ret;

    if (buf == NULL || len == 0U || reg_addr_len > 2U) return LL_ERR_INVD_PARAM;
    xSemaphoreTake(h->bus_mtx, portMAX_DELAY);

    if (reg_addr_len == 2U) {
        reg_buf_static[0] = (uint8_t)(reg_addr >> 8);
        reg_buf_static[1] = (uint8_t)(reg_addr & 0xFFU);
    } else {
        reg_buf_static[0] = (uint8_t)(reg_addr & 0xFFU);
    }

    h->op        = I2C_OP_READ;
    h->err       = LL_OK;
    h->dev_addr  = dev_addr & 0x7FU;
    h->tx0       = reg_buf_static; h->tx0_len = reg_addr_len; h->tx0_idx = 0;
    h->tx1       = NULL;           h->tx1_len = 0;            h->tx1_idx = 0;
    h->rx        = buf;            h->rx_len  = len;          h->rx_idx  = 0;

    ret = i2c_xfer_blocking(h);

    xSemaphoreGive(h->bus_mtx);
    return ret;
}

int32_t i2c_check_ack(i2c_handle_t *h, uint16_t dev_addr)
{
    int32_t ret;
    xSemaphoreTake(h->bus_mtx, portMAX_DELAY);

    h->op        = I2C_OP_CHECK_ACK;
    h->err       = LL_OK;
    h->dev_addr  = dev_addr & 0x7FU;
    h->tx0_len = h->tx0_idx = 0;
    h->tx1_len = h->tx1_idx = 0;
    h->rx_len  = h->rx_idx  = 0;

    ret = i2c_xfer_blocking(h);

    xSemaphoreGive(h->bus_mtx);
    return ret;
}
```

> **关键细节**：`reg_buf_static[2]` 必须按实例独立（I2C1 与 I2C2 各一份），不能两个实例共用。文档里写的是 per-instance 静态变量，落地时建议放到 `i2c_handle_t` 内（或写两份 static 数组）。

### 6.6 `i2c_recover` 实现

**直接复用现有 `i2c.c` 中的 `i2c_recover()` 函数体**（GPIO bit-bang 9 个时钟），结尾把 `i2c_init(h, h->cfg)` 替换为新版即可。

---

## 7. 应用层适配

### 7.1 `eeprom.c`

**无需修改**。`Eeprom_Task` 内现有：

```c
void Eeprom_Task(void *pvParameters) {
    (void)pvParameters;
    i2c1_init();        /* 在任务里调，调度器已起 → 可以安全创建信号量 */
    for (;;) { ... }
}
```

> ⚠️ 必须确保 `i2c1_init()` 在 **`vTaskStartScheduler()` 之后** 被调用（当前 `Eeprom_Task` 内调用，满足）。若以后挪到 `Board_Init`，需改用 `xSemaphoreCreateMutexStatic` 静态分配。

### 7.2 `bmi088.c`

**无需修改**。`BMI088_Task` 类似处理。

### 7.3 `app.c`

**无需修改**。已经在 `App_StartTasks` 里创建了 `Eeprom_Task` 和 `BMI088_Task`。

### 7.4 `main.h`

如果 `main.h` 或某全局头里没有 `#include "FreeRTOS.h"` 与 `"semphr.h"`，需要在新的 `i2c.h` 中显式包含（已在 §5 中写明）。

---

## 8. NVIC 与 FreeRTOS 协同检查清单

| 项目 | 期望值 | 备注 |
|------|--------|------|
| `NVIC_SetPriorityGrouping` | `3` | `App_Init` 中已设 |
| `configMAX_SYSCALL_INTERRUPT_PRIORITY` | `1 << 4 = 0x10` | I2C 的实际 NVIC 寄存器值必须 ≥ 0x10 |
| `NVIC_SetPriority(irqn, 5)` | 写入 `5<<4 = 0x50` | ✅ 满足 |
| `NVIC_SetPriority(irqn, 6)` | 写入 `6<<4 = 0x60` | ✅ 满足 |
| ISR 内仅调用 `*FromISR` 系列 | — | 严禁使用 `xSemaphoreGive` 等非 ISR API |

---

## 9. 错误处理与边界情况

| 场景 | 处理 |
|------|------|
| 任何 ISR 检测到 NACK | EEI → GenerateStop → `WAIT_STOP` → finalize(`LL_ERR`) |
| 仲裁失败（ARLO） | EEI 同 NACK 处理（清 ARLO 标志、STOP、`LL_ERR_BUSY`） |
| 信号量等待超时 | `i2c_xfer_blocking` 主动关中断 + `I2C_Cmd(DISABLE)`，返回 `LL_ERR_TIMEOUT`；调用方可调 `i2c_recover` |
| `tx0_len + tx1_len == 0`（写 0 字节） | TEI 收到地址 ACK 后直接 STOP |
| `rx_len == 1`（读 1 字节） | EEI 的 STARTF (Sr) 分支预先 `AckConfig(NACK)` |
| `reg_addr_len == 0`（无寄存器地址，直接读/写） | `tx0_len = 0`；写时跳过 `PHASE_TX_REG`，TEI 看到 `tx0_len==0 && tx1_len>0` 直接写 data；读时无 reg 阶段，但当前 EEPROM/BMI088 均传 1 或 2，**首版可仅支持 reg_addr_len ∈ {1, 2}** |
| 总线 hang（SDA 一直低） | 应用层捕获 `LL_ERR_TIMEOUT` 后调用 `i2c_recover(&g_hi2cX)` |

> EEPROM `ACK Polling`：`eeprom_wait_ready` 每 1 ms 调一次 `i2c_check_ack`，每次最多 100 ms 超时 + 自带 `vTaskDelay(1)` 让出 → 完全 RTOS 友好。

---

## 10. 测试验证计划

### 10.1 单元测试

| 测试 | 操作 | 预期 |
|------|------|------|
| TC-01 | `Eeprom_WriteByte(0, 0x55)` + `Eeprom_ReadByte(0, &v)` | `v==0x55` |
| TC-02 | 跨页写 64 字节（offset=0x10, len=64） | 全部一致 |
| TC-03 | 拔掉 EEPROM 跳线再 `Eeprom_Read` | 返回 `EEPROM_ERR_TIMEOUT`，任务不阻塞 |
| TC-04 | TC-03 后接 `Eeprom_Write` | 第一次失败 → `i2c_recover` → 重新插上跳线后恢复 |
| TC-05 | BMI088 chip-id 读取 | `0x1E`（acc）/ `0x0F`（gyro） |
| TC-06 | BMI088 持续 500 ms 周期读 1 小时 | 无丢包，无重启 |
| TC-07 | EEPROM 和 BMI088 任务并发跑 5 分钟 | 两条总线互不干扰，数据正确 |

### 10.2 调度延迟测试（量化中断化收益）

1. 增加一个 **高优先级空跑任务**（仅 `vTaskDelay(1)` + GPIO 翻转），用示波器或逻辑分析仪测 GPIO 周期抖动。
2. 同时跑 256 字节 EEPROM 读：
   - **轮询版**：高优先级任务 GPIO 周期会出现 ~25 ms 的"凹陷"
   - **中断版**：抖动应在 <100 µs 数量级

### 10.3 ISR 行为验证

- 用调试器把 4 个 ISR 函数加断点，单步走完一次 read，确认 phase 走完整：
  `WAIT_START → TX_ADDR_W → TX_REG → TX_LAST → WAIT_RESTART → TX_ADDR_R → RX_DATA → WAIT_STOP → IDLE`
- 改 `tx1_len = 0`、`tx0_len = 0`、`rx_len = 1` 等边界，逐一验证。

---

## 11. 风险与回退

| 风险 | 缓解措施 |
|------|----------|
| 状态机漏掉某条路径导致 hang | 信号量带超时；任务级 `i2c_recover` 兜底 |
| ISR 频繁打断高优先级任务 | I2C IRQ 抢占优先级设 5（≥ syscall 阈值 1，且低于 UART/CAN） |
| 两个 I2C 实例 ISR 共用 `s_irq_handle_*` 写一次后被覆盖 | 用两组独立 ISR 薄壳 + 两个独立 static 句柄指针（§6.4） |
| 静态信号量在多次 `i2cX_init()` 重入时被破坏 | `i2c_init` 内判断 `bus_mtx == NULL` 才创建 |
| BMI088 上电延时与中断时序耦合 | 上层 `DDL_DelayMS/vTaskDelay` 不变；I2C HAL 不引入额外延时 |
| 编译期 `INT_SRC_I2C1_*` 不可用（DDL 配置裁剪） | 在工程 `hc32f4xx_conf.h` 中确保 `LL_I2C_ENABLE` 与 `LL_INTERRUPTS_ENABLE` 均为 `DDL_ON` |

**回退**：保留旧 `i2c.c` 内容到 `i2c.c.poll.bak`，如发现严重问题，git revert + 重命名回滚即可（应用层无须改动）。

---

## 12. 实施步骤（Step-by-step，供 AI 工具执行）

> 严格按顺序操作。每一步完成后编译通过再进入下一步。

1. **备份**：复制现有 `communication/i2c/i2c.c` 为 `communication/i2c/i2c.c.poll.bak`。
2. **检查 DDL 配置**：打开 `hc32f4xx_conf.h`，确认 `LL_I2C_ENABLE = DDL_ON`、`LL_INTERRUPTS_ENABLE = DDL_ON`；否则改为 `DDL_ON`。
3. **改写头文件**：按 §5 替换 `i2c.h`（保留宏 / 函数原型签名）。
4. **重写实现**：按 §6 全量替换 `i2c.c`；`i2c_recover` 函数体保留原逻辑（仅末尾 `i2c_init` 调用沿用新版）。
5. **编译第一次**：解决所有 `INT_SRC_I2C*_xxx` / `INT01X_IRQn` / `INT_SRC_I2Cx_EEI` 名字相关的报错。
6. **静态检查**：跑 lint，重点关注：
   - ISR 内绝无 `xSemaphoreTake/Give`（非 FromISR）
   - 所有 `xSemaphoreCreate*Static` 的 buffer 已声明
   - `i2c_handle_t` 内 `volatile` 字段不被优化掉
7. **基础冒烟**：仅运行 EEPROM 任务，断点观察读写 4 字节。
8. **加 BMI088 任务**：观察两条总线并发。
9. **故障注入测试**：临时拔掉 EEPROM SDA 跳线，确认走 `LL_ERR_TIMEOUT` 路径。
10. **跑 TC-01 ~ TC-07** 完整测试集，固化日志。

---

## 13. 附：与官方示例的对应关系

| 官方示例 | 本方案 | 关键差异 |
|----------|--------|----------|
| `i2c_master_int/source/main.c::I2C_EEI_Callback` | `i2c_on_eei` | 增加 Sr 分支（`WAIT_RESTART → TX_ADDR_R`） |
| `i2c_master_int/source/main.c::I2C_TEI_Callback` | `i2c_on_tei` | 增加 `OP_READ` 末字节 → Sr，`OP_CHECK_ACK` 直接 STOP |
| `i2c_master_int/source/main.c::I2C_TXI_Callback` | `i2c_on_txi` | 使用双段 TX `(tx0, tx1)` 替代单一 `pBuf` |
| `i2c_master_int/source/main.c::I2C_RXI_Callback` | `i2c_on_rxi` | 逻辑一致 |
| `stcI2cCom.enComStatus = I2C_COM_IDLE` | `xSemaphoreGiveFromISR(done_sem)` | 任务阻塞唤醒替代 busy-wait |

---

完成本方案后，`eeprom.c` / `bmi088.c` 在传输 256 字节数据时的 CPU 占用应从「100% × 25 ms」降为「<1% × 中断瞬间」，其他任务调度延迟回落到 µs 量级，I2C 接口实时性彻底解决。
