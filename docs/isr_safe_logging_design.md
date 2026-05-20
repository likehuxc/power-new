# 中断安全日志设计（Uart_Printf 重构）

## 设计目标

让 `Uart_Printf` 在任意上下文（任务 / 中断 / 调度器启动前）都能安全调用,同时保持"USART1 上 DMA 帧不交错"这条单写者约束。

`source/uart.c` 当前实现使用 `xSemaphoreTake(s_printf_mutex, ...)` 进行互斥,但这套 API 在中断里非法。`source/can_port.c` 中的 `CanPort_ErrorCallback` 在中断上下文直接调用 `Uart_Printf`,只要 CAN 总线一报错,系统就可能 assert 或死锁。

## 非目标

- 不改 UART 接收路径。`Uart1_Task` / `Uart4_Task` 及其 stream buffer 保持不变。
- 不改 `drv_uart1_send` 行为。内部 `s_uart1_tx_busy` 自旋等待保留(理由见后文"为什么保留驱动里的 spin")。
- 不引入 DMA TX 完成信号量。先依赖现有 spin 充当串行化器;若实际负载证明不够,后续可加二值信号量,且不影响 `Uart_Printf` 的对外 API。

## 问题分析

### 三种调用上下文,三套不同规则

| 上下文 | 调度器状态 | 互斥锁可用? | StreamBuffer 可用? | 栈预算 |
|---|---|---|---|---|
| 调度器启动前(早期启动 / 故障钩子) | 未启动 | 否 | 否 | 紧张(~80 字节安全) |
| 任务 | 运行中 | 是 | 是 | 正常 |
| 中断 | 运行中,IRQ 内 | **否**(`xSemaphoreTake` 会 assert) | 是,需用 `*FromISR` 接口 | 紧张(用小栈缓冲) |

`Uart_Printf` 必须自动识别上下文并选择正确路径。

### 为什么用"环形缓冲 + 日志任务"

- 多个生产者(每种上下文一个)+ 单消费者(Log_Task)的结构,天然保证 DMA 帧不交错,发送侧不再需要任何互斥锁。
- FreeRTOS 的 `xStreamBufferSend` 与 `xStreamBufferSendFromISR` 都已支持,一个数据结构覆盖所有生产者。
- 日志任务把 `drv_uart1_send` 的 spin 开销集中吃掉,不影响业务任务和中断。

### 为什么保留驱动里的 spin

`drv_uart1_send` 当前会对 `s_uart1_tx_busy` 自旋等待,最多 5,000,000 次循环,然后返回 `LL_ERR_TIMEOUT`。如果删掉 spin,所有调用方(日志任务、UART1 回显任务、早期打印、栈溢出钩子)都得自己实现"等 TX 完成"逻辑。可行的两条路:

1. 每个调用方自己轮询 `tx_busy` —— 把同样的浪费从驱动搬到调用方,意义不大。
2. 在 DMA TC 中断里释放 TX 完成二值信号量 —— 干净,但与"暂不引入信号量"的决定冲突。

spin 的浪费仅在"有日志要发 + 上一帧 DMA 还没结束"的窗口出现,典型消息在 115200 波特率下持续约 7~14 ms,且发生在最低优先级的日志任务上,业务任务不受影响。

spin 唯一带来的硬约束:**中断里绝不能直接调用 `drv_uart1_send`**。本次设计正好满足这一点 —— 中断只把字节塞进 stream buffer,真正发送由日志任务做。

## 总体设计

```
                   ┌─────────────────────────────────────┐
  任务 ────────────►│  Uart_Printf(任务路径)              │
  (Uart_Printf)    │   - 取 s_log_writer_mutex           │
                   │   - vsnprintf 到 s_printf_buf       │
                   │   - xStreamBufferSend(0 超时)       │
                   │   - 失败则丢弃 + 计数               │──┐
                   │   - 释放 s_log_writer_mutex         │  │
                   └─────────────────────────────────────┘  │
                                                            │
                   ┌─────────────────────────────────────┐  │
  中断 ────────────►│  Uart_Printf(中断路径)              │  │
  (例:CAN 错误)    │   - vsnprintf 到 64B 栈缓冲         │  │
                   │   - xStreamBufferSendFromISR        │  │
                   │   - 失败则丢弃 + 计数               │──┤
                   └─────────────────────────────────────┘  │
                                                            ▼
                                              ┌─────────────────────┐
                                              │  s_log_stream       │
                                              │  StreamBuffer 512B  │
                                              │  trigger level = 1  │
                                              └─────────────────────┘
                                                            │
                                                            ▼
                                              ┌─────────────────────┐
                                              │  Log_Task           │
                                              │   循环:              │
                                              │    Receive(...)     │
                                              │    drv_uart1_send() │
                                              │    (spin 等 TC)     │
                                              └─────────────────────┘
                                                            │
                                                            ▼
                                                       USART1 TX (DMA)

  早期启动 ───────►  Uart_Printf(早期路径)
  (调度器未启动)     - vsnprintf 到 80B 栈缓冲
                     - 直接调用 drv_uart1_send(此时 spin 安全)
```

## 模块布局

改动局限于 `source/uart.c` 和 `source/uart.h`。驱动不动,`can_port.c` 不动(原有 `Uart_Printf` 调用点自动变为中断安全)。

### `source/uart.h`

对外 API 保持二进制兼容,仅新增一个任务入口:

```c
int32_t Uart_Init(void);
void    Uart1_Task(void *param);
void    Uart4_Task(void *param);
void    Log_Task(void *param);          /* 新增 —— 由 app.c 创建 */
void    Uart_Printf(const char *fmt, ...);
```

### `source/uart.c`

```c
/* 配置参数 */
#define LOG_STREAM_SIZE         512U
#define LOG_TRIGGER_LEVEL       1U
#define LOG_ISR_STACK_BUF_LEN   64U
#define LOG_TASK_BUF_LEN        128U
#define LOG_EARLY_STACK_BUF_LEN 80U

/* 新增全局变量(替代原来的 s_printf_mutex / s_printf_buf 用法) */
static StreamBufferHandle_t s_log_stream;
static SemaphoreHandle_t    s_log_writer_mutex;   /* 仅在任务路径保护 s_printf_buf */
static char                 s_printf_buf[160];
static volatile uint32_t    s_log_dropped_isr;
static volatile uint32_t    s_log_dropped_task;
```

`s_log_writer_mutex` 仅保护**共享格式化缓冲区**在多个任务之间的并发访问。任务在 `xStreamBufferSend` 返回后立即释放,日志任务再慢也不会反压生产者。

### 上下文识别

```c
static inline bool Uart_InIsr(void)
{
    return (xPortIsInsideInterrupt() == pdTRUE);
}

static inline bool Uart_SchedulerRunning(void)
{
    return (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
}
```

两个辅助函数都是 FreeRTOS 跨平台 API,在本工程使用的 Cortex-M3 port 上可用。

### `Uart_Printf` 分发逻辑

```c
void Uart_Printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    if (Uart_InIsr()) {
        Uart_PrintfFromIsr(fmt, ap);
    } else if (!Uart_SchedulerRunning() || s_log_stream == NULL) {
        Uart_PrintfEarly(fmt, ap);
    } else {
        Uart_PrintfFromTask(fmt, ap);
    }

    va_end(ap);
}
```

### 任务路径

```c
static void Uart_PrintfFromTask(const char *fmt, va_list ap)
{
    int    n;
    size_t sent;

    /* 20 ms 超时只是兜底,正常情况下 mutex 几乎无竞争 */
    if (xSemaphoreTake(s_log_writer_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        s_log_dropped_task++;
        return;
    }

    n = vsnprintf(s_printf_buf, sizeof(s_printf_buf), fmt, ap);
    if (n <= 0) {
        xSemaphoreGive(s_log_writer_mutex);
        return;
    }
    if (n >= (int)sizeof(s_printf_buf)) {
        n = (int)sizeof(s_printf_buf) - 1;
    }

    /* 0 超时:缓冲满则直接丢弃,不阻塞业务任务 */
    sent = xStreamBufferSend(s_log_stream, s_printf_buf, (size_t)n, 0U);
    if (sent != (size_t)n) {
        s_log_dropped_task++;
    }

    xSemaphoreGive(s_log_writer_mutex);
}
```

### 中断路径

```c
static void Uart_PrintfFromIsr(const char *fmt, va_list ap)
{
    char       buf[LOG_ISR_STACK_BUF_LEN];
    int        n;
    size_t     sent;
    BaseType_t hpw = pdFALSE;

    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) {
        n = (int)sizeof(buf) - 1;
    }

    sent = xStreamBufferSendFromISR(s_log_stream, buf, (size_t)n, &hpw);
    if (sent != (size_t)n) {
        s_log_dropped_isr++;
    }
    portYIELD_FROM_ISR(hpw);
}
```

`vsnprintf` 在中断里执行。对于低频错误事件(如 CAN 错误回调)开销可接受。如果后续 profile 发现影响中断响应,可以替换为定长格式的 ISR 专用辅助函数,调用点(`can_port.c`)不需要改。

### 早期路径

```c
static void Uart_PrintfEarly(const char *fmt, va_list ap)
{
    char buf[LOG_EARLY_STACK_BUF_LEN];
    int  n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) {
        n = (int)sizeof(buf) - 1;
    }
    /* 调度器还没起来,直接调驱动,spin 此时无害 */
    (void)drv_uart1_send((const uint8_t *)buf, (uint16_t)n);
}
```

与现有早期打印分支等价。

### Log_Task

```c
void Log_Task(void *param)
{
    uint8_t  buf[LOG_TASK_BUF_LEN];
    size_t   len;
    uint32_t last_dropped_isr  = 0;
    uint32_t last_dropped_task = 0;

    (void)param;

    for (;;) {
        /* 1 秒超时:既是空闲心跳,也方便调试观察 */
        len = xStreamBufferReceive(s_log_stream, buf,
                                   sizeof(buf), pdMS_TO_TICKS(1000));
        if (len > 0U) {
            (void)drv_uart1_send(buf, (uint16_t)len);
        }

        /* 周期性输出丢包统计,有变化才打印 */
        if ((s_log_dropped_isr  != last_dropped_isr) ||
            (s_log_dropped_task != last_dropped_task)) {
            char msg[64];
            int  n = snprintf(msg, sizeof(msg),
                              "\r\n[LOG] dropped isr=%lu task=%lu\r\n",
                              (unsigned long)s_log_dropped_isr,
                              (unsigned long)s_log_dropped_task);
            if (n > 0) {
                (void)drv_uart1_send((const uint8_t *)msg, (uint16_t)n);
            }
            last_dropped_isr  = s_log_dropped_isr;
            last_dropped_task = s_log_dropped_task;
        }
    }
}
```

1 秒超时既是丢包统计的输出周期,也保证空闲时任务能定期醒来,调试时方便观察状态。

### `Uart_Init` 调整

把现有 `s_printf_mutex` 的创建替换为:

```c
s_log_stream = xStreamBufferCreate(LOG_STREAM_SIZE, LOG_TRIGGER_LEVEL);
if (s_log_stream == NULL) return LL_ERR;

s_log_writer_mutex = xSemaphoreCreateMutex();
if (s_log_writer_mutex == NULL) return LL_ERR;
```

UART1/UART4 的接收 stream buffer 保持原状。

### `app.c`(调用方责任)

负责创建任务的代码,需要在创建 `Uart1_Task` 的位置同时创建 `Log_Task`,优先级低于业务任务、高于 idle。建议优先级 1,栈 256 字。最终值在实现阶段确定。

## 任务优先级与资源预算

| 资源 | 大小 | 说明 |
|---|---|---|
| `s_log_stream` | 512 字节 | StreamBuffer 数据区 |
| `s_printf_buf` | 160 字节 | 任务路径共享格式缓冲(沿用现有) |
| `s_log_writer_mutex` | 1 个互斥锁 | 仅任务路径用 |
| 中断栈缓冲 | 每次调用 64 字节 | 仅在 ISR 内部 `Uart_Printf` 期间占用 |
| 早期栈缓冲 | 每次调用 80 字节 | 仅在调度器启动前使用 |
| Log_Task 栈 | 256 字(待定) | 不用 vsnprintf,只用 snprintf 输出丢包行 |
| 丢包计数器 | 2 × `uint32_t` | 信息性 |

## 压力场景行为

| 场景 | 行为 |
|---|---|
| 空闲稳态 | Log_Task 阻塞在 stream buffer,CPU 占用 ~0 |
| 5~10 条突发日志 | Log_Task 按 DMA 速率排空缓冲,每帧约 7~14 ms;生产者发送无竞争 |
| CAN 错误风暴 | 中断路径推送短消息;缓冲满则丢弃并计数,系统稳定 |
| Log_Task 饿死(罕见) | 缓冲填满后直接丢弃,系统继续运行 |
| 启动早期故障打印 | 绕过 stream buffer,直接调驱动 spin |

## 验收检查

实现完成后,逐项验证:

1. `CanPort_ErrorCallback` 中调用 `Uart_Printf` 不再 assert,USART1 上能正常输出错误行。
2. 普通任务调用 `Uart_Printf` 输出的整行不会与 `CanPort_Task` 输出的 CAN_RX 行交错。
3. `vTaskStartScheduler` 之前的启动打印仍然能输出。
4. `main.c` 栈溢出钩子的 FATAL 行仍然能输出。
5. 强制高频日志循环时,会出现 `[LOG] dropped task=N` 行,且系统不崩溃、不卡死。
6. ISR 可达调用图(`CanPort_ErrorCallback` → `Uart_Printf` → ...)上不再出现 `xSemaphoreTake`。

## 落地步骤(实现阶段执行)

1. 修改 `source/uart.c`:
   - 新增全局变量(`s_log_stream`、`s_log_writer_mutex`、`s_log_dropped_*`)。
   - 移除原有 `s_printf_mutex`(`s_log_writer_mutex` 只是名字相近,职责不同)。
   - 新增三个私有辅助函数 `Uart_PrintfFromIsr` / `Uart_PrintfFromTask` / `Uart_PrintfEarly`。
   - 把 `Uart_Printf` 改成上下文分发器。
   - 新增 `Log_Task`。
   - 在 `Uart_Init` 里创建 stream buffer 和 writer mutex。
2. 在 `source/uart.h` 添加 `Log_Task` 声明。
3. 在 `app.c`(或现有任务创建处)创建 `Log_Task`,确认优先级低于 `CanPort_Task` 和 `Uart1_Task`。
4. 编译、烧录、按"验收检查"逐项验证。

`source/can_port.c`、`source/drv_uart1_dma.c`、`source/main.c` 不需要修改。
