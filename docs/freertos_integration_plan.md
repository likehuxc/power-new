# power-new FreeRTOS Integration Plan

## 目标

在 `power-new` 项目中集成 FreeRTOS，并参考 `C:\Users\huxiaocheng1\Desktop\work\d5w_pmu` 项目的 FreeRTOS 和 CAN 使用方式，将当前 `power-new` 中的 LED、UART、CAN 三个周期/事件处理逻辑分别改造成 FreeRTOS 任务。

本方案只描述需要执行的改动，不直接修改工程代码。

## 参考项目

- 当前项目：`C:\Users\huxiaocheng1\Desktop\work\power-new`
- 参考项目：`C:\Users\huxiaocheng1\Desktop\work\d5w_pmu`
- 下文简称：
  - `power-new`：目标工程
  - `d5`：参考工程

## 已确认需求

1. 在 `power-new` 中加入 FreeRTOS。
2. FreeRTOS 配置、目录结构、MDK 工程接入方式参考 `d5`。
3. `power-new` 中创建三个 FreeRTOS 任务：
   - LED 任务
   - UART 任务
   - CAN 任务
4. CAN 驱动配置改成 `d5` 的方式。
5. `power-new` 旧的 CAN 驱动已经删除：
   - `source/drv_can.c`
   - `source/drv_can.h`
6. `power-new` 当前保留的 CAN 文件来自 `d5`：
   - `source/can.c`
   - `source/can.h`
7. CAN 波特率使用 `CAN_BAUDRATE_1M`。
8. CAN 任务只需要打印收到的数据，不解析协议。
9. 除 FreeRTOS 接入、任务化、CAN 接收打印和必要工程配置外，其它功能保持不变。

## 当前 power-new 状态摘要

`power-new` 当前主流程为裸循环结构：

```c
int main(void)
{
    Board_PeriphUnlock();
    Board_Init();
    App_Init();

    for (;;) {
        App_Process();
    }
}
```

`App_Init()` 当前负责初始化：

```c
Led_Init();
Uart_Init();
Can_Init();
```

`App_Process()` 当前负责循环调用：

```c
Led_Task();
Uart_Task();
Can_Task();
```

需要将这个裸循环模型改为 FreeRTOS 多任务模型。

## d5 中需要参考的模式

### FreeRTOS 接入

`d5` 的 FreeRTOS 源码目录：

```text
components/freertos
```

MDK 工程中加入的核心文件包括：

```text
components/freertos/croutine.c
components/freertos/event_groups.c
components/freertos/list.c
components/freertos/queue.c
components/freertos/stream_buffer.c
components/freertos/tasks.c
components/freertos/timers.c
components/freertos/portable/GCC/ARM_CM4F/port.c
components/freertos/portable/MemMang/heap_4.c
```

MDK IncludePath 中加入：

```text
components/freertos/include
components/freertos/portable/GCC/ARM_CM4F
```

### d5 主流程

`d5` 中主流程是：

```c
drivers_init();
/* 各业务模块 init，内部创建任务 */
vTaskStartScheduler();
for (;;);
```

`power-new` 应参考这个方式：

1. 先完成板级和外设初始化。
2. 创建 LED、UART、CAN 任务。
3. 启动调度器。
4. 不再使用 `for (;;) App_Process();` 作为主循环。

### d5 CAN 接收模式

`d5` 的 CAN 接收基本数据流：

```text
CAN 中断
  -> can.c 内部 recv callback
  -> xQueueSendFromISR()
  -> CAN 处理任务 xQueueReceive()
  -> 打印/解析
```

`power-new` 只需要保留打印，不需要协议解析。

## 推荐实施方案

### 1. 拷贝 FreeRTOS 组件

从 `d5` 拷贝：

```text
C:\Users\huxiaocheng1\Desktop\work\d5w_pmu\components\freertos
```

到 `power-new`：

```text
C:\Users\huxiaocheng1\Desktop\work\power-new\components\freertos
```

如果 `power-new` 当前没有 `components` 目录，则新建：

```text
components/
```

### 2. 修改 MDK 工程

目标文件：

```text
C:\Users\huxiaocheng1\Desktop\work\power-new\MDK\power_new.uvprojx
```

需要完成：

1. 添加 FreeRTOS include 路径：

```text
..\components\freertos\include
..\components\freertos\portable\GCC\ARM_CM4F
```

2. 添加 FreeRTOS 源文件到工程：

```text
..\components\freertos\croutine.c
..\components\freertos\event_groups.c
..\components\freertos\list.c
..\components\freertos\queue.c
..\components\freertos\stream_buffer.c
..\components\freertos\tasks.c
..\components\freertos\timers.c
..\components\freertos\portable\GCC\ARM_CM4F\port.c
..\components\freertos\portable\MemMang\heap_4.c
```

3. 删除旧 CAN 文件引用：

```text
..\source\drv_can.c
..\source\drv_can.h
```

原因：这两个文件在 `power-new` 中已删除，但 MDK 工程里仍可能残留引用，会导致编译失败。

### 3. main.c 改造

目标文件：

```text
C:\Users\huxiaocheng1\Desktop\work\power-new\source\main.c
```

建议改造为：

```c
#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
    Board_PeriphUnlock();
    Board_Init();
    App_Init();
    App_StartTasks();

    vTaskStartScheduler();

    for (;;) {
    }
}
```

说明：

- `App_Init()` 继续负责外设初始化。
- 新增 `App_StartTasks()` 负责创建 LED、UART、CAN 三个任务。
- `App_Process()` 可以删除，或保留但不再由 `main()` 调用。

### 4. app.h / app.c 改造

目标文件：

```text
C:\Users\huxiaocheng1\Desktop\work\power-new\source\app.h
C:\Users\huxiaocheng1\Desktop\work\power-new\source\app.c
```

新增接口：

```c
void App_StartTasks(void);
```

任务创建建议：

```c
xTaskCreate(LedThread,  "led",  128, NULL, configMAX_PRIORITIES - 10, NULL);
xTaskCreate(UartThread, "uart", 256, NULL, configMAX_PRIORITIES - 9,  NULL);
xTaskCreate(CanThread,  "can",  256, NULL, configMAX_PRIORITIES - 8,  NULL);
```

优先级和栈大小可以根据实际编译和运行情况调整。建议初始值保持保守，不引入过大内存占用。

### 5. LED 任务设计

当前 `Led_Task()` 内部已经按 `Board_GetTick()` 做 1000 ms 周期判断。

FreeRTOS 下建议任务结构：

```c
static void LedThread(void *param)
{
    (void)param;

    for (;;) {
        Led_Task();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

说明：

- 保留现有 `Led_Task()` 逻辑。
- 用 `vTaskDelay()` 让出 CPU。
- 不建议直接把 LED 翻转逻辑搬进线程，避免扩大改动范围。

### 6. UART 任务设计

当前 `Uart_Task()` 从 ring buffer 读取 UART1 接收数据并回发。

FreeRTOS 下建议任务结构：

```c
static void UartThread(void *param)
{
    (void)param;

    for (;;) {
        Uart_Task();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

说明：

- 保留当前 UART DMA、ring buffer、回调逻辑。
- UART 任务仍采用短周期轮询方式消费 ring buffer。
- 不在本次改动中重构 UART 为 queue 或 semaphore 驱动，避免改变现有行为。

### 7. CAN 初始化与任务设计

`power-new` 当前 `source/can.c` 和 `source/can.h` 已来自 `d5`，继续使用。

建议新增应用层 CAN 封装，方式二选一：

#### 推荐方式：在 app.c 中实现 CAN 队列和任务

适合当前项目规模，文件改动少。

定义 CAN 消息结构：

```c
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  buf[8];
} app_can_msg_t;
```

定义队列：

```c
static QueueHandle_t s_can_rx_queue;
```

CAN 接收回调：

```c
static void CanRecvCallback(uint32_t id, uint8_t *buf, uint8_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    app_can_msg_t msg;
    uint8_t i;

    msg.id = id;
    msg.len = (len > 8U) ? 8U : len;

    for (i = 0U; i < msg.len; i++) {
        msg.buf[i] = buf[i];
    }
    for (; i < 8U; i++) {
        msg.buf[i] = 0U;
    }

    if (s_can_rx_queue != NULL) {
        (void)xQueueSendFromISR(s_can_rx_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

CAN 错误回调：

```c
static void CanErrorCallback(can_error_t err, const char *err_msg)
{
    printf("CAN_ERR:%d,%s\r\n", err, err_msg);
}
```

CAN 初始化：

```c
static void CanApp_Init(void)
{
    s_can_rx_queue = xQueueCreate(32, sizeof(app_can_msg_t));

    can_init(CAN_BAUDRATE_1M);
    can_set_recv_callback(CanRecvCallback);
    can_set_error_callback(CanErrorCallback);
}
```

CAN 任务：

```c
static void CanThread(void *param)
{
    app_can_msg_t msg;

    (void)param;

    for (;;) {
        if (xQueueReceive(s_can_rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            printf("CAN_RX ID:0x%08lx LEN:%u DATA:%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
                   msg.id,
                   msg.len,
                   msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3],
                   msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
        }
    }
}
```

注意：

- 打印函数如果项目中没有稳定 `printf()` 输出，应改用当前 UART 打印路径。
- 如果 `printf()` 已通过 UART 重定向，则可直接使用。
- CAN 接收任务只打印数据，不调用任何协议解析函数。

#### 备选方式：新增 can_app.c / can_app.h

如果希望职责更清晰，可以新增：

```text
source/can_app.c
source/can_app.h
```

将 CAN queue、callback、task 都放到 `can_app.c` 中，`app.c` 只调用：

```c
CanApp_Init();
CanApp_StartTask();
```

优点：模块边界更清晰。  
缺点：需要修改 MDK 工程新增两个文件。

如果其它 AI 工具执行实现，推荐优先采用此方式；如果希望最少改工程文件，则采用“在 app.c 中实现”的方式。

## FreeRTOS Hook 函数建议

参考 `d5`，建议在 `main.c` 中加入：

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    taskDISABLE_INTERRUPTS();
    printf("[STACK OVERFLOW] %s\r\n", pcTaskName);
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    printf("[MALLOC FAILED]\r\n");
    for (;;) {
    }
}
```

是否需要这些 hook 取决于 `FreeRTOSConfig.h` 中：

```c
configCHECK_FOR_STACK_OVERFLOW
configUSE_MALLOC_FAILED_HOOK
```

如果配置启用 hook，则必须提供对应函数。

## FreeRTOSConfig.h 注意事项

直接参考 `d5` 的：

```text
C:\Users\huxiaocheng1\Desktop\work\d5w_pmu\components\freertos\include\FreeRTOSConfig.h
```

迁移后需要检查：

1. CPU 时钟配置是否与 `power-new` 的系统时钟一致。
2. `configTOTAL_HEAP_SIZE` 是否足够三个任务和 CAN queue 使用。
3. `configMAX_PRIORITIES` 是否支持上述任务优先级。
4. Cortex-M4F port 是否匹配当前 HC32F460 工程。
5. SysTick 是否与现有 `Board_GetTick()` 存在冲突。

如果 `Board_GetTick()` 依赖 SysTick，而 FreeRTOS 也接管 SysTick，需要确认：

- 是否继续使用 `Board_GetTick()`。
- 或将 LED 周期完全改为 `vTaskDelay(pdMS_TO_TICKS(1000))`。

推荐初版保持 `Led_Task()` 不变，编译运行后再根据 SysTick 实际表现决定是否调整。

## 中断优先级注意事项

CAN 中断回调中如果调用 `xQueueSendFromISR()`，CAN 中断优先级必须满足 FreeRTOS 对 ISR API 的要求。

当前 `can.c` 中 CAN 中断优先级来自 d5：

```c
NVIC_SetPriority(irq.enIRQn, DDL_IRQ_PRIO_01);
```

需要结合 `FreeRTOSConfig.h` 中的中断优先级配置检查：

```c
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
configMAX_SYSCALL_INTERRUPT_PRIORITY
```

如果优先级不满足 FreeRTOS FromISR API 规则，运行时可能触发 assert 或产生不可预期行为。

如果不确定，建议参考 d5 保持一致，并确认 `FreeRTOSConfig.h` 也直接来自 d5。

## 需要避免的改动

1. 不引入 d5 的业务模块：
   - `power_manage`
   - `remote_ctrl`
   - `machine_info`
   - `hw_test`
   - `pudu_prot`
   - `fault_report`
2. 不解析 CAN 协议。
3. 不恢复旧的 `drv_can.c/.h`。
4. 不重构 UART DMA 驱动。
5. 不改变 LED、UART 原有业务行为。
6. 不把 d5 的 `drivers.c/drivers.h` 整体搬到 `power-new`，除非后续明确要求。

## 建议执行顺序

1. 备份或确认 git 工作区状态。
2. 拷贝 `components/freertos`。
3. 修改 MDK IncludePath。
4. 修改 MDK 文件列表：
   - 添加 FreeRTOS 文件。
   - 删除 `drv_can.c/.h` 引用。
5. 修改 `main.c`：
   - 引入 FreeRTOS。
   - 调用 `App_StartTasks()`。
   - 启动 `vTaskStartScheduler()`。
6. 修改 `app.h`：
   - 新增 `App_StartTasks()` 声明。
7. 修改 `app.c`：
   - 保留 `App_Init()`。
   - 新增 LED/UART/CAN 线程。
   - 新增 CAN queue 和回调。
8. 确认 `Can_Init()` 命名问题：
   - 当前 `source/can.c` 里提供的是 `can_init()`。
   - 如果 `app.c` 当前调用 `Can_Init()`，需要改为新的应用层初始化或直接调用 `can_init(CAN_BAUDRATE_1M)`。
9. 编译。
10. 根据编译错误补齐 include、工程文件列表或 hook 函数。
11. 上板验证：
   - LED 是否周期闪烁。
   - UART 是否仍能接收并回发。
   - CAN 收到数据后是否打印 ID、长度和数据。

## 验收标准

1. MDK 工程编译通过。
2. 不再引用已删除的 `drv_can.c/.h`。
3. 程序启动后 FreeRTOS 调度器正常运行。
4. LED 任务正常运行，LED 按预期闪烁。
5. UART 任务正常运行，原 UART 接收/回发逻辑保持可用。
6. CAN 使用 `CAN_BAUDRATE_1M` 初始化。
7. CAN 收到标准帧或扩展帧后，任务中打印：
   - CAN ID
   - DLC/长度
   - 最多 8 字节数据
8. CAN 接收逻辑不调用任何协议解析函数。
9. 除 FreeRTOS 接入和任务化外，其它业务行为保持不变。

## 风险点

1. `FreeRTOSConfig.h` 中系统时钟配置可能需要与 `power-new` 当前时钟确认。
2. FreeRTOS 接管 SysTick 后，`Board_GetTick()` 的行为需要验证。
3. CAN 中断优先级必须满足 FreeRTOS FromISR API 要求。
4. `printf()` 是否可在任务中稳定输出，需要结合当前 UART 重定向实现确认。
5. MDK 工程 XML 手工修改容易漏文件或路径错误，建议修改后打开 Keil 检查工程文件树。

## 推荐结论

采用“最小迁移 + 三任务改造”的方式：

- 只从 d5 引入 FreeRTOS 内核和 CAN 驱动使用方式。
- 不搬运 d5 的业务模块。
- `main()` 从裸循环改为 FreeRTOS 调度。
- LED、UART 保留原有逻辑，外面套任务。
- CAN 使用 d5 风格的中断回调 + queue + 任务打印。

这样能满足需求，同时最大限度降低对 `power-new` 现有 LED、UART、板级初始化逻辑的影响。
