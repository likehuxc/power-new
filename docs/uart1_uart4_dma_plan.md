# USART1 and USART4 DMA Driver Coding Plan

## Goal

Refactor `source/drv_uart_dma.c` and `source/drv_uart_dma.h` so the driver explicitly supports USART1 and USART4 with separate public APIs:

```c
typedef void (*drv_uart_recv_cb_t)(const uint8_t *buf, uint16_t len);

int drv_uart_init(uint32_t baudrate);

int drv_uart1_init(uint32_t baudrate);
int drv_uart1_send(const uint8_t *buf, uint16_t len);
void drv_uart1_set_recv_callback(drv_uart_recv_cb_t cb);

int drv_uart4_init(uint32_t baudrate);
int drv_uart4_send(const uint8_t *buf, uint16_t len);
void drv_uart4_set_recv_callback(drv_uart_recv_cb_t cb);
```

`drv_uart_init()` is a convenience API for the application layer. It should initialize both USART1 and USART4 by calling `drv_uart1_init()` and `drv_uart4_init()` internally.

Do not implement FreeRTOS queue logic in this change. The receive callbacks are only the driver-to-application handoff point.

## Design Direction

Use explicit USART1 and USART4 code blocks rather than a generic configuration-table framework. This project only needs two UART channels right now, and explicit functions are easier to review and debug.

Reuse small generic helpers only when the hardware operation is naturally shared, for example:

```c
static void UART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch);
static uint16_t UART_CalcTimeoutCompareValue(uint16_t timeout_bits, uint32_t clock_div);
```

Keep channel-specific logic separate:

```c
static int32_t UART1_DMA_Config(void);
static int32_t UART4_DMA_Config(void);
static void UART1_TMR0_Config(uint16_t timeout_bits);
static void UART4_TMR0_Config(uint16_t timeout_bits);
static void UART1_RX_DMA_TC_IrqCallback(void);
static void UART4_RX_DMA_TC_IrqCallback(void);
```

## USART1 Hardware Mapping

Keep the existing USART1 mapping unless board wiring changes.

| Item | USART1 |
|---|---|
| USART unit | `CM_USART1` |
| USART clock | `FCG1_PERIPH_USART1` |
| RX pin | `PA0` |
| RX function | `GPIO_FUNC_33` |
| TX pin | `PA2` |
| TX function | `GPIO_FUNC_32` |
| RX DMA | `CM_DMA1`, `DMA_CH0` |
| RX DMA trigger select | `AOS_DMA1_0` |
| RX DMA event | `EVT_SRC_USART1_RI` |
| RX DMA TC flag | `DMA_FLAG_TC_CH0` |
| RX DMA TC interrupt | `DMA_INT_TC_CH0` |
| RX DMA IRQn | existing `INT000_IRQn`, or another free IRQn |
| RX DMA INT source | `INT_SRC_DMA1_TC0` |
| TX DMA | `CM_DMA2`, `DMA_CH0` |
| TX DMA trigger select | `AOS_DMA2_0` |
| TX DMA event | `EVT_SRC_USART1_TI` |
| TX DMA TC flag | `DMA_FLAG_TC_CH0` |
| TX DMA TC interrupt | `DMA_INT_TC_CH0` |
| TX DMA IRQn | existing `INT001_IRQn`, or another free IRQn |
| TX DMA INT source | `INT_SRC_DMA2_TC0` |
| Timeout timer | `CM_TMR0_1`, `TMR0_CH_A` |
| Timer clock | `FCG2_PERIPH_TMR0_1` |
| USART TX complete source | `INT_SRC_USART1_TCI` |
| USART RX error source | `INT_SRC_USART1_EI` |
| USART RX timeout source | `INT_SRC_USART1_RTO` |

## USART4 Hardware Mapping

Implement USART4 with the following mapping.

| Item | USART4 |
|---|---|
| USART unit | `CM_USART4` |
| USART clock | `FCG1_PERIPH_USART4` |
| RX pin | `PD8` |
| RX function | `GPIO_FUNC_37` |
| TX pin | `PD9` |
| TX function | `GPIO_FUNC_36` |
| RX DMA | `CM_DMA2`, `DMA_CH1` |
| RX DMA trigger select | `AOS_DMA2_1` |
| RX DMA event | `EVT_SRC_USART4_RI` |
| RX DMA TC flag | `DMA_FLAG_TC_CH1` |
| RX DMA TC interrupt | `DMA_INT_TC_CH1` |
| RX DMA IRQn | choose a free IRQn, for example `INT005_IRQn` if unused |
| RX DMA INT source | `INT_SRC_DMA2_TC1` |
| TX DMA | `CM_DMA1`, `DMA_CH1` |
| TX DMA trigger select | `AOS_DMA1_1` |
| TX DMA event | `EVT_SRC_USART4_TI` |
| TX DMA TC flag | `DMA_FLAG_TC_CH1` |
| TX DMA TC interrupt | `DMA_INT_TC_CH1` |
| TX DMA IRQn | choose a free IRQn, for example `INT006_IRQn` if unused |
| TX DMA INT source | `INT_SRC_DMA1_TC1` |
| Timeout timer | `CM_TMR0_2`, `TMR0_CH_B` |
| Timer clock | `FCG2_PERIPH_TMR0_2` |
| USART TX complete source | `INT_SRC_USART4_TCI` |
| USART RX error source | `INT_SRC_USART4_EI` |
| USART RX timeout source | `INT_SRC_USART4_RTO` |

Notes:

- `EVT_SRC_USART4_RI/TI/TCI/RTO/EI` are already defined in `drivers/cmsis/Device/HDSC/hc32f4xx/Include/hc32f460.h`.
- `INT_SRC_DMA1_TC1` and `INT_SRC_DMA2_TC1` are also defined there.
- `GPIO_FUNC_36` and `GPIO_FUNC_37` are defined in `drivers/hc32_ll_driver/inc/hc32_ll_gpio.h`.
- Confirm `AOS_DMA1_1` and `AOS_DMA2_1` names in `hc32_ll_aos.h` before coding. If names differ, use the actual DMA channel trigger selectors provided by the SDK.

## File Changes

### 1. `source/drv_uart_dma.h`

Replace the current single-channel API with the explicit two-channel API.

Expected public interface:

```c
#ifndef DRV_UART_DMA_H__
#define DRV_UART_DMA_H__

#include <stdint.h>

#define DRV_UART_DMA_FRAME_LEN_MAX      500U
#define DRV_UART_DMA_TX_BUF_LEN_MAX     128U

typedef void (*drv_uart_recv_cb_t)(const uint8_t *buf, uint16_t len);

int drv_uart_init(uint32_t baudrate);

int drv_uart1_init(uint32_t baudrate);
int drv_uart1_send(const uint8_t *buf, uint16_t len);
void drv_uart1_set_recv_callback(drv_uart_recv_cb_t cb);

int drv_uart4_init(uint32_t baudrate);
int drv_uart4_send(const uint8_t *buf, uint16_t len);
void drv_uart4_set_recv_callback(drv_uart_recv_cb_t cb);

#endif
```

### 2. `source/drv_uart.h`

Keep this file as a simple wrapper:

```c
#ifndef DRV_UART_H__
#define DRV_UART_H__

#include "drv_uart_dma.h"

#endif
```

### 3. `source/drv_uart_dma.c`

Split global state into USART1 and USART4 state variables.

Recommended state naming:

```c
static __IO en_flag_status_t s_uart1_rx_frame_end;
static __IO en_flag_status_t s_uart1_tx_busy;
static __IO uint16_t         s_uart1_rx_len;
static uint8_t               s_uart1_rx_buf[DRV_UART_DMA_FRAME_LEN_MAX];
static uint8_t               s_uart1_tx_buf[DRV_UART_DMA_TX_BUF_LEN_MAX];
static drv_uart_recv_cb_t    s_uart1_recv_cb;

static __IO en_flag_status_t s_uart4_rx_frame_end;
static __IO en_flag_status_t s_uart4_tx_busy;
static __IO uint16_t         s_uart4_rx_len;
static uint8_t               s_uart4_rx_buf[DRV_UART_DMA_FRAME_LEN_MAX];
static uint8_t               s_uart4_tx_buf[DRV_UART_DMA_TX_BUF_LEN_MAX];
static drv_uart_recv_cb_t    s_uart4_recv_cb;
```

Do not share `tx_busy`, `rx_len`, or RX/TX buffers between USART1 and USART4.

## Macro Layout

Keep macros explicit and grouped by channel.

### USART1 macros

Rename existing generic macros to `UART1_` prefixes:

```c
#define UART1_UNIT                      (CM_USART1)
#define UART1_FCG_ENABLE()              (FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE))

#define UART1_RX_PORT                   (GPIO_PORT_A)
#define UART1_RX_PIN                    (GPIO_PIN_00)
#define UART1_RX_GPIO_FUNC              (GPIO_FUNC_33)
#define UART1_TX_PORT                   (GPIO_PORT_A)
#define UART1_TX_PIN                    (GPIO_PIN_02)
#define UART1_TX_GPIO_FUNC              (GPIO_FUNC_32)

#define UART1_RX_DMA_UNIT               (CM_DMA1)
#define UART1_RX_DMA_CH                 (DMA_CH0)
#define UART1_RX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define UART1_RX_DMA_TRIG_SEL           (AOS_DMA1_0)
#define UART1_RX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART1_RI)
#define UART1_RX_DMA_TC_INT             (DMA_INT_TC_CH0)
#define UART1_RX_DMA_TC_FLAG            (DMA_FLAG_TC_CH0)
#define UART1_RX_DMA_TC_IRQn            (INT000_IRQn)
#define UART1_RX_DMA_TC_INT_SRC         (INT_SRC_DMA1_TC0)

#define UART1_TX_DMA_UNIT               (CM_DMA2)
#define UART1_TX_DMA_CH                 (DMA_CH0)
#define UART1_TX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define UART1_TX_DMA_TRIG_SEL           (AOS_DMA2_0)
#define UART1_TX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART1_TI)
#define UART1_TX_DMA_TC_INT             (DMA_INT_TC_CH0)
#define UART1_TX_DMA_TC_FLAG            (DMA_FLAG_TC_CH0)
#define UART1_TX_DMA_TC_IRQn            (INT001_IRQn)
#define UART1_TX_DMA_TC_INT_SRC         (INT_SRC_DMA2_TC0)

#define UART1_TMR0_UNIT                 (CM_TMR0_1)
#define UART1_TMR0_CH                   (TMR0_CH_A)
#define UART1_TMR0_FCG_ENABLE()         (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE))

#define UART1_TX_CPLT_IRQn              (INT002_IRQn)
#define UART1_TX_CPLT_INT_SRC           (INT_SRC_USART1_TCI)
#define UART1_RX_ERR_IRQn               (INT003_IRQn)
#define UART1_RX_ERR_INT_SRC            (INT_SRC_USART1_EI)
#define UART1_RX_TIMEOUT_IRQn           (INT004_IRQn)
#define UART1_RX_TIMEOUT_INT_SRC        (INT_SRC_USART1_RTO)
```

### USART4 macros

Add USART4 macros:

```c
#define UART4_UNIT                      (CM_USART4)
#define UART4_FCG_ENABLE()              (FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART4, ENABLE))

#define UART4_RX_PORT                   (GPIO_PORT_D)
#define UART4_RX_PIN                    (GPIO_PIN_08)
#define UART4_RX_GPIO_FUNC              (GPIO_FUNC_37)
#define UART4_TX_PORT                   (GPIO_PORT_D)
#define UART4_TX_PIN                    (GPIO_PIN_09)
#define UART4_TX_GPIO_FUNC              (GPIO_FUNC_36)

#define UART4_RX_DMA_UNIT               (CM_DMA2)
#define UART4_RX_DMA_CH                 (DMA_CH1)
#define UART4_RX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define UART4_RX_DMA_TRIG_SEL           (AOS_DMA2_1)
#define UART4_RX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART4_RI)
#define UART4_RX_DMA_TC_INT             (DMA_INT_TC_CH1)
#define UART4_RX_DMA_TC_FLAG            (DMA_FLAG_TC_CH1)
#define UART4_RX_DMA_TC_IRQn            (INT005_IRQn)
#define UART4_RX_DMA_TC_INT_SRC         (INT_SRC_DMA2_TC1)

#define UART4_TX_DMA_UNIT               (CM_DMA1)
#define UART4_TX_DMA_CH                 (DMA_CH1)
#define UART4_TX_DMA_FCG_ENABLE()       (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define UART4_TX_DMA_TRIG_SEL           (AOS_DMA1_1)
#define UART4_TX_DMA_TRIG_EVT_SRC       (EVT_SRC_USART4_TI)
#define UART4_TX_DMA_TC_INT             (DMA_INT_TC_CH1)
#define UART4_TX_DMA_TC_FLAG            (DMA_FLAG_TC_CH1)
#define UART4_TX_DMA_TC_IRQn            (INT006_IRQn)
#define UART4_TX_DMA_TC_INT_SRC         (INT_SRC_DMA1_TC1)

#define UART4_TMR0_UNIT                 (CM_TMR0_2)
#define UART4_TMR0_CH                   (TMR0_CH_B)
#define UART4_TMR0_FCG_ENABLE()         (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_2, ENABLE))

#define UART4_TX_CPLT_IRQn              (INT007_IRQn)
#define UART4_TX_CPLT_INT_SRC           (INT_SRC_USART4_TCI)
#define UART4_RX_ERR_IRQn               (INT008_IRQn)
#define UART4_RX_ERR_INT_SRC            (INT_SRC_USART4_EI)
#define UART4_RX_TIMEOUT_IRQn           (INT009_IRQn)
#define UART4_RX_TIMEOUT_INT_SRC        (INT_SRC_USART4_RTO)
```

Before final coding, verify `INT005_IRQn` to `INT009_IRQn` are not already used elsewhere in the project. If any are used, pick other free shared interrupt numbers.

## Function Implementation Plan

### Step 1: Update public API

Remove:

```c
int drv_uart_send(const uint8_t *buf, uint16_t len);
void drv_uart_set_recv_callback(drv_uart_recv_cb_t cb);
```

Add:

```c
int drv_uart1_send(const uint8_t *buf, uint16_t len);
void drv_uart1_set_recv_callback(drv_uart_recv_cb_t cb);
int drv_uart4_send(const uint8_t *buf, uint16_t len);
void drv_uart4_set_recv_callback(drv_uart_recv_cb_t cb);
```

Keep:

```c
int drv_uart_init(uint32_t baudrate);
```

But make it initialize both UARTs:

```c
int drv_uart_init(uint32_t baudrate)
{
    int ret;

    ret = drv_uart1_init(baudrate);
    if (LL_OK != ret) {
        return ret;
    }

    return drv_uart4_init(baudrate);
}
```

### Step 2: Split send functions

Use the current `drv_uart_send()` body as the USART1 implementation:

```c
int drv_uart1_send(const uint8_t *buf, uint16_t len)
```

Then duplicate and adjust for USART4:

```c
int drv_uart4_send(const uint8_t *buf, uint16_t len)
```

USART4 send must use:

- `s_uart4_tx_busy`
- `UART4_TX_DMA_UNIT`
- `UART4_TX_DMA_CH`
- `UART4_TX_DMA_TC_FLAG`
- `UART4_UNIT`

USART1 send must use:

- `s_uart1_tx_busy`
- `UART1_TX_DMA_UNIT`
- `UART1_TX_DMA_CH`
- `UART1_TX_DMA_TC_FLAG`
- `UART1_UNIT`

### Step 3: Split callback setters

Implement:

```c
void drv_uart1_set_recv_callback(drv_uart_recv_cb_t cb)
{
    s_uart1_recv_cb = cb;
}

void drv_uart4_set_recv_callback(drv_uart_recv_cb_t cb)
{
    s_uart4_recv_cb = cb;
}
```

### Step 4: Split notify helpers

Use explicit notify functions:

```c
static void UART1_NotifyRecv(const uint8_t *buf, uint16_t len)
{
    if ((NULL != s_uart1_recv_cb) && (NULL != buf) && (0U != len)) {
        s_uart1_recv_cb(buf, len);
    }
}

static void UART4_NotifyRecv(const uint8_t *buf, uint16_t len)
{
    if ((NULL != s_uart4_recv_cb) && (NULL != buf) && (0U != len)) {
        s_uart4_recv_cb(buf, len);
    }
}
```

### Step 5: Split DMA configuration

Create:

```c
static int32_t UART1_DMA_Config(void);
static int32_t UART4_DMA_Config(void);
```

`UART1_DMA_Config()` should be the current `DMA_Config()` with macro names changed from generic to `UART1_`.

`UART4_DMA_Config()` should be copied from USART1 and changed to:

- RX source address: `&UART4_UNIT->RDR`
- RX destination buffer: `s_uart4_rx_buf`
- RX DMA: `DMA2 CH1`
- RX DMA trigger: `EVT_SRC_USART4_RI`
- RX DMA TC interrupt source: `INT_SRC_DMA2_TC1`
- TX destination address: `&UART4_UNIT->TDR`
- TX placeholder source buffer: `s_uart4_tx_buf`
- TX DMA: `DMA1 CH1`
- TX DMA trigger: `EVT_SRC_USART4_TI`
- TX DMA TC interrupt source: `INT_SRC_DMA1_TC1`

Important: if the existing LLP/AOS software-trigger reconfiguration is kept, do not share one static `stc_dma_llp_descriptor_t` between UART1 RX and UART4 RX. Use one descriptor per RX DMA channel:

```c
static stc_dma_llp_descriptor_t s_uart1_rx_llp_desc;
static stc_dma_llp_descriptor_t s_uart4_rx_llp_desc;
```

### Step 6: Split TMR0 configuration

Create:

```c
static void UART1_TMR0_Config(uint16_t timeout_bits);
static void UART4_TMR0_Config(uint16_t timeout_bits);
```

USART1:

```c
UART1_TMR0_FCG_ENABLE();
TMR0_Init(UART1_TMR0_UNIT, UART1_TMR0_CH, &stcTmr0Init);
TMR0_HWStartCondCmd(UART1_TMR0_UNIT, UART1_TMR0_CH, ENABLE);
TMR0_HWClearCondCmd(UART1_TMR0_UNIT, UART1_TMR0_CH, ENABLE);
```

USART4:

```c
UART4_TMR0_FCG_ENABLE();
TMR0_Init(UART4_TMR0_UNIT, UART4_TMR0_CH, &stcTmr0Init);
TMR0_HWStartCondCmd(UART4_TMR0_UNIT, UART4_TMR0_CH, ENABLE);
TMR0_HWClearCondCmd(UART4_TMR0_UNIT, UART4_TMR0_CH, ENABLE);
```

Keep the same compare-value calculation. According to the timer mapping image:

- USART1 uses Timer0 Unit1 A channel
- USART4 uses Timer0 Unit2 B channel

Rename:

```c
static void USART_StopTimeoutTimer(...)
```

to:

```c
static void UART_StopTimeoutTimer(...)
```

Use it for both:

```c
UART_StopTimeoutTimer(UART1_TMR0_UNIT, UART1_TMR0_CH);
UART_StopTimeoutTimer(UART4_TMR0_UNIT, UART4_TMR0_CH);
```

### Step 7: Split interrupt callbacks

Create USART1 callbacks:

```c
static void UART1_RX_DMA_TC_IrqCallback(void);
static void UART1_TX_DMA_TC_IrqCallback(void);
static void UART1_RxTimeout_IrqCallback(void);
static void UART1_TxComplete_IrqCallback(void);
static void UART1_RxError_IrqCallback(void);
```

Create USART4 callbacks:

```c
static void UART4_RX_DMA_TC_IrqCallback(void);
static void UART4_TX_DMA_TC_IrqCallback(void);
static void UART4_RxTimeout_IrqCallback(void);
static void UART4_TxComplete_IrqCallback(void);
static void UART4_RxError_IrqCallback(void);
```

USART4 RX timeout callback must use:

```c
s_uart4_rx_frame_end = SET;
s_uart4_rx_len = DRV_UART_DMA_FRAME_LEN_MAX -
                 (uint16_t)DMA_GetTransCount(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH);
UART4_NotifyRecv(s_uart4_rx_buf, s_uart4_rx_len);
UART_StopTimeoutTimer(UART4_TMR0_UNIT, UART4_TMR0_CH);
USART_ClearStatus(UART4_UNIT, USART_FLAG_RX_TIMEOUT);
```

USART4 RX error callback must use:

```c
(void)USART_ReadData(UART4_UNIT);
USART_ClearStatus(UART4_UNIT,
                  USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN);
```

USART4 TX complete callback must clear:

```c
USART_FuncCmd(UART4_UNIT, USART_TX | USART_INT_TX_CPLT, DISABLE);
s_uart4_tx_busy = RESET;
```

### Step 8: Split init functions

Create:

```c
int drv_uart1_init(uint32_t baudrate);
int drv_uart4_init(uint32_t baudrate);
```

USART1 init flow:

1. Validate `baudrate != 0`.
2. Call `UART1_DMA_Config()`.
3. Call `UART1_TMR0_Config(USART_TIMEOUT_BITS)`.
4. Configure PA0/PA2 pin functions.
5. Enable USART1 clock.
6. Initialize USART1 with `USART_UART_Init(UART1_UNIT, ...)`.
7. Register USART1 TX complete, RX error, RX timeout interrupts.
8. Stop/clear timeout timer.
9. Enable USART RX, RX interrupt, RX timeout, RX timeout interrupt.

USART4 init flow:

1. Validate `baudrate != 0`.
2. Call `UART4_DMA_Config()`.
3. Call `UART4_TMR0_Config(USART_TIMEOUT_BITS)`.
4. Configure PD8/PD9 pin functions.
5. Enable USART4 clock.
6. Initialize USART4 with `USART_UART_Init(UART4_UNIT, ...)`.
7. Register USART4 TX complete, RX error, RX timeout interrupts.
8. Stop/clear timeout timer.
9. Enable USART RX, RX interrupt, RX timeout, RX timeout interrupt.

`drv_uart_init()` should call both in order:

```c
int drv_uart_init(uint32_t baudrate)
{
    int ret;

    ret = drv_uart1_init(baudrate);
    if (LL_OK != ret) {
        return ret;
    }

    return drv_uart4_init(baudrate);
}
```

## AOS / LLP Reconfiguration Warning

The current USART1 code uses `AOS_SW_Trigger()` to reload RX DMA through LLP. With two UARTs, this can become risky if both channels use the same global software-trigger reconfiguration event.

Preferred safer implementation:

In each RX timeout callback, explicitly restart only the current UART's RX DMA channel:

1. Disable RX DMA channel.
2. Clear RX DMA transfer-complete flag.
3. Reset RX DMA destination address to that UART's RX buffer.
4. Reset RX DMA transfer count to `DRV_UART_DMA_FRAME_LEN_MAX`.
5. Reset block size to `1U`.
6. Enable RX DMA channel.

This avoids one UART timeout accidentally affecting the other UART DMA channel through shared AOS software trigger behavior.

If the implementer keeps LLP + `AOS_SW_Trigger()`, they must verify on hardware that USART1 and USART4 can receive back-to-back frames without corrupting each other's DMA reload state.

## Application Layer Use

`source/uart.c` can stay simple:

```c
int32_t Uart_Init(void)
{
    return drv_uart_init(115200UL);
}
```

If the application needs separate callbacks:

```c
static void Uart1RecvCallback(const uint8_t *buf, uint16_t len)
{
    /* Put into USART1 FreeRTOS queue later. */
}

static void Uart4RecvCallback(const uint8_t *buf, uint16_t len)
{
    /* Put into USART4 FreeRTOS queue later. */
}

void Uart_RegisterCallbacks(void)
{
    drv_uart1_set_recv_callback(Uart1RecvCallback);
    drv_uart4_set_recv_callback(Uart4RecvCallback);
}
```

Do not add polling read functions. RX data should enter application processing through callbacks first, then later through FreeRTOS queues.

## Validation Checklist

After implementation, verify:

- `rg "drv_uart_send\\(" source` finds no ambiguous old single-channel send API.
- `rg "DrvUartDma_|ReadFrame" source` finds no old driver API.
- USART1 still transmits through DMA2 CH0 and receives through DMA1 CH0.
- USART4 transmits through DMA1 CH1 and receives through DMA2 CH1.
- USART4 PD8 is configured as `GPIO_FUNC_37`.
- USART4 PD9 is configured as `GPIO_FUNC_36`.
- USART1 timeout uses `CM_TMR0_1`, `TMR0_CH_A`.
- USART4 timeout uses `CM_TMR0_2`, `TMR0_CH_B`.
- USART1 and USART4 have separate RX buffers, TX buffers, busy flags, length variables, and callbacks.
- USART1 and USART4 can transmit independently.
- USART1 and USART4 receive callbacks are triggered independently.
- No shared static LLP descriptor is used by both RX DMA channels.
- If using software-trigger DMA reload, back-to-back receive tests pass on both UARTs.

## Suggested Manual Hardware Tests

1. Build the Keil project.
2. Initialize both UARTs with `drv_uart_init(115200UL)`.
3. Send test bytes through USART1 and confirm only USART1 callback fires.
4. Send test bytes through USART4 and confirm only USART4 callback fires.
5. Call `drv_uart1_send()` and verify data exits PA2.
6. Call `drv_uart4_send()` and verify data exits PD9.
7. Send long frames close to `DRV_UART_DMA_FRAME_LEN_MAX` on both UARTs.
8. Send short frames separated by timeout gaps on both UARTs.
9. Send data to USART1 and USART4 nearly simultaneously and check for DMA reload issues.

