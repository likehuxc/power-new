/**
 *******************************************************************************
 * @file  i2c.c
 * @brief 硬件 I2C 主机驱动（轮询模式，含总线恢复）
 *        支持多实例，通过 i2c_handle_t 句柄区分不同 I2C 外设
 *******************************************************************************
 */

#include "i2c.h"
#include "hc32_ll.h"

/*******************************************************************************
 * 总线恢复参数
 ******************************************************************************/
#define BUS_RECOVERY_DELAY_US   (5UL)
#define BUS_RECOVERY_TIMEOUT_US (500UL)
#define BUS_RECOVERY_CLK_NUM    (9U)

/*******************************************************************************
 * I2C1 预定义实例（保持向后兼容）
 ******************************************************************************/
static const i2c_config_t s_i2c1_cfg = {
    .unit     = CM_I2C1,
    .fcg      = FCG1_PERIPH_I2C1,
    .scl_port = GPIO_PORT_A,
    .scl_pin  = GPIO_PIN_11,
    .sda_port = GPIO_PORT_A,
    .sda_pin  = GPIO_PIN_10,
    .scl_func = GPIO_FUNC_49,
    .sda_func = GPIO_FUNC_48,
    .baudrate = 100000UL,
    .clk_div  = I2C_CLK_DIV2,
    .scl_time = 3UL,
    .timeout  = 0x40000UL,
};

i2c_handle_t g_hi2c1;

int32_t i2c1_init(void)
{
    return i2c_init(&g_hi2c1, &s_i2c1_cfg);
}

/*******************************************************************************
 * I2C2 预定义实例（BMI088 等）
 ******************************************************************************/
static const i2c_config_t s_i2c2_cfg = {
    .unit     = CM_I2C2,
    .fcg      = FCG1_PERIPH_I2C2,
    .scl_port = GPIO_PORT_A,
    .scl_pin  = GPIO_PIN_09,
    .sda_port = GPIO_PORT_A,
    .sda_pin  = GPIO_PIN_08,
    .scl_func = GPIO_FUNC_51,
    .sda_func = GPIO_FUNC_50,
    .baudrate = 100000UL,
    .clk_div  = I2C_CLK_DIV2,
    .scl_time = 3UL,
    .timeout  = 0x40000UL,
};

i2c_handle_t g_hi2c2;

int32_t i2c2_init(void)
{
    return i2c_init(&g_hi2c2, &s_i2c2_cfg);
}

/*******************************************************************************
 * 内部函数
 ******************************************************************************/
static int32_t wait_scl_high(const i2c_config_t *cfg, uint32_t timeout_us)
{
    while (PIN_RESET == GPIO_ReadInputPins(cfg->scl_port, cfg->scl_pin)) {
        if (0UL == timeout_us) {
            return LL_ERR_TIMEOUT;
        }
        timeout_us--;
        DDL_DelayUS(1UL);
    }
    return LL_OK;
}

static void gpio_stop(const i2c_config_t *cfg)
{
    /* 1. 设置 SCL/SDA 为低电平 */
    GPIO_ResetPins(cfg->scl_port, cfg->scl_pin);
    DDL_DelayUS(BUS_RECOVERY_DELAY_US);
    GPIO_ResetPins(cfg->sda_port, cfg->sda_pin);
    DDL_DelayUS(BUS_RECOVERY_DELAY_US);
    GPIO_SetPins(cfg->scl_port, cfg->scl_pin);
    /* 2. 等待 SCL 变为高电平 */
    (void)wait_scl_high(cfg, BUS_RECOVERY_TIMEOUT_US);
    /* 3. 设置 SDA 为高电平 */
    DDL_DelayUS(BUS_RECOVERY_DELAY_US);
    GPIO_SetPins(cfg->sda_port, cfg->sda_pin);
    /* 4. 延时 */
    DDL_DelayUS(BUS_RECOVERY_DELAY_US);
}

/*******************************************************************************
 * 公共接口
 ******************************************************************************/

/**
 * @brief  初始化 I2C 实例
 * @param  hi2c  句柄指针
 * @param  cfg   硬件配置（建议定义为 const 全局变量）
 * @retval LL_OK / LL_ERR
 */
int32_t i2c_init(i2c_handle_t *hi2c, const i2c_config_t *cfg)
{
    stc_gpio_init_t stcGpio;
    stc_i2c_init_t stcI2c;
    float32_t fErr;
    int32_t ret;

    hi2c->cfg = cfg;

    (void)GPIO_StructInit(&stcGpio);
    (void)GPIO_Init(cfg->scl_port, cfg->scl_pin, &stcGpio);
    (void)GPIO_Init(cfg->sda_port, cfg->sda_pin, &stcGpio);
    GPIO_SetFunc(cfg->scl_port, cfg->scl_pin, cfg->scl_func);
    GPIO_SetFunc(cfg->sda_port, cfg->sda_pin, cfg->sda_func);

    FCG_Fcg1PeriphClockCmd(cfg->fcg, ENABLE);

    (void)I2C_DeInit(cfg->unit);
    (void)I2C_StructInit(&stcI2c);
    stcI2c.u32ClockDiv = cfg->clk_div;
    stcI2c.u32Baudrate = cfg->baudrate;
    stcI2c.u32SclTime  = cfg->scl_time;
    ret = I2C_Init(cfg->unit, &stcI2c, &fErr);
    if (ret == LL_OK) {
        I2C_BusWaitCmd(cfg->unit, ENABLE);
        I2C_Cmd(cfg->unit, ENABLE);
    }
    return ret;
}

/**
 * @brief  向从机写数据
 * @param  hi2c        I2C 句柄
 * @param  dev_addr    7 位从机地址
 * @param  reg_addr    寄存器/内存地址
 * @param  reg_addr_len 地址字节数（1 或 2）
 * @param  buf         数据缓冲区
 * @param  len         字节数
 * @retval LL_OK / LL_ERR / LL_ERR_TIMEOUT
 */
int32_t i2c_write(i2c_handle_t *hi2c, uint16_t dev_addr, uint16_t reg_addr,
                  uint8_t reg_addr_len, const uint8_t *buf, uint32_t len)
{
    const i2c_config_t *cfg = hi2c->cfg;
    int32_t ret;
    uint8_t addr_buf[2];

    /* 1. 复位 I2C 外设并重新使能 */
    I2C_SWResetCmd(cfg->unit, ENABLE);
    I2C_SWResetCmd(cfg->unit, DISABLE);
    I2C_BusWaitCmd(cfg->unit, ENABLE);
    I2C_Cmd(cfg->unit, ENABLE);

    /* 2. 发送起始信号 */
    ret = I2C_Start(cfg->unit, cfg->timeout);
    if (ret == LL_OK) {
        /* 3. 发送从机地址 */
        ret = I2C_TransAddr(cfg->unit, dev_addr, I2C_DIR_TX, cfg->timeout);
        if (ret == LL_OK) {
            /* 4. 发送寄存器/内存地址 */
            if (reg_addr_len == 2U) {
                addr_buf[0] = (uint8_t)(reg_addr >> 8);
                addr_buf[1] = (uint8_t)(reg_addr & 0xFFU);
                ret = I2C_TransData(cfg->unit, addr_buf, 2U, cfg->timeout);
            } else {
                addr_buf[0] = (uint8_t)(reg_addr & 0xFFU);
                ret = I2C_TransData(cfg->unit, addr_buf, 1U, cfg->timeout);
            }
            if (ret == LL_OK) {
                /* 5. 发送数据 */
                ret = I2C_TransData(cfg->unit, buf, len, cfg->timeout);
            }
        }
    }
    /* 6. 发送停止信号 */
    (void)I2C_Stop(cfg->unit, cfg->timeout);
    return ret;
}

/**
 * @brief  从从机读数据（Repeated START 时序）
 * @param  hi2c        I2C 句柄
 * @param  dev_addr    7 位从机地址
 * @param  reg_addr    寄存器/内存地址
 * @param  reg_addr_len 地址字节数（1 或 2）
 * @param  buf         接收缓冲区
 * @param  len         字节数
 * @retval LL_OK / LL_ERR / LL_ERR_TIMEOUT
 */
int32_t i2c_read(i2c_handle_t *hi2c, uint16_t dev_addr, uint16_t reg_addr,
                 uint8_t reg_addr_len, uint8_t *buf, uint32_t len)
{
    const i2c_config_t *cfg = hi2c->cfg;
    int32_t ret;
    uint8_t addr_buf[2];

    /* 1. 复位 I2C 外设并重新使能 */
    I2C_SWResetCmd(cfg->unit, ENABLE);
    I2C_SWResetCmd(cfg->unit, DISABLE);
    I2C_BusWaitCmd(cfg->unit, ENABLE);
    I2C_Cmd(cfg->unit, ENABLE);

    /* 2. 发送起始信号 */
    ret = I2C_Start(cfg->unit, cfg->timeout);
    if (ret == LL_OK) {
        /* 3. 发送从机地址 */
        ret = I2C_TransAddr(cfg->unit, dev_addr, I2C_DIR_TX, cfg->timeout);
        if (ret == LL_OK) {
            /* 4. 发送寄存器/内存地址 */
            if (reg_addr_len == 2U) {
                addr_buf[0] = (uint8_t)(reg_addr >> 8);
                addr_buf[1] = (uint8_t)(reg_addr & 0xFFU);
                ret = I2C_TransData(cfg->unit, addr_buf, 2U, cfg->timeout);
            } else {
                addr_buf[0] = (uint8_t)(reg_addr & 0xFFU);
                ret = I2C_TransData(cfg->unit, addr_buf, 1U, cfg->timeout);
            }
            if (ret == LL_OK) {
                /* 5. 发送重复起始信号 */
                ret = I2C_Restart(cfg->unit, cfg->timeout);
                if (ret == LL_OK) {
                    if (len == 1UL) {
                        I2C_AckConfig(cfg->unit, I2C_NACK);
                    }
                    /* 6. 发送从机地址（读方向） */
                    ret = I2C_TransAddr(cfg->unit, dev_addr, I2C_DIR_RX, cfg->timeout);
                    if (ret == LL_OK) {
                        /* 7. 接收数据 */
                        ret = I2C_MasterReceiveDataAndStop(cfg->unit, buf, len, cfg->timeout);
                    }
                    /* 8. 恢复 ACK */
                    I2C_AckConfig(cfg->unit, I2C_ACK);
                }
            }
        }
    }
    /* 9. 发送停止信号（仅在出错时） */
    if (ret != LL_OK) {
        (void)I2C_Stop(cfg->unit, cfg->timeout);
    }
    return ret;
}

/**
 * @brief  检测从机是否响应 ACK（用于写后轮询就绪）
 * @param  hi2c      I2C 句柄
 * @param  dev_addr  7 位从机地址
 * @retval LL_OK（ACK）/ LL_ERR（NACK）/ LL_ERR_TIMEOUT
 */
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
            /* TransAddr 超时说明从机返回了 NACK，设备未就绪 */
            ret = LL_ERR;
        }
    }
    (void)I2C_Stop(cfg->unit, cfg->timeout);
    return ret;
}

/**
 * @brief  总线恢复：发送 9 个时钟脉冲释放被从机拉低的 SDA，然后重新初始化外设
 * @param  hi2c  I2C 句柄
 * @retval LL_OK / LL_ERR_BUSY / LL_ERR_TIMEOUT
 */
int32_t i2c_recover(i2c_handle_t *hi2c)
{
    const i2c_config_t *cfg = hi2c->cfg;
    uint8_t i;
    int32_t ret;
    stc_gpio_init_t stcGpio;

    /* 1. 关闭 I2C 外设 */
    I2C_Cmd(cfg->unit, DISABLE);
    I2C_SWResetCmd(cfg->unit, ENABLE);
    I2C_SWResetCmd(cfg->unit, DISABLE);

    /* 2. SCL/SDA 切换为开漏 GPIO */
    GPIO_SetFunc(cfg->scl_port, cfg->scl_pin, GPIO_FUNC_0);
    GPIO_SetFunc(cfg->sda_port, cfg->sda_pin, GPIO_FUNC_0);
    (void)GPIO_StructInit(&stcGpio);
    stcGpio.u16PinState = PIN_STAT_SET;
    stcGpio.u16PinDir   = PIN_DIR_OUT;
    (void)GPIO_Init(cfg->scl_port, cfg->scl_pin, &stcGpio);
    (void)GPIO_Init(cfg->sda_port, cfg->sda_pin, &stcGpio);

    /* 3. 设置 SCL/SDA 为高电平 */
    GPIO_SetPins(cfg->scl_port, cfg->scl_pin);
    GPIO_SetPins(cfg->sda_port, cfg->sda_pin);
    DDL_DelayUS(BUS_RECOVERY_DELAY_US);

    /* 4. 等待 SCL 变为高电平 */
    ret = wait_scl_high(cfg, BUS_RECOVERY_TIMEOUT_US);
    if (ret == LL_OK) {
        /* 5. 如果 SDA 被从机拉低，则发送 9 个时钟脉冲 */
        if (PIN_RESET == GPIO_ReadInputPins(cfg->sda_port, cfg->sda_pin)) {
            for (i = 0U; i < BUS_RECOVERY_CLK_NUM; i++) {
                GPIO_ResetPins(cfg->scl_port, cfg->scl_pin);
                DDL_DelayUS(BUS_RECOVERY_DELAY_US);
                GPIO_SetPins(cfg->scl_port, cfg->scl_pin);
                ret = wait_scl_high(cfg, BUS_RECOVERY_TIMEOUT_US);
                if (ret != LL_OK) {
                    break;
                }
                DDL_DelayUS(BUS_RECOVERY_DELAY_US);
                /* 6. 如果 SDA 被释放，则停止 */
                if (PIN_SET == GPIO_ReadInputPins(cfg->sda_port, cfg->sda_pin)) {
                    break;
                }
            }
        }
        /* 7. 如果 SDA 被释放，则发送停止信号 */
        if (ret == LL_OK) {
            gpio_stop(cfg);
        }
        /* 8. 如果 SCL/SDA 仍被拉低，则返回错误 */
        if ((ret == LL_OK) &&
            ((PIN_RESET == GPIO_ReadInputPins(cfg->scl_port, cfg->scl_pin)) ||
             (PIN_RESET == GPIO_ReadInputPins(cfg->sda_port, cfg->sda_pin)))) {
            ret = LL_ERR_BUSY;
        }
    }

    /* 9. 无论恢复是否成功，都重新初始化外设，让硬件回到确定状态 */
    (void)i2c_init(hi2c, cfg);
    return ret;
}
