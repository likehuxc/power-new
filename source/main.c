/**
 *******************************************************************************
 * @file  main.c
 * @brief 入口与主循环；外设与应用逻辑拆分至 board / drv / app。
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *******************************************************************************
 */

#include "main.h"

#include "board.h"
#include "app.h"

int32_t main(void)
{
    Board_PeriphUnlock();
    Board_Init();
    (void)App_Init();

    for (;;) {
        App_Process();
    }
}
