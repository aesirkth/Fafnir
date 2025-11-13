/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_com.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "main.h"

static uint8_t can_rx_buffer[8];

int main(void)
{
    int ret = init_can(can_rx_buffer);
    if (ret < 0) {
        return 0;
    }

    k_msleep(200);


    while (1) {
        send_test_frame();
        k_msleep(20);
    }
}
