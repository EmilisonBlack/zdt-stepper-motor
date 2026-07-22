/**
 * @file    example_stm32_hal.c
 * @brief   STM32 HAL 库移植参考示例
 * 
 * 使用方法:
 *   1. 将 sdk/include/zdt_stepper.h 和 sdk/src/zdt_stepper.c 加入 STM32 工程
 *   2. 在本文件中实现平台抽象层
 *   3. 在 main.c 中 include 并调用
 */

#include "zdt_stepper.h"

/* ========================================================================
 *  平台抽象层实现 (STM32 HAL库)
 * ======================================================================== */

/*
 * 注意: 请根据实际使用的串口修改 huart1 为对应的串口句柄
 * 例如: &huart1, &huart2, &huart3 等
 */
extern UART_HandleTypeDef huart1;

/**
 * @brief 串口发送
 */
int zdt_platform_uart_send(const uint8_t *data, uint16_t len)
{
    /*
     * 方式1: 阻塞发送 (简单但会阻塞CPU)
     */
    if (HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 100) == HAL_OK) {
        return len;
    }
    return 0;

    /*
     * 方式2: DMA发送 (推荐, 不阻塞CPU)
     * 注意: 需要确保前一次DMA传输完成后再发送下一条命令
     */
    // if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, len) == HAL_OK) {
    //     return len;
    // }
    // return 0;
}

/**
 * @brief 串口接收 (带超时)
 * 
 * 推荐使用 中断+超时 方式实现:
 *   1. 每收到一个字节存入环形缓冲区
 *   2. 本函数检查缓冲区
 *   3. 超过 timeout_ms 没有新数据则认为帧结束
 */
int zdt_platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    /*
     * 简易查询方式 (适用于简单应用):
     * 用 HAL_UART_Receive 配合超时, 或自己实现帧接收状态机
     *
     * 参考官方Arduino例程中的 Emm_V5_Receive_Data():
     *   - 循环检查串口是否有数据
     *   - 有数据则存入缓冲区
     *   - 超过100ms无数据则认为一帧结束
     */
    uint16_t idx = 0;
    uint32_t tick = HAL_GetTick();
    
    while (1) {
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
            if (idx < len) {
                buf[idx++] = (uint8_t)(huart1.Instance->DR & 0xFF);
                tick = HAL_GetTick(); /* 重置超时 */
            }
            __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
        }
        
        if (idx > 0 && (HAL_GetTick() - tick) > timeout_ms) {
            return idx; /* 超时, 返回已接收的数据 */
        }
        
        if (HAL_GetTick() - tick > timeout_ms + 50) {
            return 0; /* 无数据超时 */
        }
    }
}

/**
 * @brief 毫秒延时
 */
void zdt_platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ========================================================================
 *  使用示例 (放在 main() 中调用)
 * ======================================================================== */

void stepper_demo(void)
{
    int16_t speed;
    int32_t position;
    uint32_t voltage;
    uint8_t flags;
    int ret;

    /* 1. 初始化SDK: 地址1, 固定校验0x6B */
    zdt_Init(1, ZDT_CHK_FIXED);

    /* 2. 等待驱动器上电初始化 */
    zdt_platform_delay(2000);

    /* 3. 读取总线电压 */
    ret = zdt_ReadBusVoltage(&voltage);
    if (ret == 0) {
        printf("总线电压: %lu mV\r\n", voltage);
    } else {
        printf("读取电压失败: %d\r\n", ret);
    }

    /* 4. 使能电机 */
    zdt_Enable(true, false);
    zdt_platform_delay(100);

    /* 5. 速度模式: 顺时针, 500 RPM, 加速度50 */
    zdt_SpeedMode(ZDT_DIR_CW, 500, 50, false);
    zdt_platform_delay(3000);

    /* 6. 读取转速 */
    zdt_ReadSpeed(&speed);
    printf("当前转速: %d RPM\r\n", speed);

    /* 7. 停止 */
    zdt_Stop(false);
    zdt_platform_delay(500);

    /* 8. 位置模式: 转1圈 (16细分: 3200脉冲/圈) */
    zdt_PositionModeRel(ZDT_DIR_CW, 800, 100, 3200, false);
    zdt_platform_delay(2000);

    /* 9. 读取位置 */
    zdt_ReadPosition(&position);
    printf("当前位置: %d.%d度\r\n", position / 10, position % 10);

    /* 10. 读取状态 */
    zdt_ReadStatusFlags(&flags);
    printf("状态: 0x%02X\r\n", flags);

    /* 11. 回零 */
    zdt_TriggerHome(ZDT_HOME_NEAREST, false);

    /* 12. 关闭电机 */
    zdt_Enable(false, false);
}
