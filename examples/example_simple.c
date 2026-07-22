/**
 * @file    example_simple.c
 * @brief   ZDT/Emm_V5.0 闭环步进电机SDK使用示例
 * @note    此示例需要用户实现平台抽象层接口后编译运行
 */

#include "zdt_stepper.h"
#include <stdio.h>
#include <string.h>

/* ======================================================================== *
 *  平台抽象层实现示例 (以Windows/Linux串口为例, 用户需适配自己的硬件)
 * ======================================================================== */

/*
 * !!! 重要: 用户需要根据自家平台实现以下三个函数 !!!
 *
 * 1. zdt_platform_uart_send()  - 串口发送
 * 2. zdt_platform_uart_recv()  - 串口接收 (带超时)
 * 3. zdt_platform_delay()      - 毫秒延时
 *
 * 以下是基于标准POSIX串口的参考实现框架
 */

#if 0 /* 用户需替换为实际平台代码 */

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

static int s_uart_fd = -1;

/* 初始化串口 */
int uart_open(const char *dev, int baudrate)
{
    s_uart_fd = open(dev, O_RDWR | O_NOCTTY);
    if (s_uart_fd < 0) return -1;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    cfsetospeed(&tio, baudrate);
    cfsetispeed(&tio, baudrate);
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tcflush(s_uart_fd, TCIOFLUSH);
    tcsetattr(s_uart_fd, TCSANOW, &tio);
    return 0;
}

/* 串口发送 */
int zdt_platform_uart_send(const uint8_t *data, uint16_t len)
{
    return (int)write(s_uart_fd, data, len);
}

/* 串口接收 (带超时) */
int zdt_platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    fd_set set;
    struct timeval tv;
    FD_ZERO(&set);
    FD_SET(s_uart_fd, &set);
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(s_uart_fd + 1, &set, NULL, NULL, &tv);
    if (ret <= 0) return 0;

    return (int)read(s_uart_fd, buf, len);
}

/* 延时 */
void zdt_platform_delay(uint32_t ms)
{
    usleep(ms * 1000);
}

#endif /* 平台代码框架结束 */

/* ======================================================================== *
 *  使用示例
 * ======================================================================== */

void demo_basic_control(void)
{
    int ret;

    printf("===== ZDT步进电机基础控制示例 =====\n\n");

    /* ---- 1. 初始化SDK ---- */
    /* 地址1, 使用默认校验模式(固定0x6B) */
    zdt_Init(1, ZDT_CHK_FIXED);
    printf("[OK] SDK初始化完成 (地址: 1, 校验模式: 固定0x6B)\n");

    /* 上电后等待驱动器初始化 */
    zdt_platform_delay(2000);

    /* ---- 2. 读取固件版本 ---- */
    {
        char ver[16];
        ret = zdt_ReadVersion((uint8_t*)ver);
        if (ret == 0) {
            printf("[OK] 固件版本: %s\n", ver);
        } else {
            printf("[ERR] 读取版本失败: %d\n", ret);
        }
    }

    /* ---- 3. 读取总线电压 ---- */
    {
        uint32_t voltage;
        ret = zdt_ReadBusVoltage(&voltage);
        if (ret == 0) {
            printf("[OK] 总线电压: %u mV (%.1f V)\n",
                   voltage, voltage / 1000.0f);
        } else {
            printf("[ERR] 读取电压失败: %d\n", ret);
        }
    }

    /* ---- 4. 使能电机 ---- */
    ret = zdt_Enable(true, false);
    printf("%s 使能电机\n", (ret == 0) ? "[OK]" : "[ERR]");
    zdt_platform_delay(100);

    /* ---- 5. 读取状态标志 ---- */
    {
        uint8_t flags;
        ret = zdt_ReadStatusFlags(&flags);
        if (ret == 0) {
            printf("[OK] 状态标志: 0x%02X (%s%s%s)\n", flags,
                   (flags & ZDT_FLAG_ENABLE) ? "已使能 " : "",
                   (flags & ZDT_FLAG_IN_POS) ? "已到位 " : "",
                   (flags & ZDT_FLAG_STALL)  ? "堵转! "  : "");
        }
    }

    /* ---- 6. 速度模式: CW方向, 500RPM, 加速度50 ---- */
    printf("\n--- 速度模式测试 ---\n");
    ret = zdt_SpeedMode(ZDT_DIR_CW, 500, 50, false);
    printf("%s 速度模式: CW 500RPM 加速度50\n", (ret == 0) ? "[OK]" : "[ERR]");

    /* 运行2秒 */
    zdt_platform_delay(2000);

    /* 读取实时转速 */
    {
        int16_t speed;
        ret = zdt_ReadSpeed(&speed);
        if (ret == 0) {
            printf("[OK] 当前转速: %d RPM\n", speed);
        }
    }

    /* 停止 */
    zdt_Stop(false);
    printf("[OK] 停止电机\n");
    zdt_platform_delay(500);

    /* ---- 7. 位置模式: 相对运动, 转1圈(16细分下3200脉冲) ---- */
    printf("\n--- 位置模式测试 ---\n");
    ret = zdt_PositionModeRel(ZDT_DIR_CW, 800, 100, 3200, false);
    printf("%s 位置模式: 相对运动 CW 800RPM 脉冲数3200\n", (ret == 0) ? "[OK]" : "[ERR]");

    /* 等待运动完成 */
    zdt_platform_delay(3000);

    /* 读取实时位置 */
    {
        int32_t pos;
        ret = zdt_ReadPosition(&pos);
        if (ret == 0) {
            printf("[OK] 当前位置: %d.%d°\n", pos / 10, pos % 10);
        }
    }

    /* ---- 8. 触发单圈就近回零 ---- */
    printf("\n--- 回零测试 ---\n");
    ret = zdt_TriggerHome(ZDT_HOME_NEAREST, false);
    printf("%s 触发单圈就近回零\n", (ret == 0) ? "[OK]" : "[ERR]");

    /* 等待回零完成 */
    zdt_platform_delay(3000);

    /* 读取回零状态 */
    {
        bool busy, failed;
        ret = zdt_ReadHomeStatus(&busy, &failed);
        if (ret == 0) {
            printf("[OK] 回零状态: %s%s\n",
                   busy  ? "正在回零..." : "回零完成",
                   failed ? " (失败!)" : "");
        }
    }

    /* ---- 9. 关闭电机 ---- */
    zdt_Enable(false, false);
    printf("[OK] 关闭电机\n");

    printf("\n===== 示例结束 =====\n");
}

void demo_read_all_params(void)
{
    printf("\n===== 读取全部参数 =====\n\n");

    zdt_Init(1, ZDT_CHK_FIXED);
    zdt_platform_delay(2000);

    /* 固件版本 */
    char ver[16];
    if (zdt_ReadVersion((uint8_t*)ver) == 0)
        printf("固件版本: %s\n", ver);

    /* 总线电压 */
    uint32_t val32;
    if (zdt_ReadBusVoltage(&val32) == 0)
        printf("总线电压: %u mV\n", val32);

    /* 相电流 */
    if (zdt_ReadPhaseCurrent(&val32) == 0)
        printf("相电流: %u mA\n", val32);

    /* 编码器值 */
    int32_t enc;
    if (zdt_ReadEncoder(&enc) == 0)
        printf("编码器值: %d\n", enc);

    /* 实时转速 */
    int16_t speed;
    if (zdt_ReadSpeed(&speed) == 0)
        printf("实时转速: %d RPM\n", speed);

    /* 实时位置 */
    int32_t pos;
    if (zdt_ReadPosition(&pos) == 0)
        printf("实时位置: %d.%d°\n", pos / 10, pos % 10);

    /* 目标位置 */
    if (zdt_ReadTargetPosition(&pos) == 0)
        printf("目标位置: %d.%d°\n", pos / 10, pos % 10);

    /* 位置误差 */
    if (zdt_ReadPositionError(&pos) == 0)
        printf("位置误差: %d.%d°\n", pos / 10, pos % 10);

    /* 状态标志 */
    uint8_t flags;
    if (zdt_ReadStatusFlags(&flags) == 0) {
        printf("状态标志: 0x%02X ", flags);
        if (flags & ZDT_FLAG_ENABLE) printf("[已使能]");
        if (flags & ZDT_FLAG_IN_POS) printf("[已到位]");
        if (flags & ZDT_FLAG_STALL)  printf("[堵转!]");
        printf("\n");
    }

    /* 回零状态 */
    bool busy, failed;
    if (zdt_ReadHomeStatus(&busy, &failed) == 0)
        printf("回零状态: %s%s\n",
               busy ? "忙" : "空闲",
               failed ? " (失败)" : "");
}

int main(void)
{
    demo_basic_control();
    demo_read_all_params();
    return 0;
}
