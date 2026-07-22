/**
 * @file    platform_win32.c
 * @brief   Windows 平台抽象层实现 (Win32 API 串口)
 */

#include "zdt_stepper.h"
#include <windows.h>
#include <stdio.h>

static HANDLE s_hCom = INVALID_HANDLE_VALUE;

int platform_uart_open(const char *port, int baudrate)
{
    char path[32];
    snprintf(path, sizeof(path), "\\\\.\\%s", port);

    s_hCom = CreateFileA(path, GENERIC_READ | GENERIC_WRITE,
                         0, NULL, OPEN_EXISTING, 0, NULL);
    if (s_hCom == INVALID_HANDLE_VALUE) {
        printf("[平台] 打开 %s 失败, 错误码: %lu\n", port, GetLastError());
        return -1;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    GetCommState(s_hCom, &dcb);
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    SetCommState(s_hCom, &dcb);

    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 100;
    SetCommTimeouts(s_hCom, &to);

    printf("[平台] 打开 %s @ %d bps 成功\n", port, baudrate);
    return 0;
}

int zdt_platform_uart_send(const uint8_t *data, uint16_t len)
{
    if (s_hCom == INVALID_HANDLE_VALUE) return 0;
    DWORD written = 0;
    WriteFile(s_hCom, data, len, &written, NULL);
    return (int)written;
}

int zdt_platform_uart_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (s_hCom == INVALID_HANDLE_VALUE) return 0;

    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = timeout_ms;
    SetCommTimeouts(s_hCom, &to);

    DWORD read = 0;
    if (!ReadFile(s_hCom, buf, len, &read, NULL))
        return 0;
    return (int)read;
}

void zdt_platform_delay(uint32_t ms)
{
    Sleep(ms);
}

/* 关闭串口 */
void platform_uart_close(void)
{
    if (s_hCom != INVALID_HANDLE_VALUE) {
        CloseHandle(s_hCom);
        s_hCom = INVALID_HANDLE_VALUE;
        printf("[平台] 串口已关闭\n");
    }
}
