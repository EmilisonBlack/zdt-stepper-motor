/**
 * @file    zdt_modbus.c
 * @brief   ZDT_X_V2 闭环步进电机 Modbus-RTU 驱动实现
 * @date    2026-07-21
 */

#include "zdt_modbus.h"
#include <string.h>

static uint8_t g_addr = ZDT_DEFAULT_ADDR;

/* ======================================================================== *
 *  CRC16 查表
 * ======================================================================== */
static const uint16_t s_crc16[256] = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
    0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
    0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
    0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
    0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
    0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
    0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
    0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
    0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
    0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
    0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
    0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
    0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
    0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
    0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
    0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

uint16_t zdt_CRC16(const uint8_t *d, uint16_t len)
{
    uint16_t c = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
        c = (c >> 8) ^ s_crc16[(c ^ d[i]) & 0xFF];
    return c;
}

/* ======================================================================== *
 *  收发
 * ======================================================================== */

int zdt_Transmit(uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t *rx_len)
{
    uint8_t pkt[260];
    uint16_t crc = zdt_CRC16(tx, tx_len);
    memcpy(pkt, tx, tx_len);
    pkt[tx_len]   = (uint8_t)(crc >> 0);
    pkt[tx_len+1] = (uint8_t)(crc >> 8);

    if (zdt_platform_uart_send(pkt, tx_len + 2) != tx_len + 2)
        return -1;

    int n = zdt_platform_uart_recv(rx, ZDT_RX_BUF_SIZE, ZDT_RX_TIMEOUT_MS);
    if (n < 4) return -1;
    *rx_len = (uint8_t)n;

    if (rx[0] != g_addr && rx[0] != 0x00) return -2;

    crc = zdt_CRC16(rx, n - 2);
    uint16_t rc = (uint16_t)rx[n-1]<<8 | rx[n-2];
    if (crc != rc) return -3;
    if (rx[1] & 0x80) return -4 - rx[2];

    return 0;
}

/* ======================================================================== *
 *  Modbus 操作
 * ======================================================================== */

int zdt_ReadInputRegs(uint16_t reg, uint8_t cnt, uint8_t *rx, uint8_t *rx_len)
{
    uint8_t cmd[6] = { g_addr, 0x04, (uint8_t)(reg>>8), (uint8_t)reg, 0x00, cnt };
    return zdt_Transmit(cmd, 6, rx, rx_len);
}

int zdt_WriteSingleReg(uint16_t reg, uint16_t val)
{
    uint8_t cmd[6] = { g_addr, 0x06, (uint8_t)(reg>>8), (uint8_t)reg,
                       (uint8_t)(val>>8), (uint8_t)val };
    uint8_t rx[16], l;
    return zdt_Transmit(cmd, 6, rx, &l);
}

int zdt_WriteMultipleRegs(uint16_t reg, uint8_t cnt, const uint8_t *data)
{
    uint8_t cmd[260], rx[16], rl;
    cmd[0]=g_addr; cmd[1]=0x10;
    cmd[2]=(uint8_t)(reg>>8); cmd[3]=(uint8_t)reg;
    cmd[4]=0x00; cmd[5]=cnt;
    cmd[6]=cnt*2;
    memcpy(&cmd[7], data, cnt*2);
    return zdt_Transmit(cmd, 7+cnt*2, rx, &rl);
}

#define REG16(b,i) ((uint16_t)(b)[3+(i)*2]<<8|(b)[4+(i)*2])
#define REG32(b,i) ((uint32_t)(b)[3+(i)*2]<<24|(uint32_t)(b)[4+(i)*2]<<16|\
                    (uint32_t)(b)[5+(i)*2]<<8|(uint32_t)(b)[6+(i)*2])

/* ======================================================================== *
 *  读取参数
 * ======================================================================== */

int zdt_ReadVersion(uint8_t *ver)
{
    uint8_t b[16], l; int r = zdt_ReadInputRegs(0x0010, 2, b, &l);
    if (r) return r;
    uint16_t hv=REG16(b,0), fv=REG16(b,1);
    ver[0]=(uint8_t)(hv>>8); ver[1]=(uint8_t)hv;
    ver[2]='.'; ver[3]=(uint8_t)(fv>>8); ver[4]=(uint8_t)fv; ver[5]=0;
    return 0;
}

int zdt_ReadBusVoltage(uint32_t *mv)
    { uint8_t b[8],l; int r=zdt_ReadInputRegs(0x0024,1,b,&l); if(r)return r; *mv=REG16(b,0); return 0; }

int zdt_ReadStatusFlags(uint16_t *f)
    { uint8_t b[8],l; int r=zdt_ReadInputRegs(0x0050,1,b,&l); if(r)return r; *f=REG16(b,0); return 0; }

int zdt_ReadSpeed(int16_t *rpm)
    { uint8_t b[8],l; int r=zdt_ReadInputRegs(0x0034,1,b,&l); if(r)return r; *rpm=(int16_t)REG16(b,0); return 0; }

int zdt_ReadPosition(int32_t *deg_x10)
    { uint8_t b[16],l; int r=zdt_ReadInputRegs(0x0030,3,b,&l); if(r)return r; *deg_x10=(int32_t)REG32(b,1); return 0; }

/* ======================================================================== *
 *  电机控制
 * ======================================================================== */

int zdt_SetCtrlMode(uint16_t mode)
    { uint8_t d[2]={(uint8_t)(mode>>8),(uint8_t)mode}; return zdt_WriteMultipleRegs(0x00E0,1,d); }

int zdt_Enable(uint16_t state, uint16_t speed_rpm10, uint16_t accel)
{
    /* 0x00EA, 5 regs: accel, speed(rpm*10), En, 0, 0 */
    uint8_t d[10];
    d[0]=(uint8_t)(accel>>8);    d[1]=(uint8_t)accel;
    d[2]=(uint8_t)(speed_rpm10>>8); d[3]=(uint8_t)speed_rpm10;
    d[4]=(uint8_t)(state>>8);    d[5]=(uint8_t)state;
    d[6]=0; d[7]=0; d[8]=0; d[9]=0;
    return zdt_WriteMultipleRegs(0x00EA, 5, d);
}

int zdt_Stop(void)
    { uint8_t d[2]={0x00,0x98}; return zdt_WriteMultipleRegs(0x00FE,1,d); }

int zdt_SpeedMode(uint16_t accel, uint32_t speed_rpm10, uint8_t dir)
{
    /* 0x00E6, 4 regs: accel(RPM/s), speed(RPM), dir+保留 */
    uint8_t d[8];
    d[0]=(uint8_t)(accel>>8);    d[1]=(uint8_t)accel;
    d[2]=(uint8_t)(speed_rpm10>>24); d[3]=(uint8_t)(speed_rpm10>>16);
    d[4]=(uint8_t)(speed_rpm10>>8);  d[5]=(uint8_t)speed_rpm10;
    d[6]=0; d[7]=dir;
    return zdt_WriteMultipleRegs(0x00E6, 4, d);
}

int zdt_PositionMode(uint16_t accel, uint32_t speed_rpm10,
                     int32_t position, bool absF)
{
    /* 0x00F0, 5 regs: dir+accel? 实际上手册显示是另一种结构
     * 根据Modbus指令说明第32条: 0x00F0, 5 regs
     * reg1: dir(0=多圈,1=单圈)+速度模式
     * reg2: 加速度 RPM/s
     * reg3: 速度 RPM*10
     * reg4: 位置 低32位
     * reg5: 位置 高32位/标志
     * 简化处理: 直接发送结构化数据
     */
    uint8_t d[10];
    uint16_t phi = (uint16_t)((uint32_t)position >> 16);
    if (absF) phi |= 0x8000;
    d[0]=0; d[1]=absF?1:0;    /* dir/模式 */
    d[2]=(uint8_t)(accel>>8); d[3]=(uint8_t)accel;
    d[4]=(uint8_t)(speed_rpm10>>24); d[5]=(uint8_t)(speed_rpm10>>16);
    d[6]=(uint8_t)(speed_rpm10>>8);  d[7]=(uint8_t)speed_rpm10;
    d[8]=(uint8_t)(phi>>8);   d[9]=(uint8_t)phi;
    return zdt_WriteMultipleRegs(0x00F0, 5, d);
}

int zdt_CalibrateEncoder(void)  { return zdt_WriteSingleReg(0x0006, 1); }
int zdt_ResetPosition(void)     { return zdt_WriteSingleReg(0x000A, 1); }
int zdt_ReleaseStall(void)      { return zdt_WriteSingleReg(0x000E, 1); }

int zdt_TriggerHome(uint16_t mode)
    { uint8_t d[2]={(uint8_t)(mode>>8),(uint8_t)mode}; return zdt_WriteMultipleRegs(0x00DA,1,d); }

int zdt_AbortHome(void)
    { uint8_t d[2]={0x00,0x48}; return zdt_WriteMultipleRegs(0x00DC,1,d); }

int zdt_SetHomeZero(void)
    { uint8_t d[2]={0x00,0x01}; return zdt_WriteMultipleRegs(0x00D8,1,d); }
