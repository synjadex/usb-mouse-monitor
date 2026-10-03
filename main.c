/*
 * mouse_serial_bridge.c
 * ============================================================================
 * PC端鼠标数据采集 + 串口透传桥接程序 (Windows)
 *
 * 功能:
 *   1. 扫描系统中的 COM 串口和 USB HID 鼠标设备
 *   2. 交互式让用户选择输出 COM 端口
 *   3. 通过 WH_MOUSE_LL 全局钩子捕获鼠标移动/点击/滚轮事件
 *   4. 按 A5 5A 二进制协议帧 (10字节) 通过串口发送
 *
 * 编译 (MinGW / MSVC):
 *   MinGW:  gcc -o mouse_serial_bridge.exe mouse_serial_bridge.c -luser32 -lkernel32 -lsetupapi
 *   MSVC:   cl mouse_serial_bridge.c /link user32.lib kernel32.lib setupapi.lib
 *
 * 协议格式 v3.0 (10字节固定帧):
 *   [A5][5A][06][Button][X_L][X_H][Y_L][Y_H][Wheel][CRC8]
 *   - 帧头: A5 5A
 *   - 长度: 0x06 (固定)
 *   - Button: 位域编码 (Bit0=左键, Bit1=右键, Bit2=中键, Bit3=侧键1, Bit4=侧键2)
 *   - X/Y: 16位有符号小端序相对位移
 *   - Wheel: 8位有符号滚轮偏移量 (以"格"为单位, ±1 表示一档)
 *   - CRC8: CRC-8-MAXIM 校验 (多项式0x31, 校验范围=长度+数据域共7字节)
 * ============================================================================
 */

#include <windows.h>
#include <setupapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ===========================================================================
 * 编译兼容处理
 * =========================================================================== */
#ifndef WHEEL_DELTA
#define WHEEL_DELTA 120
#endif

#ifndef XBUTTON1
#define XBUTTON1 0x0001
#endif

#ifndef XBUTTON2
#define XBUTTON2 0x0002
#endif

#ifndef WM_MOUSEHWHEEL
#define WM_MOUSEHWHEEL 0x020E
#endif

/* ===========================================================================
 * GUID 定义 (避免依赖 devguid.h)
 * =========================================================================== */
static const GUID GUID_DEVCLASS_PORTS =
    { 0x4D36E978, 0xE325, 0x11CE, { 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 } };

static const GUID GUID_DEVCLASS_MOUSE =
    { 0x4D36E96F, 0xE325, 0x11CE, { 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 } };

/* ===========================================================================
 * 协议常量
 * =========================================================================== */
#define FRAME_HEADER_1   0xA5
#define FRAME_HEADER_2   0x5A
#define DATA_LENGTH      0x06
#define FRAME_TOTAL_LEN  10

/* 按键位掩码 */
#define BTN_MASK_LEFT    0x01   /* Bit0: 左键   */
#define BTN_MASK_RIGHT   0x02   /* Bit1: 右键   */
#define BTN_MASK_MIDDLE  0x04   /* Bit2: 中键   */
#define BTN_MASK_SIDE1   0x08   /* Bit3: 侧键1  */
#define BTN_MASK_SIDE2   0x10   /* Bit4: 侧键2  */

/* ===========================================================================
 * 环形队列 (线程安全帧缓冲)
 * =========================================================================== */
#define QUEUE_SIZE 256

typedef struct {
    BYTE data[FRAME_TOTAL_LEN];
} FrameEntry;

typedef struct {
    FrameEntry entries[QUEUE_SIZE];
    LONG        head;          /* 写入位置 */
    LONG        tail;          /* 读取位置 */
    LONG        count;         /* 当前帧数 */
    CRITICAL_SECTION cs;
    HANDLE      hDataReady;    /* 自动重置事件: 有数据时置位 */
} FrameQueue;

/* ===========================================================================
 * 全局状态
 * =========================================================================== */
static FrameQueue g_queue;

/* 鼠标状态 */
static volatile LONG g_last_x        = 0;
static volatile LONG g_last_y        = 0;
static volatile int  g_first_move    = 1;   /* 跳过首次移动 */
static volatile BYTE g_button_state  = 0x00;

/* 运行控制 */
static volatile LONG g_running       = 1;

/* 串口 */
static HANDLE        g_hSerial       = INVALID_HANDLE_VALUE;
static char          g_port_name[32] = "";

/* 钩子 */
static HHOOK         g_hMouseHook    = NULL;

/* 显示时间基准 */
static DWORD         g_start_tick    = 0;

/* ===========================================================================
 * CRC-8-MAXIM 算法
 *   多项式: 0x31 (x^8 + x^5 + x^4 + 1)
 *   初始值: 0x00, 最终异或: 0x00
 *   输入反转: True, 输出反转: True
 * =========================================================================== */
static BYTE crc8_maxim(const BYTE *data, size_t len)
{
    BYTE crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x01) {
                crc = (BYTE)((crc >> 1) ^ 0x8C);  /* 0x8C = 反转后的 0x31 */
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ===========================================================================
 * 构建协议帧
 *   frame: 输出缓冲区 (至少 FRAME_TOTAL_LEN 字节)
 *   button_state: 按键位域
 *   dx, dy: 16位有符号相对位移
 *   wheel: 8位有符号滚轮偏移
 * =========================================================================== */
static void build_frame(BYTE frame[FRAME_TOTAL_LEN],
                        BYTE button_state,
                        int16_t dx, int16_t dy,
                        int8_t wheel)
{
    /* 小端序编码 */
    BYTE x_l = (BYTE)(dx & 0xFF);
    BYTE x_h = (BYTE)(((uint16_t)dx >> 8) & 0xFF);
    BYTE y_l = (BYTE)(dy & 0xFF);
    BYTE y_h = (BYTE)(((uint16_t)dy >> 8) & 0xFF);
    BYTE w_b = (BYTE)(wheel & 0xFF);

    /* 帧组装 */
    frame[0] = FRAME_HEADER_1;
    frame[1] = FRAME_HEADER_2;
    frame[2] = DATA_LENGTH;
    frame[3] = button_state;
    frame[4] = x_l;
    frame[5] = x_h;
    frame[6] = y_l;
    frame[7] = y_h;
    frame[8] = w_b;

    /* CRC8: 校验 长度 + 按键 + X_L + X_H + Y_L + Y_H + 滚轮 = 7字节 */
    frame[9] = crc8_maxim(&frame[2], 7);
}

/* ===========================================================================
 * 环形队列操作
 * =========================================================================== */
static void queue_init(FrameQueue *q)
{
    memset(q, 0, sizeof(*q));
    InitializeCriticalSection(&q->cs);
    q->hDataReady = CreateEvent(NULL, FALSE, FALSE, NULL);  /* 自动重置 */
}

static void queue_destroy(FrameQueue *q)
{
    DeleteCriticalSection(&q->cs);
    if (q->hDataReady) CloseHandle(q->hDataReady);
}

/* 入队: 队列满时丢弃新帧 (保留旧数据) */
static int queue_push(FrameQueue *q, const BYTE frame[FRAME_TOTAL_LEN])
{
    EnterCriticalSection(&q->cs);
    if (q->count >= QUEUE_SIZE) {
        LeaveCriticalSection(&q->cs);
        return 0;  /* 队列满, 丢弃 */
    }
    memcpy(q->entries[q->head].data, frame, FRAME_TOTAL_LEN);
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count++;
    LeaveCriticalSection(&q->cs);
    SetEvent(q->hDataReady);
    return 1;
}

/* 出队: 返回 1=成功, 0=队列空 */
static int queue_pop(FrameQueue *q, BYTE frame[FRAME_TOTAL_LEN])
{
    EnterCriticalSection(&q->cs);
    if (q->count <= 0) {
        LeaveCriticalSection(&q->cs);
        return 0;
    }
    memcpy(frame, q->entries[q->tail].data, FRAME_TOTAL_LEN);
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count--;
    LeaveCriticalSection(&q->cs);
    return 1;
}

/* ===========================================================================
 * COM 端口枚举 (使用 SetupAPI)
 *   返回动态分配的端口信息数组, *count 为端口数量
 *   调用者需要用 free() 释放每个条目的 name 和 desc, 以及数组本身
 * =========================================================================== */
typedef struct {
    int    number;        /* COM 端口号, 如 3 */
    char  *name;          /* 设备路径, 如 \\.\COM3 */
    char  *desc;          /* 设备描述, 如 "USB Serial Port (COM3)" */
    int    available;     /* 是否可打开 */
} ComPortInfo;

static ComPortInfo* enumerate_com_ports(int *out_count)
{
    HDEVINFO hDevInfo;
    SP_DEVINFO_DATA devInfoData;
    DWORD    i;
    int      capacity = 16;
    int      count = 0;

    ComPortInfo *ports = (ComPortInfo*)calloc(capacity, sizeof(ComPortInfo));
    if (!ports) { *out_count = 0; return NULL; }

    hDevInfo = SetupDiGetClassDevs(
        &GUID_DEVCLASS_PORTS, NULL, NULL,
        DIGCF_PRESENT
    );
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        free(ports);
        *out_count = 0;
        return NULL;
    }

    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); i++) {
        char  friendlyName[256] = "";
        DWORD regType = 0;
        DWORD required = 0;

        /* 获取友好名称 */
        if (SetupDiGetDeviceRegistryProperty(
                hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME,
                &regType, (PBYTE)friendlyName, sizeof(friendlyName) - 1, &required)) {
            friendlyName[required] = '\0';
        } else {
            /* 回退: 尝试设备描述 */
            SetupDiGetDeviceRegistryProperty(
                hDevInfo, &devInfoData, SPDRP_DEVICEDESC,
                &regType, (PBYTE)friendlyName, sizeof(friendlyName) - 1, &required);
            friendlyName[required] = '\0';
        }

        if (friendlyName[0] == '\0') {
            strcpy(friendlyName, "(未知设备)");
        }

        /* 从友好名称中提取 COM 号: 查找 "COM" 后跟数字 */
        char *com_pos = strstr(friendlyName, "COM");
        int com_num = 0;
        if (com_pos) {
            com_num = atoi(com_pos + 3);
        }

        /* 验证: 尝试 CreateFile 打开此 COM 口 */
        char port_path[32];
        sprintf(port_path, "\\\\.\\COM%d", com_num);
        int available = 0;
        if (com_num > 0 && com_num <= 256) {
            HANDLE hTest = CreateFile(
                port_path,
                GENERIC_READ | GENERIC_WRITE,
                0, NULL, OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL, NULL
            );
            if (hTest != INVALID_HANDLE_VALUE) {
                available = 1;
                CloseHandle(hTest);
            }
        }

        if (com_num > 0 && available) {
            /* 扩容 */
            if (count >= capacity) {
                capacity *= 2;
                ports = (ComPortInfo*)realloc(ports, capacity * sizeof(ComPortInfo));
                if (!ports) break;
            }

            ports[count].number    = com_num;
            ports[count].name      = _strdup(port_path);
            ports[count].desc      = _strdup(friendlyName);
            ports[count].available = available;
            count++;
        }
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    *out_count = count;
    return ports;
}

static void free_com_ports(ComPortInfo *ports, int count)
{
    for (int i = 0; i < count; i++) {
        free(ports[i].name);
        free(ports[i].desc);
    }
    free(ports);
}

/* ===========================================================================
 * USB HID 鼠标设备枚举
 * =========================================================================== */
typedef struct {
    int     vid;
    int     pid;
    char   *desc;
    char   *instance_id;
} MouseDeviceInfo;

static MouseDeviceInfo* enumerate_mouse_devices(int *out_count)
{
    HDEVINFO hDevInfo;
    SP_DEVINFO_DATA devInfoData;
    DWORD    i;
    int      capacity = 16;
    int      count = 0;

    MouseDeviceInfo *devices = (MouseDeviceInfo*)calloc(capacity, sizeof(MouseDeviceInfo));
    if (!devices) { *out_count = 0; return NULL; }

    hDevInfo = SetupDiGetClassDevs(
        &GUID_DEVCLASS_MOUSE, NULL, NULL,
        DIGCF_PRESENT
    );
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        free(devices);
        *out_count = 0;
        return NULL;
    }

    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); i++) {
        char   desc[256]       = "";
        char   instanceId[256] = "";
        DWORD  regType, required;

        /* 获取设备描述 */
        if (SetupDiGetDeviceRegistryProperty(
                hDevInfo, &devInfoData, SPDRP_DEVICEDESC,
                &regType, (PBYTE)desc, sizeof(desc) - 1, &required)) {
            desc[required] = '\0';
        }

        /* 获取实例 ID (含 VID/PID) */
        if (SetupDiGetDeviceInstanceId(
                hDevInfo, &devInfoData,
                instanceId, sizeof(instanceId) - 1, &required)) {
            instanceId[required] = '\0';
        }

        /* 解析 VID/PID */
        int vid = 0, pid = 0;
        char *vid_pos = strstr(instanceId, "VID_");
        char *pid_pos = strstr(instanceId, "PID_");
        if (vid_pos) vid = (int)strtol(vid_pos + 4, NULL, 16);
        if (pid_pos) pid = (int)strtol(pid_pos + 4, NULL, 16);

        if (count >= capacity) {
            capacity *= 2;
            devices = (MouseDeviceInfo*)realloc(devices, capacity * sizeof(MouseDeviceInfo));
            if (!devices) break;
        }

        devices[count].vid         = vid;
        devices[count].pid         = pid;
        devices[count].desc        = _strdup(desc[0] ? desc : "(未知设备)");
        devices[count].instance_id = _strdup(instanceId);
        count++;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    *out_count = count;
    return devices;
}

static void free_mouse_devices(MouseDeviceInfo *devices, int count)
{
    for (int i = 0; i < count; i++) {
        free(devices[i].desc);
        free(devices[i].instance_id);
    }
    free(devices);
}

/* ===========================================================================
 * 串口配置
 * =========================================================================== */
static int serial_open(const char *port_path, DWORD baudrate)
{
    g_hSerial = CreateFile(
        port_path,
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, NULL
    );

    if (g_hSerial == INVALID_HANDLE_VALUE) {
        printf("  [错误] 无法打开串口 %s (错误码: %lu)\n",
               port_path, GetLastError());
        return 0;
    }

    /* 配置 DCB */
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(g_hSerial, &dcb)) {
        printf("  [错误] 获取串口状态失败\n");
        CloseHandle(g_hSerial);
        g_hSerial = INVALID_HANDLE_VALUE;
        return 0;
    }

    dcb.BaudRate  = baudrate;
    dcb.ByteSize  = 8;
    dcb.StopBits  = ONESTOPBIT;
    dcb.Parity    = NOPARITY;
    dcb.fBinary   = TRUE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    if (!SetCommState(g_hSerial, &dcb)) {
        printf("  [错误] 设置串口状态失败\n");
        CloseHandle(g_hSerial);
        g_hSerial = INVALID_HANDLE_VALUE;
        return 0;
    }

    /* 配置超时 */
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(g_hSerial, &timeouts)) {
        printf("  [错误] 设置串口超时失败\n");
        CloseHandle(g_hSerial);
        g_hSerial = INVALID_HANDLE_VALUE;
        return 0;
    }

    /* 清空缓冲区 */
    PurgeComm(g_hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    return 1;
}

static void serial_close(void)
{
    if (g_hSerial != INVALID_HANDLE_VALUE) {
        PurgeComm(g_hSerial, PURGE_TXCLEAR);
        CloseHandle(g_hSerial);
        g_hSerial = INVALID_HANDLE_VALUE;
    }
}

static void serial_send(const BYTE *data, DWORD len)
{
    if (g_hSerial == INVALID_HANDLE_VALUE) return;

    DWORD written = 0;
    if (!WriteFile(g_hSerial, data, len, &written, NULL)) {
        /* 串口写入失败, 静默处理 (队列消费掉了) */
    }
}

/* ===========================================================================
 * 帧显示格式化
 * =========================================================================== */
static void print_frame(const BYTE frame[FRAME_TOTAL_LEN],
                        const char *event_type,
                        int16_t dx, int16_t dy, int8_t wheel)
{
    /* 计算运行时间 */
    DWORD elapsed = GetTickCount() - g_start_tick;
    int   hours   = (int)(elapsed / 3600000);
    int   mins    = (int)((elapsed / 60000) % 60);
    int   secs    = (int)((elapsed / 1000) % 60);
    int   ms      = (int)(elapsed % 1000);

    /* 十六进制帧 */
    char hex_buf[64];
    sprintf(hex_buf,
            "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            frame[0], frame[1], frame[2], frame[3],
            frame[4], frame[5], frame[6], frame[7],
            frame[8], frame[9]);

    /* 按键状态解码 */
    BYTE btn = frame[3];
    char btn_str[64] = "";
    if (btn & BTN_MASK_LEFT)   strcat(btn_str, "L");
    if (btn & BTN_MASK_RIGHT)  strcat(btn_str, "R");
    if (btn & BTN_MASK_MIDDLE) strcat(btn_str, "M");
    if (btn & BTN_MASK_SIDE1)  strcat(btn_str, "X1");
    if (btn & BTN_MASK_SIDE2)  strcat(btn_str, "X2");
    if (btn == 0) strcpy(btn_str, "-");

    printf("[%02d:%02d:%02d.%03d] %-6s | BTN:%-5s | dX:%+5d dY:%+5d | WHL:%+3d | [%s]\n",
           hours, mins, secs, ms,
           event_type,
           btn_str,
           dx, dy,
           wheel,
           hex_buf);
}

/* ===========================================================================
 * 鼠标钩子回调 (WH_MOUSE_LL)
 *
 *   注意: 此回调运行在消息循环线程中
 *   只做轻量工作: 计算位移、构建帧、入队
 *   串口 I/O 在独立线程中完成
 * =========================================================================== */
static LRESULT CALLBACK mouse_hook_proc(int nCode, WPARAM wParam, LPARAM lParam)
{
    if (nCode < 0) {
        return CallNextHookEx(NULL, nCode, wParam, lParam);
    }

    MSLLHOOKSTRUCT *info = (MSLLHOOKSTRUCT *)lParam;
    LONG  x = info->pt.x;
    LONG  y = info->pt.y;
    BYTE  frame[FRAME_TOTAL_LEN];
    int   send_frame = 0;
    int16_t dx = 0, dy = 0;
    int8_t  wheel = 0;
    const char *event_type = "";

    switch (wParam) {

    /* ── 鼠标移动 ── */
    case WM_MOUSEMOVE:
        if (g_first_move) {
            g_last_x = x;
            g_last_y = y;
            g_first_move = 0;
            break;
        }
        dx = (int16_t)(x - g_last_x);
        dy = (int16_t)(y - g_last_y);
        g_last_x = x;
        g_last_y = y;

        if (dx != 0 || dy != 0) {
            build_frame(frame, g_button_state, dx, dy, 0);
            send_frame = 1;
            event_type = "MOVE";
        }
        break;

    /* ── 左键 ── */
    case WM_LBUTTONDOWN:
        event_type = "L-DOWN";
        g_button_state |= BTN_MASK_LEFT;
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;
    case WM_LBUTTONUP:
        event_type = "L-UP";
        g_button_state &= (BYTE)(~BTN_MASK_LEFT);
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;

    /* ── 右键 ── */
    case WM_RBUTTONDOWN:
        event_type = "R-DOWN";
        g_button_state |= BTN_MASK_RIGHT;
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;
    case WM_RBUTTONUP:
        event_type = "R-UP";
        g_button_state &= (BYTE)(~BTN_MASK_RIGHT);
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;

    /* ── 中键 ── */
    case WM_MBUTTONDOWN:
        event_type = "M-DOWN";
        g_button_state |= BTN_MASK_MIDDLE;
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;
    case WM_MBUTTONUP:
        event_type = "M-UP";
        g_button_state &= (BYTE)(~BTN_MASK_MIDDLE);
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;

    /* ── 侧键 (XButton) ── */
    case WM_XBUTTONDOWN: {
        WORD xbutton = HIWORD(info->mouseData);
        if (xbutton == XBUTTON1) {
            event_type = "X1-DN";
            g_button_state |= BTN_MASK_SIDE1;
        } else if (xbutton == XBUTTON2) {
            event_type = "X2-DN";
            g_button_state |= BTN_MASK_SIDE2;
        }
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;
    }
    case WM_XBUTTONUP: {
        WORD xbutton = HIWORD(info->mouseData);
        if (xbutton == XBUTTON1) {
            event_type = "X1-UP";
            g_button_state &= (BYTE)(~BTN_MASK_SIDE1);
        } else if (xbutton == XBUTTON2) {
            event_type = "X2-UP";
            g_button_state &= (BYTE)(~BTN_MASK_SIDE2);
        }
        build_frame(frame, g_button_state, 0, 0, 0);
        send_frame = 1;
        break;
    }

    /* ── 垂直滚轮 ── */
    case WM_MOUSEWHEEL: {
        SHORT raw_delta = (SHORT)HIWORD(info->mouseData);
        wheel = (int8_t)(raw_delta / WHEEL_DELTA);  /* 转为"格"单位 */
        if (wheel == 0 && raw_delta != 0) {
            wheel = (raw_delta > 0) ? 1 : -1;       /* 不足一格也发 */
        }
        if (wheel != 0) {
            event_type = "WHEEL";
            build_frame(frame, g_button_state, 0, 0, wheel);
            send_frame = 1;
        }
        break;
    }

    /* ── 水平滚轮 (仅打印, 不编码) ── */
    case WM_MOUSEHWHEEL: {
        SHORT raw_delta = (SHORT)HIWORD(info->mouseData);
        int8_t hwheel = (int8_t)(raw_delta / WHEEL_DELTA);
        printf("  [水平滚轮] %+d 档 (暂不编码发送)\n", hwheel);
        break;
    }

    default:
        break;
    }

    /* 入队发送 */
    if (send_frame) {
        if (queue_push(&g_queue, frame)) {
            print_frame(frame, event_type, dx, dy, wheel);
        } else {
            /* 队列满, 丢弃 (高速移动时偶发) */
        }
    }

    return CallNextHookEx(NULL, nCode, wParam, lParam);
}

/* ===========================================================================
 * 串口发送线程
 *   从队列中取帧并通过串口发送
 * =========================================================================== */
static DWORD WINAPI serial_sender_thread(LPVOID lpParam)
{
    (void)lpParam;
    BYTE frame[FRAME_TOTAL_LEN];

    while (g_running) {
        /* 等待数据或超时 (100ms 超时以检查 g_running) */
        DWORD wait_result = WaitForSingleObject(g_queue.hDataReady, 100);

        if (wait_result == WAIT_OBJECT_0) {
            /* 批量消费队列中的所有帧 */
            while (queue_pop(&g_queue, frame)) {
                serial_send(frame, FRAME_TOTAL_LEN);
            }
        }
    }

    /* 退出前清空队列 */
    while (queue_pop(&g_queue, frame)) {
        serial_send(frame, FRAME_TOTAL_LEN);
    }

    return 0;
}

/* ===========================================================================
 * Ctrl+C 处理器
 * =========================================================================== */
static BOOL WINAPI ctrl_handler(DWORD fdwCtrlType)
{
    (void)fdwCtrlType;
    g_running = 0;
    /* 向消息循环发送退出消息 */
    PostQuitMessage(0);
    return TRUE;
}

/* ===========================================================================
 * 用户交互: 选择端口
 * =========================================================================== */
static int select_com_port(ComPortInfo *ports, int count, char *out_path, size_t path_size)
{
    if (count == 0) {
        printf("\n  未检测到可用 COM 端口!\n");
        return 0;
    }

    printf("\n  ┌─────────────────────────────────────────────────┐\n");
    printf(  "  │  可用 COM 端口列表                                │\n");
    printf(  "  ├────┬────────────────────────────────────────────┤\n");
    for (int i = 0; i < count; i++) {
        printf("  │ %2d │ COM%-3d  %-30s │\n",
               i + 1, ports[i].number, ports[i].desc);
    }
    printf(  "  └────┴────────────────────────────────────────────┘\n");

    printf("\n  请选择输出 COM 端口 (1-%d): ", count);

    char input[32];
    if (!fgets(input, sizeof(input), stdin)) return 0;

    int choice = atoi(input);
    if (choice < 1 || choice > count) {
        printf("  无效的选择!\n");
        return 0;
    }

    strncpy(out_path, ports[choice - 1].name, path_size - 1);
    out_path[path_size - 1] = '\0';
    return 1;
}

static DWORD select_baudrate(void)
{
    const DWORD baudrates[] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
    const char *labels[]    = { "9600", "19200", "38400", "57600", "115200",
                                "230400", "460800", "921600" };
    int num = sizeof(baudrates) / sizeof(baudrates[0]);

    printf("\n  ┌─────────────────────────┐\n");
    printf(  "  │  选择波特率              │\n");
    printf(  "  ├────┬────────────────────┤\n");
    for (int i = 0; i < num; i++) {
        printf("  │ %2d │ %-7s          │\n", i + 1, labels[i]);
    }
    printf(  "  └────┴────────────────────┘\n");

    printf("\n  请选择波特率 (1-%d, 默认 5=115200): ", num);

    char input[32];
    if (!fgets(input, sizeof(input), stdin)) return 115200;

    int choice = atoi(input);
    if (choice < 1 || choice > num) choice = 5;  /* 默认 115200 */

    return baudrates[choice - 1];
}

/* ===========================================================================
 * 主函数
 * =========================================================================== */
int main(void)
{
    /* ────────────────────────────────
     * 阶段1: 扫描设备
     * ──────────────────────────────── */
    printf("\n");
    printf("  ╔══════════════════════════════════════════════════╗\n");
    printf("  ║    鼠标数据采集 + 串口透传桥接程序  v1.0          ║\n");
    printf("  ║    Mouse Serial Bridge (Windows)                 ║\n");
    printf("  ╚══════════════════════════════════════════════════╝\n");

    printf("\n  [1/3] 正在扫描 COM 端口...\n");
    int com_count = 0;
    ComPortInfo *com_ports = enumerate_com_ports(&com_count);
    printf("  ── 检测到 %d 个可用 COM 端口\n", com_count);

    printf("\n  [2/3] 正在扫描 USB HID 鼠标设备...\n");
    int mouse_count = 0;
    MouseDeviceInfo *mouse_devices = enumerate_mouse_devices(&mouse_count);

    if (mouse_count > 0) {
        printf("  ┌────┬────────┬────────┬──────────────────────────────┐\n");
        printf("  │  # │  VID   │  PID   │  设备描述                     │\n");
        printf("  ├────┼────────┼────────┼──────────────────────────────┤\n");
        for (int i = 0; i < mouse_count; i++) {
            printf("  │ %2d │ 0x%04X │ 0x%04X │ %-28s │\n",
                   i + 1,
                   mouse_devices[i].vid,
                   mouse_devices[i].pid,
                   mouse_devices[i].desc);
        }
        printf("  └────┴────────┴────────┴──────────────────────────────┘\n");
        printf("\n  [注意] WH_MOUSE_LL 钩子会捕获所有鼠标的全局事件,\n"
               "         上述列表仅供参考, 无法按设备过滤。\n");
    } else {
        printf("  ── 未检测到鼠标设备 (系统钩子仍可工作)\n");
    }

    /* ────────────────────────────────
     * 阶段2: 用户选择
     * ──────────────────────────────── */
    printf("\n  [3/3] 配置输出端口\n");

    if (!select_com_port(com_ports, com_count, g_port_name, sizeof(g_port_name))) {
        printf("\n  无法选择 COM 端口, 程序退出。\n");
        free_com_ports(com_ports, com_count);
        free_mouse_devices(mouse_devices, mouse_count);
        return 1;
    }

    DWORD baudrate = select_baudrate();

    printf("\n");
    printf("  ╔══════════════════════════════════════════════════╗\n");
    printf("  ║  配置确认                                         ║\n");
    printf("  ╠══════════════════════════════════════════════════╣\n");
    printf("  ║  输出端口 : %-35s ║\n", g_port_name);
    printf("  ║  波特率   : %-6lu bps                          ║\n", baudrate);
    printf("  ║  协议     : A5 5A 10字节固定帧 (v3.0)            ║\n");
    printf("  ║  校验     : CRC-8-MAXIM                          ║\n");
    printf("  ╚══════════════════════════════════════════════════╝\n");

    /* ────────────────────────────────
     * 阶段3: 打开串口
     * ──────────────────────────────── */
    printf("\n  正在连接串口 %s @ %lu bps...\n", g_port_name, baudrate);

    if (!serial_open(g_port_name, baudrate)) {
        printf("  串口连接失败, 程序退出。\n");
        free_com_ports(com_ports, com_count);
        free_mouse_devices(mouse_devices, mouse_count);
        return 1;
    }
    printf("  串口已连接!\n");

    /* ────────────────────────────────
     * 阶段4: 初始化队列
     * ──────────────────────────────── */
    queue_init(&g_queue);
    g_start_tick = GetTickCount();

    /* ────────────────────────────────
     * 阶段5: 安装鼠标钩子
     * ──────────────────────────────── */
    printf("\n  正在安装全局鼠标钩子...\n");
    g_hMouseHook = SetWindowsHookEx(
        WH_MOUSE_LL,
        mouse_hook_proc,
        GetModuleHandle(NULL),
        0  /* 0 = 全局钩子 */
    );

    if (!g_hMouseHook) {
        printf("  [错误] 安装鼠标钩子失败 (错误码: %lu)\n", GetLastError());
        serial_close();
        queue_destroy(&g_queue);
        free_com_ports(com_ports, com_count);
        free_mouse_devices(mouse_devices, mouse_count);
        return 1;
    }
    printf("  钩子安装成功!\n");

    /* ────────────────────────────────
     * 阶段6: 启动发送线程
     * ──────────────────────────────── */
    HANDLE hSenderThread = CreateThread(
        NULL, 0, serial_sender_thread, NULL, 0, NULL
    );
    if (!hSenderThread) {
        printf("  [错误] 创建发送线程失败\n");
        UnhookWindowsHookEx(g_hMouseHook);
        serial_close();
        queue_destroy(&g_queue);
        free_com_ports(com_ports, com_count);
        free_mouse_devices(mouse_devices, mouse_count);
        return 1;
    }

    /* ────────────────────────────────
     * 阶段7: 注册 Ctrl+C 处理器
     * ──────────────────────────────── */
    SetConsoleCtrlHandler(ctrl_handler, TRUE);

    /* ────────────────────────────────
     * 阶段8: 启动信息
     * ──────────────────────────────── */
    printf("\n");
    printf("  ╔══════════════════════════════════════════════════╗\n");
    printf("  ║  正在监听鼠标事件...                              ║\n");
    printf("  ║  按 Ctrl+C 退出                                   ║\n");
    printf("  ╚══════════════════════════════════════════════════╝\n\n");

    printf("  时间          | 事件    | 按键    | 位移(X,Y)     | 滚轮 | 帧 (HEX)\n");
    printf("  ──────────────┼─────────┼─────────┼───────────────┼──────┼───────────────────────────\n");

    /* ────────────────────────────────
     * 阶段9: 消息循环 (保持钩子活跃)
     * ──────────────────────────────── */
    MSG msg;
    while (g_running && GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    /* ────────────────────────────────
     * 阶段10: 清理
     * ──────────────────────────────── */
    printf("\n\n  正在退出...\n");

    g_running = 0;

    /* 通知发送线程退出 */
    SetEvent(g_queue.hDataReady);

    /* 等待发送线程 */
    WaitForSingleObject(hSenderThread, 3000);
    CloseHandle(hSenderThread);

    /* 卸载钩子 */
    if (g_hMouseHook) {
        UnhookWindowsHookEx(g_hMouseHook);
        g_hMouseHook = NULL;
    }

    /* 关闭串口 */
    serial_close();

    /* 销毁队列 */
    queue_destroy(&g_queue);

    /* 释放设备列表 */
    free_com_ports(com_ports, com_count);
    free_mouse_devices(mouse_devices, mouse_count);

    printf("  程序已退出。\n\n");
    return 0;
}
