#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>
#include <hidpi.h>
#include <stdio.h>
#include <stdlib.h>

/**
 *  Usage 翻译
 * @param page
 * @param usage
 * @return
 */
static const char* HidTypeName(USHORT page, USHORT usage)
{
    if (page == 0x01) {
        switch (usage) {
            case 0x01: return "指针设备";
            case 0x02: return "鼠标";
            case 0x04: return "摇杆";
            case 0x05: return "游戏手柄";
            case 0x06: return "键盘";
            case 0x07: return "小键盘";
            case 0x08: return "多轴控制器";
            case 0x80: return "系统控制";
            default:   return "通用桌面设备(其他)";
        }
    }
    if (page == 0x0B) return "游戏设备";
    if (page == 0x0C) return "消费者控制(音量/媒体键)";
    if (page == 0x0D) return "物理传感设备(触摸/手写)";
    if (page >= 0xFF00) return "厂商自定义";
    return "未知类型";
}

/**
 * 打印单个设备的详细信息
 * @param devicePath 设备路径
 */
static void PrintDeviceInfo(const wchar_t *devicePath)
{
    // 权限打开，避免被系统独占的鼠标/键盘打不开
    HANDLE dev = CreateFileW(devicePath, 0,
                             FILE_SHARE_READ | FILE_SHARE_WRITE,
                             NULL, OPEN_EXISTING, 0, NULL);
    if (dev == INVALID_HANDLE_VALUE) {
        printf("      (无法打开，错误码 %lu)\n", GetLastError());
        return;
    }

    // --- 读取 Usage ---
    PHIDP_PREPARSED_DATA preparsed = NULL;
    if (HidD_GetPreparsedData(dev, &preparsed)) {
        HIDP_CAPS caps;
        if (HidP_GetCaps(preparsed, &caps) == HIDP_STATUS_SUCCESS) {
            printf("      UsagePage=0x%04X  Usage=0x%04X -> %s\n",
                   caps.UsagePage, caps.Usage,
                   HidTypeName(caps.UsagePage, caps.Usage));
            printf("      报告长度: 输入=%u 输出=%u 特征=%u\n",
                   caps.InputReportByteLength,
                   caps.OutputReportByteLength,
                   caps.FeatureReportByteLength);
        }
        HidD_FreePreparsedData(preparsed);
    }

    // --- 读取 VID / PID ---
    HIDD_ATTRIBUTES attr = { sizeof(attr) };
    if (HidD_GetAttributes(dev, &attr)) {
        printf("      VID=0x%04X  PID=0x%04X\n",
               attr.VendorID, attr.ProductID);
    }

    CloseHandle(dev);
}


/**
 * 取单个设备的路径
 * @param h
 * @param did
 * @return 成功返回 malloc 出来的路径字符串，调用者负责 free；
 *  失败返回 NULL。
 */
static wchar_t* GetDevicePath(HDEVINFO h, SP_DEVICE_INTERFACE_DATA *did)
{
    DWORD size = 0;
    SetupDiGetDeviceInterfaceDetailW(h, did, NULL, 0, &size, NULL);
    if (size == 0) return NULL;

    SP_DEVICE_INTERFACE_DETAIL_DATA_W *detail = malloc(size);
    if (!detail) return NULL;
    detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

    wchar_t *path = NULL;
    if (SetupDiGetDeviceInterfaceDetailW(h, did, detail, size, NULL, NULL)) {
        size_t len = wcslen(detail->DevicePath) + 1;
        path = malloc(len * sizeof(wchar_t));
        if (path) wcscpy_s(path, len, detail->DevicePath);
    }
    free(detail);
    return path;
}

/**
 * 枚举主流程
 * @return 设备数量
 */
static size_t EnumerateHidDevices(void)
{
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);

    HDEVINFO h = SetupDiGetClassDevsW(&hidGuid, NULL, NULL,
                                      DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (h == INVALID_HANDLE_VALUE) {
        printf("SetupDiGetClassDevsW 失败，错误码 %lu\n", GetLastError());
        return 0;
    }

    SP_DEVICE_INTERFACE_DATA did = { sizeof(did) };
    size_t count = 0;

    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(h, NULL, &hidGuid, i, &did); i++) {
        wchar_t *path = GetDevicePath(h, &did);
        if (!path) continue;

        count++;
        printf("[%zu] %ls\n", count, path);
        PrintDeviceInfo(path);
        printf("\n");

        free(path);
    }

    SetupDiDestroyDeviceInfoList(h);
    return count;
}

/* ========== 5. main ========== */

int main(void)
{
    size_t n = EnumerateHidDevices();
    printf("共找到 %zu 个 HID 设备接口\n", n);
    return 0;
}