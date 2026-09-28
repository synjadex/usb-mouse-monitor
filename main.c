/**
 * @file        main.c
 * @author      SynJade
 * @date        2026-09-27
 * @brief
 * @version     0.1
 */
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>
#include <stdio.h>
#include <hidpi.h>
#include <stdlib.h>


/**
 * 把 UsagePage / Usage 翻译成可读名字
 * @param page
 * @param usage
 * @return
 */
const char* HidTypeName(USHORT page, USHORT usage) {
    if (page == 0x01) {
        switch (usage) {
            case 0x01: { return "指针/指针设备"; }
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
    if (page == 0x0C) return "消费者控制(音量/媒体键等)";
    if (page == 0x0D) return "物理传感设备";
    if (page == 0x0B) return "游戏设备";
    if (page == 0xFF00) return "厂商自定义";
    return "未知类型";
}

int main() {
    GUID hid_guid;
    HidD_GetHidGuid(&hid_guid);

    HDEVINFO h = SetupDiGetClassDevsW(&hid_guid, NULL, NULL,
                                      DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (h == INVALID_HANDLE_VALUE) {
        printf("INVALID_HANDLE_VALUE!\n");
        return 1;
    }

    SP_DEVICE_INTERFACE_DATA did = { sizeof(did) };
    size_t hid_count = 0;

    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(h, NULL, &hid_guid, i, &did); i++) {
        hid_count++;

        DWORD size = 0;
        SetupDiGetDeviceInterfaceDetailW(h, &did, NULL, 0, &size, NULL);

        SP_DEVICE_INTERFACE_DETAIL_DATA_W *detail = malloc(size);
        if (!detail) continue;
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);

        if (SetupDiGetDeviceInterfaceDetailW(h, &did, detail, size, NULL, NULL)) {

            printf("[%zu] %ls\n", hid_count, detail->DevicePath);

            // ---- 新增部分：打开设备，读 Usage ----
            // 用 0 权限打开，避免鼠标/键盘被系统独占时打不开
            HANDLE dev = CreateFileW(detail->DevicePath, 0,
                                     FILE_SHARE_READ | FILE_SHARE_WRITE,
                                     NULL, OPEN_EXISTING, 0, NULL);
            if (dev == INVALID_HANDLE_VALUE) {
                printf("      (无法打开，错误码 %lu)\n\n", GetLastError());
            } else {
                PHIDP_PREPARSED_DATA preparsed;
                HIDP_CAPS caps;

                if (HidD_GetPreparsedData(dev, &preparsed)) {
                    if (HidP_GetCaps(preparsed, &caps) == HIDP_STATUS_SUCCESS) {
                        // 关键信息在这里
                        printf("      UsagePage=0x%04X  Usage=0x%04X -> %s\n",
                               caps.UsagePage, caps.Usage,
                               HidTypeName(caps.UsagePage, caps.Usage));
                        printf("      输入报告长度=%u  输出报告长度=%u  特征报告长度=%u\n",
                               caps.InputReportByteLength,
                               caps.OutputReportByteLength,
                               caps.FeatureReportByteLength);
                    } else {
                        printf("      HidP_GetCaps 失败\n");
                    }
                    HidD_FreePreparsedData(preparsed);
                } else {
                    printf("      HidD_GetPreparsedData 失败\n");
                }

                // 顺手读一下 VID/PID，方便对照外接设备
                HIDD_ATTRIBUTES attr = { sizeof(attr) };
                if (HidD_GetAttributes(dev, &attr)) {
                    printf("      VID=0x%04X  PID=0x%04X\n",
                           attr.VendorID, attr.ProductID);
                }
                printf("\n");

                CloseHandle(dev);
            }
            // ---- 新增部分结束 ----
        }
        free(detail);
    }

    SetupDiDestroyDeviceInfoList(h);
    printf("找到 %zu 个HID设备接口\n", hid_count);
    return 0;
}