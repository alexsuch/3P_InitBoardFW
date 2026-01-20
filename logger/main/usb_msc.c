#include "usb_msc.h"

#if defined(USE_USB_FILE_SERVER)

static const char *TAG = "USB_MSC";

// ---------------- Descriptors ----------------

enum { ITF_NUM_MSC = 0, ITF_NUM_TOTAL };

#define EPNUM_MSC_OUT 0x01
#define EPNUM_MSC_IN 0x81
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

// Device descriptor
const tusb_desc_device_t g_usb_desc_device = {.bLength = sizeof(tusb_desc_device_t),
                                              .bDescriptorType = TUSB_DESC_DEVICE,
                                              .bcdUSB = 0x0200,

                                              .bDeviceClass = 0x00,
                                              .bDeviceSubClass = 0x00,
                                              .bDeviceProtocol = 0x00,

                                              .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

                                              // You can change VID/PID to your own
                                              .idVendor = 0x303A,   // Espressif VID
                                              .idProduct = 0x4007,  // arbitrary PID
                                              .bcdDevice = 0x0100,

                                              .iManufacturer = 0x01,
                                              .iProduct = 0x02,
                                              .iSerialNumber = 0x03,

                                              .bNumConfigurations = 0x01};

// Full-speed configuration descriptor
const uint8_t g_usb_desc_fs_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0xA0, 100),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0x04, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

// High-speed configuration descriptor
// For HS bulk endpoints max packet is 512 bytes
const uint8_t g_usb_desc_hs_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0xA0, 100),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0x04, EPNUM_MSC_OUT, EPNUM_MSC_IN, 512),
};

// String descriptors (UTF-8)
const char *g_usb_string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: English (0x0409)
    "HOOTRONICS",                // 1: Manufacturer
    "LOGGER_CARD",               // 2: Product
    "0001",                      // 3: Serial
    "MSC Storage",               // 4: Interface string (used by iInterface=0x04)
};

const size_t g_usb_string_desc_count = sizeof(g_usb_string_desc_arr) / sizeof(g_usb_string_desc_arr[0]);

// Note: When using esp-usb's tinyusb_msc_new_storage_sdmmc() API, the library
// automatically provides all the TinyUSB MSC callbacks (tud_msc_*_cb functions).
// Do not implement these callbacks directly here, as they will conflict with
// the library's implementations.

#endif  // USE_USB_FILE_SERVER
