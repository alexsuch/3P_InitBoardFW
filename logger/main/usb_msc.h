// TinyUSB MSC descriptors are only needed when USB file server is enabled.
// Guard the entire header so builds without USE_USB_FILE_SERVER don't require TinyUSB.

#pragma once

#include "target.h"

#if defined(USE_USB_FILE_SERVER)

#include <stddef.h>

// Define MSC constants before including TinyUSB headers
// These must be defined before tusb.h includes usbd.h
// Note: Espressif's tusb_config.h will also define these, causing harmless redefinition warnings
// We suppress those warnings below
#ifndef CFG_TUD_MSC
#define CFG_TUD_MSC 1
#endif

// Increase MSC buffering for higher throughput
#ifndef CFG_TUD_MSC_BUFSIZE
#define CFG_TUD_MSC_BUFSIZE 8192
#endif

// Force-disable all other device classes.
// Otherwise TinyUSB will try to open endpoints for them on SET_CONFIG
// and will assert because our config descriptor contains only MSC.
#ifndef CFG_TUD_CDC
#define CFG_TUD_CDC 0
#endif
#ifndef CFG_TUD_HID
#define CFG_TUD_HID 0
#endif
#ifndef CFG_TUD_MIDI
#define CFG_TUD_MIDI 0
#endif
#ifndef CFG_TUD_VENDOR
#define CFG_TUD_VENDOR 0
#endif
#ifndef CFG_TUD_DFU_RUNTIME
#define CFG_TUD_DFU_RUNTIME 0
#endif
#ifndef CFG_TUD_NET
#define CFG_TUD_NET 0
#endif
#ifndef CFG_TUD_AUDIO
#define CFG_TUD_AUDIO 0
#endif

// Note: MSC_SUBCLASS_SCSI and MSC_PROTOCOL_BOT are defined as enum values in TinyUSB's msc.h
// Do not redefine them as macros here

// Espressif's tusb_config.h will redefine these macros based on CONFIG_TINYUSB_* values
// To avoid redefinition warnings, we undefine them here and let the config header define them
// The config header is included through tusb.h
#undef CFG_TUD_MSC
#undef CFG_TUD_CDC
#undef CFG_TUD_HID
#undef CFG_TUD_MIDI
#undef CFG_TUD_VENDOR
#undef CFG_TUD_DFU_RUNTIME

#include "tusb.h"
// Ensure MSC enum values are available for TUD_MSC_DESCRIPTOR macro
#include "class/msc/msc_device.h"

// Force our configuration values after tusb_config.h has defined them
// This ensures MSC is enabled and other classes are disabled to match our descriptor
#undef CFG_TUD_MSC
#define CFG_TUD_MSC 1
#undef CFG_TUD_CDC
#define CFG_TUD_CDC 0
#undef CFG_TUD_HID
#define CFG_TUD_HID 0
#undef CFG_TUD_MIDI
#define CFG_TUD_MIDI 0
#undef CFG_TUD_VENDOR
#define CFG_TUD_VENDOR 0
#undef CFG_TUD_DFU_RUNTIME
#define CFG_TUD_DFU_RUNTIME 0

extern const tusb_desc_device_t g_usb_desc_device;
extern const uint8_t g_usb_desc_fs_config[];
extern const uint8_t g_usb_desc_hs_config[];
extern const char *g_usb_string_desc_arr[];
extern const size_t g_usb_string_desc_count;

#endif  // USE_USB_FILE_SERVER
