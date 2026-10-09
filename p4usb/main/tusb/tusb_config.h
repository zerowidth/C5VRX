#pragma once

/* The P4's high-speed OTG controller is TinyUSB's root hub port 1. */
#define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_OS_INC_PATH freertos/
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_ENABLED 1
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 1
/* Large enough to hold a whole esptool write block while the bridge forwards it to the C5. */
#define CFG_TUD_CDC_RX_BUFSIZE 8192
#define CFG_TUD_CDC_TX_BUFSIZE 8192

#define CFG_TUD_VIDEO 1
#define CFG_TUD_VIDEO_STREAMING 1
/* The video class sends one payload per transfer, so 512-byte payloads cap a 64 KB frame near 38 fps. */
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE 16384
