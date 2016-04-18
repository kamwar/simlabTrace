#ifndef _USB_DFU_DESC_H
#define _USB_DFU_DESC_H

#include <board.h>
#include <usb/common/core/USBGenericDescriptor.h>

#define CONFIG_DFU_NUM_APP_IF    0
#define CONFIG_DFU_NUM_APP_STR    4

struct USBStringDescriptor {
    USBGenericDescriptor hdr;
    unsigned short wData[];
} __attribute__((packed));


#ifdef BOARD_USB_DFU

#define DFU_NUM_IF    1

#define DFU_IF_DESCRIPTORS_STRUCT        \
    USBInterfaceDescriptor    dfu_interface[DFU_NUM_IF];

#define DFU_IF_DESCRIPTORS            {                 \
    {                                    \
        .bLength         = sizeof(USBInterfaceDescriptor),    \
        .bDescriptorType    = USBGenericDescriptor_INTERFACE,    \
        .bInterfaceNumber    = CONFIG_DFU_NUM_APP_IF,        \
        .bAlternateSetting    = 0,                    \
        .bNumEndpoints        = 0,                    \
        .bInterfaceClass    = 0xFE,                    \
        .bInterfaceSubClass    = 0x01,                    \
        .bInterfaceProtocol    = 0x01,                    \
        .iInterface        = CONFIG_DFU_NUM_APP_STR,        \
    },                                    \
},
#if 0
    {                                    \
        .bLength         = sizeof(USBInterfaceDescriptor),    \
        .bDescriptorType    = USBGenericDescriptor_INTERFACE,    \
        .bInterfaceNumber    = CONFIG_DFU_NUM_APP_IF+1,        \
        .bAlternateSetting    = 0,                    \
        .bNumEndpoints        = 0,                    \
        .bInterfaceClass    = 0xFE,                    \
        .bInterfaceSubClass    = 0x01,                    \
        .bInterfaceProtocol    = 0x01,                    \
        .iInterface        = CONFIG_DFU_NUM_APP_STR+1,        \
    },                                    \
    {                                    \
        .bLength         = sizeof(USBInterfaceDescriptor),    \
        .bDescriptorType    = USBGenericDescriptor_INTERFACE,    \
        .bInterfaceNumber    = CONFIG_DFU_NUM_APP_IF+2,        \
        .bAlternateSetting    = 0,                    \
        .bNumEndpoints        = 0,                    \
        .bInterfaceClass    = 0xFE,                    \
        .bInterfaceSubClass    = 0x01,                    \
        .bInterfaceProtocol    = 0x01,                    \
        .iInterface        = CONFIG_DFU_NUM_APP_STR+2,        \
    },                                    \
},
#endif

extern const struct USBStringDescriptor USBDFU_string1;
extern const struct USBStringDescriptor USBDFU_string2;
extern const struct USBStringDescriptor USBDFU_string3;

#define DFU_NUM_STRINGS    3
#define DFU_STRING_DESCRIPTORS    \
    (const unsigned char *) &USBDFU_string1,    \
    (const unsigned char *) &USBDFU_string2,    \
    (const unsigned char *) &USBDFU_string3,

#else /* BOARD_USB_DFU */

/* no DFU bootloader is being used */
#define DFU_NUM_IF    0
#define DFU_IF_DESCRIPTORS_STRUCT
#define DFU_IF_DESCRIPTORS

#define DFU_NUM_STRINGS    0
#define DFU_STRING_DESCRIPTORS

#endif /* BOARD_USB_DFU */

#ifdef BOARD_USB_DFU
/*
 * This is a very minimal implementation of DFU procedure.
 * Interaction between App and OpenPCD DFU bootloader is done based on
 * shared data/code (API).
 * Please refer to the OpenPCD/SIMtrace project for more details:
 * git://git.gnumonks.org/openpcd.git
 *
*/

#define USB_TYPE_DFU  ((1<<5) | 0x01)    /* (USB_TYPE_CLASS|USB_RECIP_INTERFACE) */

#define DFU_API_LOCATION    ((const struct dfuapi *) 0x00103fd0)
static const struct dfuapi *dfu = DFU_API_LOCATION;

struct dfuapi {
    void(*udp_init)(void);    /* unsued */
    void(*ep0_send_data)(const char *data, unsigned int len, unsigned int wlen);  /* unsued */
    void(*ep0_send_zlp)(void);  /* unsued */
    void(*ep0_send_stall)(void);  /* unsued */
    int(*dfu_ep0_handler)(unsigned char req_type, unsigned char req,
        short int val, short int len);
    void(*dfu_switch)(void);
    unsigned int *dfu_state;
    //const struct usb_device_descriptor *dfu_dev_descriptor;
    //const struct _dfu_desc *dfu_cfg_descriptor;
};

enum dfu_state {
    DFU_STATE_appIDLE = 0,
    DFU_STATE_appDETACH = 1,
    DFU_STATE_dfuIDLE = 2,
    DFU_STATE_dfuDNLOAD_SYNC = 3,
    DFU_STATE_dfuDNBUSY = 4,
    DFU_STATE_dfuDNLOAD_IDLE = 5,
    DFU_STATE_dfuMANIFEST_SYNC = 6,
    DFU_STATE_dfuMANIFEST = 7,
    DFU_STATE_dfuMANIFEST_WAIT_RST = 8,
    DFU_STATE_dfuUPLOAD_IDLE = 9,
    DFU_STATE_dfuERROR = 10,
};
#endif

#endif
