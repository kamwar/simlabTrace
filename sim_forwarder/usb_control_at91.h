#ifndef _USB_CONTROL_AT91_H_
#define _USB_CONTROL_AT91_H_

#include <stdint.h>
#include <usb/common/core/USBGenericRequest.h>

void usb_control_init(void);
void usb_control_poll(void);
void usb_control_vendor_request(const USBGenericRequest *request);

#endif
