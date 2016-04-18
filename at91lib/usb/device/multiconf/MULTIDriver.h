#ifndef MULTIDRIVER_H
#define MULTIDRIVER_H

#include <usb/common/core/USBGenericRequest.h>
#include <usb/device/core/USBD.h>

#include "../composite/CDCDFunctionDriver.h"

extern void MULTIDriver_RequestHandler(const USBGenericRequest *request);

extern void MULTIDriver_RemoteWakeUp(void);

extern void MULTIDriver_Initialize(void);

#endif //#ifndef MULTIDDRIVER_H
