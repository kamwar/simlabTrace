/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//      Headers
//------------------------------------------------------------------------------

#include "MULTIDriver.h"
#include "MULTIDriverDescriptors.h"
#include <board.h>

//- USB Generic
#include <usb/common/core/USBGenericDescriptor.h>
#include <usb/common/core/USBConfigurationDescriptor.h>
#include <usb/common/core/USBInterfaceAssociationDescriptor.h>
#include <usb/common/core/USBEndpointDescriptor.h>
#include <usb/common/core/USBStringDescriptor.h>
#include <usb/common/core/USBGenericRequest.h>

//- CDC
#include <usb/common/cdc/CDCGenericDescriptor.h>
#include <usb/common/cdc/CDCDeviceDescriptor.h>
#include <usb/common/cdc/CDCCommunicationInterfaceDescriptor.h>
#include <usb/common/cdc/CDCDataInterfaceDescriptor.h>
#include <usb/common/cdc/CDCHeaderDescriptor.h>
#include <usb/common/cdc/CDCCallManagementDescriptor.h>
#include <usb/common/cdc/CDCAbstractControlManagementDescriptor.h>
#include <usb/common/cdc/CDCUnionDescriptor.h>
#include "../composite/CDCDFunctionDriverDescriptors.h"

//CCID
#include <usb/device/ccid/cciddriver.h>
#include <usb/device/ccid/cciddriverdescriptors.h>

//-----------------------------------------------------------------------------
//         Definitions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//         Macros
//-----------------------------------------------------------------------------

/// Returns the minimum between two values.
#define MIN(a, b)       ((a < b) ? a : b)

//-----------------------------------------------------------------------------
//         Internal structures
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Audio control header descriptor with one slave interface.
//-----------------------------------------------------------------------------
#ifdef __ICCARM__          // IAR
#pragma pack(1)            // IAR
#define __attribute__(...) // IAR
#endif                     // IAR

//-----------------------------------------------------------------------------
/// Configuration descriptor list for a device implementing a composite driver.
//-----------------------------------------------------------------------------
struct multi_cdc_conf_desc {
	/// Standard configuration descriptor.
	USBConfigurationDescriptor configuration;

	/// IAD 0
	USBInterfaceAssociationDescriptor cdcIAD0;
	/// Communication interface descriptor
	USBInterfaceDescriptor cdcCommunication0;
	/// CDC header functional descriptor.
	CDCHeaderDescriptor cdcHeader0;
	/// CDC call management functional descriptor.
	CDCCallManagementDescriptor cdcCallManagement0;
	/// CDC abstract control management functional descriptor.
	CDCAbstractControlManagementDescriptor cdcAbstractControlManagement0;
	/// CDC union functional descriptor (with one slave interface).
	CDCUnionDescriptor cdcUnion0;
	/// Notification endpoint descriptor.
	USBEndpointDescriptor cdcNotification0;
	/// Data interface descriptor.
	USBInterfaceDescriptor cdcData0;
	/// Data OUT endpoint descriptor.
	USBEndpointDescriptor cdcDataOut0;
	/// Data IN endpoint descriptor.
	USBEndpointDescriptor cdcDataIn0;
} __attribute__ ((packed));

struct multi_ccid_conf_desc {
	USBConfigurationDescriptor configuration;
	USBInterfaceDescriptor interface;
	CCIDDescriptor ccid;
	USBEndpointDescriptor endpoint[3];
} __attribute__ ((packed));

#ifdef __ICCARM__          // IAR
#pragma pack()             // IAR
#endif                     // IAR

//------------------------------------------------------------------------------
//         Exported variables
//------------------------------------------------------------------------------

/// Standard USB device descriptor for the composite device driver
const USBDeviceDescriptor deviceDescriptor = {
	.bLength		= sizeof(USBDeviceDescriptor),
	.bDescriptorType	= USBGenericDescriptor_DEVICE,
	.bcdUSB			= USBDeviceDescriptor_USB2_00,
	.bDeviceClass		= 0x00,
	.bDeviceSubClass	= 0x00,
	.bDeviceProtocol	= 0x00,
	.bMaxPacketSize0	= BOARD_USB_ENDPOINTS_MAXPACKETSIZE(0),
	.idVendor		= 0x16c0,
	.idProduct		= 0x0762,
	.bcdDevice		= 0x0090,
    	.iManufacturer		= 1,
	.iProduct		= 2,
	.iSerialNumber		= 0,
    	.bNumConfigurations	= 3,
};

#if defined(BOARD_USB_UDPHS)

/// USB device qualifier descriptor.
const USBDeviceQualifierDescriptor qualifierDescriptor = {

    sizeof(USBDeviceQualifierDescriptor),
    USBGenericDescriptor_DEVICEQUALIFIER,
    USBDeviceDescriptor_USB2_00,
  #if defined(usb_HIDMSD)
    0x00,
    0x00,
    0x00,
  #else
    0xEF,// MI
    0x02,//
    0x01,//
  #endif
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(0),
    3, // Device has one possible configuration
    0 // Reserved
};

#endif

/* Configuration Descriptor for config #0 (SNIFFER) */
const struct multi_cdc_conf_desc sniffer_configurationDescriptors = {
	.configuration = {
		.bLength 		= sizeof(USBConfigurationDescriptor),
		.bDescriptorType	= USBGenericDescriptor_CONFIGURATION,
		.wTotalLength		= sizeof(struct multi_cdc_conf_desc),
		.bNumInterfaces		= 2,
		.bConfigurationValue	= 1,
		.iConfiguration		= 0,
		.bmAttributes		= BOARD_USB_BMATTRIBUTES,
		.bMaxPower		= USBConfigurationDescriptor_POWER(100),
	},
	.cdcIAD0 = {
		.bLength		= sizeof(USBInterfaceAssociationDescriptor),
		.bDescriptorType	= USBGenericDescriptor_INTERFACEASSOCIATION,
		.bFirstInterface	= 0,
		.bInterfaceCount	= 2,
		.bFunctionClass		= CDCCommunicationInterfaceDescriptor_CLASS,
		.bFunctionSubClass	= CDCCommunicationInterfaceDescriptor_ABSTRACTCONTROLMODEL,
		.bFunctionProtocol	= CDCCommunicationInterfaceDescriptor_NOPROTOCOL,
		.iFunction		= 0,
	},
	.cdcCommunication0 = {
		.bLength		= sizeof(USBInterfaceDescriptor),
		.bDescriptorType	= USBGenericDescriptor_INTERFACE,
		.bInterfaceNumber	= 0,
		.bAlternateSetting	= 0,
		.bNumEndpoints		= 1,
		.bInterfaceClass	= CDCCommunicationInterfaceDescriptor_CLASS,
		.bInterfaceSubClass	= CDCCommunicationInterfaceDescriptor_ABSTRACTCONTROLMODEL,
		.bInterfaceProtocol	= CDCCommunicationInterfaceDescriptor_NOPROTOCOL,
		.iInterface		= 0,
	},
	.cdcHeader0 = {
		.bFunctionLength	= sizeof(CDCHeaderDescriptor),
		.bDescriptorType	= CDCGenericDescriptor_INTERFACE,
		.bDescriptorSubtype	= CDCGenericDescriptor_HEADER,
		.bcdCDC			= CDCGenericDescriptor_CDC1_10,
	},
	.cdcCallManagement0 = {
		.bFunctionLength	= sizeof(CDCCallManagementDescriptor),
		.bDescriptorType	= CDCGenericDescriptor_INTERFACE,
		.bDescriptorSubtype	= CDCGenericDescriptor_CALLMANAGEMENT,
		.bmCapabilities		= CDCCallManagementDescriptor_SELFCALLMANAGEMENT,
		.bDataInterface		= 1,
	},
	.cdcAbstractControlManagement0 = {
		.bFunctionLength	= sizeof(CDCAbstractControlManagementDescriptor),
		.bDescriptorType	= CDCGenericDescriptor_INTERFACE,
		.bDescriptorSubtype	= CDCGenericDescriptor_ABSTRACTCONTROLMANAGEMENT,
		.bmCapabilities		= CDCAbstractControlManagementDescriptor_LINE,
	},
	.cdcUnion0 = {
		.bFunctionLength	= sizeof(CDCUnionDescriptor),
		.bDescriptorType	= CDCGenericDescriptor_INTERFACE,
		.bDescriptorSubtype	= CDCGenericDescriptor_UNION,
		.bMasterInterface	= 0,
		.bSlaveInterface0	= 1,
	},
	.cdcNotification0 = {
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
									CDCD_Descriptors_NOTIFICATION0),
		.bmAttributes		= USBEndpointDescriptor_INTERRUPT,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCD_Descriptors_NOTIFICATION0),
					      USBEndpointDescriptor_MAXINTERRUPTSIZE_FS),
		.bInterval		= 10,
	},
	.cdcData0 = {
		.bLength		= sizeof(USBInterfaceDescriptor),
		.bDescriptorType	= USBGenericDescriptor_INTERFACE,
		.bInterfaceNumber	= 1,
		.bAlternateSetting	= 0,
		.bNumEndpoints		= 2,
		.bInterfaceClass	= CDCDataInterfaceDescriptor_CLASS,
		.bInterfaceSubClass	= CDCDataInterfaceDescriptor_SUBCLASS,
		.bInterfaceProtocol	= CDCDataInterfaceDescriptor_NOPROTOCOL,
		.iInterface		= 0,
	},
	.cdcDataOut0 = {
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
									CDCD_Descriptors_DATAOUT0),
		.bmAttributes		= USBEndpointDescriptor_BULK,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCD_Descriptors_DATAOUT0),
					      USBEndpointDescriptor_MAXBULKSIZE_FS),
		.bInterval		= 0,
	},
	.cdcDataIn0 = {
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
									CDCD_Descriptors_DATAIN0),
		.bmAttributes		= USBEndpointDescriptor_BULK,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCD_Descriptors_DATAIN0),
					      USBEndpointDescriptor_MAXBULKSIZE_FS),
		.bInterval		= 0,
	},
};

const struct multi_ccid_conf_desc reader_configurationDescriptors = {
	.configuration = {
		.bLength 		= sizeof(USBConfigurationDescriptor),
		.bDescriptorType	= USBGenericDescriptor_CONFIGURATION,
		.wTotalLength		= sizeof(struct multi_ccid_conf_desc),
		.bNumInterfaces		= 1,
		.bConfigurationValue	= 2,
		.iConfiguration		= 0,
		.bmAttributes		= BOARD_USB_BMATTRIBUTES,
		.bMaxPower		= USBConfigurationDescriptor_POWER(100),
	},
	.interface = {
		.bLength		= sizeof(USBInterfaceDescriptor),
		.bDescriptorType	= USBGenericDescriptor_INTERFACE,
		.bInterfaceNumber	= 0,
		.bAlternateSetting	= 0,
		.bNumEndpoints		= 3,
		.bInterfaceClass	= SMART_CARD_DEVICE_CLASS,
		.bInterfaceSubClass	= 0,
		.bInterfaceProtocol	= 0,
		.iInterface		= 0,
	},
	.ccid = {
		.bLength		= sizeof(CCIDDescriptor),
		.bDescriptorType	= CCID_DECRIPTOR_TYPE,
		.bcdCCID		= CCID1_10,
		.bMaxSlotIndex		= 0,
		.bVoltageSupport	= VOLTS_3_0,
		.dwProtocols		= (1 << PROTOCOL_TO),
		.dwDefaultClock		= 3580,
		.dwMaximumClock		= 3580,
		.bNumClockSupported	= 0,
		.dwDataRate		= 9600,
		.dwMaxDataRate		= 9600,
		.bNumDataRatesSupported	= 0,
		.dwMaxIFSD		= 0xfe,
		.dwSynchProtocols	= 0,
		.dwMechanical		= 0,
		.dwFeatures		= CCID_FEATURES_AUTO_CLOCK | CCID_FEATURES_AUTO_BAUD |
					  CCID_FEATURES_AUTO_PCONF | CCID_FEATURES_AUTO_PNEGO |
					  CCID_FEATURES_EXC_TPDU,
		.dwMaxCCIDMessageLength	= 0x10f,
		.bClassGetResponse	= 0xff,
		.bClassEnvelope		= 0xff,
		.wLcdLayout		= 0,
		.bPINSupport		= 0,
		.bMaxCCIDBusySlots	= 1,
	},
	.endpoint = {
		{
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
									CCID_EPT_DATA_OUT),
		.bmAttributes		= USBEndpointDescriptor_BULK,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CCID_EPT_DATA_OUT),
					      USBEndpointDescriptor_MAXINTERRUPTSIZE_FS),
		.bInterval		= 0,
		},
		{
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
									CCID_EPT_DATA_IN),
		.bmAttributes		= USBEndpointDescriptor_BULK,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CCID_EPT_DATA_IN),
					      USBEndpointDescriptor_MAXINTERRUPTSIZE_FS),
		.bInterval		= 0,
		},
		{
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
									CCID_EPT_NOTIFICATION),
		.bmAttributes		= USBEndpointDescriptor_INTERRUPT,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CCID_EPT_NOTIFICATION),
					      USBEndpointDescriptor_MAXINTERRUPTSIZE_FS),
		.bInterval		= 0x10,
		},
	},
};

const struct multi_ccid_conf_desc mitm_configurationDescriptors = {
	.configuration = {
		.bLength 		= sizeof(USBConfigurationDescriptor),
		.bDescriptorType	= USBGenericDescriptor_CONFIGURATION,
		.wTotalLength		= sizeof(struct multi_ccid_conf_desc),
		.bNumInterfaces		= 1,
		.bConfigurationValue	= 3,
		.iConfiguration		= 0,
		.bmAttributes		= BOARD_USB_BMATTRIBUTES,
		.bMaxPower		= USBConfigurationDescriptor_POWER(100),
	},
	.interface = {
		.bLength		= sizeof(USBInterfaceDescriptor),
		.bDescriptorType	= USBGenericDescriptor_INTERFACE,
		.bInterfaceNumber	= 0,
		.bAlternateSetting	= 0,
		.bNumEndpoints		= 3,
		.bInterfaceClass	= SMART_CARD_DEVICE_CLASS,
		.bInterfaceSubClass	= 0,
		.bInterfaceProtocol	= 0,
		.iInterface		= 0,
	},
	.ccid = {
		.bLength		= sizeof(CCIDDescriptor),
		.bDescriptorType	= CCID_DECRIPTOR_TYPE,
		.bcdCCID		= CCID1_10,
		.bMaxSlotIndex		= 0,
		.bVoltageSupport	= VOLTS_3_0,
		.dwProtocols		= (1 << PROTOCOL_TO),
		.dwDefaultClock		= 3580,
		.dwMaximumClock		= 3580,
		.bNumClockSupported	= 0,
		.dwDataRate		= 9600,
		.dwMaxDataRate		= 9600,
		.bNumDataRatesSupported	= 0,
		.dwMaxIFSD		= 0xfe,
		.dwSynchProtocols	= 0,
		.dwMechanical		= 0,
		.dwFeatures		= CCID_FEATURES_AUTO_CLOCK | CCID_FEATURES_AUTO_BAUD |
					  CCID_FEATURES_AUTO_PCONF | CCID_FEATURES_AUTO_PNEGO |
					  CCID_FEATURES_EXC_TPDU,
		.dwMaxCCIDMessageLength	= 0x10f,
		.bClassGetResponse	= 0xff,
		.bClassEnvelope		= 0xff,
		.wLcdLayout		= 0,
		.bPINSupport		= 0,
		.bMaxCCIDBusySlots	= 1,
	},
	.endpoint = {
		{
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
									CCID_EPT_DATA_OUT),
		.bmAttributes		= USBEndpointDescriptor_BULK,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CCID_EPT_DATA_OUT),
					      USBEndpointDescriptor_MAXINTERRUPTSIZE_FS),
		.bInterval		= 0,
		},
		{
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
									CCID_EPT_DATA_IN),
		.bmAttributes		= USBEndpointDescriptor_BULK,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CCID_EPT_DATA_IN),
					      USBEndpointDescriptor_MAXINTERRUPTSIZE_FS),
		.bInterval		= 0,
		},
		{
		.bLength		= sizeof(USBEndpointDescriptor),
		.bDescriptorType	= USBGenericDescriptor_ENDPOINT,
		.bEndpointAddress	= USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
									CCID_EPT_NOTIFICATION),
		.bmAttributes		= USBEndpointDescriptor_INTERRUPT,
		.wMaxPacketSize		= MIN(BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CCID_EPT_NOTIFICATION),
					      USBEndpointDescriptor_MAXINTERRUPTSIZE_FS),
		.bInterval		= 0x10,
		},
	},
};


/// String descriptor with the supported languages.
const unsigned char languageIdDescriptor[] = {

    USBStringDescriptor_LENGTH(1),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_ENGLISH_US
};

/// Manufacturer name.
const unsigned char manufacturerDescriptor[] = {

    USBStringDescriptor_LENGTH(5),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('A'),
    USBStringDescriptor_UNICODE('t'),
    USBStringDescriptor_UNICODE('m'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('l')
};

/// Product name.
const unsigned char productDescriptor[] = {

    USBStringDescriptor_LENGTH(10),
    USBGenericDescriptor_STRING,
    USBStringDescriptor_UNICODE('M'),
    USBStringDescriptor_UNICODE('u'),
    USBStringDescriptor_UNICODE('l'),
    USBStringDescriptor_UNICODE('t'),
    USBStringDescriptor_UNICODE('i'),
    USBStringDescriptor_UNICODE(' '),
    USBStringDescriptor_UNICODE('D'),
    USBStringDescriptor_UNICODE('e'),
    USBStringDescriptor_UNICODE('m'),
    USBStringDescriptor_UNICODE('o')
};

/// Array of pointers to the four string descriptors.
const unsigned char *stringDescriptors[] = {

    languageIdDescriptor,
    manufacturerDescriptor,
    productDescriptor,
};

//------------------------------------------------------------------------------
//         Exported variables
//------------------------------------------------------------------------------

/// List of descriptors required by an USB audio speaker device driver.
const USBDDriverDescriptors multiDriverDescriptors = {

    &deviceDescriptor,
    { (const USBConfigurationDescriptor *) &sniffer_configurationDescriptors,
      (const USBConfigurationDescriptor *) &reader_configurationDescriptors,
      (const USBConfigurationDescriptor *) &mitm_configurationDescriptors },
#ifdef BOARD_USB_UDPHS
    &qualifierDescriptor,
    { (const USBConfigurationDescriptor *) &sniffer_configurationDescriptors },
    &deviceDescriptor,
    { (const USBConfigurationDescriptor *) &sniffer_configurationDescriptors },
    &qualifierDescriptor,
    { (const USBConfigurationDescriptor *) &sniffer_configurationDescriptors },
#else
    0, { 0 }, 0, { 0 }, 0, { 0 },
#endif
    stringDescriptors,
    3 // Number of string descriptors
};

