#include "usb_control_at91.h"
#include <board.h>
#include <utility/trace.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <string.h>
#include "iso7816_slave.h"

/* SIMtrace PHONE interface uses VENDOR control requests on EP0:
   on AT91SAM7 we don't have enough endpoints for dedicated IN/OUT EPs.

   patch main USB driver request handler to pass on VENDOR control requests:

   if (USBGenericRequest_GetType(request) == USBGenericRequest_VENDOR) {
       usb_control_vendor_request(request);
       return;
   }
*/

static struct iso7816_slave *iso7816_slave;

enum iso7816_slave_cmd command;  // pc -> simtrace command (control request)

static uint8_t from_host_buf[512];
static int     from_host_size = 0;

static const uint8_t *to_host_msg  = NULL;

static uint8_t        to_host_buf[512];
static int            to_host_size = 0;


/* Callbacks run as part of ISR! */

static void read_cb(void *arg,
                    unsigned char status,
                    unsigned int transferred,
                    unsigned int remaining) {

    if (status != USBD_STATUS_SUCCESS) {
        TRACE_ERROR( "UsbDataReceived: Transfer error\n\r");
        return;
    }
    iso7816_slave_command(iso7816_slave, command,
                          from_host_buf, from_host_size);
    USBD_Write(0,0,0,0,0); // STATUS
}

static void write_cb(void *arg,
                     unsigned char status,
                     unsigned int transferred,
                     unsigned int remaining) {
    USBD_Read(0,0,0,0,0); // STATUS
}

/* Control  in  = SETUP IN [IN ...] STATUS(=OUT)
   Countrol out = SETUP OUT [OUT ...] STATUS(=IN) */

void usb_control_vendor_request(const USBGenericRequest *request) {
    // TRACE_WARNING("%d", request->bRequest);
    TRACE_DEBUG("vendor request %02X %d %d %d %d\n\r",
                request->bmRequestType,
                request->bRequest,
                request->wValue,
                request->wIndex,
                request->wLength);
    command = request->bRequest;
    switch (USBGenericRequest_GetDirection(request)) {

    case USBGenericRequest_OUT: // e.g 0x40
        /* Read payload. */

        from_host_size =  request->wLength;
        if (request->wLength > sizeof(from_host_buf)) {
            TRACE_ERROR("invalid length: %d\n\r", request->wLength);
            from_host_size = sizeof(from_host_buf);
        }
        if (from_host_size) {
            USBD_Read(0, &from_host_buf, from_host_size, read_cb, 0);
        }
        else {
            USBD_Write(0,0,0,0,0); // STATUS
        }

        break;

    case USBGenericRequest_IN:  // e.g. 0xC0
        switch(command) {
        case CMD_POLL:
            if (to_host_msg) {
                USBD_Write(0, to_host_msg, to_host_size, write_cb, 0);
                to_host_msg = NULL;
                to_host_size = 0;
            }
            else {
                /* Control requests can't NAK, so use an empty sentinel. */
                USBD_Write(0, 0, 0, write_cb, 0);
            }
            break;
        default:
            TRACE_ERROR("invalid IN command %d\n\r", command);
            USBD_Stall(0);
            to_host_msg = NULL;
            break;
        }
        break;
    }
}

static void send_evt(void *dummy, enum iso7816_slave_evt evt,
                     const uint8_t *buf, int size) {

    if (to_host_msg) {
        TRACE_ERROR("send_evt: already in progress\n\r");
    }

    struct simtrace_hdr hdr = {.evt = evt };

    memcpy(to_host_buf, &hdr, sizeof(hdr));
    if (!buf) size = 0;
    if (size) memcpy(to_host_buf + sizeof(hdr), buf, size);

    to_host_size = sizeof(hdr) + size;
    to_host_msg  = to_host_buf;  // set trigger var last
}

void usb_control_init(void) {
    /* Init phone ISO7816 USART */
    iso7816_slave = iso7816_slave_init(send_evt, NULL);
}

void usb_control_poll(void) {

    // Poll I/O state machine.
    iso7816_slave_tick(iso7816_slave);

}
