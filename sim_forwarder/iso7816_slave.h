#ifndef _ISO7816_SLAVE_H_
#define _ISO7816_SLAVE_H_

#include <stdint.h>

/* iso7816_slave implements card-side of the ISO7816 T=0 protocol.
   This object sits inbetween:
   - platform-specific serial port from iso7816_port.h  (currently hard-coded)
   - an abstract C-APDU send / R-APDU receive packet transport mechanism

   The object is implemented as a non-blocking state machine to avoid
   dependence on thread system, e.g. to allow bare metal
   implementation.
*/

struct iso7816_slave;

// Send a-synchronous command to state machine.

// IN: host->simtrace command, recorded in control request
enum iso7816_slave_cmd {
    CMD_SET_ATR = 0,
    CMD_SET_SKIP = 1,
    CMD_HALT = 2,
    CMD_POLL = 3,
    CMD_R_APDU = 4,
};
// OUT: simtrace->host, recorded in simtrace_hdr as a reply to CMD_POLL
enum iso7816_slave_evt {
    // compatible with simtrace_usb_msgt
    //EVT_NULL = 0,
    //EVT_DATA = 1,
    EVT_RESET = 2,
    //EVT_STATS = 3,
    EVT_C_APDU = 4,
};
struct simtrace_hdr {
    uint8_t evt;
    uint8_t flags;
    uint8_t res[2];
} __attribute__((packed));


// Callback to initiate C-APDU packet transfer.
typedef void (*iso7816_slave_send_event)(void *ctx, enum iso7816_slave_evt event,
                                         const uint8_t *buf, int size);

// Send R-APDU to serial port.
int iso7816_slave_r_apdu_write(struct iso7816_slave *s, const uint8_t *buf, int size);

struct iso7816_slave *iso7816_slave_init(iso7816_slave_send_event send, void *send_ctx);

// State machine transition routine.  This should be called in the
// application's event loop.
void iso7816_slave_tick(struct iso7816_slave *s);


int iso7816_slave_command(struct iso7816_slave *s,
                          enum iso7816_slave_cmd command,
                          const uint8_t *buf, int size);


#endif
