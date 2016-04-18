#ifndef _ISO7816_PORT_H_
#define _ISO7816_PORT_H_

#include <stdint.h>

/* LOW-LEVEL non-blocking I/O */

struct iso7816_port; // Abstract. implementation = platform-specific
struct iso7816_port *iso7816_port_init(int port_nb); // port_nb is board-specific
int iso7816_port_tx(struct iso7816_port *p, uint8_t c);
int iso7816_port_rx(struct iso7816_port *p, uint8_t *c);

void iso7816_port_get_rst_vcc(struct iso7816_port *p, int *rst, int *vcc);
void iso7816_port_set_fidi(uint32_t fidi, struct iso7816_port *p);

#endif
