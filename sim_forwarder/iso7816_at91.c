/* Adapted from iso7816_4.h */

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

/* PLATFORM-SPECIFIC: AT91SAM7S */

#include <stdint.h>
#include "errno.h"
#include <utility/trace.h>

#include <usart/usart.h>

enum iso7816_dir  {
    DIR_INVALID = 0,
    DIR_RX,
    DIR_TX,
};
struct iso7816_port {
    AT91S_USART *usart;
    enum iso7816_dir dir;
    uint32_t vcc_pin;
    uint32_t rst_pin;
};

static int iso7816_port_status(struct iso7816_port *p) {
    /* Check */
    uint32_t mask = AT91C_US_OVRE
        | AT91C_US_FRAME
        | AT91C_US_PARE
        | AT91C_US_TIMEOUT
        | AT91C_US_NACK
        | (1<<10);
    uint32_t csr = p->usart->US_CSR;
    uint32_t status = mask & csr;
    if (status != 0) {
        TRACE_ERROR("csr: %08x %08x\n\r", csr, status);
        p->usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTNACK;
        return -EIO; // FIXME: more descriptive?
    }
    else {
        return ENOERR;
    }
}

int iso7816_port_tx(struct iso7816_port *p, uint8_t c) {
    /* Switch RX/TX state if necessary. */
    if (p->dir != DIR_TX) {
        p->usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        p->dir = DIR_TX;
        // TRACE_DEBUG("TX\n\r");
    }
    if ((p->usart->US_CSR & AT91C_US_TXRDY) == 0) return -EAGAIN;
    p->usart->US_THR = c;
    return iso7816_port_status(p);
}

int iso7816_port_rx(struct iso7816_port *p, uint8_t *c) {
    /* Switch RX/TX state if necessary. */
    if (p->dir != DIR_RX) {
        if ((p->usart->US_CSR & AT91C_US_TXEMPTY) == 0) return -EAGAIN;
        p->usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        p->dir = DIR_RX;
        // TRACE_DEBUG("RX\n\r");
    }
    if ((p->usart->US_CSR & AT91C_US_RXRDY) == 0) return -EAGAIN;
    *c = ((p->usart->US_RHR) & 0xFF);
    return iso7816_port_status(p);
}

void iso7816_port_set_fidi(uint32_t fidi, struct iso7816_port *p) {
    if (fidi > 0 && fidi < 0x400) {
        TRACE_WARNING("FiDi ratio %d\n\r", fidi);

        /* make sure UART uses new F/D ratio */
        p->usart->US_CR |= AT91C_US_RXDIS | AT91C_US_RSTRX;
        p->usart->US_FIDI = fidi & 0x3ff;
        p->usart->US_CR |= AT91C_US_RXEN | AT91C_US_STTTO;

        /* notify ETU timer about this */
        //tc_etu_set_etu(rc);  // FIXME: implement ETU timer!
    } else {
        TRACE_ERROR("FiDi ratio %d unsupported\n\r", fidi);
    }
}



// low-level AT91SAM7 USART init
static void init_usart(AT91S_USART *usart, uint32_t master_slave) {

    /* Reset and disable receiver & transmitter */
    usart->US_CR =
        AT91C_US_RSTRX
        | AT91C_US_RSTTX
        | AT91C_US_RXDIS
        | AT91C_US_TXDIS;

    usart->US_CR =
        AT91C_US_RXDIS
        | AT91C_US_TXDIS;

    /* Set mode to ISO7816 T0, external clock source */
    usart->US_MR =
        AT91C_US_USMODE_ISO7816_0  // T0
        | AT91C_US_CHRL_8_BITS
        | AT91C_US_PAR_EVEN
        | AT91C_US_NBSTOP_1_BIT
        | AT91C_US_CKLO
        | AT91C_US_INACK
        | master_slave
        | (3<<24);

    /* In 7816 mode, Baud Rate = Selected Clock / CD / FI_DI_RATIO
       Since we use external clock, CD=1 */
    usart->US_BRGR = 1;
    usart->US_FIDI = 372 & 0x3FF;

    /* Disable Receiver Time-out */
    usart->US_RTOR = 0;

    /* Disable Transmitter Timeguard */
    usart->US_TTGR = 0;

    /* Enable RX/TX */
    usart->US_CR = AT91C_US_TXEN | AT91C_US_RXEN;

    /* read RHR once to flush?  (FIXME: necssary?) */
    usart->US_RHR;
}



/* BOARD-SPECIFIC: SIMTRACE */

// GPIOs: as annotated on schematics
#define  IO_PHONE  AT91C_PA22_TXD1
#define CLK_PHONE  AT91C_PA23_SCK1
#define RST_PHONE  AT91C_PIO_PA24
#define VCC_PHONE  AT91C_PIO_PA25

static struct iso7816_port phone = {
    .usart   = AT91C_BASE_US1,
    .vcc_pin = VCC_PHONE,
    .rst_pin = RST_PHONE,
};
#if 0
static struct iso7816_port sim = {
    .usart   = AT91C_BASE_US0,
};
#endif

void iso7816_port_get_rst_vcc(struct iso7816_port *p,
                              int *rst, int *vcc) {
    uint32_t pdsr = AT91C_BASE_PIOA->PIO_PDSR;
    if (rst) *rst = (0 != (pdsr & p->rst_pin));
    if (vcc) *vcc = (0 != (pdsr & p->vcc_pin));
}


struct iso7816_port *iso7816_port_init(int port_nb) {
    // FIXME: currently only does phone-side.
    if (port_nb != 1) return NULL;

    /* See at91lib/components/iso7816/iso7816_4.c
       and 31.6.4 in AT91SAM7S128 data sheet.

       For phone socket, clock is always slave. AT91C_US_CKLO is off:
       we never drive the clock pin.  This was on in openpcd-based
       SIMtrace FW.
    */

    /* Enable usart peripheral clock */
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US1); // | (1 << AT91C_ID_US0)

    /* GPIO setup */
    /* - PHONE I/O and CLK controlled by peripheral A (Table 10-3) */
    AT91C_BASE_PIOA->PIO_ASR = IO_PHONE | CLK_PHONE;
    AT91C_BASE_PIOA->PIO_PDR = IO_PHONE | CLK_PHONE;
    /* - PHONE RST and VCC are input.  This pin is tied to 3V3,100K.  */
    AT91C_BASE_PIOA->PIO_ODR = RST_PHONE | VCC_PHONE;
    AT91C_BASE_PIOA->PIO_PER = RST_PHONE | VCC_PHONE;

    /* Config USART peripheral in iso7816 mode, slave to external clock. */
    init_usart(phone.usart, AT91C_US_CLKS_EXT);

    return &phone;
}


