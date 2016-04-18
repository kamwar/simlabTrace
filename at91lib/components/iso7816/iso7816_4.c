/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 * Copyright (c) 2014, Kamil Wartanowicz
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
/// \unit
///
/// !Purpose
/// 
/// ISO 7816 driver
/// 
/// !Usage
/// 
/// Explanation on the usage of the code made available through the header file.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <string.h>
#include <board.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <utility/misc.h>
#include <pio/pio.h>
#include "iso7816_4.h"

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------
/// Case for APDU commands
#define CASE1  1
#define CASE2  2
#define CASE3  3

/// Flip flop for send and receive char
#define USART_SEND 0
#define USART_RCV  1

//timeout for receiving data from UICC
#define RX_DATA_TIMEOUT 300 //ms

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------
/// Variable for state of send and receive froom USART
static unsigned char StateUsartGlobal = USART_RCV;
/// Pin reset master card
static Pin st_pinIso7816RstMC;

//-----------------------------------------------------------------------------
//         Internal functions
//-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Get a character from ISO7816
/// \param pCharToReceive Pointer for store the received char
/// \param timeout in ms
/// \return 0: if timeout else status of US_CSR
//------------------------------------------------------------------------------
#define GET_CHAR_DELAY 1 //ms
static unsigned int ISO7816_GetChar(unsigned char *pCharToReceive, unsigned int timeout)
{
    unsigned int status;
    volatile unsigned int iteration=0;

    if( StateUsartGlobal == USART_SEND ) {
        while((AT91C_BASE_US0->US_CSR & AT91C_US_TXEMPTY) == 0) {}
        AT91C_BASE_US0->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        StateUsartGlobal = USART_RCV;
    }

    // Wait USART ready for reception
    while (((AT91C_BASE_US0->US_CSR & AT91C_US_RXRDY) == 0)) {
		if (iteration++ > (timeout / GET_CHAR_DELAY)) {
            TRACE_ERROR("ISO7816_GetChar TimeOut\n\r");
            return(0);
        }
		delay_ms(GET_CHAR_DELAY);
    }

    // At least one complete character has been received and US_RHR has not yet been read.
    // Get a char
    *pCharToReceive = ((AT91C_BASE_US0->US_RHR) & 0xFF);

    status = (AT91C_BASE_US0->US_CSR&(AT91C_US_OVRE|AT91C_US_FRAME|
                                      AT91C_US_PARE|AT91C_US_TIMEOUT|AT91C_US_NACK|
                                      (1<<10)));

    if (status != 0 ) {
       // TRACE_DEBUG("R:0x%X\n\r", status);
        TRACE_DEBUG("R:0x%X\n\r", AT91C_BASE_US0->US_CSR);
        TRACE_DEBUG("Nb:0x%X\n\r", AT91C_BASE_US0->US_NER );
        AT91C_BASE_US0->US_CR = AT91C_US_RSTSTA;
    }

    // Return status
    return(1);
}


//------------------------------------------------------------------------------
/// Send a char to ISO7816
/// \param CharToSend char to be send
/// \return status of US_CSR
//------------------------------------------------------------------------------
static unsigned int ISO7816_SendChar( unsigned char CharToSend )
{
    unsigned int status;

    if( StateUsartGlobal == USART_RCV ) {
        AT91C_BASE_US0->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        StateUsartGlobal = USART_SEND;
    }

    // Wait USART ready for transmit
    while((AT91C_BASE_US0->US_CSR & AT91C_US_TXRDY) == 0)  {}
    // There is no character in the US_THR

    // Transmit a char
    AT91C_BASE_US0->US_THR = CharToSend;

    status = (AT91C_BASE_US0->US_CSR&(AT91C_US_OVRE|AT91C_US_FRAME|
                                      AT91C_US_PARE|AT91C_US_TIMEOUT|AT91C_US_NACK|
                                      (1<<10)));

    if (status != 0 ) {
        TRACE_DEBUG("E:0x%X\n\r", AT91C_BASE_US0->US_CSR);
        TRACE_DEBUG("Nb:0x%X\n\r", AT91C_BASE_US0->US_NER );
        AT91C_BASE_US0->US_CR = AT91C_US_RSTSTA;
    }

    // Return status
    return (status);
}


//------------------------------------------------------------------------------
/// Iso 7816 ICC power on
//------------------------------------------------------------------------------
static void ISO7816_IccPowerOn( void )
{
    // Set RESET Master Card
    PIO_Set(&st_pinIso7816RstMC);
}

//-----------------------------------------------------------------------------
//         Exported functions
//-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Iso 7816 ICC power off
//------------------------------------------------------------------------------
void ISO7816_IccPowerOff( void )
{
    // Clear RESET Master Card
    PIO_Clear(&st_pinIso7816RstMC);
}


unsigned short ISO7816_FF_TPDU_T0(const unsigned char *pAPDU,
                                  unsigned char *pMessage,
                                  unsigned short wLength )
{
    TRACE_DEBUG("CLA=FF\n\r");
    // CLA 0xFF
    // INS ??
    // P1  ??
    // P2  ??
    // P3  Length

    unsigned short i = 0;
    pMessage[i++] = 1;
    pMessage[i++] = 2;
    pMessage[i++] = 3;
    pMessage[i++] = 4;
    return (i);
}


//------------------------------------------------------------------------------
/// Transfert Block TPDU T=0
/// \param pAPDU    APDU buffer
/// \param pMessage Message buffer
/// \param wLength  Block length
/// \return         Message index
//------------------------------------------------------------------------------
unsigned short ISO7816_XfrBlockTPDU_T0(const unsigned char *pAPDU, 
                                        unsigned char *pMessage, 
                                        unsigned short wLength )
{
    unsigned short NeNc;
    unsigned short indexCapdu = 5; //command data starts at pAPDU[5]
    unsigned short indexRapdu = 0;
    unsigned char procByte;
    unsigned char cmdCase;

    TRACE_DEBUG("pAPDU[0:4]=%02X%02X%02X%02X%02X, length=%d\n\r",
        pAPDU[0], pAPDU[1], pAPDU[2], pAPDU[3], pAPDU[4], wLength);

    ISO7816_SendChar( pAPDU[0] ); // CLA
    ISO7816_SendChar( pAPDU[1] ); // INS
    ISO7816_SendChar( pAPDU[2] ); // P1
    ISO7816_SendChar( pAPDU[3] ); // P2
    ISO7816_SendChar( pAPDU[4] ); // P3

    switch (wLength) {
    case 4:
        cmdCase = CASE1;
        NeNc = 0;
        break;
    case 5:
        cmdCase = CASE2;
        NeNc = pAPDU[4]; // C5
        if (NeNc == 0) {
            NeNc = 256;
        }
        break;
    case 6:
        NeNc = pAPDU[4]; // C5
        cmdCase = CASE3;
        break;
    case 7:
        NeNc = pAPDU[4]; // C5
        if (NeNc == 0) {
            cmdCase = CASE2;
            NeNc = (pAPDU[5] << 8) + pAPDU[6];
        }
        else {
            cmdCase = CASE3;
        }
        break;
    default:
        NeNc = pAPDU[4]; // C5
        if (NeNc == 0) {
            cmdCase = CASE3;
            NeNc = (pAPDU[5] << 8) + pAPDU[6];
        }
        else {
            cmdCase = CASE3;
        }
        break;
    }

    TRACE_DEBUG("CASE=0x%02X NeNc=0x%02X\n\r", cmdCase, NeNc);

    // Handle Procedure Bytes
    while(1) {
		procByte = 0xFF;
        if (!ISO7816_GetChar(&procByte, RX_DATA_TIMEOUT)) {
            TRACE_ERROR("failed to obtain procByte\n\r");
            goto error;
        }

        // Handle NULL
        if (procByte == ISO_NULL_VAL) {
            TRACE_DEBUG("INS_NULL\n\r");
            continue;
        }
        // Handle SW1
        else if (((procByte & 0xF0) == 0x60) || ((procByte & 0xF0) == 0x90)) {
            break;
        }
        // Handle INS
        else if ( pAPDU[1] == procByte) {
            TRACE_DEBUG("HdlINS\n\r");
            if (cmdCase == CASE2) {
                // receive data from card
                do {
                    if (!ISO7816_GetChar(&pMessage[indexRapdu++], RX_DATA_TIMEOUT)) {
                        TRACE_ERROR("failed to obtain response APDU\n\r");
                        goto error;
                    }
                } while( 0 != --NeNc );
            }
            else {
                 // Send data
                do {
                    ISO7816_SendChar(pAPDU[indexCapdu++]);
                } while( 0 != --NeNc );
            }
        }
        // Handle INS ^ 0xff
        else if ( pAPDU[1] == (procByte ^ 0xff)) {
            TRACE_DEBUG("HdlINS+\n\r");
            if (cmdCase == CASE2) {
                // receive data from card
                if (!ISO7816_GetChar(&pMessage[indexRapdu++], RX_DATA_TIMEOUT)) {
                    TRACE_ERROR("failed to obtain response APDU\n\r");
                    goto error;
                }
            }
            else {
                ISO7816_SendChar(pAPDU[indexCapdu++]);
            }
            NeNc--;
        }
        else {
            TRACE_ERROR("unknown procByte=0x%02X\n\r", procByte);
            break;
        }
    }

    // Status Bytes
    pMessage[indexRapdu] = procByte; //SW1
    indexRapdu++;

    if (!ISO7816_GetChar(&pMessage[indexRapdu], RX_DATA_TIMEOUT)) {
        TRACE_ERROR("failed to obtain SW2\n\r");
        goto error;
    }
    TRACE_DEBUG("SW1,SW2=%02X%02X\n\r", pMessage[indexRapdu-1], pMessage[indexRapdu]);
    indexRapdu++;
    return (indexRapdu);
    
error:
    return (0);
}

//------------------------------------------------------------------------------
/// Escape ISO7816
//------------------------------------------------------------------------------
void ISO7816_Escape( void )
{
    TRACE_DEBUG("For user, if needed\n\r");
}

//------------------------------------------------------------------------------
/// Restart clock ISO7816
//------------------------------------------------------------------------------
void ISO7816_RestartClock( void )
{
    TRACE_DEBUG("ISO7816_RestartClock\n\r");
    AT91C_BASE_US0->US_BRGR = 13;
}

//------------------------------------------------------------------------------
/// Stop clock ISO7816
//------------------------------------------------------------------------------
void ISO7816_StopClock( void )
{
    TRACE_DEBUG("ISO7816_StopClock\n\r");
    AT91C_BASE_US0->US_BRGR = 0;
}

//------------------------------------------------------------------------------
/// T0 APDU
//------------------------------------------------------------------------------
void ISO7816_toAPDU( void )
{
    TRACE_DEBUG("ISO7816_toAPDU\n\r");
    TRACE_DEBUG("Not supported at this time\n\r");
}

//----------------------------------------------------------------------
/// Answer To Reset (ATR)
/// \param pAtr    ATR buffer
/// \param pLength Pointer for store the ATR length
//----------------------------------------------------------------------
unsigned char ISO7816_Datablock_ATR( unsigned char* pAtr, unsigned char* pLength )
{
    unsigned int i = 0;
    unsigned int j;
    unsigned int y;
    unsigned char is_tck = 0;

    *pLength = 0;

    // Read ATR TS
    if (!ISO7816_GetChar(&pAtr[i++], RX_DATA_TIMEOUT)) {
        TRACE_ERROR("failed to obtain ATR TS\n\r");
        goto error;
    }

    // Read ATR T0
    if (!ISO7816_GetChar(&pAtr[i++], RX_DATA_TIMEOUT)) {
        TRACE_ERROR("failed to obtain ATR T0\n\r");
        goto error;
    }
    y = pAtr[1] & 0xF0;

    // Read ATR Ti
    while (y) {
        if (y & 0x10) {  // TA[i]
            if (!ISO7816_GetChar(&pAtr[i++], RX_DATA_TIMEOUT)) {
                TRACE_ERROR("failed to obtain TA[i]\n\r");
                goto error;
            }
        }
        if (y & 0x20) {  // TB[i]
            if (!ISO7816_GetChar(&pAtr[i++], RX_DATA_TIMEOUT)) {
                TRACE_ERROR("failed to obtain TB[i]\n\r");
                goto error;
            }
        }
        if (y & 0x40) {  // TC[i]
            if (!ISO7816_GetChar(&pAtr[i++], RX_DATA_TIMEOUT)) {
                TRACE_ERROR("failed to obtain TC[i]\n\r");
                goto error;
            }
        }
        if (y & 0x80) {  // TD[i]
            if (!ISO7816_GetChar(&pAtr[i], RX_DATA_TIMEOUT)) {
                TRACE_ERROR("failed to obtain TD[i]\n\r");
                goto error;
            }
            y =  pAtr[i++] & 0xF0;
            if (y != PROTOCOL_TO) {
                is_tck = 1;
			}
        }
        else {
            y = 0;
        }
    }

    // Historical Bytes
    y = pAtr[1] & 0x0F;
    for( j=0; j < y; j++ ) {
        if (!ISO7816_GetChar(&pAtr[i++], RX_DATA_TIMEOUT)){
            TRACE_ERROR("Failed to read historical byte %d\n\r", j);
            goto error;
        }
    }

    if (is_tck) {
        if (!ISO7816_GetChar(&pAtr[i++], RX_DATA_TIMEOUT)){
            TRACE_ERROR("Failed to read TCK");
            goto error;
        }
    }

    TRACE_DEBUG("ATR=");
    for (j=0; j < i; j++) {
        TRACE_DEBUG_WP("%02x", pAtr[j]);
    }
    TRACE_DEBUG_WP(", Length=%d\n\r\n\r", i);

    *pLength = i;
    return (1);
error:
    *pLength = 0;
    return (0);
}

//----------------------------------------------------------------------
/// Set data rate and clock frequency
/// \param dwClockFrequency ICC clock frequency in KHz.
/// \param dwDataRate       ICC data rate in bpd
//----------------------------------------------------------------------
void ISO7816_SetDataRateandClockFrequency( unsigned int dwClockFrequency, unsigned int dwDataRate )
{
    unsigned char ClockFrequency;

    // Define the baud rate divisor register
    // CD  = MCK / SCK
    // SCK = FIDI x BAUD = 372 x 9600
    // BOARD_MCK
    // CD = MCK/(FIDI x BAUD) = 48000000 / (372x9600) = 13
    AT91C_BASE_US0->US_BRGR = BOARD_MCK / (dwClockFrequency*1000);

    ClockFrequency = BOARD_MCK / AT91C_BASE_US0->US_BRGR;

    AT91C_BASE_US0->US_FIDI = (ClockFrequency)/dwDataRate;
}

//------------------------------------------------------------------------------
/// Pin status for ISO7816 RESET
/// \return 1 if the Pin RstMC is high; otherwise 0.
//------------------------------------------------------------------------------
unsigned char ISO7816_StatusReset( void )
{
    return PIO_Get(&st_pinIso7816RstMC);
}

//------------------------------------------------------------------------------
/// cold reset
//------------------------------------------------------------------------------
void ISO7816_cold_reset( void )
{
    volatile unsigned int i;

    // tb: wait 400 cycles, 3.58MHz => 80µs 48000000Hz  (3840)
    for( i=0; i<(120*(BOARD_MCK/1000000)); i++ ) {
    }

    AT91C_BASE_US0->US_RHR;
    AT91C_BASE_US0->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;

    ISO7816_IccPowerOn();
}

//------------------------------------------------------------------------------
/// Warm reset
//------------------------------------------------------------------------------
void ISO7816_warm_reset( void )
{
    volatile unsigned int i;

    ISO7816_IccPowerOff();

    // tb: wait 400 cycles, 3.58MHz => 80µs 48000000Hz  (3840)
    for( i=0; i<(120*(BOARD_MCK/1000000)); i++ ) {
    }

    AT91C_BASE_US0->US_RHR;
    AT91C_BASE_US0->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;

    ISO7816_IccPowerOn();
}

//----------------------------------------------------------------------
/// Decode ATR trace
/// \param pAtr pointer on ATR buffer
//----------------------------------------------------------------------
unsigned char ISO7816_Decode_ATR(unsigned char* pAtr,
                                 atParameter_t atrParameter,
                                 unsigned char *value)
{
    unsigned int i;
    unsigned int j;
    unsigned int y;
    unsigned char offset;
    unsigned char is_tck = 0;
    char historicalBytesAscii[16];

    TRACE_DEBUG("ATR: Answer To Reset:\n\r");
    TRACE_DEBUG("TS = 0x%X Initial character: ", pAtr[0]);
    if (pAtr[0] == 0x3B) {
        TRACE_DEBUG_WP("Direct Convention\n\r");
    }
    else {
        if (pAtr[0] == 0x3F) {

            TRACE_DEBUG_WP("Inverse Convention\n\r");
        }
        else {
            TRACE_DEBUG_WP("BAD Convention\n\r");
            goto error;
        }
    }

    if (atrParameter == ATR_TS) {
        *value = pAtr[0];
        goto success;
    }

    TRACE_DEBUG("T0 = 0x%X Format character\n\r", pAtr[1]);
    if (atrParameter == ATR_T0) {
        *value = pAtr[1];
        goto success;
    }

    TRACE_DEBUG("    Number of historical bytes: K = %d\n\r", pAtr[1] & 0x0F);
    if (atrParameter == ATR_HISTORICAL_NBR_OF_BYTES) {
        *value = pAtr[1] & 0x0F;
        goto success;
    }

    TRACE_DEBUG("    Presence further interface byte: ");
    if (pAtr[1] & 0x80) {
        TRACE_DEBUG_WP("TA1 ");
    }
    if (pAtr[1] & 0x40) {
        TRACE_DEBUG_WP("TB1 ");
    }
    if (pAtr[1] & 0x20) {
        TRACE_DEBUG_WP("TC1 ");
    }
    if (pAtr[1] & 0x10) {
        TRACE_DEBUG_WP("TD1 ");
    }
    if (pAtr[1] != 0) {
        TRACE_DEBUG_WP("\n\r");
    }

    i = 2;
    y = pAtr[1] & 0xF0;

    // Read ATR Ti
    offset = 1;

    while (y) {
        if (y & 0x10) {  // TA[i]
            TRACE_DEBUG("TA[%d] = 0x%X", offset, pAtr[i]);
            if( offset == 1 ) {
                TRACE_DEBUG_WP(", FI = %d", (pAtr[i]>>8));
                TRACE_DEBUG_WP(", DI = %d", (pAtr[i]&0x0F));
            }
            TRACE_DEBUG_WP("\n\r");
            if (atrParameter == ATR_TA1 + (offset - 1)*ATR_TAI_OFFSET) {
                *value = pAtr[i];
                goto success;
            }
            i++;
        }
        if (y & 0x20) {  // TB[i]
            TRACE_DEBUG("TB[%d] = 0x%X\n\r", offset, pAtr[i]);
            if (atrParameter == ATR_TB1 + (offset - 1)*ATR_TAI_OFFSET) {
                *value = pAtr[i];
                goto success;
            }
            i++;
        }
        if (y & 0x40) {  // TC[i]
            TRACE_DEBUG("TC[%d] = 0x%X", offset, pAtr[i]);
            if( offset == 1 ) {
                TRACE_DEBUG_WP(", Extra Guard Time: N = %d", pAtr[i]);
            }
            TRACE_DEBUG_WP("\n\r");
            if (atrParameter == ATR_TC1 + (offset - 1)*ATR_TAI_OFFSET) {
                *value = pAtr[i];
                goto success;
            }
            i++;
        }
        if (y & 0x80) {  // TD[i]
            TRACE_DEBUG("TD[%d] = 0x%X\n\r", offset, pAtr[i]);
            if (atrParameter == ATR_TD1 + (offset - 1)*ATR_TAI_OFFSET) {
                *value = pAtr[i];
                goto success;
            }
            y =  pAtr[i++] & 0xF0;
            if (y != PROTOCOL_TO) {
                is_tck = 1;
            }
        }
        else {
            y = 0;
        }
        offset++;
    }

    // Historical Bytes
    TRACE_DEBUG("Historical bytes: ");
    historicalBytesAscii[0] = '\0';

    y = pAtr[1] & 0x0F;
    for( j=0; j < y; j++ ) {
        TRACE_DEBUG_WP("%02X", pAtr[i]);
        if (atrParameter == ATR_HISTORICAL_INDEX_START && j == 0) {
            return i;
        }
        if( (pAtr[i] > 0x21) && (pAtr[i] < 0x7D) ) {  // ASCII
            sprintf(historicalBytesAscii + strlen(historicalBytesAscii), "%c", pAtr[i]);
        }
        i++;
    }

    if (historicalBytesAscii[0]) {
        TRACE_DEBUG_WP(", ASCII: %s", historicalBytesAscii);
    }
    TRACE_DEBUG_WP("\n\r");

    if (is_tck) {
        TRACE_DEBUG("TCK = 0x%X\n\r", pAtr[i]);
        if (atrParameter == ATR_TCK) {
            *value = pAtr[i];
            goto success;
        }
    }

error:
    return (0);
success:
    TRACE_DEBUG_WP("\n\r");
    return (1);
}

//------------------------------------------------------------------------------
//  Initializes a ISO driver
/// \param pPinIso7816RstMC Pin ISO 7816 Rst MC
//------------------------------------------------------------------------------
void ISO7816_Init( const Pin pPinIso7816RstMC )
{
    TRACE_DEBUG("ISO_Init\n\r");

    // Pin ISO7816 initialize
    st_pinIso7816RstMC  = pPinIso7816RstMC;

    USART_Configure( AT91C_BASE_US0, 
                     AT91C_US_USMODE_ISO7816_0
                     | AT91C_US_CLKS_CLOCK       
                     | AT91C_US_NBSTOP_1_BIT     
                     | AT91C_US_PAR_EVEN         
                     | AT91C_US_CHRL_8_BITS      
                     | AT91C_US_CKLO             
                     | (3<<24), // MAX_ITERATION
                     1,
                     0);

    // Configure USART0
    AT91C_BASE_PMC->PMC_PCER = ((unsigned int) 1 << AT91C_ID_US0);
    // Disable interrupts
    AT91C_BASE_US0->US_IDR = (unsigned int) -1;

    AT91C_BASE_US0->US_FIDI = 372;  // by default
    // Define the baud rate divisor register
    // CD  = MCK / SCK
    // SCK = FIDI x BAUD = 372 x 9600
    // BOARD_MCK
    // CD = MCK/(FIDI x BAUD) = 48000000 / (372x9600) = 13
    AT91C_BASE_US0->US_BRGR = BOARD_MCK / (372*9600);

    // Write the Timeguard Register
    AT91C_BASE_US0->US_TTGR = 5;

    USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);
    USART_SetReceiverEnabled(AT91C_BASE_US0, 1);

}

