/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  EDID Injector firmware
 *
 * \author
 *      Wolfgang 'datenwolf' Draxinger
 *      Support email: projects+edid_injector@datenwolf.net
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "avr_compiler.h"

#include <string.h>

#include "eeprom_driver.h"

#include "twi_master_driver.h"
#include "twi_slave_driver.h"

/*! BAUDRATE 100kHz and Baudrate Register Settings */
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(F_CPU, BAUDRATE)

#define EDID_SLAVE_ADDRESS 0x50

/* TWIC is on the display side */
TWI_t * const twiDisplay = &TWIC;
TWI_Master_t twimDisplay;    /*!< TWI master module. Used for talking with display */
/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twimDisplay);
}


/* TWIE is on the host side */
TWI_t * const twiHost = &TWIE;
TWI_Slave_t twisHost;      /*!< TWI slave module.  Used for talking with host */
/*! TWIC Slave Interrupt vector. */
ISR(TWIE_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twisHost);
}

#define EDID_BLOCK_LENGTH 128

static uint8_t edid_data[EDID_BLOCK_LENGTH];
static uint8_t edid_offset;
static uint8_t recvbuf[1];

void TWIC_SlaveProcessData(TWI_Slave_t * const twi)
{
	if( twi->bytesReceived ) {
		edid_offset = twi->recvData[0];
	}
	twi->sendData = edid_data + edid_offset;
	twi->bytesToSend = sizeof(edid_data) - edid_offset;
}

void edid_initHostTWI(void)
{
	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twisHost,
		twiHost,
		TWIC_SlaveProcessData);

	TWI_SlaveInitializeModule(&twisHost,
		EDID_SLAVE_ADDRESS,
		TWI_SLAVE_INTLVL_HI_gc,
		recvbuf,
		sizeof(recvbuf) );
}

void edid_initDisplayTWI(void)
{
	/* Initialize TWI master. */
	twimDisplay.status = TWIM_STATUS_READY;
	TWI_MasterInit(&twimDisplay,
	               twiDisplay,
	               TWI_MASTER_INTLVL_LO_gc,
	               TWI_BAUDSETTING);

	/* Enable internal pull-up on PC0, PC1. */
	/* If you don't have external pullups enable this */
#if 0
	/* Configure several PINxCTRL registers at the same time */
	PORTCFG.MPCMASK = 0x03; 

	/* Enable pull-up to get a defined level on the switches */
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;
#endif

}

uint8_t edid_checkData(uint8_t const *ediddata)
{
	uint8_t checksum = 0;
	for(uint8_t i = 0; i < EDID_BLOCK_LENGTH; i++) {
		checksum += ediddata[i];
	}
	if( 0 != checksum ) {
		return 1; /* checksum failed */
	}

	if( 0 != ediddata[0] ) {
		return 2; /* headerbyte[0] != 0 */
	}

	for(uint8_t i = 1; i < 7; i++) {
		if( 0xff != ediddata[i] ) {
			return 3; /* headerbyte[1..6] != 0xff */
		}
	}

	if( 0 != ediddata[7] ) {
		return 4; /* headerbyte[7] != 0 */
	}

	return 0; /* EDID data passed checks */
}

uint8_t edid_genChecksum(uint8_t *ediddata)
{
	uint8_t bytessum = 0;
	for(uint8_t i = 0; i < EDID_BLOCK_LENGTH-1; i++) {
		bytessum += ediddata[i];
	}

	return 0xff - bytessum + 1;
}

uint8_t edid_readFromDisplay(void)
{
	uint8_t const offset[1] = {0x00};
	uint8_t displayedid[EDID_BLOCK_LENGTH];
	memset(displayedid, 0, sizeof(displayedid));

	uint8_t error = 0;
	uint8_t retry = 20;
	while( retry-- ) {
		error = 0;

		TWI_MasterForceIdle(&twimDisplay);
		delay_ms(1);

		/* set EDID offset to 0 */
		if( !TWI_MasterWriteRead(&twimDisplay,
			EDID_SLAVE_ADDRESS,
			offset,
			sizeof(offset),
			displayedid,
			sizeof(displayedid) ) ) {
			PORTB.OUTCLR = (1<<3);
			PORTB.OUTSET = (1<<3);
		};

		while( !TWI_MasterReady(&twimDisplay) ) {
			delay_ms(1);
		}
		delay_ms(1);

		if( twimDisplay.result != TWIM_RESULT_OK ) {
			error = 1;
			continue;
		}

		if( edid_checkData(displayedid) ) {
			error = 2;
			continue;
		}

		break;
	}
	if( error ) {
		return error;
	}

	if( edid_checkData(displayedid) ) {
		return 3;
	}

	/* set EDID Extension Block count / flags to zero */
	displayedid[126] = 0;

	/* recalculate checksum */
	displayedid[127] = edid_genChecksum(displayedid);

	/* write EDID information to EEPROM */
	memcpy(edid_data, displayedid, EDID_BLOCK_LENGTH);

	return 0;
}

uint8_t edid_readFromEEPROM(void)
{
	uint8_t eepromedid[EDID_BLOCK_LENGTH];
	memset(eepromedid, 0, sizeof(EDID_BLOCK_LENGTH));

	EEPROM_WaitForNVM();
	EEPROM_FlushBuffer();
	EEPROM_DisableMapping();

	for(uint8_t i_page = 0;
	    i_page < EDID_BLOCK_LENGTH / EEPROM_PAGE_SIZE;
	    i_page++ ) {
		for(uint8_t i_b = 0; i_b < EEPROM_PAGE_SIZE; i_b++) {
			eepromedid[i_page * EEPROM_PAGE_SIZE + i_b] =
				EEPROM_ReadByte(i_page, i_b);
		}
	}

	if( edid_checkData(eepromedid) ) {
		/* EDID data read from EEPROM didn't pass sanity checks */
		// memset(edid_data, 0, EDID_BLOCK_LENGTH);
		return 1;
	}

	memcpy(edid_data, eepromedid, EDID_BLOCK_LENGTH);
	return 0;
}

void edid_writeToEEPROM(void)
{
	uint8_t pagebuf[EEPROM_PAGE_SIZE];

	EEPROM_WaitForNVM();
	EEPROM_FlushBuffer();
	EEPROM_DisableMapping();

	for(uint8_t i_page = 0;
	    i_page < EDID_BLOCK_LENGTH / EEPROM_PAGE_SIZE;
	    i_page++ ) {
		for(uint8_t i_b = 0; i_b < EEPROM_PAGE_SIZE; i_b++) {
			pagebuf[i_b] = EEPROM_ReadByte(i_page, i_b);
		}

		if( 0 == memcmp(pagebuf,
		                edid_data + i_page*EEPROM_PAGE_SIZE,
		                EEPROM_PAGE_SIZE )
		) {
			/* page unaltered skip writing it */
			continue;
		}

		memcpy(	pagebuf,
			edid_data + i_page*EEPROM_PAGE_SIZE,
			sizeof(pagebuf) );

		EEPROM_LoadPage( pagebuf );
		EEPROM_AtomicWritePage(i_page);
		EEPROM_WaitForNVM();
	}
}

int main(void)
{
	memset(edid_data, 0, sizeof(edid_data));

	edid_readFromEEPROM();

	edid_initHostTWI();
	edid_initDisplayTWI();

	/* Enable LO interrupt level. */
	PMIC.CTRL |=
		  PMIC_LOLVLEN_bm
		| PMIC_MEDLVLEN_bm
		| PMIC_HILVLEN_bm;
	sei();

	delay_ms(20); 

	/* EDID standard requires a host to wait for 20ms after switching +5V
	 * supply to display before performing the first readout attempt.
	 * Since uC supply == display +5V supply we're waiting 20ms here.
	 */
	for(;;) {
		if( 0 == edid_readFromDisplay() ) {
			edid_writeToEEPROM();
			edid_readFromEEPROM();
		}
		delay_ms(1000); 
	}

	return 0;
}

