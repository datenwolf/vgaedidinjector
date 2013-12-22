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

#define EDID_SLAVE_ADDRESS 0xA0

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


void TWIC_SlaveProcessData(void)
{
}

void edid_initHostTWI(void)
{
	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twisHost, twiHost, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twisHost,
		EDID_SLAVE_ADDRESS,
		TWI_SLAVE_INTLVL_MED_gc );
}

void edid_initDisplayTWI(void)
{
	/* Initialize TWI master. */
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

uint8_t edid_genChecksum(uint8_t *ediddata)
{
	uint8_t bytessum = 0;
	for(size_t i = 0; i < 127; i++) {
		bytessum += ediddata[i];
	}

	return 0xff - bytessum;
}

void edid_readFromDisplayToEEPROM(void)
{
	uint8_t displayedid[128];
	memset(displayedid, 0, sizeof(displayedid));

	/* read 128 EDID bytes from display */

	/* set EDID Extension Block count / flags to zero */

	/* recalculate checksum */
	displayedid[127] = edid_genChecksum(displayedid);

	/* write EDID information to EEPROM */
}

int main(void)
{
	edid_initHostTWI();

	edid_initDisplayTWI();


	sei();


	#if 0
		TWI_MasterWriteRead(&twiMaster,
		                    SLAVE_ADDRESS,
		                    &sendBuffer[BufPos],
		                    1,
		                    1);


		while (twiMaster.status != TWIM_STATUS_READY) {
			/* Wait until transaction is complete. */
		}
	#endif
}

