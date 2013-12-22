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
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include "eeprom_driver.h"

/*! AUDRATE 100kHz and Baudrate Register Settings */
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(F_CPU, BAUDRATE)

/* TWIC is on the display side */
TWI_t * const twiDisplay = &TWIC;
TWI_Master_t twimDisplay;    /*!< TWI master module. Used for talking with display */

/* TWIE is on the host side */
TWI_t * const twiHost = &TWIE;
TWI_Slave_t twimHost;      /*!< TWI slave module.  Used for talking with host */


void TWIC_SlaveProcessData(void)
{
}

void edid_initHostTWI(void)
{
}

void edid_initDisplayTWI(void)
{
}

void edid_readFromDisplayToEEPROM(void)
{
	uint8_t buffer[128];
	memset(buffer, 0, sizeof(buffer);

}

int main(void)
{
	edid_initHostTWI();

	edid_initDisplayTWI();

	/* Initialize PORTE for output and PORTD for inverted input. */
	PORTE.DIRSET = 0xFF;
	PORTD.DIRCLR = 0xFF;
	PORTCFG.MPCMASK = 0xFF;
	PORTD.PIN0CTRL |= PORT_INVEN_bm;
//      PORTCFG.MPCMASK = 0xFF;
//      PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;
	
	// Enable internal pull-up on PC0, PC1.. Uncomment if you don't have external pullups
//	PORTCFG.MPCMASK = 0x03; // Configure several PINxCTRL registers at the same time
//	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc; //Enable pull-up to get a defined level on the switches

	

	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	               &TWIC,
	               TWI_MASTER_INTLVL_LO_gc,
	               TWI_BAUDSETTING);

	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave,
	                          SLAVE_ADDRESS,
	                          TWI_SLAVE_INTLVL_LO_gc);

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
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

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}
