/* 
   DS18x20 Demo-Programm
   
   V 0.7, 3/2005
   
   by Martin Thomas <eversmith@heizung-thomas.de>
   http://www.siwawi.arubi.uni-kl.de/avr-projects
    
   features:
   - DS18X20 and 1-Wire code is based on a sample from Peter 
     Dannegger
   - uses Peter Fleury's uart-library which is very portable 
     between AVRs, added some functions in the uart-lib
   - CRC-check based on code from Colin O'Flynn
   - access multiple sensors on multiple 1-Wire busses
   - samples how to address every sensor in the bus by ROM-code
   - independant of system-timers (more portable) but some
     (very short) delays used
   - avr-libc inttypes 
   - no central include-file, parts of the code can be used as
     "library"
   - verbose output (different levels configureable)
   - one-wire-bus can be changed at runtime if OW_ONE_BUS
     is not defined in onewire.h. There are still timing issues.
	 Tests done with ATmega16 3,68MHz XTAL OK, , 8MHz intRC OK, 
	 4MHz intRC OK, 2MHz intRC OK, 1,84MHz XTAL OK, 1MHz intRC 
	 failed in runtime-configureable OW-Bus. All frequencies do 
	 work in OW_ONE_BUS-Mode.
   - support-functions for DS18x20 internal EEPROM
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "uart.h"
#include "onewire.h"
#include "ds18x20.h"

#include "delay.h"

#define BAUD 9600
// 2400 for 1MHz and 2MHz internal RC
// #define BAUD 2400

#define MAXSENSORS 5

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	uart_puts_P( "\r\nScanning Bus for DS18X20\r\n" );
	
	nSensors = 0;
	
	for( diff = OW_SEARCH_FIRST; 
		diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; )
	{
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			uart_puts_P( "No Sensor found\r\n" );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			uart_puts_P( "Bus Error\r\n" );
			break;
		}
		
		for (i=0;i<OW_ROMCODE_SIZE;i++)
			gSensorIDs[nSensors][i]=id[i];
		
		nSensors++;
	}
	
	return nSensors;
}

void uart_put_temp(const uint8_t subzero, uint8_t cel, 
	uint8_t cel_frac_bits)
{
	uint8_t buffer[sizeof(int)*8+1];
	uint16_t decicelsius;
	uint8_t i, j;
	
	uart_putc((subzero)?'-':'+');
	uart_puti((int)cel);
	uart_puts_P(".");
	itoa((cel_frac_bits*DS18X20_FRACCONV),buffer,10);
	j=4-strlen(buffer);
	for (i=0;i<j;i++) uart_puts_P("0");
	uart_puts(buffer);
	uart_puts_P("°C [");
	// "rounding"
	uart_putc((subzero)?'-':'+');
	decicelsius = DS18X20_temp_to_decicel(subzero, cel, cel_frac_bits);
	uart_puti( (int)(decicelsius/10) );
	uart_puts_P(".");
	uart_putc( (decicelsius%10) + '0');
	uart_puts_P("°C]");
}

#ifdef DS18X20_EEPROMSUPPORT
static void eeprom_test(void)
{
    uint8_t sp[DS18X20_SP_SIZE], th, tl;
	
	uart_puts_P( "\r\nDS18x20 EEPROM support test for fist sensor\r\n" ); 
	// DS18X20_recall_E2(&gSensorIDs[0][0]); // already done at power-on
	DS18X20_read_scratchpad( &gSensorIDs[0][0], sp);	
	th = sp[DS18X20_TH_REG];
	tl = sp[DS18X20_TL_REG];
	uart_puts_P( "TH/TL from EEPROM sensor 1 : " ); 
	uart_puti(th);
	uart_puts_P( " / " ); 
	uart_puti(tl);
	uart_puts_P( "\r\n" ); 
	tl++; th++;
	DS18X20_write_scratchpad( &gSensorIDs[0][0], 
		th, tl, DS18B20_12_BIT);
	uart_puts_P( "TH+1 and TL+1 written to scratchpad\r\n" ); 
	DS18X20_copy_scratchpad(  DS18X20_POWER_PARASITE,
		&gSensorIDs[0][0] );
	uart_puts_P( "scratchpad copied to DS18x20 EEPROM\r\n" );
	DS18X20_recall_E2(&gSensorIDs[0][0]);
	uart_puts_P( "DS18x20 EEPROM copied back to scratchpad\r\n" );
	DS18X20_read_scratchpad( &gSensorIDs[0][0], sp);
	if ( (th == sp[DS18X20_TH_REG]) && (tl == sp[DS18X20_TL_REG]) ) 
		uart_puts_P( "TH and TL verified\r\n" );
	else 
		uart_puts_P( "verify failed\r\n" );
	th = sp[DS18X20_TH_REG];
	tl = sp[DS18X20_TL_REG];
	uart_puts_P( "TH/TL from EEPROM sensor 1 now : " ); 
	uart_puti(th);
	uart_puts_P( " / " ); 
	uart_puti(tl);
	uart_puts_P( "\r\n" ); 
}
#endif

int main( void )
{
	uint8_t nSensors, i;
	uint8_t subzero, cel, cel_frac_bits;
	
	uart_init((UART_BAUD_SELECT((BAUD),F_OSC)));
	
#ifndef OW_ONE_BUS
	ow_set_bus(&PIND,&PORTD,&DDRD,PD6);
#endif
	
	sei();
	
	uart_puts_P( "\r\nDS18X20 1-Wire-Reader Demo by Martin Thomas\r\n" );
	uart_puts_P( "-------------------------------------------" );
	
	// dbg uart_puts_P( "\r\n" );
	// dbg uart_puti((int) OW_CONF_DELAYOFFSET);
	// dbg uart_puts_P( "\r\n" );
	
	nSensors = search_sensors();
	uart_puti((int) nSensors);
	uart_puts_P( " DS18X20 Sensor(s) available:\r\n" );
	
	#ifdef DS18X20_VERBOSE
	for (i=0; i<nSensors; i++) {
		uart_puts_P("# in Bus :");
		uart_puti((int) i+1);
		uart_puts_P(" : ");
		DS18X20_show_id_uart( &gSensorIDs[i][0], OW_ROMCODE_SIZE );
		uart_puts_P( "\r\n" );
	}
	#endif
		
	for (i=0; i<nSensors; i++) {
		uart_puts_P("Sensor# ");
		uart_puti((int) i+1);
		uart_puts_P(" is a ");
		if ( gSensorIDs[i][0] == DS18S20_ID)
			uart_puts_P("DS18S20/DS1820");
		else uart_puts_P("DS18B20");
		uart_puts_P(" which is ");
		if ( DS18X20_get_power_status( &gSensorIDs[i][0] ) ==
			DS18X20_POWER_PARASITE ) 
			uart_puts_P( "parasite" );
		else uart_puts_P( "externally" ); 
		uart_puts_P( " powered\r\n" );
	}
	
#ifdef DS18X20_EEPROMSUPPORT
	if (nSensors>0) {
		eeprom_test();
	}
#endif

	if ( nSensors == 1 ) {
		uart_puts_P( "\r\nThere is only one sensor -> Demo of \"read_meas_single\":\r\n" ); 
		i = gSensorIDs[0][0]; // family-code for conversion-routine
		DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
		delay_ms(DS18B20_TCONV_12BIT);
		DS18X20_read_meas_single(i, &subzero, &cel, &cel_frac_bits);
		uart_put_temp(subzero, cel, cel_frac_bits);
		uart_puts_P("\r\n");
	}
		
	
	for(;;) {				// main loop
	
		uart_puts_P( "\r\nConvert_T and Read Sensor by Sensor (reverse order)\r\n" ); 
		for ( i=nSensors; i>0; i-- ) {
			if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, 
				&gSensorIDs[i-1][0] ) == DS18X20_OK ) {
				delay_ms(DS18B20_TCONV_12BIT);
				uart_puts_P("Sensor# ");
				uart_puti((int) i);
				uart_puts_P(" = ");
				if ( DS18X20_read_meas( &gSensorIDs[i-1][0], &subzero,
						&cel, &cel_frac_bits) == DS18X20_OK ) {
					uart_put_temp(subzero, cel, cel_frac_bits);
				}
				else uart_puts_P("CRC Error (lost connection?)");
				uart_puts_P("\r\n");
			}
			else uart_puts_P("Start meas. failed (short circuit?)");
		}
		
		uart_puts_P( "\r\nConvert_T for all Sensors and Read Sensor by Sensor\r\n" ); 	
		if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL ) 
			== DS18X20_OK) {
			delay_ms(DS18B20_TCONV_12BIT);
			for ( i=0; i<nSensors; i++ ) {
				uart_puts_P("Sensor# ");
				uart_puti((int) i+1);
				uart_puts_P(" = ");
				if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
						&cel, &cel_frac_bits) == DS18X20_OK ) {
					uart_put_temp(subzero, cel, cel_frac_bits);
				}
				else uart_puts_P("CRC Error (lost connection?)");
				uart_puts_P("\r\n");
			}
		}
		else uart_puts_P("Start meas. failed (short circuit?)");
				
#ifdef DS18X20_VERBOSE
		// all devices:
		uart_puts_P( "\r\nVerbose output\r\n" ); 
		DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
		delay_ms(DS18B20_TCONV_12BIT);
		DS18X20_read_meas_all_verbose();
#endif
				
		delay_ms(3000); 
	}
}
