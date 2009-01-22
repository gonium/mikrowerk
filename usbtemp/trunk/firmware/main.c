/* 
 * usbtemp firmware V0.1
 * Copyright (C) 2008, 2009 Mathias Dalheimer (md@gonium.net)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Based on:
 * - DS18x20 Demo-Programm by Martin Thomas <eversmith@heizung-thomas.de>
 * 	 http://www.siwawi.arubi.uni-kl.de/avr-projects (which in turn uses
 * 	 code of Peter Danneger, Colin O'Flynn and Peter Fleury)
 * - AVR-USB of OBJECTIVE DEVELOPMENT Software GmbH
 *   http://www.obdev.at/avrusb
 */

#define F_CPU 16000000UL  
#define BAUD 9600
#define DEBUG 1
#define MAIN_DELAY_MS 5 // needed by usb stack
#define READOUT_INTERVAL_MS 60000 // readout every minute
/* For USB stress-testing, use these: */
//#define MAIN_DELAY_MS 5 
//#define READOUT_INTERVAL_MS 500 
#define READOUT_WAIT_MS DS18B20_TCONV_12BIT // wait long enough for sensors

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/eeprom.h>
//#include <avr/wdt.h>

#include "uart.h"
#include "onewire.h"
#include "ds18x20.h"

#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"
#include <util/delay.h>
#include "delay.h"

#ifdef DEBUG
#define logs_P(message) uart_puts_P(message)
#define logs(message) uart_puts(message)
#define logc(message) uart_putc(message)
#define logi(message) uart_puti(message)
#else
#define logs_P(message) 
#define logs(message) 
#define logc(message) 
#define logi(message) 
#endif

/* internal format for temperature readings */
typedef struct temp_reading {
  uint8_t subzero;
  uint8_t cel;
  uint8_t cel_frac_bits;
} temp_reading_t;
/* states of the state machine in main*/
enum read_state_t {IDLE, MEASURING, UPDATED};

/* Temperature globals. */
#define MAXSENSORS 5
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
uint8_t nSensors; // number of attached sensors
temp_reading_t readings[MAXSENSORS];

/* USB globals */
#define DS18X20_STATUS_LED_PIN PB2
#define DS18X20_STATUS_LED_PORT PORTB
#define DS18X20_STATUS_LED_DDR DDRB

#define EEPROM_LOCATION (void *)37
/* allow some inter-device compatibility */
#if !defined TCCR0 && defined TCCR0B
#define TCCR0   TCCR0B
#endif
#if !defined TIFR && defined TIFR0
#define TIFR    TIFR0
#endif
// USB command codes
#define USBTEMP_CMD_TEST			0
#define USBTEMP_CMD_NO_SENSORS		1
#define USBTEMP_CMD_QUERY_SENSOR    2
#define USBTEMP_CMD_GET_TEMP	    3


/***************************** usb code below ****************************/
USB_PUBLIC uchar usbFunctionSetup(uchar data[8])
{
  usbRequest_t    *rq = (void *)data;
  static uchar    replyBuf[64];

  usbMsgPtr = replyBuf;
  if(rq->bRequest == USBTEMP_CMD_TEST){  /* ECHO */
	replyBuf[0] = rq->wValue.bytes[0];
	replyBuf[1] = rq->wValue.bytes[1];
	return 2;
  }
  if(rq->bRequest == USBTEMP_CMD_NO_SENSORS){  /* GET_NO_SENSORS -> result = 1 bytes */
	replyBuf[0] = nSensors;
	return 1;
  }
  if(rq->bRequest == USBTEMP_CMD_QUERY_SENSOR) { 
	/* Return id of sensor, 6 bytes.  The first byte contains the type
	 * of the sensor.
	 */
	uint8_t sensorId=rq->wValue.bytes[0];
	replyBuf[0] = gSensorIDs[sensorId][0];
	replyBuf[1] = gSensorIDs[sensorId][1];
	replyBuf[2] = gSensorIDs[sensorId][2];
	replyBuf[3] = gSensorIDs[sensorId][3];
	replyBuf[4] = gSensorIDs[sensorId][4];
	replyBuf[5] = gSensorIDs[sensorId][5];
	replyBuf[6] = gSensorIDs[sensorId][6];
	return 7;
  }
  if(rq->bRequest == USBTEMP_CMD_GET_TEMP) { 
	/* Return temperature reading of the sensor. The sensor ID is given
	 * in wValue. First byte: sensorID, next three bytes: temp. reading.
	 */
	uint8_t sensorId=rq->wValue.bytes[0];
	if (sensorId > nSensors) 
	  sensorId=0;
	replyBuf[0] = sensorId;
	replyBuf[1] = readings[sensorId].subzero;
	replyBuf[2] = readings[sensorId].cel;
	replyBuf[3] = readings[sensorId].cel_frac_bits;
	return 4;
  }
  return 0;
}


/********************** Temperature sensors code below *******************/

/** 
 * Discover the available sensors.
 */
uint8_t search_sensors(void) {
  uint8_t i;
  uint8_t id[OW_ROMCODE_SIZE];
  uint8_t diff, nSensors;
  nSensors = 0;
  for( diff = OW_SEARCH_FIRST; 
	  diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; ) {
	DS18X20_find_sensor( &diff, &id[0] );
	if( diff == OW_PRESENCE_ERR ) {
	  logs_P( "No Sensor found\r\n" );
	  break;
	}
	if( diff == OW_DATA_ERR ) {
	  logs_P( "Bus Error\r\n" );
	  break;
	}
	for (i=0;i<OW_ROMCODE_SIZE;i++) {
	  gSensorIDs[nSensors][i]=id[i];
	}
	nSensors++;
  }
  return nSensors;
}

/**
 * Converts the temperature to readable output.
 */
void print_temp(const uint8_t subzero, uint8_t cel, uint8_t cel_frac_bits) {
  uint8_t buffer[sizeof(int)*8+1];
  uint8_t i, j;

  logc((subzero)?'-':'+');
  logi((int)cel);
  logs_P(".");
  itoa((cel_frac_bits*DS18X20_FRACCONV),buffer,10);
  j=4-strlen(buffer);
  for (i=0;i<j;i++) 
	logs_P("0");
  logs(buffer);
  logs_P(" Celsius");
}

void async_start_read_sensors(void) {
  logs_P( "\r\nReading temperature for all sensors - " ); 	
  if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL ) == DS18X20_OK) {
	logs_P("started measurement.\r\n");	
  } else 
	logs_P("Start meas. failed (short circuit?)");
}

void async_finish_read_sensors(void) {
  uint8_t subzero, cel, cel_frac_bits;
  for (uint8_t i=0; i<nSensors; i++ ) {
	if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
		  &cel, &cel_frac_bits) == DS18X20_OK ) {
	  // Put the reading in the global readings array
	  readings[i].subzero=subzero;
	  readings[i].cel=cel;
	  readings[i].cel_frac_bits=cel_frac_bits;
	}
	else logs_P("CRC Error (lost connection?)");
	//logs_P("\r\n");
  }
}

void sync_read_sensors(void) {
  async_start_read_sensors();
  delay_ms(DS18B20_TCONV_12BIT);
  async_finish_read_sensors();
}

void print_readings(void) {
  for (uint8_t i=0; i<nSensors; i++ ) {
	logs_P("Sensor# ");
	logi((int) i+1);
	logs_P(" = ");
	print_temp(readings[i].subzero, readings[i].cel, readings[i].cel_frac_bits);
	logs_P("\r\n");
  }
}

void enable_status_led(void) {
  // Turn LED on: Pull to GND 
  DS18X20_STATUS_LED_DDR |= (1 << DS18X20_STATUS_LED_PIN);  // Output
}

void disable_status_led(void) {
  //DS18X20_STATUS_LED_PORT |= (1 << DS18X20_STATUS_LED_PIN); // Disable
  DS18X20_STATUS_LED_DDR &= ~(1 << DS18X20_STATUS_LED_PIN);  // Output
}


/********************** main code below *******************/

int main(void) {
  uint8_t i;

  //TODO: Enable watchdog.

  // UART init
  uart_init((UART_BAUD_SELECT((BAUD),F_OSC)));
  // USB code init
  //	wdt_enable(WDTO_1S);
  //odDebugInit();
  DDRD = ~(1 << 2);   /* all outputs except PD2 = INT0 */
  PORTD = 0; /* no pullups on USB pins */
  /* We fake an USB disconnect by pulling D+ and D- to 0 during reset. This is
   * necessary if we had a watchdog reset or brownout reset to notify the host
   * that it should re-enumerate the device. Otherwise the host's and device's
   * concept of the device-ID would be out of sync.
   */
  //DDRB = ~USBMASK;    /* set all pins as outputs except USB */
  //computeOutputStatus();  /* set output status before we do the delay */
  usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
  i = 250;
  while(--i){         /* fake USB disconnect for > 500 ms */
    //wdt_reset();
    _delay_ms(2);
  }
  usbDeviceConnect();
  //TCCR0 = 5;          /* set prescaler to 1/1024 */
  usbInit();
  sei();
  // Print debugging header
  logs_P( "\r\nUSBtemp - temperature at your fingertips\r\n" );
  logs_P( "----------------------------------------\r\n" );
  // Init DS18X20 Connect LED. Allow some sleep time to signal we're up.
  enable_status_led();
  delayloop32(1000000UL);
  // search for the available sensors.
  nSensors = search_sensors();
  // TODO: Use LED to signal errors, i.e. CRC errors.
  if (nSensors == 0)
    disable_status_led();
  else
    enable_status_led();


  logi((int) nSensors);
  logs_P( " DS18X20 Sensor(s) available:\r\n" );
  for (i=0; i<nSensors; i++) {
    logs_P("# in Bus :");
    logi((int) i+1);
    logs_P(" : ");
#ifdef DEBUG
    DS18X20_show_id_uart( &gSensorIDs[i][0], OW_ROMCODE_SIZE );
#endif
    logs_P( "\r\n" );
  }
  sync_read_sensors();
  print_readings();
  // main loop
  uint32_t delay_counter=0;
  enum read_state_t state=IDLE;
  for(;;) {
    //wdt_reset();
    usbPoll();
    delay_ms(MAIN_DELAY_MS); 
    delay_counter+=MAIN_DELAY_MS;
    // statemachine
    if (state == IDLE && delay_counter > READOUT_INTERVAL_MS) {
      // start measurement.
      async_start_read_sensors();
      disable_status_led();
      delay_counter=0;
      state=MEASURING;
    }
    if (state==MEASURING && delay_counter > READOUT_WAIT_MS) {
      // Read the values from the sensors.
      async_finish_read_sensors();
      delay_counter = 0;
      state=UPDATED;
    }
    if (state==UPDATED) {
      // print temperature on uart.
      print_readings();
      enable_status_led();
      delay_counter = 0;
      state=IDLE;
    }
  }
}
