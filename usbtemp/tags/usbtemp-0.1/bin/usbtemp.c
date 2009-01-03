/***
 * USBtemp host control program
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
 * Based on powerSwitch.c by Christian Starkjohann
 * of Objective Development Software GmbH (2005)
 * (see http://www.obdev.at/avrusb)
 * Some routines by Martin Thomas, see inline comments.
 */

/*
   General Description:
   This program queries the USBtemp hardware.
   It must be linked with libusb, a library for accessing the USB bus from
   Linux, FreeBSD, Mac OS X and other Unix operating systems. Libusb can be
   obtained from http://libusb.sourceforge.net/.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */

#define USBDEV_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   0x05DC  /* Obdev's free shared PID */
/* Use obdev's generic shared VID/PID pair and follow the rules outlined
 * in firmware/usbdrv/USBID-License.txt.
 */

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

/* These are the vendor specific SETUP commands implemented by our USB device */
#define USBTEMP_CMD_TEST			0
#define USBTEMP_CMD_NO_SENSORS		1
#define USBTEMP_CMD_QUERY_SENSOR    2
#define USBTEMP_CMD_GET_TEMP	    3

/* DS18X20 specific values (see datasheet) */
#define DS18S20_ID 0x10
#define DS18B20_ID 0x28
#define DS18X20_FRACCONV 625// constant to convert the fraction bits to cel*(10^-4)
#define DS18X20_ID_LENGTH 6 // The hardware ID is 6 bytes long.

static void usage(char *name)
{
  fprintf(stderr, "usbtemp commandline client\n");
  fprintf(stderr, "usage:\n");
  fprintf(stderr, "  %s sensors\n", name);
  fprintf(stderr, "  %s temp <sensorid>\n", name);
}


static int  usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen)
{
  char    buffer[256];
  int     rval, i;

  if((rval = usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, buffer, sizeof(buffer), 1000)) < 0)
    return rval;
  if(buffer[1] != USB_DT_STRING)
    return 0;
  if((unsigned char)buffer[0] < rval)
    rval = (unsigned char)buffer[0];
  rval /= 2;
  /* lossy conversion to ISO Latin1 */
  for(i=1;i<rval;i++){
    if(i > buflen)  /* destination buffer overflow */
      break;
    buf[i-1] = buffer[2 * i];
    if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
      buf[i-1] = '?';
  }
  buf[i-1] = 0;
  return i-1;
}

static int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName)
{
  struct usb_bus      *bus;
  struct usb_device   *dev;
  usb_dev_handle      *handle = NULL;
  int                 errorCode = USB_ERROR_NOTFOUND;
  static int          didUsbInit = 0;

  if(!didUsbInit){
    didUsbInit = 1;
    usb_init();
  }
  usb_find_busses();
  usb_find_devices();
  for(bus=usb_get_busses(); bus; bus=bus->next){
    for(dev=bus->devices; dev; dev=dev->next){
      if(dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product){
        char    string[256];
        int     len;
        handle = usb_open(dev); /* we need to open the device in order to query strings */
        if(!handle){
          errorCode = USB_ERROR_ACCESS;
          fprintf(stderr, "Warning: cannot open USB device: %s\n", usb_strerror());
          continue;
        }
        if(vendorName == NULL && productName == NULL){  /* name does not matter */
          break;
        }
        /* now check whether the names match: */
        len = usbGetStringAscii(handle, dev->descriptor.iManufacturer, 0x0409, string, sizeof(string));
        if(len < 0){
          errorCode = USB_ERROR_IO;
          fprintf(stderr, "Warning: cannot query manufacturer for device: %s\n", usb_strerror());
        }else{
          errorCode = USB_ERROR_NOTFOUND;
          /* fprintf(stderr, "seen device from vendor ->%s<-\n", string); */
          if(strcmp(string, vendorName) == 0){
            len = usbGetStringAscii(handle, dev->descriptor.iProduct, 0x0409, string, sizeof(string));
            if(len < 0){
              errorCode = USB_ERROR_IO;
              fprintf(stderr, "Warning: cannot query product for device: %s\n", usb_strerror());
            }else{
              errorCode = USB_ERROR_NOTFOUND;
              /* fprintf(stderr, "seen product ->%s<-\n", string); */
              if(strcmp(string, productName) == 0)
                break;
            }
          }
        }
        usb_close(handle);
        handle = NULL;
      }
    }
    if(handle)
      break;
  }
  if(handle != NULL){
    errorCode = 0;
    *device = handle;
  }
  return errorCode;
}

/* Developed by Martin Thomas, DS18X20 demo */
void printhex_nibble(const unsigned char b) {
  unsigned char  c = b & 0x0f;
  if (c>9) c += 'A'-10;
  else c += '0';
  fprintf(stderr, "%c", c);
} 

/* Developed by Martin Thomas, DS18X20 demo */
void printhex_byte(const unsigned char  b) {
  printhex_nibble(b>>4);
  printhex_nibble(b);
} 

/***
 * retrieve the number of attached sensors.
 */
unsigned char getNumSensors(usb_dev_handle *handle) {
  unsigned char       buffer[8];
  int                 nBytes;
  nBytes = usb_control_msg(handle, 
      USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
      USBTEMP_CMD_NO_SENSORS,		  // Command ID
      0,							  // Value 
      0,							  // Index 
      (char *) buffer, 
      sizeof(buffer), 
      5000);
  if(nBytes < 1){
    if(nBytes < 0)
      fprintf(stderr, "USB error: %s\n", usb_strerror());
    fprintf(stderr, "only %d bytes status received\n", nBytes);
    exit(1);
  }
  return buffer[0];
}
void getTemperature(usb_dev_handle *handle, char* sensor_name) {
  unsigned char       buffer[8];
  int                 nBytes;
  fprintf(stderr, "searching sensor with id %s - ", sensor_name);
  unsigned char amount=getNumSensors(handle);
  unsigned char sensorId=0;
  unsigned char found=1;
  for (int i=0; i<amount; i++) { 
    nBytes = usb_control_msg(handle, 
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
        USBTEMP_CMD_QUERY_SENSOR,	  // Command ID
        i,							  // Value 
        0,							  // Index 
        (char *) buffer, 
        sizeof(buffer), 
        5000);
    if(nBytes < 1+DS18X20_ID_LENGTH){
      if(nBytes < 0)
        fprintf(stderr, "USB error: %s\n", usb_strerror());
      fprintf(stderr, "only %d bytes status received\n", nBytes);
      exit(1);
    }
    found=1;
    for(int j=0; j<DS18X20_ID_LENGTH; j++) {
      char id_part = buffer[j+1];
      // Convert to string - nibble-wise hex representation.
      unsigned char lsn = id_part & 0x0f;
      if (lsn>9) lsn += 'A'-10;
      else lsn += '0';
      unsigned char msn = (id_part>>4) & 0x0f;
      if (msn>9) msn += 'A'-10;
      else msn += '0';
      // compare this byte to the given ID represenation
      if (! (msn == sensor_name[2*j] && lsn == sensor_name[2*j+1])) {
        found = 0;
        break;
      }
    }
    if (found) {
      sensorId=i;
      break;
    }
  }
  if (! found) {
    fprintf(stderr, "Error: sensor not found.\n");
    exit(-2);
  }
  fprintf(stderr, "using sensor handle %u\n", sensorId);
  nBytes = usb_control_msg(handle, 
      USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
      USBTEMP_CMD_GET_TEMP,		  // Command ID
      sensorId,					  // sensor id 
      0,							  // Index 
      (char *) buffer, 
      sizeof(buffer), 
      5000);
  if(nBytes < 4){
    if(nBytes < 0)
      fprintf(stderr, "USB error: %s\n", usb_strerror());
    fprintf(stderr, "only %d bytes status received\n", nBytes);
    exit(1);
  }
  unsigned char sensor = buffer[0];
  unsigned char subzero, cel, cel_frac_bits;
  subzero=buffer[1];
  cel=buffer[2];
  cel_frac_bits=buffer[3];
  fprintf(stderr, "reading sensor %d (°C): ", sensor);
  if (subzero)
    printf("-");
  else 
    printf("+");
  fprintf(stdout, "%d.", cel);
  fprintf(stdout, "%04d\n", cel_frac_bits*DS18X20_FRACCONV);
}

void printSensors(usb_dev_handle *handle) {
  unsigned char       buffer[8];
  int                 nBytes;
  unsigned char amount=getNumSensors(handle);
  fprintf(stderr, "%d sensor(s) found, querying\n", amount);
  for (int i=0; i<amount; i++) { 
    nBytes = usb_control_msg(handle, 
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
        USBTEMP_CMD_QUERY_SENSOR,	  // Command ID
        i,							  // Value 
        0,							  // Index 
        (char *) buffer, 
        sizeof(buffer), 
        5000);
    if(nBytes < 2){
      if(nBytes < 0)
        fprintf(stderr, "USB error: %s\n", usb_strerror());
      fprintf(stderr, "only %d bytes status received\n", nBytes);
      exit(1);
    }
    // TODO: Clean up, use struct to get clean response!
    // buffer[0]: Type of sensor
    // buffer[1-7]: 6-byte hardware id of sensor.
    fprintf(stderr, "sensor %d: ID ", i);
    for(int j=1; j<7; j++) {
      char id_part = buffer[j];
      printhex_byte(id_part);
    }
    fprintf(stderr, " type: ");
    if (buffer[0] == DS18S20_ID)
      fprintf(stderr, "(DS18S20)\n");
    else if (buffer[0] == DS18B20_ID)
      fprintf(stderr, "(DS18B20)\n");
  }
}

/**
 * Main routine. Parse commandline args and trigger actions.
 */
int main(int argc, char **argv) {
  usb_dev_handle      *handle = NULL;
  unsigned char       buffer[8];
  int                 nBytes;

  if(argc < 2){
    usage(argv[0]);
    exit(1);
  }
  usb_init();
  if(usbOpenDevice(&handle, USBDEV_SHARED_VENDOR, "gonium.net", USBDEV_SHARED_PRODUCT, "usbtemp") != 0){
    fprintf(stderr, "Could not find USB device \"usbtemp\" with vid=0x%x pid=0x%x\n", USBDEV_SHARED_VENDOR, USBDEV_SHARED_PRODUCT);
    exit(1);
  }
  /* We have searched all devices on all busses for our USB device above. Now
   * try to open it and perform the vendor specific control operations for the
   * function requested by the user.
   */
  if(strcmp(argv[1], "test") == 0){
    int i, v, r;
    /* The test consists of writing 1000 random numbers to the device and checking
     * the echo. This should discover systematic bit errors (e.g. in bit stuffing).
     */
    fprintf(stderr, "testing stability of usb connection. please wait.\n");
    for(i=0;i<1000;i++){
      v = rand() & 0xffff;
      nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBTEMP_CMD_TEST, v, 0, (char *)buffer, sizeof(buffer), 5000);
      if(nBytes < 2){
        if(nBytes < 0)
          fprintf(stderr, "USB error: %s\n", usb_strerror());
        fprintf(stderr, "only %d bytes received in iteration %d\n", nBytes, i);
        fprintf(stderr, "value sent = 0x%x\n", v);
        exit(1);
      }
      r = buffer[0] | (buffer[1] << 8);
      if(r != v){
        fprintf(stderr, "data error: received 0x%x instead of 0x%x in iteration %d\n", r, v, i);
        exit(1);
      }
    }
    printf("test succeeded\n");
  } else if (strcmp(argv[1], "sensors") == 0) {
    //fprintf(stderr, "checking attached sensors\n");
    printSensors(handle);

  } else if (strcmp(argv[1], "temp") == 0) {
    if (argc < 3 || strlen(argv[2]) != 2*DS18X20_ID_LENGTH) {
      fprintf(stderr, "usage: usbtemp temp <SensorID>\n");
      exit(-1);
    }
    getTemperature(handle, argv[2]);
  } else {
    usage(argv[0]);
    exit(1);
  }
  usb_close(handle);
  return 0;
}
