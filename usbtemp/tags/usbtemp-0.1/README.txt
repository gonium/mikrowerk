USBtemp readme file
###################

1. About this project

2. License
All contents of this software are licensed under the terms of the GNU
Public License v2, see LICENSE.txt.

3. Installation

Note: On Linux you must be root in order to query the device. You can
also choose to set the SUID-bit on the usbtemp binary and have it owned
by root. The Makefile contains a target for this:

  make setuid

attempts to modify the usbtemp binary accordingly.

3. Logging temperature
In order to log temperature I use Tobi Oetinger's RRDTool. While this is
a comprehensive tool it also is a complex tool. You might want to read a 
tutorial before delving into the details, I highly recommend this one:

  http://www.cuddletech.com/articles/rrd/

In the bin directory, there are some scripts that read the sensors
and put the data in a round-robin database. 
