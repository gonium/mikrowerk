#!/bin/bash
# Check for installation directory, load configuration.
USBTEMP_CMD="$USBTEMP_HOME/bin/usbtemp"
USBTEMP_CONFIG="$USBTEMP_HOME/etc/config.sh"

if [ -d "$USBTEMP_HOME" ]; then
  if [ ! -x "$USBTEMP_CMD" ]; then
    echo "Environment variable USBTEMP_HOME doesn't point to the USBtemp installation."
    exit 1
  fi
else
  echo "Env variable USBTEMP_HOME doesn't point to a directory."
  exit 1
fi

if [ -e "$USBTEMP_CONFIG" ]; then
  . "$USBTEMP_CONFIG"
else
  echo "Cannot find config file $USBTEMP_CONFIG."
  exit 1
fi

# No config below.
rrdtool_bin=`which rrdtool`
if [ ! -x "$rrdtool_bin" ]; then
  echo "Cannot find rrdtool executable. Exiting."
  exit 1
fi

# Global variables
temperature1="UNDEF"
temperature2="UNDEF"

# Query the available sensors.
if [ -n $SENSOR1_ID ]; then
  # TODO: Check exit code. If != 0 then retry.
  query_cmd="$USBTEMP_CMD temp $SENSOR1_ID"
  temperature1=`$query_cmd 2> /dev/null`
else
  echo "You must define the ID of the builtin sensor in the config file"
  echo "$USBTEMP_CONFIG"
  exit 1
fi

if [ -n "$SENSOR2_ID" -a \( "$SENSOR2_ID" != "" \) ]; then
  query_cmd="$USBTEMP_CMD temp $SENSOR2_ID 2>/dev/null"
  temperature2=`$query_cmd 2> /dev/null`
fi

echo "Found readings: temp1=$temperature1 temp2=$temperature2"
# Now, log the readings to the RRDTool database.
$rrdtool_bin update $TEMP_DATABASE "N:$temperature1:$temperature2"
