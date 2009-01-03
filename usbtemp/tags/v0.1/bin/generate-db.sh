#!/bin/sh
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
if [ -e "$USBTEMP_CONFIG" ]; then
  . "$USBTEMP_CONFIG"
else
  echo "Cannot find config file $USBTEMP_CONFIG."
  exit 1
fi

rrdtool_bin=`which rrdtool`
if [ ! -x "$rrdtool_bin" ]; then
  echo "Cannot find rrdtool executable. Exiting."
  exit 1
fi

# Calculate parameters for the database.
interval=300 # 300 secs = 5 minutes
timeout=`echo "scale=0; (2*$interval + 0.2*$interval)/1" | bc -l`
mintemp=-25.0
maxtemp=50.0
avgnumsample=12
xfactor=0.5
avgnumdays=1825 # 5 years
realnumdays=365 # 1 year
secondsperday=`echo "scale=0; 60*60*24" | bc -l`
avgnumsamples=`echo "scale=0; ($avgnumdays * ($secondsperday / ($interval * $avgnumsample)))/1" | bc -l`
realnumsamples=`echo "scale=0; ($realnumdays * ($secondsperday / ($interval * 1)))/1" | bc -l`

echo "Creating database $TEMP_DATABASE with parameters"
echo "- measurement interval: $interval s"
echo "- timeout: $timeout s"
echo "- mintemp: $mintemp, maxtemp: $maxtemp"
echo "- min/max average sample: $avgnumsample samples per data entry."
echo "- will store $avgnumdays days of average samples ($avgnumsamples samples)"
echo "- will store $realnumdays days of real samples ($realnumsamples samples)"


# see also: http://www.cuddletech.com/articles/rrd/ar01s02.html
rrdtool create $TEMP_DATABASE \
  --start N --step $interval \
  DS:probe1-temp:GAUGE:$timeout:$mintemp:$maxtemp \
  DS:probe2-temp:GAUGE:$timeout:$mintemp:$maxtemp \
  RRA:MIN:$xfactor:$avgnumsample:$avgnumsamples \
  RRA:MAX:$xfactor:$avgnumsample:$avgnumsamples \
  RRA:AVERAGE:$xfactor:1:$realnumsamples

