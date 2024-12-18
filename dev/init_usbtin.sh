#!/bin/bash

# This script initializes a USBtin device (http://www.fischl.de/usbtin/) and prepares it for use with KaCanOpen's socket driver.

command_exists(){
	[ -x "$(command -v $1)" ]
}

check_for_command() {
	if ! command_exists $1; then
		echo "ERROR: Please install $1." >&2
		exit 1
	fi
}

check_for_command "git"

if [[ $# -eq 1 ]] ; then
	if ! [[ "$1" =~ ^[0-8]$ ]] ; then
		echo "Error: Baudrate must be given as a number between 0 and 8:"
		echo "0 = 10k"
		echo "1 = 20k"
		echo "2 = 50k"
		echo "3 = 100k"
		echo "4 = 125k"
		echo "5 = 250k"
		echo "6 = 500k"
		echo "7 = 800k"
		echo "8 = 1M"
		exit 1
	fi
	baudrate=$1
else
	echo "You can set the baudrate as first command line argument. Using level 6 = 500k as default."
	echo "Available levels:"
	echo "0 = 10k"
	echo "1 = 20k"
	echo "2 = 50k"
	echo "3 = 100k"
	echo "4 = 125k"
	echo "5 = 250k"
	echo "6 = 500k"
	echo "7 = 800k"
	echo "8 = 1M"
	baudrate=6
fi

SCRIPTDIR=$(dirname "$(readlink -f "$0")")

cd "$SCRIPTDIR"

if [ ! -d "$SCRIPTDIR/can-utils" ]; then
	echo "can-utils not yet available."
	echo "Cloning and building now."
	git clone https://github.com/linux-can/can-utils
	cd "$SCRIPTDIR/can-utils"
	make
fi

cd "$SCRIPTDIR/can-utils"

usb_device=$(dmesg | grep 'USBtin' | tail -1 | sed 's/.* usb \([0-9.-]*\): .*/\1/')
tty_device=$(dmesg | grep "cdc_acm $usb_device" | tail -1 | sed 's/.*ttyACM\([0-9]*\):.*/ttyACM\1/')
sudo killall slcand
sudo ./slcan_attach -f -s$baudrate -nslcan0 -o /dev/$tty_device
sudo ./slcand $tty_device slcan0
sudo ip link set slcan0 up
