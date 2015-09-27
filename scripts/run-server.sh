#!/bin/bash

# The location of this script's folder
BASE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The executable
SERVER=$BASE/../code/bin/picopter
CONFIG=$BASE/../code/configs/auto.json

# Colourful text
# Red text
function log_error() {
    echo -ne "\e[31m"; echo "$@"; echo -ne "\e[0m"
}

# Yellow text
function log_status() {
    echo -ne "\e[33m"; echo "$@"; echo -ne "\e[0m"
}

# Green text
function log_note() {
    echo -ne "\e[32m"; echo "$@"; echo -ne "\e[0m"
}

function bail () {
    echo -ne "\e[31m\e[1m"; echo "!!! Fatal error: ${@}"; echo -ne "\e[0m"
    exit 1
}

# Check existence of program
if [ ! -e "$SERVER" ]; then
	bail "Rebuild the server."
fi

# Check running as root
if [ "$(whoami)" != "root" ]; then
	bail "Run $0 as root."
fi

# Run the program with parameters
# NOTE: Having the program automatically restart itself after a *FATAL ERROR* doesn't seem like such a good idea now
# (Some things that call Fatal happen under circumstances where they will just keep calling Fatal every time the program starts)
# Change the number of fails to 1 for now. We can potentially use different error codes for different types of errors, but that seems overkill.
fails=0
while [ $fails -lt 1 ]; do
	if [ $fails -gt 0 ]; then
		log_status "Restarting server after Fatal Error #$fails"
	fi;

	log_status "Rotating the system logs..."
	logrotate -f /etc/logrotate.d/picopter.conf
	log_status "(Re)starting the server..."
    #$SERVER $CONFIG
	gdb -ex 'handle SIGILL nostop' -ex "set args $CONFIG" -ex run $SERVER
	error=$?
	if [ "$error" == "0" ]; then
		exit 0
        fi
	fails=$(( $fails + 1 ))
done

log_error "Server had too many Fatal Errors ($fails); giving up."
exit $fails
