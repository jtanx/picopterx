#!/bin/bash

# The executable
SERVER=../code/bin/picopter

# Colourful text
# Red text
function log_error() {
    (echo -e "\e[31m$@\e[0m") 1>&2
}

# Yellow text
function log_status() {
    (echo -e "\e[33m$@\e[0m") 1>&2
}

# Green text
function log_note() {
    (echo -e "\e[32m$@\e[0m") 1>&2
}

# Red text and exit
function bail () {
    (echo -e "\e[31m\e[1m!!! Fatal error: ${@}\e[0m") 1>&2
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

	log_status "(Re)starting ServoBlaster..."
	service servoblaster stop || log_error "Failed to stop ServoBlaster"
	service servoblaster start || log_error "Failed to start ServoBlaster"
	#service servoblaster restart || log_error "Failed to restart ServoBlaster"

	log_status "(Re)starting gpsd..."
	service gpsd restart || log_error "Failed to restart gpsd"

	log_status "(Re)starting the server..."
	$SERVER
	error=$?
	if [ "$error" == "0" ]; then
		exit 0
        fi
	fails=$(( $fails + 1 ))
done

log_error "Server had too many Fatal Errors ($fails); giving up."
exit $fails
