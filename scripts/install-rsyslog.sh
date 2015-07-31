#!/bin/bash

# The location of this script's folder
BASE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

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

# Check running as root
if [ "$(whoami)" != "root" ]; then
	bail "Run $0 as root."
fi

# Copy syslog configs
rsyslog=/etc/rsyslog.d
lrotate=/etc/logrotate.d
if [ -d "$rsyslog" ]; then
	log_status "Copying rsyslog configs..."
	cp -v $BASE/../system/etc/rsyslog.d/30-picopter.conf $rsyslog/ || bail "Could not copy rsyslog config!"
	log_status "Restarting rsyslog..."
    service rsyslog restart
else
	log_error "Could not find rsyslog at $rsyslog. Skipping."
fi

if [ -d "$lrotate" ]; then
	log_status "Copying logrotate configs..."
	cp -v $BASE/../system/etc/logrotate.d/picopter.conf $lrotate/ || bail "Could not copy logrotate config!"
else
	log_error "Could not find logrotate at $lrotate. Skipping."
fi

log_note "Install complete."
