#!/bin/bash

# Intended to be executed on local machine
# Scans the network, checks if tortoisebotpro is connected or not and initiates the ssh connection
# Usage => bash connect_tortoisebotpro.sh amrita-pro 192.168.0.139

# Positional argument received by the user, might change
USRNAME=$1
IP=$2

# Assuming single IP is assigned
LOCAL_IP=$(hostname -I)

# Assuming subnet mask /24
NETWORK_ADDRESS=$(echo $LOCAL_IP | awk -F. '{print $1"."$2"."$3".0/24"}')

is_device_up() {
    echo "Scanning the local network..."
    local result=$(nmap -sn $NETWORK_ADDRESS | grep $IP)
    if [[ -n $result ]]; then
        return 0
    else
        return 1
    fi
}


while true; do
    if is_device_up; then
        echo "TortoiseBotPro is up! Initiating SSH connection..."
	ssh $USRNAME@$IP
        break
    else
        echo "TortoiseBotPro not connected to the network! Trying again..."
    fi
done
