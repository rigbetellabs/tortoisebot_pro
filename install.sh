#!/bin/bash

# Usage : source install.sh

# # Install udev rules
RULE_CONTENT=$(cat <<EOL
SUBSYSTEM=="tty", KERNELS=="1-2", SYMLINK+="esp"
SUBSYSTEM=="tty", KERNELS=="1-1", SYMLINK+="lidar"
EOL
)

RULE_FILE="/etc/udev/rules.d/rbl.rules"

echo "$RULE_CONTENT" | sudo tee "$RULE_FILE" > /dev/null

sudo udevadm control --reload-rules
sudo usermod -a -G dialout $USER
sudo usermod -a -G input $USER

# No error checking
echo "USB Ports configured!.."
sleep 3
# Package installation
echo -e "\nChecking for dependencies, installing if necessary..."
cat requirements.txt | xargs sudo apt-get install -y 

echo "Installation Successful"