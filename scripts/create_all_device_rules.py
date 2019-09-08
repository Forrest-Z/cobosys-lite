#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

print("Restarting udev")

os.system("sudo cp `find ~/ -name MRL`/scripts/robotbase.rules /etc/udev/rules.d")
os.system("sudo cp `find ~/ -name MRL`/scripts/rplidar.rules /etc/udev/rules.d")
os.system("sudo udevadm control --reload-rules")
os.system("sudo service udev restart")
os.system("sudo udevadm trigger")
print("finish")
