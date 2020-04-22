#!/usr/bin/env python
from roboclaw import Roboclaw
import serial

rc = Roboclaw("/dev/ttyS0",115200)
rc.Open()
rc.ReadVersion(128)

print("Connected")

