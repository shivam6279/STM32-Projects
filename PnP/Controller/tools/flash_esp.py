#!/usr/bin/env python3
# Flash the ESP32 through the H7: diags "esp boot" + "esp bridge", esptool on the same port,
# then +++ and "esp reset". Needs esptool (pip install esptool)
# Usage: flash_esp.py [--port COMx] [--baud 921600] <esptool command>
#   flash_esp.py write-flash 0x0 merged.bin
# No command = chip-id. On the ST-Link UART use --baud 115200
import argparse
import importlib.util
import subprocess
import sys
import time

import serial
from serial.tools import list_ports

VID, APP_PID = 0xCAFE, 0x4001
GUARD = 1.2	# s, 1s quiet around the +++


def FindPort():
	ports = [p.device for p in list_ports.comports() if p.vid == VID and p.pid == APP_PID]
	if not ports:
		sys.exit("No PnP-Diags port, is the app running?")
	return ports[0]


def ReadUntil(s, expect, timeout=3):
	buf = b""
	t = time.time()
	while time.time() - t < timeout:
		buf += s.read(s.in_waiting or 1)
		if expect.encode() in buf:
			return True
	return False


def Diags(s, cmd, expect):
	s.reset_input_buffer()
	s.write(cmd.encode() + b"\r")
	if not ReadUntil(s, expect):
		sys.exit(f"No '{expect}' after '{cmd}'")


def EndBridge(s):
	time.sleep(GUARD)
	s.reset_input_buffer()
	s.write(b"+++")
	time.sleep(GUARD)
	return ReadUntil(s, "Bridge closed")


def StartBridge(port):
	with serial.Serial(port, 115200, timeout=0.1) as s:
		# 'x' ends a diags stream, "diags" gets out of G-code mode. No answer = left in the bridge
		s.write(b"x\r")
		time.sleep(0.15)
		s.write(b"\rdiags\r")
		time.sleep(0.3)
		s.reset_input_buffer()
		s.write(b"esp\r")
		if not ReadUntil(s, "UART6:", 1):
			print("No answer, ending a leftover bridge")
			EndBridge(s)
		Diags(s, "esp", "UART6:")
		Diags(s, "esp boot", "download mode")
		Diags(s, "esp bridge", "Bridge open")


def StopBridge(port):
	with serial.Serial(port, 115200, timeout=0.1) as s:
		if not EndBridge(s):
			print("Bridge didn't close")
			return
		Diags(s, "esp reset", "ESP32 reset")
		s.write(b"exit\r") # back to G-code
	print("ESP32 reset, running the new app")


def Main():
	ap = argparse.ArgumentParser(usage="flash_esp.py [--port COMx] [--baud 921600] <esptool command>")
	ap.add_argument("--port", help="PnP-Diags port (default: find it)")
	ap.add_argument("--baud", type=int, default=921600, help="esptool baud after its stub loads")
	args, cmd = ap.parse_known_args()

	if importlib.util.find_spec("esptool") is None:
		sys.exit("esptool isn't installed: pip install esptool")
	import esptool
	v5 = int(esptool.__version__.split(".")[0]) >= 5
	no_reset = "no-reset" if v5 else "no_reset"
	if not cmd:
		cmd = ["chip-id" if v5 else "chip_id"]

	port = args.port or FindPort()
	StartBridge(port)
	print(f"ESP32 in download mode, bridged on {port}", flush=True)

	try:
		r = subprocess.call([sys.executable, "-m", "esptool", "--chip", "esp32c3", "--port", port,
			"--baud", str(args.baud), "--before", no_reset, "--after", no_reset] + cmd)
	finally:
		StopBridge(port)
	sys.exit(r)


if __name__ == "__main__":
	Main()
