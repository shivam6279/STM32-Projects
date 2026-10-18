#!/usr/bin/env python3
# Flash the app over USB through our bootloader (boot/).
# Sends "boot" on the debug port (USB, or the ST-Link UART if USB is busy), waits for the
# PnP-Boot COM port, erases, writes, CRC checks, then starts the app.
# Usage: flash_usb.py [bin] [--port COMx]
import argparse
import struct
import sys
import time
import zlib

import serial
from serial.tools import list_ports

VID = 0xCAFE
APP_PID, BOOT_PID = 0x4001, 0x4002
CHUNK = 1024
FLASH_WORD = 32
SECTOR = 128 * 1024
ERRORS = {1: "unknown command", 2: "bad arguments", 3: "erase failed", 4: "write failed",
	5: "CRC mismatch", 6: "first 32 bytes never sent", 7: "readback mismatch"}


def Ports(pid):
	return [p.device for p in list_ports.comports() if p.vid == VID and p.pid == pid]


def DebugPorts():
	ports = Ports(APP_PID)
	ports += [p.device for p in list_ports.comports() if "STLink Virtual COM" in (p.description or "")]
	return ports


def SendBoot(ports):
	for port in ports:
		try:
			with serial.Serial(port, 115200, timeout=0.2) as s:
				# 'x' first: ends a diags stream (-s), which would otherwise swallow the command.
				# Then M997 does it from G-code mode. If that didn't take (diags mode, or a build
				# without M997), "diags" + "boot" does
				s.write(b"x\r")
				time.sleep(0.15)
				s.write(b"\rM997\r")
				time.sleep(0.15)
				s.write(b"\rdiags\r")
				time.sleep(0.15)
				s.write(b"\rboot\r")
				time.sleep(0.2)
			print(f"Sent boot on {port}")
			return True
		except serial.SerialException as e:
			print(f"Couldn't use {port}: {e}")
	return False


def WaitFor(check, timeout, what):
	t = time.time()
	while time.time() - t < timeout:
		r = check()
		if r:
			return r
		time.sleep(0.2)
	sys.exit(f"Timed out waiting for {what}")


def Cmd(s, cmd, payload=b"", reply_len=0, timeout=5):
	s.write(struct.pack("<cH", cmd, len(payload)) + payload)
	s.timeout = timeout
	r = s.read(1)
	if r == b"K":
		return s.read(reply_len) if reply_len else b""
	if r == b"E":
		code = s.read(1)
		code = code[0] if code else 0
		sys.exit(f"Bootloader error on '{cmd.decode()}': {ERRORS.get(code, code)}")
	sys.exit(f"No reply to '{cmd.decode()}'")


def Main():
	ap = argparse.ArgumentParser()
	ap.add_argument("bin", nargs="?", default="build/pnp_ctrl.bin")
	ap.add_argument("--port", help="debug port to send boot on (default: find it)")
	args = ap.parse_args()

	image = open(args.bin, "rb").read()
	image += b"\xFF" * (-len(image) % FLASH_WORD)
	crc = zlib.crc32(image)

	if Ports(BOOT_PID):
		print("Bootloader already running")
	else:
		if not SendBoot([args.port] if args.port else DebugPorts()):
			sys.exit("No debug port to send boot on")
	port = WaitFor(lambda: Ports(BOOT_PID), 10, "PnP-Boot port")[0]
	time.sleep(0.3)

	t = time.time()
	with serial.Serial(port, 115200) as s:
		magic, ver, base, size = struct.unpack("<4sHII", Cmd(s, b"I", reply_len=14))
		if magic != b"PNPB":
			sys.exit(f"{port} isn't the PnP bootloader")
		if len(image) > size:
			sys.exit(f"Image is {len(image)} bytes, app area is {size}")
		print(f"Bootloader v{ver} on {port}, app at 0x{base:08X}, {len(image)} bytes, CRC {crc:08X}")

		for sector in range(1, 1 + (len(image) + SECTOR - 1) // SECTOR):
			Cmd(s, b"E", struct.pack("<I", sector), timeout=10)
		print(f"Erased in {time.time() - t:.1f}s")

		for off in range(0, len(image), CHUNK):
			Cmd(s, b"W", struct.pack("<I", off) + image[off:off + CHUNK])
		Cmd(s, b"V", struct.pack("<II", len(image), crc))
		print(f"Written and verified in {time.time() - t:.1f}s")

		Cmd(s, b"R")

	port = WaitFor(lambda: Ports(APP_PID), 10, "app COM port")[0]
	print(f"App running on {port}")


if __name__ == "__main__":
	Main()
