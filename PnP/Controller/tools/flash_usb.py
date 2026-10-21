#!/usr/bin/env python3
# Flash the app through our bootloader (boot/), over USB or WiFi (through the ESP32).
# USB: sends "boot" on the debug port (or the ST-Link UART if USB is busy), waits for PnP-Boot.
# WiFi: pnp-esp.local:3233, the ESP sends "!boot" and passes bytes to the bootloader on UART6.
# Usage: flash_usb.py [bin] [--port COMx] [--wifi [host]] [--erase-settings]
# Sector 3 holds the settings (app/settings.c): never erased unless --erase-settings.
import argparse
import socket
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
SETTINGS_SECTOR = 3
APP_SECTORS = 2		# 1-2, the app is linked to 256K so the settings survive
BOOT_VERSION = 2
WIFI_PORT = 3233
TRIES = 8
ERRORS = {1: "unknown command", 2: "bad arguments", 3: "erase failed", 4: "write failed",
	5: "image CRC mismatch", 6: "first 32 bytes never sent", 7: "readback mismatch", 8: "packet CRC"}


class SerialLink:
	def __init__(self, port):
		self.s = serial.Serial(port, 115200)

	def write(self, b):
		self.s.write(b)

	def read(self, n, timeout):
		self.s.timeout = timeout
		return self.s.read(n)

	def flush(self):
		self.s.reset_input_buffer()

	def close(self):
		self.s.close()


class TcpLink:
	def __init__(self, sock):
		self.s = sock

	def write(self, b):
		self.s.sendall(b)

	def read(self, n, timeout):
		self.s.settimeout(timeout)
		buf = b""
		try:
			while len(buf) < n:
				d = self.s.recv(n - len(buf))
				if not d:
					break
				buf += d
		except socket.timeout:
			pass
		return buf

	def flush(self):
		self.s.setblocking(False)
		try:
			while self.s.recv(4096):
				pass
		except (BlockingIOError, socket.error):
			pass
		self.s.setblocking(True)

	def close(self):
		self.s.close()


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
			# port went away = it already rebooted
			if Ports(BOOT_PID) or "does not recognize" in str(e):
				print(f"Sent boot on {port}")
				return True
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


def Cmd(link, cmd, payload=b"", reply_len=0, timeout=5):
	pkt = struct.pack("<cH", cmd, len(payload)) + payload
	pkt += struct.pack("<I", zlib.crc32(pkt))
	err = "no reply"
	for _ in range(TRIES):
		link.write(pkt)
		r = link.read(1, timeout)
		if r == b"K":
			data = link.read(reply_len, timeout) if reply_len else b""
			if len(data) == reply_len:
				return data
			err = "short reply"
		elif r == b"E":
			code = link.read(1, 1)
			code = code[0] if code else 0
			err = ERRORS.get(code, f"error {code}")
			if code == 5:
				break # image is wrong, resending won't help
		elif r:
			err = f"garbage reply {r!r}"
		# bootloader drops a half packet after 100ms
		time.sleep(0.15)
		link.flush()
	sys.exit(f"Bootloader error on '{cmd.decode()}': {err}")


def OpenUsb(port_arg):
	if Ports(BOOT_PID):
		print("Bootloader already running")
	else:
		if not SendBoot([port_arg] if port_arg else DebugPorts()):
			sys.exit("No debug port to send boot on")
	port = WaitFor(lambda: Ports(BOOT_PID), 15, "PnP-Boot port")[0]
	time.sleep(0.3)
	return SerialLink(port), port


def OpenWifi(host):
	try:
		sock = socket.create_connection((host, WIFI_PORT), timeout=5)
	except OSError as e:
		sys.exit(f"Can't reach {host}:{WIFI_PORT}: {e}")
	sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
	link = TcpLink(sock)
	# ESP says how the app took !boot
	status = link.read(64, 3).split(b"\n")[0].strip().decode(errors="replace")
	if status == "BUSY":
		sys.exit("H7 is busy (motion running), not rebooting it")
	if status == "OK":
		time.sleep(0.5) # app resets into the bootloader
	else:
		print(f"App didn't answer ({status or 'nothing'}), trying the bootloader anyway")
	link.flush()
	return link, f"{host}:{WIFI_PORT}"


def Main():
	ap = argparse.ArgumentParser()
	ap.add_argument("bin", nargs="?", default="build/pnp_ctrl.bin")
	ap.add_argument("--port", help="debug port to send boot on (default: find it)")
	ap.add_argument("--wifi", nargs="?", const="pnp-esp.local", metavar="HOST",
		help="flash through the ESP32 (default pnp-esp.local)")
	ap.add_argument("--erase-settings", action="store_true",
		help="also wipe the saved settings, the app comes up on its defaults")
	args = ap.parse_args()

	image = open(args.bin, "rb").read()
	image += b"\xFF" * (-len(image) % FLASH_WORD)
	crc = zlib.crc32(image)

	link, where = OpenWifi(args.wifi) if args.wifi else OpenUsb(args.port)

	t = time.time()
	magic, ver, base, size = struct.unpack("<4sHII", Cmd(link, b"I", reply_len=14, timeout=1))
	if magic != b"PNPB":
		sys.exit(f"{where} isn't the PnP bootloader")
	if ver != BOOT_VERSION:
		sys.exit(f"Bootloader v{ver}, this script needs v{BOOT_VERSION} (flash pnp_boot.elf over SWD)")
	if len(image) + FLASH_WORD > size:
		sys.exit(f"Image is {len(image)} bytes, app area is {size} (minus 32 for the info word)")
	if len(image) + FLASH_WORD > APP_SECTORS * SECTOR:
		sys.exit(f"Image is {len(image)} bytes, it would run into the settings sector")
	print(f"Bootloader v{ver} on {where}, app at 0x{base:08X}, {len(image)} bytes, CRC {crc:08X}")

	# + 32 for the info word after the image
	for sector in range(1, 1 + (len(image) + FLASH_WORD + SECTOR - 1) // SECTOR):
		Cmd(link, b"E", struct.pack("<I", sector), timeout=10)
	if args.erase_settings:
		Cmd(link, b"E", struct.pack("<I", SETTINGS_SECTOR), timeout=10)
		print("Settings erased")
	print(f"Erased in {time.time() - t:.1f}s")

	for off in range(0, len(image), CHUNK):
		Cmd(link, b"W", struct.pack("<I", off) + image[off:off + CHUNK])
	Cmd(link, b"V", struct.pack("<II", len(image), crc))
	print(f"Written and verified in {time.time() - t:.1f}s")

	Cmd(link, b"R")
	link.close()

	if args.wifi:
		print("App starting")
	else:
		port = WaitFor(lambda: Ports(APP_PID), 10, "app COM port")[0]
		print(f"App running on {port}")


if __name__ == "__main__":
	Main()
