#!/usr/bin/env python3
# Find PnP controllers on the ethernet link (run on the Pi, or a PC on the same wire).
# Broadcasts on UDP 41077, prints every "PnP ip=... mac=..." reply.
# Usage: pnp_discover.py [--dev eth0] [--iface-ip 10.77.0.1] [--timeout 1.5]
# Linux: 255.255.255.255 goes out the default route (the Pi's wlan0) and a socket bound to a
# unicast address never gets broadcasts, so we tie the socket to the device instead (default eth0).
# Windows: no SO_BINDTODEVICE, --iface-ip picks the interface.
import argparse
import os
import socket
import time

PORT = 41077


def Main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--dev", default="eth0" if os.path.exists("/sys/class/net/eth0") else "",
		help="Linux: send / listen on this device only (default eth0 if there is one)")
	ap.add_argument("--iface-ip", default="", help="Windows: bind to this local address (pick the ethernet interface)")
	ap.add_argument("--timeout", type=float, default=1.5)
	args = ap.parse_args()

	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	if args.dev:
		s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, args.dev.encode())
		s.bind(("", PORT))			# replies are broadcast to this port
	else:
		s.bind((args.iface_ip, PORT))
	s.settimeout(0.2)

	s.sendto(b"PnP?", ("255.255.255.255", PORT))
	found = set()
	t = time.time()
	while time.time() - t < args.timeout:
		try:
			data, addr = s.recvfrom(512)
		except socket.timeout:
			continue
		msg = data.decode(errors="replace").strip()
		if msg.startswith("PnP ") and msg not in found:
			found.add(msg)
			print(f"{addr[0]}: {msg}")
	if not found:
		print("No PnP controller answered")


if __name__ == "__main__":
	Main()
