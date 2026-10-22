# Find the ESP32's IP from its name. Windows' own resolver sometimes never asks the network
# for .local names (no mDNS query goes out at all, it just gets NXDOMAIN from the router),
# so for .local we send the mDNS query ourselves, out of every interface, then fall back to the
# OS resolver. Used by pnp_tool.pyw and flash_usb.py --wifi.
import ipaddress
import socket
import struct
import time

MDNS_ADDR = ("224.0.0.251", 5353)
MDNS_TRIES = 2
MDNS_WAIT = 0.5		# per try, seconds
TYPE_A = 1

last_ip = {}		# host -> last IP that resolved, used if a lookup fails


def IsIp(host):
	try:
		ipaddress.IPv4Address(host)
		return True
	except ValueError:
		return False


def LocalIps():
	ips = set()
	try:
		for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
			ips.add(info[4][0])
	except OSError:
		pass
	return [ip for ip in ips if not ip.startswith("127.") and not ip.startswith("169.254.")]


# Past a (maybe compressed) name, returns the offset after it
def SkipName(data, i):
	while i < len(data):
		n = data[i]
		if n == 0:
			return i + 1
		if n & 0xC0 == 0xC0:
			return i + 2
		i += n + 1
	return len(data)


# First A record in an mDNS answer, None if there isn't one
def ParseA(data):
	if len(data) < 12 or not data[2] & 0x80:
		return None
	qd, an, ns, ar = struct.unpack(">HHHH", data[4:12])
	i = 12
	for _ in range(qd):
		i = SkipName(data, i) + 4
	for _ in range(an + ns + ar):
		i = SkipName(data, i)
		if i + 10 > len(data):
			return None
		rtype, _, _, rdlen = struct.unpack(">HHIH", data[i:i + 10])
		i += 10
		if rtype == TYPE_A and rdlen == 4:
			return socket.inet_ntoa(data[i:i + 4])
		i += rdlen
	return None


# One-shot ("legacy") mDNS query: sent from a random port, so the answer comes back unicast
def MdnsResolve(host):
	name = b"".join(bytes([len(p)]) + p.encode() for p in host.rstrip(".").split(".")) + b"\0"
	query = struct.pack(">HHHHHH", 0x504E, 0, 1, 0, 0, 0) + name + struct.pack(">HH", TYPE_A, 1)
	socks = []
	for ip in LocalIps() or [""]:
		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
			s.bind((ip, 0))
			if ip:
				s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(ip))
			s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
			s.setblocking(False)
			socks.append(s)
		except OSError:
			pass
	try:
		for _ in range(MDNS_TRIES):
			for s in socks:
				try:
					s.sendto(query, MDNS_ADDR)
				except OSError:
					pass
			t = time.time()
			while time.time() - t < MDNS_WAIT:
				for s in socks:
					try:
						data, _ = s.recvfrom(1500)
					except OSError:
						continue
					ip = ParseA(data)
					if ip:
						return ip
				time.sleep(0.01)
	finally:
		for s in socks:
			s.close()
	return None


def OsResolve(host):
	try:
		return socket.gethostbyname(host)
	except OSError:
		return None


# IP for host, or the last one that worked, or None
def Resolve(host):
	host = host.strip()
	if not host or IsIp(host):
		return host or None
	if host.lower().endswith(".local"):
		ip = MdnsResolve(host) or OsResolve(host)
	else:
		ip = OsResolve(host)
	if ip:
		last_ip[host] = ip
		return ip
	return last_ip.get(host)


if __name__ == "__main__":
	import sys
	h = sys.argv[1] if len(sys.argv) > 1 else "pnp-esp.local"
	print(f"{h}: {Resolve(h)}")
