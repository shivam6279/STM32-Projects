#!/usr/bin/env python3
# PnP tool: serial terminal (USB COM, ST-Link UART, or WiFi through the ESP32) + build / flash.
# Usage: double click (.pyw = no console window), or pythonw tools/pnp_tool.pyw
#
# Flash method, unless overridden in the Flash menu:
#   H7:  the terminal's own connection if it's open (WiFi / board USB / ST-Link UART), else
#        board USB > ESP answers a ping (WiFi) > ST-Link plugged in (SWD). Bootloader = SWD only.
#   ESP: OTA if it answers a ping, else through the H7 (board USB).
# The ESP host is resolved with our own mDNS query (esp_host.py), Windows often can't do .local.
# The terminal lets go of its port while a flash runs and reconnects after.
import ctypes
import datetime
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog

import serial
from serial.tools import list_ports

from esp_host import Resolve

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))	# Controller/
ESP_DIR = os.path.join(os.path.dirname(ROOT), "ESP32")
BUNDLES = os.path.join(os.environ.get("LOCALAPPDATA", ""), "stm32cube", "bundles")
CMAKE = os.path.join(BUNDLES, "cmake", "4.0.1+st.3", "bin", "cmake.exe")
PROGRAMMER = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
# Console python next to the pythonw running us, so the scripts get the same install (pyserial, pio)
PYTHON = os.path.join(os.path.dirname(sys.executable), "python.exe")
VID, APP_PID, BOOT_PID = 0xCAFE, 0x4001, 0x4002
WIFI = "WiFi (ESP32 console)"
CON_PORT = 3234
RECONNECT_S = 15
DETECT_MS = 3000
NO_WINDOW = 0x08000000	# CREATE_NO_WINDOW, no console popping up per command

H7_METHODS = ["Auto", "USB", "WiFi", "ST-Link"]
ESP_METHODS = ["Auto", "OTA", "Through H7"]

FONT_UI = ("Segoe UI", 10)
FONT_TERM = ("Consolas", 10)


# ---- what's plugged in ----

def BoardPort(pid):
	for p in list_ports.comports():
		if p.vid == VID and p.pid == pid:
			return p.device
	return None


def StlinkPort():
	for p in list_ports.comports():
		if "STLink" in (p.description or "") or "ST-Link" in (p.description or ""):
			return p.device
	return None


def Ping(host):
	try:
		r = subprocess.run(["ping", "-n", "1", "-w", "500", host], capture_output=True, creationflags=NO_WINDOW, timeout=3)
		return r.returncode == 0 and b"TTL=" in r.stdout
	except Exception:
		return False


# term: the terminal's open link (port name or WIFI), None if closed
def Detect(host, term, h7_over, esp_over, boot):
	app, loader, stlink = BoardPort(APP_PID), BoardPort(BOOT_PID), StlinkPort()
	usb = app or loader
	ip = Resolve(host)	# Windows can't always resolve pnp-esp.local itself
	wifi = bool(ip) and Ping(ip)
	where = host if ip == host else f"{host}, {ip}"

	if boot:
		h7 = ("ST-Link", "bootloader needs SWD")
	elif h7_over != "Auto":
		h7 = (h7_over, "override")
	elif term == WIFI:
		h7 = ("WiFi", "terminal is on WiFi")
	elif term and usb and term.startswith(usb):
		h7 = ("USB", f"terminal on {usb}")
	elif term and stlink and term.startswith(stlink):
		h7 = ("ST-Link", "terminal on the ST-Link UART")
	elif usb:
		h7 = ("USB", usb + (" (in bootloader)" if loader and not app else ""))
	elif wifi:
		h7 = ("WiFi", where)
	elif stlink:
		h7 = ("ST-Link", "ST-Link plugged in")
	else:
		h7 = (None, "no connection found")

	if esp_over != "Auto":
		esp = (esp_over, "override")
	elif wifi:
		esp = ("OTA", where)
	elif app:
		esp = ("Through H7", app)
	else:
		esp = (None, "ESP not reachable")
	return {"h7": h7, "esp": esp, "wifi": wifi, "usb": usb, "stlink": stlink, "host": host, "ip": ip}


# ---- commands ----

def H7Build():
	return (ROOT, [CMAKE, "--build", "build"])


def H7Flash(method, host, boot):
	if method == "USB":
		return (ROOT, [PYTHON, "-u", "tools/flash_usb.py"])
	if method == "WiFi":
		return (ROOT, [PYTHON, "-u", "tools/flash_usb.py", "--wifi", host])
	cmd = [PROGRAMMER, "-c", "port=SWD", "mode=UR"]
	if boot:
		cmd += ["-w", "build/pnp_boot.elf", "-v"]
	return (ROOT, cmd + ["-w", "build/pnp_ctrl.elf", "-v", "-rst"])


def Pio(*args):
	return (ESP_DIR, [PYTHON, "-m", "platformio", "run"] + list(args))


def EspFlash(method, host):
	if method == "OTA":
		return Pio("-e", "ota", "-t", "upload", "--upload-port", host)
	return Pio("-e", "usb", "-t", "upload")


class App:
	def __init__(self, root):
		self.root = root
		self.q = queue.Queue()
		self.link = None
		self.link_name = None
		self.busy = False
		self.detecting = False
		self.found = None
		self.history = []
		self.hist_i = 0
		self.line_start = True

		root.title("PnP Tool")
		k = root.winfo_fpixels("1i") / 96 # DPI aware: sizes are real pixels now
		root.geometry(f"{int(1150 * k)}x{int(820 * k)}")
		root.minsize(int(900 * k), int(560 * k))
		root.option_add("*Font", FONT_UI)
		style = ttk.Style()
		if "vista" in style.theme_names():
			style.theme_use("vista")
		style.configure(".", font=FONT_UI)
		style.configure("Go.TButton", font=("Segoe UI Semibold", 11), padding=(16, 6))
		style.configure("Head.TLabel", font=("Segoe UI Semibold", 10))
		style.configure("Dim.TLabel", foreground="#666")
		style.configure("Ok.TLabel", foreground="#1a7f37")
		style.configure("Bad.TLabel", foreground="#c62828")

		self.h7_over = tk.StringVar(value="Auto")
		self.esp_over = tk.StringVar(value="Auto")
		self.stamps = tk.BooleanVar(value=True)
		self.autoscroll = tk.BooleanVar(value=True)
		self.echo = tk.BooleanVar(value=False)
		self.host = tk.StringVar(value="pnp-esp.local")
		self.h7_build = tk.BooleanVar(value=True)
		self.h7_flash = tk.BooleanVar(value=True)
		self.h7_boot = tk.BooleanVar(value=False)
		self.esp_build = tk.BooleanVar(value=False)
		self.esp_flash = tk.BooleanVar(value=False)

		self.Menus()
		top = ttk.Frame(root, padding=(10, 8, 10, 4))
		top.pack(fill=tk.X)
		self.ConnectionBox(top).pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 8))
		self.FlashBox(top).pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

		panes = ttk.PanedWindow(root, orient=tk.VERTICAL)
		panes.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)
		panes.add(self.TerminalBox(panes), weight=4)
		panes.add(self.LogBox(panes), weight=1)

		self.status = ttk.Label(root, text="Not connected", style="Dim.TLabel", padding=(10, 2, 10, 6))
		self.status.pack(fill=tk.X)

		for v in (self.h7_build, self.h7_flash, self.h7_boot, self.esp_build, self.esp_flash):
			v.trace_add("write", lambda *a: self.UpdateGo())
		for v in (self.h7_over, self.esp_over, self.h7_boot):
			v.trace_add("write", lambda *a: self.DetectNow())

		self.Refresh()
		self.UpdateGo()
		root.protocol("WM_DELETE_WINDOW", self.Quit)
		root.after(30, self.Poll)
		self.DetectNow()

	# ---- layout ----

	def Menus(self):
		m = tk.Menu(self.root)
		f = tk.Menu(m, tearoff=0)
		f.add_command(label="Save terminal...", command=self.Save)
		f.add_separator()
		f.add_command(label="Exit", command=self.Quit)
		m.add_cascade(label="File", menu=f)

		fl = tk.Menu(m, tearoff=0)
		h7 = tk.Menu(fl, tearoff=0)
		for x in H7_METHODS:
			h7.add_radiobutton(label=x, variable=self.h7_over, value=x)
		esp = tk.Menu(fl, tearoff=0)
		for x in ESP_METHODS:
			esp.add_radiobutton(label=x, variable=self.esp_over, value=x)
		fl.add_cascade(label="H7 flash via", menu=h7)
		fl.add_cascade(label="ESP32 flash via", menu=esp)
		fl.add_separator()
		fl.add_command(label="Detect now", command=self.DetectNow)
		m.add_cascade(label="Flash", menu=fl)

		v = tk.Menu(m, tearoff=0)
		v.add_checkbutton(label="Timestamps", variable=self.stamps)
		v.add_checkbutton(label="Auto-scroll", variable=self.autoscroll)
		v.add_checkbutton(label="Local echo", variable=self.echo)
		v.add_separator()
		v.add_command(label="Clear terminal", command=self.Clear)
		m.add_cascade(label="View", menu=v)
		self.root.config(menu=m)

	def ConnectionBox(self, parent):
		box = ttk.LabelFrame(parent, text=" Terminal connection ", padding=(10, 6))
		ttk.Label(box, text="Port").grid(row=0, column=0, sticky="w")
		self.port = ttk.Combobox(box, width=34, state="readonly")
		self.port.grid(row=0, column=1, columnspan=2, sticky="we", padx=6)
		self.port.bind("<Button-1>", lambda e: self.Refresh())
		ttk.Label(box, text="ESP host").grid(row=1, column=0, sticky="w", pady=(6, 0))
		ttk.Entry(box, textvariable=self.host, width=20).grid(row=1, column=1, sticky="w", padx=6, pady=(6, 0))
		self.conn_btn = ttk.Button(box, text="Connect", width=12, command=self.Toggle)
		self.conn_btn.grid(row=1, column=2, sticky="e", padx=6, pady=(6, 0))
		self.conn_state = ttk.Label(box, text="● Not connected", style="Dim.TLabel")
		self.conn_state.grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))
		return box

	def FlashBox(self, parent):
		box = ttk.LabelFrame(parent, text=" Build & flash ", padding=(10, 6))
		ttk.Label(box, text="", style="Head.TLabel").grid(row=0, column=0)
		ttk.Label(box, text="Build", style="Head.TLabel").grid(row=0, column=1, padx=8)
		ttk.Label(box, text="Flash", style="Head.TLabel").grid(row=0, column=2, padx=8)
		ttk.Label(box, text="Flash via", style="Head.TLabel").grid(row=0, column=3, sticky="w", padx=8)

		ttk.Label(box, text="H7").grid(row=1, column=0, sticky="w")
		ttk.Checkbutton(box, variable=self.h7_build).grid(row=1, column=1)
		ttk.Checkbutton(box, variable=self.h7_flash).grid(row=1, column=2)
		self.h7_via = ttk.Label(box, text="detecting...", style="Dim.TLabel")
		self.h7_via.grid(row=1, column=3, sticky="w", padx=8)

		ttk.Label(box, text="ESP32").grid(row=2, column=0, sticky="w")
		ttk.Checkbutton(box, variable=self.esp_build).grid(row=2, column=1)
		ttk.Checkbutton(box, variable=self.esp_flash).grid(row=2, column=2)
		self.esp_via = ttk.Label(box, text="detecting...", style="Dim.TLabel")
		self.esp_via.grid(row=2, column=3, sticky="w", padx=8)

		ttk.Checkbutton(box, text="Include H7 bootloader (ST-Link)", variable=self.h7_boot).grid(
			row=3, column=0, columnspan=4, sticky="w", pady=(6, 0))
		self.go = ttk.Button(box, text="Build & Flash", style="Go.TButton", command=self.Go)
		self.go.grid(row=0, column=4, rowspan=4, sticky="e", padx=(16, 0))
		box.columnconfigure(3, weight=1)
		return box

	def TerminalBox(self, parent):
		box = ttk.Frame(parent)
		bar = ttk.Frame(box)
		bar.pack(fill=tk.X, pady=(0, 4))
		ttk.Label(bar, text="Terminal", style="Head.TLabel").pack(side=tk.LEFT)
		ttk.Button(bar, text="Save...", command=self.Save).pack(side=tk.RIGHT)
		ttk.Button(bar, text="Clear", command=self.Clear).pack(side=tk.RIGHT, padx=4)
		ttk.Checkbutton(bar, text="Local echo", variable=self.echo).pack(side=tk.RIGHT, padx=8)
		ttk.Checkbutton(bar, text="Auto-scroll", variable=self.autoscroll,
			command=lambda: self.autoscroll.get() and self.term.see(tk.END)).pack(side=tk.RIGHT, padx=8)
		ttk.Checkbutton(bar, text="Timestamps", variable=self.stamps).pack(side=tk.RIGHT, padx=8)

		frame = ttk.Frame(box)
		frame.pack(fill=tk.BOTH, expand=True)
		self.term = tk.Text(frame, font=FONT_TERM, bg="white", fg="black", insertbackground="black",
			relief=tk.SOLID, borderwidth=1, wrap=tk.CHAR, state=tk.DISABLED, padx=6, pady=4)
		sb = ttk.Scrollbar(frame, command=self.term.yview)
		self.term.config(yscrollcommand=sb.set)
		sb.pack(side=tk.RIGHT, fill=tk.Y)
		self.term.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
		self.term.tag_config("ts", foreground="#8a8a8a", elide=not self.stamps.get())
		# Every line always gets a stamp, the toggle only shows / hides them (old lines too)
		self.stamps.trace_add("write", lambda *_: self.term.tag_config("ts", elide=not self.stamps.get()))
		self.term.tag_config("tx", foreground="#1f5fbf")
		self.term.tag_config("note", foreground="#a05a00")

		ibar = ttk.Frame(box)
		ibar.pack(fill=tk.X, pady=(4, 0))
		self.entry = ttk.Entry(ibar, font=FONT_TERM)
		self.entry.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=2)
		self.entry.bind("<Return>", self.Send)
		self.entry.bind("<Up>", lambda e: self.History(-1))
		self.entry.bind("<Down>", lambda e: self.History(1))
		ttk.Button(ibar, text="Send", command=self.Send).pack(side=tk.LEFT, padx=(6, 0))
		self.entry.focus()
		return box

	def LogBox(self, parent):
		box = ttk.Frame(parent)
		ttk.Label(box, text="Build / flash output", style="Head.TLabel").pack(anchor="w", pady=(4, 4))
		frame = ttk.Frame(box)
		frame.pack(fill=tk.BOTH, expand=True)
		self.log = tk.Text(frame, font=("Consolas", 9), height=8, bg="#f6f7f9", fg="#222",
			relief=tk.SOLID, borderwidth=1, state=tk.DISABLED, padx=6, pady=4)
		sb = ttk.Scrollbar(frame, command=self.log.yview)
		self.log.config(yscrollcommand=sb.set)
		sb.pack(side=tk.RIGHT, fill=tk.Y)
		self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
		self.log.tag_config("cmd", foreground="#666")
		self.log.tag_config("head", foreground="#1f5fbf", font=("Consolas", 9, "bold"))
		self.log.tag_config("err", foreground="#c62828", font=("Consolas", 9, "bold"))
		self.log.tag_config("ok", foreground="#1a7f37", font=("Consolas", 9, "bold"))
		return box

	# ---- ports ----

	def PortNames(self):
		names = [WIFI]
		for p in sorted(list_ports.comports(), key=lambda p: p.device):
			desc = p.description or ""
			if p.vid == VID and p.pid == APP_PID:
				desc = "PnP board USB"
			elif p.vid == VID and p.pid == BOOT_PID:
				desc = "PnP bootloader"
			elif "STLink" in desc:
				desc = "ST-Link UART"
			else:
				desc = desc.replace(f" ({p.device})", "")
			names.append(f"{p.device} - {desc}")
		return names

	def Refresh(self):
		names = self.PortNames()
		cur = self.port.get()
		self.port["values"] = names
		if cur in names:
			return
		usb = [n for n in names if "PnP board" in n]
		self.port.set(usb[0] if usb else WIFI)

	# The ESP's IP from the last detect (it runs every DETECT_MS), else the name as typed
	def EspAddr(self):
		host = self.host.get().strip()
		if self.found and self.found["host"] == host and self.found["ip"]:
			return self.found["ip"]
		return host

	def Open(self, name):
		if name == WIFI:
			return serial.serial_for_url(f"socket://{self.EspAddr()}:{CON_PORT}", timeout=0.1)
		return serial.Serial(name.split(" ")[0], 115200, timeout=0.1)

	def Connect(self, name, quiet=False):
		try:
			self.link = self.Open(name)
		except Exception as e:
			if not quiet:
				self.Status(f"Can't open {name}: {e}")
			return False
		self.link_name = name
		threading.Thread(target=self.Reader, args=(self.link,), daemon=True).start()
		self.conn_btn.config(text="Disconnect")
		self.conn_state.config(text=f"● Connected: {name}", style="Ok.TLabel")
		self.Status(f"Connected to {name}")
		self.DetectNow()
		return True

	def Disconnect(self):
		link, self.link = self.link, None
		if link:
			try:
				link.close()
			except Exception:
				pass
		self.conn_btn.config(text="Connect")
		self.conn_state.config(text="● Not connected", style="Dim.TLabel")

	def Toggle(self):
		if self.link:
			self.Disconnect()
			self.link_name = None
			self.Status("Disconnected")
			self.DetectNow()
		else:
			self.Connect(self.port.get())

	def Reader(self, link):
		while self.link is link:
			try:
				d = link.read(4096)
			except Exception as e:
				if self.link is link:
					self.q.put(("lost", str(e)))
				return
			if d:
				self.q.put(("rx", d.decode(errors="replace").replace("\r", "")))

	def Reconnect(self, name, deadline):
		# The board's COM port / the ESP takes a few seconds to come back after a reset
		if self.link or self.busy or name is None:
			return
		self.Refresh()
		if self.Connect(name, quiet=True):
			self.TermNote(f"[reconnected to {name}]\n")
			return
		if time.time() < deadline:
			self.Status(f"Waiting for {name}...")
			self.root.after(500, self.Reconnect, name, deadline)
		else:
			self.Status(f"{name} didn't come back")

	# ---- terminal ----

	def Send(self, _=None):
		line = self.entry.get()
		self.entry.delete(0, tk.END)
		if line and (not self.history or self.history[-1] != line):
			self.history.append(line)
		self.hist_i = len(self.history)
		if not self.link:
			self.Status("Not connected")
			return
		try:
			self.link.write((line + "\n").encode())
		except Exception as e:
			self.Status(f"Send failed: {e}")
			return
		if self.echo.get():
			self.TermText(line + "\n", "tx")

	def History(self, step):
		if not self.history:
			return "break"
		self.hist_i = max(0, min(len(self.history), self.hist_i + step))
		self.entry.delete(0, tk.END)
		if self.hist_i < len(self.history):
			self.entry.insert(0, self.history[self.hist_i])
		return "break"

	# Timestamp at the start of every line, as it arrives. Always there, hidden when they're off
	def TermText(self, text, tag=None):
		t = self.term
		t.config(state=tk.NORMAL)
		stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3] + "  "
		for part in text.splitlines(keepends=True):
			if self.line_start:
				t.insert(tk.END, stamp, "ts")
			t.insert(tk.END, part, tag)
			self.line_start = part.endswith("\n")
		if int(t.index("end-1c").split(".")[0]) > 50000:
			t.delete("1.0", "10000.0")
		t.config(state=tk.DISABLED)
		if self.autoscroll.get():
			t.see(tk.END)

	def TermNote(self, text):
		if not self.line_start:
			text = "\n" + text
		self.TermText(text, "note")

	def Clear(self):
		self.term.config(state=tk.NORMAL)
		self.term.delete("1.0", tk.END)
		self.term.config(state=tk.DISABLED)
		self.line_start = True

	# Terminal text as shown: the stamps left out while they're hidden
	def TermPlain(self):
		t = self.term
		if self.stamps.get():
			return t.get("1.0", "end-1c")
		out, pos = [], "1.0"
		r = t.tag_ranges("ts")
		for i in range(0, len(r), 2):
			out.append(t.get(pos, r[i]))
			pos = r[i + 1]
		out.append(t.get(pos, "end-1c"))
		return "".join(out)

	def Save(self):
		name = filedialog.asksaveasfilename(defaultextension=".txt",
			initialfile=datetime.datetime.now().strftime("pnp_%Y%m%d_%H%M%S.txt"),
			filetypes=[("Text", "*.txt"), ("All files", "*.*")])
		if name:
			with open(name, "w", encoding="utf-8") as f:
				f.write(self.TermPlain())
			self.Status(f"Saved {name}")

	def Log(self, text, tag=None):
		self.log.config(state=tk.NORMAL)
		self.log.insert(tk.END, text, tag)
		self.log.config(state=tk.DISABLED)
		self.log.see(tk.END)

	def Status(self, text):
		self.status.config(text=text)

	# ---- detect ----

	def DetectNow(self):
		if self.detecting:
			return
		self.detecting = True
		args = (self.host.get(), self.link_name if self.link else None, self.h7_over.get(),
			self.esp_over.get(), self.h7_boot.get())
		threading.Thread(target=lambda: self.q.put(("detect", Detect(*args))), daemon=True).start()

	def DetectTick(self):
		if not self.busy:
			self.DetectNow()
			if not self.link:
				self.Refresh()
		self.root.after(DETECT_MS, self.DetectTick)

	def ShowDetect(self, d):
		self.found = d
		for label, (method, why), over in ((self.h7_via, d["h7"], self.h7_over), (self.esp_via, d["esp"], self.esp_over)):
			head = "Auto → " if over.get() == "Auto" else ""
			if method:
				label.config(text=f"{head}{method}   ({why})", style="Ok.TLabel")
			else:
				label.config(text=f"{head}{why}", style="Bad.TLabel")

	# ---- build / flash ----

	def UpdateGo(self):
		build = self.h7_build.get() or self.esp_build.get()
		flash = self.h7_flash.get() or self.esp_flash.get() or self.h7_boot.get()
		text = "Build & Flash" if build and flash else "Build" if build else "Flash" if flash else "Nothing selected"
		self.go.config(text=text)
		self.go.state(["!disabled"] if (build or flash) and not self.busy else ["disabled"])

	def Go(self):
		if self.busy or not self.found:
			return
		host = self.EspAddr()
		boot = self.h7_boot.get()
		h7_method = self.found["h7"][0]
		esp_method = self.found["esp"][0]
		h7_flash = self.h7_flash.get() or boot
		steps = []

		# H7 first: if the link protocol changed, an old H7 can't take the new ESP's traffic
		if self.h7_build.get():
			steps.append(("Build H7", H7Build()))
		if h7_flash:
			if not h7_method:
				self.Log("H7: no way to flash it found (board USB, ESP WiFi or ST-Link)\n", "err")
				return
			steps.append((f"Flash H7 over {h7_method}", H7Flash(h7_method, host, boot)))
		if self.esp_flash.get():
			if not esp_method:
				self.Log("ESP32: not reachable over WiFi, and no board USB to go through\n", "err")
				return
			steps.append((f"Build + flash ESP32 ({esp_method})", EspFlash(esp_method, host)))
		elif self.esp_build.get():
			steps.append(("Build ESP32", Pio("-e", "ota")))

		was = self.link_name if self.link else None
		if h7_flash or self.esp_flash.get():
			self.Disconnect() # the flash needs the port (USB) or the ESP (WiFi)
			if was:
				self.TermNote("[disconnected for flashing]\n")
		self.busy = True
		self.UpdateGo()
		threading.Thread(target=self.RunSteps, args=(steps, was), daemon=True).start()

	def RunSteps(self, steps, was):
		t0 = time.time()
		ok = True
		for name, (cwd, cmd) in steps:
			self.q.put(("log", (f"\n=== {name} ===\n", "head")))
			self.q.put(("log", ("> " + " ".join(cmd) + "\n", "cmd")))
			self.q.put(("status", name + "..."))
			try:
				p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, creationflags=NO_WINDOW)
			except Exception as e:
				self.q.put(("log", (f"Can't run it: {e}\n", "err")))
				ok = False
				break
			for raw in p.stdout:
				self.q.put(("log", (raw.decode(errors="replace").replace("\r", ""), None)))
			if p.wait() != 0:
				self.q.put(("log", (f"{name}: failed (exit {p.returncode})\n", "err")))
				ok = False
				break
		dt = time.time() - t0
		if ok:
			self.q.put(("log", (f"Done in {dt:.1f}s\n", "ok")))
		self.q.put(("done", (was, ok, dt)))

	# ---- main loop ----

	def Poll(self):
		try:
			while True:
				kind, d = self.q.get_nowait()
				if kind == "rx":
					self.TermText(d)
				elif kind == "log":
					self.Log(*d)
				elif kind == "status":
					self.Status(d)
				elif kind == "detect":
					self.detecting = False
					self.ShowDetect(d)
				elif kind == "lost":
					name = self.link_name
					self.Disconnect()
					if name == WIFI:
						# The ESP takes one console client, a new one replaces the old: someone else
						# connected. Reconnecting would just kick them off in turn
						self.TermNote("[WiFi console closed: another client took it, or the ESP restarted]\n")
						self.Status("WiFi console closed, Connect to take it back")
					else:
						self.TermNote(f"[connection lost: {d}]\n")
						self.root.after(1000, self.Reconnect, name, time.time() + RECONNECT_S)
				elif kind == "done":
					was, ok, dt = d
					self.busy = False
					self.UpdateGo()
					self.Status(f"Done in {dt:.1f}s" if ok else "Failed, see the output below")
					if was:
						self.root.after(1500, self.Reconnect, was, time.time() + RECONNECT_S)
					self.DetectNow()
		except queue.Empty:
			pass
		self.root.after(30, self.Poll)

	def Quit(self):
		self.Disconnect()
		self.root.destroy()


if __name__ == "__main__":
	# Without this Windows bitmap-scales the whole window on a high DPI screen: blurry text
	try:
		ctypes.windll.shcore.SetProcessDpiAwareness(1)
	except Exception:
		pass
	root = tk.Tk()
	app = App(root)
	root.after(DETECT_MS, app.DetectTick)
	root.mainloop()
