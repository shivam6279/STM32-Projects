#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include "pad.h"
#include "secrets.h"

// Serial = UART0 -> H7 UART6, two channels on it (H7 side: Controller/drivers/esp_link.c).
// DLE (0x10) + a code: DLE C = console bytes follow, DLE E = our channel follows (the default),
// DLE DLE = a 0x10 in the data. Us -> H7: DLE + / DLE - = console client on / gone,
// DLE P = one pad packet follows (pad.cpp). H7 -> us: DLE S / DLE Q = stop / go on sending
// console bytes, DLE ? = send + or -.
// Console and pad frames always end with DLE E, after FRAME_MS of quiet we're back on our channel.
//
// Our channel: commands from the H7, one per line, reply ends with OK:
//   scan, wifi, link ("<connected>, <RSSI>"), pad, pad pair, pad state (pad.cpp)
// Log lines go out on it too, the H7 ignores them.
//
// TCP CON_PORT = the H7's G-code / diags console, raw bytes, same as its USB COM port.
// One client, a new one replaces it.
//
// TCP H7_PORT = flash the H7 (flash_usb.py --wifi). Sends !boot to the H7 app, tells the
// client OK / BUSY / NOANSWER, then pipes to the H7 bootloader at 921600

#define HOSTNAME	"pnp-esp"	// pnp-esp.local, also the OTA name
#define LED_WIFI	4			// ESP_LED0, on = connected, flickers with traffic
#define LED_PAD		5			// ESP_LED1, on = Xbox controller connected, fast blink = pairing
#define ACT_MS		100			// flicker this long after the last traffic
#define CMD_LEN		64
#define BAUD		115200		// H7 app
#define BOOT_BAUD	921600		// H7 bootloader
#define H7_PORT		3233
#define H7_IDLE_MS	20000		// pipe gives up if nothing moves this long
#define CON_PORT	3234
#define CON_CHUNK	128			// console bytes per frame, the H7 stops us with 256 left
#define DLE			0x10
#define FRAME_MS	100

WiFiServer h7_server(H7_PORT);
WiFiServer con_server(CON_PORT);
WiFiClient con;
bool con_up = false;			// console client connected
bool con_hold = false;			// H7 sent DLE S
volatile bool h7_link = false;	// piping to the H7 bootloader, no log on Serial
bool was_connected = false;
uint32_t act_ms = 0;	// last WiFi traffic

void WifiActivity() {
	act_ms = millis();
}

// Also called from the loops that block (H7 flash pipe, OTA)
void LedTask() {
	uint32_t now = millis();
	bool wifi = WiFi.status() == WL_CONNECTED;

	if (now - act_ms < ACT_MS && (now / 25) & 1) wifi = !wifi;
	digitalWrite(LED_WIFI, wifi);
	digitalWrite(LED_PAD, PadPairing() ? (now / 100) & 1 : PadConnected());
}
volatile uint8_t last_reason = 0;
char cmd[CMD_LEN];
uint8_t cmd_len = 0;
char h7_reply[8];				// last "!..." line from the H7
bool rx_con = false, rx_dle = false;
uint32_t rx_ms = 0;

const char *ReasonName(uint8_t r) {
	switch (r) {
		case 0: return "none";
		case 2: return "auth expired";
		case 8: return "left";
		case 15: return "4-way handshake timeout (wrong password?)";
		case 200: return "beacon timeout (lost the AP)";
		case 201: return "AP not found";
		case 202: return "auth failed";
		case 203: return "association failed";
		case 204: return "handshake timeout (wrong password?)";
		default: return "?";
	}
}

const char *StatusName(wl_status_t s) {
	switch (s) {
		case WL_IDLE_STATUS: return "idle";
		case WL_NO_SSID_AVAIL: return "AP not found";
		case WL_CONNECTED: return "connected";
		case WL_CONNECT_FAILED: return "connect failed";
		case WL_CONNECTION_LOST: return "connection lost";
		case WL_DISCONNECTED: return "disconnected";
		default: return "?";
	}
}

const char *AuthName(wifi_auth_mode_t a) {
	switch (a) {
		case WIFI_AUTH_OPEN: return "open";
		case WIFI_AUTH_WEP: return "WEP";
		case WIFI_AUTH_WPA_PSK: return "WPA";
		case WIFI_AUTH_WPA2_PSK: return "WPA2";
		case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
		case WIFI_AUTH_WPA3_PSK: return "WPA3";
		case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
		default: return "other";
	}
}

void PrintScan() {
	int i, n = WiFi.scanNetworks();
	if (n < 0) {
		Serial.printf("Scan failed (%d)\n", n);
		return;
	}
	Serial.printf("%d networks:\n", n);
	for (i = 0; i < n; i++) {
		Serial.printf("  %s, ch %d, %d dBm, %s\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), AuthName(WiFi.encryptionType(i)));
	}
	WiFi.scanDelete();
}

void PrintWifi() {
	wl_status_t s = WiFi.status();
	Serial.printf("SSID: %s\n", WIFI_SSID);
	Serial.printf("Status: %s\n", StatusName(s));
	if (s == WL_CONNECTED) {
		Serial.printf("IP: %s (%s.local)\n", WiFi.localIP().toString().c_str(), HOSTNAME);
		Serial.printf("RSSI: %d dBm, ch %d\n", WiFi.RSSI(), WiFi.channel());
	}
	Serial.printf("Last disconnect: %d, %s\n", last_reason, ReasonName(last_reason));
	Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());
	Serial.printf("Console: port %d, %s\n", CON_PORT, con_up ? con.remoteIP().toString().c_str() : "no client");
}

void RunCmd(char *c) {
	if (strcmp(c, "scan") == 0) {
		PrintScan();
	} else if (strcmp(c, "wifi") == 0) {
		PrintWifi();
	} else if (strcmp(c, "link") == 0) {
		bool up = WiFi.status() == WL_CONNECTED;
		Serial.printf("%d, %d\n", up, up ? WiFi.RSSI() : 0);
	} else if (PadCmd(c)) {
	} else {
		Serial.printf("ERR unknown command: %s\n", c);
		return;
	}
	Serial.println("OK");
}

void SendCode(char c) {
	uint8_t b[2] = { DLE, (uint8_t)c };
	Serial.write(b, 2);
}

// Our channel from the H7: "!..." lines are answers (h7_reply), the rest commands
void CmdChar(char c) {
	if (c == '\r' || c == '\n') {
		if (cmd_len == 0) return;
		cmd[cmd_len] = '\0';
		cmd_len = 0;
		if (cmd[0] == '!') {
			strlcpy(h7_reply, cmd, sizeof(h7_reply));
		} else {
			RunCmd(cmd);
		}
	} else if (cmd_len < CMD_LEN - 1) {
		cmd[cmd_len++] = c;
	}
}

void ConWrite(const uint8_t *buf, int n) {
	if (!con_up) return;
	con.write(buf, n);
	WifiActivity();
}

// Everything from the H7: console bytes -> the TCP client, the rest -> CmdChar
void ReadH7() {
	uint8_t out[256];
	int n = 0;
	uint32_t now = millis();

	while (Serial.available()) {
		uint8_t c = Serial.read();
		if (now - rx_ms > FRAME_MS) rx_con = rx_dle = false;
		rx_ms = now;
		if (rx_dle) {
			rx_dle = false;
			switch (c) {
				case DLE: if (rx_con) out[n++] = DLE; break;
				case 'C': rx_con = true; break;
				case 'E': rx_con = false; break;
				case 'S': con_hold = true; break;
				case 'Q': con_hold = false; break;
				case '?': SendCode(con_up ? '+' : '-'); break;
			}
		} else if (c == DLE) {
			rx_dle = true;
		} else if (rx_con) {
			out[n++] = c;
		} else {
			CmdChar(c);
		}
		if (n == sizeof(out)) {
			ConWrite(out, n);
			n = 0;
		}
	}
	if (n) ConWrite(out, n);
}

// Console client <-> H7, one framed chunk per call
void ConsoleTask() {
	uint8_t buf[CON_CHUNK], frame[4 + 2 * CON_CHUNK];
	WiFiClient c = con_server.available();
	int i, n, len = 0;

	if (c) {
		// Newest wins, the old one may be a dead connection
		if (con_up) con.stop();
		con = c;
		con.setNoDelay(true);
		con_up = true;
		SendCode('+');
	} else if (con_up && !con.connected()) {
		con.stop();
		con_up = false;
		SendCode('-');
	}

	if (!con_up || con_hold || (n = con.available()) <= 0) return;
	n = con.read(buf, min(n, CON_CHUNK));
	if (n <= 0) return;
	WifiActivity();
	frame[len++] = DLE;
	frame[len++] = 'C';
	for (i = 0; i < n; i++) {
		if (buf[i] == DLE) frame[len++] = DLE;
		frame[len++] = buf[i];
	}
	frame[len++] = DLE;
	frame[len++] = 'E';
	Serial.write(frame, len); // one write, so a log line from the WiFi event task can't land inside
}

// Wait for "!ok" / "!busy" from the H7 app
const char *H7BootReply() {
	uint32_t t = millis();

	h7_reply[0] = '\0';
	while (millis() - t < 300) {
		ReadH7();
		if (strcmp(h7_reply, "!ok") == 0) return "OK";
		if (strcmp(h7_reply, "!busy") == 0) return "BUSY";
		delay(1);
	}
	return "NOANSWER";
}

void H7Link() {
	WiFiClient c = h7_server.available();
	uint8_t buf[512];
	uint32_t t;
	const char *status;
	int n;

	if (!c) return;
	c.setNoDelay(true);
	h7_link = true;

	Serial.print("!boot\n");
	status = H7BootReply();
	c.printf("%s\n", status);
	if (strcmp(status, "BUSY") == 0) {
		c.stop();
		h7_link = false;
		return;
	}

	Serial.flush();
	Serial.updateBaudRate(BOOT_BAUD);
	while (Serial.available()) Serial.read();

	t = millis();
	while (c.connected() && millis() - t < H7_IDLE_MS) {
		if ((n = c.available()) > 0) {
			n = c.read(buf, min(n, (int)sizeof(buf)));
			Serial.write(buf, n);
			t = millis();
			WifiActivity();
		}
		if ((n = Serial.available()) > 0) {
			n = Serial.readBytes(buf, min(n, (int)sizeof(buf)));
			c.write(buf, n);
			t = millis();
			WifiActivity();
		}
		LedTask();
		delay(0);
	}
	c.stop();

	Serial.flush();
	Serial.updateBaudRate(BAUD);
	while (Serial.available()) Serial.read();
	rx_con = rx_dle = false;
	con_hold = false;
	h7_link = false;
	SendCode(con_up ? '+' : '-'); // the new app asks too, but it may have asked mid pipe
}

void OtaSetup() {
	ArduinoOTA.setHostname(HOSTNAME);
	ArduinoOTA.onStart([]() {
		Serial.println("OTA start");
	});
	ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
		WifiActivity();
		LedTask();
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("OTA done, rebooting");
	});
	ArduinoOTA.onError([](ota_error_t e) {
		Serial.printf("OTA error %u\n", e);
	});
	ArduinoOTA.begin();
}

void setup() {
	Serial.setRxBufferSize(4096);
	Serial.setTxBufferSize(1024);
	Serial.begin(BAUD);
	SendCode('-'); // the H7 may still think a client is on from before our reset
	pinMode(LED_WIFI, OUTPUT);
	pinMode(LED_PAD, OUTPUT);
	Serial.printf("\nPnP ESP32, built " __DATE__ " " __TIME__ "\n");

	WiFi.mode(WIFI_STA);
	WiFi.setHostname(HOSTNAME);
	PrintScan();

	WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t info) {
		last_reason = info.wifi_sta_disconnected.reason;
		if (!h7_link) Serial.printf("WiFi disconnected, reason %d, %s\n", last_reason, ReasonName(last_reason));
	}, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
	WiFi.setAutoReconnect(true);
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	Serial.printf("Connecting to %s\n", WIFI_SSID);

	OtaSetup();
	PadSetup();
	h7_server.begin();
	h7_server.setNoDelay(true);
	con_server.begin();
	con_server.setNoDelay(true);
}

void loop() {
	bool connected = WiFi.status() == WL_CONNECTED;

	if (connected != was_connected) {
		if (connected) {
			Serial.printf("WiFi up: %s, %s.local, RSSI %d dBm\n", WiFi.localIP().toString().c_str(), HOSTNAME, WiFi.RSSI());
		} else {
			Serial.println("WiFi down");
		}
		was_connected = connected;
	}

	ReadH7();
	ConsoleTask();
	H7Link();
	ArduinoOTA.handle();
	PadLoop();

	LedTask();
	delay(2);
}
