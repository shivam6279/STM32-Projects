#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include "secrets.h"

// Serial = UART0 -> H7 UART6. Commands from the H7, one per line, reply ends with OK:
//   scan, wifi, link ("<connected>, <RSSI>")

#define HOSTNAME	"pnp-esp"	// pnp-esp.local, also the OTA name
#define LED_WIFI	4			// ESP_LED0, on = connected
#define LED_BEAT	5			// ESP_LED1, 1Hz blink = running, fast = OTA
#define CMD_LEN		64
#define BAUD		115200		// H7 app

bool was_connected = false;
uint32_t beat_ms = 0;
volatile uint8_t last_reason = 0;
char cmd[CMD_LEN];
uint8_t cmd_len = 0;

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
}

void RunCmd(char *c) {
	if (strcmp(c, "scan") == 0) {
		PrintScan();
	} else if (strcmp(c, "wifi") == 0) {
		PrintWifi();
	} else if (strcmp(c, "link") == 0) {
		bool up = WiFi.status() == WL_CONNECTED;
		Serial.printf("%d, %d\n", up, up ? WiFi.RSSI() : 0);
	} else {
		Serial.printf("ERR unknown command: %s\n", c);
		return;
	}
	Serial.println("OK");
}

void ReadCmds() {
	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\r' || c == '\n') {
			if (cmd_len == 0) continue;
			cmd[cmd_len] = '\0';
			cmd_len = 0;
			RunCmd(cmd);
		} else if (cmd_len < CMD_LEN - 1) {
			cmd[cmd_len++] = c;
		}
	}
}

void OtaSetup() {
	ArduinoOTA.setHostname(HOSTNAME);
	ArduinoOTA.onStart([]() {
		Serial.println("OTA start");
	});
	ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
		digitalWrite(LED_BEAT, (done / 4096) & 1);
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
	Serial.begin(BAUD);
	pinMode(LED_WIFI, OUTPUT);
	pinMode(LED_BEAT, OUTPUT);
	Serial.printf("\nPnP ESP32, built " __DATE__ " " __TIME__ "\n");

	WiFi.mode(WIFI_STA);
	WiFi.setHostname(HOSTNAME);
	PrintScan();

	WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t info) {
		last_reason = info.wifi_sta_disconnected.reason;
		Serial.printf("WiFi disconnected, reason %d, %s\n", last_reason, ReasonName(last_reason));
	}, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
	WiFi.setAutoReconnect(true);
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	Serial.printf("Connecting to %s\n", WIFI_SSID);

	OtaSetup();
}

void loop() {
	bool connected = WiFi.status() == WL_CONNECTED;

	if (connected != was_connected) {
		if (connected) {
			Serial.printf("WiFi up: %s, %s.local, RSSI %d dBm\n", WiFi.localIP().toString().c_str(), HOSTNAME, WiFi.RSSI());
		} else {
			Serial.println("WiFi down");
		}
		digitalWrite(LED_WIFI, connected);
		was_connected = connected;
	}

	ReadCmds();
	ArduinoOTA.handle();

	if (millis() - beat_ms >= 500) {
		beat_ms = millis();
		digitalWrite(LED_BEAT, !digitalRead(LED_BEAT));
	}
	delay(2);
}
