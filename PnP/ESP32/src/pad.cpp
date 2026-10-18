#include <Arduino.h>
#include <Preferences.h>

// The library's debug output goes here instead of a serial port (ours is the H7 link): kept are the
// lines about the paired address and the scan / connect events, the rest of the BLE noise is dropped.
// pad log prints it
class PadLog : public Print {
public:
	size_t write(uint8_t c) override;
	void Show();
	char match[18] = "";	// address to keep advertisements of
private:
	static const int LINES = 40, LEN = 160;
	char line[LEN];
	int len = 0, keep_next = 0;
	char ring[LINES][LEN];
	uint32_t at[LINES];
	uint16_t count[LINES];
	int head = 0, used = 0;
	void Add(const char *s);
};
extern PadLog pad_log;
#define XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL pad_log

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include "pad.h"

namespace xbox = XboxSeriesXControllerESP32_asukiaaa;

#define DLE			0x10
#define PAD_LEN		18	// packet bytes, see Controller/drivers/esp.h
#define BEAT_MS		20	// resend this often while connected, the H7 lets go after 150ms of nothing
#define MIN_GAP_MS	5	// at most 200 packets/s when things move
#define SUPERVISION_MS	500	// controller gone (switched off, out of range) after this much silence

xbox::Core pad;
Preferences prefs;
ble_addr_t pad_addr = {};	// the paired one, all 0 = none
bool pairing = false;
bool pad_was_connected = false;
uint8_t last_sent[PAD_LEN];
uint32_t sent_ms = 0;
uint8_t seq = 0;

PadLog pad_log;

void PadLog::Add(const char *s) {
	int last = (head + LINES - 1) % LINES;
	if (used && strcmp(ring[last], s) == 0) {
		count[last]++; // same advertisement again: count it, don't flood
		return;
	}
	strlcpy(ring[head], s, LEN);
	at[head] = millis();
	count[head] = 1;
	head = (head + 1) % LINES;
	if (used < LINES) used++;
}

size_t PadLog::write(uint8_t c) {
	if (c == '\r') return 1;
	if (c != '\n') {
		if (len < LEN - 1) line[len++] = c;
		return 1;
	}
	static const char *events[] = { "Found", "Try", "Connected", "New client", "Max", "Done", "RSSI", "no conn", "Address type", "Link" };
	bool keep = false;

	line[len] = '\0';
	len = 0;
	// An advertisement prints over several lines: keep the paired one's up to its uuidService line
	if (strncmp(line, "Advertised", 10) == 0) keep_next = match[0] && strstr(line, match);
	if (keep_next) {
		keep = true;
		if (strncmp(line, "uuidService", 11) == 0) keep_next = 0;
	}
	for (const char *e : events) keep |= strncmp(line, e, strlen(e)) == 0;
	keep |= strstr(line, "Disconnected") != nullptr || (match[0] && strstr(line, match));
	if (keep) Add(line);
	return 1;
}

void PadLog::Show() {
	int i, k;
	if (!used) Serial.println("Pad log: empty");
	for (i = 0; i < used; i++) {
		k = (head + LINES - used + i) % LINES;
		Serial.printf("%7.3f %s", at[k] / 1000.0, ring[k]);
		if (count[k] > 1) Serial.printf("  (x%u)", count[k]);
		Serial.println();
	}
	used = 0;
}

const char *AddrStr(const ble_addr_t &a) {
	static char s[18];
	snprintf(s, sizeof(s), "%02x:%02x:%02x:%02x:%02x:%02x", a.val[5], a.val[4], a.val[3], a.val[2], a.val[1], a.val[0]);
	return s;
}

bool HaveAddr() {
	for (int i = 0; i < 6; i++) {
		if (pad_addr.val[i]) return true;
	}
	return false;
}

void StartPairing() {
	pairing = true;
	// Target stays set (all 0 if none), the flag also lets in anything in pairing mode.
	// clearTargetAddress() frees the pointer without clearing it, so don't use it.
	pad.setTargetAddress(pad_addr);
	pad.setIgnoreTargetAddressToConnectToUnkownSearchingDevice(true);
}

bool PadConnected() {
	return pad_was_connected;
}

bool PadPairing() {
	return pairing && !pad_was_connected;
}

void PadSetup() {
#ifdef PAD_OFF
	Serial.println("Pad: BLE off in this build (PAD_OFF)");
	return;
#endif
	pad.begin();
	prefs.begin("pad", false);
	if (prefs.getBytes("addr", &pad_addr, sizeof(pad_addr)) != sizeof(pad_addr)) memset(&pad_addr, 0, sizeof(pad_addr));
	if (HaveAddr()) {
		pad.setTargetAddress(pad_addr);
		Serial.printf("Pad: looking for %s\n", AddrStr(pad_addr));
		strlcpy(pad_log.match, AddrStr(pad_addr), sizeof(pad_log.match));
	} else {
		StartPairing();
		Serial.println("Pad: none paired, pairing mode");
	}
}

void Put16(uint8_t *b, uint16_t v) {
	b[0] = v;
	b[1] = v >> 8;
}

// State -> packet: seq, flags, battery, LX, LY, RX, RY, LT, RT, buttons (u16 LE), sum
void Pack(uint8_t *b, bool up) {
	XboxControllerNotificationParser &n = pad.xboxNotif;
	bool btns[16] = {
		n.btnA, n.btnB, n.btnX, n.btnY, n.btnLB, n.btnRB, n.btnLS, n.btnRS,
		n.btnDirUp, n.btnDirDown, n.btnDirLeft, n.btnDirRight, n.btnSelect, n.btnStart, n.btnShare, n.btnXbox,
	};
	uint16_t bits = 0;
	int i;

	for (i = 0; i < 16; i++) {
		if (btns[i]) bits |= 1 << i;
	}
	b[0] = 0; // seq, filled in when sent
	b[1] = up;
	b[2] = pad.battery;
	Put16(b + 3, n.joyLHori);
	Put16(b + 5, n.joyLVert);
	Put16(b + 7, n.joyRHori);
	Put16(b + 9, n.joyRVert);
	Put16(b + 11, n.trigLT);
	Put16(b + 13, n.trigRT);
	Put16(b + 15, bits);
	b[17] = 0;
}

// On a change (at most every MIN_GAP_MS) and every BEAT_MS while connected
void PadSend(bool up, bool force) {
	uint8_t b[PAD_LEN], frame[4 + 2 * PAD_LEN];
	uint32_t now = millis();
	int i, len = 0;

	Pack(b, up);
	if (!force && now - sent_ms < MIN_GAP_MS) return;
	if (!force && memcmp(b + 1, last_sent + 1, PAD_LEN - 1) == 0 && now - sent_ms < BEAT_MS) return;
	memcpy(last_sent, b, PAD_LEN);
	sent_ms = now;

	b[0] = seq++;
	for (i = 0; i < PAD_LEN - 1; i++) b[PAD_LEN - 1] += b[i];
	frame[len++] = DLE;
	frame[len++] = 'P';
	for (i = 0; i < PAD_LEN; i++) {
		if (b[i] == DLE) frame[len++] = DLE;
		frame[len++] = b[i];
	}
	frame[len++] = DLE;
	frame[len++] = 'E';
	Serial.write(frame, len); // one write, so a log line can't land inside
}

// A controller switched off doesn't always say goodbye: the link is only dropped after the
// supervision timeout, and until then the H7 keeps getting the last stick state. NimBLE's default
// is 4s. Same interval / latency as the controller picked, just the timeout shorter
void ShortTimeout() {
	NimBLEConnInfo ci = xbox::pConnectedClient->getConnInfo();
	uint16_t itvl = ci.getConnInterval(), lat = ci.getConnLatency();
	// Has to be more than (1 + latency) * interval * 2: 1.25ms units vs 10ms units
	uint16_t min_to = (uint16_t)((1 + lat) * itvl * 2 * 1.25f / 10) + 1;
	uint16_t to = SUPERVISION_MS / 10 > min_to ? SUPERVISION_MS / 10 : min_to;

	// Stored on the client too: when the controller asks for new parameters NimBLE answers with
	// these, otherwise its defaults put the timeout back to seconds. Also used on a reconnect
	xbox::pConnectedClient->setConnectionParams(itvl, itvl, lat, to);
	bool ok = xbox::pConnectedClient->updateConnParams(itvl, itvl, lat, to);
	pad_log.printf("Link interval %.2fms, latency %u, timeout %ums -> %ums%s\n", itvl * 1.25f, lat,
		ci.getConnTimeout() * 10, to * 10, ok ? "" : " (update refused)");
}

// Backstop: back to the short timeout if something changed it
void KeepTimeout() {
	static uint32_t check_ms = 0;
	if (millis() - check_ms < 3000 || xbox::pConnectedClient == nullptr) return;
	check_ms = millis();
	if (xbox::pConnectedClient->getConnInfo().getConnTimeout() * 10 > SUPERVISION_MS * 2) ShortTimeout();
}

// Longest time between two reports from the controller since the last 'pad'
uint32_t max_gap = 0;

void ReportGap() {
	static unsigned long last = 0;
	unsigned long at = pad.getReceiveNotificationAt();
	if (at == last) return;
	if (last && at - last > max_gap) max_gap = at - last;
	last = at;
}

// The library only reconnects to an advertisement whose address equals the target, type included.
// If the paired controller shows up with the same 6 bytes but another type, take that type
void CheckAddrType() {
	static uint32_t check_ms = 0;
	NimBLEScanResults res;

	if (pairing || !HaveAddr() || pad.isConnected() || millis() - check_ms < 1000) return;
	check_ms = millis();
	res = NimBLEDevice::getScan()->getResults();
	for (auto d : res) {
		const ble_addr_t *a = d->getAddress().getBase();
		if (memcmp(a->val, pad_addr.val, sizeof(a->val)) != 0 || a->type == pad_addr.type) continue;
		Serial.printf("Pad: %s advertises as type %u, was looking for type %u\n", AddrStr(pad_addr), a->type, pad_addr.type);
		pad_log.printf("Address type %u seen, target was %u: switched\n", a->type, pad_addr.type);
		pad_addr.type = a->type;
		prefs.putBytes("addr", &pad_addr, sizeof(pad_addr));
		pad.setTargetAddress(pad_addr);
		return;
	}
}

void PadLoop() {
	bool up;

#ifdef PAD_OFF
	return;
#endif
	CheckAddrType();
	pad.onLoop(); // scans / connects, blocks for a second or so while connecting
	up = pad.isConnected();
	if (up) {
		PadSend(true, false);
		ReportGap();
		if (pad_was_connected) KeepTimeout();
	}
	if (up == pad_was_connected) return;
	pad_was_connected = up;

	if (!up) {
		PadSend(false, true); // tell the H7 now instead of waiting out its 150ms
		Serial.println("Pad: disconnected");
		return;
	}
	if (xbox::pConnectedClient == nullptr) return;
	ble_addr_t a = *xbox::pConnectedClient->getPeerAddress().getBase();
	Serial.printf("Pad: connected %s\n", AddrStr(a));
	ShortTimeout();
	max_gap = 0;
	if (pairing) {
		pairing = false;
		pad_addr = a;
		pad_addr.type &= 1; // identity types (2, 3) -> public / random: what it advertises with
		prefs.putBytes("addr", &pad_addr, sizeof(pad_addr));
		pad.setIgnoreTargetAddressToConnectToUnkownSearchingDevice(false);
		pad.setTargetAddress(pad_addr);
		Serial.printf("Pad: paired, saved %s\n", AddrStr(pad_addr));
		strlcpy(pad_log.match, AddrStr(pad_addr), sizeof(pad_log.match));
	}
}

// One line: LX, LY, RX, RY (0-65535, 32768 = centre), LT, RT (0-1023), buttons held
void PrintState() {
	XboxControllerNotificationParser &n = pad.xboxNotif;
	char b[96] = "";
	struct {
		bool on;
		const char *name;
	} btns[] = {
		{ n.btnA, "A" }, { n.btnB, "B" }, { n.btnX, "X" }, { n.btnY, "Y" },
		{ n.btnLB, "LB" }, { n.btnRB, "RB" }, { n.btnLS, "LS" }, { n.btnRS, "RS" },
		{ n.btnDirUp, "up" }, { n.btnDirDown, "down" }, { n.btnDirLeft, "left" }, { n.btnDirRight, "right" },
		{ n.btnSelect, "view" }, { n.btnStart, "menu" }, { n.btnShare, "share" }, { n.btnXbox, "xbox" },
	};

	for (auto &x : btns) {
		if (!x.on) continue;
		if (b[0]) strlcat(b, " ", sizeof(b));
		strlcat(b, x.name, sizeof(b));
	}
	Serial.printf("%d, %u, %u, %u, %u, %u, %u, %s\n", pad.isConnected(), n.joyLHori, n.joyLVert, n.joyRHori, n.joyRVert,
		n.trigLT, n.trigRT, b[0] ? b : "-");
}

bool PadCmd(const char *c) {
#ifdef PAD_OFF
	if (strncmp(c, "pad", 3) != 0) return false;
	Serial.println("Pad: BLE off in this build (PAD_OFF)");
	return true;
#endif
	if (strcmp(c, "pad log") == 0) {
		pad_log.Show();
	} else if (strcmp(c, "pad") == 0) {
		if (pad.isConnected()) {
			NimBLEConnInfo ci = xbox::pConnectedClient->getConnInfo();
			Serial.printf("Pad: connected %s, battery %u%%\n", AddrStr(pad_addr), pad.battery);
			Serial.printf("Link: interval %.2fms, latency %u, timeout %ums. Last report %lums ago, longest gap %lums\n",
				ci.getConnInterval() * 1.25f, ci.getConnLatency(), ci.getConnTimeout() * 10,
				millis() - pad.getReceiveNotificationAt(), (unsigned long)max_gap);
			max_gap = 0;
		} else if (pairing) {
			Serial.println("Pad: pairing mode, hold the controller's pair button until the logo flashes fast");
		} else {
			Serial.printf("Pad: looking for %s, press its Xbox button\n", AddrStr(pad_addr));
		}
		if (HaveAddr()) Serial.printf("Paired: %s, type %u\n", AddrStr(pad_addr), pad_addr.type);
		Serial.printf("Failed connects: %u\n", pad.getCountFailedConnection());
		Serial.print("connected, LX, LY, RX, RY, LT, RT, buttons\n");
		PrintState();
	} else if (strcmp(c, "pad pair") == 0) {
		// Forget the old one, take the next controller in pairing mode
		if (xbox::pConnectedClient != nullptr) xbox::pConnectedClient->disconnect();
		NimBLEDevice::deleteAllBonds();
		memset(&pad_addr, 0, sizeof(pad_addr));
		prefs.remove("addr");
		StartPairing();
		Serial.println("Pairing mode: hold the controller's pair button until the logo flashes fast");
	} else if (strcmp(c, "pad state") == 0) {
		PrintState();
	} else {
		return false;
	}
	return true;
}
