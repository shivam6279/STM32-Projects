#include "net.h"
#include "clock.h"
#include "pwm.h"
#include "stepgen.h"
#include "stm32h7xx.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"

#include <stdio.h>
#include <string.h>

#define RX_PER_TASK 8	// frames per NetTask call, so a flood can't starve the main loop
#define CON_WRITE_TIMEOUT_MS 50	// same as USB: a stuck client drops output, it doesn't stall us

static struct netif nif;
static struct udp_pcb *disc;
static uint8_t mac[6];
static bool up = false;
static EthLink link;
static unsigned long rand_state;

// TCP console: one client, a new one replaces it (the old one may be a dead connection)
static struct tcp_pcb *con_listen;
static struct tcp_pcb *con;
static struct pbuf *con_rx;		// received, not read yet. The window only reopens as it's read
static bool con_replaced;		// a new client took over, report one disconnect so it gets greeted

static void ConListen();

// ---- lwIP port hooks ----

u32_t sys_now(void) {
	return Millis();
}

unsigned long NetRand() {
	rand_state ^= rand_state << 13;
	rand_state ^= rand_state >> 17;
	rand_state ^= rand_state << 5;
	return rand_state;
}

// lwIP found a bug in itself or us. Make the machine safe and stop.
void NetAssert(const char *msg, const char *file, int line) {
	StepAbort();
	PwmAllOff();
	printf("lwIP assert: %s (%s:%d)\n", msg, file, line);
	while (1);
}

// ---- netif ----

static err_t LinkOutput(struct netif *n, struct pbuf *p) {
	uint8_t *buf;
	(void)n;

	if (p->tot_len > ETH_MTU_FRAME) return ERR_BUF;
	buf = EthTxBuffer();
	if (buf == NULL) return ERR_MEM;
	pbuf_copy_partial(p, buf, p->tot_len, 0);
	EthTxSend(p->tot_len);
	return ERR_OK;
}

static err_t IfInit(struct netif *n) {
	n->name[0] = 'e';
	n->name[1] = '0';
	n->output = etharp_output;
	n->linkoutput = LinkOutput;
	n->mtu = 1500;
	n->hwaddr_len = 6;
	memcpy(n->hwaddr, mac, 6);
	n->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
	return ERR_OK;
}

// Runs as an lwIP timer: PHY over MDIO -> MAC speed/duplex + lwIP link up/down
static void LinkPoll(void *arg) {
	(void)arg;
	link = EthLinkUpdate();
	if (link.up && !netif_is_link_up(&nif)) netif_set_link_up(&nif);
	else if (!link.up && netif_is_link_up(&nif)) netif_set_link_down(&nif);
	sys_timeout(NET_LINK_POLL_MS, LinkPoll, NULL);
}

// Anything on the discovery port gets a broadcast reply, so it works even if the Pi is on
// the wrong subnet and couldn't be answered directly
static void DiscoveryRx(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	char msg[192], ip[16], mask[16];
	const uint32_t *uid = (const uint32_t *)UID_BASE;
	struct pbuf *r;
	int len;
	(void)arg;
	(void)addr;

	// Don't answer replies (ours looped back, or another board's), or two boards ping-pong forever
	if (p->len >= 4 && memcmp(p->payload, "PnP ", 4) == 0) {
		pbuf_free(p);
		return;
	}
	pbuf_free(p);
	ip4addr_ntoa_r(netif_ip4_addr(&nif), ip, sizeof(ip));
	ip4addr_ntoa_r(netif_ip4_netmask(&nif), mask, sizeof(mask));
	len = snprintf(msg, sizeof(msg), "PnP ip=%s mask=%s mac=%02X:%02X:%02X:%02X:%02X:%02X uid=%08lX%08lX%08lX fw=%s %s\n",
		ip, mask, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
		(unsigned long)uid[0], (unsigned long)uid[1], (unsigned long)uid[2], __DATE__, __TIME__);

	r = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
	if (r == NULL) return;
	memcpy(r->payload, msg, len);
	udp_sendto(pcb, r, IP_ADDR_BROADCAST, port);
	pbuf_free(r);
}

bool NetInit() {
	const uint32_t *uid = (const uint32_t *)UID_BASE;
	uint32_t h = uid[0] ^ (uid[1] * 31) ^ (uid[2] * 1009);
	ip4_addr_t ip, mask, gw;

	// MAC from the unique ID, locally administered + unicast (02:...)
	mac[0] = 0x02;
	mac[1] = uid[0] >> 8;
	mac[2] = h >> 24;
	mac[3] = h >> 16;
	mac[4] = h >> 8;
	mac[5] = h;
	rand_state = h ^ Millis() ^ 0x9E3779B9;

	up = EthInit(mac);

	lwip_init();
	ip4addr_aton(NET_IP, &ip);
	ip4addr_aton(NET_MASK, &mask);
	ip4_addr_set_zero(&gw);	// no gateway: the Pi's internet is its own business
	netif_add(&nif, &ip, &mask, &gw, NULL, IfInit, ethernet_input);
	netif_set_default(&nif);
	netif_set_up(&nif);
	if (!up) return false;

	LinkPoll(NULL);

	disc = udp_new();
	udp_bind(disc, IP_ADDR_ANY, NET_DISCOVERY_PORT);
	udp_recv(disc, DiscoveryRx, NULL);
	ConListen();
	return true;
}

static void Pump() {
	uint8_t *buf;
	uint32_t len, n;
	struct pbuf *p;

	for (n = 0; n < RX_PER_TASK && (buf = EthRxPeek(&len)) != NULL; n++) {
		p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
		if (p != NULL) {
			pbuf_take(p, buf, len);
			if (nif.input(p, &nif) != ERR_OK) pbuf_free(p);
		}
		EthRxRelease();
	}
	sys_check_timeouts();
}

void NetTask() {
	if (!up) return;
	Pump();
}

// ---- TCP console ----

static void ConDrop() {
	if (con_rx) {
		pbuf_free(con_rx);
		con_rx = NULL;
	}
	con = NULL;
}

static void ConClose(struct tcp_pcb *pcb) {
	tcp_arg(pcb, NULL);
	tcp_recv(pcb, NULL);
	tcp_err(pcb, NULL);
	if (tcp_close(pcb) != ERR_OK) tcp_abort(pcb);
}

// lwIP already freed the pcb (reset, or aborted)
static void ConErr(void *arg, err_t err) {
	(void)err;
	if (arg == con) ConDrop();
}

static err_t ConRecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
	(void)arg;
	(void)err;

	if (pcb != con) {
		if (p) pbuf_free(p);
		return ERR_OK;
	}
	if (p == NULL) {
		// Client closed its side
		ConDrop();
		ConClose(pcb);
		return ERR_OK;
	}
	if (con_rx) pbuf_cat(con_rx, p);
	else con_rx = p;
	return ERR_OK;
}

static err_t ConAccept(void *arg, struct tcp_pcb *pcb, err_t err) {
	(void)arg;

	if (err != ERR_OK || pcb == NULL) return ERR_VAL;
	if (con) {
		struct tcp_pcb *old = con;
		ConDrop();
		ConClose(old);
		con_replaced = true;
	}
	con = pcb;
	tcp_arg(pcb, pcb);
	tcp_recv(pcb, ConRecv);
	tcp_err(pcb, ConErr);
	tcp_nagle_disable(pcb); // "ok" goes out now, not after the Pi's delayed ACK
	return ERR_OK;
}

static void ConListen() {
	struct tcp_pcb *pcb = tcp_new();

	if (pcb == NULL) return;
	if (tcp_bind(pcb, IP_ADDR_ANY, NET_CON_PORT) != ERR_OK) {
		tcp_close(pcb);
		return;
	}
	con_listen = tcp_listen(pcb);
	if (con_listen) tcp_accept(con_listen, ConAccept);
}

int NetConReadChar() {
	uint8_t c;

	if (con_rx == NULL || con == NULL) return -1;
	c = pbuf_get_at(con_rx, 0);
	con_rx = pbuf_free_header(con_rx, 1);
	tcp_recved(con, 1);
	return c;
}

// Copies into lwIP's send buffer, waits (running the RX side for ACKs) while it's full
uint32_t NetConWrite(const char *buf, uint32_t len) {
	uint32_t done = 0, n, t = Millis();
	struct tcp_pcb *pcb = con;

	if (pcb == NULL) return len; // nobody listening, drop it
	while (done < len && con == pcb) {
		n = tcp_sndbuf(pcb);
		if (n > len - done) n = len - done;
		if (n > 0 && tcp_write(pcb, buf + done, n, TCP_WRITE_FLAG_COPY) == ERR_OK) {
			done += n;
			continue;
		}
		// Full: send what's queued and wait for ACKs
		tcp_output(pcb);
		Pump();
		if (Millis() - t > CON_WRITE_TIMEOUT_MS) break;
	}
	if (con == pcb) tcp_output(pcb);
	return done;
}

bool NetConConnected() {
	if (con_replaced) {
		con_replaced = false;
		return false;
	}
	return con != NULL;
}

uint32_t NetConPeer() {
	return con ? ip4_addr_get_u32(ip_2_ip4(&con->remote_ip)) : 0;
}

bool NetUp() {
	return up;
}

EthLink NetLink() {
	return link;
}

void NetGetMac(uint8_t *m) {
	memcpy(m, mac, 6);
}

uint32_t NetIp() {
	return ip4_addr_get_u32(netif_ip4_addr(&nif));
}

uint32_t NetMask() {
	return ip4_addr_get_u32(netif_ip4_netmask(&nif));
}

bool NetSetIp(const char *ip_str, const char *mask_str) {
	ip4_addr_t ip, mask, gw;

	if (!ip4addr_aton(ip_str, &ip)) return false;
	if (mask_str == NULL) mask = *netif_ip4_netmask(&nif);
	else if (!ip4addr_aton(mask_str, &mask)) return false;
	ip4_addr_set_zero(&gw);
	netif_set_addr(&nif, &ip, &mask, &gw);
	return true;
}
