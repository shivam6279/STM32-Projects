#ifndef _LWIPOPTS_H_
#define _LWIPOPTS_H_

// lwIP config: no RTOS, everything runs from the main loop (NetTask), nothing from ISRs.
// IPv4 + ARP + ICMP (ping) + UDP + TCP. No DHCP (fixed address on the Pi link).

#define NO_SYS					1
#define SYS_LIGHTWEIGHT_PROT	0		// single context
#define LWIP_SOCKET				0
#define LWIP_NETCONN			0
#define LWIP_SINGLE_NETIF		1

#define LWIP_IPV4				1
#define LWIP_IPV6				0
#define LWIP_ARP				1
#define LWIP_ICMP				1
#define LWIP_UDP				1
#define LWIP_TCP				1
#define TCP_MSS					1460
#define TCP_WND					(4 * TCP_MSS)
#define TCP_SND_BUF				(4 * TCP_MSS)
#define TCP_SND_QUEUELEN		(2 * TCP_SND_BUF / TCP_MSS)
#define MEMP_NUM_TCP_PCB		4
#define MEMP_NUM_TCP_PCB_LISTEN	2
#define MEMP_NUM_TCP_SEG		(TCP_SND_QUEUELEN + 4)
#define LWIP_DHCP				0
#define LWIP_AUTOIP				0
#define LWIP_IGMP				0
#define LWIP_DNS				0
#define IP_REASSEMBLY			0
#define IP_FRAG					0

#define LWIP_NETIF_LINK_CALLBACK	1

#define MEM_ALIGNMENT			4
#define MEM_SIZE				(24 * 1024)	// TCP send buffers come from here
#define MEMP_NUM_PBUF			16
#define MEMP_NUM_UDP_PCB		4
// lwIP only counts its own timers, add ours (link poll + spare)
#define MEMP_NUM_SYS_TIMEOUT	(LWIP_NUM_SYS_TIMEOUT_INTERNAL + 2)
#define PBUF_POOL_SIZE			16
#define PBUF_POOL_BUFSIZE		1536

#define ETH_PAD_SIZE			0
#define LWIP_STATS				0
#define LWIP_DEBUG				0

#endif
