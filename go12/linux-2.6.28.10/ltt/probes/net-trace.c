/*
 * ltt/probes/net-trace.c
 *
 * Net tracepoint probes.
 */

#include <linux/module.h>
#include <trace/net.h>
#include <trace/ipv4.h>
#include <trace/ipv6.h>
#include <trace/socket.h>

void probe_net_dev_xmit(struct sk_buff *skb)
{
	trace_mark_tp(net_dev_xmit, net_dev_xmit,
		probe_net_dev_xmit,
		"skb %p protocol #2u%hu", skb, skb->protocol);
}

void probe_net_dev_receive(struct sk_buff *skb)
{
	trace_mark_tp(net_dev_receive, net_dev_receive,
		probe_net_dev_receive, "skb %p protocol #2u%hu",
		skb, skb->protocol);
}

void probe_ipv4_addr_add(struct in_ifaddr *ifa)
{
	trace_mark_tp(net_insert_ifa_ipv4, ipv4_addr_add,
		probe_ipv4_addr_add, "label %s address #4u%u",
		ifa->ifa_label, (unsigned int)ifa->ifa_address);
}

void probe_ipv4_addr_del(struct in_ifaddr *ifa)
{
	trace_mark_tp(net_del_ifa_ipv4, ipv4_addr_del,
		probe_ipv4_addr_del, "label %s address #4u%u",
		ifa->ifa_label, (unsigned int)ifa->ifa_address);
}

void probe_ipv6_addr_add(struct inet6_ifaddr *ifa)
{
	__u8 *addr = ifa->addr.s6_addr;

	trace_mark_tp(net_insert_ifa_ipv6, ipv6_addr_add, probe_ipv6_addr_add,
		"label %s "
		"a15 #1x%c a14 #1x%c a13 #1x%c a12 #1x%c "
		"a11 #1x%c a10 #1x%c a9 #1x%c a8 #1x%c "
		"a7 #1x%c a6 #1x%c a5 #1x%c a4 #1x%c "
		"a3 #1x%c a2 #1x%c a1 #1x%c a0 #1x%c",
		ifa->idev->dev->name,
		addr[15], addr[14], addr[13], addr[12],
		addr[11], addr[10], addr[9], addr[8],
		addr[7], addr[6], addr[5], addr[4],
		addr[3], addr[2], addr[1], addr[0]);
}

void probe_ipv6_addr_del(struct inet6_ifaddr *ifa)
{
	__u8 *addr = ifa->addr.s6_addr;

	trace_mark_tp(net_insert_ifa_ipv6, ipv6_addr_del, probe_ipv6_addr_del,
		"label %s "
		"a15 #1x%c a14 #1x%c a13 #1x%c a12 #1x%c "
		"a11 #1x%c a10 #1x%c a9 #1x%c a8 #1x%c "
		"a7 #1x%c a6 #1x%c a5 #1x%c a4 #1x%c "
		"a3 #1x%c a2 #1x%c a1 #1x%c a0 #1x%c",
		ifa->idev->dev->name,
		addr[15], addr[14], addr[13], addr[12],
		addr[11], addr[10], addr[9], addr[8],
		addr[7], addr[6], addr[5], addr[4],
		addr[3], addr[2], addr[1], addr[0]);
}

void probe_socket_sendmsg(struct socket *sock, struct msghdr *msg,
		size_t size, int ret)
{
	trace_mark_tp(net_socket_sendmsg, socket_sendmsg, probe_socket_sendmsg,
		"sock %p family %d type %d protocol %d size %zu",
		sock, sock->sk->sk_family, sock->sk->sk_type,
		sock->sk->sk_protocol, size);
}

void probe_socket_recvmsg(struct socket *sock, struct msghdr *msg,
		size_t size, int flags, int ret)
{
	trace_mark_tp(net_socket_recvmsg, socket_recvmsg, probe_socket_recvmsg,
		"sock %p family %d type %d protocol %d size %zu",
		sock, sock->sk->sk_family, sock->sk->sk_type,
		sock->sk->sk_protocol, size);
}

void probe_socket_create(struct socket *sock, int fd)
{
	trace_mark_tp(net_socket_create, socket_create, probe_socket_create,
		"sock %p family %d type %d protocol %d fd %d",
		sock, sock->sk->sk_family, sock->sk->sk_type,
		sock->sk->sk_protocol, fd);
}

void probe_socket_call(int call, unsigned long a0)
{
	trace_mark_tp(net_socket_call, socket_call, probe_socket_call,
		"call %d a0 %lu", call, a0);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Net Tracepoint Probes");
