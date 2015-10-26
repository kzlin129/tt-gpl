/*
 * linux/include/asm-generic/topology.h
 *
 * Written by: Matthew Dobson, IBM Corporation
 *
 * Copyright (C) 2002, IBM Corp.
 *
 * All rights reserved.          
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Send feedback to <colpatch@us.ibm.com>
 */
#ifndef _ASM_GENERIC_TOPOLOGY_H
#define _ASM_GENERIC_TOPOLOGY_H

/*
 * Other architectures wishing to use this simple topology API should fill
 * in the below functions as appropriate in their own <asm/topology.h> file,
 * and _don't_ include asm-generic/topology.h.
 */

struct pci_bus;

static inline int cpu_to_node(int cpu)
{
	return 0;
}

static inline int parent_node(int node)
{
	return 0;
}

static inline cpumask_t node_to_cpumask(int node)
{
	return cpu_online_map;
}

static inline int node_to_first_cpu(int node)
{
	return 0;
}

static inline int pcibus_to_node(struct pci_bus *bus)
{
	return -1;
}

static inline cpumask_t pcibus_to_cpumask(struct pci_bus *bus)
{
	return pcibus_to_node(bus) == -1 ?
		CPU_MASK_ALL :
		node_to_cpumask(pcibus_to_node(bus));
}

/* returns pointer to cpumask for specified node */
#define	node_to_cpumask_ptr(v, node) 					\
		cpumask_t _##v = node_to_cpumask(node);			\
		const cpumask_t *v = &_##v

#define node_to_cpumask_ptr_next(v, node)				\
			  _##v = node_to_cpumask(node)

#endif /* _ASM_GENERIC_TOPOLOGY_H */
