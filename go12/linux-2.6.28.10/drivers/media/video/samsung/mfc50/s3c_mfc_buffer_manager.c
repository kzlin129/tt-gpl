/*
 * drivers/media/video/samsung/mfc40/s3c_mfc_buffer_manager.c
 *
 * C file for Samsung MFC (Multi Function Codec - FIMV) driver
 *
 * Jaeryul Oh, Copyright (c) 2009 Samsung Electronics
 * http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/slab.h>
#include <plat/media.h>

#include "s3c_mfc_buffer_manager.h"
#include "s3c_mfc_errorno.h"
#include "s3c_mfc_logmsg.h"

s3c_mfc_alloc_mem_t *s3c_mfc_alloc_mem_head[MFC_MAX_PORT_NUM];
s3c_mfc_alloc_mem_t *s3c_mfc_alloc_mem_tail[MFC_MAX_PORT_NUM];
s3c_mfc_free_mem_t *s3c_mfc_free_mem_head[MFC_MAX_PORT_NUM];
s3c_mfc_free_mem_t *s3c_mfc_free_mem_tail[MFC_MAX_PORT_NUM];

extern dma_addr_t s3c_mfc_phys_data_buf;	// port 1
extern unsigned char *s3c_mfc_virt_data_buf;	// port 1

extern dma_addr_t s3c_mfc_phys_dpb_luma_buf;		// port 0
extern unsigned char *s3c_mfc_virt_dpb_luma_buf;	// port 0


/* insert node ahead of s3c_mfc_alloc_mem_head */
static void s3c_mfc_insert_node_to_alloc_list(s3c_mfc_alloc_mem_t *node, int inst_no, int port_no)
{
	mfc_debug("[%d]instance, [%d]port (p_addr : 0x%08x size:%d)\n",
			inst_no, port_no, node->p_addr, node->size);
	node->next = s3c_mfc_alloc_mem_head[port_no];
	node->prev = s3c_mfc_alloc_mem_head[port_no]->prev;
	s3c_mfc_alloc_mem_head[port_no]->prev->next = node;
	s3c_mfc_alloc_mem_head[port_no]->prev = node;
	s3c_mfc_alloc_mem_head[port_no]= node;
}

void s3c_mfc_print_list(void)
{
	s3c_mfc_alloc_mem_t *node1;
	s3c_mfc_free_mem_t *node2;
	int count = 0;
	unsigned int p_addr;
	int port_no;

	for (port_no=0; port_no < MFC_MAX_PORT_NUM; port_no++) {

	for (node1 = s3c_mfc_alloc_mem_head[port_no]; node1 != s3c_mfc_alloc_mem_tail[port_no]; node1 = node1->next) {
		p_addr = (unsigned int)node1->p_addr;

 		printk("s3c_mfc_print_list [AllocList][%d] inst_no : %d port_no : %d  p_addr : 0x%08x v_addr:0x%08x size:%d\n", 
				count++, node1->inst_no, node1->port_no, p_addr, (unsigned int)node1->v_addr, node1->size);

	}

	count = 0;
	for (node2 = s3c_mfc_free_mem_head[port_no]; node2 != s3c_mfc_free_mem_tail[port_no]; node2 = node2->next) {
		printk("s3c_mfc_print_list [FreeList][%d] startAddr : 0x%08x size:%d\n", 
				count++, node2->start_addr , node2->size);
	}

	}
}

/*
int list_count()
{
	int count = 0;
	s3c_mfc_free_mem_t *node;

	node = s3c_mfc_free_mem_head;
	
	while (node != s3c_mfc_free_mem_tail) {	
		node = node->next;
		count++;
	}

	return count;
}
*/

static void s3c_mfc_insert_first_node_to_free_list(s3c_mfc_free_mem_t *node,  int inst_no, int port_no)
{
	mfc_debug("[%d]instance(startAddr : 0x%08x size:%d)\n",
			inst_no, node->start_addr, node->size);	
	
	node->next = s3c_mfc_free_mem_head[port_no];
	node->prev = s3c_mfc_free_mem_head[port_no]->prev;
	s3c_mfc_free_mem_head[port_no]->prev->next = node;
	s3c_mfc_free_mem_head[port_no]->prev = node;
	s3c_mfc_free_mem_head[port_no]= node;
	
}

/* insert node ahead of s3c_mfc_free_mem_head */
static void s3c_mfc_insert_node_to_free_list(s3c_mfc_free_mem_t *node,  int inst_no, int port_no)
{
	s3c_mfc_free_mem_t *itr_node;
	
	mfc_debug("[%d]instance, [%d]port (startAddr : 0x%08x size:%d)\n",
			inst_no, port_no, node->start_addr, node->size);	

	itr_node = s3c_mfc_free_mem_head[port_no];
	
	while (itr_node != s3c_mfc_free_mem_tail[port_no]) {
		
		if (itr_node->start_addr >= node->start_addr) {
			/* head */
			if (itr_node == s3c_mfc_free_mem_head[port_no]) {
				node->next = s3c_mfc_free_mem_head[port_no];
				node->prev = s3c_mfc_free_mem_head[port_no]->prev;
				s3c_mfc_free_mem_head[port_no]->prev->next = node;
				s3c_mfc_free_mem_head[port_no]->prev = node;
				s3c_mfc_free_mem_head[port_no]= node;
				break;
			} else { /* mid */
				node->next = itr_node;
				node->prev = itr_node->prev;
				itr_node->prev->next = node;
				itr_node->prev = node;
				break;
			}
		
		}

		itr_node = itr_node->next;
	}

	/* tail */
	if (itr_node == s3c_mfc_free_mem_tail[port_no]) {
		node->next = s3c_mfc_free_mem_tail[port_no];
		node->prev = s3c_mfc_free_mem_tail[port_no]->prev;
		s3c_mfc_free_mem_tail[port_no]->prev->next = node;
		s3c_mfc_free_mem_tail[port_no]->prev = node;
	}
	
}

static void s3c_mfc_del_node_from_alloc_list(s3c_mfc_alloc_mem_t *node, int inst_no, int port_no)
{
	mfc_debug("[%d]instance, [%d]port (p_addr : 0x%08x size:%d)\n",
			inst_no, port_no, node->p_addr, node->size);

	if(node == s3c_mfc_alloc_mem_tail[port_no]){
		mfc_info("InValid node\n");
		return;
	}

	if(node == s3c_mfc_alloc_mem_head[port_no])
		s3c_mfc_alloc_mem_head[port_no]= node->next;

	node->prev->next = node->next;
	node->next->prev = node->prev;

	kfree(node);
}

static void s3c_mfc_del_node_from_free_list( s3c_mfc_free_mem_t *node, int inst_no, int port_no)
{
	mfc_debug("[%d]s3c_mfc_del_node_from_free_list(startAddr : 0x%08x size:%d)\n", 
						inst_no, node->start_addr, node->size);
	if(node == s3c_mfc_free_mem_tail[port_no]){
		mfc_err("InValid node\n");
		return;
	}

	if(node == s3c_mfc_free_mem_head[port_no])
		s3c_mfc_free_mem_head[port_no]= node->next;

	node->prev->next = node->next;
	node->next->prev = node->prev;

	kfree(node);
}

/* Remove Fragmentation in FreeMemList */
void s3c_mfc_merge_frag(int inst_no)
{
	s3c_mfc_free_mem_t *node1, *node2;
	int port_no;

	for (port_no=0; port_no < MFC_MAX_PORT_NUM; port_no++) {

	node1 = s3c_mfc_free_mem_head[port_no];

	while (node1 != s3c_mfc_free_mem_tail[port_no]) {
		node2 = s3c_mfc_free_mem_head[port_no];
		while (node2 != s3c_mfc_free_mem_tail[port_no]) {
			if (node1->start_addr + node1->size == node2->start_addr) {
				node1->size += node2->size;
				mfc_debug("find merge area !! ( node1->start_addr + node1->size == node2->start_addr)\n");
				s3c_mfc_del_node_from_free_list(node2, inst_no, port_no);
				break;
			} else if(node1->start_addr == node2->start_addr + node2->size) {
				mfc_debug("find merge area !! ( node1->start_addr == node2->start_addr + node2->size)\n");
				node1->start_addr = node2->start_addr;
				node1->size += node2->size;
				s3c_mfc_del_node_from_free_list(node2, inst_no, port_no);
				break;
			}
			node2 = node2->next;
		}
		node1 = node1->next;
	}

	}
}

static unsigned int s3c_mfc_get_mem_area(int allocSize, int inst_no, int port_no)
{
	s3c_mfc_free_mem_t	*node, *match_node = NULL;
	unsigned int	allocAddr = 0;


	mfc_debug("request Size : %ld\n", allocSize);

	if (s3c_mfc_free_mem_head[port_no]== s3c_mfc_free_mem_tail[port_no]) {
		mfc_err("all memory is gone\n");
		return(allocAddr);
	}

	/* find best chunk of memory */
	for (node = s3c_mfc_free_mem_head[port_no]; node != s3c_mfc_free_mem_tail[port_no]; 
		node = node->next) {
		if (match_node != NULL) {
			if ((node->size >= allocSize) && (node->size < match_node->size))
				match_node = node;			
		} else {
			if (node->size >= allocSize)
				match_node = node;			
		}
	}

	if (match_node != NULL) {
		mfc_debug("match : startAddr(0x%08x) size(%ld)\n", 
			match_node->start_addr, match_node->size);
	}

	/* rearange FreeMemArea */
	if (match_node != NULL) {
		allocAddr = match_node->start_addr;
		match_node->start_addr += allocSize;
		match_node->size -= allocSize;

		if(match_node->size == 0)          /* delete match_node. */
			s3c_mfc_del_node_from_free_list(match_node, inst_no, port_no);

		return(allocAddr);
	} else {
		printk("there is no suitable chunk\n");
		return 0;
	}

	return(allocAddr);
}


int s3c_mfc_init_buffer_manager(void)
{
	s3c_mfc_free_mem_t	*free_node;
	s3c_mfc_alloc_mem_t	*alloc_node;
	//s3c_mfc_port_type	port_type;
	int	port_no;
		

	for (port_no=0; port_no < MFC_MAX_PORT_NUM; port_no++) {

	/* init alloc list, if(s3c_mfc_alloc_mem_head == s3c_mfc_alloc_mem_tail) then, the list is NULL */
	alloc_node = (s3c_mfc_alloc_mem_t *)kmalloc(sizeof(s3c_mfc_alloc_mem_t), GFP_KERNEL);
	memset(alloc_node, 0x00, sizeof(s3c_mfc_alloc_mem_t));
	alloc_node->next = alloc_node;
	alloc_node->prev = alloc_node;
	s3c_mfc_alloc_mem_head[port_no] = alloc_node;
	s3c_mfc_alloc_mem_tail[port_no] = s3c_mfc_alloc_mem_head[port_no];

	/* init free list, if(s3c_mfc_free_mem_head == s3c_mfc_free_mem_tail) then, the list is NULL */
	free_node = (s3c_mfc_free_mem_t *)kmalloc(sizeof(s3c_mfc_free_mem_t), GFP_KERNEL);
	memset(free_node, 0x00, sizeof(s3c_mfc_free_mem_t));
	free_node->next = free_node;
	free_node->prev = free_node;
	s3c_mfc_free_mem_head[port_no]= free_node;
	s3c_mfc_free_mem_tail[port_no]= s3c_mfc_free_mem_head[port_no];

	/* init free head node */
	free_node = (s3c_mfc_free_mem_t *)kmalloc(sizeof(s3c_mfc_free_mem_t), GFP_KERNEL);
	memset(free_node, 0x00, sizeof(s3c_mfc_free_mem_t));
	free_node->start_addr = (port_no) ? s3c_mfc_phys_data_buf : s3c_mfc_phys_dpb_luma_buf;
	free_node->size = (port_no) ? s3c_get_media_memsize(S3C_MDEV_MFC) : s3c_get_media_memsize(S3C_MDEV_MFC);
	// peter, it should be changed
	s3c_mfc_insert_first_node_to_free_list(free_node, -1, port_no);

	}

	return 0;
}


/* Releae memory */
MFC_ERROR_CODE s3c_mfc_release_alloc_mem(s3c_mfc_inst_ctx  *mfc_ctx,  s3c_mfc_args *args)
{		
	int ret;

	s3c_mfc_free_mem_t *free_node;
	s3c_mfc_alloc_mem_t *node;
	int port_no = 0;
	int matched_u_addr = 0;

	for (port_no=0; port_no < MFC_MAX_PORT_NUM; port_no++) {			
		for(node = s3c_mfc_alloc_mem_head[port_no]; node != s3c_mfc_alloc_mem_tail[port_no]; 
			node = node->next) {
			if(node->u_addr == (unsigned char *)args->mem_free.u_addr) {
				matched_u_addr = 1;
				break;
			}	
		}
		if (matched_u_addr)
			break;			
	}	

	if (node == s3c_mfc_alloc_mem_tail[port_no]) {
		mfc_err("invalid virtual address(0x%x)\r\n", args->mem_free.u_addr);
		ret = MFCINST_MEMORY_INVAILD_ADDR;
		goto out_releaseallocmem;
	}

	free_node = (s3c_mfc_free_mem_t	*)kmalloc(sizeof(s3c_mfc_free_mem_t), GFP_KERNEL);

	free_node->start_addr = node->p_addr;	

	free_node->size = node->size;
	s3c_mfc_insert_node_to_free_list(free_node, mfc_ctx->InstNo, port_no);

	/* Delete from AllocMem list */
	s3c_mfc_del_node_from_alloc_list(node, mfc_ctx->InstNo, port_no);	

	ret = MFCINST_RET_OK;

out_releaseallocmem:
	return ret;
}

MFC_ERROR_CODE s3c_mfc_get_phys_addr(s3c_mfc_inst_ctx *mfc_ctx, s3c_mfc_args *args)
{
	int ret;
	s3c_mfc_alloc_mem_t *node;
	s3c_mfc_get_phys_addr_arg_t *codec_get_phy_addr_arg = (s3c_mfc_get_phys_addr_arg_t *)args;
	int port_no = mfc_ctx->port_no;

	for(node = s3c_mfc_alloc_mem_head[port_no]; node != s3c_mfc_alloc_mem_tail[port_no]; node = node->next) {
		if(node->u_addr == (unsigned char *)codec_get_phy_addr_arg->u_addr)
			break;
	}

	if(node == s3c_mfc_alloc_mem_tail[port_no]){
		mfc_err("invalid virtual address(0x%x)\r\n", codec_get_phy_addr_arg->u_addr);
		ret = MFCINST_MEMORY_INVAILD_ADDR;
		goto out_getphysaddr;
	}

	codec_get_phy_addr_arg->p_addr = node->p_addr;
	

	ret = MFCINST_RET_OK;

out_getphysaddr:
	return ret;

}

MFC_ERROR_CODE s3c_mfc_get_virt_addr(s3c_mfc_inst_ctx  *mfc_ctx,  s3c_mfc_args *args)
{
	int ret;
	int inst_no = mfc_ctx->InstNo;
	int port_no = mfc_ctx->port_no;
	unsigned int p_startAddr;
	s3c_mfc_mem_alloc_arg_t *in_param;	
	s3c_mfc_alloc_mem_t *p_allocMem;
	

	in_param = (s3c_mfc_mem_alloc_arg_t *)args;

	/* if user request area, allocate from reserved area */
	p_startAddr = s3c_mfc_get_mem_area((int)in_param->buff_size, inst_no, port_no);
	mfc_debug("p_startAddr = 0x%X\n\r", p_startAddr);

	if (!p_startAddr) {
		mfc_debug("There is no more memory\n\r");
		in_param->out_addr = -1;
		ret = MFCINST_MEMORY_ALLOC_FAIL;
		goto out_getcodecviraddr;
	}

	p_allocMem = (s3c_mfc_alloc_mem_t *)kmalloc(sizeof(s3c_mfc_alloc_mem_t), GFP_KERNEL);
	memset(p_allocMem, 0x00, sizeof(s3c_mfc_alloc_mem_t));

	p_allocMem->p_addr = p_startAddr;
	if (port_no) {	// port 1
		p_allocMem->v_addr = s3c_mfc_virt_data_buf + (p_allocMem->p_addr - s3c_mfc_phys_data_buf);
		p_allocMem->u_addr = (unsigned char *)(in_param->mapped_addr + 
				(p_allocMem->p_addr - s3c_mfc_phys_data_buf));
	} else {
		p_allocMem->v_addr = s3c_mfc_virt_dpb_luma_buf + (p_allocMem->p_addr - s3c_mfc_phys_dpb_luma_buf);
		p_allocMem->u_addr = (unsigned char *)(in_param->mapped_addr + 
				(p_allocMem->p_addr - s3c_mfc_phys_dpb_luma_buf));
	}	

	if (p_allocMem->v_addr == NULL) {
		mfc_debug("Mapping Failed [PA:0x%08x]\n\r", p_allocMem->p_addr);
		ret = MFCINST_MEMORY_MAPPING_FAIL;
		goto out_getcodecviraddr;
	}	

	in_param->out_addr = (unsigned int)p_allocMem->u_addr;
	mfc_debug("u_addr : 0x%x v_addr : 0x%x p_addr : 0x%x\n",
		p_allocMem->u_addr, p_allocMem->v_addr, p_allocMem->p_addr);

	p_allocMem->size = (int)in_param->buff_size;
	p_allocMem->inst_no = inst_no;
	p_allocMem->port_no = port_no;
	
	s3c_mfc_insert_node_to_alloc_list(p_allocMem, inst_no, port_no);
	ret = MFCINST_RET_OK;

out_getcodecviraddr:	
	return ret;
}

