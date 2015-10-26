/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


/****************************************************************************/
/**
*   @file   user-eth.h
*
*   @brief  Reflects the ethlib API into user space through a driver.
*/
/****************************************************************************/

#if !defined( USER_ETH_H )
#define USER_ETH_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/ioctl.h>
#include <linux/compiler.h>

#include <linux/broadcom/bcmring/eth_defs.h>

/* ---- Constants and Types ---------------------------------------------- */

/* The network interface can define its own ioctl commands to modify network configurations.  
   The ioctl implementation for sockets recognizes 16 commands as private to the interface: 
   SIOCDEVPRIVATE through SIOCDEVPRIVATE+15.  To call a custom ioctl function, user will supply 
   a cmd parameter and an argument structure.

   To expand beyond 16 custom ioctls, all Broadcom-specific network ioctls will be accessed through
   a single cmd SIOC_BCMRING_PRIVATE, which is defined in the range of SIOCDEVPRIVATE through 
   SIOCDEVPRIVATE+15.  Another cmd type will be embedded into the argument structure (cmd field in
   ETH_IOCTL_ARGS to specify the custom function to call.  This will allow unlimited number of 
   custom functions.
*/

/* BCMRING Device private ioctl calls */
#define SIOC_BCMRING_PRIVATE    SIOCDEVPRIVATE
#define SIOCMIIMASKSET          SIOCDEVPRIVATE + 1
#define SIOCMIIMASKCLR          SIOCDEVPRIVATE + 2


typedef struct
{
   int          cmd;    /* Command to run */
   void __user  *arg;   /* Pointer to the argument structure */
} ETH_IOCTL_ARGS;

#define ETH_MAGIC  'g'

#define ETH_CMD_SWITCH_CONTROL_GET           0x80
#define ETH_CMD_SWITCH_CONTROL_SET           0x81
#define ETH_CMD_STAT_GET                     0x82
#define ETH_CMD_STAT_GET32                   0x83
#define ETH_CMD_STAT_CLEAR                   0x84
#define ETH_CMD_PORT_PHY_SET                 0x85
#define ETH_CMD_PORT_PHY_GET                 0x86
#define ETH_CMD_VLAN_INIT                    0x87
#define ETH_CMD_VLAN_CREATE                  0x88
#define ETH_CMD_VLAN_DESTROY                 0x89
#define ETH_CMD_VLAN_DESTROY_ALL             0x8A
#define ETH_CMD_VLAN_PORT_GET                0x8B
#define ETH_CMD_VLAN_PORT_ADD                0x8C
#define ETH_CMD_VLAN_PORT_REMOVE             0x8D
#define ETH_CMD_VLAN_TRANSLATE_ADD           0x8E
#define ETH_CMD_VLAN_TRANSLATE_DELETE        0x8F
#define ETH_CMD_PORT_UNTAGGED_VLAN_SET       0x90
#define ETH_CMD_PORT_UNTAGGED_VLAN_GET       0x91
#define ETH_CMD_PORT_UNTAGGED_PRIORITY_SET   0x92
#define ETH_CMD_PORT_UNTAGGED_PRIORITY_GET   0x93
#define ETH_CMD_PORT_PHY_ID_SET              0x94
#define ETH_CMD_PORT_PHY_ID_GET              0x95
#define ETH_CMD_SWITCH_RESET                 0x96
#define ETH_CMD_VLAN_CONTROL_SET             0x97
#define ETH_CMD_PRIO_INIT                    0x98
#define ETH_CMD_PRIO_REMAP_EN                0x99
#define ETH_CMD_PRIO_REMAP_SET               0x9A
#define ETH_CMD_PRIO_REMAP_GET               0x9B
#define ETH_CMD_L2_SEARCH_START              0x9C
#define ETH_CMD_l2_SEARCH_NEXT               0x9D
#define ETH_CMD_RATE_SET                     0x9E
#define ETH_CMD_RATE_GET                     0x9F
#define ETH_CMD_L2_ADDR_T_INIT               0xA0
#define ETH_CMD_L2_ADDR_ADD                  0xA1
#define ETH_CMD_L2_CLEAR                     0xA2

typedef struct
{
   int   unit;
} ETH_unit_t;

typedef struct
{
   int                  unit;
   eth_switch_control_t type;
   int                  *arg;
} ETH_switch_control_get_t;

typedef struct
{
   int                  unit;
   eth_switch_control_t type;
   int                  arg;
} ETH_switch_control_set_t;

typedef struct
{
   int            unit; 
   eth_port_t     port; 
   eth_stat_val_t type; 
   uint64_t       *value;
} ETH_stat_get_t;

typedef struct
{
   int            unit; 
   eth_port_t     port; 
   eth_stat_val_t type; 
   uint32_t       *value;
} ETH_stat_get32_t;

typedef struct
{
   int         unit; 
   eth_port_t  port;
} ETH_stat_clear_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   uint32_t    flags; 
   uint32_t    phy_reg_addr; 
   uint32_t    phy_data;
} ETH_port_phy_set_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   uint32_t    flags; 
   uint32_t    phy_reg_addr; 
   uint32_t    *phy_data;
} ETH_port_phy_get_t;

typedef ETH_unit_t ETH_vlan_init_t;

typedef struct
{
   int        unit; 
   eth_vlan_t vid;
} ETH_vlan_create_t;

typedef ETH_vlan_create_t ETH_vlan_destroy_t;

typedef ETH_unit_t ETH_vlan_destroy_all_t;

typedef struct
{
   int         unit; 
   eth_vlan_t  vid; 
   eth_pbmp_t  *pbmp; 
   eth_pbmp_t  *ubmp;
} ETH_vlan_port_get_t;

typedef struct
{
   int         unit;
   eth_vlan_t  vid; 
   eth_pbmp_t  pbmp; 
   eth_pbmp_t  ubmp;
} ETH_vlan_port_add_t;

typedef struct
{
   int         unit; 
   eth_vlan_t  vid; 
   eth_pbmp_t  pbmp;
} ETH_vlan_port_remove_t;

typedef struct
{
   int         unit; 
   int         port; 
   eth_vlan_t  old_vid; 
   eth_vlan_t  new_vid; 
   int         prio;
} ETH_vlan_translate_add_t;

typedef struct
{
   int         unit; 
   int         port; 
   eth_vlan_t  old_vid;
} ETH_vlan_translate_delete_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   eth_vlan_t  vid;
} ETH_port_untagged_vlan_set_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   eth_vlan_t  *vid_ptr;
} ETH_port_untagged_vlan_get_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   int         priority;
} ETH_port_untagged_priority_set_t;

typedef struct
{
    int        unit; 
    eth_port_t port; 
    int        *priority;
} ETH_port_untagged_priority_get_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   uint8_t     phy_id;
} ETH_port_phy_id_set_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   uint8_t     *phy_id;
} ETH_port_phy_id_get_t;

typedef ETH_unit_t ETH_switch_reset_t;

typedef struct
{
   int                  unit; 
   eth_vlan_control_t   type; 
   int                  arg;
} ETH_vlan_control_set_t;

typedef ETH_unit_t ETH_prio_init_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   int         enable;
} ETH_prio_remap_en_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   uint8_t     pre_prio; 
   uint8_t     prio;
} ETH_prio_remap_set_t;

typedef struct
{
   int         unit; 
   eth_port_t  port; 
   uint8_t     pre_prio; 
   uint8_t     *prio;
} ETH_prio_remap_get_t;

typedef ETH_unit_t ETH_l2_search_start_t;

typedef struct
{
   int            unit;
   eth_l2_addr_t  *l2addr;
   eth_l2_addr_t  *l2addr2;
} ETH_l2_search_next_t;

typedef struct
{
   int   unit;
   int   pps;
   int   flags;
} ETH_rate_set_t;
 
typedef struct
{
   int   unit;
   int   *pps;
   int   *flags;
} ETH_rate_get_t;

typedef struct 
{
   eth_l2_addr_t  *l2addr; 
   eth_mac_t      mac; 
   eth_vlan_t     vlan;
} ETH_l2_addr_t_init_t;

typedef struct 
{
   int            unit; 
   eth_l2_addr_t  *l2addr ;
} ETH_l2_addr_add_t;

typedef ETH_unit_t ETH_l2_clear_t;


#define ETH_IOCTL_SWITCH_CONTROL_GET         _IOWR(   ETH_MAGIC, ETH_CMD_SWITCH_CONTROL_GET, ETH_switch_control_get_t )
#define ETH_IOCTL_SWITCH_CONTROL_SET         _IOW(    ETH_MAGIC, ETH_CMD_SWITCH_CONTROL_SET, ETH_switch_control_set_t )
#define ETH_IOCTL_STAT_GET                   _IOWR(   ETH_MAGIC, ETH_CMD_STAT_GET, ETH_stat_get_t )
#define ETH_IOCTL_STAT_GET32                 _IOWR(   ETH_MAGIC, ETH_CMD_STAT_GET32, ETH_stat_get32_t )
#define ETH_IOCTL_STAT_CLEAR                 _IOW(    ETH_MAGIC, ETH_CMD_STAT_CLEAR, ETH_stat_clear_t )
#define ETH_IOCTL_PORT_PHY_SET               _IOW(    ETH_MAGIC, ETH_CMD_PORT_PHY_SET, ETH_port_phy_set_t )
#define ETH_IOCTL_PORT_PHY_GET               _IOWR(   ETH_MAGIC, ETH_CMD_PORT_PHY_GET, ETH_port_phy_get_t )
#define ETH_IOCTL_VLAN_INIT                  _IOW(    ETH_MAGIC, ETH_CMD_VLAN_INIT, ETH_vlan_init_t )
#define ETH_IOCTL_VLAN_CREATE                _IOW(    ETH_MAGIC, ETH_CMD_VLAN_CREATE, ETH_vlan_create_t )
#define ETH_IOCTL_VLAN_DESTROY               _IOW(    ETH_MAGIC, ETH_CMD_VLAN_DESTROY, ETH_vlan_destroy_t )
#define ETH_IOCTL_VLAN_DESTROY_ALL           _IOW(    ETH_MAGIC, ETH_CMD_VLAN_DESTROY_ALL, ETH_vlan_destroy_all_t )
#define ETH_IOCTL_VLAN_PORT_GET              _IOWR(   ETH_MAGIC, ETH_CMD_VLAN_PORT_GET, ETH_vlan_port_get_t )
#define ETH_IOCTL_VLAN_PORT_ADD              _IOW(    ETH_MAGIC, ETH_CMD_VLAN_PORT_ADD, ETH_vlan_port_add_t )
#define ETH_IOCTL_VLAN_PORT_REMOVE           _IOW(    ETH_MAGIC, ETH_CMD_VLAN_PORT_REMOVE, ETH_vlan_port_remove_t )
#define ETH_IOCTL_VLAN_TRANSLATE_ADD         _IOW(    ETH_MAGIC, ETH_CMD_VLAN_TRANSLATE_ADD, ETH_vlan_translate_add_t )
#define ETH_IOCTL_VLAN_TRANSLATE_DELETE      _IOW(    ETH_MAGIC, ETH_CMD_VLAN_TRANSLATE_DELETE, ETH_vlan_translate_delete_t )
#define ETH_IOCTL_PORT_UNTAGGED_VLAN_SET     _IOW(    ETH_MAGIC, ETH_CMD_PORT_UNTAGGED_VLAN_SET, ETH_port_untagged_vlan_set_t )
#define ETH_IOCTL_PORT_UNTAGGED_VLAN_GET     _IOWR(   ETH_MAGIC, ETH_CMD_PORT_UNTAGGED_VLAN_GET, ETH_port_untagged_vlan_get_t )
#define ETH_IOCTL_PORT_UNTAGGED_PRIORITY_SET _IOW(    ETH_MAGIC, ETH_CMD_PORT_UNTAGGED_PRIORITY_SET, ETH_port_untagged_priority_set_t )
#define ETH_IOCTL_PORT_UNTAGGED_PRIORITY_GET _IOWR(   ETH_MAGIC, ETH_CMD_PORT_UNTAGGED_PRIORITY_GET, ETH_port_untagged_priority_get_t )
#define ETH_IOCTL_PORT_PHY_ID_SET            _IOW(    ETH_MAGIC, ETH_CMD_PORT_PHY_ID_SET, ETH_port_phy_id_set_t )
#define ETH_IOCTL_PORT_PHY_ID_GET            _IOWR(   ETH_MAGIC, ETH_CMD_PORT_PHY_ID_GET, ETH_port_phy_id_get_t )
#define ETH_IOCTL_SWITCH_RESET               _IOW(    ETH_MAGIC, ETH_CMD_SWITCH_RESET, ETH_switch_reset_t )
#define ETH_IOCTL_VLAN_CONTROL_SET           _IOW(    ETH_MAGIC, ETH_CMD_VLAN_CONTROL_SET, ETH_vlan_control_set_t )
#define ETH_IOCTL_PRIO_INIT                  _IOW(    ETH_MAGIC, ETH_CMD_PRIO_INIT, ETH_prio_init_t )
#define ETH_IOCTL_PRIO_REMAP_EN              _IOW(    ETH_MAGIC, ETH_CMD_PRIO_REMAP_EN, ETH_prio_remap_en_t )
#define ETH_IOCTL_PRIO_REMAP_SET             _IOW(    ETH_MAGIC, ETH_CMD_PRIO_REMAP_SET, ETH_prio_remap_set_t )
#define ETH_IOCTL_PRIO_REMAP_GET             _IOWR(   ETH_MAGIC, ETH_CMD_PRIO_REMAP_GET, ETH_prio_remap_get_t )
#define ETH_IOCTL_L2_SEARCH_START            _IOW(    ETH_MAGIC, ETH_CMD_L2_SEARCH_START, ETH_l2_search_start_t )
#define ETH_IOCTL_l2_SEARCH_NEXT             _IOWR(   ETH_MAGIC, ETH_CMD_l2_SEARCH_NEXT, ETH_l2_search_next_t )
#define ETH_IOCTL_RATE_SET                   _IOW(    ETH_MAGIC, ETH_CMD_RATE_SET, ETH_rate_set_t )
#define ETH_IOCTL_RATE_GET                   _IOWR(   ETH_MAGIC, ETH_CMD_RATE_GET, ETH_rate_get_t )
#define ETH_IOCTL_L2_ADDR_T_INIT             _IOW(    ETH_MAGIC, ETH_CMD_L2_ADDR_T_INIT, ETH_l2_addr_t_init_t )
#define ETH_IOCTL_L2_ADDR_ADD                _IOW(    ETH_MAGIC, ETH_CMD_L2_ADDR_ADD, ETH_l2_addr_add_t )
#define ETH_IOCTL_L2_CLEAR                   _IOW(    ETH_MAGIC, ETH_CMD_L2_CLEAR, ETH_l2_clear_t )

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */
#if !defined( __KERNEL__ )

int eth_init( void );

// The following APIs are available in user-space - using the same API as the kernel side functions
int eth_switch_control_get( int unit, eth_switch_control_t type, int *arg );
int eth_switch_control_set( int unit, eth_switch_control_t type, int arg );

int eth_stat_get( int unit, eth_port_t port, eth_stat_val_t type, uint64_t *value );
int eth_stat_get32( int unit, eth_port_t port, eth_stat_val_t type, uint32_t *value );
int eth_stat_clear( int unit, eth_port_t port);

int eth_port_phy_set( int unit, eth_port_t port, uint32_t flags, uint32_t phy_reg_addr, uint32_t phy_data );
int eth_port_phy_get( int unit, eth_port_t port, uint32_t flags, uint32_t phy_reg_addr, uint32_t *phy_data );

int eth_vlan_init( int unit );
int eth_vlan_create( int unit, eth_vlan_t vid );
int eth_vlan_destroy( int unit, eth_vlan_t vid );
int eth_vlan_destroy_all( int unit );
int eth_vlan_port_get( int unit, eth_vlan_t vid, eth_pbmp_t *pbmp, eth_pbmp_t *ubmp );
int eth_vlan_port_add( int unit, eth_vlan_t vid, eth_pbmp_t pbmp, eth_pbmp_t ubmp );
int eth_vlan_port_remove( int unit, eth_vlan_t vid, eth_pbmp_t pbmp );
int eth_vlan_translate_add( int unit, int port, eth_vlan_t old_vid, eth_vlan_t new_vid, int prio );
int eth_vlan_translate_delete( int unit, int port, eth_vlan_t old_vid );

int eth_port_untagged_vlan_set( int unit, eth_port_t port, eth_vlan_t vid );
int eth_port_untagged_vlan_get( int unit, eth_port_t port, eth_vlan_t *vid_ptr );
int eth_port_untagged_priority_set( int unit, eth_port_t port, int priority );
int eth_port_untagged_priority_get( int unit, eth_port_t port, int *priority );

int eth_port_phy_id_set( int unit, eth_port_t port, uint8_t phy_id );
int eth_port_phy_id_get( int unit, eth_port_t port, uint8_t *phy_id );
int eth_switch_reset( int unit );

int eth_vlan_control_set( int unit, eth_vlan_control_t type, int arg );

int eth_prio_init( int unit );
int eth_prio_remap_en( int unit, eth_port_t port, int enable );
int eth_prio_remap_set( int unit, eth_port_t port, uint8_t pre_prio, uint8_t prio );
int eth_prio_remap_get( int unit, eth_port_t port, uint8_t pre_prio, uint8_t *prio );

int eth_l2_search_start( int unit );
int eth_l2_search_next( int unit, eth_l2_addr_t *l2addr, eth_l2_addr_t *l2addr2 );

int eth_rate_set( int unit, int pps, int flags );
int eth_rate_get( int unit, int *pps, int *flags );

void eth_l2_addr_t_init( eth_l2_addr_t *l2addr, eth_mac_t mac, eth_vlan_t vlan );
int eth_l2_addr_add( int unit, eth_l2_addr_t *l2addr );
int eth_l2_clear( int unit );

#endif  // !defined( __KERNEL__ )
        // 
#endif  // USER_ETH_H

