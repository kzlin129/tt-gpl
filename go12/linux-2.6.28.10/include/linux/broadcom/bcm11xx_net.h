/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
*  @file    bcm11xx_net.h
*
*  @brief   Upper ethernet device driver header file
*
*  This file contains the upper device driver header information for the 
*  ethernet module.
*  The functions in this file provide the hooks required to interface with a 
*  POSIX-compatible character driver.
*/
/****************************************************************************/

#ifndef BCM11XX_NET_H
#define BCM11XX_NET_H

/* ---- Include Files ----------------------------------------------------- */

#include <linux/sockios.h>

/* ---- Public Constants and Types ---------------------------------------- */

/* BCM11XX Device private ioctl calls */
#define SIOC_BCM11XX_PRIVATE    SIOCDEVPRIVATE
#define SIOCMIIMASKSET          SIOCDEVPRIVATE + 1
#define SIOCMIIMASKCLR          SIOCDEVPRIVATE + 2


typedef struct
{
   int          cmd;    /* Command to run */
   void __user  *arg;   /* Pointer to the argument structure */
} ETH_IOCTL_ARGS;


/**
*  @brief   Ethernet I/O control commands
*
*  The ethernet commands enumeration is used as the command argument of calls
*  to ethIoctl(), whose function prototype of ethIoctl():
*
*  @code
*  ethIoctl( int fd, int cmd, int arg )
*  @endcode
*
*  where:
*  - @c fd is the file descriptor used to uniquely identify the device
*  - @c cmd is ethernet I/O control command
*  - @c arg is an arbitrary argument used to pass data to/from the function.
*    This argument may be cast to any type, such as a pointer to a structure.
*
*  The following enumeration values are used as the @c cmd argument in the
*  call to ethIoctl().  The use of the arbitrary argument, @c arg, is 
*  described in description of the enumeration.
*
*  Unless otherwise stated, ethIoctl() will return 0 if the command was 
*  completed successfully, or -1 otherwise, and @c errno set to indicate the
*  error.
*/
typedef enum
{
   /**
   *  Get the transmit/receive status of the port at the MAC level.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MAC_ENABLE_GET
   *  @param   arg - [OUT] Pointer of structure @ref ETH_MAC_ENABLE, indicating  
   *                 the retrieved status 
   *
   *  <hr>
   */
   ETH_IOCTL_MAC_ENABLE_GET,

   /**
   *  Enable/Disable the transmit/receive function of the port at the MAC level.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MAC_ENABLE_SET
   *  @param   arg - [IN] Pointer of structure @ref ETH_MAC_ENABLE, indicating  
   *                 the desired state of the transmit/receive function
   *
   *  <hr>
   */
   ETH_IOCTL_MAC_ENABLE_SET,

   /**
   *  Configure and enable/disable perfect match register
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_PERFECT_MATCH_SET
   *  @param   arg - [IN] Pointer of structure @ref ETH_PM, indicating  
   *                 the MAC address and the desired state of the PM register
   *
   *  <hr>
   */
   ETH_IOCTL_PERFECT_MATCH_SET,

   /**
   *  Set bits in a port-based MII register.
   *  The MII is used to control features and query the status of the PHY.
   *  The bit mask is used to set bits in the MII data value.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MII_MASK_SET
   *  @param   arg - [IN] Pointer of type ETH_MII, indicating the desired bit
   *                 mask to be applied to the data value of the specified
   *                 port and specified MII register address
   *
   *  @sa
   *     @ref eth_features_mii example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MII_MASK_SET,

   /**
   *  Clear bits in a port-based MII register.
   *  The MII is used to control features and query the status of the PHY.
   *  The bit mask is used to clear bits in the MII data value.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MII_MASK_CLR
   *  @param   arg - [IN] Pointer of type ETH_MII, indicating the desired bit
   *                 mask to be applied to the data value of the specified
   *                 port and specified MII register address
   *
   *  @sa
   *     @ref eth_features_mii example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MII_MASK_CLR,
   
   /**
   *  Get the value of a port-based MIB counter.
   *  The MIB is used to keep statistics on Rx and Tx frames.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MIB_GET
   *  @param   arg - [OUT] Pointer of type ETH_MIB, indicating the current
   *                 data value of the specified port and specified MIB type
   *
   *  @sa
   *     @ref eth_features_mib example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MIB_GET,

   /**
   *  Clear the values of the port-based MIB counters.
   *  Depending on the capabilities of MIB counters in the silicon-dependent
   *  switch, clearing the counters of one port may clear counters for all
   *  other ports as well.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MIB_CLR
   *  @param   arg - [IN] Data of type @c int with the desired port to clear
   *
   *  @sa
   *     @ref eth_features_mib example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MIB_CLR,

   /**
   *  Get the high priority queue configuration.
   *  All frames deemed to be high priority (based on either the 802.1p field
   *  or how the frame is queued by the internal port) will be placed in the
   *  high priority queue and processed by the switch before non-high priority
   *  frames.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_HIQ_GET
   *  @param   arg - [OUT] Pointer of type ETH_HIQ, indicating the current
   *                 configuration of the high priority queues
   *
   *  @sa
   *     @ref eth_features_hiq example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_HIQ_GET,

   /**
   *  Set the high priority queue configuration.
   *  All frames deemed to be high priority (based on either the 802.1p field
   *  or how the frame is queued by the internal port) will be placed in the
   *  high priority queue and processed by the switch before non-high priority
   *  frames.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_HIQ_SET
   *  @param   arg - [IN] Pointer of type ETH_HIQ, indicating the desired
   *                 configuration of the high priority queues
   *
   *  @sa
   *     @ref eth_features_hiq example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_HIQ_SET,

   /**
   *  Get the 802.1p module configuration.
   *  The 802.1p module supports priority field remapping.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_802_1P_GET
   *  @param   arg - [OUT] Pointer of type ETH_802_1P, indicating the current
   *                 configuration of the 802.1p module
   *
   *  @sa
   *     @ref eth_features_vlan example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_802_1P_GET,

   /**
   *  Set the 802.1p configuration.
   *  The 802.1p module supports priority field remapping.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_802_1P_SET
   *  @param   arg - [IN] Pointer of type ETH_802_1P, indicating the desired
   *                 configuration of the 802.1p module
   *
   *  @sa
   *     @ref eth_features_vlan example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_802_1P_SET,
   
   /**
   *  Get the 802.1Q module configuration.
   *  The 802.1Q module supports features such as VLAN ID filtering
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_802_1Q_GET
   *  @param   arg - [OUT] Pointer of type ETH_802_1Q, indicating the current
   *                 configuration of the 802.1Q module
   *
   *  @sa
   *     @ref eth_features_vlan example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_802_1Q_GET,

   /**
   *  Set the 802.1Q configuration.
   *  The 802.1Q module supports features such as VLAN ID filtering
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_802_1Q_SET
   *  @param   arg - [IN] Pointer of type ETH_802_1Q, indicating the desired
   *                 configuration of the 802.1Q module
   *
   *  @sa
   *     @ref eth_features_vlan example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_802_1Q_SET,

   /**
   *  Get the secured port configuration.
   *  The secured port settings are used to create private tunnels between
   *  ports for applications such as VPN.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_SECURE_PORT_GET
   *  @param   arg - [OUT] Pointer of type ETH_SECURE_PORT, indicating the
   *                 current configuration of the secured port settings
   *
   *  @sa
   *     @ref eth_features_portIso example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_SECURE_PORT_GET,

   /**
   *  Set the secured port configuration.
   *  The secured port settings are used to create private tunnels between
   *  ports for applications such as VPN.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_SECURE_PORT_SET
   *  @param   arg - [IN] Pointer of type ETH_SECURE_PORT, indicating the
   *                 desired configuration of the secured port settings
   *
   *  @sa
   *     @ref eth_features_portIso example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_SECURE_PORT_SET,

   /**
   *  Get the mirror port configuration.
   *  The mirror port settings are used to mirror ingress packets between
   *  ports.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MIRROR_PORT_GET
   *  @param   arg - [OUT] Pointer of type ETH_MIRROR_PORT, indicating the
   *                 current configuration of the mirror port settings
   *
   *  @sa
   *     @ref eth_features_mirror example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MIRROR_PORT_GET,

   /**
   *  Set the mirror port configuration.
   *  The mirror port settings are used to mirror packets between ports
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MIRROR_PORT_SET
   *  @param   arg - [IN] Pointer of type ETH_MIRROR_PORT, indicating the
   *                 desired configuration of the mirror port settings
   *
   *  @sa
   *     @ref eth_features_mirror example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MIRROR_PORT_SET,

   /**
   *  Get the multiport vector configuration.
   *  The multiport vector is used to forward a configured destination MAC
   *  address to configured port(s), bypassing the ARL and VLAN forwarding
   *  rules.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MULTIPORT_GET
   *  @param   arg - [OUT] Pointer of type ETH_MULTIPORT, indicating the
   *                 current configuration of the multiport vector settings
   *
   *  @sa
   *     @ref eth_features_multiport example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MULTIPORT_GET,

   /**
   *  Set the multiport vector configuration.
   *  The multiport vector is used to forward a configured destination MAC
   *  address to configured port(s), bypassing the ARL and VLAN forwarding
   *  rules.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_MULTIPORT_SET
   *  @param   arg - [IN] Pointer of type ETH_MULTIPORT, indicating the
   *                 desired configuration of the multiport vector settings
   *
   *  @sa
   *     @ref eth_features_multiport example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_MULTIPORT_SET,

   /**
   *  Get the BPDU MAC address.
   *  The BPDU MAC address is used to override the existing address.  All
   *  incoming frames with a destination MAC address matching the BPDU MAC
   *  address are forwarded to the internal port.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_BPDU_GET
   *  @param   arg - [OUT] Pointer of type @ref ETH_MAC, indicating the
   *                 current BPDU MAC address
   *
   *  @sa
   *     @ref eth_features_bpdu example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_BPDU_GET,

   /**
   *  Set the BPDU MAC address.
   *  The BPDU MAC address is used to override the existing address.  All
   *  incoming frames with a destination MAC address matching the BPDU MAC
   *  address are forwarded to the internal port.  By default, the BPDU MAC
   *  address is 01-80-c2-00-00-00.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_BPDU_SET
   *  @param   arg - [IN] Pointer of type @ref ETH_MAC, indicating the desired
   *                 BPDU MAC address
   *
   *  @sa
   *     @ref eth_features_bpdu example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_BPDU_SET,

   /**
   *  Get the internal forwarding port mask.
   *  When a particular frame type bit mask is set, all incoming frames
   *  with that frame type are forwarded to the internal port.  If
   *  @ref ETH_MAC_TYPE_MASK_UCAST mask is set, unlearned unicast destination
   *  addresses will be forwarded to the internal port.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_FWD_INT_GET
   *  @param   arg - [OUT] Data of type @ref ETH_MAC_TYPE_MASK, indicating the
   *                 current mask used to define which frame types are
   *                 forwarded to the internal port
   *
   *  @sa
   *     @ref eth_features_intFwd example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_FWD_INT_GET,

   /**
   *  Set the internal forwarding port mask.
   *  When a particular frame type bit mask is set, all incoming frames with
   *  that frame type are forwarded to the internal port.  If
   *  @ref ETH_MAC_TYPE_MASK_UCAST mask is set, unlearned unicast destination
   *  addresses will be forwarded to the internal port.  By default, the mask
   *  is set to (@ref ETH_MAC_TYPE_MASK_MCAST | @ref ETH_MAC_TYPE_MASK_BCAST),
   *  so incoming multicast and broadcast frames are forwarded to the internal
   *  port.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_FWD_INT_SET
   *  @param   arg - [IN] Data of type @ref ETH_MAC_TYPE_MASK, indicating
   *                 the current mask used to define which frame types are
   *                 forwarded to the internal port
   *
   *  @sa
   *     @ref eth_features_intFwd example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_FWD_INT_SET,

   /**
   *  Get the address resolution logic configuration.
   *  The ARL is used to control how frames are bridged in the switch.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_ARL_GET
   *  @param   arg - [OUT] Pointer of type ETH_ARL, indicating the current
   *                 configuration of the ARL
   *
   *  @sa
   *     @ref eth_features_arl example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_ARL_GET,

   /**
   *  Set the address resolution logic configuration.
   *  The ARL is used to control how frames are bridged in the switch.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_ARL_SET
   *  @param   arg - [IN] Pointer of type ETH_ARL, indicating the desired
   *                 configuration of the ARL
   *
   *  @sa
   *     @ref eth_features_arl example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_ARL_SET,

   /**
   *  Get an entry from the ARL table.
   *  An ARL entry associates an external MAC address to a physical port.
   * 
   *  @param   cmd - [IN] @c ETH_IOCTL_ARL_ENTRY_GET
   *  @param   arg - [OUT] Pointer of type ETH_ARL_ENTRY, indicating the
   *                 current settings for the entry matching the specified
   *                 MAC address in the ARL table
   *
   *  @sa
   *     @ref eth_features_arl example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_ARL_ENTRY_GET,

   /**
   *  Set an entry in the ARL table.
   *  An ARL entry associates an external MAC address to a physical port.
   * 
   *  @param   cmd - [IN] @c ETH_IOCTL_ARL_ENTRY_SET
   *  @param   arg - [IN] Pointer of type ETH_ARL_ENTRY, indicating the
   *                 desired settings for the entry matching the specified
   *                 MAC address in the ARL table
   *
   *  @sa
   *     @ref eth_features_arl example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_ARL_ENTRY_SET,

   /**
   *  Clear an entry in the ARL table.
   *  An ARL entry associates an external MAC address to a physical port.
   * 
   *  @param   cmd - [IN] @c ETH_IOCTL_ARL_ENTRY_CLR
   *  @param   arg - [IN] Pointer of type @ref ETH_MAC, indicating the MAC
   *                 address for the entry in the ARL table to be cleared
   *
   *  @sa
   *     @ref eth_features_arl example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_ARL_ENTRY_CLR,

   /**
   *  Get a valid entry from the ARL table.
   *  The ARL table contains a finite list of ARL entries.  Each call to
   *  ethIoctl() with this command will increment an internal pointer to the
   *  next valid entry.  It can be used to display all of the valid entries in
   *  the ARL table.  If this command is called with a NULL argument, the
   *  internal entry pointer is reset to the beginning of the ARL table.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_ARL_TABLE_GET
   *  @param   arg - [OUT] Pointer of type ETH_ARL_ENTRY, indicating the
   *                 settings of a valid entry in the ARL table
   *
   *  @sa
   *     @ref eth_features_arl example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_ARL_TABLE_GET,

   /**
   *  Clear the ARL table.
   *  The ARL table contains a finite list of ARL entries. 
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_ARL_TABLE_CLR
   *  @param   arg - [IN] @c 0
   *
   *  @sa
   *     @ref eth_features_arl example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_ARL_TABLE_CLR,

   /**
   *  Get the source address filtering configuration.
   *  The source MAC address filtering feature can be used for security
   *  purposes by allowing or denying frames with a particular source MAC
   *  address.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_SA_FILTER_GET
   *  @param   arg - [OUT] Pointer of type ETH_SA_FILTER, indicating the
   *                 current configuration of the source address filtering
   *                 settings
   *
   *  @sa
   *     @ref eth_features_saFilter example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_SA_FILTER_GET,

   /**
   *  Set the source MAC address filtering configuration.
   *  The source MAC address filtering feature can be used for security
   *  purposes by allowing or denying frames with a particular source MAC
   *  address.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_SA_FILTER_SET
   *  @param   arg - [IN] Pointer of type ETH_SA_FILTER, indicating the
   *                 current configuration of the source address filtering
   *                 settings
   *
   *  @sa
   *     @ref eth_features_saFilter example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_SA_FILTER_SET,

   /**
   *  Get the 802.1X authentication configuration.
   *  The 802.1X authentication feature can be used for security purposes by
   *  allowing or denying frames with a particular source MAC address, based
   *  on the 802.1X standard.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_802_1X_GET
   *  @param   arg - [OUT] Pointer of type ETH_802_1X, indicating the current
   *                 configuration of the 802.1X authentication settings
   *
   *  @sa
   *     @ref eth_features_auth example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_802_1X_GET,

   /**
   *  Set the 802.1X authentication configuration.
   *  The 802.1X authentication feature can be used for security purposes by
   *  allowing or denying frames with a particular source MAC address, based
   *  on the 802.1X standard.
   *
   *  @param   cmd - [IN] @c ETH_IOCTL_802_1X_SET
   *  @param   arg - [IN] Pointer of type ETH_802_1X, indicating the desired
   *                 configuration of the 802.1X authentication settings
   *
   *  @sa
   *     @ref eth_features_auth example in ethernet driver documentation
   *
   *  <hr>
   */
   ETH_IOCTL_802_1X_SET

} ETH_IOCTL;


/**
*  @brief   Ethernet MAC address type
*
*  The ethernet MAC address type is used to define the type for an ethernet
*  MAC address.
*/
typedef char ETH_MAC[6];


/**
*  @brief   MAC address print string macro 
*
*  Defines a string that can be used in a printf() statement to format and
*  display a MAC address.
*/
#define ETH_MAC_PRINT_STR        "%02X-%02X-%02X-%02X-%02X-%02X"


/**
*  @brief   MAC address print argument macro 
*
*  Defines an argument list that can be used in a printf() statement to
*  format and display a MAC address.
*/
#define ETH_MAC_PRINT_ARG(mp)    ((unsigned char *)(mp))[0], \
                                 ((unsigned char *)(mp))[1], \
                                 ((unsigned char *)(mp))[2], \
                                 ((unsigned char *)(mp))[3], \
                                 ((unsigned char *)(mp))[4], \
                                 ((unsigned char *)(mp))[5]


/**
*  @brief   Ethernet port enumeration
*
*  The ethernet port enumeration is used to define the physical switch port
*  used for selected @ref ETH_IOCTL commands in calls to ethIoctl().
*/
typedef enum
{
   ETH_PORT_0 = 0,         /**< External port 0 */
   ETH_PORT_1,             /**< External port 1 */
   ETH_PORT_INT = 10       /**< Internal port of device */
} ETH_PORT;


/**
*  @brief   Ethernet port mask enumeration
*
*  The ethernet port mask enumeration is used to define the physical switch
*  ports used for selected @ref ETH_IOCTL commands in calls to ethIoctl().
*/
typedef enum
{
   ETH_PORT_MASK_NONE = 0x0000,  /**< No mask */
   ETH_PORT_MASK_0    = 0x0001,  /**< External port 0 mask */
   ETH_PORT_MASK_1    = 0x0002,  /**< External port 1 mask */
   ETH_PORT_MASK_INT  = 0x0400   /**< Internal port of device mask */
} ETH_PORT_MASK;


/**
*  @brief   Ethernet MAC types
*
*  The ethernet MAC type enumeration is used to define the various types of
*  MAC types.
*/
typedef enum
{
   ETH_MAC_TYPE_UCAST,     /**< Unicast MAC type */
   ETH_MAC_TYPE_BCAST,     /**< Broadcast MAC type */
   ETH_MAC_TYPE_MCAST,     /**< Multicast MAC type */
   ETH_MAC_TYPE_ALL        /**< All MAC types */
} ETH_MAC_TYPE;


/**
*  @brief   Ethernet MAC types
*
*  The ethernet MAC type enumeration is used to define the various types 
*  of MAC addresses.
*/
typedef enum
{
   ETH_MAC_TYPE_MASK_NONE  = 0x00000000,  /**< No mask */
   ETH_MAC_TYPE_MASK_UCAST = 0x00000001,  /**< Unicast MAC type mask */
   ETH_MAC_TYPE_MASK_BCAST = 0x00000002,  /**< Broadcast MAC type mask */
   ETH_MAC_TYPE_MASK_MCAST = 0x00000004,  /**< Multicast MAC type mask */
   ETH_MAC_TYPE_MASK_BPDU = 0x00000008   /**< Multicast MAC type mask */
} ETH_MAC_TYPE_MASK;


/**
*  @brief   Ethernet MAC transmit/receive function status
*
*  The ethernet MAC transmit and receive function status 
*
*  @sa
*     ETH_IOCTL_MAC_ENABLE_GET, ETH_IOCTL_MAC_ENABLE_SET
*
*/
typedef struct
{
   ETH_PORT port;
   int txEnable;
   int rxEnable;
} ETH_MAC_ENABLE;


/**
*  @brief   Ethernet priority information type
*
*  The ethernet priority information type is used to define the type for
*  configuring priority information.  Array index 0 refers to priority 
*  level 0, and so on.
*/
typedef int ETH_PRI_INFO[8];


/**
*  @brief   Ethernet high priority queue configuration
*
*  The ethernet high priority queue configuration structure is use to define
*  the behaviour of the high priority queues.
*
*  @sa
*     ETH_IOCTL_HIQ_GET, ETH_IOCTL_HIQ_SET
*
*/
typedef struct
{
   ETH_PORT port;             /**< Port number to configure */
   int depth;                 /**< Queue depth */
   int force;                 /**< Force high priority queuing on ingress frames */
   int weight;                /**< Weighting to define the rate at which non-high priority
                                   frames are passed (one non-high priority frame for every
                                   @c weight number of high priority frames.  A @c weight of
                                   -1 causes the non-high priority frames to be blocked until
                                   all high priority frames are passed) */
   ETH_PRI_INFO map;          /**< Flags to control the 802.1p priority level mapping which
                                   will force a frame into the high priority queue */   
} ETH_HIQ;                    /**< High-priority queue configuration */


/**
*  @brief   Ethernet 802.1p configuration
*
*  The ethernet 802.1p configuration structure is used to define
*  the behaviour of the 802.1p priority retagging.  Note that the 
*  802.1pQ feature enable flag is shared with the ETH_802_1Q structure.  If
*  it is disabled here, it will also disable it for 802.1Q VLAN.
*
*  @sa
*     ETH_IOCTL_802_1P_GET, ETH_IOCTL_802_1P_SET
*
*/
typedef struct
{
   ETH_PORT port;          /**< Port number to configure */
   int enable;             /** < 802.1pQ feature enable flag */
   struct
   {
      int enable;          /**< Enable flag for regeneration control */
      ETH_PRI_INFO map;    /**< Priority mapping table */
   } regen;                /**< Priority regeneration control */
} ETH_802_1P;


/**
*  @brief   Ethernet 802.1Q configuration
*
*  The ethernet 802.1Q configuration structure is used to define
*  the behaviour of the 802.1Q VLAN features.  Note that the 
*  802.1pQ feature enable flag is shared with the ETH_802_1P structure.  If
*  it is disabled here, it will also disable it for 802.1p priority retagging.
*
*  @sa
*     ETH_IOCTL_802_1Q_GET, ETH_IOCTL_802_1Q_SET
*
*/
typedef struct
{
   ETH_PORT port;          /**< Port number to configure */
   int enable;             /**< 802.1pQ feature enable flag */
   int priority;           /**< Configured priority for port */
   int vlanId;             /**< Configured VLAN ID for port */
   int vlanIdOther[2];     /**< Other VLAN ID used for filtering (set to -1 to disable) */
   int gmrpFwd;            /**< Flag to forward GMRP frames to internal port */
   int gvrpFwd;            /**< Flag to forward GVRP frames to internal port */
   int filter;             /**< Flag to enable VLAN ID filtering */
   int admitVlanTagged;    /**< Admit only ingress VLAN-tagged frames */
   int vlanIdFilter;       /**< Discard ingress frames with unmatching VLAN ID */
   int replaceTag;         /**< Replace existing ingress tag with configured priority and VLAN ID */
   int addTag;             /**< Add tag with configured priority and VLAN ID to ingress frames */
   int allowUntagged;      /**< Allow untagged ingress frames to be forwarded to port */
   int removeTag;          /**< Remove tag from ingress frames forwarded to port */
   int allowAnyVlanId;     /**< Allow any ingress VLAN ID to be forwarded to port */
} ETH_802_1Q;


/**
*  @brief   Ethernet secure port configuration
*
*  The ethernet secure port configuration structure is use to define how the
*  switch bridges frames within a secured network configuration.  The secured
*  source ports will only forward ingress frames to the secured destination
*  ports.
*
*  @sa
*     ETH_IOCTL_SECURE_PORT_GET, ETH_IOCTL_SECURE_PORT_SET
*
*/
typedef struct
{
   ETH_PORT_MASK src;         /**< Source port mask */
   ETH_PORT_MASK dst;         /**< Destination port mask */
} ETH_SECURE_PORT;


/**
*  @brief   Ethernet mirror port configuration
*
*  The ethernet mirror port configuration structure is use to define the
*  source and destination ports for the port mirroring feature.  This feature
*  will mirror a copy of a frame from the specified source port(s) to the 
*  specified destination port(s).  Note that most chip implementations 
*  support a single destination port.  In such cases, specifying multiple 
*  destination ports will result in unpredictable behaviour.
*
*  @sa
*     ETH_IOCTL_MIRROR_PORT_GET, ETH_IOCTL_MIRROR_PORT_SET
*
*/
typedef struct
{
   struct
   {
      ETH_PORT_MASK ingress;  /**< Ingress source port mask */
      ETH_PORT_MASK egress;   /**< Egress source port mask */
   } src;
   ETH_PORT_MASK dst;         /**< Destination port mask */
} ETH_MIRROR_PORT;


/**
*  @brief   Ethernet multiport vector configuration
*
*  The ethernet multiport vector configuration structure is used to define
*  where the configured destination MAC address is forwarded, bypassing the
*  ARL forwarding rules.
*
*  @sa
*     ETH_IOCTL_MULTIPORT_GET, ETH_IOCTL_MULTIPORT_SET
*
*/
typedef struct
{
   int enable;                /**< Enable */
   ETH_MAC mac;               /**< Destination multiport MAC address */
   ETH_PORT_MASK dst;         /**< Forwarding port mask */
} ETH_MULTIPORT;


/**
*  @brief   Ethernet address resolution logic configuration
*
*  The ethernet ARL configuration structure is used to define the behaviour
*  of the ARL.
*
*  @sa
*     ETH_IOCTL_ARL_GET, ETH_IOCTL_ARL_SET
*
*/
typedef struct
{
   int age;                   /**< Aging time (in seconds) of entries in the ARL table */
} ETH_ARL;


/**
*  @brief   Ethernet address resolution logic table entry
*
*  The ethernet ARL table entry structure is used to define the settings of the
*  specified MAC address in the ARL table.
*
*  @sa
*     ETH_IOCTL_ARL_ENTRY_GET, ETH_IOCTL_ARL_ENTRY_SET,
*     ETH_IOCTL_ARL_ENTRY_CLR, ETH_IOCTL_ARL_TABLE_GET,
*     ETH_IOCTL_ARL_TABLE_CLR
*
*/
typedef struct
{
   ETH_MAC mac;               /**< MAC address */
   ETH_PORT port;             /**< Port number */
   int learned;               /**< Flag to indicate MAC address is learned */
   int staticEntry;           /**< Flag to indicate MAC address is a static entry */
   int index;                 /**< ARL table index value (only used with get operations) */
} ETH_ARL_ENTRY;


/**
*  @brief   Ethernet source MAC address filtering configuration
*
*  The ethernet source MAC address filtering configuration is used to define
*  the behaviour of the source MAC address filtering feature for the specified
*  port.
*
*  @sa
*     ETH_IOCTL_SA_FILTER_GET, ETH_IOCTL_SA_FILTER_SET
*
*/
typedef struct
{
   ETH_PORT port;             /**< Port number */
   int enable;                /**< Flag to indicate whether source MAC address filtering is enabled */
   int dropOnMatch;           /**< Flag to indicate whether ingress source MAC adresses matching 
                                   the specified source MAC filter address are dropped */
   ETH_MAC mac;               /**< Source MAC filter address */
} ETH_SA_FILTER;


/**
*  @brief   Ethernet 802.1X authentication configuration
*
*  The ethernet 802.1X authentication configuration is used to define the
*  behaviour of the 802.1X authentication for the specified port.  When 802.1X
*  authentication is disabled, switch operation proceeds as a normal ethernet
*  switch.  When a port is configured in the <i>unauthorized</i> state, only
*  frames with EAPOL and BPDU destination MAC addresses will be forwarded by
*  the port.  All other frames are denied access to the LAN and will be
*  dropped.  When a port is configured in the <i>authorized</i> state, only
*  frames with EAPOL and BPDU destination MAC addresses, and source MAC
*  addresses that are valid in the ARL table will be forwarded by the port to
*  the LAN.  All other frames (frames whose source MAC address is not in the
*  ARL table) will be dropped.
*
*  @sa
*     ETH_IOCTL_802_1X_GET, ETH_IOCTL_802_1X_SET
*
*/
typedef struct
{
   ETH_PORT port;             /**< Port number */
   int enable;                /**< Flag to indicate whether specified port is in 802.1X mode */
   int unauthorized;          /**< Flag to indicate whether the specified port is in the
                                   unauthorized state */
} ETH_802_1X;

/**
*  @brief   Perfect match register configuration
*
*  The perfect match registers are used to filter out packets with unmatching DA
*  when promiscuous mode is not enabled.
*
*  @sa
*     ETH_IOCTL_PERFECT_MATCH_SET
*
*/
typedef struct
{
   int port;                  /**< port */
   int index;                 /**< index of perfect match register */
   int isValid;               /**< flag to indicate whether the content of the PM register is valid */
   ETH_MAC mac;               /**< MAC to preload the PM register */
} ETH_PM;

/**
*  @brief   MIB type
*
*  The MIB type enumeration is used to select the appropriate MIB when doing
*  MIB operations.
*/
typedef enum
{
   MIB_TYPE_TX_OCTETS,           /**< Number of octets transmitted (including FCS) */
   MIB_TYPE_TX_OCTETS2,          /**< Extension to number of octets transmitted (including FCS) */
   MIB_TYPE_TX_DROP,             /**< Number of frames dropped on transmit */
   MIB_TYPE_TX_BCAST,            /**< Number of frames broadcast on transmit */
   MIB_TYPE_TX_MCAST,            /**< Number of frames multicast on transmit */
   MIB_TYPE_TX_UCAST,            /**< Number of frames unicast on transmit */
   MIB_TYPE_TX_COL,              /**< Number of transmit collisions */
   MIB_TYPE_TX_COL_SINGLE,       /**< Number of transmit single collisions */
   MIB_TYPE_TX_COL_MULTI,        /**< Number of transmit multiple collisions */
   MIB_TYPE_TX_COL_LATE,         /**< Number of transmit late collisions */
   MIB_TYPE_TX_COL_EXCESS,       /**< Number of transmit excessive collisions */
   MIB_TYPE_TX_DEFERRED,         /**< Number of deferred transmits */
   MIB_TYPE_TX_PAUSE,            /**< Number of pause frames transmitted */
   MIB_TYPE_TX_DISCARDED,        /**< Number of frames discarded due to lack of space on output queue */

   MIB_TYPE_RX_OCTETS,           /**< Number of octets received from all frames (including FCS) */
   MIB_TYPE_RX_OCTETS2,          /**< Extension to number of octets received (including FCS) */
   MIB_TYPE_RX_OCTETS_GOOD,      /**< Number of octets received from good frames (including FCS) */
   MIB_TYPE_RX_OCTETS_GOOD2,     /**< Extension to number of octets received from good frames (including FCS) */
   MIB_TYPE_RX_UNDERSIZED,       /**< Number of undersized (runt) frames received */
   MIB_TYPE_RX_PAUSE,            /**< Number of pause frames received */
   MIB_TYPE_RX_OVERSIZED,        /**< Number of oversized frames received */
   MIB_TYPE_RX_JABBERS,          /**< Number of jabber frames received */
   MIB_TYPE_RX_ALIGN_ERR,        /**< Number of frames with alignment errors received */
   MIB_TYPE_RX_FCS_ERR,          /**< Number of frames with FCS errors received */
   MIB_TYPE_RX_DROP,             /**< Number of frames receive frames dropped */
   MIB_TYPE_RX_UCAST,            /**< Number of unicast frames received */
   MIB_TYPE_RX_MCAST,            /**< Number of multicast frames received */
   MIB_TYPE_RX_BCAST,            /**< Number of broadcast frames received */
   MIB_TYPE_RX_SA_CHANGE,        /**< Number of SA changes received */
   MIB_TYPE_RX_FRAGMENT,         /**< Number of fragments received */
   MIB_TYPE_RX_SIZE_EXCESS,      /**< Number of frames with excessive size received */
   MIB_TYPE_RX_SYMBOL_ERR,       /**< Number of frames with symbol errors */

   MIB_TYPE_RX_64_OCTETS,        /**< Number of 64 octet frames transmitted and/or received */
   MIB_TYPE_RX_65_127_OCTETS,    /**< Number of 65 to 127 octet frames transmitted and/or received */
   MIB_TYPE_RX_128_255_OCTETS,   /**< Number of 128 to 255 octet frames transmitted and/or received */
   MIB_TYPE_RX_256_511_OCTETS,   /**< Number of 256 to 511 octet frames transmitted and/or received */
   MIB_TYPE_RX_512_1023_OCTETS,  /**< Number of 512 to 1023 octet frames transmitted and/or received */
   MIB_TYPE_RX_1024_1522_OCTETS  /**< Number of 1024 to 1522 octet frames transmitted and/or received */
} MIB_TYPE;


/**
*  @brief   Ethernet MIB data
*
*  The ethernet MIB data structure is used to retrieve MIB information from
*  the switch for the specified port.
*
*  @sa
*     ETH_IOCTL_MIB_GET, ETH_IOCTL_MIB_SET
*
*/
typedef struct
{
   ETH_PORT port;          /**< Port number */
   int data;               /**< MIB data value */
   MIB_TYPE type;          /**< MIB type */
} ETH_MIB;

#endif
