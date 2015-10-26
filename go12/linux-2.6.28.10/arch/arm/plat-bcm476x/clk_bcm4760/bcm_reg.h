/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/** 
 * 
 *   @file   bcm_reg.h
 * 
 *   @brief  Definitions used for 32/16/8bit register read / write access.
 *           Also defined here are helper macros for wrapping a 
 *           union around a bitfield register definition and a basic scalar
 *           type.
 * 
 ****************************************************************************/

#ifndef _BCM_REG_H_
#define _BCM_REG_H_

/**
    <b>How to declare a register using bit fields?</b>

        Follow these steps to declare a register as a bit 
           field structure :
           1. Define the register structure
                typedef struct
                {                        
                  //register bit info goes here..
                  uint32_t bit0:1;  //Bit-0 start here for Little Endian Systems
                  .............
                  .............
                  uint32_t bit31:1; //Bit-31 here for Little Endian Systems
                }clkmgr_cmplls_t;

           2. Use the macro to create a union of the register 
              struct and uint32_t
                AMBA_REG_BLD_UNION32(clkmgr_cmplls);


     <b> How to access register as word or as bit-fields? </b>

        Once the register has been declared as above,
           (1) Get the base address for the module from the addressmap.h file.
               Note: This is a one run-time operation and you don't have 
                     obtain the address for each register during run-time.

               static clkmgr_regs_t *clkmgr_base = NULL;
               clkmgr_base = (clkmgr_regs_t *)IO_ADDRESS(CLKMGR_ARM_ADDRBASE0);

           (2) writing to a register

               clkmgr_cmplls_u_t  clkreg;
               uint32_t data2write;

               clkreg.wdata = some_value; //will set the entire register
               amba_reg_write32(clkmgr_base, cmplls, data2write);

           (2) Reading from a register

               clkmgr_cmplls_u_t  clkreg;
               uint32_t data2write;

               amba_reg_read32(clkmgr_base, cmplls)

               clkreg.wdata = some_value; //will set the entire register
               amba_reg_write32(clkmgr_base, cmplls, data2write);        


        For read-modify-write just to one field, you can use the following macro:
        AMBA_REG_WR_BF32(block-regaddr, register-type, register, bit-field, value-to-write)

        For eg: AMBA_REG_WR_BF32(clkmgr_base, clkmgr_cmplls_u_t, cmplls, enab, 1)

   @ingroup BCM_BASETYPES

  */
/***********************************************************
*
* Included files
*
***********************************************************/
#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_amba_io.h"

/***********************************************************
*
* Defined values
*
***********************************************************/

/**
    @def  BCM_BIT_CLR
    @brief Used for clearing a bit in the register to 0
*/
#define BCM_BIT_CLR  0

/**
    @def  BCM_BIT_SET
    @brief Used for setting a bit in the register to 1
*/
#define BCM_BIT_SET  1

/**
    @def  REG_GETBIT_STAT
    @brief  0 or 1 depending on whether the bit is set or clear
 */
#define REG_GETBIT_STAT(value, bit_pos) ((value & (0x00000001 << bit_pos)) >> bit_pos)


/***********************************************************
*
* Enumerated types
*
***********************************************************/

/***********************************************************
*
* Macro definitions
*
***********************************************************/
#ifdef SYM_OSTYPE_LINUX
//--------------------------------------------------
/**
   Macro for reading a 32bit register value.  

   @ingroup BCM_BASETYPES
 */
//--------------------------------------------------
#define amba_reg_read32( regbase, regname )                 \
    amba_read32((volatile uint32_t *)&(*regbase).regname.w_data)                                

//--------------------------------------------------
/**
   Macro for writing a 32bit register value.  

   @ingroup BCM_BASETYPES
 */
//--------------------------------------------------

#define amba_reg_write32( regbase, regname,  regdata)        \
    amba_write32((volatile uint32_t *)&(*regbase).regname.w_data, (uint32_t) regdata);

#else //for CLDB

#define amba_reg_read32( regbase, regname )                 \
    amba_read32((volatile uint32_t *)&(*regbase).regname.w_data)                                

#define amba_reg_write32( regbase, regname,  regdata)        \
    amba_write32((volatile uint32_t *)&(*regbase).regname.w_data, (uint32_t) regdata);

#endif

//--------------------------------------------------
/**
  Macro for building a union register definition using
  both the bitfield definition and underlying 32bit word type.

  @ingroup BCM_REG
*/
//-------------------------------------------------- 
#define AMBA_REG_BLD_UNION32(reg_base_name)                          \
typedef union                                                        \
{                                                                    \
    uint32_t       w_data;                                           \
    reg_base_name##_t bf;                                            \
} reg_base_name##_u_t, *reg_base_name##_u_ptr_t


//--------------------------------------------------
/**
  Macro for building a union register definition using
  both the bitfield definition and underlying 32bit word type.
  Same as the AMBA_REF_BLD_UNION32 macro. But this one defines
  a compare function as well.

  @ingroup BCM_REG
*/
//-------------------------------------------------- 
#define AMBA_REG_BLD_UNION32_MASK(reg_base_name, mask)               \
AMBA_REG_BLD_UNION32(reg_base_name);									 \
BCM_INLINE int reg_base_name##_is_equal(reg_base_name##_u_t lhs, reg_base_name##_u_t rhs) \
{																	 \
	return ( (lhs.w_data & (mask)) == (rhs.w_data & (mask) )); \
}




//--------------------------------------------------
/**
  Macro for reading a bitfield value in a 32bit register.  resulting
  bit value is returned in var

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#define AMBA_REG_RD_BF32(p_reg, regtype, fieldname, var)             \
{                                                                    \
    regtype _tReg;                                                   \
    _tReg.w_data = amba_read32(p_reg);                               \
    var = _tReg.bf.fieldname;                                        \
}

//--------------------------------------------------
/**
  Macro for writing a bitfield value in a 32bit register.  A read modify
  write is done here in order to preserve the other fields of the register.

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#define AMBA_REG_WR_BF32(p_reg, regtype, fieldname, value)           \
{                                                                    \
    regtype _tReg;                                                   \
    _tReg.w_data = amba_read32(p_reg);                               \
    _tReg.bf.fieldname = value;                                      \
    amba_write32(p_reg, _tReg.w_data);                               \
}


//--------------------------------------------------
/**
  Macro for building a union register definition using
  both the bitfield definition and underlying 16bit word type.

  @ingroup BCM_REG
*/
//-------------------------------------------------- 
#define AMBA_REG_BLD_UNION16(reg_base_name)                          \
typedef union                                                        \
{                                                                    \
    uint16_t         w_data;                                         \
    reg_base_name##_t bf;                                            \
} reg_base_name##u_t, *reg_base_name_##u_ptr_t


//--------------------------------------------------
/**
  Macro for reading a bitfield value in a 16bit register.  resulting
  bit value is returned in var

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#define AMBA_REG_RD_BF16(p_reg, regtype, regname, fieldname, var)    \
{                                                                    \
    regtype _tReg;                                                   \
    _tReg.w_data = amba_read16(p_reg);                               \
    var = _tReg.bf.fieldname;                                        \
}


//--------------------------------------------------
/**
  Macro for writing a bitfield value in a 16bit register.  A read modify
  write is done here in order to preserve the other fields of the register.

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#define AMBA_REG_WR_BF16( p_reg, regtype, fieldname, value)          \
{                                                                    \
    regtype _tReg;                                                   \
    _tReg.w_data = amba_read16( p_reg );                             \
    _tReg.bf.fieldname = value;                                      \
    AMBA_REG_WR16p_reg, _tReg.w_data);                               \
}



//--------------------------------------------------
/**
  Macro for building a union register definition using
  both the bitfield definition and underlying 8bit word type.

  @ingroup BCM_BASETYPES
*/
//-------------------------------------------------- 
#define AMBA_REG_BLD_UNION8( reg_base_name )                        \
typedef union                                                       \
{                                                                   \
    uint8_t         c_data;                                      \
    reg_base_name##_t bf;                                           \
} reg_base_name##_u_t, *reg_base_name##_u_ptr_t

//--------------------------------------------------
/**
  Macro for reading a bitfield value in a 8bit register.  resulting
  bit value is returned in var

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#define AMBA_REG_RD_BF8(p_reg, regtype, regname, fieldname, var)    \
{                                                                   \
    regtype _tReg;                                                  \
    _tReg.w_data = amba_read8(p_reg);                               \
    var = _tReg.bf.fieldname;                                       \
}

//--------------------------------------------------
/**
  Macro for writing a bitfield value in a 8bit register.  A read modify
  write is done here in order to preserve the other fields of the register.

  @ingroup BCM_BASETYPES
*/
//--------------------------------------------------
#define AMBA_REG_WR_BF8(p_reg, regtype, fieldname, value)           \
{                                                                   \
    regtype _tReg;                                                  \
    _tReg.w_data = amba_read8(p_reg);                               \
    _tReg.bf.fieldname = value;                                     \
    amba_write8(p_reg, _tReg.w_data);                               \
}


/***********************************************************
*
* Typedefs
*
***********************************************************/
/*
 * Basic register types.  These will be used for hardware registers that
 * don't have a bitwise definition.  They only have a scalar component to 
 * them
 */ 

/* 32bit register */
typedef struct 
{                                                                    
    uint32_t w_data;                                           
}bcm_reg32_t;

/* 16bit register */
typedef struct 
{                                                                    
    uint16_t s_data;                                           
} __attribute__((__packed__)) bcm_reg16_t ;

/* 8bit register */
typedef struct 
{                                                                    
    uint8_t  c_data;                                           
} __attribute__((__packed__)) bcm_reg8_t ;

#define amba_reg_read8(regbase, regname) amba_read8((volatile uint8_t *)&(*regbase).regname.c_data)   
#define amba_reg_write8(regbase, regname, regdata) amba_write8((volatile uint8_t *)&(*regbase).regname.c_data, regdata) 

/***********************************************************
*
* Structure definitions
*
***********************************************************/

/***********************************************************
*
* External declarations of global variables
*
***********************************************************/

/***********************************************************
*
* Function prototypes
*
***********************************************************/

#endif  /* _BCM_REG_H_  */

















