/**
 * \file
 * Synaptics RMI over I2C Physical Layer Driver Header File.
 * Copyright (c) 2007-2009 Synaptics Incorporated
 *
 * This file is triple licensed under the GPL2, MPL, and Apache licenses.
 */
/*
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 *
 * Mozilla Public License
 *
 * The contents of this file are subject to the Mozilla Public License
 * Version 1.1 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS"
 * basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See the
 * License for the specific language governing rights and limitations
 * under the License.
 *
 * The Original Code is this file.
 *
 * The Initial Developer of the Original Code is Synaptics, Inc.  Portions
 * created by Synaptics, Inc. are Copyright (c) 2007-2008 Synaptics, Inc. All
 * Rights Reserved.
 *
 * Alternatively, the contents of this file may be used under the terms of the
 * GNU General Public License version 2 or Apache License version 2.0 (the
 * "Alternate License"), in which case the provisions of Alternate License are
 * applicable instead of those above. If you wish to allow use of your version
 * of this file only under the terms of one of the Alternate Licenses and not
 * to allow others to use your version of this file under the MPL, indicate
 * your decision by deleting the provisions above and replace them with the
 * notice and other provisions required by the Alternate License. If you do not
 * delete the provisions above, a recipient may use your version of this file
 * under either the MPL or one of the Alternate Licenses."
 *
 *#############################################################################
 *
 * Apache License
 *
 * Copyright (c) 2007-2008 Synaptics, Inc. Licensed under the Apache License,
 * Version 2.0 (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 *#############################################################################
 */

/** Platform-specific configuration data.
 * This structure is used by the platform-specific driver to designate
 * specific information about the hardware.  A platform client may supply
 * an array of these to the rmi_phys_i2c driver.
 */
struct rmi_i2c_clientdata {
	/** The seven-bit i2c address of the device. */
	int i2c_address;
	/** The number of the irq.  Set to zero if polling is required. */
	int irq;
	/** The type of the irq (e.g., IRQF_TRIGGER_FALLING).  Only valid if
	 * irq != 0 */
	int irq_type;
	/** Function used to query the state of the attention line.  It always
	 * returns 1 for "active" regardless of the polarity of the attention
	 * line. */
	int (*get_attention)(void);
};

/** Descriptor structure.
 * Describes the number of i2c devices on the bus that speak RMI.
 */

typedef struct {
	void (*init)(void);
	int (*get_attention)(void);
	int i2c_bus;
}  rmi_pdata_t;

#define SYNAPTIC_DEVNAME                     "tomtom-rmi-ctsic"

/* vim600: set noexpandtab sw=8 ts=8 :*/
