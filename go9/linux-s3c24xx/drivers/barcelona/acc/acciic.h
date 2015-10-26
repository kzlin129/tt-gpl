/* drivers/barcelona/acc/acciic.h
 *
 * Implementation of the accelerometer IIC driver.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

void ACCIIC_Start(void);
void ACCIIC_Stop(void);
int ACCIIC_Transmit(unsigned char data);
unsigned char ACCIIC_Receive(int generateAck);
