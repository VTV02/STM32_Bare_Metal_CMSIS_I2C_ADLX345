/*
 * i2c.h
 *
 *  Created on: Jun 18, 2024
 *      Author: VOVAN
 */

#ifndef I2C_H_
#define I2C_H_
void I2C1_byteRead(char saddr, char maddress, char* data);
void I2C_Init(void);
void I2C1_burstRead(char saddr, char maddr,int n, char* data);
void I2C1_burstWrite(char saddr, char maddr,int n, char* data);


#endif /* I2C_H_ */
