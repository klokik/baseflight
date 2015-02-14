#include "board.h"

#define DRONO_ADDRESS 0x35

#define REG_VER				0x00
#define REG_READY			0x01
#define REG_LED				0x02
#define REG_I2C_ADDR		0x03
#define REG_MOTOR_UPD_RATE	0x0A
#define REG_MOTOR_NUM		0x0B
#define REG_MOTOR_VAL0		0x10
#define REG_RC_AVAIL		0x0C
#define REG_RC_NUM			0x0D
#define REG_RC_VAL0			0x20
#define REG_SONAR_AVAIL		0x0E
#define REG_SONAR_VAL0		0x30
#define REG_BATT_AVAIL		0x0F
#define REG_BATT_CELL_NUM	0x32
#define REG_BATT_LOW		0x33
#define REG_BATT_VAL0		0x34
#define REG_CMD_CALIBRATE	0x3C
#define REG_CMD_RESET		0x3D
#define REG_GOD_MODE		0x3E
#define REG_SEED			0x3F
#define REG_KEY				0x40
#define REG_SYS_ADDR0		0x41
#define REG_SYS_VAL0		0x43
#define REG_USART_ENABLE	0x4A
#define REG_USART_BR_VAL0	0x4B

uint16_t i2cReadRawRC(uint8_t chan)
{
	uint8_t buf[2];

	for(int q=0;q<4;q++)
		if(i2cRead(DRONO_ADDRESS,REG_RC_VAL0+chan*2,2,&buf[0]))
			break;

	uint16_t val = 1000+buf[0]*256+buf[1];
	return val;
	//return 1337;//val;
}

void i2cWriteMotor(uint8_t index, uint16_t value)
{
	uint8_t buf[] = {(value-1000)/256,(value-1000)%256};

    if (index < 4)
        i2cWriteBuffer(DRONO_ADDRESS,REG_MOTOR_VAL0+index*2,2,&buf[0]);
}

int32_t i2cSonarGetDistance(void)
{
	return -1;
	uint8_t buf[2];

	if(i2cRead(DRONO_ADDRESS,REG_SONAR_AVAIL,1,&buf[0]))
	{
		i2cRead(DRONO_ADDRESS,REG_MOTOR_VAL0,2,&buf[0]);

		return buf[0]*256+buf[1];
	}

	return -1;
}
// bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data);
// bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
// bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
