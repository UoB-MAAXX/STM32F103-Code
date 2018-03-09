#ifndef _I2C_SUPPORT_H_
#define _I2C_SUPPORT_H_

#define 						I2C_CLK_SPEED          						200000
#define 						PXFLOW_I2C_ADR     								0x42			// 100_0010

typedef struct i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;									//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;														//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;														//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;															//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;															//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;															//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;														//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;																	// time since last sonar update [microseconds]
    int16_t ground_distance;																	// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;																	// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;																					// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} i2c_integral_frame;



void I2C_Init_fnc(void);

void i2c_start(void);
void i2c_stop(void);

void i2c_address_direction(uint8_t address, uint8_t direction);
void i2c_transmit(uint8_t byte);

uint8_t i2c_receive_ack(void);
uint8_t i2c_receive_nack(void);

void i2c_write(uint8_t address, uint8_t data);
void i2c_read(uint8_t address, uint8_t* data);

uint8_t read8(void);
uint16_t read16(void);
uint32_t read32(void);

void update_integral(i2c_integral_frame * iframe);


#endif
