#include "stm32f10x.h"
#include "I2C_support.h"


//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void I2C_Init_fnc(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB2Periph_GPIOB, ENABLE);		
	
  GPIO_InitTypeDef 	GPIO_InitStructure;
	I2C_InitTypeDef   I2C_InitStructure;

	
  /* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  /* I2C1 configuration ------------------------------------------------------*/
  I2C_InitStructure.I2C_ClockSpeed = I2C_CLK_SPEED;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStructure);	
	
	I2C_Cmd(I2C1, ENABLE);	
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void i2c_start()
{
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop()
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void i2c_address_direction(uint8_t address, uint8_t direction)
{
    // Send slave address
    I2C_Send7bitAddress(I2C1, address, direction);

    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter)
    {
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if (direction == I2C_Direction_Receiver)
    { 
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void i2c_transmit(uint8_t byte)
{
    I2C_SendData(I2C1, byte);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
uint8_t i2c_receive_ack()
{
    // Enable ACK of received data
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    return I2C_ReceiveData(I2C1);
}

uint8_t i2c_receive_nack()
{
    // Disable ACK of received data
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    return I2C_ReceiveData(I2C1);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void i2c_write(uint8_t address, uint8_t data)
{
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Transmitter);
    i2c_transmit(data);
    i2c_stop();
}

void i2c_read(uint8_t address, uint8_t* data)
{
    i2c_start();
    i2c_address_direction(address << 1, I2C_Direction_Receiver);
    *data = i2c_receive_nack();
    i2c_stop();
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
uint8_t read8() {	
  return i2c_receive_ack();
}

uint16_t read16() {
  return read8() + (uint16_t)(read8() << 8);
}

uint32_t read32() {
  return (uint32_t) read16() + (uint32_t)(read16() << 16);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
void update_integral(i2c_integral_frame * iframe)
{
	// Does a read of the Px4Flow sensor. Gets data such as sonar distance, flow (x,y) since last read etc.
	i2c_write(PXFLOW_I2C_ADR, 0x16);
	
	i2c_start();
	i2c_address_direction(PXFLOW_I2C_ADR << 1, I2C_Direction_Receiver);

	// read the data
	iframe->frame_count_since_last_readout 	= read16();
	iframe->pixel_flow_x_integral  					= read16();
	iframe->pixel_flow_y_integral  					= read16();
	iframe->gyro_x_rate_integral   					= read16();
	iframe->gyro_y_rate_integral   					= read16();
	iframe->gyro_z_rate_integral   					= read16();
	iframe->integration_timespan   					= read32();
	iframe->sonar_timestamp        					= read32();
	iframe->ground_distance        					= read16();
	iframe->gyro_temperature       					= read16();
	iframe->quality                					= read8();	

	i2c_receive_nack();
	i2c_stop();		
}
