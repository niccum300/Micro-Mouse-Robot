// Authors: Brayton Niccum
// Created: 1/26/2022
// Updated: 

#include <TOF.h>

// public methods
TOF::TOF(volatile uint32_t *p_pcr_reg, volatile uint32_t *p_pddr_reg, volatile uint32_t *p_pdor_reg, int p_pin_mask, int p_i2c_addr, SensorQueue * p_sensor_q)
    :m_pcr_reg(p_pcr_reg), m_pdder_reg(p_pddr_reg), m_pdor_reg(p_pdor_reg), 
    m_pin_mask(p_pin_mask), m_i2c_addr(p_i2c_addr), m_sensor_q(p_sensor_q)
{
}

void TOF::Init()
{
	if (!m_vl530x.init())
 	{
    while (1){Serial.printf("Failed to detect and initalze sensor on pin %d! \n", m_pin_mask);}
  	}else {
		  Serial.printf("initalzed sensor on pin %d! \n", m_pin_mask);
	  }
	m_vl530x.setAddress(m_i2c_addr);

	m_vl530x.setMeasurementTimingBudget(HIGH_SPEED_MODE);

}

void TOF::SetShutdownIOState(IO_STATE p_state){
	if(p_state == ON)
	{
		*m_pdor_reg |= m_pin_mask;
		m_io_state = ON;
	}else if(p_state == OFF){
		*m_pdor_reg |= ~m_pin_mask;
		m_io_state = OFF;	
	}
}

void TOF::Update(){
	getRawData();
	filterRawData();

	m_sensor_q->Push(m_data);
}

SENSOR_DATA TOF::GetData()
{
	Update();
	return m_data;
}

// private methods
void TOF::configureShutdownIO()
{
    // set as io pin
    *m_pcr_reg = PORT_PCR_MUX(IO_MUX_MASK);
    // configure as output 
    *m_pdder_reg |= m_pin_mask;

	SetShutdownIOState(OFF);
}

void TOF::filterRawData()
{
	if(m_data.average == 0.00)
	{
		m_data.average = m_data.new_data_point;
		m_data.total_data_points = 1;
	}
	// if its outside devation reset sensor in terms of average
	else if(abs(((m_data.new_data_point - m_data.average) / m_data.average)) > SENSOR_DEVIATION)
	{
		m_data.average = m_data.new_data_point;
		m_data.total_data_points = 1;
	}
	else{
		m_data.total_data_points += 1;
		m_data.average = m_data.average + 
			((m_data.new_data_point - m_data.average)/m_data.total_data_points);
	}

}

void TOF::getRawData()
{
	m_data.new_data_point = m_vl530x.readRangeSingleMillimeters()/MILIMETERS_INCHES;
}
