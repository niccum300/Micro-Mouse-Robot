// Authors: Brayton Niccum
// Created: 1/26/2022
// Updated: 

#include <TOF.h>

TOF::TOF(int *p_pcr_reg, int *p_pddr_reg, int *p_pdor_reg, int p_pin_mask, int p_i2c_addr)
    :m_pcr_reg(p_pcr_reg), m_pdder_reg(p_pddr_reg), m_pdor_reg(p_pdor_reg), 
    m_pin_mask(p_pin_mask), m_i2c_addr(p_i2c_addr)
{
}

void TOF::init()
{


}

void TOF::configureShutdownIO()
{
    // set as io pin
    *m_pcr_reg = PORT_PCR_MUX(IO_MUX_MASK);
    // configure as output 
    *m_pdder_reg |= m_pin_mask;

    setShutdownIOState(OFF);
}

void TOF::setShutdownIOState(IO_STATE p_state){
	if(p_state == ON)
	{
		*m_pdor_reg |= m_pin_mask;
		m_io_state = ON;
	}else if(p_state == OFF){
		*m_pdor_reg |= ~m_pin_mask;
		m_io_state = OFF;	
	}
}