#ifndef TOF_H
#define TOF_H

#include <VL53L0X.h>
#include <global.h>

enum IO_STATE {ON, OFF};

class TOF
{
    public:
        TOF(int *p_pcr_reg, int *p_pddr_reg, int *p_pdor_reg, int p_pin_mask, int p_i2c_addr);
        
        void Init();
        // shutdown io refers to the state of the shutdown pin for the VL53L0X sensor
        // the pin is active low meaing a logic 0 turns the sensor
        void setShutdownIOState(IO_STATE p_state);
        void getShutdownIOState();

    private:
        void configureShutdownIO();

    private:
        VL53L0X m_vl530x;
        int *m_pcr_reg;
        int *m_pdder_reg;
        int *m_pdor_reg;
        int m_pin_mask;
        int m_i2c_addr;

        IO_STATE m_io_state;
};


#endif