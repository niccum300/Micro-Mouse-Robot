#ifndef TOF_H
#define TOF_H

#include <VL53L0X.h>
#include <global.h>
#include <SENSOR_DATA.h>

#define SCL0 (19)
#define SDA0 (20)

#define HIGH_ACCURACY_MODE (200000) //value in microseconds
#define MEDIUM_ACCURACY_MODE (200000) //value in microseconds
#define DEFAULT_MODE (33000)        //value in microseconds
#define HIGH_SPEED_MODE (20000)     //value in microseconds

#define SENSOR_DEVIATION  (0.07)

#define MILIMETERS_INCHES (25.4001)


enum IO_STATE {ON, OFF};

class TOF
{
    public:
        TOF(volatile uint32_t * p_pcr_reg, volatile uint32_t * p_pddr_reg, volatile uint32_t * p_pdor_reg, 
        int p_pin_mask, int p_i2c_addr, SENSOR_LOCATION p_sensor_id);
        
        void Init();
        void SetShutdownIOState(IO_STATE p_state);
        void GetShutdownIOState();
        void Update();
        SENSOR_DATA GetData();

    private:
        void configureShutdownIO();
        void filterRawData();
        void getRawData();

    private:
        VL53L0X m_vl530x;
        volatile uint32_t *m_pcr_reg;
        volatile uint32_t *m_pdder_reg;
        volatile uint32_t *m_pdor_reg;
        int m_pin_mask;
        int m_i2c_addr;
        SENSOR_LOCATION m_sensor_id;
        SENSOR_DATA m_data;

        IO_STATE m_io_state;
};


#endif