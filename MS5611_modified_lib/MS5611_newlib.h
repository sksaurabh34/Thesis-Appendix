// Copyright (C) 2021  Parswa Nath, Graduate student at TIFR Hyderabad, India
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301, USA

#include <Arduino.h>
#include <Wire.h>

#define real_t float

/*
Convert D1 (OSR=256) 0x40
Convert D1 (OSR=512) 0x42
Convert D1 (OSR=1024) 0x44
Convert D1 (OSR=2048) 0x46
Convert D1 (OSR=4096) 0x48

Convert D2 (OSR=256) 0x50
Convert D2 (OSR=512) 0x52
Convert D2 (OSR=1024) 0x54
Convert D2 (OSR=2048) 0x56
Convert D2 (OSR=4096) 0x58
ADC Read 0x00
*/

// For D1 measurement, 
// To measure D2, offset every address by 0x10
enum OSR_t {
    OSR_256=0x40, 
    OSR_512=0x42,
    OSR_1024=0x44,
    OSR_2048=0x46,
    OSR_4096=0x48
};

#define RESET_REG 0x1E
#define PROM_ROOT_REG 0xA0

class MS5611_newlib {
public:
    // constructor
    MS5611_newlib(const uint8_t _add);
    
    // Returns 0 if the device reponds
    int begin(TwoWire *_wire=&Wire);

    // Reset the device
    // Then read PROM and populate the Calibration coeff array C
    int reset();

    // Initiate conversion -> read D2 -> convert D2 to temperature
    // TODO low temp compensation
    real_t measure_temp(const OSR_t osr = OSR_4096); 

    // Initiate conversion -> read D1 -> convert D1 to predsure 
    // This requires dT which is calculated in measure_temp
    // If it is not available or if the user wants to force measure it
    // then keep the second argument measure_dT = true, default is false
    real_t measure_press(const OSR_t osr = OSR_4096, bool measure_dT=false);


private:
    TwoWire *wire;
    uint8_t dev_address;
    uint16_t C[8];
    real_t dT = -1; // cache dT for pressure measurement

    int write(uint8_t);
    uint16_t readPROM(uint8_t);
    uint32_t convert_and_read_ADC(const uint8_t);

    /* D2 -> temperatuen in deg C
    dT = Difference between actual and reference temperature
    dT = D2 - TREF = D2 - C5 * 2^8 [signed int32, 25 
    
    TEMP = Actual temperature (-40…85°C with 0.01°C resolution)
    TEMP = 20°C + dT*TEMPSENS = 2000 + dT * C6 / 2^23
    */
    real_t D2_to_temp(uint32_t);

    /* D1 -> pressure in mbar
    OFF  = Offset at actual temperature
    OFF  = OFFT1 +TCO* dT = C2 * 2^16 + (C4 * dT )/ 2^7 [signed int 64, 41 bits]

    SENS = Sensitivity at actual temperature [4]
    SENS = SENS_T1 + TCS*dT= C1*2^15 + (C3*dT)/2^8 [signed int 64, 41 bits] 

    P = Temperature compensated pressure (10…1200mbar with)
    P = D1 * SENS - OFF = (D1*SENS/2^21 - OFF)/2^15
    */
    real_t D1_to_press(uint32_t, bool measure_dT=false);

};