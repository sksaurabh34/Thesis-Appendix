#include "MS5611_newlib.h"

int MS5611_newlib::write(uint8_t _cmd) {
        wire->beginTransmission(dev_address);
        wire->write(_cmd);
        uint8_t _result = wire->endTransmission();

        if(_result !=0) {
            Serial.print("COM Failed, res = ");
			Serial.println(_result);
        }

        return _result;
}

// Constructor =>> Sets dev address
MS5611_newlib::MS5611_newlib(const uint8_t _add) {
        dev_address = _add;
}

// Begin
// Check for communication
int MS5611_newlib::begin(TwoWire *_wire) {
        wire = _wire;

        // check if connected
        wire->begin();
        wire->beginTransmission(dev_address);
        int ret = wire->endTransmission(); // ==0 if dev present

        // reset too
        if(ret==0) {
            reset();
        }

        return ret; // return value same as wire.endtransmission()
}

// Read from PROM and populate 8 PROM register array C
uint16_t MS5611_newlib::readPROM(uint8_t reg_add) {
        write(reg_add);

        wire->requestFrom( dev_address, uint8_t(2));
        uint16_t val = wire->read() * 256;
        val += wire->read();

        return val;
}

//----------------------------------------------------------------//
// Reset and read PROM
int MS5611_newlib::reset() {
    Serial.println("MS5611 Reset");
    uint8_t res = write(RESET_REG) ;

    delay(5);
    
    // read C
    Serial.println("Reading PROM...");
    
    for(int i=0; i<8; i++) {
        C[i] = readPROM(PROM_ROOT_REG + 2*i);
        delay(1);
    }

    for(int i=0; i<8; i++) {
        Serial.print("C["); 
		Serial.print(i); 
		Serial.print("] = ");
		Serial.println(C[i]);
    }

    return res;
}

//----------------------------------------------------------------//
// convert & read ADC
uint32_t MS5611_newlib::convert_and_read_ADC(const uint8_t convert_reg_address) {
  write(convert_reg_address);

  delay(10);
  
  write(0);
  wire->requestFrom(dev_address, uint8_t(3) );
  uint32_t _temp = wire->read() * 65536UL;
  _temp += wire->read() * 256UL;
  _temp += wire->read();

  return _temp;
}

//----------------------------------------------------------------//
/* convert D2 -> temperature in degree C
dT = Difference between actual and reference temperature
dT = D2 - TREF = D2 - C5 * 2^8 [signed int32, 25 

TEMP = Actual temperature (-40…85°C with 0.01°C resolution)
TEMP = 20°C + dT*TEMPSENS = 2000 + dT * C6 / 2^23 */

real_t MS5611_newlib::D2_to_temp(uint32_t _temp) {
    dT = _temp - C[5]*pow(2, 8.0);
    real_t temp_f = 20.0 + dT*C[6]*pow(2.0, -23)*0.01;

    // todo low temp compensation

    return temp_f;
}

real_t MS5611_newlib::measure_temp(const OSR_t osr) {
    uint32_t D2 = convert_and_read_ADC(osr+0x10);
    
    //Serial.println(D2);
    return D2_to_temp(D2) ;
}

//----------------------------------------------------------------//
/* D1 -> pressure in mbar
OFF  = Offset at actual temperature
OFF  = OFFT1 +TCO* dT = C2 * 2^16 + (C4 * dT )/ 2^7 [signed int 64, 41 bits]

SENS = Sensitivity at actual temperature [4]
SENS = SENS_T1 + TCS*dT= C1*2^15 + (C3*dT)/2^8 [signed int 64, 41 bits] 

P = Temperature compensated pressure (10…1200mbar with)
P = D1 * SENS - OFF = (D1*SENS/2^21 - OFF)/2^15
*/

real_t MS5611_newlib::D1_to_press(uint32_t _d1, bool measure_dT) {
    if(dT==-1 || measure_dT) {
      Serial.println("dT not known - measuring temperature first");
      real_t temp = this->measure_temp();
      Serial.print("Measured T = ");
      Serial.print(temp);
      Serial.print(" dT = ");
      Serial.println(this->dT);
    }

    real_t OFF = C[2]*pow(2.0, 16.0) + (C[4]*this->dT)/pow(2.0, 7.0);
    real_t SENS = C[1]*pow(2.0, 15.0) + (C[3]*this->dT)/pow(2.0, 8.0);
    real_t P = (_d1*SENS/pow(2.0, 21.0) - OFF)*pow(2.0, -15.0)*0.01;

    return P;

}

real_t MS5611_newlib::measure_press(const OSR_t osr, bool measure_dT) {
    uint32_t D1 = convert_and_read_ADC(osr);
    return D1_to_press(D1, measure_dT);
}
//----------------------------------------------------------------//