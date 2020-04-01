// ----------------------------------------------------------------------------------------------------------------
// C++ FILE (.cpp)
// ----------------------------------------------------------------------------------------------------------------
#include "M_i2c_multiplexer.h"

M_i2c_multiplexer::M_i2c_multiplexer(uint8_t addr){     // Constructor
  Wire.begin();

  _addr = addr;
}
// ---------------------------------------------------------------------
void M_i2c_multiplexer::tca_select(uint8_t i){
  if (i > 7) return;
 
  Wire.beginTransmission(_addr);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
// ---------------------------------------------------------------------
/*
void M_i2c_multiplexer::check_channels{
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
    
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
         Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }
  Serial.println("\ndone");
}
*/
// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
