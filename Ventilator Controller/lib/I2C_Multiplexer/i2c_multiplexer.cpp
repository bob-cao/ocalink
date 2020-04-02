// ----------------------------------------------------------------------------------------------------------------
// C++ FILE (.cpp)
// ----------------------------------------------------------------------------------------------------------------
#include "i2c_multiplexer.h"

I2cMultiplexer::I2cMultiplexer(uint8_t addr){     // Constructor
  Wire.begin();

  _addr = addr;
}
// ---------------------------------------------------------------------
void I2cMultiplexer::tca_select(uint8_t i){
  if (i > 7) return;
 
  Wire.beginTransmission(_addr);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
