// ----------------------------------------------------------------------------------------------------------------
// HEADER FILE (.h)
// ----------------------------------------------------------------------------------------------------------------

#ifndef M_i2c_multiplexer_h                         // To prevent the main program of calling this header (*.h) several times one cas to put in the following two lines before the code and the #endif statement at the end. 
#define M_i2c_multiplexer_h                         // Note the different syntax for the #ifndef (if not defined)

#include <Arduino.h>
#include <Wire.h>
extern "C"{ 
  #include "utility/twi.h"  // from Wire library, so we can do bus scanning
}


class M_i2c_multiplexer{                                                         // Define the class type object named Ard33WiFi
  public:                                                                        // Specify the functions and variables below here as public
    M_i2c_multiplexer(uint8_t addr);                                             // Constructor

    void tca_select(uint8_t i);

  private:                                                                        // Specify the functions and variables below here as private to the Ard33WiFi class
    uint8_t _addr;
};
#endif
