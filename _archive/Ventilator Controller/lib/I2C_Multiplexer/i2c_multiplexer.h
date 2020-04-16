// ----------------------------------------------------------------------------------------------------------------
// HEADER FILE (.h)
// ----------------------------------------------------------------------------------------------------------------

#ifndef I2cMultiplexer_h                         // To prevent the main program of calling this header (*.h) several times one cas to put in the following two lines before the code and the #endif statement at the end. 
#define I2cMultiplexer_h                         // Note the different syntax for the #ifndef (if not defined)

#include <Arduino.h>
#include <Wire.h>

class I2cMultiplexer{                                                         // Define the class type object named Ard33WiFi
  public:                                                                        // Specify the functions and variables below here as public
    I2cMultiplexer(uint8_t addr);                                             // Constructor

    void tca_select(uint8_t i);

  private:                                                                        // Specify the functions and variables below here as private to the Ard33WiFi class
    uint8_t _addr;
};

#endif
