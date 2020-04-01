// ----------------------------------------------------------------------------------------------------------------
// HEADER FILE (.h)
// ----------------------------------------------------------------------------------------------------------------

#ifndef M_dlhrDiffPress_avrBoard_h                         // To prevent the main program of calling this header (*.h) several times one cas to put in the following two lines before the code and the #endif statement at the end. 
#define M_dlhrDiffPress_avrBoard_h                         // Note the different syntax for the #ifndef (if not defined)

#include <Arduino.h>
#include <Wire.h>
#include <AllSensors_DLHR.h>


class M_dlhrDiffPress_avrBoard{                                                         // Define the class type object named Ard33WiFi
  public:                                                                               // Specify the functions and variables below here as public
    M_dlhrDiffPress_avrBoard();                                                         // Constructor

    void dlhrBegin();
    float getDiffPressure();

  private:                                                                          // Specify the functions and variables below here as private to the Ard33WiFi class
    float _fPressOffset;
    float _fDiffPress;
    
};
#endif
