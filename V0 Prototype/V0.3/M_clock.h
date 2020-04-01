// Set a clock from which one can get:
//    - the lapsed time from set clock. 
//    - frequency of clock
//    - reset clock to zero
// https://www.youtube.com/watch?v=S_uaROFnWSg&t=552s
// ----------------------------------------------------------------------------------------------------------------

#ifndef M_clock_h                         // To prevent the main program of calling this header (*.h) several times one cas to put in the following two lines before the code and the #endif statement at the end. 
#define M_clock_h                         // Note the different syntax for the #ifndef (if not defined)

#include "Arduino.h"

class M_clock{                                  // Define the class type object named clock
  public:                                   // Specify the functions and variables below here as public
    M_clock();                                // Constructor. One can input a value into the constructor(and thus into the class). this is sothat one can create several objects (say for several LED's) by passing different pin numbers int othe object. 
    void clockBegin();
    void clockReset();
    float clockTime();
    float clockFreq();
  private:                                  // Specify the functions and variables below here as private to the Time class
    float _fTime;                             // Add underscore to define variable as a private member/variable
    float _fTimeOld;
    float _fFreq;
    float _fTimeBegin;
};

#endif
