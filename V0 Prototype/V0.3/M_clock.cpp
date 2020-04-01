#include "M_clock.h"


M_clock::M_clock(){                                 // constructor
}
void M_clock::clockBegin(){                        // start the clock
  _fTimeBegin = millis()/1000.0;
}
void M_clock::clockReset(){
   _fTimeBegin = millis()/1000.0;
  _fTimeOld = 0;
}
float M_clock::clockTime(){
  _fTime = millis()/1000.0 - _fTimeBegin;
  return _fTime;
}
float M_clock::clockFreq(){
  _fTime = millis()/1000.0 - _fTimeBegin;
  _fFreq = 1.0/(_fTime-_fTimeOld);
  _fTimeOld = _fTime;
  return _fFreq;
}
