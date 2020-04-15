// ----------------------------------------------------------------------------------------------------------------
// C++ FILE (.cpp)
// ----------------------------------------------------------------------------------------------------------------
#include "M_dlhrDiffPress_avrBoard.h"
AllSensors_DLHR_L60D_6 gagePressure(&Wire);

M_dlhrDiffPress_avrBoard::M_dlhrDiffPress_avrBoard(){     // Constructor
  Wire.begin();
}
// ---------------------------------------------------------------------
void M_dlhrDiffPress_avrBoard::dlhrBegin(){
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::PASCAL);
  
  gagePressure.startMeasurement();
  gagePressure.readData(true);

  //Serial.println("Caibration Start");
  int iLen = 100;
  float fPressTotal = 0;
  for (int i=0; i<iLen; i++){
    gagePressure.startMeasurement();
    gagePressure.readData(true);
    fPressTotal = fPressTotal + gagePressure.pressure;
  }
  _fPressOffset = fPressTotal/iLen;

  //Serial.println("Caibration End");
}
// ---------------------------------------------------------------------
float M_dlhrDiffPress_avrBoard::getDiffPressure(){
  
  gagePressure.startMeasurement();
  gagePressure.readData(true);

  _fDiffPress = gagePressure.pressure - _fPressOffset;

  return _fDiffPress;
}
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
