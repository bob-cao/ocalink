#include "includes.h"

void chase(byte primary, byte secondary, double cycleDelay)
{
  for(uint16_t i = 0; i < AlarmLED.numPixels() + 4; i++)
  {
    AlarmLED.setPixelColor(i  , primary);     // Draw new pixel
    AlarmLED.setPixelColor(i - 8, secondary); // Erase pixel a few steps back
    AlarmLED.show();
    delay(cycleDelay);
  }
}
