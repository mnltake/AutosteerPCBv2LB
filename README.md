# AutosteerPCBv2LB
forked from farmerbriantee/AgOpenGPS

WS2812B(Neopixel)　strips

use Adafruit NeoPixel Library

https://github.com/adafruit/Adafruit_NeoPixel

Neopixel  -- Arduino

VCC      -- 5V

DI       -- PIN9 (PCBv2 PWM2)

GND     --GND

**default setting**
```
  #define NUMPIXELS   13

  #define Neopixel_Pin 9
  
  #define cmPerLightbarPixel  4
```

**AOG send distanceFromLine data only in Autosteer mode**
