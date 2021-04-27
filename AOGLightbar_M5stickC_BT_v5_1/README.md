# AOpenGPS(v5.1.3)
https://github.com/farmerbriantee/AgOpenGPS/releases

# Libraries

https://github.com/m5stack/M5Stack

https://github.com/FastLED/FastLED

# Parts

- M5stickC (with LCD ,Battery)

https://m5stack.com/products/stick-c

https://www.switch-science.com/catalog/6350/

- ATOM Lite (no LCD)

https://m5stack.com/products/atom-lite-esp32-development-kit

https://www.switch-science.com/catalog/6262/

- M5Stack Official RGB LEDs Cable SK6812 with GROVE Port 

https://aliexpress.com/item/32950831315.html

https://www.switch-science.com/catalog/5209/


- M5Stack Official Universal 4Pin Buckled Grove Cable 

https://aliexpress.com/item/32949298454.html

https://www.switch-science.com/catalog/5217/

# default setting
```

  #define NUMPIXELS   31                 // Odd number dont use =0 
  #define Neopixel_Pin 32                 //GPIO32:M5stickC  GPIO26:ATOMLite Set this to the pin number you are using for the Neopixel strip controll line
  #define cmPerLightbarPixel  2          // Must be a multiple of cmPerDistInt
  #define cmPerDistInt  2                // The number of centimeters represented by a change in 1 of the AOG cross track error byte
  uint8_t Neopixel_Brightness = 150;// default brightness value between 0 and 255
```

**AOG send distanceFromLine data only in Autosteer mode**

**Press the front button to switch to Neopixel Brightness change**


