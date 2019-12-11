# Physical computing sensor servo tutorial - Colour finder and sorter with ams TCS34725 and HD-1900A

Identifying and sorting colours reliably via their Euclidean distance. Reading from a sensor and driving a servo with Arduino using millis() and an ISR. Say goodbye to blocking delay()

### Result

![](Assets/6a%20result.jpg)

### Schematic

![](Assets/6a%20schematic.png)

### Excel RGB value averaging or "colour training"

![](Assets/6a%20RGB%20value%20averaging.png)

### BOM

<pre>
€ 14,00 Adafruit Metro Mini 328 5V 16MHz microcontroller
€  8,00 Adafruit TCS34725 Proximity, Light, RGB, and Gesture Sensor
€  7,00 Pololu power HD Micro Servo HD-1900A
€  4,00 Half-size transparent breadboard
€  2,00 Breadboard mini modular black
€  1,00 Jumper cables
€  1,00 2,1mm DC barrel-jack
€  1,00 100µF 10V el. cap.
€ 10,00 MEANWELL GS12E05-P1I PSU
€  5,00 Opaque ultra matt black and coloured cardboard
€ 53,00
</pre>  

### Useful links  

μc https://www.adafruit.com/product/2590  
Sensor https://learn.adafruit.com/adafruit-color-sensors/overview  
Sensor library https://github.com/adafruit/Adafruit_TCS34725  
Sensor calibration https://learn.adafruit.com/calibrating-sensors/two-point-calibration  
Servo https://www.pololu.com/file/0J321/HD-1900A.pdf  
Servo library https://github.com/arduino-libraries/Servo  
