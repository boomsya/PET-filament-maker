# Bottle PET filament maker for 3D printers

based on maker https://electronoobs.com/eng_arduino_tut174.php (really think it is not works)
but have some additional features like:
1. forward/reverse direction of motor
2. changed PID regulator to other library
3. added sensor of filament ending
4. deleted PID parameters configuration (leave only temperature configuring)
5. added counting length of filament maked (cm)
6. forward motor steps = 1/16 and revers = 1/1
7. motor activated only when temperature is >= 94% of target
8. and many others improvements based on actual device usage
9. speed controlling with rotary
10. rotary encoder more and more stable controlling when rotating
11. emergency stop motor when heater temperature more than 255c and if heater temperature more than 255 - heating is disabled
12. added filter to temperature sensor - less noise in measurement
13. much better PID parameters for 40W 12V heater. Setpoint = 222C and temperature bouncing in range 220...227C
14. I builded second machine and mechanically some different. So have some code modifications (```#define MACHINE2```)

At now firmware works at Arduino Nano (atmega168) and takes 99% of avaliable space. 
Used cheap PSU 250W.

![pet filament maker](https://github.com/boomsya/PET-filament-maker/blob/main/maker.jpg)

![pet filament maker](https://github.com/boomsya/PET-filament-maker/blob/main/20230823_114803.jpg)

Result:
![pet filament maker](https://github.com/boomsya/PET-filament-maker/blob/main/DSC_0013.JPG)

# Stand with Ukraine! -> [United24](https://u24.gov.ua/)
