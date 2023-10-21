# Bottle PET filament maker for 3D printers

based on maker https://electronoobs.com/eng_arduino_tut174.php (really think it is not works)
but have some additional features like:
1. added potentiometer to speed regulation
2. forward/reverse direction of motor
3. changed PID regulator to other library
4. added sensor of filament ending
5. deleted PID parameters configuration (leave only temperature configuring)
6. added counting length of filament maked (cm)
7. forward motor steps = 1/16 and revers = 1/1
8. motor activated only when temperature is >= 94% of target
9. and many others improvements based on actual device usage
10. v1.8: remove potentiometer (now speed controlling with rotary)
11. v1.8: rotary encoder more and more stable controlling when rotating
12. v1.8: emergency stop motor when heater temperature more than 255c and if heater temperature more than 255 - heating is disabled
13. v1.9: added filter to temperature sensor - less noise in measurement
14. v1.9: much much better PID parameters for 40W 12V heater. Setpoint = 222C and temperature bouncing in range 220...227C
15. v1.10: limit of scace program :( some code deleted
16. v1.10: fix percentage display
17. v.1.11: modify tempertaure filter to Median algorythm

At now firmware works at Arduino Nano (atmega168) and takes 99% of avaliable space. 
Used cheap PSU 250W.

![pet filament maker](https://github.com/boomsya/PET-filament-maker/blob/main/20230801_170722.jpg)

![pet filament maker](https://github.com/boomsya/PET-filament-maker/blob/main/20230823_114803.jpg)

# Stand with Ukraine! -> [United24](https://u24.gov.ua/)
