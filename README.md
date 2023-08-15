# PET filament maker for 3D printers

based on maker https://electronoobs.com/eng_arduino_tut174.php
but have some additional features like:
1. added potentiometer to speed regulation
2. forward/reverse direction of motor
3. changed PID regulator to other library
4. added sensor of filament ending
5. deleted PID parameters configuration (leave only temperature configuring)
6. added counting length of filament maked (cm)
7. forward motor steps = 1/16 and revers = 1/1
8. motor activated only when temperature is >= 94% of target
9. motor speed decreasing -3.3% every 6 meters of filament maked (my bobin inner diamenter is 10.5cm and every layer of filament is +3.5mm and this leads to increasing real speed of winding +3.3%)
10. and many others improvements based on actual device usage

At now firmware works at Arduino Nano (atmega168) and takes 95% of avaliable space.

![pet filament maker](https://github.com/boomsya/PET-filament-maker/blob/main/20230801_170722.jpg)

# Stand with Ukraine! -> [United24](https://u24.gov.ua/)
