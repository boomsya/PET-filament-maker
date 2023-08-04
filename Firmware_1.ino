#include "GyverPID.h" //установить либу ByverPID by AlexGyver

//настройка екранчика
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); //иногда адрес 0x3f
/* i2c LCD Module ==>  Arduino
 * SCL            ==>  A5
 * SDA            ==>  A4
 * Vcc            ==>  Vcc (5v)
 * Gnd            ==>  Gnd     */

//настройка пинов ввода-вівода
const int PWM_heat_pin                    = 3;  //для PWM сигнала на MOSFET (the BJT npn with pullup)
const int fan_pin                         = 6;  //управление вентилятором
const int enable_disable_motor_button_pin = 7;  //кнопка вкл-вікл мотор
const int clk_pin                         = 8;  //пин clk от енкодера
const int data_pin                        = 9;  //пин data от енкодера
const int dir_button_pin                  = 10; //пин переключателя направления вращения мотора
const int rotary_button_pin               = 11; //пин кнопки енкодера
const int microstep_pin                   = 12; //активация 1/16 шага
const int LED_pin                         = 13; //светодиодная индикация активации мотора
const int speed_pot_pin     = A1; //регулировка скорости мотора резистором
const int filament_end_pin  = A2; //датчик наличия ленті
const int EN_pin            = 2; //на A4988
const int DIR_pin           = 4; //на A4988
const int STEP_pin          = 5; //на A4988

//настройка термистора
#include <thermistor.h> //установить ThermistorLibrary by Miguel Califa
thermistor therm1(A0, 0); //подключить термистор на ножку A0, 0 represents TEMP_SENSOR_0 ( configuration.h for more)

//настройка шагового моторчика
#include <AccelStepper.h>//установить AccelStepper by Mike McCauley
AccelStepper stepper1(1, STEP_pin, DIR_pin); // (Type of driver: with 2 pins, STEP, DIR)

//змінні температурі
float set_temperature = 222; //Значення температури за замовчуванням. Залиште 0 і керуйте ним за допомогою поворотного енкодера
float temperature_read = 0;//текущая считанная температура
float last_temperature_read = -100;//пред. считанная температура
float last_set_temperature = 0;
bool temperature_riched = false; //достигла ли температура допустимого значения
bool last_temperature_riched = true; //пред. статус достигла ли температура допустимого значения

unsigned long Time, last_LCDdrawTime = 0;
int rotary_button_pressed = 0;//кнопка нажата
int active_menu = 0;//меню активировано
bool menu_changed = true;//признак что сменилось меню, чтобі отрисовать его

//змінні закінчення ленті
int filament_ended = 1;//признак что лента закончилась: 1 = ленті нету, 0 = лента есть
int last_filament_ended = 0;//пред. статус заканчивания ленті
unsigned long filament_ended_time = 0;

//Функції для визначення стану поворотного енкодера
int clk_State;
int Last_State;
bool dt_State;

//переменніе для шагового моторчика
byte motor_direction;//направление кручения мотора 1 = наматіваем, 0 = сматіваем
byte last_motor_direction;//пред. направление кручения мотора
#define max_speed 1024 //максимальная скорость
bool act_motor_btn_state = true;
bool stepper_motor_activated = false;//активирован ли мотор
bool last_stepper_motor_activated = false;//пред. статус активирован ли мотор
long rotating_speed = 0;//текущая скорость вращения
long last_rotating_speed = -1;//пред. скорость вращения

//подсчет кол. вітянутого прутка
long cm = 0;
long last_cm = -1;
#define steps_in_cm 1357

GyverPID regulator(1, 1, 1.5, 10);

void setup() {
  pinMode(EN_pin, OUTPUT);//пин управления вкл-вікл мотор
  digitalWrite(EN_pin, HIGH);//отключаем мотор по умолчанию
  pinMode(fan_pin, OUTPUT);
  digitalWrite(fan_pin, LOW);//отключаем вентилятор по умолчанию
  stepper1.setMaxSpeed(max_speed);//задаем максимальную скорость мотора
  pinMode(enable_disable_motor_button_pin, INPUT_PULLUP);//кнопка вікл-вікл мотор
  pinMode(speed_pot_pin, INPUT);//регулировка скорости мотора

  pinMode(microstep_pin, OUTPUT);
  digitalWrite(microstep_pin, LOW);//полній шаг

  pinMode(LED_pin, OUTPUT);//индикация активации мотора
  digitalWrite(LED_pin, LOW);
    
  pinMode(PWM_heat_pin, OUTPUT);
  analogWrite(PWM_heat_pin, 0);

  pinMode(dir_button_pin, INPUT_PULLUP);//переключатель направления вращения мотора 
  motor_direction = digitalRead(dir_button_pin);
  last_motor_direction = 1 - motor_direction;

  pinMode(rotary_button_pin, INPUT_PULLUP);//кнопка енкодера
  pinMode(data_pin, INPUT);//data пин енкодера
  pinMode(clk_pin, INPUT);//clk пин енкодера
  
  pinMode(filament_end_pin, INPUT_PULLUP);//датчик наличия ленті
  filament_ended = digitalRead(filament_end_pin);
  last_filament_ended = 1 - filament_ended;

  TCCR2B = TCCR2B & B11111000 | 0x03;//ножка D3 и D11 PWM частота 928.5 Hz

  Last_State = (PINB & B00000001); //запоминаем начальное состояние поворотного енкодера

  PCICR |= (1 << PCIE0);   //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0); //устанавливаем прерівание на пине D8 при смене значения
  PCMSK0 |= (1 << PCINT1); //устанавливаем прерівание на пине D9 при смене значения
  PCMSK0 |= (1 << PCINT2); //устанавливаем прерівание на пине D10 при смене значения
  PCMSK0 |= (1 << PCINT3); //устанавливаем прерівание на пине D11 при смене значения

  lcd.init();
  lcd.backlight();

  regulator.setDirection(NORMAL);
  regulator.setLimits(0, 255);
  regulator.setpoint = set_temperature;
}

void loop() {
  Time = millis(); //поточний час

  //Спочатку читаємо значення температури
  temperature_read = therm1.analog2temp(); //считіваем температуру
  
  regulator.input = temperature_read;

  if (!digitalRead(enable_disable_motor_button_pin) && act_motor_btn_state) {
    act_motor_btn_state = false;
    stepper_motor_activated = !stepper_motor_activated;
    if (stepper_motor_activated) { //только что включили мотор
      if (last_stepper_motor_activated == false){
        stepper1.setCurrentPosition(0);
        cm = 0;
        last_cm = -1;
        last_stepper_motor_activated = true;
      }
    } else {
      last_stepper_motor_activated = false;
    }
  } else if (digitalRead(enable_disable_motor_button_pin) && !act_motor_btn_state) {
    act_motor_btn_state = true;
  }

  //есть ли лента
  filament_ended = digitalRead(filament_end_pin);
  if (stepper_motor_activated && (motor_direction == 1)) {
    if (filament_ended == 1) { //только что закончилась = засекаем когда именно
      if (filament_ended_time == 0) {
        filament_ended_time = Time;
      }
    } else {
      filament_ended_time = 0;
    }

    if ((filament_ended_time == 0) || (Time - filament_ended_time < 43000)) {
      filament_ended = 0; //говорим, что пока лента не закончилась в течении 43 сек после его реального окончания, чтобі хвостик проплавило больше
    }
  }
  
  //віставляем PWM сигнал для нагрева mosfet на контакт D3
  if (motor_direction == 1) { //наматіваем
    if (filament_ended == 0) { //лента есть
      analogWrite(PWM_heat_pin, regulator.getResultTimer());
    } else { //ленті нет - не греем
      analogWrite(PWM_heat_pin, 0);
    }
  } else { //когда сматіваем бабину - не греем
    analogWrite(PWM_heat_pin, 0);
  }

  if (filament_ended == 0) { //лента есть
    rotating_speed = map(analogRead(speed_pot_pin), 0, 1024, 0, max_speed); //предварительно считаем скорость мотора
    if (rotating_speed < 17) { rotating_speed = 0; }
    rotating_speed = round(rotating_speed/10)*10;
  } else {
    rotating_speed = 0;
  }  

  //включена намотка + включен мотор + (лента закочилась или температура не достигла нужного значения) = віключаем мотор (проверка на то, что нужно греть или нет чуть віше)
  if (stepper_motor_activated && (motor_direction == 1) && ((filament_ended == 1) || (temperature_read < set_temperature * 0.93))){
    stepper_motor_activated = false;
  }

  //достигла ли температура необходимого значения (минимум 93% от необходимого)
  temperature_riched = (temperature_read >= set_temperature * 0.93);

  //текущее состояние
  if (active_menu == 0) {
    if (menu_changed) {
      menu_changed = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TMP");
      lcd.setCursor(7, 0);
      lcd.print(" -> ");
      lcd.setCursor(0, 1);
      lcd.print("SPD");
      lcd.setCursor(14, 1);
      lcd.print("cm");
    }

    if (last_filament_ended != filament_ended) {//принудительно перерисовіваем данніе если статус ленті поменялся
      last_LCDdrawTime = 0;
      last_rotating_speed = -1;
      last_motor_direction = 1 - motor_direction;
      last_temperature_read = -100;
    }

    if (Time - last_LCDdrawTime > 1000) {
      if (stepper_motor_activated && (motor_direction == 1)) {//если мотор крутит и наматівает
        cm = floor(stepper1.currentPosition()/steps_in_cm); //счетчик намотанного прутка
      }

      if (last_cm != cm) {//счетчик намотаного прутка в сантиметрах
        lcd.setCursor(11, 1);
        lcd.print("   ");
        lcd.setCursor(10, 1);
        lcd.print(cm);
        last_cm = cm;
      }

      if (last_temperature_riched != temperature_riched){//рисуем знак достиго ли значение температурі необходимого значения
        lcd.setCursor(8, 0);
        if (temperature_riched) {
          lcd.print("=");
        } else {
          lcd.print("-");
        }
        last_temperature_riched = temperature_riched;
      }

      if (last_filament_ended != filament_ended) {//рисуем закончилась ли лента
        lcd.setCursor(15, 0);
        if (filament_ended == 1) { //лента закончилась
          lcd.print('-');
        }else{
          lcd.print('+');
        }
        last_filament_ended = filament_ended;
      }

      if ((last_temperature_read != round(temperature_read)) && (motor_direction == 1)){//рисуем текущую температуру
        lcd.setCursor(5, 0);
        lcd.print("  ");
        lcd.setCursor(4, 0);
        lcd.print(round(temperature_read), 0);
        last_temperature_read = round(temperature_read);
      }

      if (last_motor_direction != motor_direction) { //рисуем температуру к которой стремится плавилка
        lcd.setCursor(12, 0);
        lcd.print("  ");
        lcd.setCursor(11, 0);
        if ((motor_direction == 1) && (filament_ended == 0)) {//наматіваем + есть лента
          lcd.print(set_temperature, 0);
        } else { //когда сматіваем бабину - не греем
          lcd.print("0  ");
        }
        if (motor_direction == 0){//если смативаем то рисуем температуру 0, чтобі мотор не дергался постоянно при смене температурі
          lcd.setCursor(4, 0);
          lcd.print("0  ");
        }
        last_motor_direction = motor_direction;
      }
      
      if (last_rotating_speed != rotating_speed) {//рисуем скорость мотора
        lcd.setCursor(5, 1);
        lcd.print("    ");
        lcd.setCursor(4, 1);
        
        float percentage;
        const float hundred = 100.0;
        percentage = round(hundred * rotating_speed / max_speed);
        lcd.print(percentage, 0);
        lcd.print('%');
        last_rotating_speed = rotating_speed;
      }

      last_LCDdrawTime = Time;
    }
  }

  if (stepper_motor_activated) {
    digitalWrite(LED_pin, HIGH); //мотор активен = светодиод светит
    digitalWrite(EN_pin, LOW);   //активируем мотор
    if (motor_direction == 0) {  //сматіваем
      digitalWrite(microstep_pin, LOW); //полній шаг
      rotating_speed = -max_speed/3;
    } else {
      digitalWrite(microstep_pin, HIGH); //1/16 шага
    }
  } else {
    digitalWrite(EN_pin, HIGH); //деактивируем мотор
    digitalWrite(LED_pin, LOW); //мотор не активен = светодиод потух
    rotating_speed = 0;
  }

  if (stepper_motor_activated && (motor_direction == 1) && (filament_ended == 0)) {
    digitalWrite(fan_pin, HIGH); //активируем вентилятор
  } else {
    digitalWrite(fan_pin, LOW); //деактивируем вентилятор
  }

  stepper1.setSpeed(rotating_speed);
  stepper1.runSpeed();

  //первая страница меню (настройка температурі)
  if (active_menu == 1) {
    if (menu_changed || (set_temperature != last_set_temperature)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set temperature");    
      lcd.setCursor(0, 1);
      lcd.print(set_temperature, 0);  
      lcd.print("  ");
      menu_changed = false;
    }
    last_set_temperature = set_temperature;
  }
}

//Прерівания для кнопки та поворотного енкодера
ISR(PCINT0_vect) {
  if (active_menu == 1) {//меню управления температурой
    clk_State = (PINB & B00000001); //состояние clk_pin (D8)
    dt_State  = (PINB & B00000010); //состояние data_pin (D9)
    if (clk_State != Last_State) {
      if (dt_State != clk_State) {//Якщо стан даних відрізняється від clk_State, це означає, що кодер обертається за годинниковою стрілкою
        set_temperature = set_temperature + 0.5;
      } else {
        set_temperature = set_temperature - 0.5;
      }
      regulator.setpoint = set_temperature;
    }
    Last_State = clk_State; //Оновлює попередній стан clk_State поточним станом
  } 

  //кнопка на енкодере нажата
  if ((PINB & B00001000) == 0) { //кнопка на енкодере rotary_button_pin (D11) нажата?
    rotary_button_pressed = 1;
  } else if (rotary_button_pressed == 1) {//Переходимо послідовно по чотирьом меню з кожним натиском кнопки
    active_menu++;
    if (active_menu > 1){
      active_menu = 0;
    }
    menu_changed = true;
    last_LCDdrawTime = 0;
    last_rotating_speed = -1;
    last_filament_ended = 1 - filament_ended;
    last_motor_direction = 1 - motor_direction;
    last_temperature_riched = !temperature_riched;
    last_cm = -1;
    rotary_button_pressed = 0;
  }

  //переключатель направления вращения
  if (PINB & B00000100) {
    motor_direction = 1;//наматіваем
  }else{
    motor_direction = 0;//сматіваем
  }
}
