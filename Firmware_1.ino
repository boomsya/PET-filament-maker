//#define MACHINE2 //плавілка 2 - та що з підсвіткою колеса намотки

#include <Bounce2.h>

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
#include "GyverPID.h" //установить либу ByverPID by AlexGyver
GyverPID regulator(1, 256, 1.4, 10);
#include "GyverFilters.h" //установить лібу GyverFilters by AlexGyver
GMedian<10, int> filtered_temperature; //фильтр
byte set_temperature = 222; //Значення температури за замовчуванням. Керуйте ним за допомогою поворотного енкодера
float temperature_read = 0;//текущая считанная температура
float last_temperature_read = -100;//пред. считанная температура
byte last_set_temperature = 0;
bool temperature_riched = false; //достигла ли температура допустимого значения
bool last_temperature_riched = true; //пред. статус достигла ли температура допустимого значения

unsigned long Time, last_LCDdrawTime = 0;
byte rotary_button_pressed = 0;//кнопка натиснута
byte active_menu = 0;//меню активовано
bool menu_changed = true;//признак что сменилось меню, чтобі отрисовать его

//змінні закінчення стрічки
int filament_ended = 1;//признак что лента закончилась: 1 = ленті нету, 0 = лента есть
int last_filament_ended = 0;//пред. статус заканчивания ленті
unsigned long filament_ended_time = 0;

//Функції для визначення стану поворотного енкодера
int clk_State;
int dt_State;
static uint8_t prevNextCode = 0;
static uint16_t store = 0;

//змінні для крокового мотора
byte motor_direction;//направление кручения мотора 1 = наматіваем, 0 = сматіваем
byte last_motor_direction;//пред. направление кручения мотора

#define max_speed 1000.0 //максимальна швидкість

const float speeds_percent_arr[7] = {0.0, 6.0, 8.0, 10.0, 16.0, 24.0, 48.0};

Bounce2::Button start_stop_button = Bounce2::Button();
bool stepper_motor_activated = false;//чи активовано мотор
bool last_stepper_motor_activated = false;//пред. статус активирован ли мотор
float rotating_speed = 0;//поточна швидкість обертання
float last_rotating_speed = 0;//пред. скорость вращения
byte current_speed_idx = 3;//текущая скорость вращения по умолчанию

//подсчет кол. вітянутого прутка
long cm = 0;
long last_cm = -1;
#ifndef MACHINE2
  #define steps_in_cm 1357
#endif
#ifdef MACHINE2
  #define steps_in_cm 629
#endif

void setup() {
  pinMode(EN_pin, OUTPUT);//пин управления вкл-вікл мотор
  digitalWrite(EN_pin, HIGH);//отключаем мотор по умолчанию
  pinMode(fan_pin, OUTPUT);
  digitalWrite(fan_pin, LOW);//отключаем вентилятор по умолчанию
  stepper1.setMaxSpeed(max_speed);//задаем максимальную скорость мотора
  //pinMode(enable_disable_motor_button_pin, INPUT_PULLUP);//кнопка вікл-вікл мотор
  start_stop_button.attach(enable_disable_motor_button_pin, INPUT_PULLUP);
  start_stop_button.interval(50);
  start_stop_button.setPressedState(LOW);
  
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

  clk_State = (PINB & B00000001); //запоминаем начальное состояние поворотного енкодера
  dt_State  = (PINB & B00000010); //запоминаем начальное состояние поворотного енкодера

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
  //Спочатку читаємо значення температури
  temperature_read = filtered_temperature.filtered(therm1.analog2temp()); //считіваем температуру

  //есть ли лента
  filament_ended = digitalRead(filament_end_pin);

  Time = millis(); //поточний час

  start_stop_button.update();

  if (start_stop_button.pressed()) {//кнопка нажата
    stepper_motor_activated = !stepper_motor_activated;
    if (stepper_motor_activated) { //только что включили мотор
      if ((filament_ended == 1) && (motor_direction == 1)) { 
        stepper_motor_activated = 0; //если нет прутка - не стартуем
      } 
      if ((stepper_motor_activated == true) && (last_stepper_motor_activated == false)){ //только что включили мотор
        stepper1.setCurrentPosition(0);
        cm = 0;
        last_cm = -1;
        last_stepper_motor_activated = true;
      }
    } else { //только что віключили мотор
      last_stepper_motor_activated = false;
    }
  }

  //откладіваем заканчивание ленті
  if (stepper_motor_activated && (motor_direction == 1)) {
    if (filament_ended == 1) { //только что закончилась = засекаем когда именно
      if (filament_ended_time == 0) {
        filament_ended_time = Time;
      }
    } else {
      filament_ended_time = 0;
    }

    #ifndef MACHINE2
    if ((filament_ended_time == 0) || (Time - filament_ended_time < 44000)) {
      filament_ended = 0; //говорим, что пока лента не закончилась в течении 40 сек после его реального окончания, чтобі хвостик проплавило больше
    }
    #endif
    #ifdef MACHINE2
    if ((filament_ended_time == 0) || (Time - filament_ended_time < 40000)) {
      filament_ended = 0; //говорим, что пока лента не закончилась в течении 40 сек после его реального окончания, чтобі хвостик проплавило больше
    }
    #endif
  }
  
  //віставляем PWM сигнал для нагрева mosfet на контакт D3
  if (motor_direction == 1) { //наматіваем
    if ((filament_ended == 0) && (temperature_read < 255)) { //наматіваем + есть лента + нет перегрева
      regulator.input = temperature_read;
      analogWrite(PWM_heat_pin, regulator.getResultTimer());
      rotating_speed = max_speed * speeds_percent_arr[current_speed_idx] / 100.0; //предварительно считаем скорость мотора, максимум 60%
    } else { //ленті нет, перегрев - не греем
      analogWrite(PWM_heat_pin, 0);
      rotating_speed = 0;
    }

    //достигла ли температура необходимого значения (минимум 96% от необходимого)
    temperature_riched = (temperature_read >= set_temperature * 0.96);

    //включена намотка + включен мотор + (лента закочилась или температура не достигла нужного значения) = віключаем мотор (проверка на то, что нужно греть или нет чуть віше)
    if (stepper_motor_activated && ((filament_ended == 1) || (temperature_read < set_temperature * 0.96) || (temperature_read >= 255))){
      stepper_motor_activated = false;
    }
  } else { //когда сматіваем бабину - не греем
    analogWrite(PWM_heat_pin, 0);
    temperature_riched = false;
  }

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

    if (last_filament_ended != filament_ended) { //принудительно перерисовіваем данніе если статус ленті поменялся
      last_LCDdrawTime = 0;
      last_rotating_speed = -1;
      last_motor_direction = 1 - motor_direction;
      last_temperature_read = -100;
    }

    if (Time - last_LCDdrawTime > 1500) {
      if (stepper_motor_activated && (motor_direction == 1)) { //якщо мотор крутить і намотує
        cm = floor(stepper1.currentPosition()/steps_in_cm); //лічильник намотаного прутка
      }

      if (last_temperature_riched != temperature_riched) { //рисуем знак достиго ли значение температурі необходимого значения
        lcd.setCursor(8, 0);
        if (temperature_riched) {
          lcd.print("=");
        } else {
          lcd.print("-");
        }
        last_temperature_riched = temperature_riched;
      }

      if (last_filament_ended != filament_ended) {//малюємо чи закінчилася стрічка
        lcd.setCursor(15, 0);
        if (filament_ended == 1) { //стрічка закінчилася
          lcd.print('-');
        } else {
          lcd.print('+');
        }
        last_filament_ended = filament_ended;
      }

      if ((last_temperature_read != round(temperature_read)) && (motor_direction == 1)) { //рисуем текущую температуру
        lcd.setCursor(5, 0);
        lcd.print("  ");
        lcd.setCursor(4, 0);
        lcd.print((int) temperature_read);
        last_temperature_read = round(temperature_read);
      }

      if (last_motor_direction != motor_direction) { //малюємо температуру до якої прагне плавилка
        lcd.setCursor(12, 0);
        lcd.print("  ");
        lcd.setCursor(11, 0);
        if ((motor_direction == 1) && (filament_ended == 0)) { //наматіваем + есть лента
          char tmpstr[3];
          lcd.print(itoa(set_temperature, tmpstr, 10));
        } else { //когда сматіваем бабину - не греем
          lcd.print("0  ");
        }
        if (motor_direction == 0){ //если смативаем то рисуем температуру 0, чтобі мотор не дергался постоянно при смене температурі
          lcd.setCursor(4, 0);
          lcd.print("0  ");
          cm = 0; //показуємо відразу що не рахуємо скільки намотано було
        }
        last_motor_direction = motor_direction;
      }

      if (last_cm != cm) { //счетчик намотаного прутка в сантиметрах
        lcd.setCursor(11, 1);
        lcd.print("   ");
        lcd.setCursor(10, 1);
        lcd.print(cm);
        last_cm = cm;
      }
      
      if (last_rotating_speed != rotating_speed) {//рисуем скорость мотора
        lcd.setCursor(5, 1);
        lcd.print("    ");
        lcd.setCursor(4, 1);
        lcd.print(speeds_percent_arr[current_speed_idx], 0);
        lcd.print('%');
        last_rotating_speed = rotating_speed;
      }

      last_LCDdrawTime = Time;
    }
  } else if (active_menu == 1) { //меню настройка температурі
    if (menu_changed || (set_temperature != last_set_temperature)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set temperature");    
      lcd.setCursor(0, 1);
      char tmpstr[3];
      lcd.print(itoa(set_temperature, tmpstr, 10));
      lcd.print("  ");
      menu_changed = false;
    }
    last_set_temperature = set_temperature;
  }

  if (stepper_motor_activated) {//мотор активен
    digitalWrite(LED_pin, HIGH); //светодиод светит
    digitalWrite(EN_pin, LOW); //активируем мотор
    if (motor_direction == 0) { //сматіваем
      digitalWrite(microstep_pin, LOW); //полній шаг
      #ifndef MACHINE2
      rotating_speed = -max_speed/2.25;
      #endif
      #ifdef MACHINE2
      rotating_speed = -max_speed/2.13;
      #endif
    } else { //наматіваем
        digitalWrite(microstep_pin, HIGH); //1/16 шага
    }

    if ((motor_direction == 1) && (filament_ended == 0)) {
      digitalWrite(fan_pin, HIGH); //активируем вентилятор
    } else {
      digitalWrite(fan_pin, LOW); //деактивируем вентилятор
    }
  } else {
    digitalWrite(EN_pin, HIGH); //деактивируем мотор
    digitalWrite(fan_pin, LOW); //деактивируем вентилятор
    digitalWrite(LED_pin, LOW); //светодиод вимкнено
    rotating_speed = 0;
  }

  stepper1.setSpeed(rotating_speed);
  stepper1.runSpeed();
}

//Прерівания для кнопки та поворотного енкодера
ISR(PCINT0_vect) {
  static unsigned long last_interrupt_time = 0;
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  clk_State = (PINB & B00000001); //состояние clk_pin (D8)
  dt_State  = (PINB & B00000010); //состояние data_pin (D9)

  prevNextCode <<= 2;
  if (dt_State != 0) {
    prevNextCode |= 0x02;
  }
  if (clk_State != 0) {
    prevNextCode |= 0x01;
  }
  prevNextCode &= 0x0f;

  if (rot_enc_table[prevNextCode]) {
    store <<= 4;
    store |= prevNextCode;
    if ((store&0xff)==0x2b) {//проти часової стрілки
      if (active_menu == 0) {//главное меню - тут регулируем скорость
        if (current_speed_idx > 0) { 
          current_speed_idx--; 
          last_LCDdrawTime = 0; 
        }
      } else if (active_menu == 1) {//меню управления температурой 
        if (set_temperature > 0) { set_temperature -= 1; }
        regulator.setpoint = set_temperature;
      }
    }
    if ((store&0xff)==0x17) {//по часовій стрілці
      if (active_menu == 0) {//главное меню - тут регулируем скорость
        if (current_speed_idx < 6) {
          current_speed_idx++; 
          last_LCDdrawTime = 0; 
        }
      } else if (active_menu == 1) {//меню управления температурой 
        if (set_temperature <= 244) { set_temperature += 1; }
        regulator.setpoint = set_temperature;
      }
    }
  }

  //кнопка на енкодере нажата
  if ((PINB & B00001000) == 0) { //кнопка на енкодере rotary_button_pin (D11) нажата?
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 200){
      rotary_button_pressed = 1;
      last_interrupt_time = interrupt_time;
    }
  } else if (rotary_button_pressed == 1) { //Переходимо послідовно по двум меню з кожним натиском кнопки
    active_menu++;
    if (active_menu > 1) {
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
  } else {
    motor_direction = 0;//сматіваем
  }
}
