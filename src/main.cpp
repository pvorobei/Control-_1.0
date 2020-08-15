#include <Arduino.h>                             //#include <avr/eeprom.h>(если нужен будет еепром)
#include <DS3231.h>
#include <Wire.h>
#include <SHT2x.h>
#define PERIOD_RTC 60000
#define PERIOD_READ 2000
#define PERIOD_SUNRISE 4000
  DS3231 Clock;
  bool h12;                                      //хз зачем но с этими переменными rtc работает, без нет
  bool PM;
  uint8_t sunrise = 8;                           //Час расввета
 // uint8_t* ptr_sunrise = &sunrise;               //Указатель на час рассвета
  uint8_t night = 20;                            //Час заката  
 // uint8_t* ptr_night = &night;                   //Указатель на час заката    
  float Temp_Ust = 23.9;                         //Уставка температуры для парника
 // float* ptr_temp_ust = &Temp_Ust;               //Указатель на уставку температуры для парника
  float Temp_Ust_Inc = 27.9;                     //Уставка температуры для инкубатора
 // float* ptr_temp_ust_inc = &Temp_Ust_Inc;       //Указатель на уставку температуры для Инккубатора   
  float TemperatureOne;                          //реальное  значение температуры   
  uint8_t Hour;                                  //Часы полученные с ds3231 
  uint8_t minut;                                 //Минуты полученные c ds3231
  uint32_t myTimer1 = 0;                         //таймер для получения и отпрправки времени
  uint32_t myTimer2 = 0;                         //таймер для получения темп и влажности
  uint32_t myTimer3 = 0;                         //таймер для Рассвет/закат
    //float Humid_Ust;                           //Уставка влажности
    // float* ptr_humid_ust = &Humid_Ust;        //Указатель на уставку влажности
    // float HumidityOne;                        //реальное значение влажности 

//____________Функция реакции на температуру _____________//
void ReactTemp(float TempOne, float TempUst){
  if(TempOne < TempUst){
   PORTD |= (1<<4);                               //digitalWrite (Pin_Head, HIGH);
  }
   else
    {
     PORTD &= ~(1<<4);                             //digitalWrite(Pin_Head, LOW);
    }          
}

//________Функция для вкл/выкл PIN 9 в режим ШИМ_________//
void PwmOnOff(boolean On_Off){
  if (On_Off){
    TCCR1A |= (1<<COM1A1);                         // Подключаем шим  в неинвертирующем режиме.. 
    TCCR1A &= ~(1<<COM1A0);                        // ..для PB1 согласно таблице 15-3 даташита
  }
  else if (!On_Off){
    TCCR1A &= ~(1<<COM1A1);                        // Отключаем шим от порта.. 
    TCCR1A &= ~(1<<COM1A0);                        // ..для PB1 согласно таблице 15-3 даташита    
  }
}

void setup() {
                                                     // DDRB = DDRB | 0b00111111;так можно выставить биты с 0по5 не меняя 6и7(может пригодиться для подтяжки пинов к земле)
  DDRD = 0b01110010;                                 // настроили порт D(D4,5,6,7 на выход 2 и 3 на вход, 0(RX)на вход, 1(TX)на вход)
                                                     // D4- нагрев,D5 -вентиляция, D6 - увлажнитель, D7 - освещение(соответ-ют пинам промини)
 // PORTD |= (1<<3);                                   //Подтянем 3 пин к +5
  DDRB |= (1<<1);                                    // уст. 1 бит порта B (пин 9) на выход(освещение)
  PORTB &= ~(1<<1);                                  // изначально подадим 0(gnd) 
  PORTD |= (1<<1);                                   // Чтобы не горел светодиод ТХ

//Для шим
  PwmOnOff(true);                                    //Подключаем шим  в неинвертирующем режиме.. 
  TCCR1A |= (1<<WGM10);                              // Устанавливаем биты в регистрах TCCR1A и TCCR1B
  TCCR1A &= ~(1<<WGM11);                             // для использования 8 битной быстрой(FAST) ШИМ 
  TCCR1B |= (1<<WGM12);                              // согласно таблице 15.5
  TCCR1B &= ~(1<<WGM13);                             // ...

  TCCR1B |= (1<<CS11);                               // Настраиваем предделитель основной частоты МК 
  TCCR1B &= ~(1<<CS10);                              // с помощью установки бит в регистре TCCR1B
  TCCR1B &= ~(1<<CS12);                              // согласно таблице 15.6 (стр.110)

 // OCR1A = 255;                                       // Задаем значение  шим 
 
  Serial.begin(9600);
  Wire.begin();   
  delay(50);
  Hour = Clock.getHour(h12, PM);                      // Считываем часы
  minut = Clock.getMinute();                          // Считываем минуты
  TemperatureOne = SHT2x.readT();                     // Чтение температуры и влажности
                                                      // HumidityOne = SHT2x.readRH();
}

void loop() {
/* if (millis() - myTimer1 >= 60000) {                  // таймер на минуту для считывания времени
   myTimer1 = millis();                               // сброс таймера
   Hour = Clock.getHour(h12, PM);
   minut = Clock.getMinute();
  }*/

  if (millis() - myTimer1 >= PERIOD_RTC){             // таймер на минуту для считывания времени
    Hour = Clock.getHour(h12, PM);                    // Считываем часы
    minut = Clock.getMinute();                        // Считываем минуты
    do {
      myTimer1 += PERIOD_RTC;                         // обновляем значение таймера
      if (myTimer1 < PERIOD_RTC) break;               // переполнение uint32_t
    } while (myTimer1 < millis() - PERIOD_RTC);       // защита от пропуска шага
  }

  if (millis() - myTimer2 >= PERIOD_READ){            // таймер на 2 сек для считывания темп и вл
    TemperatureOne = SHT2x.readT();                   // Чтение температуры и влажности
                                                      // HumidityOne = SHT2x.readRH(); 
    if(~PIND & (1<<3)) {                              // Если на 3 пине 0
      ReactTemp(TemperatureOne, Temp_Ust);            // Отправляем в функцию темп и уставку для парника
    }   
    else if(PIND & (1<<3)) {                          // Если если на 3 пине 1 
      ReactTemp(TemperatureOne, Temp_Ust_Inc);        // Отправляем в функцию темп и уставку для инкубатора
    }
    do {
      myTimer2 += PERIOD_READ;                        // обновляем значение таймера
      if (myTimer2 < PERIOD_READ) break;              // переполнение uint32_t
    } while (myTimer2 < millis() - PERIOD_READ);      // защита от пропуска шага
  }

if(~PIND & (1<<3)) {                                  // Если на 3 пине 0  (Парник)
  if((minut >= 0) & (minut <= 2)){PORTD |= (1<<5);}   // каждый час в 0 минут включаем вент, через 3 мин выключаем
     else{PORTD &= ~(1<<5);}                          //
  if((minut >= 3) & (minut <= 5)){PORTD |= (1<<6);}   // каждый час в 3 минуты включаем увлажнитель, через 2 мин выключаем
     else{PORTD &= ~(1<<6);}                          //    
  if ((Hour >= sunrise) & (Hour <= night)){           // если настал час рассвета
    //рассвет
  }   
  if((Hour >= night) & (Hour <= sunrise)){            // если настал час заката
    //закат
  }

  }
  else if(PIND & (1<<3)) {                            // Если на 3 пине 1 (Инкубатор)
   PORTD &= ~((1<<5) | (1<<6));                       // Увлажнитель, вентиляцию  уст в 0 (выключить)

   PwmOnOff(false);                                   //Отключаем шим от порта
   PORTB &= ~(1<<1);                                  // Выставляем РВ1 в ноль (освещение)
  }

}
/*
//--------Правильный millis--------
#define PERIOD 500
uint32_t timer = 0;
void loop() {
  if (millis() - timer >= PERIOD) {
    // ваше действие
    do {
      timer += PERIOD;
      if (timer < PERIOD) break;  // переполнение uint32_t
    } while (timer < millis() - PERIOD); // защита от пропуска шага
  }
}
*/
/*
   TemperatureOne = SHT2x.readT();                    // Чтение температуры и влажности
                                                      // HumidityOne = SHT2x.readRH(); 
    if(~PIND & (1<<3)) {                              // Если на 3 пине 0
      ReactTemp(TemperatureOne, Temp_Ust);            // Отправляем в функцию темп и уставку для парника
    }   
    else if(PIND & (1<<3)) {                          // Если если на 3 пине 1 
      ReactTemp(TemperatureOne, Temp_Ust_Inc);        // Отправляем в функцию темп и уставку для инкубатора
    }
*/