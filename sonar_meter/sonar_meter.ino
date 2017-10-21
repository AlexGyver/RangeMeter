//Created 2017 by AlexGyver

// -------НАСТРОЙКИ-------
// длина корпуса в сантиметрах, для переноса начала отсчёта в заднюю часть корпуса
float case_offset = 10.0;
// -------НАСТРОЙКИ-------

// сонар
#define ECHO 2
#define TRIG 3
#define sensVCC 4

// дисплей
#define dispGND 5
byte DIO = 6;
byte RCLK = 7;
byte SCLK = 8;
#define dispVCC 9

// переключатель
#define buttPIN 11
#define buttGND 12

// создаём дисплей
#include <TM74HC595Display.h>
#include <TimerOne.h>
TM74HC595Display disp(SCLK, RCLK, DIO);
unsigned char SYM[47];

// крутая библиотека сонара
#include <NewPing.h>
NewPing sonar(TRIG, ECHO, 400);

float dist_3[3] = {0.0, 0.0, 0.0};   // массив для хранения трёх последних измерений
float middle, dist, dist_filtered;
float k;
byte i, delta;
unsigned long dispIsrTimer, sensTimer;

void setup() {
  Serial.begin(9600);
  symbols();            // создать символы для отображения на дисплее

  // настройка пинов
  pinMode(sensVCC, OUTPUT);
  pinMode(dispGND, OUTPUT);
  pinMode(dispVCC, OUTPUT);
  pinMode(buttPIN, INPUT_PULLUP);
  pinMode(buttGND, OUTPUT);

  // подаём сигналы на пины
  digitalWrite(sensVCC, 1);
  digitalWrite(dispGND, 0);
  digitalWrite(dispVCC, 1);
  digitalWrite(buttGND, 0);
}

void loop() {
  if (millis() - sensTimer > 50) {                          // измерение и вывод каждые 50 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
    if (i > 1) i = 0;
    else i++;

    dist_3[i] = (float)sonar.ping() / 57.5;                 // получить расстояние в текущую ячейку массива
    if (!digitalRead(buttPIN)) dist_3[i] += case_offset;    // если включен переключатель стороны измерения, прибавить case_offset
    dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);    // фильтровать медианным фильтром из 3ёх последних измерений

    delta = abs(dist_filtered - dist);                      // расчёт изменения с предыдущим
    if (delta > 1) k = 0.7;                                 // если большое - резкий коэффициент
    else k = 0.08;                                          // если маленькое - плавный коэффициент

    dist_filtered = dist * k + dist_filtered * (1 - k);     // фильтр "бегущее среднее"

    disp.clear();                                           // очистить дисплей
    disp.float_dot(dist_filtered, 1);                       // вывести
    sensTimer = millis();                                   // сбросить таймер
  }

  if (micros() - dispIsrTimer > 300) {       // таймер динамической индикации (по-русски: КОСТЫЛЬ!)
    disp.timerIsr();                         // "пнуть" дисплей
    dispIsrTimer = micros();                 // сбросить таймер
  }
}

// медианный фильтр из 3ёх значений
float middle_of_3(float a, float b, float c) {
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

// символы для дисплея
void symbols() {
  // обычные
  SYM[0] = 0xC0; //0
  SYM[1] = 0xF9; //1
  SYM[2] = 0xA4; //2
  SYM[3] = 0xB0; //3
  SYM[4] = 0x99; //4
  SYM[5] = 0x92; //5
  SYM[6] = 0x82; //6
  SYM[7] = 0xF8; //7
  SYM[8] = 0x80; //8
  SYM[9] = 0x90; //9

  // с точкой
  SYM[10] = 0b01000000; //0.
  SYM[11] = 0b01111001; //1.
  SYM[12] = 0b00100100; //2.
  SYM[13] = 0b00110000; //3.
  SYM[14] = 0b00011001; //4.
  SYM[15] = 0b00010010; //5.
  SYM[16] = 0b00000010; //6.
  SYM[17] = 0b01111000; //7.
  SYM[18] = 0b00000000; //8.
  SYM[19] = 0b00010000; //9.
}
