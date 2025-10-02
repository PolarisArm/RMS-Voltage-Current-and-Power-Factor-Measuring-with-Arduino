// Last update 10/02/2025
// Circuit board making is completed.

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define volt A0
#define curr A1
#define voltMultiplier 363
#define currMultiplier 100

float Volt_Calib = 1.65;
float Curr_Calib = 1.65;

unsigned long previousMicros = 0;
unsigned long interval = 200;

double sqrVolt = 0.0;
double sqrCurr = 0.0;
double sumP    = 0.0;

int sample = 0;
float RMSV = 0;
float RMSC = 0;
float alpha = 0.95;

float RMSV_T = 0.0;
float PREV_RMSV = 0.0;
float PREV_V = 0.0;
float PREV_C = 0.0;

float RMSC_T = 0.0;
float PREV_RMSC = 0.0;

// ---- LCD ----
LiquidCrystal_I2C lcd(0x27, 16, 2);   // check your I2C address (0x27 or 0x3F)
char buf[17];

void setup() {
  pinMode(volt, INPUT);
  Serial.begin(9600);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Timer
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = 15999;
  delay(10);
}

volatile int sampleReady = 0;
volatile float voltVal = 0.0;
volatile float currVal = 0.0;

ISR(TIMER1_COMPA_vect) {
  voltVal = (((analogRead(volt)) * 5.0 / 1023.0) - 2.55) * alpha + (1 - alpha) * PREV_V;
  currVal = (((analogRead(curr)) * 5.0 / 1023.0) - 2.55) * 0.98 + (1 - 0.98) * PREV_C;
  PREV_V = voltVal;
  PREV_C = currVal;
  sampleReady = 1;
}

void loop() {
  if (sampleReady == 1) {
    sqrVolt += voltVal * voltVal;
    sqrCurr += currVal * currVal;
    sumP    += voltVal * currVal;  // instantaneous power accumulation

    voltVal = 0;
    currVal = 0;
    sampleReady = 0;
    sample++;
  }

  if (sample >= 1000) {
    sample = 0;

    float tempVolt = sqrVolt / 1000;
    float tempCurr = sqrCurr / 1000;
    float tempP    = sumP / 1000;

    RMSV = sqrt(tempVolt) ;
    RMSC = sqrt(tempCurr);

    RMSV_T = (RMSV * voltMultiplier) * alpha + (1 - alpha) * PREV_RMSV;
    RMSC_T = (RMSC * currMultiplier) * 0.98 + (1 - 0.98) * PREV_RMSC;

    PREV_RMSV = RMSV;
    PREV_RMSC = RMSC_T;

    float realPower     = tempP * voltMultiplier * currMultiplier;
    float apparentPower = RMSV_T * RMSC_T;
    float powerFactor   = realPower / apparentPower;

    sqrVolt = 0;
    sqrCurr = 0;
    sumP    = 0;

    // --- Debug to Serial ---
    Serial.print(">RMSV_T: ");
    Serial.print(RMSV_T);
    Serial.print(" RMSC_T: ");
    Serial.print(RMSC_T);
    Serial.print(" RealPower: ");
    Serial.print(realPower);
    Serial.print(" PF: ");
    Serial.println(powerFactor, 3);

    // --- LCD Update ---
    lcd.setCursor(0, 0);
    lcd.print("V:");
    dtostrf(RMSV_T, 6, 2, buf);
    lcd.print(buf);
    lcd.print(" I:");
    dtostrf(RMSC_T, 6, 2, buf);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    lcd.print("PF:");
    dtostrf(powerFactor, 4, 2, buf);
    lcd.print(buf);
  }
}
