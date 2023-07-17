#include <LiquidCrystal_I2C.h>
#include "TimerOne.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include "DHT.h"
//LCD for show status and trash fill
LiquidCrystal_I2C lcd(0x27, 16, 2);
//Servo for open and close trash cap
Servo myservo;
//Comunicate with ESP8266 though Serial method port 9600
SoftwareSerial modemWifi(3, 4);
const int DHTPIN = 2;       //Đọc dữ liệu từ DHT11 ở chân 2 trên mạch Arduino
const int DHTTYPE = DHT11;  //Khai báo loại cảm biến, có 2 loại là DHT11 và DHT22
 
DHT dht(DHTPIN, DHTTYPE);
//Ultra Sonic for check trash capacity
const unsigned int CAP_TRIG_PIN = 13;
const unsigned int CAP_ECHO_PIN = 12;
//Ultra Sonic  for check people nearby
const unsigned int CHECK_TRIG_PIN = 11;
const unsigned int CHECK_ECHO_PIN = 10;
//LED RGB
const unsigned int RED_LED = 8;
const unsigned int GREEN_LED = 7;
const unsigned int BLUE_LED = 6;
//SERVO_PIN
const unsigned int SERVO_PIN = 5;
const unsigned int MIN_DISTANCE = 80;

const unsigned int DELAYS = 100;
const unsigned int BAUD_RATE = 9600;
int pos = 180;
const int numReadings = 10;
String trashCapStatus = "CLOSE";
long capReadings[numReadings];    // the readings from the analog input
int capReadIndex = 0;             // the index of the current reading
long capTotal = 0;                // the running total
long capAverage = 0;              // the average
long checkReadings[numReadings];  // the readings from the analog input
int checkReadIndex = 0;           // the index of the current reading
long checkTotal = 0;              // the running total
long checkAverage = 0;            // the average
long distance = 0;
long currentTrashFill = 0;
float temperature = 0;
float humid = 0;
void setup() {
  Serial.begin(BAUD_RATE);
  modemWifi.begin(9600);
  myservo.attach(SERVO_PIN);
  dht.begin();
  lcd.init();
  lcd.backlight();
  myservo.write(0);
  pinMode(CAP_TRIG_PIN, OUTPUT);
  pinMode(CHECK_TRIG_PIN, OUTPUT);
  pinMode(CAP_ECHO_PIN, INPUT);
  pinMode(CHECK_ECHO_PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    capReadings[thisReading] = 0;
    checkReadings[thisReading] = 0;
  }
  // Timer1.initialize(500000);
  // Timer1.attachInterrupt(sendDataOnline);
}

void loop() {
  openclose();
  getTemperatureAndHumid();
  sendDataOnline();
  // testSendMCU();
  screenView();
  setTrashLedStatusDisplay();
  // Serial.println(pos);
}

void openclose() {
  long distance = getDistanceFromUltraSonicSensor(2);
  if (distance < 100) {
    myservo.write(90);
    pos = 90;
    trashCapStatus = "OPEN";
  }
  if (distance > 100) {
    myservo.write(180);
    pos = 0;
    trashCapStatus = "CLOSE";
  }
}

long percentTrashFill() {
  long distance = getDistanceFromUltraSonicSensor(1);
  // currentTrashFill = (25 - distance) * 2;
  currentTrashFill = map(distance, 25, 5, 0, 100);
  if (currentTrashFill < 0) currentTrashFill = 0;
  if (currentTrashFill > 100) currentTrashFill = 100;
  return currentTrashFill;
}

void screenView() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("STATUS: " + trashCapStatus);
  lcd.setCursor(1, 1);
  lcd.print(currentTrashFill);
  lcd.print("%");
  delay(100);
}

void testSendMCU(){
  int ran = random(1,100);
  String send = (String)ran + ","+trashCapStatus+","+(String) temperature+"," + (String) humid;
  Serial.println(send);
  modemWifi.println(send);
  delay(1000);
}

long getDistanceFromUltraSonicSensor(int distance_device) {
  unsigned long duration = 0;
  switch (distance_device) {
    case 1:
      digitalWrite(CAP_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(CAP_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(CAP_TRIG_PIN, LOW);
      duration = pulseIn(CAP_ECHO_PIN, HIGH);
      break;
    case 2:
      digitalWrite(CHECK_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(CHECK_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(CHECK_TRIG_PIN, LOW);
      duration = pulseIn(CHECK_ECHO_PIN, HIGH);
      break;
  }
  return (34 * duration) / 2000;
}

long smoothDistanceValue(int distance_device) {

  switch (distance_device) {
    case 1:
      capTotal = capTotal - capReadings[capReadIndex];
      // read from the sensor:
      capReadings[capReadIndex] = getDistanceFromUltraSonicSensor(distance_device);
      // add the reading to the total:
      capTotal = capTotal + capReadings[capReadIndex];
      // advance to the next position in the array:
      capReadIndex = capReadIndex + 1;

      // if we're at the end of the array...
      if (capReadIndex >= numReadings) {
        // ...wrap around to the beginning:
        capReadIndex = 0;
      }

      // calculate the average:
      capAverage = capTotal / numReadings;
      // send it to the computer as ASCII digits
      Serial.println(capAverage);  // delay in between reads for stability
      return capAverage;
      break;

    case 2:
      checkTotal = checkTotal - checkReadings[checkReadIndex];
      // read from the sensor:
      checkReadings[checkReadIndex] = getDistanceFromUltraSonicSensor(distance_device);
      // add the reading to the total:
      checkTotal = checkTotal + checkReadings[checkReadIndex];
      // advance to the next position in the array:
      checkReadIndex = checkReadIndex + 1;

      // if we're at the end of the array...
      if (checkReadIndex >= numReadings) {
        // ...wrap around to the beginning:
        checkReadIndex = 0;
      }

      // calculate the average:
      checkAverage = checkTotal / numReadings;
      // send it to the computer as ASCII digits
      Serial.println(checkAverage);  // delay in between reads for stability
      return checkAverage;
      break;
  }
}

void sendDataOnline() {
  long temp = percentTrashFill();
  String send = (String)temp + ","+trashCapStatus+","+(String) temperature+"," + (String) humid;
  modemWifi.println(send);
  Serial.println(send);
  delay(1000);
}

void setTrashLedStatusDisplay() {
  if (currentTrashFill < 50) setLedColor(13, 220, 34);
  else if (currentTrashFill >= 50 && currentTrashFill < 90) setLedColor(255, 179, 0);
  else setLedColor(255, 0, 0);
}

void setLedColor(int red, int green, int blue) {
  analogWrite(RED_LED, red);
  analogWrite(GREEN_LED, green);
  analogWrite(BLUE_LED, blue);
}

void getTemperatureAndHumid(){
  temperature = dht.readTemperature();
  humid = dht.readHumidity();
}