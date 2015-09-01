#include <LIS331.h>
#include <Wire.h>
#include <SPI.h>

#include "Adafruit_BLE_UART.h"

/*
 * BLE 
 */
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 6
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

/*
 * TOUCHE
 */
// Touche
#define SET(x,y) (x |=(1<<y))        //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))           // |
#define CHK(x,y) (x & (1<<y))               // |
#define TOG(x,y) (x^=(1<<y))                //-+

#define MULTIPLIER 10
#define N 16  //How many frequencies

float lastD = 0;
//float results[N];            //-Filtered result buffer
//float freq[N];            //-Filtered result buffer
//int sizeOfArray = N;
boolean request = false;

/*
 * ACCELEROMETER
 */
LIS331 lis;
int val;

float val1;

//SETUP FOR SMOOTHING
const int numReadings = 12;
int arrReadings[4][numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
float arrTotal[4];                  // the running total
int arrAverage[4];                // the average
int startVal[4];

boolean vInit = false;
long sTime;

String s = "[calibrate]|[calibrate]";

int lStartTouch = 0;
 
void setup() {

  val1 = 0;

  lis.begin();
  
  
  
  //Serial.begin(9600);

  for (int i=0; i<4; i++)
  {
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {

      arrReadings[i][thisReading] = 0;
      arrTotal[i] = 0;                  // the running total
      arrAverage[i] = 0;
    }
  }

  sTime = millis();

  /*
   * BLE
   */
  BTLEserial.setDeviceName("KYTS"); /* 7 characters max! */
  BTLEserial.begin();

}


void loop() {

  /*
   * BLE
   */
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }

    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }

    String _s = s + '\n';
    uint8_t sendbuffer[_s.length()];
    s.getBytes(sendbuffer, _s.length());
    char sendbuffersize = min(20, _s.length());

    BTLEserial.write(sendbuffer, sendbuffersize);
  }
  
  /*
  Serial.print(outVal(lis.getXValue(),0));
  Serial.print("\t");
  
  Serial.print(outVal(lis.getYValue(),1));
  Serial.print("\t");
  
  Serial.print(outVal(lis.getZValue(),2));
  Serial.println();
  */
  
  //Serial.println("Movement Index : " + stringOutVal(lis.getXValue(), lis.getYValue(), lis.getZValue() ));

  /*
   * CALIBRATE THE NUMBERS
   */
  if (vInit == false)
  {

    if (millis()  > 3000) {
         
      vInit = true;
      lStartTouch = totalOutVal(lis.getXValue(), lis.getYValue(), lis.getZValue());
    }
  }

  /*
   * ACCELEROMETER
   */

  for (int i=0; i<3; i++)
  {

    arrTotal[i] = arrTotal[i] - arrReadings[i][index];
 
    switch(i) {

      case 0:
        arrReadings[i][index] = _min(lis.getXValue());
        break;

      case 1:
        arrReadings[i][index] = _min(lis.getYValue());
        break;

      case 2:
        arrReadings[i][index] = _min(lis.getZValue());
        break;
    }

    // add the reading to the total:
    arrTotal[i] = arrTotal[i] + arrReadings[i][index];

    // calculate the average:
    arrAverage[i] = arrTotal[i] / numReadings;         
    // send it to the computer as ASCII digits
  }

  /*
   * TOUCHE
   */
  int counter = 0;

  TCCR1A = 0b10000010;        //-Set up frequency generator
  TCCR1B = 0b00011001;        //-+
  
  //ICR1 = 110;
  //OCR1A = 55;
  
  ICR1 = 110;
  OCR1A = 55;
  
  //pinMode(7, INPUT);        //put wrong connection into Hi-Z
  pinMode(8, OUTPUT);        //-Signal generator pin
  pinMode(9, OUTPUT);        //-Sync (test) pin

  /*
  for (int i = 0; i < N; i++)      //-Preset results
    results[i] = 0;  
  */
  
  s = "";
  float top_v = 0;
  float top_index = 0;

  /*
  for (unsigned int d = 0; d < N; d++)
  {
    int mod_d = d * MULTIPLIER * 100;
    int v = analogRead(0);    //-Read response signal
    
    CLR(TCCR1B, 0);           //-Stop generator
    //delay(100);

    TCNT1 = 0;                //-Reload new frequency
    ICR1 = mod_d;                 // |
    OCR1A = mod_d/2;              //-+
    SET(TCCR1B, 0);           //-Restart generator

    //float _d = results[d] * 0.5 + (float)(v) * 0.5;
    float _d = lastD * 0.5 + (float)(v) * 0.5;

    if (_d > top_v) {
      top_v = _d;
    }
    //s += String(mod_d) + "|" + String(_d) + ",";
 
                                 //Set new frequency
    //NRF_TIMER0->TASKS_START = 1; //restart generator

    //results[d] = _d;
    lastD = _d;
    
    //results[d] = results[d] * 0.5 + (float)(v) * 0.5; //Filter results

    //freq[d] = mod_d;
  }
  s = String(top_v);
  */
  
  TOG(PORTB, 0); 


  if (vInit == true) {
    String m = String(arrAverage[3]);
    s = m + "|" + String(totalOutVal(lis.getXValue(), lis.getYValue(), lis.getZValue()));
    Serial.println(s);
  }

  //s = String(analogRead(0)) + "\n";

    arrTotal[3] = arrTotal[3] - arrReadings[3][index];
    arrReadings[3][index] = analogRead(1);
    arrTotal[3] = arrTotal[3] + arrReadings[3][index];
    arrAverage[3] = arrTotal[3] / numReadings;  
    
  index = index + 1;
  if (index >= numReadings)
    index = 0;
}

int totalOutVal(int _val0, int _val1, int _val2) {

  return (outVal(_val0,0) + outVal(_val1,1) + outVal(_val2,2))/3;

}

int outVal(int _val, int _index) {

  int _t1 = _min(_val) ;
  int _t2 = abs(arrAverage[_index]);

  int _v = _t1 - _t2;
  
  return abs(_v);
}

int _min(int _t) {

  _t = abs(_t);
  return _t/10;
}

