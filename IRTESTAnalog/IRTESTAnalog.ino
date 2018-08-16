
//#include <DueTimer.h>
#include <SimpleTimer.h>
SimpleTimer timer;

#define normal_blink_rate  0.5                  // LED blink rate in Hz
#define error_blink_rate   4.0             z     // LED blink rate in Hz
#define blink_normal_period floor(1000000/normal_blink_rate)       //   blink period in us
#define blink_error_period  floor(1000000/error_blink_rate)        //   blink period in us

#define fs 10                                     //   sample rate in Hz
#define ts floor(1000000/fs)                 22       //   sample period in us
#define Fc 1.0                                    //   corner frequency of EMA filter (Hz)
const float coA (1-exp(-6.3*Fc/fs));                   //   can auto-calculate the value at compile time

static signed int filter_IR(signed int , signed int , float);

static int EMAVal, IRVal, IRValLED, x, dist;
static unsigned int tSampCount;
static int temp;
static float percentage, start_prog, stop_time, prog_time, endTime2, onTime, offTime, time1, time2;


//----------------------------- IO DEFINITIONS -----------------------------------------
const int GREENLED = 46;  
const int IRACL = 52;
const int IRLED = 50;
//--------------------------------------------------------------------------------------

void setup() {
  
  //----------------------------- IO -----------------------------------------
  pinMode(GREENLED, OUTPUT);
  pinMode(IRACL, OUTPUT);
  pinMode(IRLED, OUTPUT);
  

  analogWrite(IRACL, 0);
  analogWrite(IRLED, 0);
  digitalWrite(GREENLED, LOW);
  //--------------------------------------------------------------------------

  Serial.begin(9600);          // start serial communication at 115200bps
  while (!Serial);
  Serial.println("\n IR test code");
  
  tSampCount = 0;

  //timer.setInterval(1, LEDBlink);

  //--------------Timers-----------------
//  Timer3.attachInterrupt(measure_IR).setFrequency(fs).start();
 // Timer5.attachInterrupt(LEDBlink).setFrequency(1).start();
  //--------------------------------------------------------------------------
}

void loop() {
   measure_IR();
  // timer.run();
 Serial.println("IRVal:");
  Serial.println(IRVal);
    Serial.println("");
  Serial.println("IRValLED:");
  Serial.println(IRValLED);
    Serial.println("");
      Serial.println("Change:");
  Serial.println(abs(IRVal - IRValLED));
    Serial.println("");
    
Serial.println("PROGRAM RUNTIME");
  Serial.println(prog_time);
  Serial.println("");

  /*Serial.println("1 FOR COVERED, 0 FOR OPEN");
  dist = 0;
  if ((abs(IRVal - IRValLED) > 1) || IRVal < 2){
      
    dist = 1;
    
  }
  Serial.println(dist);
  Serial.println("");
 */

  delay(3000);

  //Serial.println(IRVal);
  //print every 10th pressure value:
//  if (tSampCount % 10 == 0) {
//      Serial.println(IRVal);   
//      Serial.println(EMAVal);   
//      Serial.println(' ');  
//  }


 //Serial.println(EMAVal);
 EMAVal = filter_IR(EMAVal, IRVal, coA);   //use recursive EMAVal to store processed sensor value
  
  tSampCount++;

}

void measure_IR(void){
  onTime = micros();
 //read IR:
 
 IRVal = 0;
 IRValLED = 0;
 x = 0;

 
 digitalWrite(IRLED, LOW); 
 pinMode(IRACL, OUTPUT);
 digitalWrite(IRACL, HIGH);  //pre-charge IR holding cap

 //delay to make sure cap is precharged
 delayMicroseconds(25);
 

 //discharging capacitor
 pinMode(IRACL, INPUT);      //release IR pin
 digitalWrite(IRACL, LOW);  //turn off pullups
 x = 1;

 //loop that continues until IRACL gets to 1 V(arbitrary cutoff VL)
start_prog = micros();
 while((x  == 1) && (IRVal < 50000)) {
    IRVal++;  //count until IRpin goes low
    x = digitalRead(IRACL);
 }
stop_time = micros();
time1 = stop_time - start_prog;

 //repeat measurement with IRLED on 


 
 pinMode(IRLED, OUTPUT);
 digitalWrite(IRLED, HIGH);
 pinMode(IRACL, OUTPUT);
 digitalWrite(IRACL, HIGH);  //pre-charge IR holding cap


 //delay to make sure cap is precharged
 delayMicroseconds(25);


 
 //discharging capacitor
 pinMode(IRACL, INPUT);      //release IR pin4
 digitalWrite(IRACL, LOW);  //turn off pullups

 
 x = 1;

 start_prog = micros();
 pinMode(IRLED, OUTPUT);
 digitalWrite(IRLED, HIGH);
 while((x == 1 ) && (IRValLED < 50000)) {
    IRValLED++;  //count until IRpin goes low
    x = digitalRead(IRACL);
 }
   stop_time = micros();
    pinMode(IRLED, OUTPUT);
    digitalWrite(IRLED, LOW);


 time2 = stop_time - start_prog;
 offTime = micros();
 prog_time = offTime - onTime;


}

void LEDBlink(void){

  digitalWrite(IRLED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delayMicroseconds(10);                       // wait for a second
  digitalWrite(IRLED, LOW);    // turn the LED off by making the voltage LOW
  delayMicroseconds(10);
}

signed int filter_IR(signed int EMA_OLD, signed int NewVal, float coA){
  signed int EMA_NEW;
  
  EMA_NEW = NewVal;   
  EMA_NEW = EMA_NEW >> 4;  //rotate NewVal right to truncate - this can be adjusted to scale the resolution/range
  //EMA_NEW is the current value, EMA_OLD is the last filter value
  
  EMA_NEW = EMA_OLD + ( coA * (EMA_NEW - EMA_OLD));  //this version only uses one multiply
      
  return EMA_NEW;
    
} //end of function filter_IR
//---------------------------------------------------------------
