#include<Wire.h>
#include "Kalman.h"
#include "Timer.h"

#define STD_LOOP_TIME 10
#define degconvert 57.2957786 //there are like 57 degrees in a radian.

int timeGoneBy = STD_LOOP_TIME;
double timer ;
unsigned long loopStartTime = 0;
int mpu_addr = 0x68; //who am i ? (mpu6050)
double angle ; 
//////////for pid//////////////////////
float Kp = 0;                       // (P)roportional Tuning Parameter
float Ki = 0.005;                   // (I)ntegral Tuning Parameter        
float Kd = 20;                  // (D)erivative Tuning Parameter       
float lastpitch;                // Keeps track of error over time
float iTerm;                    // Used to accumalate error (intergral)
float targetAngle = 0;          // Can be adjusted according to centre of gravity 
 
double thisTime = 0;
double lastTime = 0;
////////////////////////////////////////
Kalman kalmanX;


//////////  moteurs /////////
int moteurA = 10, moteurB = 11; // input pwm for motors 
int sensA = 12, sensB = 13;
int motorOutput ; 
////////////////////////////

//////////// Potentiomete///////////
int port_kp = 3, port_ki = 2,port_kd= 1;

Timer t;

// =====================================================================
// *************************** SETUP ***********************************
// =====================================================================
void setup()
{
  // put your setup code here, to run once:
  pinMode(moteurA, OUTPUT);
  pinMode(moteurB, OUTPUT);
  pinMode(sensA, OUTPUT);
  pinMode(sensB, OUTPUT);

  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B);
  Wire.write(0); // wake up the mpu6050
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(1000);
  timer = micros();
  t.every(9.9,get_angle);
  t.every(13,PID);
}



// ======================================================================
// ====                         MAIN LOOP                            ====
// ======================================================================

void loop() {            

    double sens;
    // double angle = get_angle();
    t.update();
    sens = (angle>0) ? 1: 0 ;
    //motorOutput = PID();
    tourne_moteurs(sens,motorOutput);

    /////// potentiometre//////
    Kp = analogRead(port_kp);
    Ki = analogRead(port_ki);
    Kd = analogRead(port_kd);
    Kp = map(Kp, 0, 1024, 0, 50);
    Kd = map(Kd, 0, 1024, 0, 15);
    Ki = 0.001 * Ki;
    ////////////////////////////
    affichage(angle,motorOutput); 
    
   // timekeeper();
}



// ======================================================================
// ====                       PID CONTROLLER                         ====
// ======================================================================

void PID() 
{                
    float pitch=angle ; 
   // thisTime = millis();
    //double timeChange = double(thisTime - lastTime);
    //Serial.println(timeChange);
    float error = targetAngle - pitch;
 
    float pTerm = Kp*error;
    iTerm += Ki*error;
    if(Ki<0.0009) iTerm=0;
    if(iTerm> 230) iTerm = 230 ;
    else if(iTerm <-230) iTerm = -230 ;
    float dTerm = Kd*(pitch - lastpitch) ; 
 
    lastpitch = pitch;
    
    //lastTime = thisTime;

    float PIDValue = pTerm + iTerm - dTerm;

    if (PIDValue > 230) PIDValue = 230;
    else if (PIDValue < -230) PIDValue = -230;
    motorOutput = abs(int(PIDValue)); 
    //return abs(int(PIDValue));
}



// ================================================================
// ===                   TIME KEEPER  ~100Hz                    ===
// ================================================================

void timekeeper() 
{
    timeGoneBy = millis() - loopStartTime;
    if (timeGoneBy < STD_LOOP_TIME) 
    {
        delay(abs(STD_LOOP_TIME - timeGoneBy));
    } else 
    {
        stop_moteurs();  
    }
    loopStartTime = millis();   
}
// ==================================================================
// ==                      Get angle                              ===
// ==================================================================
void get_angle()
{
  double AcY = Acc_Y() ;
  double AcZ = Acc_Z() ;
  double GyX = Gyro_X() ;
  double Roll= atan2(AcY, AcZ)*degconvert;
  GyX = GyX/131.0 ; 
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
  double tilt_X = kalmanX.getAngle(Roll, GyX,dt); 
  //tilt_X = Complementary_Filter(Roll, GyX, dt);
  angle = tilt_X ;
}


void stop_moteurs()
{
  analogWrite(moteurA, 0);
  analogWrite(moteurB, 0);
}

void tourne_moteurs(int sens, int pwm)
{
  digitalWrite(sensA, sens);
  analogWrite(moteurA, pwm);
  digitalWrite(sensB, sens);
  analogWrite(moteurB, pwm);
}

double Gyro_X()
{
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr,2);
  double GyX=Wire.read()<<8|Wire.read();
  Wire.endTransmission(true);
  return GyX;
}

double Acc_Y()
{
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr,2);
  double AccY=Wire.read()<<8|Wire.read();
  Wire.endTransmission(true);
  return AccY;  
}

double Acc_Z()
{
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3F);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr,2);
  double AccZ=Wire.read()<<8|Wire.read();
  Wire.endTransmission(true);
  return AccZ;  
}

void affichage(double tilt_X,double pwm)
{
    Serial.print("kp ==  ");
    Serial.print(Kp);
    Serial.print("   ki ==  ");
    Serial.print(Ki,6);
    Serial.print("   kd ==  ");
    Serial.print(Kd);
    Serial.print("  tilt==  ");
    Serial.print(tilt_X);
    Serial.print("  PWM==   ");
    Serial.println(pwm);
  
}
