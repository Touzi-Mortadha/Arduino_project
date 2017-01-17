//copyright Touzi Mortadha 


#include<Wire.h>
#include "Kalman.h"
int mpu_addr = 0x68; //who am i ? (mpu6050)
int16_t AcY, AcZ, GyX;
uint32_t timer;

double tilt_X; //These is the angle
int moteurA = 10, moteurB = 11; // input pwm for motors 
int sensA = 12, sensB = 13;
int sens ; // les sens de rotation des moteurs
double lastTime;
double reference = -0.4;
Kalman kalmanX;

int speed; //pwm

// PID
 float Kp = 6;
 float Ki = 1;
 float Kd = 100;
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
const float K = 1.9 * 1.12;
#define   GUARD_GAIN   20.0

//////////////
int port_kp = 3, port_ki = 2,port_kd= 1;

#define degconvert 57.2957786 //there are like 57 degrees in a radian.


///pid 2 
double ITerm ; 
double lastInput,thisTime,timeChange,lastpitch;

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
}

void loop()
{
    /// lire les valeurs de l'acc et gyro a partir de l'MPU6050 ///    
    AcY = Acc_Y() ;
    AcZ = Acc_Z() ;
    GyX = Gyro_X() ;
    
    double Roll= atan2(AcY, AcZ)*degconvert;
    double GyX = GyX/131.0 ;

    
    double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
    timer = micros(); //start the timer again so that we can calculate the next dt.
    //determiner speed(pwm) a l'aide d'un PID controller
    
    //pid();
    PID();
    
    //determiner le sense du moteur
    sens = (tilt_X > 0) ? 1 : 0 ;
    
    speed = abs(speed);
    
    //determiner l'angle soit a partir de kalman filter soit a partir d'un filter complementaire
    //tilt_X = Complementary_Filter(Roll, GyX, dt);
    tilt_X = kalmanX.getAngle(Roll, GyX,dt);
    
    // faire tourner les moteur
    tourne_moteur(sens, speed);
    delay(5);
    /////// potentiometre//////
    Kp = analogRead(port_kp);
    Ki = analogRead(port_ki);
    Kd = analogRead(port_kd);
    Kp = map(Kp, 0, 1024, 0, 20);
    Ki = 0.000488 * Ki;
    /////// simple affichage /////
        
    Serial.print("        kp ==  ");
    Serial.print(Kp);
    Serial.print("   ki ==  ");
    Serial.print(Ki);
    Serial.print("   kd ==  ");
    Serial.print(Kd);
    Serial.print("  tilt==  ");
    Serial.print(tilt_X);
    Serial.print("  PWM==   ");
    Serial.println(speed);
}


void tourne_moteur(int sens, int pwm)
{
  digitalWrite(sensA, sens);
  analogWrite(moteurA, pwm);
  digitalWrite(sensB, sens);
  analogWrite(moteurB, pwm);
}

/*
int pid()
{

      unsigned long now = millis();
      timeChange = (now - lastTime);
      //Compute all the working error variables
      double input = tilt_X;
      double error = reference - input;
      
      Serial.print("      error == ");
      Serial.print(error);
      
      
     
      
      if(ITerm>80) ITerm = 80 ;
      else if (ITerm<-80) ITerm = -80 ; 
      Serial.print("      iterm 222== ");
      Serial.print(ITerm);
      
      ITerm= (ITerm+error*timeChange)/2.0;
      double dInput = (input - lastInput)/timeChange;
      
      Serial.print("      dterm == ");
      Serial.print(dInput);
      speed = Kp * error + ITerm*Ki - Kd * dInput;
      
      speed=constrain(speed,-240,240);
    
      lastInput = input;
      lastTime = now; 
}
*/
int PID() 
{       
    float pitch = tilt_X ;      
    thisTime = millis();
    timeChange = double(thisTime - lastTime);
    
    float error = reference - pitch;

    float pTerm = Kp * error;
    iTerm = (iTerm + error * timeChange)/2.0;
    double integrale ; 
    integrale = Ki * iTerm ;
    integrale =constrain(integrale,-100,100); //action antiwindup 
    float dTerm = Kd * (error - last_error); // timeChange; 
 
    lastpitch = pitch;
    lastTime = thisTime;    
    
    float PIDValue = pTerm + integrale - dTerm;
    if(PIDValue>240) PIDValue=240;
    else if(PIDValue< -240) PIDValue = -240;
    //constrain(PIDValue,-240,240);
     
    last_error = error  ; 
    speed=PIDValue;
    //return int(PIDValue);
}

double absolute(double x)
{
  return (x > 0) ? x : x * (-1.0);
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


