#include<Wire.h>
#include "Kalman.h"
int mpu_addr=0x68;
int16_t AcY,AcZ,GyX;
uint32_t timer;
int i ;
double tilt_X,tilt_offset; //These are the angles in the complementary filter
double reference=-2.4 ,output_PID;
double errSum,lasErr;
unsigned long now,lastErr ;
int moteurA=10,moteurB=11;
int sensA=12,sensB=13;
Kalman kalmanX;
double last_error ;
////////////////
#define STD_LOOP_TIME 10
float Kp = 7;                   // (P)roportional Tuning Parameter
float Ki = 0;                   // (I)ntegral Tuning Parameter        
float Kd = 7;                  // (D)erivative Tuning Parameter       
float lastpitch;                // Keeps track of error over time
float iTerm;                    // Used to accumalate error (intergral)
float targetAngle = 0;          // Can be adjusted according to centre of gravity 
int timeGoneBy = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
////////////// 
int port_kp = 3,port_kd = 2,port_ki = 1;
double thisTime = 0;
double lastTime = 0;
double timeChange ; 
#define degconvert 57.2957786 //there are like 57 degrees in a radian.
#define DT 25 

void setup() 
{
  // put your setup code here, to run once:
  pinMode(moteurA,OUTPUT);
  pinMode(moteurB,OUTPUT);
  pinMode(sensA,OUTPUT);
  pinMode(sensB,OUTPUT);
  
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(1000);
  
  timer = micros();
}

void loop() 
{

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr,4);
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Wire.endTransmission(true);
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr,2);
  GyX=Wire.read()<<8|Wire.read();

  /////// potentiometre//////
  Kp = analogRead(port_kp);
  Kd = analogRead(port_kd);
  Ki = analogRead(port_ki);
  Kp = map(Kp,0,1024,0,40);
  Kd =map(Kd,0,1024,0,500);;
  Ki = 0.0001 * Ki;
  Serial.print("kp ==  ");
  Serial.print(Kp);
  Serial.print("   kd ==  ");
  Serial.print(Kd);
  Serial.print("   ki ==  ");
  Serial.print(Ki);
  ///////////////////////////
  Serial.print("    tilt==    ");
  Serial.print(tilt_X);
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  //calculer l'angle a l'aide d'un filtre complementaire
  tilt_X = Complementary_Filter(AcY,AcZ,GyX,dt);
  //int sens = (tilt_X >0 )? 1: 0 ;
  //Serial.println(tilt_X);
  
  int pid=PID(tilt_X);
  output_PID =  absolute(pid); 
  Serial.print("   PID     ");
  Serial.println(output_PID);
  int sens = (tilt_X>0) ? 1 : 0 ;
  //controlling PWM in [0..255]
  output_PID = constrain(output_PID,0,240); 
/*  Serial.print(output_PID);
  Serial.print("     ********      ");
  Serial.print(sens);
  Serial.print("     ********      ");
  Serial.println(tilt_X);
  */
  tourne_moteur(sens,output_PID);
  delay(5);
 
}

void tourne_moteur(int sens,int pwm)
{
  digitalWrite(sensA,sens);
  analogWrite(moteurA,pwm);
  digitalWrite(sensB,sens);
  analogWrite(moteurB,pwm);
}

int PID(float pitch) 
{            
 
    thisTime = millis();
    timeChange = double(thisTime - lastTime);
    
    float error = targetAngle - pitch;

    float pTerm = Kp * error;
    iTerm = (iTerm + error * DT)/2;
    double integrale ; 
    integrale = Ki * iTerm ;
    integrale =constrain(integrale,-100,100); //action antiwindup 
    float dTerm = Kd * (error - last_error) / DT; 
 
    lastpitch = pitch;
    lastTime = thisTime;
double D=(error - last_error) / DT; 
Serial.print("    pTerm == ");
Serial.print(error);
Serial.print("    iTerm == ");
Serial.print(iTerm);
Serial.print("    dTerm == ");
Serial.print(D);

    
    float PIDValue = pTerm + integrale - dTerm;
    if (PIDValue > 240) PIDValue = 240;
    else if (PIDValue < -240) PIDValue = -240; 
    last_error = error  ; 
    return int(PIDValue);
}


double absolute(double x)
{
  return (x> 0) ? x :x*(-1.0);
}

double Complementary_Filter(double AcY,double AcZ,double GyX,double dt)
{
  //angle returned by the accelerometer
  double roll = atan2(AcY, AcZ)*degconvert;
  //converting velocity from raw data to deg/seconde, we need to divide by 131 (see data sheet)
  double gyroXrate = GyX/131.0;
  /*
  * tilt= 0.95 * (tilt + gyro* dt) + 0.05 * x_acc
  * le filtre est composé d'un filtre passe bas et un filtre passe haut
  * on prend toujourd 0.95 de la valeur retrouner par le gyro pour eleminer
  * la constante de l'integration
  * on prend le reste(1-0.95 = 0.05) pour quand elimine la variation brusque 
  * de l'accelerometre a court terme
  * donc si on a une variation uniforme on prend la valeur retourner par le gyro
  * et si on a une variation accélerer et a long terme (prend un peut du temp) on prend la valeur 
  * retourner par l'acceleromtre
  * The accelerometer data is reliable only on the long term, so a "low pass" filter has to be used
  * The gyroscope data is reliable only on the short term, as it starts to drift on the long term.
  */
  return kalmanX.getAngle(roll,gyroXrate,dt);
  //return ( 0.95 * (tilt_X + gyroXrate * dt) + 0.05 * (roll) ); 
}


