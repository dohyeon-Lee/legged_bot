float H;
float P;
float D;
float pH;
float a;
float pa;
double rollAcc;
double pitchAcc;
double dx;
double dy;
float angle_x;
float angle_y;
float bump = 10000;
float errer;
float now;
float Kp;
float Kd;        

#include<Wire.h>
const int MPU=0x68;  
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void PD(){
  a=0-(-angle_x)+27;
  Kp=0.99;
  Kd=0.01;
  P=Kp*(pH+a/100);
  D=Kd*((a-pa)/0.005);
  H=P+D;
  pH=P;
  pa=a;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();      
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU);  
  Wire.write(0x3B);              
  Wire.endTransmission(false);    
  Wire.requestFrom(MPU,14,true);  
  AcX=Wire.read()<<8|Wire.read();      
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
 
  
  rollAcc = atan2(AcY,AcZ)*180/PI;
  dx=GyX/131.;
  angle_x=(0.89*(angle_x+(dx*0.001)))+(0.11*rollAcc);

  pitchAcc = atan2(AcX, AcZ)*180/PI;
  dy=GyY/131.;
  angle_y=(0.89*(angle_y+(dy*0.001)))+(0.11*pitchAcc);
  /*Serial.print((int)(angle_x*10000));
  Serial.print(",");
  Serial.println((int)(angle_y*10000));
  */
  Serial.print(angle_x);
  Serial.print(",");
  Serial.print(angle_y);
  Serial.print(".");
  Serial.println(bump);
}
