
#include <Wire.h>
#include<EEPROM.h>
int ch1, ch2, ch3, ch4;

int mp1, mp2, mp3, mp4;
int spd1, spd2, spd3, spd4;
long currTime;
bool last1, last2, last3, last4;
long lastTime1, lastTime2, lastTime3, lastTime4;
int max1,min1,max2,min2,max3,min3,max4,min4;
int mpu = 0x68;

long pulseTimer;
long loopTimer;
bool armed = false;
int gyroX, gyroY,gyroZ;
float calX, calY, calZ;
long accelX, accelY, accelZ;
float measuredYawing, measuredPitching, measuredRolling;
float targetYawing, targetPitching, targetRolling;
float maxYawing = 200;
float maxPitching =150;
float maxRolling = 150;
float throttle;
float integralYaw, integralPitch, integralRoll;

float MAXINTEGRAL = 100;

float errorYaw, errorPitch, errorRoll;
float outputYaw, outputPitch, outputRoll;
float lastErrorYaw,lastErrorPitch,lastErrorRoll;
 
float kpy=.5;
float kiy=0;
float kdy=.01;

float kpp=.7;
float kip=0;
float kdp=.3;

float kpr=.7;
float kir=0;
float kdr=.3;

int count = 0;
float last = 0;
int offset;
int armCount = 0;
int sp1,sp2,sp3,sp4;
bool armCheck;
 // how much to rely on new gyro data, 1 would be entirely new data 0 would not change value at all

void setup() {
  pinMode(13, OUTPUT);
  Wire.begin();
  Serial.begin(115200);
  DDRB |= 0b00011110;
  PCICR |= 0b00000100;
  PCMSK2 |= 0b01111000;
  spd1=1000;
  spd2=1000;
  spd3=1000;
  spd4=1000;
  measuredYawing =0;
  measuredPitching=0;
  measuredRolling=0;
  Wire.beginTransmission(mpu);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  EEPROM.get(0, calX);
  EEPROM.get(4, calY);
  EEPROM.get(8, calZ);
  Serial.println("gyro calibration values: "+(String)calX+", "+(String)calY+", "+(String)calZ);
  EEPROM.get(12,min1);
  EEPROM.get(14,max1);
  EEPROM.get(16,min2);
  EEPROM.get(18,max2);
  EEPROM.get(20,min3);
  EEPROM.get(22,max3);
  EEPROM.get(24,min4);
  EEPROM.get(26,max4);
  Serial.println("stick calibration values: ");
  Serial.println("Channel 1: \t"+(String)min1+"\t"+(String)max1);
  Serial.println("Channel 2: \t"+(String)min2+"\t"+(String)max2);
  Serial.println("Channel 3: \t"+(String)min3+"\t"+(String)max3);
  Serial.println("Channel 4: \t"+(String)min4+"\t"+(String)max4);
  EEPROM.get(28,sp1);
  EEPROM.get(30,sp2);
  EEPROM.get(32,sp3);
  EEPROM.get(34,sp4);
  Serial.println("motor calibration values: "+(String)sp1+", "+(String)sp2+", "+(String)sp3+", "+(String)sp4);
  delay(1000);
  loopTimer=micros();
  
}

void loop() {
 
  if(micros()-loopTimer > 4000){
    Serial.println("WOAHHH LONG TIME");
  }
//  
  //Serial.println((String)spd1+"\t"+(String)spd2+"\t"+(String)spd3+"\t"+(String)spd4);
  
  while(micros()-loopTimer < 3900){}
  
//    Serial.println(micros()-loopTimer);
//  Serial.println(offset);
//  
  loopTimer=micros();

  if(armed){
    flightLoop();
  }else{
    convertInput();
    spd1 =1000;
    spd2 = 1000;
    spd3 =1000;
    spd4=1000;
    
    startPulse();
  
    getPosition();
  
    stopPulse();
  }
  
  checkArmed();
  
}


void flightLoop(){
//  Serial.println((String)spd1+"\t"+(String)spd2+"\t"+(String)spd3+"\t"+(String)spd4);

  Serial.println((String)outputYaw+"\t"+(String)outputPitch+"\t"+(String)outputRoll+"\t"+(String)throttle);
//  Serial.println((String)ch1+"\t"+(String)ch2+"\t"+(String)ch3+"\t"+(String)ch4);
//  Serial.println((String)measuredYawing+"\t"+(String)measuredPitching+"\t"+(String)measuredRolling+"\t");
// Serial.println((String)measuredPitching+"\t"+(String)targetPitching);

  if(count>100){
    convertInput();
    
    computeErrors();
    
    setMotorSpeeds();
    
    startPulse();
    
    getPosition();
  
    stopPulse();
    
  }else{
    count++;
    spd1 =1000;
    spd2 = 1000;
    spd3 =1000;
    spd4=1000;
    
    startPulse();
  
    getPosition();
  
    stopPulse();
  }

}
void checkArmed(){
  if(mp1 < 1100 && mp2 >1900 && mp3 < 1900 && mp4 < 1100){
    if(armCheck){
      armCount++;  
//      Serial.println(armCount);
    }

  }else{
    armCount = 0;
    armCheck = true;
  }
  if (armCheck && armCount > 250){
    armCount = 0;
    armCheck = false;
    armed = !armed;
    if(armed){
      Serial.println("ARMED");
      digitalWrite(13,HIGH);
    }else{
      Serial.println("DISARMED");
      digitalWrite(13,LOW);
    }
   
    
  }
  
}
void startPulse(){
//  Serial.println(micros()-pulseTimer);
  pulseTimer = micros();
  PORTB |= 0b000011110;
}
void setMotorSpeeds(){
  spd1= sp1 + abs(outputYaw - outputRoll - outputPitch + throttle);
  spd2= sp2 + abs(-outputYaw + outputRoll - outputPitch + throttle);
  spd3= sp3 + abs(outputYaw + outputRoll + outputPitch + throttle);
  spd4= sp4 + abs(-outputYaw - outputRoll + outputPitch + throttle);
  if (spd1 > 2000){
    spd1=2000;
  }
  if (spd1 < 1000){
    spd1=1000;
  }
  if (spd2 > 2000){
    spd2=2000;
  }
  if (spd2 < 1000){
    spd2=1000;
  }
  if (spd3 > 2000){
    spd3=2000;
  }
  if (spd3 < 1000){
    spd3=1000;
  }
  if (spd4 > 2000){
    spd4=2000;
  }
  if (spd4 < 1000){
    spd4=1000;
  }
}
void stopPulse(){
//  Serial.println(micros()-pulseTimer);
  while(PORTB & 0b00011110){
      if(micros()-pulseTimer>spd1){
        PORTB &= 0b11111101;
      }
      if(micros()-pulseTimer>spd2){
        PORTB &= 0b11111011;
      }
      if(micros()-pulseTimer>spd3){
        PORTB &= 0b11110111;
      }
      if(micros()-pulseTimer>spd4){
        PORTB &= 0b11101111;
      }
    }
    
}

void getPosition(){
  Wire.beginTransmission(mpu);
    Wire.write(0x43); 
    Wire.endTransmission();
    Wire.requestFrom(mpu,6);
    
    while(Wire.available() < 6);

    gyroX = Wire.read()<<8|Wire.read();                              
    gyroY = Wire.read()<<8|Wire.read();
    gyroZ = Wire.read()<<8|Wire.read();

  gyroX -= calX;
  gyroY -= calY;
  gyroZ -= calZ;

  
 
  
}

void computeErrors(){
    float xDegree = gyroX /65.5;
  float yDegree = gyroY /65.5;
  float zDegree = -gyroZ /65.5;
   measuredYawing = measuredYawing * (.7) + zDegree * .3;
  measuredPitching = measuredPitching * (.7) + yDegree * .3;
  measuredRolling = measuredRolling * (.7) + xDegree * .3;
  errorYaw = targetYawing - measuredYawing;
  outputYaw = kpy*errorYaw+kdy*(lastErrorYaw-errorYaw);
  integralYaw += kiy*errorYaw;
  if (integralYaw > MAXINTEGRAL){
    integralYaw = MAXINTEGRAL;
  }
  if (integralYaw < -MAXINTEGRAL){
    integralYaw = -MAXINTEGRAL;
  }
  outputYaw += integralYaw;
  lastErrorYaw = errorYaw;

  errorPitch = targetPitching - measuredPitching;
  outputPitch = kpp*errorPitch+kdp*(lastErrorPitch-errorPitch);
  integralPitch += kip*errorPitch;
  if (integralPitch > MAXINTEGRAL){
    integralPitch = MAXINTEGRAL;
  }
  if (integralPitch < -MAXINTEGRAL){
    integralPitch = -MAXINTEGRAL;
  }
  outputPitch += integralPitch;
  lastErrorPitch = errorPitch;
  errorRoll = targetRolling - measuredRolling;
  outputRoll = kpr*errorRoll+kdr*(lastErrorRoll-errorRoll);
  integralRoll += kir*errorRoll;
  if (integralRoll > MAXINTEGRAL){
    integralRoll = MAXINTEGRAL;
  }
  if (integralRoll < -MAXINTEGRAL){
    integralRoll = -MAXINTEGRAL;
  }
  outputRoll += integralRoll;
  lastErrorRoll = errorRoll;
}

void convertInput(){
  mp1 = map(ch1, min1, max1,1000,2000);
  mp2 = map(ch2, min2, max2,1000,2000);
  mp3 = map(ch3, min3, max3,1000,2000);
  mp4 = map(ch4, min4, max4,1000,2000);
  targetYawing = maxYawing*(-3+(mp2/500.0));
  targetPitching = maxPitching*(-3+(mp3/500.0));
  targetRolling = maxRolling*(-3+(mp4/500.0));
  throttle = mp1-1000;
}

ISR(PCINT2_vect){
  currTime = micros();
  if(PIND & 0b00001000){
    if(!last1){
      lastTime1 = currTime;
      last1 = true;
    }
  }
  else if(last1){
    ch1 = currTime-lastTime1;
    last1=false;
  }
  
  if(PIND & 0b00010000){
    if(!last2){
      lastTime2 = currTime;
      last2 = true;
    }
  }
  else if(last2){
    ch2 = currTime-lastTime2;
    last2=false;
  }
  
  if(PIND & 0b00100000){
    if(!last3){
      lastTime3 = currTime;
      last3 = true;
    }
  }
  else if(last3){
    ch3 = currTime-lastTime3;
    last3=false;
  }

  if(PIND & 0b01000000){
    if(!last4){
      lastTime4 = currTime;
      last4 = true;
    }
  }
  else if(last4){
    ch4 = currTime-lastTime4;
    last4=false;
  }
}
