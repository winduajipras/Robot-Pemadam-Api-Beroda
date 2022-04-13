#include <NewPing.h>
#include <Pixy.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Tpa81.h>

//---------------------shortchut----------------------//
boolean jump1=false;
boolean jump2=false;
boolean jump3=false;
boolean jump4=false;
boolean jump5=false;
boolean jump6=false;
boolean jump7=false;
int putar;

//---------------------proximity----------------------//
#define proxka1 A10
#define proxka2 A11
#define proxki1 A12
#define proxki2 A13

int proxkidep;
int proxkibel;
int proxkadep;
int proxkabel;

//-----------------------servo------------------------//
Servo myservo;
Servo myservo1;
Servo myservo2;

//-----------------------tpa-------------------------//
Tpa81 thermo = Tpa81(0);
int batasSuhu = 100;

//--------------------ultrasonik------------------------//
#define trig1  28
#define echo1  30
#define trig2  32
#define echo2  34
#define trig3  36
#define echo3  38
#define trig4  40
#define echo4  42
#define trig5  44
#define echo5  46
#define jarakMaksimum 35

NewPing sonar[3] = {     // Sensor object array.
  NewPing(trig1, echo1, jarakMaksimum),  // us_kananDepan
  NewPing(trig2, echo2, jarakMaksimum),  // us_kananSerong 
  NewPing(trig3, echo3, jarakMaksimum),  // us_depan        
};

NewPing sonar1[3] = {     // Sensor object array.
  NewPing(trig5, echo5, jarakMaksimum),  // us_kananDepan
  NewPing(trig4, echo4, jarakMaksimum),  // us_kananSerong 
  NewPing(trig3, echo3, jarakMaksimum),  // us_depan        
};

NewPing sonar2[3] = {     // Sensor object array.
  NewPing(trig2, echo2, jarakMaksimum),  // us_kananDepan
  NewPing(trig3, echo3, jarakMaksimum),  // us_kananSerong 
  NewPing(trig4, echo4, jarakMaksimum),  // us_depan        
};

NewPing sonar3[5] = {     // Sensor object array.
  NewPing(trig1, echo1, jarakMaksimum),  // us_kananDepan
  NewPing(trig3, echo3, jarakMaksimum),  // us_kananSerong  
  NewPing(trig5, echo5, jarakMaksimum),  // us_kananDepan
  NewPing(trig2, echo2, jarakMaksimum),  // us_kananDepan 
  NewPing(trig4, echo4, jarakMaksimum),  // us_depan      
};

int cm[3];

//------------------------IR--------------------------//
#define irDepan 47
#define irKanan 45
#define irKiri 49
#define irBelakangKiri 51
#define irBelakangKanan 53
char irDep, irKa, irKi, irBki, irBka;

//---------------------------PID---------------------------//
int error, last_error;
int outputKanan, outputKiri;
int Kp=3, Kd=12;          //3 12
int set_pwm=120, top_speed=120;
int maks=255;

//-------------------------motor----------------------------//
int b;
const int driverKa_fbPin = A14;
const int driverKa_in1Pin = 22;
const int driverKa_in2Pin = 23;
const int driverKa_pwmPin = 3;
const int driverKa_enPin = 25;
const int driverKi_fbPin = A15;
const int driverKi_in1Pin = 31;
const int driverKi_in2Pin = 29;
const int driverKi_pwmPin = 2;
const int driverKi_enPin = 27;
const int arrayDriverMotor[10] = {
  driverKi_fbPin,
  driverKa_fbPin,
  driverKi_in1Pin,
  driverKi_in2Pin,
  driverKi_pwmPin,
  driverKi_enPin,
  driverKa_in1Pin,
  driverKa_in2Pin,
  driverKa_pwmPin,
  driverKa_enPin};
  
//===============================================================================//
Pixy pixy;
void camera(){
  int pos=100;
  boolean kanan=true;
  static int i=0;
  uint16_t blocks;
  int x, y;
  jump3=false;
  while(1){
    if(kanan){
      blocks = pixy.getBlocks();
      if(blocks){
        if(pos<100) jump7=true;
        else if(pos>=100) jump6=true; 
        i++;
        if(i%5==0){
          jump5=true;
          Serial.print("api");
          Serial.write(4);
          Serial1.write(4);
          delay(6000);
          break;
        }
      }
      pos += 3;
      myservo2.write(pos);
      if (pos > 185) kanan=false;
      delay(50); 
    }
    else if(!kanan){
      blocks = pixy.getBlocks();
      if(blocks){
        if(pos<100) jump7=true;
        else if(pos>=100) jump6=true; 
        i++;
        if(i%5==0){
          jump5=true;
          Serial.print("api");
          Serial.write(4);
          Serial1.write(4);
          delay(6000);
          break;
        }
      }
      pos -= 3;
      myservo2.write(pos);
      if (pos < 15){
        kanan=true;
        break;
      }
      delay(50);
    }
  }
  myservo2.write(100);
  delay(200);
}

void scanTpa(){
  int i;
  unsigned char reading[8];
  int pos=100;
  boolean kanan=true;
  while(1){
    if (kanan){
      i = thermo.getData(reading);
      for(int k=0;k<8;k++){
        Serial.print(reading[k]);
        if((int)reading[k]>80){
          myservo2.write(pos);
          rem(100);
          Serial.write(3);
          Serial1.write(3);
          delay(14000);
          jump2=true;
          goto keluar;
        }
      }
      pos += 3;
      myservo2.write(pos);
      if (pos > 170) kanan=false;
      delay(50); 
    }
    else if (!kanan){
      i = thermo.getData(reading);
      for(int k=0;k<8;k++){
        Serial.print(reading[k]);
        if((int)reading[k]>80){
          myservo2.write(pos);
          Serial.write(3);
          Serial1.write(3);
          rem(100);
          delay(7000);
          jump2=true;
          goto keluar;
        }
      }
      pos -= 3;
      myservo2.write(pos);
      if (pos < 30){
        kanan=true;
        break;
      }
      delay(50);
    }
  }
  keluar:;
  myservo2.write(100);          
}

void scanUltrasonik(int mode){
  for(int i=0; i<3; i++){
    if (mode==0) cm[i]=sonar[i].ping_cm();
    else if (mode==1) cm[i]=sonar1[i].ping_cm();
    else if (mode==2) cm[i]=sonar2[i].ping_cm();
    if(cm[i]==0) cm[i]=35;
    Serial.print(cm[i]);
    delay(2);
    Serial.print(", ");
  }
  delay(30);
}
void scanUltrasonik1(){
  for(int i=0; i<5; i++){
    cm[i]=sonar3[i].ping_cm();
    if(cm[i]==0) cm[i]=35;
    Serial.print(cm[i]);
    delay(3);
    Serial.print(", ");
  }
  delay(10);
  Serial.println();
}

void cariApi(int mode){
  int i, j, height;
  int pos=100;
  unsigned char reading[8];
  uint16_t blocks;
  jump1=false;
  //scanTpa();
  while(1){
      i++;
      if(i>40){
        jump3=true;
        jump1=true;
      }
      blocks = pixy.getBlocks();
      if(blocks){
        height = pixy.blocks[0].height;
        if (height > 27) {
          rem(100);
          scanTpa();
        }
      }
      if(jump1){
        proxkadep = analogRead(proxka1);
        proxkidep = analogRead(proxki1);
        if(proxkidep>400 || proxkadep>400){
          rem(100);
          myservo2.write(100);
          break;
        }
      }
      /*j = thermo.getData(reading);
      for(int k=0;k<8;k++){
        Serial.print(reading[k]);
        if((int)reading[k]>80){
          rem(100);
          Serial.write(3);
          Serial1.write(3);
          delay(14000);
          jump2=true;
          goto keluar;
        }
      }*/
      if (mode==0){
        scanUltrasonik(0);
        nilaiError();
        pid(0);
      }
      if (mode==1){
        scanUltrasonik(1);
        nilaiError();
        pid(1);
      }
      if (jump4){
        i=0;
        jump4=false;
      }
      //keluarkan();
      feedback();
      Serial.print(", ");
      Serial.print(outputKiri);
      Serial.print(", ");
      Serial.print(outputKanan);
      Serial.println();
    }
    jump1=false;
    keluar:;
}

void scanKanan(){
  int i=0, j=0;
  boolean jump=false;
    while(1){
      i++;
      set_pwm=140;
      top_speed=140;
      if(i>=20){
        jump1=true;
        set_pwm=120;
        top_speed=120;
      }
      if(jump1){
        proxkadep = analogRead(proxka1);
        proxkidep = analogRead(proxki1);
        if(proxkidep>400){
          rem(500);
          myservo2.write(100);
           break;
        }
      }
      scanUltrasonik(0);
      nilaiError();
      //if (jump2) goto keluar;
      pid(0);
      keluarkan();
      feedback();
      Serial.print(", ");
      Serial.print(outputKiri);
      Serial.print(", ");
      Serial.print(outputKanan);
      Serial.print(", ");
      Serial.print(i);
      Serial.println();
    }
    jump1=false;
    jump2=false;
}

void scanKiri(){
  int i=0, j=0;
  boolean jump=false;
    while(1){
      i++;
      set_pwm=140;
      top_speed=140;
      if(i>=20){
        jump1=true;
        set_pwm=120;
        top_speed=120;
      }
      if(jump1){
        proxkadep = analogRead(proxka1);
        proxkidep = analogRead(proxki1);
        if(proxkadep>400){
          rem(500);
          myservo2.write(100);
          break;
        }
      }
      scanUltrasonik(1);
      nilaiError();
      //if (jump2) goto keluar;
      pid(1);
      keluarkan();
      feedback();
      Serial.print(", ");
      Serial.print(outputKiri);
      Serial.print(", ");
      Serial.print(outputKanan);
      Serial.print(", ");
      Serial.print(i);
      Serial.println();
    }
    jump1=false;
    jump2=false;
}

void scanKanan1(){
  int i=0, j=0;
  boolean jump=false;
    while(1){
      i++;
      set_pwm=140;
      top_speed=140;
      if(i>=20){
        jump1=true;
        jump3=true;
        set_pwm=120;
        top_speed=120;
      }
      if(jump1){
        proxkabel = analogRead(proxka2);
        proxkibel = analogRead(proxki2);
        if(proxkibel>400 || proxkabel>400){
          rem(500);
          myservo2.write(100);
          break;
        }
      }
      scanUltrasonik(0);
      nilaiError2(0);
      //if (jump2) goto keluar;
      pid(0);
      keluarkan();
      feedback();
      Serial.print(", ");
      Serial.print(outputKiri);
      Serial.print(", ");
      Serial.print(outputKanan);
      Serial.print(", ");
      Serial.print(i);
      Serial.println();
    }
    jump1=false;
    jump2=false;
}

void scanKiri1(){
  int i=0, j=0;
  boolean jump=false;
    while(1){
      i++;
      set_pwm=140;
      top_speed=140;
      if(i>=20){
        jump1=true;
        jump3=true;
        set_pwm=120;
        top_speed=120;
      }
      if(jump1){
        proxkabel = analogRead(proxka2);
        proxkibel = analogRead(proxki2);
        if(proxkibel>400 || proxkabel>400){
          rem(500);
          myservo2.write(100);
          break;
        }
      }
      scanUltrasonik(1);
      nilaiError2(1);
      //if (jump2) goto keluar;
      pid(1);
      keluarkan();
      feedback();
      Serial.print(", ");
      Serial.print(outputKiri);
      Serial.print(", ");
      Serial.print(outputKanan);
      Serial.print(", ");
      Serial.print(i);
      Serial.println();
    }
    jump1=false;
    jump2=false;
}

void cariDinding(){
  irDep = digitalRead(irDep);
  if  (irDep==0){
    while(1){
      irDep = digitalRead(irDep);
      scanUltrasonik(0);
      putarKiri(200,140);
      feedback();
      if (irDep==1 && cm[2]==35) break;
      goto keluar; 
    }
  }
  while(1){
    irKa = digitalRead(irKanan);
    scanUltrasonik(0);
    putarKanan(140,200);
    feedback();
    if (irKa==0) break;
  }
  keluar:;
  while(1){
    scanUltrasonik(0);
    nilaiError();
    pid(0);
    keluarkan();
    feedback();
    proxkabel = analogRead(proxka2);
    proxkibel = analogRead(proxki2);
    if(proxkabel<=300 && proxkibel<=300) break;
  }
}

void keluarRuangan(int mode){
  while(1){
    mundur(130,130);
    proxkadep = analogRead(proxka1);
    proxkidep = analogRead(proxki1);
    if(proxkadep>400 || proxkidep>400) break;
  }
  mundur(130,130);
  delay(600);
  if (mode==0){
    putarKanan(130,150);
    delay(550);
    while(1){
      irKa = digitalRead(irKanan);
      putarKanan(140,200);
      if (irKa==0) break;
    }
  }
  else if (mode==1){
    putarKiri(150,130);
    delay(550);
    while(1){
      irKi = digitalRead(irKiri);
      putarKiri(200,140);
      if (irKi==0) break;
    }
  }
}

void nilaiError(){  
  int ultra=cm[0];
  int ultra1=cm[1];
  int ultra2=cm[2];
  boolean jump, jump2;
  int i;
  set_pwm=120;
  if      (ultra<=7)  error=-5;
  else if (ultra==8)  error=-3;
  else if (ultra==9)  error=-1;
  else if (ultra==10)  error=0; 
  else if (ultra==11)  error=0; 
  else if (ultra==12)  error=1; 
  else if (ultra==13)  error=3; 
  else if (ultra==14)  error=5;
  else if (ultra==15) error=7;
  else if (ultra==16) error=9;
  else if (ultra==17) error=11;
  else if (ultra==18) error=13;
  else if (ultra==19) error=15;
  else if (ultra==20) error=17;
  else if (ultra==21) error=19;
  else if (ultra==21) error=21;
  else if (ultra==21) error=23;
  else if (ultra==22) error=25;
  else if (ultra==35) error=29;
  else if (ultra>=23) error=27;
  
  if      (ultra1<=4 && ultra <30)  error=-23;
  else if (ultra1==6 && ultra <30)  error=-21;
  else if (ultra1==7 && ultra <30)  error=-19;
  else if (ultra1==8 && ultra <30)  error=-17;
  else if (ultra1==9 && ultra <30)  error=-15;
  else if (ultra1==10 && ultra <30)  error=-13;
  else if (ultra1==11 && ultra <30)  error=-9;
  else if (ultra1==12 && ultra <30)  error=-7;
  else if (ultra1==13 && ultra <30)  error=-5;
  else if (ultra1==14 && ultra <30)  error=-3;
  else if (ultra1==15 && ultra <30)  error=-1;

  if (ultra<=22||ultra<35){
    if(ultra2==20){set_pwm =-30; error=-25;}
    else if(ultra2==19){set_pwm =-30; error=-27;}
    else if(ultra2==18){set_pwm =-30; error=-30;}
    else if(ultra2==17){set_pwm =-30; error=-33;}
    else if(ultra2==16){set_pwm =-30; error=-35;}
    else if(ultra2==15){set_pwm =-30; error=-37;}
    else if(ultra2==14){set_pwm =-30; error=-40;}
    else if(ultra2==13){set_pwm =-30; error=-43;}
    else if(ultra2==12){set_pwm =-30; error=-45;}
    else if(ultra2==11){set_pwm =-30; error=-47;}
    else if(ultra2==10){set_pwm =-30; error=-50;}
  }
}

void nilaiError1(){
  int ultra=cm[0];
  int ultra1=cm[1];
  int ultra2=cm[2];
  boolean jump, jump2;
  int i;
  set_pwm=120;
  if      (ultra<=7)  error=-5;
  else if (ultra==8)  error=-3;
  else if (ultra==9)  error=-1;
  else if (ultra==10)  error=0; 
  else if (ultra==11)  error=0; 
  else if (ultra==12)  error=1; 
  else if (ultra==13)  error=3; 
  else if (ultra==14)  error=5;
  else if (ultra==15) error=7;
  else if (ultra==16) error=9;
  else if (ultra==17) error=11;
  else if (ultra==18) error=13;
  else if (ultra==19) error=15;
  else if (ultra==20) error=17;
  else if (ultra==21) error=19;
  else if (ultra==21) error=21;
  else if (ultra==21) error=23;
  else if (ultra==22) error=25;
  else if (ultra==35) error=29;
  else if (ultra>=23) error=27;
  
  if      (ultra1<=4 && ultra <30)  error=-23;
  else if (ultra1==6 && ultra <30)  error=-21;
  else if (ultra1==7 && ultra <30)  error=-19;
  else if (ultra1==8 && ultra <30)  error=-17;
  else if (ultra1==9 && ultra <30)  error=-15;
  else if (ultra1==10 && ultra <30)  error=-13;
  else if (ultra1==11 && ultra <30)  error=-9;
  else if (ultra1==12 && ultra <30)  error=-7;
  else if (ultra1==13 && ultra <30)  error=-5;
  else if (ultra1==14 && ultra <30)  error=-3;
  else if (ultra1==15 && ultra <30)  error=-1;

  //irDep = analogRead(irDepan);
  if (ultra<35){
    if(ultra2==13){
      if(jump3){
        rem(100);
        scanTpa();
        jump3=false;
        jump4=true;
      }
      set_pwm =-30; error=-25;
    }
    else if(ultra2==12){
      if(jump3){
        rem(100);
        scanTpa();
        jump3=false;
        jump4=true;
      }set_pwm =-30; error=-27;}
    else if(ultra2==11){
      if(jump3){
        rem(100);
        scanTpa();
        jump3=false;
        jump4=true;
      }set_pwm =-30; error=-30;}
    else if(ultra2==10){
      if(jump3){
        rem(100);
        scanTpa();
        jump3=false;
        jump4=true;
      }set_pwm =-30; error=-33;}
    else if(ultra2==9){
      if(jump3){
        rem(100);
        scanTpa();
        jump3=false;
        jump4=true;
      }set_pwm =-30; error=-35;}
    else if(ultra2==8){set_pwm =-30; error=-37;}
    else if(ultra2==7){set_pwm =-30; error=-40;}
    else if(ultra2==6){set_pwm =-30; error=-43;}
    else if(ultra2==5){set_pwm =-30; error=-45;}
    else if(ultra2==4){set_pwm =-30; error=-47;}
    else if(ultra2==3){set_pwm =-30; error=-50;}
  }
}

void nilaiError2(int mode){
  int ultra=cm[0];
  int ultra1=cm[1];
  int ultra2=cm[2];
  boolean jump, jump2;
  int i;
  set_pwm=120;
  if      (ultra<=7)  error=-5;
  else if (ultra==8)  error=-3;
  else if (ultra==9)  error=-1;
  else if (ultra==10)  error=0; 
  else if (ultra==11)  error=0; 
  else if (ultra==12)  error=1; 
  else if (ultra==13)  error=3; 
  else if (ultra==14)  error=5;
  else if (ultra==15) error=7;
  else if (ultra==16) error=9;
  else if (ultra==17) error=11;
  else if (ultra==18) error=13;
  else if (ultra==19) error=15;
  else if (ultra==20) error=17;
  else if (ultra==21) error=19;
  else if (ultra==21) error=21;
  else if (ultra==21) error=23;
  else if (ultra==22) error=25;
  else if (ultra==35) error=29;
  else if (ultra>=23) error=27;
  
  if      (ultra1<=4 && ultra <30)  error=-23;
  else if (ultra1==6 && ultra <30)  error=-21;
  else if (ultra1==7 && ultra <30)  error=-19;
  else if (ultra1==8 && ultra <30)  error=-17;
  else if (ultra1==9 && ultra <30)  error=-15;
  else if (ultra1==10 && ultra <30)  error=-13;
  else if (ultra1==11 && ultra <30)  error=-9;
  else if (ultra1==12 && ultra <30)  error=-7;
  else if (ultra1==13 && ultra <30)  error=-5;
  else if (ultra1==14 && ultra <30)  error=-3;
  else if (ultra1==15 && ultra <30)  error=-1;

  irDep = analogRead(irDepan);
  if (ultra<35){
    if(ultra2<=18){
      if(jump3){
        rem(100);
        if (mode==0){
          putarKiri(200,140);
          delay(300);
        }
        else if (mode==1){
          putarKanan(140,200);
          delay(300);
        }
        jump3=false;
        jump4=true;
      }
    }
  }
}

void pid(int mode)                                                                                               
{  
  if (mode==0){
    outputKanan=(int)((-error* Kp) +set_pwm)-((error-last_error)* Kd);
    outputKiri=(int)(( error* Kp) +set_pwm)+((error-last_error )* Kd);
  }
  else if (mode=1){
    outputKiri=(int)((-error* Kp) +set_pwm)-((error-last_error)* Kd);
    outputKanan=(int)(( error* Kp) +set_pwm)+((error-last_error )* Kd);
  } 
  last_error=error;
}


void keluarkan()
{                                                                        
         if (outputKiri > maks){    outputKiri =maks;         digitalWrite(driverKi_enPin, HIGH); digitalWrite(driverKi_in1Pin, LOW); digitalWrite(driverKi_in2Pin, HIGH); analogWrite(driverKi_pwmPin, outputKiri); Serial.print("maju, "); b=analogRead(driverKi_fbPin); Serial.print(analogRead(driverKi_fbPin));}
    else if (outputKiri ==set_pwm){ outputKiri =top_speed;    digitalWrite(driverKi_enPin, HIGH); digitalWrite(driverKi_in1Pin, LOW); digitalWrite(driverKi_in2Pin, HIGH); analogWrite(driverKi_pwmPin, outputKiri); Serial.print("maju, "); b=analogRead(driverKi_fbPin); Serial.print(analogRead(driverKi_fbPin));}
    else if (outputKiri >=0) {      outputKiri =outputKiri;   digitalWrite(driverKi_enPin, HIGH); digitalWrite(driverKi_in1Pin, LOW); digitalWrite(driverKi_in2Pin, HIGH); analogWrite(driverKi_pwmPin, outputKiri); Serial.print("maju, "); b=analogRead(driverKi_fbPin); Serial.print(analogRead(driverKi_fbPin));}
    else if (outputKiri <-maks) {   outputKiri =maks;         digitalWrite(driverKi_enPin, HIGH); digitalWrite(driverKi_in1Pin, HIGH);digitalWrite(driverKi_in2Pin, LOW);  analogWrite(driverKi_pwmPin, outputKiri); Serial.print("mundur, ");b=analogRead(driverKi_fbPin); Serial.print(analogRead(driverKi_fbPin));}
    else if (outputKiri <0) {       outputKiri =(-outputKiri);digitalWrite(driverKi_enPin, HIGH); digitalWrite(driverKi_in1Pin, HIGH);digitalWrite(driverKi_in2Pin, LOW);  analogWrite(driverKi_pwmPin, outputKiri); Serial.print("mundur, ");b=analogRead(driverKi_fbPin); Serial.print(analogRead(driverKi_fbPin));}

         if (outputKanan >maks) {   outputKanan =maks;        digitalWrite(driverKa_enPin, HIGH); digitalWrite(driverKa_in1Pin, LOW); digitalWrite(driverKa_in2Pin, HIGH); analogWrite(driverKa_pwmPin, outputKanan);Serial.print(", maju, ");b=analogRead(driverKa_fbPin); Serial.print(analogRead(driverKa_fbPin));}  
    else if (outputKanan ==set_pwm){outputKanan =top_speed;   digitalWrite(driverKa_enPin, HIGH); digitalWrite(driverKa_in1Pin, LOW); digitalWrite(driverKa_in2Pin, HIGH); analogWrite(driverKa_pwmPin, outputKanan);Serial.print(", maju, ");b=analogRead(driverKa_fbPin); Serial.print(analogRead(driverKa_fbPin));}
    else if (outputKanan >=0) {     outputKanan =outputKanan; digitalWrite(driverKa_enPin, HIGH); digitalWrite(driverKa_in1Pin, LOW); digitalWrite(driverKa_in2Pin, HIGH); analogWrite(driverKa_pwmPin, outputKanan);Serial.print(", maju, ");b=analogRead(driverKa_fbPin); Serial.print(analogRead(driverKa_fbPin));}
    else if (outputKanan <-maks) {  outputKanan = maks;       digitalWrite(driverKa_enPin, HIGH); digitalWrite(driverKa_in1Pin, HIGH);digitalWrite(driverKa_in2Pin, LOW);  analogWrite(driverKa_pwmPin, outputKanan);Serial.print(", mundur, ");b=analogRead(driverKa_fbPin); Serial.print(analogRead(driverKa_fbPin));}
    else if (outputKanan <0) {      outputKanan =(-outputKanan);digitalWrite(driverKa_enPin, HIGH);digitalWrite(driverKa_in1Pin, HIGH);digitalWrite(driverKa_in2Pin, LOW); analogWrite(driverKa_pwmPin, outputKanan);Serial.print(", mundur, ");b=analogRead(driverKa_fbPin); Serial.print(analogRead(driverKa_fbPin));}
}

void feedback(){
  if(b>90 && cm[0]<2 || cm[1]<3 || cm[2]<5){
    mundur(130,130);
    delay(400);
  }
}
void feedback1(){
  if(b>90 && cm[0]<2 || cm[1]<3 || cm[2]<3 || cm[3]<3 ||cm[4]<2){
    mundur(130,130);
    delay(400);
  }
}

void maju(unsigned int kiri, unsigned int kanan)
{
  digitalWrite(driverKi_enPin, HIGH);
  digitalWrite(driverKi_in1Pin, LOW);
  digitalWrite(driverKi_in2Pin, HIGH);
  digitalWrite(driverKa_enPin, HIGH);
  digitalWrite(driverKa_in1Pin, LOW);
  digitalWrite(driverKa_in2Pin, HIGH);
  b = analogRead(driverKi_fbPin);
  b = analogRead(driverKa_fbPin);
  analogWrite(driverKi_pwmPin, kiri);
  analogWrite(driverKa_pwmPin, kanan);
}

void mundur(unsigned int kiri, unsigned int kanan)
{
  digitalWrite(driverKi_enPin, HIGH);
  digitalWrite(driverKi_in1Pin, HIGH);
  digitalWrite(driverKi_in2Pin, LOW);
  digitalWrite(driverKa_enPin, HIGH);
  digitalWrite(driverKa_in1Pin, HIGH);
  digitalWrite(driverKa_in2Pin, LOW);
  b = analogRead(driverKi_fbPin);
  b = analogRead(driverKa_fbPin);
  analogWrite(driverKi_pwmPin, kiri);
  analogWrite(driverKa_pwmPin, kanan);
}

void putarKanan(unsigned int kiri, unsigned int kanan)
{
  digitalWrite(driverKi_enPin, HIGH);
  digitalWrite(driverKi_in1Pin, LOW);
  digitalWrite(driverKi_in2Pin, HIGH);
  digitalWrite(driverKa_enPin, HIGH);
  digitalWrite(driverKa_in1Pin, HIGH);
  digitalWrite(driverKa_in2Pin, LOW);
  b = analogRead(driverKi_fbPin);
  b = analogRead(driverKa_fbPin);
  analogWrite(driverKi_pwmPin, kiri);
  analogWrite(driverKa_pwmPin, kanan);
}

void putarKiri(unsigned int kiri, unsigned int kanan)
{
  digitalWrite(driverKi_enPin, HIGH);
  digitalWrite(driverKi_in1Pin, HIGH);
  digitalWrite(driverKi_in2Pin, LOW);
  digitalWrite(driverKa_enPin, HIGH);
  digitalWrite(driverKa_in1Pin, LOW);
  digitalWrite(driverKa_in2Pin, HIGH);
  analogRead(driverKi_fbPin);
  analogRead(driverKa_fbPin);
  analogWrite(driverKi_pwmPin, kiri);
  analogWrite(driverKa_pwmPin, kanan);
}

void rem(unsigned int jeda)
{
  digitalWrite(driverKi_enPin, HIGH);
  digitalWrite(driverKi_in1Pin, HIGH);
  digitalWrite(driverKi_in2Pin, LOW);
  digitalWrite(driverKa_enPin, HIGH);
  digitalWrite(driverKa_in1Pin, HIGH);
  digitalWrite(driverKa_in2Pin, LOW);
  b = analogRead(driverKi_fbPin);
  b = analogRead(driverKa_fbPin);
  analogWrite(driverKi_pwmPin, 150);
  analogWrite(driverKa_pwmPin, 150);
  delay(50);
  digitalWrite(driverKi_in1Pin, LOW);
  digitalWrite(driverKa_in1Pin, LOW);
  delay(jeda);
}

void strategi(){
  while(1){
  cariDinding(); 
  //r1
  scanKanan();
  maju(140,120);
  delay(200);
  rem(100);
  camera();
  if (!jump5){
    keluarRuangan(0);
    goto keluar;
  }
  cariApi(0);
  if(jump2){
    scanKanan();
    scanKiri();
  }
  keluar:;
  //r2
  scanKanan();
  maju(140,120);
  delay(200);
  rem(100);
  camera();
  if (!jump5){
    keluarRuangan(0);
    goto keluar1;
  }
  cariApi(1);
  if(jump2){
    scanKiri();
    scanKiri();
    keluarRuangan(0);
    scanKiri();
  }
  keluar1:;
  //r3
  scanKanan();
  camera();
  if (!jump5){
    //keluarRuangan(1);
    goto keluar2;
  }
  if (jump6){
    cariApi(0);
    jump6=false;
  }
  else if(jump7){
    cariApi(1);
    jump7=false;
  }
  if (jump2){
    scanKanan();
    scanKanan1();
    keluarRuangan(1);
    scanKanan();
  }
  keluar2:;
  scanKanan();
  scanKanan1();
  //r4
  camera();
  if (!jump5){
    keluarRuangan(0);
    goto keluar3;
  }
  cariApi(1);
  if (jump2){
    scanKiri();
    scanKanan();  
  }
  keluar3:;
  scanKanan(); 
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  Serial1.begin(115200);

  myservo.attach(4);
  myservo1.attach(5);
  myservo2.attach(6);
  
  for (int i=2; i<10; i++){
    pinMode(arrayDriverMotor[i], OUTPUT);
  }
  for (int i=0; i<2; i++) {
    pinMode(arrayDriverMotor[i], INPUT);
  }
  pixy.init();
}

void loop() {
  if (Serial1.available()){
    int inByte = Serial1.read();
    Serial.println(inByte);
    if(inByte == 1) strategi();
  }
  /*static int i=0;
  int j;
  uint16_t blocks;
  char buf[32];

  blocks=pixy.getBlocks();
  if(blocks){
    i++;
    if(i%5==0){
      Serial.write(2);
      Serial1.write(2);
    }
  }*/
}
