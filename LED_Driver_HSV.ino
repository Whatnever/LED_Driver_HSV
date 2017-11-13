#include <Wire.h>
#include <extEEPROM.h>
#include <SoftwareSerial.h>
#include <MD_DS1307.h>
#include <Adafruit_PWMServoDriver.h>
#include "LED_Driver_HSV.h"

//##################################
//############## TODO ##############
//##################################
//Change bluetooth commands from checking length to checking for "," (valid amount of variables) instead
//command +eeprom code testing  
//remove Debug Serial out and switch to BluetoothSerial
//save current preset in eeprom and load on setup (position 0)


//###################################
//############## Setup ##############
//###################################
void setup() {
  pinMode(CycleButton,INPUT_PULLUP);
  int s =1;
  for (uint8_t i = 0; i < numStrips; i++ ) {
    colorState[i].h = s;
    s+=800;
    colorState[i].s = 4095;
    colorState[i].v = s;
    colorState[i].flag = false;
    blinkcycle[i] = 2.0;
    fadecycle[i] = 0.7;
    strobocycle[i] = 50;
  }

  for (uint8_t i = 0; i < numStrips; i++ ) {
    statefunc[i] = fade;
  }
  previousMillis = millis();
  Serial.begin(9600);
  Serial.println("16 channel PWM LED Driver test!");

  //EEPROM Setup
  twiClockFreq_t freq = twiClock100kHz;
  byte i2cStat = myEEPROM.begin(freq);
  if ( i2cStat != 0 ) {
    Serial.println("EEPROM connection failed");
  }
  
  //PWM setup
  pwm.begin();
  pwm.setPWMFreq(1000);//1600 before

  //Bluetooth Setup
  //BluetoothSerial.begin(4800);
}

//##################################
//############## Loop ##############
//##################################
void loop() {
  //Approx. Time for Last Loop 
  currentMillis = millis();
  deltaT = currentMillis - previousMillis;
  previousMillis = currentMillis;
  
  //Bluetooth
  processBluetooth();
  
  //Functionpointer as a statemachine to calculate each LED change
  if(!pause){
    for (uint8_t i = 0; i < numStrips; i++ ) {
      statefunc[i](i,deltaT);
      drawRGBStrip(i);
    }
  }
}

//################################################
//########### Function Implementations ###########
//################################################

//Go through H range in HSV
void fade(uint8_t Strip, unsigned long deltaT) {
    colorState[Strip].h += fadecycle[Strip]*deltaT;
    if(colorState[Strip].h >4090){ //only go to 4090 because of rounding errors due to non float hsv to rgb conversion
      colorState[Strip].h = 0;
    }
}

//Go Up and Down in V
void blinken(uint8_t Strip, unsigned long deltaT){
  if(!colorState[Strip].flag){
    colorState[Strip].v -= blinkcycle[Strip]*deltaT;
  }else{
    colorState[Strip].v += blinkcycle[Strip]*deltaT; 
  }
  if(colorState[Strip].v > 10000){
    colorState[Strip].flag = true;
    colorState[Strip].v = 0;
  }
  if(colorState[Strip].v > 4095 && colorState[Strip].v < 10000){
    colorState[Strip].flag = false;
   colorState[Strip].v = 4095;
  }
}

//Idle/this function does literally nothing
void stay(uint8_t Strip, unsigned long deltaT){
}

//Initialize with h = 0 //Fast on/off for extra partey // Color is white
void strobo(uint8_t Strip, unsigned long deltaT){
  if(colorState[Strip].s != 0){
    colorState[Strip].s = 0;
  }
  colorState[Strip].h += deltaT;
  if(colorState[Strip].h > strobocycle[Strip]){
    colorState[Strip].h = 0;
    if(!colorState[Strip].flag){
      colorState[Strip].v = 4095;
      colorState[Strip].flag = true;
    }else{
      colorState[Strip].v = 0;
      colorState[Strip].flag = false;
    }
  }
}

//Communicate change to multiplexer
void drawRGBStrip(uint8_t Strip) {
  if(!off){
    HSV Rgb = hsv2rgb(colorState[Strip]);
    pwm.setPWM(Strip * 3, 0, Rgb.h);
    pwm.setPWM(Strip * 3 + 1, 0,Rgb.s);
    pwm.setPWM(Strip * 3 + 2, 0,Rgb.v);
  }else{
    pwm.setPWM(Strip * 3, 0, 0);
    pwm.setPWM(Strip * 3 + 1, 0,0);
    pwm.setPWM(Strip * 3 + 2, 0,0);
  }
}

//Calculate RGB from HSV and use HSV Struct to pass RGB Values
//Modified Algorithm from  http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
HSV hsv2rgb(HSV hsv){
    HSV rgb;
    long region, remainder, p, q, t;
    if (hsv.s == 0)
    {
        rgb.h = hsv.v;
        rgb.s = hsv.v;
        rgb.v = hsv.v;
        return rgb;
    }
    region = hsv.h / 682L;
    remainder = (hsv.h - (region * 682L)) * 6L; 
    p = (hsv.v * (4095L - hsv.s)) >> 12;
    q = (hsv.v * (4095L - ((hsv.s * remainder) >> 12))) >> 12;
    t = (hsv.v * (4095L - ((hsv.s * (4095L - remainder)) >> 12))) >> 12;
    switch (region)
    {
        case 0:
            rgb.h = hsv.v; rgb.s = t; rgb.v = p;
            break;
        case 1:
            rgb.h = q; rgb.s = hsv.v; rgb.v = p;
            break;
        case 2:
            rgb.h = p; rgb.s = hsv.v; rgb.v = t;
            break;
        case 3:
            rgb.h = p; rgb.s = q; rgb.v = hsv.v;
            break;
        case 4:
            rgb.h = t; rgb.s = p; rgb.v = hsv.v;
            break;
        default:
            rgb.h = hsv.v; rgb.s = p; rgb.v = q;
            break;
    }
    return rgb;
}

//Save all necessary information to EEPROM
void saveToEEPROM(unsigned int pos)
{
  //EEPROM Buffer for 1 Strip
  byte temp[19];
  unsigned int adress = pos + 1; //adress counter //(adress - 19*i) is adress in current iteration //+1 because first bit shows which preset was used last time
  bytefloat bf;
  byteint bi;
  for(uint8_t i = 0;i<numStrips;i++)
  {
    //Enter mode in first byte
    if(statefunc[i] == fade){
      temp[(adress - 19*i)] == (byte) 1;
    }else if(statefunc[i] == blinken){
      temp[(adress - 19*i)] == (byte) 2;
    }else if(statefunc[i] == stay){
      temp[(adress - 19*i)] == (byte) 3;
    }else if(statefunc[i] == strobo){
      temp[(adress - 19*i)] == (byte) 4;
    }
    //add flag at highest bit of mode byte
    if(colorState[i].flag == true){
      temp[(adress - 19*i)] = temp[adress] | B10000000;
    }
    adress++;
    
    //HSV 2 bytes each
    bi.i[0] = colorState[i].h;
    bi.i[1] = colorState[i].s;
    bi.i[2] = colorState[i].v;
    for(uint8_t j = 0;j < 6;j++){
      temp[(adress - 19*i)] = bi.b[j];
      adress++;
    }
    
    //Cycletimes 4 bytes each
    bf.f[0] = fadecycle[i];
    bf.f[1] = blinkcycle[i];
    bf.f[2] = strobocycle[i];
    for(uint8_t h = 0;h < 6;h++){
      temp[(adress - 19*i)] = bf.b[h];
      adress++;
    }
    byte i2cStat = myEEPROM.write((int)(adress - 19),(byte*) &temp, 19);
    if ( i2cStat != 0 ) {
      //there was a problem
      if ( i2cStat == EEPROM_ADDR_ERR) {
        //bad address
      }
      else {
        //some other I2C error
      }
    }
  }
}

//Load all necessary information from EEPROM
void loadFromEEPROM(unsigned int pos)
{
  byte temp[19];
  unsigned int adress = pos + 1;
  bytefloat bf;
  byteint bi;
  for(uint8_t i = 0;i<numStrips;i++)
  {
    byte i2cStat = myEEPROM.read(adress,(byte*) &temp, 19);
    if ( i2cStat != 0 ) {
      //there was a problem
      if ( i2cStat == EEPROM_ADDR_ERR) {
        //bad address
      }
      else {
        //some other I2C error
      }
    }
    if((temp[0] >> 7) == 1){
      colorState[i].flag = true;
    }else{
      colorState[i].flag = false;
    }
    switch(temp[0] & B01111111)
    {
      case 1:
        statefunc[i] = fade;
        break;
      case 2:
        statefunc[i] = blinken;
        break;
      case 3:
        statefunc[i] = stay;
        break;
      case 4:
        statefunc[i] = strobo;
        break;
      default:
        statefunc[i] = stay;
        break;
    }
    for(uint8_t j=0;j < 6;j++){
      bi.b[j] = temp[1+j];
    }
    colorState[i].h = bi.i[0];
    colorState[i].s = bi.i[1];
    colorState[i].v = bi.i[2];
    for(uint8_t h=0;h < 12;h++){
      bf.b[h] = temp[7+h];
    }
    fadecycle[i] = bf.f[0];
    blinkcycle[i] = bf.f[1];
    strobocycle[i] = bf.f[2];
    
    adress += 19;
  }
}

//Watch for Bluetooth Serial Commands and direct them to be read in CommandX Function
void processBluetooth(){
  if(Serial.available() > 0){
      c = (char) Serial.peek();
      if(c == '<'){
        readFlag = true;
        c = Serial.read();
        Serial.println("Starter Found!");
      }
      if(readFlag){
        if(Serial.available() > 0){
          if(commandChar == '#'){
            commandChar = (char) Serial.read();
            Serial.print("Command Found:");
            Serial.println(commandChar);
          }else{
            switch(commandChar){
              case 'a':
                commandA();
                break;
              case 'b':
                commandB();
                break;
              case 'c':
                commandC();
                break;
              case 'd':
                commandD();
                break;
              case 'e':
                commandE();
                break;
              case 'f':
                commandF();
                break;
              case 'g':
                commandG();
                break;
              case 'h':
                commandH();
                break;
              case 'i':
                commandI();
                break;
              case 'j':
                commandJ();
                break;
              case 'k':
                commandK();
                break;
              case 'l':
                commandL();
                break;
              case 'm':
                commandM();
                break;
              case 'z':
                commandZ();
                break;

            }
          }
        }
      }else{
        while (Serial.available() > 0){
          if(Serial.read() == '<'){
            readFlag = true;
            commandChar = '#';
            c = '#';
            Serial.println("Removed unecessary letters");
            break;
          }
          Serial.println("Clearing Buffer");
        }
      }
   }
}

//Use button to switch presets
void processKeys(){
  if(digitalRead(CycleButton) == LOW){
    while(digitalRead(CycleButton) == LOW){
    }
    bool loop1 = true;
    bool loop2 = true;
    unsigned int slot = 0;
    unsigned long waitcounter = 0;
    while(loop1){
      for (uint8_t i = 0; i < numStrips; i++ ) {
        if(slot == i){
          colorState[i].h = 0;
          colorState[i].s = 4095;
          colorState[i].v = 4095;
        }else{
          colorState[i].h = 0;
          colorState[i].s = 0;
          colorState[i].v = 0;
        }
      }
      waitcounter = millis();
      while(loop2){
        if(digitalRead(CycleButton) == LOW){
          loop2 = false;
          waitcounter = millis();
          slot++; 
          if(slot > 4){
            slot = 0;
          }
        }
        while(digitalRead(CycleButton) == LOW){
        }
        if(millis - waitcounter > 5000){
          loop1 = false;
          loop2 = false;
        }
      }
    }
    loadFromEEPROM((unsigned int) slot * 95);
  }
}

/*
#############################################
############ Bluetooth  Commands ############
#############################################
*/
/*
Command list:
Command parameters are length dependend for the moment
 (first char is command identifier)
 i = 4 chars
 s = 1 chars
 f = 4 chars  
 t = 2 chars
 SET HSV                   "<a,s,i,i,i>"  17     "strip;h;s;v"
 SET MODE                  "<b,s,s>" 4           "strip;mode"
 SET MODE WITH STARTVALUES "<c,s,s,i,i,i>" 19    "strip;mode;h;s;v"
 SET HSV ALL               "<d,i,i,i>" 15        "h;s;v"
 SET MODE ALL              "<e,s>" 2             "mode"
 SET BLINKTIME             "<f,s,f>" 7           "strip;time"
 SET FADETIME              "<g,s,f>" 7           "strip;time"
 SET STROBOTIME            "<h,s,i>" 7           "strip;time"
 LOAD PRESET               "<i,s>" 2             "preset num"
 SAVE PRESET               "<j,s>" 2             "preset num" //0 just forces save
 SET TIME                  "<k,t,t>" 6           "hours;mins"
 SET ON INTERVAL           "<l,t,t,t,t>" 12      "onhour;onmin;offhour;offmin"      //on time = off time => always on
 PAUSE                     "<m,s>" 2             "1:pause/0:unpause"                (for starting synced fades etc)//dont call funcpointer in loop
 SET FLAG                  "<n,s,s> 4            "strip;flag"                       //TODO!
 ON/OFF                    "<z,s>" 2             "1:on/0:off"                       //draw 0,0,0
 */

//SET HSV "<a,s,i,i,i>" "strip;h;s;v"
void commandA(){
  if(Serial.available() > 17){
    int strp,h,s,v;
    strp = Serial.parseInt();
    h = Serial.parseInt();
    s = Serial.parseInt();
    v = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command A
      colorState[strp].h = h;
      colorState[strp].s = s;
      colorState[strp].v = v;
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// SET MODE "<b,s,s>" "strip;mode"
void commandB(){
  if(Serial.available() > 4){
    int strp,mode;
    strp = Serial.parseInt();
    mode = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command B
      switch(mode){
        case 0:
          statefunc[strp] = stay;
          break;
        case 1:
          statefunc[strp] = fade;
          break;
        case 2:
          statefunc[strp] = blinken;
          break;
        case 3:
          statefunc[strp] = strobo;
          break;
      }
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

//SET MODE WITH STARTVALUES "<c,s,s,i,i,i>" "strip;mode;h;s;v"
void commandC(){
  if(Serial.available() > 19){
    int strp,mode,h,s,v;
    strp = Serial.parseInt();
    mode = Serial.parseInt();
    h = Serial.parseInt();
    s = Serial.parseInt();
    v = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command C
      colorState[strp].h = h;
      colorState[strp].s = s;
      colorState[strp].v = v;
      switch(mode){
        case 0:
          statefunc[strp] = stay;
          break;
        case 1:
          statefunc[strp] = fade;
          break;
        case 2:
          statefunc[strp] = blinken;
          break;
        case 3:
          statefunc[strp] = strobo;
          break;
      }
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// SET HSV ALL "<d,i,i,i>" "h;s;v"
void commandD(){
  if(Serial.available() > 15){
    int h,s,v;
    h = Serial.parseInt();
    s = Serial.parseInt();
    v = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command D
      for(uint8_t i = 0;i < numStrips;i++){
        colorState[i].h = h;
        colorState[i].s = s;
        colorState[i].v = v;
      }
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// SET MODE ALL "<e,s>" "mode"
void commandE(){
  if(Serial.available() > 2){
    int mode;
    mode = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command E
      for(uint8_t i = 0;i < numStrips;i++){
        switch(mode){
          case 0:
            statefunc[i] = stay;
            break;
          case 1:
            statefunc[i] = fade;
            break;
          case 2:
            statefunc[i] = blinken;
            break;
          case 3:
            statefunc[i] = strobo;
            break;
        }
      }
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

//SET BLINKTIME "<f,s,f>" "strip;time"
void commandF(){
  if(Serial.available() > 7){
    int strp;
    float tme;
    strp = Serial.parseInt();
    tme = Serial.parseFloat();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command F
      blinkcycle[strp] = tme;
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

//SET FADETIME "<g,s,f>" "strip;time"
void commandG(){
  if(Serial.available() > 7){
    int strp;
    float tme;
    strp = Serial.parseInt();
    tme = Serial.parseFloat();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command G
      fadecycle[strp] = tme;
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

//SET STROBOTIME "<h,s,i>" "strip;time"
void commandH(){
  if(Serial.available() > 7){
    int strp;
    int tme;
    strp = Serial.parseInt();
    tme = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command H
      strobocycle[strp] = (unsigned long) tme;
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// LOAD PRESET "<i,s>" "preset num"
void commandI(){
  if(Serial.available() > 2){
    int num;
    num = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command I
      loadFromEEPROM((unsigned int) num * 95);
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// SAVE PRESET "<j,s>" "preset num" //0 just forces save
void commandJ(){
  if(Serial.available() > 2){
    int num;
    num = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command J
      saveToEEPROM((unsigned int) num * 95);
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// SET TIME "<k,t,t>" "hours;mins"
void commandK(){
  if(Serial.available() > 6){
    int hrs,mins;
    hrs = Serial.parseInt();
    mins = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command K
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// SET ON INTERVAL "<l,t,t,t,t>" "onhour;onmin;offhour;offmin"      //on time = off time => always on
void commandL(){
  if(Serial.available() > 12){
    int onhr,onmin,offhr,offmin;
    onhr = Serial.parseInt();
    onmin = Serial.parseInt();
    offhr = Serial.parseInt();
    offmin = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command L
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// PAUSE "<m,s>" "1:pause/0:unpause" (for starting synced fades etc)//dont call funcpointer in loop
void commandM(){
  if(Serial.available() > 2){
    int pse;
    pse = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command M
      if(pse == 1){
        pause = true;
      }else{
        pause = false;
      }
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

// ON/OFF "<z,s>" "1:on/0:off"  //draw 0,0,0
void commandZ(){
  if(Serial.available() > 2){
    int onoff;
    onoff = Serial.parseInt();
    if(Serial.peek() == '>'){
      Serial.println("Close  Found");
      commandChar = '#';
      c = '#';
      Serial.read();
      readFlag = false;
      //Execute Command Z
      if(onoff == 1){
        off = false;
      }else{
        off = true;
      }
    }else{
      //Well something done fucked up
      commandChar = '#';
      c = '#';
      readFlag = false;
    }
  }  
}

