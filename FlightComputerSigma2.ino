#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "SPIFFS.h"
#include <PID_v1.h>
#include <MPU6050_light.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define LED_PIN    16
#define LED_COUNT 1


MPU6050 mpu(Wire);
Adafruit_BMP280 bmp;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//Contantes y objeto para el PID
double RW_Setpoint, RW_Input, RW_Output;
PID RW_PID(&RW_Input, &RW_Output, &RW_Setpoint, 2, 1, 1, DIRECT);

//Constantes para la reaction wheel
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
const int motor1Pin1 = 27; 
const int motor1Pin2 = 26; 
const int enable1Pin = 14;
const bool invertirRotacion = false;
int low = LOW;
int high = HIGH;
int dutyCycle = 255; //Velocidad con ledcWrite(pwmChannel, dutyCycle); (Para alante)

//Constantes para mechas
const int mecha1 = 25;
const int mecha2 = 32;
const int mecha3 = 33;

String FileName = "/lecturas.csv";
long timer = 0;
String instantData;
int Status = 0;
float AlturaLanzamiento = 0;
bool hasCopiedFile = false;

void writeFile(fs::FS &fs, String path, String message){
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        return;
    }
    if(!file.print(message)){
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, String path, String message){
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        return;
    }
    if(!file.print(message)){
        Serial.println("- append failed");
    }
    file.close();
}


void controlReactionWheel() {
    RW_Input = mpu.getGyroZ();
    RW_PID.Compute();
    if (RW_Output > 0) {
      digitalWrite(motor1Pin1, low);
      digitalWrite(motor1Pin2, high);
      dutyCycle = RW_Output;
    } else {
      digitalWrite(motor1Pin1, high);
      digitalWrite(motor1Pin2, low);
      dutyCycle = -RW_Output;
    }
    ledcWrite(pwmChannel, dutyCycle);
}

void copyFileToSD(fs::FS &fs, String path){
  File file2 = fs.open(path, FILE_WRITE);
  File file = SPIFFS.open(path);
  while( file.available() ) {
      file2.write( file.read() );
  }
  file2.close();
  file.close();
  hasCopiedFile = true;
  Serial.println("Copied File");
  strip.setPixelColor(0, strip.Color(255,255,255));
  strip.show();
}
void deployParachute() {
  //Funcion de despliegue de paracaidas
  digitalWrite(mecha1, HIGH);
  digitalWrite(mecha2, HIGH);
  digitalWrite(mecha3, HIGH);
  strip.setPixelColor(0, strip.Color(255,0,255));
  strip.show();
  Serial.println("DEPLOYED!");
}

void setup(){
    Serial.begin(9600);
    
    //Led setup
    strip.begin();  
    strip.setBrightness(50);      
    strip.show();   
    //Para mostrar color
    //************************************************//
    //strip.setPixelColor(1, (0,0,0)); //  Set pixel's color (in RAM)
    //strip.show();
    //************************************************//

    //SPIFFS Setup (Memoria interna)
    strip.setPixelColor(0, strip.Color(255,0,0));
    strip.show();
    while(!SPIFFS.begin(true)){
        while (1) delay(10);
    }
    //SD Setup
    strip.setPixelColor(0, strip.Color(0,255,0));
    strip.show();
    if(!SD.begin()){
        strip.setPixelColor(0, strip.Color(255,0,255));
        strip.show();
        Serial.println("ERROR SD");
        while (1) delay(10);
    }
    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE){
        strip.setPixelColor(0, strip.Color(255,0,0));
        strip.show();
        while (1) delay(10);
    }

    int i = 0;
    while(SD.exists(FileName)){
      FileName = "/lecturas"+String(i)+".csv";
      i++;
    }
    
    //MPU6050 Setup
    Wire.begin();
    byte status = mpu.begin();
    while(status!=0){  // stop everything if could not connect to MPU6050
      strip.setPixelColor(0, strip.Color(255,255,0));
      strip.show();
    }
    delay(1000);
    strip.setPixelColor(0, strip.Color(255,255,255));
    strip.show();
    mpu.calcOffsets(true,true); // gyro and accelero
    strip.setPixelColor(0, strip.Color(0,255,0));
    strip.show();

    //BMP Setup
    status = bmp.begin(0x76);
    if (!status) {
      strip.setPixelColor(0, strip.Color(0,0,255));
      strip.show();
      while (1) delay(10);
    }
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    //PID Setup
    RW_PID.SetMode(AUTOMATIC);
    RW_PID.SetOutputLimits(-255,255);
    RW_PID.SetSampleTime(50);
    RW_Setpoint = 0;
    
    //DC motor setup
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    ledcSetup(pwmChannel, freq, resolution);
    ledcAttachPin(enable1Pin, pwmChannel);
    if (invertirRotacion){
      low = HIGH;
      high = LOW;
    }

    //Setup mechas
    pinMode(mecha1, OUTPUT);
    pinMode(mecha2, OUTPUT);
    pinMode(mecha3, OUTPUT);
    digitalWrite(mecha1, LOW);
    digitalWrite(mecha2, LOW);
    digitalWrite(mecha3, LOW);
    
    writeFile(SPIFFS, FileName, "ACCELERO_X,ACCELERO_Y,ACCELERO_Z,GYRO_X,GYRO_Y,GYRO_Z,ACC_ANGLE_X,ACC_ANGLE_Y,ANGLE_X,ANGLE_Y,ANGLE_Z,TEMPERATURE,PREASSURE,HEIGHT,TIME,STATUS, \n");

    //Establezco altura inicial
    AlturaLanzamiento = bmp.readAltitude(1020.03);
    //Todo correcto
    strip.setPixelColor(0, strip.Color(0,255,0)); //  Set pixel's color (in RAM)
    strip.show();
}

void loop(){
    mpu.update();
    controlReactionWheel();
    timer = millis();
    instantData = String(mpu.getAccX()) + "," + String(mpu.getAccY()) + "," + String(mpu.getAccZ()) + 
                  "," + String(mpu.getGyroX()) + "," + String(mpu.getGyroY()) + "," + String(mpu.getGyroZ()) + 
                  "," + String(mpu.getAccAngleX()) + "," + String(mpu.getAccAngleY()) + 
                  "," + String(mpu.getAngleX()) + "," + String(mpu.getAngleY()) + "," + String(mpu.getAngleZ()) + 
                  "," + bmp.readTemperature() + "," + bmp.readPressure() + "," + bmp.readAltitude(1020.03) + 
                  "," + String(timer) + "," + String(Status) + "," + "\n";
    appendFile(SPIFFS, FileName, instantData);
    if((mpu.getAngleX() > 80 || mpu.getAngleY() > 80 || mpu.getAngleX() < -80 || mpu.getAngleY() < -80) && Status == 0){
      Status=1;
      deployParachute();
    }

    if (Status == 1){
      if (abs(bmp.readAltitude(1020.03) - AlturaLanzamiento) < 10 && hasCopiedFile == false) {
        copyFileToSD(SD, FileName);
      }
    }
    
    delay(1);
}
