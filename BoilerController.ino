// DS18B20
#include <OneWire.h>
#define OW_PIN 7
#define NUM_DS18B20 1
OneWire ds(OW_PIN);
byte owAddr[NUM_DS18B20][8], owData[NUM_DS18B20][12];
bool gettingTemp=false;
unsigned long readTempTimer=0;

// Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
#define DEGREE_SYMBOL char(247)
Adafruit_SSD1306 display(OLED_RESET);

// PID
#include <PID_v1.h>
#define RELAY_PIN 6
double setPoint, input, output;
//double Kp=2, Ki=5, Kd=1;
//int windowSize = 5000; // ms
#define Kp 2
#define Ki 5
#define Kd 1
#define windowSize 5000
unsigned long windowStartTime;
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  // Setup Serial
  Serial.begin(115200);
  while(!Serial) continue;

  // Setup DS18B20
  for(int i=0; i<NUM_DS18B20; i++){
    if(!ds.search(owAddr[i])){
      Serial.print("error,DS18B20,nodevice;");
      return;
    }
  
    if(OneWire::crc8(owAddr[i],7)!=owAddr[i][7]){
      Serial.print("error,DS18B20,crcfail;");
      return;
    } 
  
    if(owAddr[i][0]!=0x28){
      Serial.print("error,DS18B20,notDS18B20;");
      return;
    }  
  }  
  input=0;

  // Setup Display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(3);
  display.display();
  delay(2000);
    
  // Setup PID
  pinMode(RELAY_PIN, OUTPUT);
  windowStartTime = millis();
  setPoint = 67.0; // °C
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  if(!gettingTemp) startReadTemp(0);
  else if(gettingTemp && millis()>=readTempTimer) input=endReadTemp(0);

  myPID.Compute();

  if ((millis() - windowStartTime) > windowSize){
    windowStartTime += windowSize;
    Serial.print(input);
    Serial.print(",");
    Serial.println(output);
    displayData(input, setPoint);
  }
  if (output < (millis() - windowStartTime))
    digitalWrite(RELAY_PIN, LOW);
  else digitalWrite(RELAY_PIN, HIGH);
}

// DS18B20 Functions
float readTemp(int j){
  ds.reset();
  ds.select(owAddr[j]);
  ds.write(0x44,1);
  
  delay(1000);
  
  int present=ds.reset();
  ds.select(owAddr[j]);
  ds.write(0xBE);
  
  for(int i=0; i<9; i++) owData[j][i]=ds.read(); 

  int16_t raw=(owData[j][1]<<8)|owData[j][0];
  byte cfg=(owData[j][4]&0x60);
  if(cfg==0x00) raw=raw&~7;
  else if(cfg==0x20) raw=raw&~3;
  else if(cfg==0x40) raw=raw&~1;
  return float(raw)/16.;
}

void startReadTemp(int j){
  ds.reset();
  ds.select(owAddr[j]);
  ds.write(0x44,1);
  readTempTimer=millis()+1000;
  gettingTemp=true;
}

double endReadTemp(int j){
  gettingTemp=false;
  int present=ds.reset();
  ds.select(owAddr[j]);
  ds.write(0xBE);
  
  for(int i=0; i<9; i++) owData[j][i]=ds.read(); 

  int16_t raw=(owData[j][1]<<8)|owData[j][0];
  byte cfg=(owData[j][4]&0x60);
  if(cfg==0x00) raw=raw&~7;
  else if(cfg==0x20) raw=raw&~3;
  else if(cfg==0x40) raw=raw&~1;
  return float(raw)/16.;  
}

// Display Functions
void displayData(double input, double setp){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(input);
  display.print(DEGREE_SYMBOL);
  display.print("C");
  display.setCursor(0,32);
  display.print(setp);
  display.print(DEGREE_SYMBOL);
  display.println("C");
  display.display();
}

void displayError(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("ERROR!");
  display.display(); 
}
