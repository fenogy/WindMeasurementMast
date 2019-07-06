#include <Wire.h>

/*I2C address of BMP085 */
#define BMP085_ADDRESS 0x77  

/*Oversampling Setting*/
const unsigned char OSS = 0;  

/*Sensor Calibration values*/
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

/* b5 is calculated in bmp085GetTemperature(...), this variable is also used */
/* in bmp085GetPressure(...), so ...Temperature(...) must be called before */
/*...Pressure(...)*/
long b5; 
short temperature;
long pressure;
int pulseCount = 0;
/*Use these for altitude conversions*/
const float p0 = 101325;     
float altitude;

/*Wind Speed and direction calculations and vars*/
int windSpeed = 0;
int wnd = 0;
String strWindSpeed = "000";
int windDirection = 0;
String strWindDirection = "0";
String strPressure = "000000";
String strTemperature = "000";
String strUrlEncodedData ="";
String strDataLength = "";

/*There are two serial ports.The definitions below are */
/*related to the serial ports */
char serialData = 0;
char serial2Data = 0;
const int PowerPin =  27; 
const int HESensor = 19;
const int EncoderB0 = 22;
const int EncoderB1 = 23;
const int EncoderB2 = 24;
const int EncoderB3 = 25;
const int StatusLed = 53;
byte LedState = LOW;

/*The Serial Port 2 command related Variables */
String receivedCmd = "";         
boolean cmdRdy = false; 

/*Initializtion SM states */
const int POWER_ON = 1;
const int WAIT_POWER_UP = 2;
const int WAIT_CALL_RDY = 3;
const int GPRS_CGATT = 4;
const int GPRS_CON_TYPE = 5;
const int GPRS_APN = 6;
const int GPRS_SAPBR = 7;
const int HTTP_INIT = 8;
const int HTTP_PARAM_URL = 9;
const int HTTP_PARAM_CONTENT = 10;
const int HTTP_INIT_SUCCESS = 11;
const int GPRS_RDY = 20;


/*Http Post SM states */
const int SET_LENGTH = 12;
const int SEND_DATA  = 13;
const int HTTP_POST  = 14;
const int HTTP_POST_OK = 15;
const int HTTP_POST_FAIL = 16;
const int HTTP_POST_READY = 17;


int initStatus = POWER_ON;
int httpPostStatus = SET_LENGTH;
boolean waitForOk = false;
int waitingTime = 0;

/*Timer related tasks are extensively used in the main working loop */
/* The supporting constant defines and variables are defined below */
unsigned long prevTime = 0;
unsigned long updateTime = 0;
unsigned long currentTime = 0;
unsigned long toggleTime = 0;
unsigned long INTERVAL = 500;
unsigned long UPDATE_INTERVAL = 0;

/*Since the command responses are hard to decode inside the SM */
/* we use a mechanism of flag based state machine driving for */
/*state transitions */
boolean statusCPIN = false;
boolean statusCALLREADY = false;
boolean statusGPRS = false;
boolean statusDOWNLOAD = false;
boolean statusPOST = false;
boolean statusPowerDown = false;
boolean statusGprsNotRdy = false;
boolean statusPOSTFail = false;
boolean statusError = false;

/*Initial setup will proceed here */
void setup()
{
    Serial.begin(9600);  
    Serial2.begin(9600); 

    /*Setup all the Digital IO pins Here */
    pinMode(PowerPin, OUTPUT);
    pinMode(HESensor, INPUT);
    pinMode(EncoderB0, INPUT_PULLUP);
    pinMode(EncoderB1, INPUT_PULLUP);
    pinMode(EncoderB2, INPUT_PULLUP);
    pinMode(EncoderB3, INPUT_PULLUP);
    pinMode(StatusLed, OUTPUT);
    digitalWrite(StatusLed,LOW);
    
    /* We need to capture any change of HESensor pin */
    /* We define a pin change interrupt on interrupt line 4 */
    attachInterrupt(4, blink, CHANGE);
    
    /*Start I2C and initialize the temperature Sensor */
    Wire.begin();
    bmp085Calibration();
    
    /*debug delay while we reconnect a serial port for debug message monitoring */ 
    delay(3000);      
    PrintDebug("Started");


}

void loop()
{

  /*All the program is timed using the time stamps taken from here */
  currentTime = millis();

  /*Regardless of time, we continously monitor for, any possible */
  /*command from SIM900 GSM Module */
  ModemCommandHandler();
  
  /*If the GPRS is not yet attached the SUer will be indicated by flashing LED */
  if(currentTime - toggleTime > 100) {
    
     if(initStatus < GPRS_SAPBR){
       
       ToggleStatusLED();
       
     }else {
       
       digitalWrite(StatusLed,HIGH);
     }
     toggleTime = currentTime;
    
  }

  /*Once each 500ms, We run the GPRS modem Task */
  if(currentTime - prevTime > INTERVAL) {
    
      InitGprsModem();

      prevTime = currentTime;
      
      UPDATE_INTERVAL++;
      
  }
  
  

     if(UPDATE_INTERVAL > 60) {
      
      statusPOST = false; 
      statusDOWNLOAD = false;

      windSpeed = GetWindSpeed();

      windDirection = GetWindDirection();

      if(initStatus == HTTP_POST_OK || initStatus == HTTP_INIT_SUCCESS) {
        
          temperature = bmp085GetTemperature(bmp085ReadUT());
          pressure = bmp085GetPressure(bmp085ReadUP());
          altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));
          
          strTemperature = String(temperature);
          strPressure = String(pressure);
          strWindSpeed = String(windSpeed);
          strWindDirection = String(windDirection);
          
          strUrlEncodedData = "device_id=1&field_a=" + strWindSpeed + "&field_b=" + strWindDirection +"&field_c=" 
                              + strTemperature + "&field_d=" + strPressure;
          strDataLength = String(strUrlEncodedData.length()); 
          Serial.print("Wind Speed: ");
          Serial.print(strWindSpeed);
          Serial.println("");
          Serial.print("Wind Direction: ");
          Serial.print(strWindDirection);
          Serial.println(" *45 deg from North");
          Serial.print("Temperature: ");
          Serial.print(strTemperature);
          Serial.println(" *0.1 deg C");
          Serial.print("Pressure: ");
          Serial.print(strPressure);
          Serial.println(" Pa");
          Serial.print("Altitude: ");
          Serial.print(altitude, 2);
          Serial.println(" m");
          Serial.println();
          Serial.println(strUrlEncodedData);
          Serial.print(strUrlEncodedData.length());
          
          initStatus = SET_LENGTH;
        
      }

      UPDATE_INTERVAL = 0;
      updateTime = currentTime;
   }

}

 /*Pin toggle function for status LED */
 void ToggleStatusLED() {
   
   if(LedState == LOW) {
     LedState = HIGH;
   }else {
     LedState = LOW;
   }
   
   digitalWrite(StatusLed,LedState);   
   
 }
 /*This defines what to do when a Hall efect sensor pin changes */
  void blink()
  {
    pulseCount++;

  }
  
  /*Calculate the Wind speed from the colleted pulse count information*/
  int GetWindSpeed(){
    
    float windSpd = 0;
    float rpm = 0;
    int iWindSpd = 0;
    
    /*Revolutions Per Minute = No of Low to high transistions per minute/4 */
    /*Since we calculate once  ina 30 Sec,*/
    /*Revolutions Per Minute = No of Low to high transistions per minute/4 x 2 */
    /*No of Low to high transistions = pulseCount/2 */
    
    rpm = pulseCount / 4; 
    Serial.print("RPM:");
    Serial.print(rpm);
    Serial.println("");
    /*Tangential Velocity = pi x Angular velocity x diameter */
    
    /*windSpd = 3.14 * 0.3 * rpm / 60; in short*/
    
    windSpd = rpm * 0.016;
    windSpd = windSpd * 100.0f;
    iWindSpd = (int)windSpd;
    pulseCount = 0;
    
    return iWindSpd;
    
  }
  
  /*Read the Encoder and determin the direction */
  int GetWindDirection(){
    
    int windDir = 0;
    int b0 = LOW;
    int b1 = LOW;
    int b2 = LOW;
    
    b0 = digitalRead(EncoderB0);
    b1 = digitalRead(EncoderB1);
    b2 = digitalRead(EncoderB2);
    
    if(b2 == LOW && b1 == HIGH && b0 == LOW){
      
      windDir = 0; //North
      
    }else if(b2 == LOW && b1 == LOW && b0 == LOW){
            
      windDir = 1; //North East
      
    }else if(b2 == HIGH && b1 == LOW && b0 == LOW){
      
      windDir = 2; //East
       
    }else if(b2 == HIGH && b1 == HIGH && b0 == LOW){
      
      windDir = 3; //South East
      
    }else if(b2 == HIGH && b1 == HIGH && b0 == HIGH){
          
      windDir = 4; //South
      
    }else if(b2 == HIGH && b1 == LOW && b0 == HIGH){
      
      windDir = 5; //South Wsst
            
    }else if(b2 == LOW && b1 == LOW && b0 == HIGH){
      
      windDir = 6; //West
          
    }else if(b2 == LOW && b1 == HIGH && b0 == HIGH){
          
      windDir = 7; //North West
      
    }      
    
    return windDir;
  }

/*Simulate the powering on the GSM modem */
void PressPowerButton() {
  
  digitalWrite(PowerPin,LOW);
  delay(2000);
  digitalWrite(PowerPin,HIGH);
  delay(3000);
  digitalWrite(PowerPin,LOW);
  
}

/*When any charactor received from Debug port, We direct it to SIM900 */
void serialEvent() {
  
    serialData = (char)Serial.read();
    Serial2.write(serialData);
  
}

/*When we receive from SIM900, We cxapture teh data here */
void serialEvent2() {
  
  while (Serial2.available()) {

    char serial2Data = (char)Serial2.read();
    receivedCmd += serial2Data;

    if (serial2Data == '\r') {
      cmdRdy = true;
    }
  }
  
}

/*SIM900 GSM modem state machine is defined here */
void InitGprsModem() {
  
  
  switch(initStatus){
    
    case POWER_ON : PressPowerButton();
                    
                    initStatus = WAIT_POWER_UP;
                    statusCPIN = false;
                    statusCALLREADY = false;
                    statusGPRS = false;
                    statusDOWNLOAD = false;
                    statusPOST = false;
                    statusPowerDown = false;
                    statusGprsNotRdy = false;
                    statusPOSTFail = false;
                    digitalWrite(StatusLed,LOW);
                    PrintDebug("Power on sequence sent");
                    delay(2000);
                    break;
                    
                    
case WAIT_POWER_UP: 
    
                    if(statusCPIN){
                      
                      initStatus = WAIT_CALL_RDY;
                      PrintDebug("Power on completed");
                      PrintDebug("Waiting for call ready");
                    }else{
                      
                      initStatus = WAIT_POWER_UP;
                      PrintDebug("Waiting for power up reset");
                    }
                    
                    if(statusPowerDown) {
                      
                      initStatus = POWER_ON;
                      delay(2000);
                      PrintDebug("Power on sequence sent again");
                      statusPowerDown = false;
                    }
                    break;
    
    case WAIT_CALL_RDY :
    
                    if(statusCALLREADY) {
                      
                      initStatus = GPRS_CGATT;
                      PrintDebug("Modem init success!");
                      Serial2.write("AT+CGATT?\r");
                      statusGPRS = false;
                      waitForOk = false;
                      waitingTime = 0;
                      PrintDebug("Waiting for GPRS status");
                      
                    }else{
                      
                      initStatus = WAIT_CALL_RDY;
                      waitingTime ++;
                      Serial.write(".");
                    }
                    
                    if(statusPowerDown || waitingTime > 80) {
                      
                      initStatus = POWER_ON;
                      waitingTime =0;
                      delay(2000);
                      PrintDebug("Power on sequence sent again");
                      statusPowerDown = false;
                    }
                  break;
                  
    case GPRS_CGATT : 
      
                  if(statusGPRS) {
                    
                    initStatus = GPRS_CON_TYPE;
                    Serial2.write("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r");
                    waitForOk = false;
                    waitingTime = 0;
                    PrintDebug("Waiting for GPRS Con Type");
                    
                  }else {
                    
                    initStatus = GPRS_CGATT;
                    waitingTime++;
                  }
                  
                  if(statusGprsNotRdy) {
                    
                    delay(2000);
                    initStatus = GPRS_CGATT;
                    statusGprsNotRdy = false;
                    PrintDebug("Waiting till Gprs Attached..");
                    Serial2.write("AT+CGATT?\r");

                    
                  }

                  break;
               
    case GPRS_CON_TYPE : 
    
                  if(waitForOk) {
                    
                    initStatus = GPRS_APN;
                    Serial2.write("AT+SAPBR=3,1,\"APN\",\"MOBITEL3G\"\r");
                    waitForOk = false;
                    waitingTime = 0;
                    PrintDebug("Waiting for APN configuration");
                    
                  }else {
                    
                    initStatus = GPRS_CON_TYPE;
                    waitingTime++;
                  }

                  break;
                  
    case GPRS_APN : 
    
                  if(waitForOk) {
                    
                    initStatus = GPRS_SAPBR;
                    Serial2.write("AT+SAPBR=1,1\r");
                    waitForOk = false;
                    waitingTime = 0;
                    PrintDebug("Waiting for Bearer configuration");
                    
                  }else {
                    
                    initStatus = GPRS_APN;
                    waitingTime++;
                  }

                  break;
                  
    case GPRS_SAPBR : 
    
                   if(waitForOk) {
                    
                    initStatus = HTTP_INIT;
                    Serial2.write("AT+HTTPINIT\r");
                    waitForOk = false;
                    waitingTime = 0;
                    PrintDebug("Waiting for HTTP Init");
                    
                  }else {
                    
                    initStatus = GPRS_SAPBR;
                    waitingTime++;
                  }

                  break;
                  
    case HTTP_INIT :
    
                  if(waitForOk) {
                    
                    initStatus = HTTP_PARAM_URL;
                    Serial2.write("AT+HTTPPARA=\"URL\",\"www.wind4lanka.info/save_data.php\"\r");
                    waitForOk = false;
                    waitingTime = 0;
                    PrintDebug("Waiting for HTTP set URL Param");
                    
                  }else {
                    
                    initStatus = HTTP_INIT;
                    waitingTime++;
                  }
                  break;
                  
     case HTTP_PARAM_URL :
    
                  if(waitForOk) {
                    
                    initStatus = HTTP_PARAM_CONTENT;
                    Serial2.write("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"\r");
                    waitForOk = false;
                    waitingTime = 0;
                    PrintDebug("Waiting for HTTP set Content Param");
                    
                  }else {
                    
                    initStatus = HTTP_PARAM_URL;
                    waitingTime++;
                  }
                  break;
                  
     case HTTP_PARAM_CONTENT :
    
                  if(waitForOk) {
                    
                    initStatus = HTTP_INIT_SUCCESS;
                    waitForOk = false;
                    waitingTime = 0;
                    PrintDebug("HTTP Init Success!");
                    
                  }else {
                    
                    initStatus = HTTP_PARAM_CONTENT;
                    waitingTime++;
                  }
                  break; 
  
    case HTTP_INIT_SUCCESS :

                initStatus = HTTP_INIT_SUCCESS;
                break;
         
              
    case SET_LENGTH :
      
                     PrintDebug("Post Data Starting");
                     PrintDebug("Setting Data Length");
                     Serial2.print("AT+HTTPDATA="+ strDataLength +",10000\r");
                     //Serial2.write("AT\r")
                     initStatus = SEND_DATA;
                     break;
                     
      case SEND_DATA :
      
                     if(statusDOWNLOAD) {
                       
                       PrintDebug("Data uploading");
                       //Serial2.write("field_a=31&field_b=7&field_c=8&field_d=2");
                       Serial2.print(strUrlEncodedData);
                       initStatus = HTTP_POST;
                       waitForOk = false;
                       waitingTime = 0;
                       
                     }else {
                       
                       initStatus = SEND_DATA;
                       waitingTime++;
                     }
                     break;
      
      case HTTP_POST :
      
                     if(waitForOk) {
                       
                       PrintDebug("Http Posted");
                       Serial2.write("AT+HTTPACTION=1\r");
                       initStatus = HTTP_POST_READY;
                       waitForOk = false;
                       waitingTime = 0;
                       
                     }else {
                       
                       initStatus = HTTP_POST;
                       waitingTime++;
                     }
                 
                     break;    
      
      case HTTP_POST_READY :
      
                      if(statusPOST) {
                        
                       //PrintDebug("");
                       Serial.println("Post Sucecess !");
                       initStatus = HTTP_POST_OK;
                       waitForOk = false;
                       waitingTime = 0;
                       
                       statusPOST = false; 
                       statusDOWNLOAD = false;
                       
                      }else {
                        
                       initStatus = HTTP_POST_READY;
                       waitingTime++;
                       
                       if(statusPOSTFail) {
                        
                        Serial.println("Post Failed!");
                        Serial.println("We Init HTTP Again and try!");
                        //waitForOk = false;
                        initStatus = POWER_ON;
                        statusDOWNLOAD = false;
                        statusPOST = false;
                        statusPOSTFail = false;
                        }

                      }
                      

                      
                      
                      break;
      
      case HTTP_POST_OK : 
                      initStatus = HTTP_POST_OK;
                      Serial.write(">");
                          break;
      
      case HTTP_POST_FAIL : break;
      
      default :
    
                  break;
    
  }
  
  
  
}


/*Process the received commadn from the SIM900 GSm Modem */
void ModemCommandHandler() {
  
  if(cmdRdy) {
    
    Serial.println(receivedCmd);
    
    if(!statusCPIN) {
      
      if(receivedCmd.indexOf("READY") != (-1)){
      
        statusCPIN = true;
            
      }
    }
    
    if(!statusPowerDown && !statusCPIN) {
      
       if(receivedCmd.indexOf("POWER DOWN") != (-1)){
      
            statusPowerDown = true;
        }          
    }
    
    if(!statusCALLREADY) {
      
      if(receivedCmd.indexOf("Call Ready") != (-1)){
      
        statusCALLREADY = true;
        receivedCmd = "";
            
      }
      
    }
    
    if(!statusGPRS) {
      
      if(receivedCmd.indexOf("+CGATT: 1") != (-1)){
      
        statusGPRS = true;
        waitForOk = true;
        waitingTime = 0;
            
      }
    
    }
    
    if(!statusGprsNotRdy && !statusGPRS) {
      
      if(receivedCmd.indexOf("+CGATT: 0") != (-1)){
      
        statusGprsNotRdy = true;
        waitForOk = true;
        waitingTime ++;
            
      }
    
    }
    
    
    if(statusCALLREADY && !waitForOk) {
      
      if(receivedCmd.indexOf("OK") != (-1)){
      
        waitForOk = true;
        waitingTime = 0;
            
      }
      
    }
 
    if(!statusDOWNLOAD && statusCALLREADY) {
      
        if(receivedCmd.indexOf("DOWNLOAD") != (-1)){
      
          waitForOk = true;
          waitingTime = 0;
          statusDOWNLOAD = true;
            
        }
      
    }
    
    if(!statusPOST && statusCALLREADY && initStatus == HTTP_POST_READY) {
      
        if(receivedCmd.indexOf("30") != (-1) || receivedCmd.indexOf("20") != (-1)){
      
          waitForOk = true;
          waitingTime = 0;
          statusPOST = true;
            
       }
     
         if(receivedCmd.indexOf("60") != (-1) || receivedCmd.indexOf("40") != (-1) || receivedCmd.indexOf("50") != (-1) ){
      
          waitForOk = true;
          waitingTime = 0;
          //statusPOST = true;
          statusPOSTFail = true;
            
         }
        
  
      
      
    }
    receivedCmd = "";
    cmdRdy = false;
  }
 
}

void PrintDebug(String x) {
  
  Serial.println(x);
  
}
  
void SendCommand(String x) {
  
  Serial2.println(x);
  
}

/*Stores all of the bmp085's calibration values into global variables*/
/*Calibration values are required to calculate temp and pressure*/
/*This function should be called at the beginning of the program */
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

/*Calculate temperature given ut.*/
/*Value returned will be in units of 0.1 deg C */
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}






