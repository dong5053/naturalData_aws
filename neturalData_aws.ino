#include <DHT11.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <Stream.h>
#include <Wire.h>
#include <stdlib.h>

// Sensor Pin Number
#define dustSensor 2            // 미세먼지 센서
#define ledGREEN 3              // Green LED
#define ledRED 4                // Red LED
#define ultrasonic_Trig 5       // 초음파센서 (Trig) => 출력
#define ultrasonic_Echo 6       // 초음파센서 (Echo) => 입력
#define temper_Hum 8            // 온습도 센서
//#define wifi_RX 10              // WIFI 모듈 RX(수신)
//#define wifi_TX 11              // WIFI 모듈 TX(송신)
#define illuminSensor A0        // 조도센서
#define gasSensor A1            // 가스센서
#define pressureSensor_SDA A4   // 대기압센서 SDA
#define pressureSensor_SCL A5   // 대기압센서 SCL
//#define port    443     // IOT 통신할 포트번호

// I2C Data Type
#define DST "DST"             // 미센먼지 데이터
#define UTW "UTW"            // 초음파센서 데이터
#define TPH "TPH"           // 온습도센서 데이터
#define ILM "ILM"             // 조도센서 데이터
#define GAS "GAS"             // 가스센서 데이터
#define PSS "PSS"             // 대기압센서 데이터

typedef struct
{
  float temp, humi;       // 온도, 습도 변수
}TemperHumiSensor;

typedef struct
{
  float Ftemperature = 0.0;    // 온도 측정 값
  float Fpressure = 0.0;    // 기압 측정 값
  float Faltitude = 0.0;    // 고도 측정 값

  char CAtemperature[20] = "";    // 온도
  char CApressure[20] = "";       // 기압
  char CAaltitude[20] = "";       // 고도
  char CAbuffer[254] = "";
}PressureSensor;

LiquidCrystal_I2C lcd(0x27, 16, 2);
String buf = "";
String sensorData = "";
int dataSize = 0;

void setup()
{
  Serial.begin(9600); // 통신 속도 설정
  Serial.println("Hello, Arduino!");  // 시리얼 포트 TX로 Hello, Arduino! 출력
  Wire.begin(8);                // join i2c bus with address 8 //
  Wire.onReceive(receiveEvent); // register receive event //
  Wire.onRequest(requestEvent); // register request event //
  pinMode(ledRED, OUTPUT);        // LED핀 출력모드
  pinMode(ledGREEN, OUTPUT);      
  pinMode(ultrasonic_Trig, OUTPUT);   // 초음파센서 출력핀 설정
  pinMode(ultrasonic_Echo, INPUT);    // 초음파센서 입력핀 설정
  pinMode(gasSensor, INPUT);
  initLCD();
}

void loop()
{
  void * ptr = NULL;
  
  //printLCD(ptr = TempHum_Check(), temper_Hum);        // 온습도센서 측정 및 출력
  //sensorData = String(((TemperHumiSensor *)ptr)->humi);
  //Serial.println(sensorData.length());
  //dataSize = size.length();
  /* sensorData = String(((TemperHumiSensor *)ptr)->temp);
  Serial.println(sensorData.length());
  dataSize = sensorData.length();
  free(ptr); */
  /* digitalWrite(ledRED, HIGH);     // LED ON
  digitalWrite(ledGREEN, HIGH);   // LED ON
  delay(3000);
  digitalWrite(ledRED, LOW);     // LED OFF
  digitalWrite(ledGREEN, LOW);   // LED OFF

  printLCD(ptr = UltrasonicWave_Check(), ultrasonic_Trig);    // 초음파센서 측정 및 출력
  free(ptr);
  digitalWrite(ledRED, HIGH);     // LED ON
  digitalWrite(ledGREEN, HIGH);   // LED ON
  delay(3000);
  digitalWrite(ledRED, LOW);     // LED OFF
  digitalWrite(ledGREEN, LOW);   // LED OFF

  printLCD(ptr = Illuminance_Check(illuminSensor), illuminSensor);    // 조도센서 측정 및 출력
  free(ptr);
  digitalWrite(ledRED, HIGH);     // LED ON
  digitalWrite(ledGREEN, HIGH);   // LED ON
  delay(3000);
  digitalWrite(ledRED, LOW);     // LED OFF
  digitalWrite(ledGREEN, LOW);   // LED OFF

  printLCD(ptr = Gas_Check(gasSensor), gasSensor);            // 일산화탄소센서 측정 및 출력
  free(ptr);
  digitalWrite(ledRED, HIGH);     // LED ON
  digitalWrite(ledGREEN, HIGH);   // LED ON
  delay(3000);
  digitalWrite(ledRED, LOW);     // LED OFF
  digitalWrite(ledGREEN, LOW);   // LED OFF

  printLCD(ptr = DustDensity_Check(), dustSensor);            // 미세먼지센서 측정 및 출력
  free(ptr);
  digitalWrite(ledRED, HIGH);     // LED ON
  digitalWrite(ledGREEN, HIGH);   // LED ON
  delay(3000);
  digitalWrite(ledRED, LOW);     // LED OFF
  digitalWrite(ledGREEN, LOW);   // LED OFF

  printLCD(ptr = Pressure_Check(), pressureSensor_SDA);       // 대기압센서 측정 및 출력
  free(ptr);
  digitalWrite(ledRED, HIGH);     // LED ON
  digitalWrite(ledGREEN, HIGH);   // LED ON
  delay(3000);
  digitalWrite(ledRED, LOW);     // LED OFF
  digitalWrite(ledGREEN, LOW);   // LED OFF */

}

int * UltrasonicWave_Check()                  // 초음파 센서 측정
{
    int * distance = (int *)malloc(sizeof(int));
    digitalWrite(ultrasonic_Trig, HIGH);
    delayMicroseconds(30);
    digitalWrite(ultrasonic_Trig, LOW);

    *distance = pulseIn(ultrasonic_Echo, HIGH)*340/2/10000;  // echo핀으로 들어온 신호를 마이크로 단위로 변환
    // 음파의 속도 초당 340m / 2 / cm 변환
    delay(1000);

    return distance;
}

TemperHumiSensor * TempHum_Check()                 // 온습도 센서 측정
{
    DHT11 dht11(temper_Hum);
    TemperHumiSensor * ths = (TemperHumiSensor *)malloc(sizeof(TemperHumiSensor));
    dht11.read(ths->humi, ths->temp);         // 측정
    delay(1000);

    return ths;
}

int * Illuminance_Check(int analogPinNum)         // 조도 측정
{
    int * illumin_Value = (int *)malloc(sizeof(int));
    *illumin_Value = analogRead(analogPinNum);
    delay(1000);     // 측정 딜레이

    return illumin_Value;
}

float * Gas_Check(int analogPinNum)           // 일산화탄소 측정
{
    float RS_gas = 0;
    float ratio = 0;
    float sensor_volt = 0;
    float R0 = 7200.0;                  // 깨끗한 공기에서의 센서 저항값 (2K옴 기준)
    float gas_Value = analogRead(analogPinNum);

    sensor_volt = gas_Value/1024*5.0;
    RS_gas = (5.0-sensor_volt)/sensor_volt;       // 5V 전압 기준
    ratio = RS_gas/R0;                           //Replace R0 with the value found using the sketch above
    float x = 1538.46 * ratio;
    float * ppm = (float *)malloc(sizeof(float));
    *ppm = pow(x,-1.709);                   // PPM값 계산
    delay(1000);

    return ppm;
}

float * DustDensity_Check()       // 미세먼지 측정
{
    // 국제 미세먼지농도에 따른 경계단계 기준분류 
    // 30ug/m^3 이하 : 좋음 / 30~80ug/m^3 : 보통 / 80~150ug/m^3 : 나쁨 / 150ug/m^3 초과 : 매우 나쁨
    float ratio = 0;
    float concentration = 0;
    float * dustDensity = (float *)malloc(sizeof(float));                      // 미세먼지 밀도
    unsigned long sampletime_ms = 2000;         // 미세먼지 센서의 측정시간
    float duration = 0.0;
    for(int i=4; i>0; i--)          // 정확한 측정을 위해 여러번 펄스값을 읽음
    {
        duration = pulseIn(dustSensor, LOW);  // Pin에서 LOW가 될때 펄스값을 읽음
        delay(500);
    }
    unsigned long lowpulseoccupancy = lowpulseoccupancy + duration;
    
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0); // Integer percentage 0=>100
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
    *dustDensity = concentration * 100 / 13000;
    lowpulseoccupancy = 0;

    return dustDensity;
}

PressureSensor * Pressure_Check()       // 대기압 측정
{
    Adafruit_BMP085 bmp;
    PressureSensor * ps = (PressureSensor *)malloc(sizeof(PressureSensor));
    if (!bmp.begin(BMP085_ULTRAHIGHRES)) {
        Serial.println("BMP180 센서를 찾을 수 없습니다. 연결을 확인해 주세요!");
        return;
    }
 
    ps->Ftemperature = bmp.readTemperature();
    ps->Fpressure = bmp.readPressure();
    ps->Faltitude = bmp.readAltitude(101600);     // 101600 : 인천 송도 해면기압, 101325 : 표준기압

    dtostrf(ps->Ftemperature, 4, 2, ps->CAtemperature);
    dtostrf(ps->Fpressure, 9, 2, ps->CApressure);
    dtostrf(ps->Faltitude, 5, 2, ps->CAaltitude);
 
    // 온도 압력 고도
    sprintf(ps->CAbuffer, "Temp(.C): %s\tPres(PA): %s\tAlt(M): %s", ps->CAtemperature, ps->CApressure, ps->CAaltitude);
    delay(1000);

    return ps;
}

void initLCD()          // LCD 초기화
{
    byte humidityImage[] = {0x04,0x0E,0x0E,0x1F,0x1F,0x1F,0x1F,0x0E};       // 물방울 아이콘
    byte temperatureImage[] = {0x04,0x0A,0x0A,0x0A,0x0E,0x1F,0x1F,0x0E};    // 온도 아이콘
    byte doImage[] = {0x1C,0x14,0x1C,0x00,0x00,0x00,0x00,0x00};             // ˚ 아이콘
    byte microImage[] = {0x11,0x11,0x11,0x13,0x15,0x18,0x10,0x10};          //LCD "m" 이미지
    byte threeImage[] = {0x18,0x04,0x18,0x04,0x18,0x00,0x00,0x00};          //LCD "3" 이미지

    lcd.init();
    lcd.backlight();
    lcd.createChar(0, humidityImage);
    lcd.createChar(1, temperatureImage);
    lcd.createChar(2, doImage);
    lcd.createChar(3, microImage);
    lcd.createChar(4, threeImage);
    lcd.home();
    lcd.print("Loading...");
}

void printLCD(void * value, int sensorType)         // LCD에 출력, sensorType을 지정하여 각 센서에 맞게 출력
{
    //char printBuffer[254] = "";
    
    if(sensorType == temper_Hum)
    {
        // Serial 출력
        Serial.print("Humidity : ");
        Serial.print(((TemperHumiSensor *)value)->humi);
        Serial.print("% / Temperature : ");
        Serial.print(((TemperHumiSensor *)value)->temp);
        Serial.println(" C");
        // LCD 출력
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(0);
        lcd.print("Humi : ");
        lcd.print(((TemperHumiSensor *)value)->humi);
        lcd.print("%");
        lcd.setCursor(0,1);
        lcd.write(1);
        lcd.print("Temp : ");
        lcd.print(((TemperHumiSensor *)value)->temp);
        lcd.write(2);
        lcd.print("C");
    }
    else if(sensorType == ultrasonic_Trig)
    {
        Serial.print(*(int *)value);
        Serial.println("CM");

        lcd.clear();
        lcd.setCursor(2,0);     // 열, 행
        lcd.print("# Distance #");
        lcd.setCursor(6,1);
        lcd.print(*(int *)value);
        lcd.print("CM");
    }
    else if(sensorType == illuminSensor)
    {
        Serial.print("Illumin : ");
        Serial.println(*(int *)value);

        lcd.clear();
        lcd.setCursor(1,0);     // 열, 행
        lcd.print("*Illumination*");
        lcd.setCursor(6,1);
        lcd.print(*(int *)value);
        lcd.print("LX");
    }
    else if(sensorType == gasSensor)
    {
        Serial.print("gas(ppm) : ");
        Serial.println(*(float *)value);

        lcd.clear();
        lcd.setCursor(1,0);     // 열, 행
        lcd.print("CarbonMonoxide");
        lcd.setCursor(5,1);
        lcd.print(*(float *)value);
        lcd.print("PPM");
    }
    else if(sensorType == dustSensor)
    {
        Serial.print("Dust : ");
        Serial.print(*(float *)value);
        Serial.println("ug/m^3");

        lcd.clear();
        lcd.setCursor(2,0);     // 열, 행
        lcd.print("*Micro_Dust*");
        lcd.setCursor(3,1);
        lcd.print(*(float *)value);
        lcd.print("ug/");
        lcd.write(3);
        lcd.write(4);
    }
    else if(sensorType == pressureSensor_SCL || pressureSensor_SDA)
    {
        Serial.println(((PressureSensor *)value)->CAbuffer);

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.write(1);
        lcd.print(((PressureSensor *)value)->CAtemperature);
        lcd.write(2);
        lcd.print("C ");
        lcd.print("A:");
        lcd.print(((PressureSensor *)value)->CAaltitude);
        lcd.print("M");
        lcd.setCursor(0,1);
        lcd.print("Pres:");
        lcd.print(((PressureSensor *)value)->CApressure);
        lcd.print("PA");
    }
}


// function that executes whenever data is received from master
void receiveEvent(int howMany) 
{
  while (0 <Wire.available()) {
    char c = Wire.read();      // receive byte as a character //
    Serial.print(c);           // print the character //
    buf += c;
  }
  Serial.println();             // to newline //
}
// function that executes whenever data is requested from master

void requestEvent() 
{
  char c_tmp[32] = {0};
  String s_tmp = "";

  if(buf == "Connect...")
    Wire.write("Connect Success");
  else if(buf == "GET TPH DataSize")
  {
    /* s_tmp = String(dataSize);
    s_tmp.toCharArray(c_tmp, s_tmp.length());
    Wire.write(c_tmp);  //send string on request */
    Wire.write("5");
  }
  else if(buf == "GET TPH")
  {
    /* sensorData.toCharArray(c_tmp, sensorData.length());
    Wire.write(c_tmp); */
    Wire.write("25.00");
  }
  buf = "";      // 버퍼 초기화
}