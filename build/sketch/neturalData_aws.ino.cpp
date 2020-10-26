#line 1 "d:\\neturalData_aws\\neturalData_aws.ino"
#include <DHT11.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <Stream.h>
#include <Wire.h>
#include <stdlib.h>

// Sensor Pin Number
#define dustSensor 2      // �̼����� ����
#define ledGREEN 3        // Green LED
#define ledRED 4          // Red LED
#define ultrasonic_Trig 5 // �����ļ��� (Trig) => ���
#define ultrasonic_Echo 6 // �����ļ��� (Echo) => �Է�
#define temper_Hum 8      // �½��� ����
//#define wifi_RX 10              // WIFI ��� RX(����)
//#define wifi_TX 11              // WIFI ��� TX(�۽�)
#define illuminSensor A0      // ��������
#define gasSensor A1          // ��������
#define pressureSensor_SDA A4 // ���м��� SDA
#define pressureSensor_SCL A5 // ���м��� SCL
//#define port    443     // IOT ����� ��Ʈ��ȣ

// I2C Data Type
#define DST "DST" // �̼����� ������
#define UTW "UTW" // �����ļ��� ������
#define TPH "TPH" // �½������� ������
#define ILM "ILM" // �������� ������
#define GAS "GAS" // �������� ������
#define PSS "PSS" // ���м��� ������

typedef struct
{
  float temp = 0.0, humi = 0.0; // �µ�, ���� ����
} TemperHumiSensor;

typedef struct
{
  float Ftemperature = 0.0; // �µ� ���� ��
  float Fpressure = 0.0;    // ��� ���� ��
  float Faltitude = 0.0;    // �� ���� ��

  char CAtemperature[20] = ""; // �µ�
  char CApressure[20] = "";    // ���
  char CAaltitude[20] = "";    // ��
  char CAbuffer[128] = "";
} PressureSensor;

typedef struct
{
  TemperHumiSensor ths;        // �½���
  char CAtemperature[20] = ""; // �µ�(���м���)
  char CApressure[20] = "";    // ���(���м���)
  char CAaltitude[20] = "";    // ��(���м���)
  int distance = 0;            // �����ļ��� �Ÿ�
  int illumin_Value = 0;       // �������� ��
  float ppm = 0.0;             // �ϻ�ȭź�� ��
  float dustDensity = 0.0;     // �̼����� ��
} NaturalData;                 // ������ ȯ�� ������ ������ ���� �� ������Ʈ�� ����ü

LiquidCrystal_I2C lcd(0x27, 16, 2);
String buf = "";        // I2C ��/���� ����
String sensorData = ""; // ���� ������ ��
int dataSize = 0;       // �������� ����
NaturalData clt_dt;     // ȯ�浥���� ������ ���� ����ü ����
bool update = false;    // ȯ�� ������ ������Ʈ ����(�����Ϸ� true / ���� �� false)

#line 68 "d:\\neturalData_aws\\neturalData_aws.ino"
void setup();
#line 85 "d:\\neturalData_aws\\neturalData_aws.ino"
void loop();
#line 145 "d:\\neturalData_aws\\neturalData_aws.ino"
int * UltrasonicWave_Check();
#line 159 "d:\\neturalData_aws\\neturalData_aws.ino"
TemperHumiSensor* TempHum_Check();
#line 169 "d:\\neturalData_aws\\neturalData_aws.ino"
int * Illuminance_Check(int analogPinNum);
#line 178 "d:\\neturalData_aws\\neturalData_aws.ino"
float * Gas_Check(int analogPinNum);
#line 197 "d:\\neturalData_aws\\neturalData_aws.ino"
float * DustDensity_Check();
#line 227 "d:\\neturalData_aws\\neturalData_aws.ino"
PressureSensor* Pressure_Check();
#line 252 "d:\\neturalData_aws\\neturalData_aws.ino"
void initLCD();
#line 271 "d:\\neturalData_aws\\neturalData_aws.ino"
void printLCD(void *value, int sensorType);
#line 371 "d:\\neturalData_aws\\neturalData_aws.ino"
void receiveEvent(int howMany);
#line 383 "d:\\neturalData_aws\\neturalData_aws.ino"
void requestEvent();
#line 68 "d:\\neturalData_aws\\neturalData_aws.ino"
void setup()
{
  Serial.begin(9600);                // ��� �ӵ� ����
  Serial.println("Start, Arduino!"); // �ø��� ��Ʈ TX�� Hello, Arduino! ���
  Wire.begin(8);                     // join i2c bus with address 8 //
  Wire.onReceive(receiveEvent);      // register receive event //
  Wire.onRequest(requestEvent);      // register request event //
  pinMode(ledRED, OUTPUT);           // LED�� ��¸��
  pinMode(ledGREEN, OUTPUT);
  pinMode(ultrasonic_Trig, OUTPUT); // �����ļ��� ����� ����
  pinMode(ultrasonic_Echo, INPUT);  // �����ļ��� �Է��� ����
  pinMode(gasSensor, INPUT);
  initLCD();
  Serial.println("Initializing I2C..."); // I2C ��� ����Ȯ��
  delay(3000);
}

void loop()
{
  void *ptr = NULL;
  update = false;

  delay(2000);
  digitalWrite(ledGREEN, HIGH);                      // LED ON
  printLCD(ptr = TempHum_Check(), temper_Hum);       // �½������� ���� �� ���
  clt_dt.ths.humi = ((TemperHumiSensor *)ptr)->humi; // ���� �� ������Ʈ
  clt_dt.ths.temp = ((TemperHumiSensor *)ptr)->temp; // �µ� �� ������Ʈ
  delay(2000);
  digitalWrite(ledGREEN, LOW); // LED OFF
  free(ptr);

  digitalWrite(ledGREEN, HIGH);                            // LED ON
  printLCD(ptr = UltrasonicWave_Check(), ultrasonic_Trig); // �����ļ��� ���� �� ���
  clt_dt.distance = *(int *)ptr;                           // �����ļ��� �� ������Ʈ
  delay(2000);
  digitalWrite(ledGREEN, LOW); // LED OFF
  free(ptr);

  digitalWrite(ledGREEN, HIGH);                                    // LED ON
  printLCD(ptr = Illuminance_Check(illuminSensor), illuminSensor); // �������� ���� �� ���
  clt_dt.illumin_Value = *(int *)ptr;                              // ���� �� ������Ʈ
  delay(2000);
  digitalWrite(ledGREEN, LOW); // LED OFF
  free(ptr);

  digitalWrite(ledGREEN, HIGH);                    // LED ON
  printLCD(ptr = Gas_Check(gasSensor), gasSensor); // �ϻ�ȭź�Ҽ��� ���� �� ���
  clt_dt.ppm = *(float *)ptr;                      // �ϻ�ȭź�� �� ������Ʈ
  delay(2000);
  digitalWrite(ledGREEN, LOW); // LED OFF
  free(ptr);

  digitalWrite(ledGREEN, HIGH);                    // LED ON
  printLCD(ptr = DustDensity_Check(), dustSensor); // �̼��������� ���� �� ���
  clt_dt.dustDensity = *(float *)ptr;              // �̼����� �� ������Ʈ
  delay(2000);
  digitalWrite(ledGREEN, LOW); // LED OFF
  free(ptr);

  digitalWrite(ledGREEN, HIGH);                                         // LED ON
  printLCD(ptr = Pressure_Check(), pressureSensor_SDA);                 // ���м��� ���� �� ���
  strcpy(clt_dt.CAtemperature, ((PressureSensor *)ptr)->CAtemperature); // �µ� �� ������Ʈ
  strcpy(clt_dt.CApressure, ((PressureSensor *)ptr)->CApressure);       // ��� �� ������Ʈ
  strcpy(clt_dt.CAaltitude, ((PressureSensor *)ptr)->CAaltitude);       // �� �� ������Ʈ
  delay(2000);
  digitalWrite(ledGREEN, LOW); // LED OFF
  free(ptr);

  update = true; // ȯ�� ������ ���� ������Ʈ �Ϸ�
  Serial.println("All Sensor Data Update Complete !");
  Serial.println("=========================================================");

  while (update == true) // NodeMcu���� �����͸� �޾ư� �� �ֵ��� ���
    delay(2000);
  buf = "";
}

int *UltrasonicWave_Check() // ������ ���� ����
{
  int *distance = (int *)malloc(sizeof(int));
  digitalWrite(ultrasonic_Trig, HIGH);
  delayMicroseconds(30);
  digitalWrite(ultrasonic_Trig, LOW);

  *distance = pulseIn(ultrasonic_Echo, HIGH) * 340 / 2 / 10000; // echo������ ���� ��ȣ�� ����ũ�� ������ ��ȯ
  // ������ �ӵ� �ʴ� 340m / 2 / cm ��ȯ
  delay(1000);

  return distance;
}

TemperHumiSensor *TempHum_Check() // �½��� ���� ����
{
  DHT11 dht11(temper_Hum);
  TemperHumiSensor *ths = (TemperHumiSensor *)malloc(sizeof(TemperHumiSensor));
  dht11.read(ths->humi, ths->temp); // ����
  delay(1000);

  return ths;
}

int *Illuminance_Check(int analogPinNum) // ���� ����
{
  int *illumin_Value = (int *)malloc(sizeof(int));
  *illumin_Value = analogRead(analogPinNum);
  delay(1000); // ���� ������

  return illumin_Value;
}

float *Gas_Check(int analogPinNum) // �ϻ�ȭź�� ����
{
  float RS_gas = 0;
  float ratio = 0;
  float sensor_volt = 0;
  float R0 = 7200.0; // ������ ���⿡���� ���� ���װ� (2K�� ����)
  float gas_Value = analogRead(analogPinNum);

  sensor_volt = gas_Value / 1024 * 5.0;
  RS_gas = (5.0 - sensor_volt) / sensor_volt; // 5V ���� ����
  ratio = RS_gas / R0;                        //Replace R0 with the value found using the sketch above
  float x = 1538.46 * ratio;
  float *ppm = (float *)malloc(sizeof(float));
  *ppm = pow(x, -1.709); // PPM�� ���
  delay(1000);

  return ppm;
}

float *DustDensity_Check() // �̼����� ����
{
  // ���� �̼������󵵿� ���� ���ܰ� ���غз�
  // 30ug/m^3 ���� : ���� / 30~80ug/m^3 : ���� / 80~150ug/m^3 : ���� / 150ug/m^3 �ʰ� : �ſ� ����
  float ratio = 0;
  float concentration = 0;
  float *dustDensity = (float *)malloc(sizeof(float)); // �̼����� �е�
  unsigned long sampletime_ms = 2000;                  // �̼����� ������ �����ð�
  float duration = 0.0;
  /* for (int i = 4; i > 0; i--) // ��Ȯ�� ������ ���� ������ �޽����� ����
  {
    duration = pulseIn(dustSensor, LOW); // Pin���� LOW�� �ɶ� �޽����� ����
    delay(500);
  } */

  while (duration == 0) // ��Ȯ�� ������ ���� ������ �޽����� ����
  {
    duration = pulseIn(dustSensor, LOW); // Pin���� LOW�� �ɶ� �޽����� ����
    delay(500);
  }
  unsigned long lowpulseoccupancy = lowpulseoccupancy + duration;

  ratio = lowpulseoccupancy / (sampletime_ms * 10.0);                             // Integer percentage 0=>100
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
  *dustDensity = concentration * 100 / 13000;
  lowpulseoccupancy = 0;

  return dustDensity;
}

PressureSensor *Pressure_Check() // ���� ����
{
  Adafruit_BMP085 bmp;
  PressureSensor *ps = (PressureSensor *)malloc(sizeof(PressureSensor));
  if (!bmp.begin(BMP085_ULTRAHIGHRES))
  {
    Serial.println("BMP180 ������ ã�� �� �����ϴ�. ������ Ȯ���� �ּ���!");
    return;
  }

  ps->Ftemperature = bmp.readTemperature();
  ps->Fpressure = bmp.readPressure();
  ps->Faltitude = bmp.readAltitude(101600); // 101600 : ��õ �۵� �ظ���, 101325 : ǥ�ر��

  dtostrf(ps->Ftemperature, 4, 2, ps->CAtemperature);
  dtostrf(ps->Fpressure, 9, 2, ps->CApressure);
  dtostrf(ps->Faltitude, 5, 2, ps->CAaltitude);

  // �µ� �з� ��
  sprintf(ps->CAbuffer, "Temp(.C): %s\tPres(PA): %s\tAlt(M): %s", ps->CAtemperature, ps->CApressure, ps->CAaltitude);
  delay(1000);

  return ps;
}

void initLCD() // LCD �ʱ�ȭ
{
  byte humidityImage[] = {0x04, 0x0E, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E};    // ����� ������
  byte temperatureImage[] = {0x04, 0x0A, 0x0A, 0x0A, 0x0E, 0x1F, 0x1F, 0x0E}; // �µ� ������
  byte doImage[] = {0x1C, 0x14, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00};          // �� ������
  byte microImage[] = {0x11, 0x11, 0x11, 0x13, 0x15, 0x18, 0x10, 0x10};       //LCD "m" �̹���
  byte threeImage[] = {0x18, 0x04, 0x18, 0x04, 0x18, 0x00, 0x00, 0x00};       //LCD "3" �̹���

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

void printLCD(void *value, int sensorType) // LCD�� ���, sensorType�� �����Ͽ� �� ������ �°� ���
{
  //char printBuffer[254] = "";

  if (sensorType == temper_Hum)
  {
    // Serial ���
    Serial.print("LCD Humidity : ");
    Serial.print(((TemperHumiSensor *)value)->humi);
    Serial.print("% / Temperature : ");
    Serial.print(((TemperHumiSensor *)value)->temp);
    Serial.println(" C");
    // LCD ���
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write(0);
    lcd.print("Humi : ");
    lcd.print(((TemperHumiSensor *)value)->humi);
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.write(1);
    lcd.print("Temp : ");
    lcd.print(((TemperHumiSensor *)value)->temp);
    lcd.write(2);
    lcd.print("C");
  }
  else if (sensorType == ultrasonic_Trig)
  {
    Serial.print("LCD ");
    Serial.print(*(int *)value);
    Serial.println("CM");

    lcd.clear();
    lcd.setCursor(2, 0); // ��, ��
    lcd.print("# Distance #");
    lcd.setCursor(6, 1);
    lcd.print(*(int *)value);
    lcd.print("CM");
  }
  else if (sensorType == illuminSensor)
  {
    Serial.print("LCD Illumin : ");
    Serial.println(*(int *)value);

    lcd.clear();
    lcd.setCursor(1, 0); // ��, ��
    lcd.print("*Illumination*");
    lcd.setCursor(6, 1);
    lcd.print(*(int *)value);
    lcd.print("LX");
  }
  else if (sensorType == gasSensor)
  {
    Serial.print("LCD gas(ppm) : ");
    Serial.println(*(float *)value);

    lcd.clear();
    lcd.setCursor(1, 0); // ��, ��
    lcd.print("CarbonMonoxide");
    lcd.setCursor(5, 1);
    lcd.print(*(float *)value);
    lcd.print("PPM");
  }
  else if (sensorType == dustSensor)
  {
    Serial.print("LCD Dust : ");
    Serial.print(*(float *)value);
    Serial.println("ug/m^3");

    lcd.clear();
    lcd.setCursor(2, 0); // ��, ��
    lcd.print("*Micro_Dust*");
    lcd.setCursor(3, 1);
    lcd.print(*(float *)value);
    lcd.print("ug/");
    lcd.write(3);
    lcd.write(4);
  }
  else if (sensorType == pressureSensor_SCL || pressureSensor_SDA)
  {
    Serial.print("LCD ");
    Serial.println(((PressureSensor *)value)->CAbuffer);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write(1);
    lcd.print(((PressureSensor *)value)->CAtemperature);
    lcd.write(2);
    lcd.print("C ");
    lcd.print("A:");
    lcd.print(((PressureSensor *)value)->CAaltitude);
    lcd.print("M");
    lcd.setCursor(0, 1);
    lcd.print("Pres:");
    lcd.print(((PressureSensor *)value)->CApressure);
    lcd.print("PA");
  }
}

// function that executes whenever data is received from master
void receiveEvent(int howMany)
{
  while (0 < Wire.available())
  {
    char c = Wire.read(); // receive byte as a character //
    //Serial.print(c);      // print the character //
    buf += c;
  }
  //Serial.println(); // to newline //
}

// function that executes whenever data is requested from master
void requestEvent()
{
  char c_tmp[32] = {0}; // ������ ������ ���� ���� ����
  String s_tmp = "";    // ���� ������ ���� ���� ���� ����

  if (buf == "Connect...") // I2C ���� Ȯ��
  {
    Wire.write("Connect Success");
    Serial.println("Connect Success");
  }
  else if (buf == "Update Status") // ���������� ������Ʈ ���� Ȯ��
  {
    Serial.println("Receive : " + buf);
    if (update == true)
    {
      Wire.write("TR");
      Serial.println("Transmission Update Status : TR");
    }
    else if (update == false)
    {
      Wire.write("FS");
      Serial.println("Transmission Update Status : FS");
    }
  }
  else if (buf == "Data Update Complete") // ������ ������Ʈ �Ϸ� �ǵ�� ������ update�� false�� ����
  {
    Serial.println("Data Transfer Completed !");
    update = false;
  }
  else if (buf == "GET TPH DataSize") // �½�������
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.ths.humi);  // ����
    sensorData += String(clt_dt.ths.temp); // �µ�
    s_tmp = String(sensorData.length());   // ���� ������ ���� ���� ����
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // ���� ������ ���� ���� ������ ���� �迭�� ����
    Wire.write(c_tmp);                            // ����
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET TPH")
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // ������ ���� ������ ���� ������ ���� �迭�� ����
    Serial.println("Transmission TPH : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET UTW DataSize") // �����ļ���
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.distance); // �����ļ��� �Ÿ�
    s_tmp = String(sensorData.length());  // ���� ������ ���� ���� ����
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // ���� ������ ���� ���� ������ ���� �迭�� ����
    Wire.write(c_tmp);                            // ����
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET UTW")
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // ������ ���� ������ ���� ������ ���� �迭�� ����
    Serial.println("Transmission UTW : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET DST DataSize") // �̼���������
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.dustDensity); // �̼����� ��
    s_tmp = String(sensorData.length());     // ���� ������ ���� ���� ����
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // ���� ������ ���� ���� ������ ���� �迭�� ����
    Wire.write(c_tmp);                            // ����
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET DST")
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // ������ ���� ������ ���� ������ ���� �迭�� ����
    Serial.println("Transmission DST : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET GAS DataSize") // ��������
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.ppm);     // �������� ��
    s_tmp = String(sensorData.length()); // ���� ������ ���� ���� ����
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // ���� ������ ���� ���� ������ ���� �迭�� ����
    Wire.write(c_tmp);                            // ����
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET GAS")
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // ������ ���� ������ ���� ������ ���� �迭�� ����
    Serial.println("Transmission GAS : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET ILM DataSize") // ��������
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.illumin_Value); // �������� ��
    s_tmp = String(sensorData.length());       // ���� ������ ���� ���� ����
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // ���� ������ ���� ���� ������ ���� �迭�� ����
    Wire.write(c_tmp);                            // ����
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET ILM")
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // ������ ���� ������ ���� ������ ���� �迭�� ����
    Serial.println("Transmission ILM : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET PSS DataSize") // ���м���
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.CAtemperature); // �µ�
    sensorData += String(clt_dt.CApressure);   // ���
    sensorData += String(clt_dt.CAaltitude);   // ��
    s_tmp = String(sensorData.length());       // ���� ������ ���� ���� ����
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // ���� ������ ���� ���� ������ ���� �迭�� ����
    Wire.write(c_tmp);                            // ����
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
  }
  else if (buf == "GET PSS")
  {
    digitalWrite(ledRED, HIGH); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // ������ ���� ������ ���� ������ ���� �迭�� ����
    Serial.println("Transmission PSS : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(ledRED, LOW); // LED OFF
    update = false;
  }
  buf = ""; // ���� �ʱ�ȭ
}
