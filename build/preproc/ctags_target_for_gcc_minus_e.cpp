# 1 "d:\\neturalData_aws\\neturalData_aws.ino"
# 2 "d:\\neturalData_aws\\neturalData_aws.ino" 2
# 3 "d:\\neturalData_aws\\neturalData_aws.ino" 2
# 4 "d:\\neturalData_aws\\neturalData_aws.ino" 2
# 5 "d:\\neturalData_aws\\neturalData_aws.ino" 2
# 6 "d:\\neturalData_aws\\neturalData_aws.ino" 2
# 7 "d:\\neturalData_aws\\neturalData_aws.ino" 2
# 8 "d:\\neturalData_aws\\neturalData_aws.ino" 2


# 9 "d:\\neturalData_aws\\neturalData_aws.ino"
// Sensor Pin Number






//#define wifi_RX 10              // WIFI 모듈 RX(수신)
//#define wifi_TX 11              // WIFI 모듈 TX(송신)




//#define port    443     // IOT 통신할 포트번호

// I2C Data Type







typedef struct
{
  float temp = 0.0, humi = 0.0; // 온도, 습도 변수
} TemperHumiSensor;

typedef struct
{
  float Ftemperature = 0.0; // 온도 측정 값
  float Fpressure = 0.0; // 기압 측정 값
  float Faltitude = 0.0; // 고도 측정 값

  char CAtemperature[20] = ""; // 온도
  char CApressure[20] = ""; // 기압
  char CAaltitude[20] = ""; // 고도
  char CAbuffer[128] = "";
} PressureSensor;

typedef struct
{
  TemperHumiSensor ths; // 온습도
  char CAtemperature[20] = ""; // 온도(대기압센서)
  char CApressure[20] = ""; // 기압(대기압센서)
  char CAaltitude[20] = ""; // 고도(대기압센서)
  int distance = 0; // 초음파센서 거리
  int illumin_Value = 0; // 조도센서 값
  float ppm = 0.0; // 일산화탄소 값
  float dustDensity = 0.0; // 미세먼지 값
} NaturalData; // 수집된 환경 데이터 정보를 저장 및 업데이트할 구조체

LiquidCrystal_I2C lcd(0x27, 16, 2);
String buf = ""; // I2C 송/수신 버퍼
String sensorData = ""; // 센서 데이터 값
int dataSize = 0; // 데이터의 길이
NaturalData clt_dt; // 환경데이터 정보를 담을 구조체 선언
bool update = false; // 환경 데이터 업데이트 여부(수집완료 true / 수집 중 false)

void setup()
{
  Serial.begin(9600); // 통신 속도 설정
  Serial.println("Start, Arduino!"); // 시리얼 포트 TX로 Hello, Arduino! 출력
  Wire.begin(8); // join i2c bus with address 8 //
  Wire.onReceive(receiveEvent); // register receive event //
  Wire.onRequest(requestEvent); // register request event //
  pinMode(4 /* Red LED*/, 0x1); // LED핀 출력모드
  pinMode(3 /* Green LED*/, 0x1);
  pinMode(5 /* 초음파센서 (Trig) => 출력*/, 0x1); // 초음파센서 출력핀 설정
  pinMode(6 /* 초음파센서 (Echo) => 입력*/, 0x0); // 초음파센서 입력핀 설정
  pinMode(A1 /* 가스센서*/, 0x0);
  initLCD();
  Serial.println("Initializing I2C..."); // I2C 통신 연결확인
  delay(3000);
}

void loop()
{
  void *ptr = 
# 87 "d:\\neturalData_aws\\neturalData_aws.ino" 3 4
             __null
# 87 "d:\\neturalData_aws\\neturalData_aws.ino"
                 ;
  update = false;

  delay(2000);
  digitalWrite(3 /* Green LED*/, 0x1); // LED ON
  printLCD(ptr = TempHum_Check(), 8 /* 온습도 센서*/); // 온습도센서 측정 및 출력
  clt_dt.ths.humi = ((TemperHumiSensor *)ptr)->humi; // 습도 값 업데이트
  clt_dt.ths.temp = ((TemperHumiSensor *)ptr)->temp; // 온도 값 업데이트
  delay(2000);
  digitalWrite(3 /* Green LED*/, 0x0); // LED OFF
  free(ptr);

  digitalWrite(3 /* Green LED*/, 0x1); // LED ON
  printLCD(ptr = UltrasonicWave_Check(), 5 /* 초음파센서 (Trig) => 출력*/); // 초음파센서 측정 및 출력
  clt_dt.distance = *(int *)ptr; // 초음파센서 값 업데이트
  delay(2000);
  digitalWrite(3 /* Green LED*/, 0x0); // LED OFF
  free(ptr);

  digitalWrite(3 /* Green LED*/, 0x1); // LED ON
  printLCD(ptr = Illuminance_Check(A0 /* 조도센서*/), A0 /* 조도센서*/); // 조도센서 측정 및 출력
  clt_dt.illumin_Value = *(int *)ptr; // 조도 값 업데이트
  delay(2000);
  digitalWrite(3 /* Green LED*/, 0x0); // LED OFF
  free(ptr);

  digitalWrite(3 /* Green LED*/, 0x1); // LED ON
  printLCD(ptr = Gas_Check(A1 /* 가스센서*/), A1 /* 가스센서*/); // 일산화탄소센서 측정 및 출력
  clt_dt.ppm = *(float *)ptr; // 일산화탄소 값 업데이트
  delay(2000);
  digitalWrite(3 /* Green LED*/, 0x0); // LED OFF
  free(ptr);

  digitalWrite(3 /* Green LED*/, 0x1); // LED ON
  printLCD(ptr = DustDensity_Check(), 2 /* 미세먼지 센서*/); // 미세먼지센서 측정 및 출력
  clt_dt.dustDensity = *(float *)ptr; // 미세먼지 값 업데이트
  delay(2000);
  digitalWrite(3 /* Green LED*/, 0x0); // LED OFF
  free(ptr);

  digitalWrite(3 /* Green LED*/, 0x1); // LED ON
  printLCD(ptr = Pressure_Check(), A4 /* 대기압센서 SDA*/); // 대기압센서 측정 및 출력
  strcpy(clt_dt.CAtemperature, ((PressureSensor *)ptr)->CAtemperature); // 온도 값 업데이트
  strcpy(clt_dt.CApressure, ((PressureSensor *)ptr)->CApressure); // 기압 값 업데이트
  strcpy(clt_dt.CAaltitude, ((PressureSensor *)ptr)->CAaltitude); // 고도 값 업데이트
  delay(2000);
  digitalWrite(3 /* Green LED*/, 0x0); // LED OFF
  free(ptr);

  update = true; // 환경 데이터 정보 업데이트 완료
  Serial.println("All Sensor Data Update Complete !");
  Serial.println("=========================================================");

  while (update == true) // NodeMcu에서 데이터를 받아갈 수 있도록 대기
    delay(2000);
  buf = "";
}

int *UltrasonicWave_Check() // 초음파 센서 측정
{
  int *distance = (int *)malloc(sizeof(int));
  digitalWrite(5 /* 초음파센서 (Trig) => 출력*/, 0x1);
  delayMicroseconds(30);
  digitalWrite(5 /* 초음파센서 (Trig) => 출력*/, 0x0);

  *distance = pulseIn(6 /* 초음파센서 (Echo) => 입력*/, 0x1) * 340 / 2 / 10000; // echo핀으로 들어온 신호를 마이크로 단위로 변환
  // 음파의 속도 초당 340m / 2 / cm 변환
  delay(1000);

  return distance;
}

TemperHumiSensor *TempHum_Check() // 온습도 센서 측정
{
  DHT11 dht11(8 /* 온습도 센서*/);
  TemperHumiSensor *ths = (TemperHumiSensor *)malloc(sizeof(TemperHumiSensor));
  dht11.read(ths->humi, ths->temp); // 측정
  delay(1000);

  return ths;
}

int *Illuminance_Check(int analogPinNum) // 조도 측정
{
  int *illumin_Value = (int *)malloc(sizeof(int));
  *illumin_Value = analogRead(analogPinNum);
  delay(1000); // 측정 딜레이

  return illumin_Value;
}

float *Gas_Check(int analogPinNum) // 일산화탄소 측정
{
  float RS_gas = 0;
  float ratio = 0;
  float sensor_volt = 0;
  float R0 = 7200.0; // 깨끗한 공기에서의 센서 저항값 (2K옴 기준)
  float gas_Value = analogRead(analogPinNum);

  sensor_volt = gas_Value / 1024 * 5.0;
  RS_gas = (5.0 - sensor_volt) / sensor_volt; // 5V 전압 기준
  ratio = RS_gas / R0; //Replace R0 with the value found using the sketch above
  float x = 1538.46 * ratio;
  float *ppm = (float *)malloc(sizeof(float));
  *ppm = pow(x, -1.709); // PPM값 계산
  delay(1000);

  return ppm;
}

float *DustDensity_Check() // 미세먼지 측정
{
  // 국제 미세먼지농도에 따른 경계단계 기준분류
  // 30ug/m^3 이하 : 좋음 / 30~80ug/m^3 : 보통 / 80~150ug/m^3 : 나쁨 / 150ug/m^3 초과 : 매우 나쁨
  float ratio = 0;
  float concentration = 0;
  float *dustDensity = (float *)malloc(sizeof(float)); // 미세먼지 밀도
  unsigned long sampletime_ms = 2000; // 미세먼지 센서의 측정시간
  float duration = 0.0;
  /* for (int i = 4; i > 0; i--) // 정확한 측정을 위해 여러번 펄스값을 읽음

  {

    duration = pulseIn(dustSensor, LOW); // Pin에서 LOW가 될때 펄스값을 읽음

    delay(500);

  } */
# 212 "d:\\neturalData_aws\\neturalData_aws.ino"
  while (duration == 0) // 정확한 측정을 위해 여러번 펄스값을 읽음
  {
    duration = pulseIn(2 /* 미세먼지 센서*/, 0x0); // Pin에서 LOW가 될때 펄스값을 읽음
    delay(500);
  }
  unsigned long lowpulseoccupancy = lowpulseoccupancy + duration;

  ratio = lowpulseoccupancy / (sampletime_ms * 10.0); // Integer percentage 0=>100
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
  *dustDensity = concentration * 100 / 13000;
  lowpulseoccupancy = 0;

  return dustDensity;
}

PressureSensor *Pressure_Check() // 대기압 측정
{
  Adafruit_BMP085 bmp;
  PressureSensor *ps = (PressureSensor *)malloc(sizeof(PressureSensor));
  if (!bmp.begin(3 /*!< Ultra high-res mode*/))
  {
    Serial.println("BMP180 센서를 찾을 수 없습니다. 연결을 확인해 주세요!");
    return;
  }

  ps->Ftemperature = bmp.readTemperature();
  ps->Fpressure = bmp.readPressure();
  ps->Faltitude = bmp.readAltitude(101600); // 101600 : 인천 송도 해면기압, 101325 : 표준기압

  dtostrf(ps->Ftemperature, 4, 2, ps->CAtemperature);
  dtostrf(ps->Fpressure, 9, 2, ps->CApressure);
  dtostrf(ps->Faltitude, 5, 2, ps->CAaltitude);

  // 온도 압력 고도
  sprintf(ps->CAbuffer, "Temp(.C): %s\tPres(PA): %s\tAlt(M): %s", ps->CAtemperature, ps->CApressure, ps->CAaltitude);
  delay(1000);

  return ps;
}

void initLCD() // LCD 초기화
{
  byte humidityImage[] = {0x04, 0x0E, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E}; // 물방울 아이콘
  byte temperatureImage[] = {0x04, 0x0A, 0x0A, 0x0A, 0x0E, 0x1F, 0x1F, 0x0E}; // 온도 아이콘
  byte doImage[] = {0x1C, 0x14, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00}; // ˚ 아이콘
  byte microImage[] = {0x11, 0x11, 0x11, 0x13, 0x15, 0x18, 0x10, 0x10}; //LCD "m" 이미지
  byte threeImage[] = {0x18, 0x04, 0x18, 0x04, 0x18, 0x00, 0x00, 0x00}; //LCD "3" 이미지

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

void printLCD(void *value, int sensorType) // LCD에 출력, sensorType을 지정하여 각 센서에 맞게 출력
{
  //char printBuffer[254] = "";

  if (sensorType == 8 /* 온습도 센서*/)
  {
    // Serial 출력
    Serial.print("LCD Humidity : ");
    Serial.print(((TemperHumiSensor *)value)->humi);
    Serial.print("% / Temperature : ");
    Serial.print(((TemperHumiSensor *)value)->temp);
    Serial.println(" C");
    // LCD 출력
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
  else if (sensorType == 5 /* 초음파센서 (Trig) => 출력*/)
  {
    Serial.print("LCD ");
    Serial.print(*(int *)value);
    Serial.println("CM");

    lcd.clear();
    lcd.setCursor(2, 0); // 열, 행
    lcd.print("# Distance #");
    lcd.setCursor(6, 1);
    lcd.print(*(int *)value);
    lcd.print("CM");
  }
  else if (sensorType == A0 /* 조도센서*/)
  {
    Serial.print("LCD Illumin : ");
    Serial.println(*(int *)value);

    lcd.clear();
    lcd.setCursor(1, 0); // 열, 행
    lcd.print("*Illumination*");
    lcd.setCursor(6, 1);
    lcd.print(*(int *)value);
    lcd.print("LX");
  }
  else if (sensorType == A1 /* 가스센서*/)
  {
    Serial.print("LCD gas(ppm) : ");
    Serial.println(*(float *)value);

    lcd.clear();
    lcd.setCursor(1, 0); // 열, 행
    lcd.print("CarbonMonoxide");
    lcd.setCursor(5, 1);
    lcd.print(*(float *)value);
    lcd.print("PPM");
  }
  else if (sensorType == 2 /* 미세먼지 센서*/)
  {
    Serial.print("LCD Dust : ");
    Serial.print(*(float *)value);
    Serial.println("ug/m^3");

    lcd.clear();
    lcd.setCursor(2, 0); // 열, 행
    lcd.print("*Micro_Dust*");
    lcd.setCursor(3, 1);
    lcd.print(*(float *)value);
    lcd.print("ug/");
    lcd.write(3);
    lcd.write(4);
  }
  else if (sensorType == A5 /* 대기압센서 SCL*/ || A4 /* 대기압센서 SDA*/)
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
  char c_tmp[32] = {0}; // 데이터 전송을 위한 저장 공간
  String s_tmp = ""; // 센서 데이터 길이 정보 저장 변수

  if (buf == "Connect...") // I2C 연결 확인
  {
    Wire.write("Connect Success");
    Serial.println("Connect Success");
  }
  else if (buf == "Update Status") // 센서데이터 업데이트 유무 확인
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
  else if (buf == "Data Update Complete") // 데이터 업데이트 완료 피드백 받으면 update를 false로 변경
  {
    Serial.println("Data Transfer Completed !");
    update = false;
  }
  else if (buf == "GET TPH DataSize") // 온습도센서
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.ths.humi); // 습도
    sensorData += String(clt_dt.ths.temp); // 온도
    s_tmp = String(sensorData.length()); // 센서 데이터 길이 정보 저장
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // 센서 데이터 길이 정보 전송을 위해 배열에 저장
    Wire.write(c_tmp); // 전송
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET TPH")
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // 수집된 센서 데이터 정보 전송을 위해 배열에 저장
    Serial.println("Transmission TPH : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET UTW DataSize") // 초음파센서
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.distance); // 초음파센서 거리
    s_tmp = String(sensorData.length()); // 센서 데이터 길이 정보 저장
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // 센서 데이터 길이 정보 전송을 위해 배열에 저장
    Wire.write(c_tmp); // 전송
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET UTW")
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // 수집된 센서 데이터 정보 전송을 위해 배열에 저장
    Serial.println("Transmission UTW : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET DST DataSize") // 미세먼지센서
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.dustDensity); // 미세먼지 값
    s_tmp = String(sensorData.length()); // 센서 데이터 길이 정보 저장
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // 센서 데이터 길이 정보 전송을 위해 배열에 저장
    Wire.write(c_tmp); // 전송
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET DST")
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // 수집된 센서 데이터 정보 전송을 위해 배열에 저장
    Serial.println("Transmission DST : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET GAS DataSize") // 가스센서
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.ppm); // 가스센서 값
    s_tmp = String(sensorData.length()); // 센서 데이터 길이 정보 저장
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // 센서 데이터 길이 정보 전송을 위해 배열에 저장
    Wire.write(c_tmp); // 전송
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET GAS")
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // 수집된 센서 데이터 정보 전송을 위해 배열에 저장
    Serial.println("Transmission GAS : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET ILM DataSize") // 조도센서
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.illumin_Value); // 조도센서 값
    s_tmp = String(sensorData.length()); // 센서 데이터 길이 정보 저장
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // 센서 데이터 길이 정보 전송을 위해 배열에 저장
    Wire.write(c_tmp); // 전송
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET ILM")
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // 수집된 센서 데이터 정보 전송을 위해 배열에 저장
    Serial.println("Transmission ILM : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET PSS DataSize") // 대기압센서
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData = String(clt_dt.CAtemperature); // 온도
    sensorData += String(clt_dt.CApressure); // 기압
    sensorData += String(clt_dt.CAaltitude); // 고도
    s_tmp = String(sensorData.length()); // 센서 데이터 길이 정보 저장
    Serial.println("Transmission SensorDataSize : " + s_tmp);
    s_tmp.toCharArray(c_tmp, s_tmp.length() + 1); // 센서 데이터 길이 정보 전송을 위해 배열에 저장
    Wire.write(c_tmp); // 전송
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
  }
  else if (buf == "GET PSS")
  {
    digitalWrite(4 /* Red LED*/, 0x1); // LED ON
    delay(2000);
    Serial.println("Receive : " + buf);
    sensorData.toCharArray(c_tmp, sensorData.length() + 1); // 수집된 센서 데이터 정보 전송을 위해 배열에 저장
    Serial.println("Transmission PSS : " + sensorData);
    Wire.write(c_tmp);
    delay(2000);
    digitalWrite(4 /* Red LED*/, 0x0); // LED OFF
    update = false;
  }
  buf = ""; // 버퍼 초기화
}
