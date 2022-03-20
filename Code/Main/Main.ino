/*
          𝐐𝐮𝐚𝐢𝐥 𝐈𝐧𝐜𝐮𝐛𝐚𝐭𝐨𝐫
  KMITL - Feedback Control
  November 2021 (2021-11-22 02:00:00)
  Made with ♥ by Piyawud Koonmanee (Micro)
  
               ------
            𝐔𝐩𝐥𝐨𝐚𝐝 𝐒𝐞𝐭𝐮𝐩
Board:            "DOIT ESP32 DEVKIT V1"
Upload speed:     "115200"
Flash frequency:  "40 MHz"
Flash mode:       "QIO"
Partition scheme: "Default" (change to "Huge APP" if it's not working)
               ------
Schematics: soon!
*/


//##// -> Libraries <- //##//

#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <time.h>



//##// -> Constants <- //##//

#define DEVICE "ESP32"
#define DHT1_PIN 18 
#define DHT1_TYPE DHT11   // DHT 11
#define DHT2_PIN 19
#define DHT2_TYPE DHT22   // DHT 22  (AM2302)
#define LED_PIN 2         
#define RELAY_PIN 23      // Bulb
#define BUTTON_PIN 4      // Quail turn counter 
#define FanIN1 16         // Fan IN1
#define FanIN2 17         // Fan IN2
#define FanENA 5          // Fan ENA

#define WIFI_SSID "Never gonna give you up"
#define WIFI_PASSWORD "Never gonna let you down"
#define INFLUXDB_URL "Never gonna run around and desert you"
#define INFLUXDB_TOKEN "Never gonna make you cry"
#define INFLUXDB_ORG "Never gonna say goodbye"
#define INFLUXDB_BUCKET "Never gonna tell a lie and hurt you"
#define TZ_INFO "ICT-7"   // Bangkok Timezone



//##// -> Kalman Constants <- //##//

#define Emea_Temp1 2         // DHT11 - Sensor error: ±2°C
#define Emea_Humi1 5        // DHT11 - Sensor error: ±5%
#define Emea_Temp2 0.5         // DHT22 - Sensor error: ±0.5°C
#define Emea_Humi2 2         // DHT22 - Sensor error: ±2%
#define Kalman_Samples 50    // Estimate initial value using 50 data samples



//##// -> Initialization <- //##//

WiFiMulti wifiMulti;
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Point sensor("incubator_sensor");

DHT dht1(DHT1_PIN, DHT1_TYPE); 
DHT dht2(DHT2_PIN, DHT2_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);



//##// -> Variables <- //##//

float humi1, humi2;       // Stores humidity value
float temp1, temp2;       // Stores temperature value
int counter = 0;          // Quail turn counter
int button_state = 0;
int prev_button_state = 0;
int light_status = 0, fan_power = 0;
char timeHour[3], timeMin[3], timeSec[3];  // Stores time
const int pwmChannel = 0;   
float limit = 37.5;   // Prefer temperature for Incubator



//##// -> Kalman Variables <- //##//

    //# DHT11 #//
float mea_Temp1 = 0, mea_Humi1 = 0;
float KG_Temp1 = 0, KG_Humi1= 0;
float est_Temp1 = 0,est_Humi1 = 0;
float Eest_Temp1 = Emea_Temp1, Eest_Humi1 = Emea_Humi1;
float est_Temp1_0 = 0, est_Humi1_0 = 0;
float Eest_Temp1_0 = Emea_Temp1, Eest_Humi1_0 = Emea_Humi1;

    //# DHT22 #//
float mea_Temp2 = 0, mea_Humi2 = 0;
float KG_Temp2 = 0, KG_Humi2= 0;
float est_Temp2 = 0,est_Humi2 = 0;
float Eest_Temp2 = Emea_Temp2, Eest_Humi2 = Emea_Humi2;
float est_Temp2_0 = 0, est_Humi2_0 = 0;
float Eest_Temp2_0 = Emea_Temp2, Eest_Humi2_0 = Emea_Humi2;



//##// -> Main function <- //##//

void setup()
{
    Serial.begin(115200);

    //# Sensors Initialization #//
    dht1.begin();
    dht2.begin();
    lcd.begin();  
    lcd.backlight();
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_PIN,INPUT_PULLUP);
    pinMode(FanIN1, OUTPUT);
    pinMode(FanIN2, OUTPUT);
    pinMode(FanENA, OUTPUT);

    setupFan();
    
    estKalman1();   // Estimate initial value of DHT11
    estKalman2();   // Estimate initial value of DHT22
 
    setupWiFi();
    checkInflux();  // Check InfluxDB connection
    delay(3000);
}

void loop()
{   
    turnCounter();
    checkTime();
    
    readDHT();
    
    Kalman1();
    resetKalman1();
    Kalman2();
    resetKalman2();
    
    printTemp1();
    printTemp2();
    
    enableFan();
    enableBulb();
    bulbChecker();
    
    sendData();             //Send data to InfluxDB
    checkWiFi();
}



//##// -> Custom Function <- //##//

void readDHT()
{
    //# Read data from both DHT sensors #//
    humi1 = dht1.readHumidity();
    temp1= dht1.readTemperature();
    humi2 = dht2.readHumidity();
    temp2= dht2.readTemperature();
}

void estKalman1()
{
    //# Estimate initial value for Kalman on DHT-11 #//
    int count = 0;
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("Est. Kalman1");
    Serial.println("Estimating Kalman 1 ...");
  
    for(int i = 0 ; i < Kalman_Samples ; i++)
    {
      readDHT();
      est_Temp1_0 += temp1;
      est_Humi1_0 += humi1;
      delay(500);
      
      if((i%10) == 0)
      {
        Serial.print(".....");
        Serial.print(i*2.499);
        Serial.print("%");
        lcd.setCursor(5,1);
        lcd.print(i*2.499); 
        lcd.print("%");
        
        if(count == (Kalman_Samples/10))
        {
          count = 0;
        }
        count++; 
      }
    }
    
    est_Temp1_0 /= Kalman_Samples;
    est_Humi1_0 /= Kalman_Samples; 
    Serial.println();
    Serial.println("-- Ready! --");
    Serial.println();
    lcd.clear();
    lcd.setCursor(5,0); 
    lcd.print("Ready!");
    delay(1000);
}

void Kalman1()
{
    //# Kalman equation for DHT-11 #//
    mea_Temp1 = temp1;
    KG_Temp1 = (Eest_Temp1) / (Eest_Temp1 + Emea_Temp1);
    est_Temp1 = (est_Temp1_0) + (KG_Temp1 * (mea_Temp1 - est_Temp1_0));
    Eest_Temp1 = (1 - KG_Temp1) * (Eest_Temp1_0);
  
    mea_Humi1 = humi1;
    KG_Humi1 = (Eest_Humi1) / (Eest_Humi1 + Emea_Humi1);
    est_Humi1 =(est_Humi1_0) + (KG_Humi1 * (mea_Humi1 - est_Humi1_0));
    Eest_Humi1 = (1 - KG_Humi1) * (Eest_Humi1_0);
}

void resetKalman1() 
{   
    //# Change initial value to last estimated value for Kalman on DHT-11 #//
    est_Temp1_0 = est_Temp1;
    est_Humi1_0 = est_Humi1;
    Eest_Temp1_0 = Eest_Temp1;
    Eest_Humi1_0 = Eest_Humi1;
}

void estKalman2()
{
    //# Estimate initial value for Kalman on DHT-22 #//
    int count = 0;
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("Est. Kalman2");
    Serial.println("Estimating Kalman 2 ...");
  
    for(int i = 0 ; i < Kalman_Samples ; i++ )
    {
      readDHT();
      est_Temp2_0 += temp2;
      est_Humi2_0 += humi2;
      delay(500);
      
      if((i%10) == 0)
      {
        Serial.print(".....");
        Serial.print(i*2.499);
        Serial.print("%");
        lcd.setCursor(5,1);
        lcd.print(i*2.499); 
        lcd.print("%");
        
        if(count == (Kalman_Samples/10) )
        {
          count = 0;
        }
        count++; 
      }
    }
    
    est_Temp2_0 /= Kalman_Samples;
    est_Humi2_0 /= Kalman_Samples; 
    Serial.println();
    Serial.println("-- Ready! --");
    Serial.println();
    lcd.clear();
    lcd.setCursor(5,0); 
    lcd.print("Ready!");
    delay(1000);
}

void Kalman2()
{
    //# Kalman equation for DHT-22 #//
    mea_Temp2 = temp2;
    KG_Temp2 = (Eest_Temp2) / (Eest_Temp2 + Emea_Temp2);
    est_Temp2 = (est_Temp2_0) + (KG_Temp2 * (mea_Temp2 - est_Temp2_0));
    Eest_Temp2 = (1 - KG_Temp2) * (Eest_Temp2_0);
  
    mea_Humi2 = humi2;
    KG_Humi2 = (Eest_Humi2) / (Eest_Humi2 + Emea_Humi2);
    est_Humi2 =(est_Humi2_0) + (KG_Humi2 * (mea_Humi2 - est_Humi2_0));
    Eest_Humi2 = (1 - KG_Humi2) * (Eest_Humi2_0);

    Serial.println(KG_Temp2);
}

void resetKalman2()
{
    //# Change initial value to last estimated value for Kalman on DHT-11 #//
    est_Temp2_0 = est_Temp2;
    est_Humi2_0 = est_Humi2;
    Eest_Temp2_0 = Eest_Temp2;
    Eest_Humi2_0 = Eest_Humi2;
}

void setupFan()
{
    //# Setup PWM fan #//
    ledcSetup(pwmChannel, 30000, 8); // 30000 = PWM frequency, 8 = PWM resolution (0-255)
    ledcAttachPin(FanENA, pwmChannel);
    digitalWrite(FanIN2, HIGH); // Out 2 = Live wire
    digitalWrite(FanIN1, LOW);  // Out 1 = Ground wire
}

void setupWiFi()
{
    //# Setup WiFi and sync time #//
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
    configTime(7*60*60, 0, "pool.ntp.org");   // GMT+7 Time , No DST

    lcd.clear();
    lcd.setCursor(1, 0); 
    lcd.print("Connecting to:");
    lcd.setCursor(2, 1);
    lcd.print(WIFI_SSID);
        
    Serial.print("Connecting to wifi");
    
    while (wifiMulti.run() != WL_CONNECTED) {
      Serial.print(".");
      delay(100);
    }
    Serial.println();

    sensor.addTag("device", DEVICE);  // Add device tags
  
    //# Sync Time #//
    timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");  // Sync device time
}

void checkInflux()
{
    //# Check InfluxDB connection #//
    if (client.validateConnection()) {
      Serial.print("Connected to InfluxDB: ");
      Serial.println(client.getServerUrl());
      
      lcd.clear();
      lcd.setCursor(2, 0); 
      lcd.print("Connected to");
      lcd.setCursor(4, 1);
      lcd.print("InfluxDB");
    }
    else {
      Serial.print("InfluxDB connection failed: ");
      Serial.println(client.getLastErrorMessage());
      
      lcd.clear();
      lcd.setCursor(3, 0); 
      lcd.print("Connection");
      lcd.setCursor(5, 1);
      lcd.print("Error!");
    }
}

void turnCounter()
{
    //# Quail turn counter using button #//
    button_state = digitalRead(BUTTON_PIN);
    
    if (button_state != prev_button_state){
      if (button_state == LOW){
      counter++;
      Serial.print(counter);
      Serial.println(" Turn");
      Serial.println();
      
      lcd.clear();
      lcd.setCursor(3, 0); 
      lcd.print("Completed!");
      lcd.setCursor(5, 1);
      lcd.print(counter);
      lcd.setCursor(7, 1);
      lcd.print("Turn");
      delay(2500);
      }
    }
    prev_button_state = button_state;
}

void checkTime()
{
    //# Reset Quail turn counter every midnight #//
    struct tm timeinfo;
    
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      return;
    }
    
    strftime(timeHour,3, "%H", &timeinfo);
    strftime(timeMin,3, "%M", &timeinfo);
    strftime(timeSec,3, "%S", &timeinfo);
    
    if (strcmp(timeHour,"00") == 0){
      if ((strcmp(timeMin,"00") == 0) && counter != 0){
        counter = 0;
        prev_button_state = 0;
        Serial.println("Counter Reset!");
      }
    }
}

void printTemp1()
{   
    //# Print DHT-11 data on serial monitor and LCD #//
    Serial.print("  > Temp1: ");
    Serial.print(est_Temp1);
    Serial.print("°C, Humi1: ");
    Serial.print(est_Humi1);
    Serial.println("%");
    
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("OUT| Temp: ");
    lcd.print(est_Temp1,1);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("SDE| Humi: ");
    lcd.print(est_Humi1,1);
    lcd.print("%");
    delay(3500);
}

void printTemp2()
{   
    //# Print DHT-22 data on serial monitor and LCD #//
    Serial.print("  > Temp2: ");
    Serial.print(est_Temp2);
    Serial.print("°C, Humi2: ");
    Serial.print(est_Humi2);
    Serial.println("%");
    Serial.println();

    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("INS| Temp: ");
    lcd.print(est_Temp2,1);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("IDE| Humi: ");
    lcd.print(est_Humi2,1);
    lcd.print("%");
    delay(5000);
}

void enableBulb()
{   
    //# Turn on Bulb #//
    if (temp2 > limit-0.1){
      
      digitalWrite(RELAY_PIN, LOW);
    }
    else{
      digitalWrite(RELAY_PIN, HIGH);
    }
}

void enableFan()
{
    //# Turn on Fan #//
    
    /* 37.7°C and higher */
    if (temp2 > limit+0.1){
      ledcWrite(pwmChannel, 160);
      fan_power = 63;
    }
    
    /* 37.6°C */
    else if ((temp2 > limit) and (temp2 < limit+0.2)){
      ledcWrite(pwmChannel, 140);
      fan_power = 55;
    }

    /* 37.5°C */
    else if (temp2 == limit){
      ledcWrite(pwmChannel, 130);
      fan_power = 50;
    }

    /* 37.4°C */
    else if ((temp2 > limit-0.2) and (temp2 < limit)){ //37.4
      ledcWrite(pwmChannel, 120);
      fan_power = 47;
    }

    /* 37.3°C */
    else if ((temp2 > limit-0.3) and (temp2 < limit-0.1)){ //37.3
      ledcWrite(pwmChannel, 100);
      fan_power = 39;
    }

    /* 37.2 °C and lower */
    else{
      ledcWrite(pwmChannel, 90);
      fan_power = 35;
    }
    
    Serial.print("Fan Power:");
    Serial.print(fan_power);
    Serial.println("%");
}

void bulbChecker()
{
    //# Check if Bulb is on #//
    if (digitalRead(RELAY_PIN) == HIGH){
      light_status = 1;
      Serial.println("Light on");
    } else {
      light_status = 0;
      Serial.println("Light off");
    }
    Serial.println();
}

void sendData()
{
    //# Send data to InfluxDB #//
    sensor.clearFields();
    digitalWrite(LED_PIN, HIGH);   // Turn LED on if it's starting to send
    
    sensor.addField("kalman_temperature1", est_Temp1);
    sensor.addField("kalman_humidity1", est_Humi1);
    sensor.addField("kalman_temperature2", est_Temp2);
    sensor.addField("kalman_humidity2", est_Humi2);
    
    sensor.addField("raw_temperature1", temp1);
    sensor.addField("raw_humidity1", humi1);
    sensor.addField("raw_temperature2", temp2);
    sensor.addField("raw_humidity2", humi2);
    
    sensor.addField("turn", counter);
    sensor.addField("light_status", light_status);
    sensor.addField("fan_power", fan_power);
    
    // Print data that we sent on serial monitor
    Serial.print("Writing: ");
    Serial.println(client.pointToLineProtocol(sensor));
    
    digitalWrite(LED_PIN, LOW);   // Turn LED off if it's finished
}

void checkWiFi()
{
    //# Check WiFi connection and reconnect if needed #//
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("Wifi connection lost");
      lcd.clear();
      lcd.setCursor(4, 0); 
      lcd.print("WiFi");
      lcd.setCursor(0, 1);
      lcd.print("Connection Lost!");
    }
  
    //# Check if it can't send data to InfluxDB #//
    if (!client.writePoint(sensor)) {
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
      lcd.clear();
      lcd.setCursor(4, 0); 
      lcd.print("InfluxDB");
      lcd.setCursor(2, 1);
      lcd.print("Write Failed");
    }
}
