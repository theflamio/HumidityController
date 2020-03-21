#include <Adafruit_DHT_Particle.h>
#include <LiquidCrystal_I2C_Spark.h>
#include "application.h"
#include <Wire.h>
//#include "MQTT.h"



void setup();
void loop();
void Updatelcd(float humid,float temp,float setPointHumidity);
float ReadTemp();
float ReadHumid();
void callback(char* topic, byte* payload, unsigned int length);
float GetSetPointFromUser();
void VentilatorControl(float setPoint,float humid);
SYSTEM_THREAD(ENABLED);

#define VENTILATORONE D4

#define VENTILATORTWO D5

#define DHTPIN D2 //Data read pin 

#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
  
LiquidCrystal_I2C lcd(0x27,20,4); //set the LCD address to 0x27 for a 20 chars and 4 line display

/**
 * if want to use IP address,
 * byte server[] = { XXX,XXX,XXX,XXX };
 * MQTT client(server, 1883, callback);
 * want to use domain name,
 * exp) iot.eclipse.org is Eclipse Open MQTT Broker: https://iot.eclipse.org/getting-started
 * MQTT client("iot.eclipse.org", 1883, callback);
 **/
//MQTT mqttmclient("server_name", 1883, callback);

void setup() 
{
    // Setup LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print(" Ventilator Control");
    
    dht.begin();

    pinMode(VENTILATORONE,OUTPUT);
    pinMode(VENTILATORTWO,OUTPUT);

    //Setup Mqtt server
    //MqttSetup();
}


void loop()
{ 
  // Mqtt loop
  //if (mqttmclient.isConnected())
  //      mqttmclient.loop();

  //read temperatur and humidity
  delay(2000);
  float temp = ReadTemp();
  float humid = ReadHumid();

  //Control ventilation
  float setPointHumidity = GetSetPointFromUser();
  VentilatorControl(setPointHumidity , humid);

  // update lcd screen
  Updatelcd(humid,temp,setPointHumidity);
  PublishTempHumid(temp,humid);
}

void Updatelcd(float humid,float temp,float setPointHumidity)
{
  //Humidity
  lcd.setCursor(0, 1);
  lcd.print("Humid   : ");
  lcd.setCursor(9, 1);
  lcd.print(humid);
  lcd.setCursor(15, 1);
  lcd.print("%");
  // Temperature
  lcd.setCursor(0, 2);
  lcd.print("Temp    : ");
  lcd.setCursor(9, 2);
  lcd.print(temp);
  lcd.setCursor(15, 2);
  lcd.print("Cel");
  // Humid Setpoint
  lcd.setCursor(0, 3);
  lcd.print("SP Humid: ");
  lcd.setCursor(9, 3);
  lcd.print(setPointHumidity);
  lcd.setCursor(15, 3);
  lcd.print("%");
  delay(100);
}

void PublishTempHumid(float temp, float humid)
{
    Particle.publish("Temperature",String::format("%.2f",temp) , PUBLIC);
    Particle.publish("Humidity",String::format("%.2f",humid) , PUBLIC);
    //mqttmclient.publish("outTopic/message","hello world");
    //mqttmclient.publish("outTopic/message","hello world");
}

float ReadTemp()
{
  float t = dht.getTempCelcius();
  
  return t;
}

float ReadHumid()
{
  float h = dht.getHumidity();
  
  return h;
}

float GetSetPointFromUser()
{
  float val = 50;
  return val;
}

void VentilatorControl(float setPoint, float humid)
{
  if (humid >= setPoint) {
    digitalWrite(VENTILATORONE,LOW);
    digitalWrite(VENTILATORTWO,LOW);
  }else if(humid < (setPoint - 3))
  {
    digitalWrite(VENTILATORONE,HIGH);
    digitalWrite(VENTILATORTWO,HIGH); 
  }
  else {

  }
/*
  void MqttSetup()
  {
    // connect to the server
    client.connect("sparkclient");

    // publish/subscribe
    if (client.isConnected()) {
        client.publish("outTopic/message","hello world");
        client.subscribe("inTopic/message");
    }
  }
  */
}
