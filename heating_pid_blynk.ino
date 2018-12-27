
/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <SimpleTimer.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AutoPID.h>

#define PWM_PIN 5
#define ONEWIRE_PIN 4

#include "config.h"
/*
const char blynk_auth[] = "put_key_here";
const char wifi_ssid[] = "network_ssid";
const char wifi_pass[] = "password";
DeviceAddress devaddr_mixed = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
DeviceAddress devaddr_hot = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
DeviceAddress devaddr_cool = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };
*/

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);

double tmp_read, temp_set, temp_mixed, temp_hot, temp_cool;
double kp=200, ki=0.1, kd=350;
double bb_on=10, bb_off=50;
double tmp_d, offset=200, actuator;
bool act=true;

SimpleTimer timer;

AutoPID myPID(&temp_mixed, &temp_set, &actuator, -offset, 1024-offset, kp, ki, kd);


WidgetTerminal terminal(V10);
char term_buff[100];

BLYNK_WRITE(V10)
{
  if (strcmp(param.asStr(), "gpid")==0)
  {
    sprintf(term_buff, "pid: %lf,%lf,%lf\n", kp, ki, kd);
    terminal.print(term_buff);
  }
  else if (strcmp(param.asStr(), "gint")==0)
  {
    sprintf(term_buff, "integral: %lf\n", myPID.getIntegral());
    terminal.print(term_buff);
  }
  else if (strcmp(param.asStr(), "gbb")==0)
  {
    sprintf(term_buff, "bangbang: %lf - %lf\n", bb_on, bb_off);
    terminal.print(term_buff);
  }
  else if (strcmp(param.asStr(), "goff")==0)
  {
    sprintf(term_buff, "offset: %lf\n", offset);
    terminal.print(term_buff);
  }
  else if (strcmp(param.asStr(), "stop")==0)
  {
    act = false;
    terminal.println("stopped");
  }
  else if (strcmp(param.asStr(), "start")==0)
  {
    act = true;
    terminal.println("stopped");
  }
  else if (sscanf(param.asStr(), "spid %lf,%lf,%lf", &kp, &ki, &kd) == 3)
  {
    myPID.setGains(kp, ki, kd);
    terminal.println("pid set");
  }
  else if (sscanf(param.asStr(), "sint %lf", &tmp_d) == 1)
  {
    myPID.setIntegral(tmp_d);
    terminal.println("integral set");
  }
  else if (sscanf(param.asStr(), "soff %lf", &offset) == 1)
  {
    myPID.setOutputRange(0-offset, 1024-offset);
    terminal.println("offset set");
  }
  else if (sscanf(param.asStr(), "sbb %lf,%lf", &bb_on, &bb_off) == 2)
  {
    myPID.setBangBang(bb_on, bb_off);
    terminal.println("bangbang set");
  }
  else
  {
    terminal.println("?");
  }
  terminal.flush();
}


// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 1
BLYNK_WRITE(V1)
{
  temp_set = param.asDouble();
  Blynk.virtualWrite(V0, temp_set);
  // Serial.print("temp set to: ");
  // Serial.println(temp_set);
}



void setup()
{
  temp_set = 32.0;
  myPID.setBangBang(bb_on, bb_off);
  // Debug console
  Serial.begin(115200);
  Serial.print("Start\n");
  pinMode(PWM_PIN, OUTPUT);

  Blynk.begin(blynk_auth, wifi_ssid, wifi_pass);
  while (Blynk.connect() == false) {
    // Wait until connected
  }
  sensors.begin();
  sensors.setResolution(12);
  sensors.setWaitForConversion(false);
  timer.setInterval(750L, send_temp);
  Blynk.virtualWrite(V0, temp_set);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  Serial.print("Connected\n");
  terminal.println("start!");
}

void loop()
{
  Blynk.run();
  if (act) {
    timer.run();
    myPID.run();
  }
  analogWrite(PWM_PIN, (int)(actuator+offset));
}

void send_temp() {
  tmp_read = sensors.getTempC(devaddr_mixed);
  if (tmp_read != -127.0) {
    temp_mixed = tmp_read;
    Blynk.virtualWrite(V2, temp_mixed);
  }
  tmp_read = sensors.getTempC(devaddr_hot);
  if (tmp_read != -127.0) {
    temp_hot = tmp_read;
    Blynk.virtualWrite(V3, temp_hot);
  }
  tmp_read = sensors.getTempC(devaddr_cool);
  if (tmp_read != -127.0) {
    temp_cool = tmp_read;
    Blynk.virtualWrite(V4, temp_cool);
  }
  Blynk.virtualWrite(V5, actuator/10.24);
  sensors.requestTemperatures();
}
