//processes web server and interface page

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>


#include "main.h"
#include "configuration.h"


const char* ssid = NET_SSID;
const char* password = NET_PASSWORD;

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);


bool SwitchStatus = false;


//forward declarations
void handleOnConnect();
void handle_switchOn();
void handle_NotFound();
String SendHTML(uint8_t switchStat);

void initNetwork(void)
{
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);


    server.on("/", handleOnConnect);
    server.on("/pushSwitch", handle_switchOn);
    server.onNotFound(handle_NotFound);
    server.begin();


}

void handleNetwork(void)
{
    server.handleClient();
}



void handleOnConnect() {

  server.send(200, "text/html", SendHTML(SwitchStatus)); 
}

void handle_switchOn() {
    SwitchStatus = !SwitchStatus;
    server.send(200, "text/html", SendHTML(SwitchStatus)); 
}


void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

bool getSwitchState(void)
{
    return(SwitchStatus);
}


String SendHTML(uint8_t switchStat){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>LED Control</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
   if(switchStat)
  {ptr +="<p>LED1 Status: ON</p><a class=\"button button-off\" href=\"/\">OFF</a>\n";}
  else
  {ptr +="<p>LED1 Status: OFF</p><a class=\"button button-on\" href=\"/pushSwitch\">ON</a>\n";}


  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}