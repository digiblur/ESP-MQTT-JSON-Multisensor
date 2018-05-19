
/*
  Thanks to Bruh Automation for getting me hooked on the ESP8266 world for coding and creating my own sensors! https://github.com/bruhautomation
   - corbanmailloux for providing a great framework for implementing flash/fade with HomeAssistant https://github.com/corbanmailloux/esp-mqtt-rgb-led
  
  - Support for the ESP8266 boards. 
        - You can add it to the board manager by going to File -> Preference and pasting http://arduino.esp8266.com/stable/package_esp8266com_index.json (2.3.0 used) into the Additional Board Managers URL field.
        - Next, download the ESP8266 dependencies by going to Tools -> Board -> Board Manager and searching for ESP8266 and installing it.

  - Flash settings:
      - NodeMCU 1.0 (ESP-12E Module)
      - CPU Freq 80mhz
      - Flash Size 4M (1M SPIFFS)
      - Uploads Speed 115200
  
  - You will also need to download the follow libraries by going to Sketch -> Include Libraries -> Manage Libraries
      - DHT sensor library 
      - Adafruit unified sensor
      - PubSubClient
      - ArduinoJSON

- To Do:
    - WifiManager
    - Saving configs to SPIFFS JSON and remove config.h file
    - Implement long press to button 1
    - Clean up publish code branches
    - Saved state of extLED1
    - Control states of intLED1 and intLED2Pin
    - Add extLED2 (single color - hitting GPIO max)

*/


#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <FS.h>

// Set configuration defults for LED, pins, WiFi, MQTT, etc in the following file:  (if you pulled the Git, rename the sample_config.h to config.h to compile the code)
#include "config.h"

/**************************** PIN DEFINITIONS ********************************************/
const int redPin = D1;
const int greenPin = D2;
const int bluePin = D3;

#define pirPin    D5
#define DHTPin    D7
#define DHTType   DHT22
#define luxPin    A0
#define intLED1Pin   LED_BUILTIN  // D0
#define intLED2Pin   D4
#define but1Pin   D6
#define intLED1on  LOW
#define intLED1off HIGH
#define intLED2on  LOW
#define intLED2off HIGH

/**************************** SENSOR DEFINITIONS *******************************************/
float luxValue;
int lux;
float calcLux;
float diffLux = 25;

float diffTemp = 0.1;
float tempValue;

float diffHum = 0.1;
float humValue;

float diffFeel = 0.1;
float feelValue;

int pirValue;
int pirStatus;
char motionStatus[10];
char push1Status[10];

int but1Value;
int but1Status;

char ESP_Chip_ID[8];
char NodeID[16];

char LWT_top[40];
char temp_state_top[40];
char feel_state_top[40];
char humid_state_top[40];
char lux_state_top[40];
char pir_state_top[40];
char but1_state_top[40];
char extLED1_set_top[40];
char extLED1_state_top[40];
char intLED1_state_top[40];
char intLED1_set_top[40];
char intLED2_state_top[40];
char intLED2_set_top[40];

char message_buff[100];

const int Buffer_Size = 300;

#define MQTT_MAX_PACKET_SIZE 512

/******************************** GLOBALS for fade/flash *******************************/
byte red = 255;
byte green = 255;
byte blue = 255;
byte brightness = 255;

byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

bool stateOn = false;

bool startFade = false;
unsigned long lastLoop = 0;
int transitionTime = 0;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB;
int redVal, grnVal, bluVal;

bool flash = false;
bool startFlash = false;
int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = red;
byte flashGreen = green;
byte flashBlue = blue;
byte flashBrightness = brightness;

char node_hostname[25]     = def_hostname;
char wifi_ssid[25]    = "";
char wifi_pass[30]    = "";

char mqtt_server[25]  = "";
int  mqtt_port        = 0;
char mqtt_user[25]    = "";
char mqtt_pass[25]    = "";

char cfg_ver[5]       = "dg01";
int  LED1option       = 1;
int  LED2option       = 2;

#define digicfg  "/digicfg.json"

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPin, DHTType);

/********************************** START SETUP*****************************************/
void setup() {

  Serial.begin(115200);

  pinMode(pirPin, INPUT);
  pinMode(DHTPin, INPUT);
  pinMode(luxPin, INPUT);
  pinMode(intLED1Pin, OUTPUT); 
  pinMode(intLED2Pin, OUTPUT);
  pinMode(redPin,OUTPUT);
  pinMode(greenPin,OUTPUT);
  pinMode(bluePin,OUTPUT); 
  pinMode(but1Pin,INPUT_PULLUP); 
  digitalWrite(intLED1Pin, intLED1off);
  digitalWrite(intLED2Pin, intLED2off);
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
  delay(10);

  // initial support for SPIFFS based configuration file and WiFi Manager
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists(digicfg)) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open(digicfg, "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        configFile.close();
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(node_hostname, json["hostname"]);
          strcpy(wifi_ssid, json["wifi_ssid"]);
          strcpy(wifi_pass, json["wifi_pass"]);
          strcpy(mqtt_server, json["mqtt_server"]);
          mqtt_port = json["mqtt_port"];
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          LED1option = json["led1option"];
          LED2option = json["led2option"];
          strcpy(cfg_ver, json["cfg_ver"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    } else {
      // /config not found
      Serial.println("No config found.  Formatting SPIFFS");
      SPIFFS.format();
      Serial.println("Creating new config file.");
      
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["hostname"] = node_hostname;
      json["wifi_ssid"] = wifi_ssid;
      json["wifi_pass"] = wifi_pass;
      json["mqtt_server"] = mqtt_server;
      json["mqtt_port"] = mqtt_port;
      json["mqtt_user"] = mqtt_user;
      json["mqtt_pass"] = mqtt_pass;
      json["led1option"] = LED1option;
      json["led2option"] = LED2option;
      json["cfg_ver"] = cfg_ver;
      File configFile = SPIFFS.open(digicfg, "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }

      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
    //end save
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  sprintf(ESP_Chip_ID, "%06X", ESP.getChipId());
  sprintf(NodeID, def_hostname, ESP_Chip_ID);

// setup topics w/ NODEID 
  sprintf(LWT_top, LWT_topic, NodeID);
  sprintf(extLED1_state_top, extLED1_state_topic, NodeID);
  sprintf(extLED1_set_top, extLED1_set_topic, NodeID);
  sprintf(temp_state_top, temp_state_topic, NodeID);
  sprintf(feel_state_top, feel_state_topic, NodeID);
  sprintf(humid_state_top, humid_state_topic, NodeID);
  sprintf(lux_state_top, lux_state_topic, NodeID);
  sprintf(pir_state_top, pir_state_topic, NodeID);
  sprintf(but1_state_top, but1_state_topic, NodeID);
  sprintf(intLED1_state_top, intLED1_state_topic, NodeID);
  sprintf(intLED1_set_top, intLED1_set_topic, NodeID);
  sprintf(intLED2_state_top, intLED2_state_topic, NodeID);
  sprintf(intLED2_set_top, intLED2_set_topic, NodeID);

// OTA Flash Sets
  ArduinoOTA.setPort(OTAport);
  ArduinoOTA.setHostname(NodeID);
  ArduinoOTA.setPassword((const char *)OTApassword);

  Serial.print("Starting Node: ");
  Serial.println(String(NodeID));

  setup_wifi();

  client.setServer(def_mqtt_server, def_mqtt_port);
  client.setCallback(callback);

  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
//  Serial.println("Ready");
//  Serial.print("IP: ");
//  Serial.println(WiFi.localIP());
  reconnect();
}


/********************************** START SETUP WIFI*****************************************/
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting ");
  Serial.print(NodeID);
  Serial.print(" to ");
  Serial.println(def_wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.hostname(NodeID);
  WiFi.begin(def_wifi_ssid, def_wifi_password);

  bool cycleInd = false;
  // toggle on board LEDs as WiFi comes up
  while (WiFi.status() != WL_CONNECTED) {
    if (cycleInd) {
       digitalWrite(intLED1Pin, intLED1off);
       delay(25);
       digitalWrite(intLED1Pin, intLED1on);
       delay(25);
       digitalWrite(intLED1Pin, intLED1off);
       delay(25);
       digitalWrite(intLED1Pin, intLED1on);
       delay(25);
       digitalWrite(intLED1Pin, intLED1off);
       digitalWrite(intLED2Pin, intLED2on);
       cycleInd = false;
    }
    else {
       digitalWrite(intLED2Pin, intLED2off);
       delay(25);
       digitalWrite(intLED2Pin, intLED2on);
       delay(25);
       digitalWrite(intLED2Pin, intLED2off);
       delay(25);
       digitalWrite(intLED2Pin, intLED2on);
       delay(25);
       digitalWrite(intLED2Pin, intLED2off);
       digitalWrite(intLED1Pin, intLED1on);
       cycleInd = true;
       Serial.print(".");
    }
    delay(50);
  }
  Serial.println("");
  Serial.print("WiFi connected - ");
  Serial.print("IP: ");
  Serial.print(WiFi.localIP());
  long rssi = WiFi.RSSI();
  Serial.print(" - RSSI: ");
  Serial.println(rssi);

}

/********************************** START CALLBACK*****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (!processJson(message)) {
    return;
  }

  if (stateOn) {
    // Update lights
    realRed = map(red, 0, 255, 0, brightness);
    realGreen = map(green, 0, 255, 0, brightness);
    realBlue = map(blue, 0, 255, 0, brightness);
  }
  else {
    realRed = 0;
    realGreen = 0;
    realBlue = 0;
  }

  startFade = true;
  inFade = false; // Kill the current fade

  sendState(5);
}



/********************************** START PROCESS JSON*****************************************/
bool processJson(char* message) {
  StaticJsonBuffer<Buffer_Size> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      stateOn = false;
    }
  }

  // If "flash" is included, treat RGB and brightness differently
  if (root.containsKey("flash")) {
    flashLength = (int)root["flash"] * 1000;

    if (root.containsKey("brightness")) {
      flashBrightness = root["brightness"];
    }
    else {
      flashBrightness = brightness;
    }

    if (root.containsKey("color")) {
      flashRed = root["color"]["r"];
      flashGreen = root["color"]["g"];
      flashBlue = root["color"]["b"];
    }
    else {
      flashRed = red;
      flashGreen = green;
      flashBlue = blue;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
  else { // Not flashing
    flash = false;

    if (root.containsKey("color")) {
      red = root["color"]["r"];
      green = root["color"]["g"];
      blue = root["color"]["b"];
    }

    if (root.containsKey("brightness")) {
      brightness = root["brightness"];
    }

    if (root.containsKey("transition")) {
      transitionTime = root["transition"];
    }
    else {
      transitionTime = 0;
    }
  }

  return true;
}

bool sendpub(char* topic, char* mqmess, bool retain = true) {
    char myPubMsg[80];
    sprintf(myPubMsg,"%s = %s",topic,mqmess);
    Serial.println(myPubMsg);
    return client.publish(topic, mqmess, retain);
}

/********************************** START SEND STATE*****************************************/
void sendState(int topnum) {
// clean this code up and optimize
  if (topnum == 1 || topnum == 2) {
       float newfeelValue = dht.computeHeatIndex(tempValue, humValue, IsFahrenheit);
       if (checkBoundSensor(newfeelValue, feelValue, diffFeel)) {
         feelValue = newfeelValue;
         char result2[5]; 
         dtostrf(feelValue, 3, 1, result2); 
         sendpub(feel_state_top,result2,true);
     }     
  }
  if (topnum == 1) {
     char result1[5]; 
     dtostrf(tempValue, 3, 1, result1); 
     sendpub(temp_state_top,result1,true);
  }
  if (topnum == 2) {
     char result3[5]; 
     dtostrf(humValue, 3, 1, result3); 
     sendpub(humid_state_top,result3,true);
  }  
  if (topnum == 3) {
     char result4[16];
     itoa(lux, result4, 10);
     sendpub(lux_state_top,result4,true);
  }
  if (topnum == 4) {
     sendpub(pir_state_top,motionStatus,true);
  }
  if (topnum == 5) {
     StaticJsonBuffer<Buffer_Size> jsonBuffer;

     JsonObject& root = jsonBuffer.createObject();

     root["state"] = (stateOn) ? on_cmd : off_cmd;
     JsonObject& color = root.createNestedObject("color");
     color["r"] = red;
     color["g"] = green;
     color["b"] = blue;
     char buffer[root.measureLength() + 1];
     root.printTo(buffer, sizeof(buffer));
     sendpub(extLED1_state_top,buffer,true);
  }
  if (topnum == 6) {
     sendpub(but1_state_top,push1Status,true);
  }
  
}

// Leave this in for future testing comparing built in function

/*
 * Calculate Heat Index value AKA "Real Feel"
 * NOAA heat index calculations taken from
 * http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
 */
//float calculateHeatIndex(float humidity, float temp) {
//  float heatIndex= 0;
//  if (temp >= 80) {
//    heatIndex = -42.379 + 2.04901523*temp + 10.14333127*humidity;
//    heatIndex = heatIndex - .22475541*temp*humidity - .00683783*temp*temp;
//    heatIndex = heatIndex - .05481717*humidity*humidity + .00122874*temp*temp*humidity;
//    heatIndex = heatIndex + .00085282*temp*humidity*humidity - .00000199*temp*temp*humidity*humidity;
//  } else {
//     heatIndex = 0.5 * (temp + 61.0 + ((temp - 68.0)*1.2) + (humidity * 0.094));
//  }
//
//  if (humidity < 13 && 80 <= temp <= 112) {
//     float adjustment = ((13-humidity)/4) * sqrt((17-abs(temp-95.))/17);
//     heatIndex = heatIndex - adjustment;
//  }
//
//  return heatIndex;
//}


/********************************** START SET COLOR *****************************************/
void setColor(int inR, int inG, int inB) {
  analogWrite(redPin, inR);
  analogWrite(greenPin, inG);
  analogWrite(bluePin, inB);

  Serial.print("Setting LEDs: ");
  Serial.print("r: ");
  Serial.print(inR);
  Serial.print(", g: ");
  Serial.print(inG);
  Serial.print(", b: ");
  Serial.println(inB);
}



/********************************** START RECONNECT*****************************************/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
//    if (client.connect(NodeID, def_mqtt_user, def_mqtt_password)) {
    if (client.connect(NodeID, def_mqtt_user, def_mqtt_password, LWT_top, 1,1,"Offline")) {
      client.publish(LWT_top,"Online", true);
      Serial.println("connected");
      client.subscribe(extLED1_set_top);
      setColor(0, 0, 0);
      sendState(5);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


/********************************** START MAIN LOOP***************************************/
void loop() {

  ArduinoOTA.handle();
  
  if (!client.connected()) {
    // reconnect();
    software_Reset();
  }
  client.loop();

  if (!inFade) {

    float newTempValue = dht.readTemperature(IsFahrenheit); 
    float newHumValue = dht.readHumidity();

    //PIR CODE
    pirValue = digitalRead(pirPin); //read state of the PIR

    if (pirValue == LOW && pirStatus != 1) {
      strcpy(motionStatus, pir_off_str);
      digitalWrite(intLED1Pin, intLED1off);
      digitalWrite(intLED2Pin, intLED2on);
      sendState(4);
      pirStatus = 1;
    }
    else if (pirValue == HIGH && pirStatus != 2) {
      strcpy(motionStatus, pir_on_str);
      digitalWrite(intLED1Pin, intLED1on);
      digitalWrite(intLED2Pin, intLED2off);
      sendState(4);
      pirStatus = 2;
    }
    delay(100);

    // Read state off Button1
    but1Value = digitalRead(D6);

    if (but1Value == LOW && but1Status != 1) {
      strcpy(push1Status, but1_on_str);
      sendState(6);
      but1Status = 1;
    }
    else if (but1Value == HIGH && but1Status != 2) {
      strcpy(push1Status, but1_off_str);
      sendState(6);
      but1Status = 2;
    }

    // check temp difference - do we need to update the status
    if (checkBoundSensor(newTempValue, tempValue, diffTemp)) {
      tempValue = newTempValue;
      sendState(1);
    }

    // check humidity difference - do we need to update the status
    if (checkBoundSensor(newHumValue, humValue, diffHum)) {
      humValue = newHumValue;
      sendState(2);
    }

    // read the LUX sensor
    int newLUX = analogRead(luxPin);
    // check LUX difference - do we need to update the status
    if (checkBoundSensor(newLUX, lux, diffLux)) {
      lux = newLUX;
      sendState(3);
    }
  }

  if (flash) {
    if (startFlash) {
      startFlash = false;
      flashStartTime = millis();
    }

    if ((millis() - flashStartTime) <= flashLength) {
      if ((millis() - flashStartTime) % 1000 <= 500) {
        setColor(flashRed, flashGreen, flashBlue);
      }
      else {
        setColor(0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue);
      }
    }
    else {
      flash = false;
      setColor(realRed, realGreen, realBlue);
    }
  }

  if (startFade) {
    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      setColor(realRed, realGreen, realBlue);

      redVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;

      startFade = false;
    }
    else {
      loopCount = 0;
      stepR = calculateStep(redVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);

      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
      if (loopCount <= 1020) {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);

        setColor(redVal, grnVal, bluVal); // Write current values to LED pins

        Serial.print("Loop count: ");
        Serial.println(loopCount);
        loopCount++;
      }
      else {
        inFade = false;
      }
    }
  }
}




/**************************** START TRANSITION FADER *****************************************/
// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS

  The program works like this:
  Imagine a crossfade that moves the red LED from 0-10,
    the green from 0-5, and the blue from 10 to 7, in
    ten steps.
    We'd want to count the 10 steps and increase or
    decrease color values in evenly stepped increments.
    Imagine a + indicates raising a value by 1, and a -
    equals lowering it. Our 10 step fade would look like:

    1 2 3 4 5 6 7 8 9 10
  R + + + + + + + + + +
  G   +   +   +   +   +
  B     -     -     -

  The red rises from 0 to 10 in ten steps, the green from
  0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.

  In the real program, the color percentages are converted to
  0-255 values, and there are 1020 steps (255*4).

  To figure out how big a step there should be between one up- or
  down-tick of one of the LED values, we call calculateStep(),
  which calculates the absolute gap between the start and end values,
  and then divides that gap by 1020 to determine the size of the step
  between adjustments in the value.
*/
int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero,
    step = 1020 / step;          //   divide by 1020
  }

  return step;
}

/* The next function is calculateVal. When the loop value, i,
   reaches the step size appropriate for one of the
   colors, it increases or decreases the value of that color by 1.
   (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i) {
  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;
    }
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    }
  }

  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  }
  else if (val < 0) {
    val = 0;
  }

  return val;
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
Serial.print("resetting");
ESP.reset(); 
}
