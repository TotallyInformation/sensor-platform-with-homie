/*
  Generic sensor platform firmware for Wemos D1 Mini (ESP8266) using Homie as a base.
  
  Author: Julian Knight (Totally Information), 
  
  Dependencies: 
  - Homie v2 http://marvinroger.github.io/homie-esp8266/2.0.0-beta.1
    - ArduinoJson >= 5.0.8 https://github.com/bblanchon/ArduinoJson
    - Bounce2 https://github.com/thomasfredericks/Bounce2
    - ESPAsyncTCP >= c8ed544 https://github.com/me-no-dev/ESPAsyncTCP
    - AsyncMqttClient https://github.com/marvinroger/async-mqtt-client
  * config.json file loaded to the ESP's filing system

  @see http://marvinroger.github.io/homie-esp8266/2.0.0-beta.1/quickstart/getting-started/

  TO DO:
  - Add PIR
  - Add pressure
  - Add sound
  - Add NeoPixel code
  - Add optional for sensors, change custom settings to include pin numbers, etc.
  - Inputs: change sensor interval
*/

 // --- Libraries to include ---- //
#include <Homie.h>
// Adafruit DHT Arduino library // https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>
#include <DHT_U.h>
// I2C (Wire) Library
#include <Wire.h>  // I2C, DH1750 Light sensor
#include <BH1750FVI.h>

// --- UPDATE FIRMWARE NAME/VERSION --- //
#define FWNAME "sensors"
#define FWVERSION "0.2.1"
// ------------------------------------------------------------ //

// Homie custom variables 
HomieSetting<long> intervalSetting("SENSOR_INTERVAL", "The interval between sensor readings in seconds");
HomieSetting<const char*> sensorList("SENSORS", "The types of sensors on this instance");

// DHT22 Temperature/Humidity
#define DHTPIN D4             // pin DHT22 connected to - For Wemos DHT board, pin 4 (but that also is used for the onboard LED)
#define DHTTYPE DHT22   // DHT 22  (AM2302)
float humidity;
float temperature;
float heatindex;
float dewpoint;
#define tAdj 0 //D1=3, D2=0 // Adjust the temperature down by this amount if using a sensor too close to the microprocessor as on the Wemos hats

// BH1750 (I2C) - Light Sensor
//   Addr_LOW 0x23 // Device address when address pin LOW
//  Addr_HIGH 0x5C // Device address when address pin High
float lux;

const int DEFAULT_SENSOR_INTERVAL = 30;
unsigned long lastSensorSent = 0;

HomieNode sensorNode("sensors", "sensors");

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);
// Initialize the BH1750FVI class
BH1750FVI LightSensor;


void setup() {
  Serial.begin(115200);
  Serial << endl << endl;
  
  Homie_setFirmware(FWNAME, FWVERSION);

  // Default settings - load a config.json to the file system to set differently
  intervalSetting.setDefaultValue(DEFAULT_SENSOR_INTERVAL).setValidator([] (long candidate) {
    return candidate > 0; // setting has to be a long integer >0
  });
  sensorList.setDefaultValue(""); // default to no sensors
  
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);

  sensorNode.advertise("temperature");
  sensorNode.advertise("humidity");
  sensorNode.advertise("light");

  Homie.getLogger() << F("Sensors will be read every ") << intervalSetting.get() << F(" seconds") << endl;
  Homie.getLogger() << F("Sensors on this unit: ") << sensorList.get() << endl;

  Homie.setup();
}

void loop() {
  Homie.loop();
}

// ---- HELPER FUNCTIONS ---- //

void loopHandler() {
  if (millis() - lastSensorSent >= intervalSetting.get() * 1000UL || lastSensorSent == 0) {
    lastSensorSent = millis();
    Homie.getLogger() << F("READING SENSORS") << endl;
    
    read_dht22();
    if (isnan(humidity) || isnan(temperature)) {
      Homie.getLogger() << F("Failed to read from DHT sensor!") << endl;
    } else {
      Homie.getLogger() << F("Temperature: ") << temperature << F(" °C") << endl;
      Homie.getLogger() << "Humidity: " << humidity << "%" << endl;
      sensorNode.setProperty("temperature").send(String(temperature));
      sensorNode.setProperty("humidity").send(String(humidity));
      sensorNode.setProperty("heatindex").send(String(heatindex));
      sensorNode.setProperty("dewpoint").send(String(dewpoint));
    }
    
    read_bh1750();
    if (isnan(humidity) || isnan(temperature)) {
      Homie.getLogger() << F("Failed to read from light sensor!") << endl;
    } else {
      Homie.getLogger() << "Light: " << lux << " LUX" << endl;
      sensorNode.setProperty("light").send(String(lux));
    }
  }
} // ---- End of loopHandler ---- //

void setupHandler() {
  // Do what you want to prepare your sensor
  pinMode(DHTPIN, INPUT);                 // pin DHT22 connected to

  // Initialise Sensors
  dht.begin();
  Configure_BH1750();

   // Initial sensor read
  read_dht22(); read_bh1750();
} // ---- End of SetupHandler ---- //

// Read the DHT22 (Temperature & Humidity)
void read_dht22() {
  // Wait at least 2 seconds seconds between measurements.
  // Done in loopHandler once for all sensors

  // Reading temperature and humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)

  humidity = dht.readHumidity();        // Read humidity as a percent
  temperature = dht.readTemperature();  // Read temperature as Celsius
  
  // Check if any reads failed
  if (isnan(humidity) || isnan(temperature)) {
    //Homie.getLogger() << F("Failed to read from DHT sensor!") << endl;
  } else {
    // Adjust the temperature if using a sensor too close to the microprocessor as on the Wemos hats
    temperature = temperature - tAdj;
  
    // Compute heat index in Celsius (isFahreheit = false)
    heatindex = dht.computeHeatIndex(temperature, humidity, false);

    // Compute dew point in Celsius
    dewpoint = dewPointFast(temperature, humidity);
  }
} // ---- End of Read Sensor ---- //

// Light
void Configure_BH1750() {
  /*
   *   ##### I2C Addresses #####
   *  Addr_LOW 0x23 // Device address when address pin LOW
   *  Addr_HIGH 0x5C // Device address when address pin High
   *  
   *  ##### Modes ##### 
   *  Continuous_H - 1 lx resolution (16 bit), 120ms sampling time
   *  Continuous_H2 -  0.5 lx resolution (18 bit), 120ms sampling time
   *  Continuous_L - 4 lx resolution (15 bit), 16ms sampling time
   *  OneTime_H
   *  OneTime_H2
   *  OneTime_L
   */
  
  // Start the light sensor by turning on and initializing it.
  LightSensor.Begin(Addr_LOW, Continuous_H2);
 
  // Sets the measurement time register
  // This allows for adjusting the sensitivity
  // It also allows for extension of the sensor's range.
  //LightSensor.SetMTReg(69);
  
  // Scales the sensitivity of the sensor by changing measurement time w/o re-scaling
  // Increasing the sensitivity accounts for something covering sensor (window)
  // Decreasing the sensitivity accounts 
  // The range in sensitivity scaling is 0.45 to 3.68.  Default is 1.00
  //LightSensor.SetSensitivity(1.00);

}

void read_bh1750() {
  // Get the value in lux.  It returns a float that's significant to 1 or 2 digits
  lux = LightSensor.GetLux();// Get Lux value
} // ---- End of read_bh1750 ---- //

// --- Additional Calculations --- //
// See: http://arduinotronics.blogspot.co.uk/2013/12/temp-humidity-w-dew-point-calcualtions.html

// delta max = 0.6544 wrt dewPoint()
// 6.9 x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity) {
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
}

/*
double heatIndex(double tempF, double humidity) {
  double c1 = -42.38, c2 = 2.049, c3 = 10.14, c4 = -0.2248, c5= -6.838e-3, c6=-5.482e-2, c7=1.228e-3, c8=8.528e-4, c9=-1.99e-6  ;
  double T = tempF;
  double R = humidity;
  
  double A = (( c5 * T) + c2) * T + c1;
  double B = ((c7 * T) + c4) * T + c3;
  double C = ((c9 * T) + c8) * T + c6;
  
  double rv = (C * R + B) * R + A;
  return rv;
}
*/
