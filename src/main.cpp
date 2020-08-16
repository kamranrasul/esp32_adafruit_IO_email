/*********
  
  Kamran R.
  Waqas M.
  Nabil E.

  Robbin Law
  Rui Santos
  
*********/

#include <Arduino.h>         // base arduino library
#include <EMailSender.h>     // for sending email
#include "time.h"            // for time setup
#include "AdafruitIO_WiFi.h" // for connecting WiFi
#include "AdafruitIO_Feed.h" // for connecting adafruit IO server
#include "TaskScheduler.h"   // for creating schedules delay

// for updates between the webserver and central esp32
#define RXD2 16
#define TXD2 17

// Local WiFi Credentials
#define WIFI_SSID "Hidden_network"
#define WIFI_PASS "pak.awan.pk"

#define IO_USERNAME "y238student3"
#define IO_KEY "aio_EQQT54PN3hqn8qrEJSpVYIzr8awW"

// To send Email using Gmail use port 465 (SSL) and SMTP Server smtp.gmail.com
#define emailSenderAccount "y238student3@gmail.com"
#define emailSenderPassword "Y238#003"

// email sender initializer
EMailSender emailSend(emailSenderAccount, emailSenderPassword);

// fuctions for operation
void taskInitilizer();
void connectIO();
void printLocalTime();
void sendEmail();
void rcvSerial();
bool detectedChange();
void updateIO();

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

// Must match the sender receiver structure
typedef struct struct_message
{
  int id;            // must be unique for each sender board
  int pinStatus[4];  // for peripheral status
  float temperature; // for storing temperature
  float humidity;    // for storing himmidity
  float pressure;    // for storing pressure
  float altitude;    // for storing altitude

  float temp6050;    // for storing onboard temperature
  float A_values[3]; // for storing accelrometer values
  float G_values[3]; // for storing gyroscope values
} struct_message;

// Create an array with all the structures
struct_message rcdStruct;

// Adafruit.IO feeds
// control pin feeds
AdafruitIO_Feed *pin14Feed = io.feed("controller.pin14");
AdafruitIO_Feed *pin25Feed = io.feed("controller.pin25");
AdafruitIO_Feed *pin26Feed = io.feed("controller.pin26");
AdafruitIO_Feed *pin27Feed = io.feed("controller.pin27");

// BME Sensor feeds
AdafruitIO_Feed *temperatureFeed = io.feed("sensorBME280.temperature");
AdafruitIO_Feed *humidityFeed = io.feed("sensorBME280.humidity");
AdafruitIO_Feed *barpressureFeed = io.feed("sensorBME280.barpressure");
AdafruitIO_Feed *altitudeFeed = io.feed("sensorBME280.altitude");

// MPU6040 Sensor feeds
AdafruitIO_Feed *temp = io.feed("sensorMPU6050.temp");
AdafruitIO_Feed *acx = io.feed("sensorMPU6050.acx");
AdafruitIO_Feed *acy = io.feed("sensorMPU6050.acy");
//AdafruitIO_Feed *acz  = io.feed("sensor_MPU6050.acz");
AdafruitIO_Feed *gyx = io.feed("sensorMPU6050.gyx");
AdafruitIO_Feed *gyy = io.feed("sensorMPU6050.gyy");
//AdafruitIO_Feed *gyz  = io.feed("sensor_MPU6050.gyz");

// time variable setup
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -25362;
const int daylightOffset_sec = 3600;

// time variable
struct tm timeinfo;

// test pins and initialize them to 0
int testPin[4] = {0, 0, 0, 0};

// enabling email server once the update recieves
bool firstTime = true;

// scheduled tasks
Task sendIOUpdate(60000, TASK_FOREVER, &updateIO);
Task emailSender(600000, TASK_FOREVER, &sendEmail);

// Create the scheduler
Scheduler runner;

void setup()
{
  taskInitilizer();
  connectIO();

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
}

void loop()
{
  // Update the MQTT queue and stay connected to Adafruit IO
  io.run();

  // Execute the scheduler runner
  runner.execute();

  if (Serial2.available())
  {
    rcvSerial();
    if (firstTime)
    {
      emailSender.enable();
      firstTime = false;
    }
  }

  if (detectedChange())
  {
    sendIOUpdate.disable();
    sendIOUpdate.enable();
  }
}

// connecting to IO Server
void connectIO()
{
  Serial.printf("Connecting to %s ", WIFI_SSID);
  // Connect to Wifi
  io.connect();
  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" CONNECTED");
}

// initializing the tasks
void taskInitilizer()
{
  // communication with the computer
  Serial.begin(115200);

  // communication with the masater IC
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Start the task scheduler
  runner.init();

  // Add the task to the scheduler
  runner.addTask(sendIOUpdate);
  runner.addTask(emailSender);

  // enable the task
  sendIOUpdate.enable();
}

// time function
void printLocalTime()
{
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "\n%A, %B %d, %Y %H:%M:%S");
}

// email send function
void sendEmail()
{
  // printing at Serial Port
  Serial.println();
  Serial.println("Preparing to send email");
  Serial.println();

  // Sending email here
  EMailSender::EMailMessage message;
  getLocalTime(&timeinfo);

  // setting up subject according to current date and time
  message.subject = "ESP32 HA Update on " +
                    String(timeinfo.tm_year + 1900) + "-" +
                    String(timeinfo.tm_mon + 1) + "-" +
                    String(timeinfo.tm_mday) + " at " +
                    String(timeinfo.tm_hour) + ":" +
                    String(timeinfo.tm_min);

  message.message = String(" *********************_BME280_********************")
                  + String(" Temperature: \n\n")     + String(rcdStruct.temperature) + String(" °C")
                  + String(" **************************************************")
                  + String(" Humidity: ")        + String(rcdStruct.humidity)    + String(" %")
                  + String(" **************************************************")
                  + String(" Pressure: ")        + String(rcdStruct.pressure)    + String(" hPa")
                  + String(" **************************************************")
                  + String(" Altitude: ")        + String(rcdStruct.altitude)    + String(" m")
                  + String(" **************************************************")
                  + String(" Furnace: ")         + String(rcdStruct.pinStatus[0] ? "ON" : "OFF")
                  + String(" **************************************************")
                  + String(" Exhaust: ")         + String(rcdStruct.pinStatus[1] ? "ON" : "OFF")
                  + String(" **************************************************")
                  + String(" Humidifier: ")      + String(rcdStruct.pinStatus[2] ? "ON" : "OFF")
                  + String(" **************************************************")
                  + String(" Light: ")           + String(rcdStruct.pinStatus[3] ? "ON" : "OFF")
                  + String(" *********************_MPU6050_********************")
                  + String(" Temperature: ")     + String(rcdStruct.temp6050) + String(" °C")
                  + String(" **************************************************")
                  + String(" Acceleration: X: ") + String(rcdStruct.A_values[0]) + ", Y: "+ String(rcdStruct.A_values[1]) + ", Z: " + String(rcdStruct.A_values[2]) + " m/s^2"
                  + String(" **************************************************")
                  + String(" Rotation      X: ") + String(rcdStruct.A_values[0]) + ", Y: "+ String(rcdStruct.A_values[1]) + ", Z: " + String(rcdStruct.A_values[2]) + " rad/s";

  EMailSender::Response resp;

  resp = emailSend.send("kamran.rasul@gmail.com", message);

  Serial.printf("Sending status: %s", resp.status ? "Success" : "Failure");
  Serial.println();
}

// for receiving data from Serial port
void rcvSerial()
{
  printLocalTime();

  rcdStruct.temperature = Serial2.parseFloat();
  Serial.println(rcdStruct.temperature);
  Serial2.read();

  rcdStruct.humidity = Serial2.parseFloat();
  Serial.println(rcdStruct.humidity);
  Serial2.read();

  rcdStruct.pressure = Serial2.parseFloat();
  Serial.println(rcdStruct.pressure);
  Serial2.read();

  rcdStruct.altitude = Serial2.parseFloat();
  Serial.println(rcdStruct.altitude);
  Serial2.read();

  for (int i = 0; i < 4; i++)
  {
    rcdStruct.pinStatus[i] = Serial2.parseInt();
    Serial.println(rcdStruct.pinStatus[i]);
    Serial2.read();
  }

  rcdStruct.temp6050 = Serial2.parseFloat();
  Serial.println(rcdStruct.temp6050);
  Serial2.read();

  for (int i = 0; i < 3; i++)
  {
    rcdStruct.A_values[i] = Serial2.parseFloat();
    Serial.println(rcdStruct.A_values[i]);
    Serial2.read();
  }

  for (int i = 0; i < 3; i++)
  {
    rcdStruct.G_values[i] = Serial2.parseFloat();
    Serial.println(rcdStruct.G_values[i]);
    Serial2.read();
  }

  Serial.println("*** Received from Master ***");
  Serial.println();
}

// check if there is any update on the pins
bool detectedChange()
{
  for (int i = 0; i < 4; i++)
  {
    if (testPin[i] != rcdStruct.pinStatus[i])
    {
      // if there is change then update
      for (int j = 0; j < 4; j++)
      {
        testPin[j] = rcdStruct.pinStatus[j];
      }
      // return change flag
      return true;
    }
  }
  // return no change flag
  return false;
}

// update the IO Server
void updateIO()
{
  // updating BME values to IO
  temperatureFeed->save(rcdStruct.temperature);
  humidityFeed->save(rcdStruct.humidity);
  barpressureFeed->save(rcdStruct.pressure);
  altitudeFeed->save(rcdStruct.altitude);

  // updating pin state to IO
  pin14Feed->save(rcdStruct.pinStatus[0]);
  pin25Feed->save(rcdStruct.pinStatus[1]);
  pin26Feed->save(rcdStruct.pinStatus[2]);
  pin27Feed->save(rcdStruct.pinStatus[3]);

  // updating MPU6050 values to IO
  temp->save(rcdStruct.temp6050);
  acx->save(rcdStruct.A_values[0]);
  acy->save(rcdStruct.A_values[1]);
  //acz->save(rcdStruct.A_values[2]);
  gyx->save(rcdStruct.G_values[0]);
  gyx->save(rcdStruct.G_values[1]);
  //gyx->save(rcdStruct.G_values[2]);

  Serial.println("IO Feed updated...");
}