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
#define WIFI_SSID "YOUR SSID"
#define WIFI_PASS "YOUR PASS"

#define IO_USERNAME "YOUR IO ID"
#define IO_KEY "YOUR ID KEY"

// To send Email using Gmail use port 465 (SSL) and SMTP Server smtp.gmail.com
#define emailSenderAccount "YOUR SENDER EMAIL"
#define emailSenderPassword "YOUR PASS"
// #define smtpServer "smtp.gmail.com"
// #define smtpServerPort 465

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

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  int id; // must be unique for each sender board
  int pinStatus[4];
  float temperature;
  float humidity;
  float pressure;
  float altitude;
} struct_message;

// Create an array with all the structures
struct_message rcdStruct;

// Adafruit.IO feeds
// Avoid underscores in the feed names, they cause problems with groupings.
AdafruitIO_Feed *temperatureFeed = io.feed("project.temperature");
AdafruitIO_Feed *humidityFeed = io.feed("project.humidity");
AdafruitIO_Feed *barpressureFeed = io.feed("project.barpressure");
AdafruitIO_Feed *altitudeFeed = io.feed("project.altitude");
AdafruitIO_Feed *pin14Feed = io.feed("project.pin14");
AdafruitIO_Feed *pin25Feed = io.feed("project.pin25");
AdafruitIO_Feed *pin26Feed = io.feed("project.pin26");
AdafruitIO_Feed *pin27Feed = io.feed("project.pin27");

// time variable setup
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -25362;
const int daylightOffset_sec = 3600;

// time variable
struct tm timeinfo;

// test pins and initialize them to 0
int testPin[4] = {0, 0, 0, 0};

// scheduled tasks
Task sendIOUpdate(30000, TASK_FOREVER, &updateIO);
Task emailSender(86400000, TASK_FOREVER, &sendEmail);

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
  // Connect to Wifi
  io.connect();
  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
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
  emailSender.enable();
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

  message.message = String("Temperature: ") + String(rcdStruct.temperature) + String(" °C") +
                    String("Humidity:    ") + String(rcdStruct.humidity) + String(" %") +
                    String("Pressure:    ") + String(rcdStruct.pressure) + String(" hPa") +
                    String("Altitude:    ") + String(rcdStruct.altitude) + String(" m") +
                    String("Furnace:     ") + String(rcdStruct.pinStatus[0] ? "ON" : "OFF") +
                    String("Exhaust:     ") + String(rcdStruct.pinStatus[1] ? "ON" : "OFF") +
                    String("Humidifier:  ") + String(rcdStruct.pinStatus[2] ? "ON" : "OFF") +
                    String("Light:       ") + String(rcdStruct.pinStatus[3] ? "ON" : "OFF");

  EMailSender::Response resp;

  resp = emailSend.send("kamran.rasul@gmail.com", message);

  Serial.printf("Sending status: %s", resp.status ? "Success" : "Failure");
  Serial.println();

  resp = emailSend.send("wmalik@ualberta.ca", message);

  Serial.printf("Sending status: %s", resp.status ? "Success" : "Failure");
  Serial.println();

  resp = emailSend.send("nabil_elassaad@yahoo.com", message);

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
    Serial2.read();
    Serial.println(rcdStruct.pinStatus[i]);
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
  temperatureFeed->save(rcdStruct.temperature);
  humidityFeed->save(rcdStruct.humidity);
  barpressureFeed->save(rcdStruct.pressure);
  altitudeFeed->save(rcdStruct.altitude);

  pin14Feed->save(rcdStruct.pinStatus[0]);
  pin25Feed->save(rcdStruct.pinStatus[1]);
  pin26Feed->save(rcdStruct.pinStatus[2]);
  pin27Feed->save(rcdStruct.pinStatus[3]);
}