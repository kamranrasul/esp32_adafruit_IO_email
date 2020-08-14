#ifndef EMailSenderKey_h
#define EMailSenderKey_h

// Uncomment if you use esp8266 core <= 2.4.2
//#define ARDUINO_ESP8266_RELEASE_2_4_2

#define ENABLE_ATTACHMENTS

// Uncomment to enable printing out nice debug messages.
//#define EMAIL_SENDER_DEBUG

// Define where debug output will be printed.
#define DEBUG_PRINTER Serial

#define NETWORK_ESP32 (4)
#define NETWORK_ESP32_ETH (5)

#define DEFAULT_EMAIL_NETWORK_TYPE_ESP32 NETWORK_ESP32

#endif
