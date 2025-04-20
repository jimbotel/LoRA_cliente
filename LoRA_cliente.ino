/**
 * @file LoRaP2P_TX.ino
 * @author rakwireless.com
 * @brief Transmitter node for LoRa point to point communication
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */

/*
Wisblock meshtastic starter kit based on:
RAK19007 ( WisBlock Base Board 2nd Gen)
RAK4631 (LPWAN )
RAK1921 (OLED display)
RAK12500 ( GNSS Location Sensor Module)
RAK1904 (3-Axis Acceleration Sensor Module)
RAK1901 (temp&hum sensor)
*/

// RAK4631 (LPWAN )
#include <Arduino.h>
#include <ArduinoJson.h>  //http://librarymanager/All#ArduinoJson
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
// RAK1921 (OLED display) 
#include <Wire.h>
#include <U8g2lib.h>		   // Click to install library: http://librarymanager/All#u8g2
// RAK1901 (temp&hum sensor)
#include "SparkFun_SHTC3.h" 		//Click here to get the library: http://librarymanager/All#SparkFun_SHTC3
// RAK12500 ( GNSS Location Sensor Module)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
// RAK1904 (3-Axis Acceleration Sensor Module)
#include "SparkFunLIS3DH.h" //http://librarymanager/All#SparkFun-LIS3DH

// RAK4631 (LPWAN )
// Function declarations
void OnTxDone(void);
void OnTxTimeout(void);

#define CLIENTNAME "01_ElNota"

#ifdef NRF52_SERIES
#define LED_BUILTIN 35
#endif

// Define LoRa parameters
#define RF_FREQUENCY 868300000	// Hz
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;
static uint8_t TxdBuffer[64];

JsonDocument jsonMessage;
char field[32] = {0};
// int msgnum = 0;

// RAK1901 (temp&hum sensor)
SHTC3 g_shtc3;						      // Declare an instance of the SHTC3 class
// RAK1921 (OLED display) 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
char data[32] = {0};
// RAK12500 ( GNSS Location Sensor Module)
SFE_UBLOX_GNSS g_myGNSS;
long g_lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
// RAK1904 (3-Axis Acceleration Sensor Module)
LIS3DH SensorTwo(I2C_MODE, 0x18);

void setup()
{
  // RAK 19007 WisBlock Base: enable power on 3V3_S for SLOT A-D and IO-SLOT
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 1);

  jsonMessage["cli"] = CLIENTNAME;

	// Initialize Serial for debug output
	time_t timeout = millis();
	Serial.begin(115200);
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
		{
            delay(100);
        }
        else
        {
            break;
        }
	}

  // RAK1901 (temp&hum sensor)
	Wire.begin();
	Serial.println("shtc3 init");
	Serial.print("Beginning sensor. Result = "); // Most SHTC3 functions return a variable of the type "SHTC3_Status_TypeDef" to indicate the status of their execution
	errorDecoder(g_shtc3.begin());              // To start the sensor you must call "begin()", the default settings use Wire (default Arduino I2C port)
	Wire.setClock(400000);						          // The sensor is listed to work up to 1 MHz I2C speed, but the I2C clock speed is global for all sensors on that bus so using 400kHz or 100kHz is recommended
	Serial.println();

	if (g_shtc3.passIDcrc)                      // Whenever data is received the associated checksum is calculated and verified so you can be sure the data is true
	{					   						                    // The checksum pass indicators are: passIDcrc, passRHcrc, and passTcrc for the ID, RH, and T readings respectively
		Serial.print("ID Passed Checksum. ");
		Serial.print("Device ID: 0b");
		Serial.println(g_shtc3.ID, BIN); 		      // The 16-bit device ID can be accessed as a member variable of the object
	}
	else
	{
		Serial.println("ID Checksum Failed. ");
	}

	// RAK1921 (OLED display) 
	u8g2.begin();

  // RAK12500 ( GNSS Location Sensor Module)
  Serial.println("GPS ZOE-M8Q Example(I2C)");
  if (g_myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  g_myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  g_myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  get_gps_data();

	// RAK4631 (LPWAN )
	// Initialize LoRa chip.
	lora_rak4630_init();
	// Initialize the Radio callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = NULL;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = NULL;
	RadioEvents.RxError = NULL;
	RadioEvents.CadDone = NULL;
	// Initialize the Radio
	Radio.Init(&RadioEvents);
	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);
	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
	send();

  // RAK1904 (3-Axis Acceleration Sensor Module)
	if (SensorTwo.begin() != 0)
	{
		Serial.println("Problem starting the sensor at 0x18.");
	}
	else
	{
		Serial.println("Sensor at 0x18 started.");
		// Set low power mode
		uint8_t data_to_write = 0;
		SensorTwo.readRegister(&data_to_write, LIS3DH_CTRL_REG1);
		data_to_write |= 0x08;
		SensorTwo.writeRegister(LIS3DH_CTRL_REG1, data_to_write);
		delay(100);
		data_to_write = 0;
		SensorTwo.readRegister(&data_to_write, 0x1E);
		data_to_write |= 0x90;
		SensorTwo.writeRegister(0x1E, data_to_write);
		delay(100);
	}

}

void loop()
{
  // Put your application tasks here, like reading of sensors,
	shtc3_read_data();
	delay(6000);
  get_gps_data();
	delay(6000);
  lis3dh_read_data();
  delay(6000);
}

// RAK4631 (LPWAN )
/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
	Serial.println("OnTxDone");
	delay(5000);
	send();
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
	Serial.println("OnTxTimeout");
}

void send()
{
  static uint8_t output[200];
  int count = 0;
  // Add values in the document
  // snprintf(field,sizeof(field), "%04d", msgnum);
  // jsonMessage["ct"] = field;
  count = serializeJson(jsonMessage, output, sizeof(output)-1);
  Serial.print("jsonMessage = ");
  serializeJson(jsonMessage, Serial);
  Serial.println("");
  Serial.print("Count = ");
  Serial.println(count);

  count = serializeMsgPack(jsonMessage, output, sizeof(output)-1);
  Serial.print("jsonMessage MsgPack = ");
  serializeMsgPack(jsonMessage, Serial);
  Serial.println("");
  Serial.print("Count = ");
  Serial.println(count);

	Radio.Send(output, count);
  // msgnum++;
}

// RAK1901 (temp&hum sensor)
void errorDecoder(SHTC3_Status_TypeDef message)   // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
{
  switch (message)
  {
    case SHTC3_Status_Nominal:
      Serial.print("Nominal");
      break;
    case SHTC3_Status_Error:
      Serial.print("Error");
      break;
    case SHTC3_Status_CRC_Fail:
      Serial.print("CRC Fail");
      break;
    default:
      Serial.print("Unknown return code");
      break;
  }
}

void shtc3_read_data(void)
{
	float Temperature = 0;
	float Humidity = 0;
	
	g_shtc3.update();
	if (g_shtc3.lastStatus == SHTC3_Status_Nominal) // You can also assess the status of the last command by checking the ".lastStatus" member of the object
	{
		Temperature = g_shtc3.toDegC();	// "toDegF" and "toDegC" return the temperature as a floating point number in deg F and deg C respectively
		Humidity = g_shtc3.toPercent(); // "toPercent" returns the percent humidity as a floating point number
		Serial.print("RH = ");
		Serial.print(Humidity); 			      
		Serial.print("% (checksum: ");
		if (g_shtc3.passRHcrc) 	// Like "passIDcrc" this is true when the RH value is valid from the sensor (but not necessarily up-to-date in terms of time)
			Serial.print("pass");
		else
			Serial.print("fail");
		Serial.print("), T = ");
		Serial.print(Temperature); 	
		Serial.print(" deg C (checksum: ");
		if (g_shtc3.passTcrc) // Like "passIDcrc" this is true when the T value is valid from the sensor (but not necessarily up-to-date in terms of time)
			Serial.print("pass");
		else
			Serial.print("fail");
		Serial.println(")");

	  // RAK1921 (OLED display) 
	  u8g2.clearBuffer();					// clear the internal memory
	  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font

  	memset(data, 0, sizeof(data));
	  sprintf(data, "T=%.1fC", Temperature);
	  u8g2.drawStr(3, 15, data);

	  memset(data, 0, sizeof(data));
	  snprintf(data, 64, "RH=%.0f%%", Humidity);
	  u8g2.drawStr(3, 30, data);

  	u8g2.sendBuffer(); // transfer internal memory to the display

    // Json Message
    snprintf(field,sizeof(field), "%.1f", Temperature);
    jsonMessage["temp"] = field;
    snprintf(field,sizeof(field), "%.0f", Humidity);
    jsonMessage["RH"] = field;
	}
	else
	{
    Serial.print("Update failed, error: ");
		errorDecoder(g_shtc3.lastStatus);
		Serial.println();
	}
}

void get_gps_data(void) 
{
  if (g_myGNSS.getGnssFixOk())
  {
    byte fix_type = g_myGNSS.getFixType ();	// Get the fix type
    char fix_type_str[32] = { 0 };
    if (fix_type == 0)
      sprintf (fix_type_str, "No Fix");
    else if (fix_type == 1)
      sprintf (fix_type_str, "Dead reckoning");
    else if (fix_type == 2)
      sprintf (fix_type_str, "Fix type 2D");
    else if (fix_type == 3)
      sprintf (fix_type_str, "Fix type 3D");
    else if (fix_type == 4)
      sprintf (fix_type_str, "GNSS fix");
    else if (fix_type == 5)
      sprintf (fix_type_str, "Time fix");
    Serial.print(F("GnssFix OK!! Fix Type: "));
    Serial.println(fix_type_str);
  }
  else 
  {
    Serial.println(F("GnssFix KO!"));
  }

  if(g_myGNSS.getTimeValid())
    Serial.println("getTime Valid!");
  else
    Serial.println("getTime NO Valid!");
  if(g_myGNSS.getDateValid())
    Serial.println("getDate Valid!");
  else
    Serial.println("getDate NO Valid!");

  char date_time[100] = { 0 };
  date_time[0] = '\0';
  char temp[12]; // Temporary buffer for each value
  sprintf(temp, "%04u-", g_myGNSS.getYear()); 
  strcat(date_time, temp); 
  sprintf(temp, "%02u-", g_myGNSS.getMonth()); 
  strcat(date_time, temp); 
  // sprintf(temp, "%lu/", g_myGNSS.getTimeOfWeek()); 
  // strcat(date_time, temp); 
  sprintf(temp, "%02uT", g_myGNSS.getDay()); 
  strcat(date_time, temp); 
  sprintf(temp, "%02u:", g_myGNSS.getHour()); 
  strcat(date_time, temp); 
  sprintf(temp, "%02u:", g_myGNSS.getMinute()); 
  strcat(date_time, temp); 
  sprintf(temp, "%02u", g_myGNSS.getSecond()); 
  strcat(date_time, temp); 

  Serial.print(F("Date Time: "));
  Serial.println(date_time);
  // Json message
  jsonMessage["time"] = date_time;

  // Get the current latitude in degrees
  // Returns a long representing the number of degrees *10^-7
  int32_t latitude = g_myGNSS.getLatitude();
  float latf = latitude * 0.0000001;
  Serial.print(F("Lat: "));
  Serial.print(latitude);

  // Get the current longitude in degrees
  // Returns a long representing the number of degrees *10^-7
  int32_t longitude = g_myGNSS.getLongitude();
  float lonf = longitude * 0.0000001;
  Serial.print(F(" Long: "));
  Serial.print(longitude);
  Serial.print(F(" (degrees * 10^-7)"));

  int32_t altitude = g_myGNSS.getAltitude();
  int alti = altitude/1000;
  Serial.print(F(" Alt: "));
  Serial.print(altitude);
  Serial.print(F(" (mm)"));
  
  int32_t speed = g_myGNSS.getGroundSpeed();
  int spei = speed*3.6/1000;
  Serial.print(F(" Speed: "));
  Serial.print(speed);
  Serial.print(F(" (mm/s)"));

  int32_t heading = g_myGNSS.getHeading();
  float headf = heading * 0.00001;
  Serial.print(F(" Heading: "));
  Serial.print(heading);
  Serial.print(F(" (degrees * 10^-5)"));

  byte SIV = g_myGNSS.getSIV();
  Serial.print(F(" SIV: "));
  Serial.print(SIV);

  // accuracy = (float) (g_myGNSS.getHorizontalDOP() / 100.0);
  uint16_t HDOP = g_myGNSS.getHorizontalDOP();
  Serial.print(F(" HorizontalDOP: "));
  Serial.println(HDOP);

  // RAK1921 (OLED display) 
  u8g2.clearBuffer();					// clear the internal memory
  //u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
  u8g2.setFont(u8g2_font_6x12_tf); // choose a suitable font

  memset(data, 0, sizeof(data));
  snprintf(data, 64, "Lat: %.7f\xb0", latf);
  u8g2.drawStr(3, 12, data);

  memset(data, 0, sizeof(data));
  snprintf(data, 64, "Lon: %.7f\xb0", lonf);
  u8g2.drawStr(3, 24, data);

  memset(data, 0, sizeof(data));
  snprintf(data, 64, "Alt: %dm Vel: %dkm/h", alti, spei);
  u8g2.drawStr(3, 36, data);
  
  // memset(data, 0, sizeof(data));
  // snprintf(data, 64, "Vel: %dm/s", spei);
  // u8g2.drawStr(3, 48, data);

  memset(data, 0, sizeof(data));
  snprintf(data, 64, "Heading: %.2f\xb0", headf);
  u8g2.drawStr(3, 48, data);

  memset(data, 0, sizeof(data));
  snprintf(data, 64, "Sat In View: %d", SIV);
  u8g2.drawStr(3, 60, data);

  u8g2.sendBuffer(); // transfer internal memory to the display

  // Json message
  snprintf(field,sizeof(field), "%ld", latitude);
  jsonMessage["lat"] = field;
  snprintf(field,sizeof(field), "%ld", longitude);
  jsonMessage["lon"] = field;
  snprintf(field,sizeof(field), "%ld", altitude);
  jsonMessage["alt"] = field;
  snprintf(field,sizeof(field), "%ld", speed);
  jsonMessage["speed"] = field;
  snprintf(field,sizeof(field), "%ld", heading);
  jsonMessage["hdg"] = field;
  snprintf(field,sizeof(field), "%u", SIV);
  jsonMessage["SIV"] = field;

}

// RAK1904 (3-Axis Acceleration Sensor Module)
void lis3dh_read_data()
{
  // read the sensor value
  uint8_t cnt = 0;
  float xg = SensorTwo.readFloatAccelX();
  float yg = SensorTwo.readFloatAccelY();
  float zg = SensorTwo.readFloatAccelZ();
  Serial.print(" X(g) = ");
  Serial.print(xg, 4);
  Serial.print(" Y(g) = ");
  Serial.print(yg, 4);
  Serial.print(" Z(g)= ");
  Serial.println(zg, 4);

  // RAK1921 (OLED display) 
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font

  memset(data, 0, sizeof(data));
  sprintf(data, "**Aceleracion**");
  u8g2.drawStr(3, 15, data);
 
  memset(data, 0, sizeof(data));
  sprintf(data, "X(g)=%.3f", xg);
  u8g2.drawStr(3, 30, data);

  memset(data, 0, sizeof(data));
  sprintf(data, "Y(g)=%.3f", yg);
  u8g2.drawStr(3, 45, data);

  memset(data, 0, sizeof(data));
  sprintf(data, "Z(g)=%.3f", zg);
  u8g2.drawStr(3, 60, data);

  u8g2.sendBuffer(); // transfer internal memory to the display

  // Json message
  snprintf(field,sizeof(field), "%.3f", xg);
  jsonMessage["xg"] = field;
  snprintf(field,sizeof(field), "%.3f", yg);
  jsonMessage["yg"] = field;
  snprintf(field,sizeof(field), "%.3f", zg);
  jsonMessage["zg"] = field;

}

// void draw_lcd(char *inpdata, int x, int y)
// {
// 	memset(data, 0, sizeof(data));
// 	sprintf(data, "T=%.2fC", temp);
// 	u8g2.drawStr(3, 15, data);
// }

