#include <STM32L0.h>
#include "wiring_private.h" 
#include "SparkFun_Ublox_Arduino_Library.h" 

SFE_UBLOX_GPS myGPS;

#define SerialMon  Serial
#define SerialGNSS Serial1

#define GNSS_ENABLE  (5u)  // enable for GNSS 3.0 V LDO
#define GNSS_PPS     (4u)  // 1 Hz fix pulse
#define GNSS_BACKUP  A0  

#define STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL  0x00000000

uint16_t stm32_enable_pin = g_APinDescription[GNSS_ENABLE].pin;
uint16_t stm32_pps_pin    = g_APinDescription[GNSS_PPS].pin;
uint16_t stm32_backup_pin = g_APinDescription[GNSS_BACKUP].pin;
volatile bool pps_irq;

long lastTime = 0;

void int_pps(void) {
  pps_irq = true;
}

void gnssBegin_STM32()
{
  stm32l0_gpio_pin_configure(stm32_backup_pin, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
  stm32l0_gpio_pin_write(stm32_backup_pin, 1);

  stm32l0_gpio_pin_configure(stm32_enable_pin, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
  stm32l0_gpio_pin_write(stm32_enable_pin, 1);

  stm32l0_gpio_pin_configure(stm32_pps_pin, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
  stm32l0_exti_attach(stm32_pps_pin, STM32L0_EXTI_CONTROL_EDGE_FALLING | STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL, (stm32l0_exti_callback_t)&int_pps, (void*)NULL);
}

void setup()
{
  SerialMon.begin(9600);
  SerialMon.println("Projet Sentinel");

  // Wait for USB serial mounted in 5s max
  while (!SerialMon && millis()<5000) {
    delay(10);
  }

  gnssBegin_STM32();
  SerialGNSS.begin(9600);
  
  if (myGPS.begin(SerialGNSS) == false)
  {
    SerialMon.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  
  //This will pipe all NMEA sentences to the serial port so we can see them
  myGPS.setI2COutput(COM_TYPE_UBX); 
  myGPS.saveConfiguration();
  myGPS.enableDebugging(SerialMon);

  /*        SENDING COMMAND SETTING MAJOR CONSTELLATIONS TO GPS, GLONASS AND GALILEO CONCURRENTLY           */

  uint8_t customPayload[MAX_PAYLOAD_SIZE];
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  customCfg.cls = UBX_CLASS_CFG; 
  customCfg.id  = UBX_CFG_GNSS;
  customCfg.len = 0;
  customCfg.startingSpot = 0;
  
  uint16_t maxWait = 250;

  if (myGPS.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
  {
    Serial.println(F("sendCommand (poll / get) failed! Freezing..."));
    while (1)
    ;
  }
  
}

void loop()
{
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGPS.getLatitude();
    SerialMon.print(F("Lat: "));
    SerialMon.print(latitude);

    long longitude = myGPS.getLongitude();
    SerialMon.print(F(" Long: "));
    SerialMon.print(longitude);
    SerialMon.print(F(" (degrees * 10^-7)"));

    long altitude = myGPS.getAltitude();
    SerialMon.print(F(" Alt: "));
    SerialMon.print(altitude);
    SerialMon.print(F(" (mm)"));

    byte SIV = myGPS.getSIV();
    SerialMon.print(F(" SIV: "));
    SerialMon.print(SIV);

    SerialMon.println();
    SerialMon.print(myGPS.getYear());
    SerialMon.print("-");
    SerialMon.print(myGPS.getMonth());
    SerialMon.print("-");
    SerialMon.print(myGPS.getDay());
    SerialMon.print(" ");
    SerialMon.print(myGPS.getHour());
    SerialMon.print(":");
    SerialMon.print(myGPS.getMinute());
    SerialMon.print(":");
    SerialMon.println(myGPS.getSecond());
    
    SerialMon.println();
  }
}



