#include <STM32L0.h>
#include "wiring_private.h" //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS

SFE_UBLOX_GPS myGPS;

#define SerialMon  Serial
#define SerialGNSS Serial1

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
  myGPS.setNMEAOutputPort(SerialGNSS);
}

void loop()
{
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.
  SerialMon.println("Udntttt");
  delay(250); //Don't pound too hard on the I2C bus
}