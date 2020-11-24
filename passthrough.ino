#include <STM32L0.h>
#include "wiring_private.h"

#define SerialMon  Serial
#define SerialGNSS Serial1

// CAM M8Q GNSS configuration (Arduino pin number mode)
#define GNSS_ENABLE  (5u)  // enable for GNSS 3.0 V LDO
#define GNSS_PPS     (4u)  // 1 Hz fix pulse
#define GNSS_BACKUP  A0  
#define STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL  0x00000000

#if defined (ARDUINO_STM32L0_GNAT)
void ledOn() { digitalWrite(LED_BUILTIN, LOW); }
void ledOff() { digitalWrite(LED_BUILTIN, HIGH); }
#else
#error "Board should be Gnat only"
#endif

// CAM M8Q GNSS configuration (STM32 framework pin number mode)
uint16_t stm32_enable_pin = g_APinDescription[GNSS_ENABLE].pin;
uint16_t stm32_pps_pin    = g_APinDescription[GNSS_PPS].pin;
uint16_t stm32_backup_pin = g_APinDescription[GNSS_BACKUP].pin;
volatile bool pps_irq;
volatile uint32_t led_color;

void int_pps(void) {
  pps_irq = true;
}

void gnssReset()
{
  digitalWrite(GNSS_ENABLE, LOW);
  delay(1000);
  digitalWrite(GNSS_ENABLE, HIGH);
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
  // Start communication
  SerialMon.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  ledOff();

  // Wait for USB serial mounted in 5s max
  while (!SerialMon && millis()<5000) {
    delay(10);
  }

  SerialMon.println("\r\n\r\n*** Starting firmware GNSS Passtrhu ***");
  SerialMon.println("GNSS Arduino/STM32 Connected pins");
  SerialMon.print("ENABLE=");  SerialMon.print(GNSS_ENABLE);
  SerialMon.print("/");  SerialMon.print(stm32_enable_pin);
  SerialMon.print("  BACKUP=");SerialMon.print(GNSS_BACKUP);
  SerialMon.print("/");  SerialMon.print(stm32_backup_pin);
  SerialMon.print("  PPS=");   SerialMon.print(GNSS_PPS);
  SerialMon.print("/");  SerialMon.println(stm32_pps_pin);
  
  gnssBegin_STM32();
  SerialGNSS.begin(9600);

  gnssReset();
}

// Forward every message to the other serial
void loop()
{
  while (SerialMon.available()) {
    SerialGNSS.write(SerialMon.read());
  }

  while (SerialGNSS.available()) {
    SerialMon.write(SerialGNSS.read());
  }

  // Got PPS (each second once fixed)
  if (pps_irq){
    pps_irq = false;
    ledOn();
    delay(5);
    ledOff();
  }
}