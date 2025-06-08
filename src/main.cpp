#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_SH110X.h>
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h> // for Serial
#endif
#include <Ewma.h>
#include "debug.h"

// Definition of hardware
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// Smoothing filter

Ewma thermoFilter1(0.5);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// Buttons on display
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5
#define POWER_OUTPUT 11
#define LED 3

void setup() {
  // Setup Pins
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(POWER_OUTPUT, OUTPUT);

  // Setup Serial Interface
  Serial.begin(115200);
  Serial.println("WaxBathController");

  // Setup PT100 Sensor
  thermo.begin(MAX31865_3WIRE);

  // Setup Display
  display.begin(0x3C, true); // Address 0x3C default

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);

  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.print("Hello world\n");
  display.display();

  DEBUG_COMMENT("setup() finished.\n");
}

uint8_t thermo_check_fault(void) {
  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    DEBUG_PRINT("Fault 0x%0x\n", fault);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      DEBUG_COMMENT("RTD High Threshold\n");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      DEBUG_COMMENT("RTD Low Threshold\n");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      DEBUG_COMMENT("REFIN- > 0.85 x Bias\n");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      DEBUG_COMMENT("REFIN- < 0.85 x Bias - FORCE- open\n");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      DEBUG_COMMENT("RTDIN- < 0.85 x Bias - FORCE- open\n");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      DEBUG_COMMENT("Under/Over voltage\n");
    }
    thermo.clearFault();
  }

  return fault;
}

uint32_t loop_counter = 0;
float setpoint = 88;
float hysteresis = 2.5;
bool op = 0;
bool flipflop = 0;

void loop() {
  if(!(loop_counter % 8)) {
    flipflop = flipflop ^ 1;
  }

  if(!thermo_check_fault()) {
    // We have a valid temp
    float temp = thermo.temperature(RNOMINAL, RREF);
    uint16_t resistance = thermo.readRTD();
    float temp_filtered = thermoFilter1.filter(temp);

    DEBUG_PRINT("Temp            : %d\n", (int)(temp * 1000));
    DEBUG_PRINT("Temp (filtered) : %d\n", (int)(temp_filtered * 1000));

    if((temp_filtered - setpoint) > hysteresis) {
      op = 0;
    }
    if((temp_filtered - setpoint) < (-1.0 * hysteresis)) {
      op = 1;
    }

    digitalWrite(POWER_OUTPUT, op);

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    if(flipflop) {
      display.print("Temp :   *\n\n");
      digitalWrite(LED, 1);
    } else {
      display.print("Temp :    \n\n");
      digitalWrite(LED, 0);
    }
    display.print(temp_filtered);
    display.print(" ");
    display.print((char)247);
    display.print("C\n");
    if(op) {
      display.print("ON \n");
    } else {
      display.print("OFF\n");
    }

    display.display();

    Serial.print(temp_filtered);
    Serial.print(",");
    Serial.print(resistance);
    Serial.print(",");
    Serial.print(op);
    Serial.print("\n");
  }

  loop_counter++;
}
