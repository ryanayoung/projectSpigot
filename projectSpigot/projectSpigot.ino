#include <Arduino.h>
#include "CirquePinnacle.h"

#define ss_pin 2
#define dr_pin 5

PinnacleTouchSPI tpad = PinnacleTouchSPI(dr_pin, ss_pin);
RelativeReport report_rel;
AbsoluteReport report_abs;

PinnacleDataMode mode = PINNACLE_RELATIVE;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);  // some boards need this to access USB Serial
  }
  
  if (tpad.begin()) {
    Serial.println("found Cirque Pinnacle!");
  }
  else {
    Serial.println("Cirque Pinnacle not responding!");
  }

  tpad.setDataMode(PINNACLE_ABSOLUTE);
  tpad.absoluteModeConfig(0);  // set count of z-idle packets to 1
  tpad.setAdcGain(0x80); //available values: 0x00, 0x40, 0x80, 0xC0, in increasing attenuation.
  tpad.tuneEdgeSensitivity();
  
  tpad.setDataMode(PINNACLE_RELATIVE);
  tpad.relativeModeConfig(true, true, true, false, true); //bool rotate90, bool allTaps, bool secondaryTap, bool glideExtend, bool intellimouse
  
}

void loop() {
  if (tpad.available()) {
    switch(mode) {
      case PINNACLE_ABSOLUTE :
        bool hovering;
        tpad.read(&report_abs);
        hovering = tpad.CheckHovering(report_abs);

        Serial.print("Left: ");
        Serial.print(report_abs.buttons & 1);
        Serial.print(" Right: ");
        Serial.print(report_abs.buttons & 2);
        Serial.print(" Middle: ");
        Serial.print(report_abs.buttons & 4);
        Serial.print("\tX: ");
        Serial.print(report_abs.x);
        Serial.print("\tY: ");
        Serial.print(report_abs.y);
        Serial.print("\tZ: ");
        Serial.print(report_abs.z);
        Serial.print("\t");
        Serial.println(hovering);
        if( report_abs.buttons & 4) {
          tpad.setDataMode(PINNACLE_RELATIVE);
          mode = PINNACLE_RELATIVE;
          Serial.println("Change to Relative Mode");
          delay(1);
        }
        break;
      case PINNACLE_RELATIVE :
        tpad.read(&report_rel);
        Serial.print("Left: ");
        Serial.print(report_rel.buttons & 0x01);
        Serial.print(" Right: ");
        Serial.print((report_rel.buttons & 0x02) >> 1);
        Serial.print(" Middle: ");
        Serial.print((report_rel.buttons & 0x04) >> 2);
        Serial.print("\tdelta X: ");
        Serial.print(report_rel.x);
        Serial.print("\tdelta Y: ");
        Serial.print(report_rel.y);
        Serial.print("\tdelta Scroll: ");
        Serial.println(report_rel.scroll);
        if( report_rel.buttons & 4) {
          tpad.setDataMode(PINNACLE_ABSOLUTE);
          tpad.absoluteModeConfig(0);  // set count of z-idle packets to 1
          //tpad.setAdcGain(0x02);
          mode = PINNACLE_ABSOLUTE;
          Serial.println("Change to Absolute Mode");
          delay(1);
        }
        break;
      default :
        break;
    } 
  }
}

//void btn_debounce(char * previous, AbsoluteReport * report) {
//  char current = report.buttons;
//  if (current != previous) {
//        
//        }
//  previous = current.buttons;
//}
