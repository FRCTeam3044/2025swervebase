#include <Arduino.h>
#include <Base64.h>
#include <ArduinoJson.h>

int startingIndex = 12; // 0 For arduino 1
                       // 12 for arduino 2
                       // 25 for arduino 3

void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {
    ;
  }
}

void loop() {
  if (Serial.available()) {
    String encoded = Serial.readStringUntil('\n');
    int inputLength = encoded.length();
    char input[inputLength + 1];
    encoded.toCharArray(input, inputLength + 1);

    int decodedLength = Base64.decodedLength(input, inputLength);
    char decoded[decodedLength + 1];
    Base64.decode(decoded, input, inputLength);
    decoded[decodedLength] = '\0';

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, decoded);

    if (error) {
      // Serial.print(F("deserializeJson() failed: "));
      // Serial.println(error.f_str());
      return;
    } 
    for (int i = 0; i < 13; i++) {
      if(doc[i + startingIndex]) {
        digitalWrite(i+2, HIGH);
      } else {
        digitalWrite(i+2, LOW);
      }
    }
  }
}