#include <Arduino.h>
#include <Base64.h>
#include <ArduinoJson.h>

bool AlgaeMode = false;
bool ClimbDown = false;
bool ClimbUp = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  pinMode(13, OUTPUT);
  pinMode(4, OUTPUT);
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
      digitalWrite(4, HIGH);
      // Serial.print(F("deserializeJson() failed: "));
      // Serial.println(error.f_str());
      return;
    } 
    digitalWrite(4, LOW);

    AlgaeMode = doc["AlgaeMode"];
    ClimbDown = doc["ClimbDown"];
    ClimbUp = doc["ClimbUp"];
  }

  if (AlgaeMode) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}