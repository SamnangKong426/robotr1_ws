void deserialJson(String input) {
  DeserializationError error = deserializeJson(doc, input);

  if (error) {
    debug.print(F("deserializeJson() failed: "));
    debug.println(error.f_str());
    return;
  }

  JsonArray cmd_vel = doc["Cmd_vel"];
  vx = cmd_vel[0];
  vy = cmd_vel[1];
  w = cmd_vel[2];
}

void serialization() {
  delay(20);
  JsonArray ch = doc.createNestedArray("ch");
  for (int8_t i = 0; i < data.NUM_CH; i++) {
    ch.add(data.ch[i]);
  }
  serializeJson(doc, debug);
  debug.println();
}

void readSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    deserialJson(input);
    debug.println(input);
    
  }
}
