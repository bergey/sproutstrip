#include <EEPROM.h>

#define SPIN 5
#define THRESHOLD 512
#define MEMLOC 128

unsigned long rise, fall, ontime, total_time;
int last_state;
char c;

unsigned long load(byte addr) {
  int i;
  unsigned long ret=0x00;
  for(i=0; i<4; i++) {
    ret |= (EEPROM.read(addr+i) << (8*i));
  }
  return ret;
}

void store(byte addr, unsigned long val) {
  int i;
  for (i=0; i<4; i++) {
    Serial.print(0xFF&(val>>(i*8)));
    EEPROM.write(addr+i, 0xFF&(val>>(i*8)));
  }
  Serial.println();
}
    
void setup() {
  Serial.begin(9600);
  
  Serial.print("ontime was: ");
  ontime = load(MEMLOC);
  Serial.print(ontime);
  Serial.println(" seconds");
  
  Serial.print("total time was: ");
  total_time = load(MEMLOC+4);
  Serial.print(total_time);
  Serial.println(" seconds");
  
  Serial.println("Press 'r' to reset");
  while(!Serial.available());
  c = Serial.read();
  if (c == 'r') {
    ontime = 0;
    total_time = 0;
    Serial.println(total_time);
  }
  
  rise = 0;
  fall = 0;
  if (analogRead(SPIN) > THRESHOLD) {
    last_state = 1;
  } else {
    last_state = 0;
  }
}

void loop() {
  if ( (analogRead(SPIN) < THRESHOLD) && (last_state == 1)) {
    Serial.println("falling edge");
    fall = millis();
    last_state = 0;
    if(fall-rise < 0) { // looks like millis rolled over
      Serial.println("millis rolled over");
      rise = fall; // TODO calculate the right value here
    }
    ontime += (fall - rise)/1000;
    total_time += (fall-rise)/1000;
  }
  
  if ( (analogRead(SPIN) >= THRESHOLD) && (last_state == 0)) {
    Serial.println("rising edge");
    rise = millis();
    last_state = 1;
    if (rise-fall < 0) {
      fall = rise;  // TODO
    }
    total_time += (rise-fall)/1000;
  }
  
  if (millis() % 1000 == 0) {
      Serial.print(analogRead(SPIN));
      Serial.print(" ");
      Serial.println(ontime);
  }
  
  if ((millis() % 3600000) == 0) { // once an hour; 100k read-write cycles
    Serial.println("writing to EEPROM");
    store(MEMLOC, ontime);
    store(MEMLOC+4, total_time);
  }
}
