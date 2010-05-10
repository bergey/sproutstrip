#define MOSI 11
#define MISO 12
#define SPICLOCK  13

unsigned long a, b;

char spi_transfer(volatile char b_in) {
  char b_out=0;
  unsigned long t;
  int i;
  for(i=0; i<7; i++) {
    digitalWrite(SPICLOCK, HIGH);
    t = micros();
    if(b_in| (1<<i)) {digitalWrite(MOSI, HIGH);}
    else {digitalWrite(MOSI, LOW);}
    while (micros() < t+32) {}
    digitalWrite(SPICLOCK, LOW);
    b_out & (digitalRead(MISO)<<i);
    while (micros() < t+64) {}
  }
  return b_out;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  a = micros();
  spi_transfer(0x00);
  b = micros();
  Serial.println(b-a);
  delay(500);
}
