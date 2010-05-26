#define MOSI 11
#define MISO 12
#define SPICLOCK  13
const int METER = 10;

unsigned long a, b;

char spi_transfer(volatile char b_in) {
  char b_out=0;
  unsigned long t;
  int i;
  for(i=7; i>=0; i--) {
    digitalWrite(SPICLOCK, HIGH);
    t = micros();
    if(b_in & (1<<i)) {digitalWrite(MOSI, HIGH);}
    else {digitalWrite(MOSI, LOW);}
    b_out = b_out & (digitalRead(MISO)<<i);
    while (micros() < t+160) {}
    digitalWrite(SPICLOCK, LOW);
    while (micros() < t+320) {}
  }
  while (micros() < t+480) {}
  return b_out;
}

void _start_read(byte address) {
  digitalWrite(METER, LOW);
  spi_transfer(address & 0x3F);
}

unsigned int _ade7763_read_8u(byte address) {
  _start_read(address);
  byte low;
  low = spi_transfer(0x00);
  digitalWrite(METER, HIGH);
  return int(low);
}

unsigned int ade7763_read_dierev() {
  return _ade7763_read_8u(0x3F);
}

unsigned int ade7763_read_chksum() {
  return _ade7763_read_8u(0x3E);
}

void setup() {
  pinMode(SPICLOCK, OUTPUT);
  pinMode(MOSI,     OUTPUT); 
  pinMode(METER,    OUTPUT); 
  pinMode(MISO,     INPUT); 
  Serial.begin(9600);
}

void loop() {
  Serial.print(ade7763_read_dierev(),HEX);
  Serial.print(" ");
  Serial.println(ade7763_read_chksum(),HEX);
  
  delay(500);
}
