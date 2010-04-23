// place in communication mode
//    toggle reset low or falling edge on SPI select
// write byte to communication register
// read or write byte from function register
// registers up to 3 bytes wide, MSB first
//      12 bit register ignores 4 MSB of first byte
// if we're writing above 250 kHz, may hit 4 us / byte limit
//
// SPI: slave reads at falling edge, writes at rising edge
//

#include <Spi.h>
#include <Wire.h>
#define MUX_ADDRESS (B1001100) //  I2C MUX ADDRESS

const int METER = 10;
const int DIMMER = 9;

void _start_read(byte address) {
  digitalWrite(METER, LOW);
  Spi.transfer(address & 0x3F);
}

void _start_write(byte address) {
  digitalWrite(METER, LOW);
  Spi.transfer( (address & 0x3F) | 0x80 );
}

long _ade7763_read_24s(byte address) {
  _start_read(address);
  long ret;
  byte high, mid, low;
  high = Spi.transfer(0x00);
  mid = Spi.transfer(0x00);
  low = Spi.transfer(0x00);
  digitalWrite(METER, HIGH);
  ret = ( high << 16 | mid << 8 | low );
  if ( high | 0x80 ) { ret = ret | 0xFF000000; } // 2-complement 32 bit
  else { ret = ret & 0x007FFFFF; } // just make sure high bits are zero
  return ret;
}

long _ade7763_read_24u(byte address) {
  _start_read(address);
  byte high, mid, low;
  high = Spi.transfer(0x00);
  mid = Spi.transfer(0x00);
  low = Spi.transfer(0x00);
  digitalWrite(METER, HIGH);
  return ( high << 16 | mid << 8 | low );
}

int _ade7763_read_8u(byte address) {
  _start_read(address);
  byte low;
  delay(500);
  low = Spi.transfer(0x00);
  digitalWrite(METER, HIGH);
  return int(low);
}

long ade7763_read_waveform() {
  return _ade7763_read_24s(0x01); // Would be clearer to use the enum here
}

long ade7763_read_aenergy() {
  return _ade7763_read_24s(0x02);
}

long ade7763_read_raenergy() {
  return _ade7763_read_24s(0x03);
}

long ade7763_read_laenergy() {
  return _ade7763_read_24s(0x04);
}

long ade7763_read_vaenergy() {
  return _ade7763_read_24u(0x05);
}

long ade7763_read_rvaenergy() {
  return _ade7763_read_24u(0x06);
}

long ade7763_read_lvaenergy() {
  return _ade7763_read_24u(0x07);
}

int ade7763_read_dierev() {
  return _ade7763_read_8u(0x3F);
}

int ade7763_read_chksum() {
  return _ade7763_read_8u(0x3E);
}

long get_energy() {
	// returns total energy since last call
	// so first value on startup may not be useful
	// or may be, if you have the timestamp of the last call
	// TODO convert to useful units
	return ade7763_read_rvaenergy();
}

byte set_dimmer(int channel, int value)
{
  byte Hbyte;
  byte Lbyte;
  Hbyte=((channel&7)<<4)|((value&241)>>4);  // 0CCCVVVV VVVV0000
  Lbyte=(value&15)<<4;

  digitalWrite(DIMMER,LOW);
  //2 byte opcode
  Spi.transfer(Hbyte);
  Spi.transfer(Lbyte);
  digitalWrite(DIMMER,HIGH); //release chip, signal end transfer
}

/*
  Sprout Programmable Power Strip
  MUX Control
  for outlets 0-5
  */

// SWITCH MUX TO CHANNEL (0-5) OVER i2c
void switch_mux(int sense_channel) {
   Wire.beginTransmission(MUX_ADDRESS);
   sense_channel &= 7; // keep only relevent bits
   Wire.send(1 << sense_channel);
   Wire.endTransmission();
   //  Serial.print(sense_channel);
}

void setup()
{
  Serial.begin(9600);

	// 7763 init
  pinMode(METER, OUTPUT);
  digitalWrite(METER,HIGH); //disable device
	Spi.mode(0x57);

	// mux init
   Wire.begin(); // begin i2c with arduino as master
   int x = 0;
   switch_mux(x);

	// dimming init
  digitalWrite(DIMMER,HIGH); //disable device

  delay(10);
}

void loop() {
  Serial.print("running\n");
  //Serial.println(SPCR,BIN);
  Serial.println(ade7763_read_dierev());
  delay(500);
  Serial.println(ade7763_read_chksum());
     set_dimmer(2,(analogRead(0)>>2));
     //a++;
     delay(10);
}
