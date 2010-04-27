// place in communication mode
// toggle reset low or falling edge on SPI select
// write byte to communication register
// read or write byte from function register
// registers up to 3 bytes wide, MSB first
// 12 bit register ignores 4 MSB of first byte
// if we're writing above 250 kHz, may hit 4 us / byte limit
//
// SPI: slave reads at falling edge, writes at rising edge
//

#include <Spi.h>
#include <Wire.h>
#define MUX_ADDRESS (B1001100) // I2C MUX ADDRESS

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

  Serial.print(high,HEX); Serial.print(" ");
  Serial.print(mid,HEX); Serial.print(" ");
  Serial.print(low,HEX); Serial.print(" ");

  return ret;
}

unsigned long _ade7763_read_24u(byte address) {
  _start_read(address);
  byte high, mid, low;
  high = Spi.transfer(0x00);
  mid = Spi.transfer(0x00);
  low = Spi.transfer(0x00);
  digitalWrite(METER, HIGH);

  Serial.print(high,HEX); Serial.print(" ");
  Serial.print(mid,HEX); Serial.print(" ");
  Serial.print(low,HEX); Serial.print(" ");
  
  return ( high << 16 | mid << 8 | low );
}

int _ade7763_read_16s(byte address) {
	_start_read(address);
	int ret;
	byte high, low;
	high = Spi.transfer(0x00);
	low = Spi.transfer(0x00);
	digitalWrite(METER, HIGH);
	ret = high << 8 | low; // for now, assume we don't need  a cast
	return ret; 
}

unsigned int _ade7763_read_16u(byte address) {
	_start_read(address);
	int ret;
	byte high, low;
	high = Spi.transfer(0x00);
	low = Spi.transfer(0x00);
	digitalWrite(METER, HIGH);
	ret = high << 8 | low; // for now, assume we don't need  a cast
	return ret; 
}

unsigned int _ade7763_read_8u(byte address) {
  _start_read(address);
  byte low;
  low = Spi.transfer(0x00);
  digitalWrite(METER, HIGH);
  return int(low);
}

void _ade7763_write_16u(byte address, unsigned int val) {
	_start_write(address);
	Spi.transfer(val>>8);
	Spi.transfer(val & 0xFF);
	digitalWrite(METER, HIGH);
}

void _ade7763_write_8u(byte address, byte val) {
	_start_write(address);
	Spi.transfer(val);
	digitalWrite(METER, HIGH);
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

unsigned long ade7763_read_vaenergy() {
  return _ade7763_read_24u(0x05);
}

unsigned long ade7763_read_rvaenergy() {
  return _ade7763_read_24u(0x06);
}

unsigned long ade7763_read_lvaenergy() {
  return _ade7763_read_24u(0x07);
}

unsigned int ade7763_read_mode() {
	return _ade7763_read_16u(0x09);
}

void ade7763_write_mode(unsigned int val) {
	_ade7763_write_16u(0x09, val);
}

unsigned int ade7763_read_irqen() {
	return _ade7763_read_16u(0x0A);
}

void ade7763_write_irqen(unsigned int val) {
	_ade7763_write_16u(0x0A, val);
}

unsigned int ade7763_read_status() {
	return _ade7763_read_16u(0x0B);
}

unsigned int ade7763_read_rststatus() {
	return _ade7763_read_16u(0x0C);
}

byte ade7763_read_ch1os() {
	return _ade7763_read_8u(0x0D);
}

void ade7763_write_ch1os(byte val) {
	_ade7763_write_8u(0x0D, val);
}

byte ade7763_read_ch2os() {
	return _ade7763_read_8u(0x0E);
}

void ade7763_write_ch2os(byte val) {
	_ade7763_write_8u(0x0E, val);
}

byte ade7763_read_gain() {
	return _ade7763_read_8u(0x0F);
}

void ade7763_write_gain(byte val) {
	_ade7763_write_8u(0x0F, val);
}

// I'm not sure how do deal with 6-bit signed

int ade7763_read_apos() {
	return _ade7763_read_16s(0x11);
}

void ade7763_write_apos(int val) {
	_ade7763_write_16s(0x11, val);
}

// 12-bit signed, also unclear

byte ade7763_read_wdiv() {
	return _ade7763_read_8u(0x13);
}

void ade7763_write_wdiv(byte val) {
	_ade7763_write_8u(0x13, val);
}

unsigned int ade7763_read_cfnum() {
	return _ade7763_read_16u(0x14);
}

void ade7763_write_cfnum(unsigned int val) {
	_ade7764_write_16u(0x14, val);
}

unsigned int ade7763_read_cfden() {
	return _ade7763_read_16u(0x15);
}

void ade7763_write_cfden(unsigned int val) {
	_ade7763_write_16u(0x15, val);
}

unsigned long ade7763_read_irms() {
	return _ade7763_read_24u(0x16);
}

unsigned long ade7763_read_vrms() {
	return _ade7763_read_24u(0x17);
}

// 12-bit signed values

byte ade7763_read_vadiv() {
	return _ade7763_read_8u(0x1B);
}

void ade7763_write_vadiv(byte val) {
	_ade7763_write_8u(0x1B, val);
}

byte ade7763_read_temp() {
	return _ade7763_read_8u(0x26);
}

unsigned int ade7763_read_dierev() {
  return _ade7763_read_8u(0x3F);
}

unsigned int ade7763_read_chksum() {
  return _ade7763_read_8u(0x3E);
}

unsigned long get_energy() {
// returns total energy since last call
// so first value on startup may not be useful
// certainly not if we've switched the MUX
// TODO convert to useful units
return ade7763_read_rvaenergy();
}

byte set_dimmer(int channel, int value)
{
  byte Hbyte;
  byte Lbyte;
  Hbyte=((channel&7)<<4)|((value&241)>>4); // 0CCCVVVV VVVV0000
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
   // Serial.print(sense_channel);
}

void setup()
{
  Serial.begin(9600);

// 7763 init
  pinMode(METER, OUTPUT);
  digitalWrite(METER,HIGH); //disable device
  Spi.mode(0x57);
  ade7763_write_gain(0x22); // 2x on Voltage, 4x on Current (15A, 120 Vrms)

// mux init
   Wire.begin(); // begin i2c with arduino as master
   int x = 0;
   switch_mux(x);

// dimming init
  pinMode(DIMMER, OUTPUT);
  digitalWrite(DIMMER,HIGH); //disable device

  delay(10);
}

void loop() {
  unsigned long a;
  //Serial.println(SPCR,BIN);
  
  a=ade7763_read_rvaenergy();
  Serial.print(" = ");
  
  Serial.print(a);
  Serial.print(" ");
  Serial.print(ade7763_read_dierev());
  Serial.print(" ");
  Serial.println(ade7763_read_chksum());
  set_dimmer(2,(analogRead(0)>>2));
 
  delay(2000);
}
