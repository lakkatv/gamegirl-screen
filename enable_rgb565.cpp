#include <wiringPi.h>
#include <wiringPiSPI.h>

using namespace std;

#define ILI9340_SWRESET 0x01
#define ILI9340_SLPOUT  0x11
#define ILI9340_GAMMASET 0x26
#define ILI9340_DISPOFF 0x28
#define ILI9340_DISPON  0x29
#define ILI9340_MADCTL  0x36
#define ILI9340_PIXFMT  0x3A

#define ILI9340_PWCTR1  0xC0
#define ILI9340_PWCTR2  0xC1
#define ILI9340_VMCTR1  0xC5
#define ILI9340_VMCTR2  0xC7

#define ILI9340_GMCTRP1 0xE0
#define ILI9340_GMCTRN1 0xE1

#define ILI9340_CASET   0x2A
#define ILI9340_PASET   0x2B
#define ILI9340_RAMWR   0x2C

int _clock = 0; // GPIO 0
int _den = 1; // GPIO 1
int _vsync = 2; // GPIO 2
int _hsync = 3; // GPIO 3
int _blu3 = 4; // GPIO 4
int _blu4 = 5; // GPIO 5
int _blu5 = 6; // GPIO 6
int _blu6 = 7; // GPIO 7
int _blu7 = 8; // GPIO 8
int _gre2 = 12; // GPIO 12
int _gre3 = 13; // GPIO 13
int _gre4 = 14; // GPIO 14
int _gre5 = 15; // GPIO 15
int _gre6 = 16; // GPIO 16
int _gre7 = 17; // GPIO 17
int _red3 = 20; // GPIO 20
int _red4 = 21; // GPIO 21
int _red5 = 22; // GPIO 22
int _red6 = 23; // GPIO 23
int _red7 = 24; // GPIO 24

int _dc = 25; // GPIO 25
int _rst = 27; // GPIO 27
int _mosi = 10; // GPIO 10
int _miso = 9; // GPIO 9
int _sclk = 11; // GPIO 11
int _backlight = 18; // GPIO 18
int _audio = 19;

int _channel = 1;
int _speed = 80000000;//80000000;//500000;//80000000;

void spiwritearray(unsigned char c[], int length) {
	wiringPiSPIDataRW(_channel, c, length);
}

void spiwrite(unsigned char c) {
	wiringPiSPIDataRW(_channel, &c, 1);
}

void writecommand(int c) {
  digitalWrite(_dc, LOW);
//  digitalWrite(_sclk, LOW);
  
  spiwrite(c);
}

void writedata(int c) {
  digitalWrite(_dc, HIGH);
//  digitalWrite(_sclk, LOW);
  
  spiwrite(c);
}

void begin() {
  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, LOW);
  
	pinMode(_dc, OUTPUT);

  // Initialise SPI
	wiringPiSPISetup(_channel, _speed);
  
  // Initialise DPI pins
  const int ALT2 = 0b110;
  pinModeAlt(_clock, ALT2);
  pinModeAlt(_den, ALT2);
  pinModeAlt(_vsync, ALT2);
  pinModeAlt(_hsync, ALT2);
  pinModeAlt(_blu3, ALT2);
  pinModeAlt(_blu4, ALT2);
  pinModeAlt(_blu5, ALT2);
  pinModeAlt(_blu6, ALT2);
  pinModeAlt(_blu7, ALT2);
  pinModeAlt(_gre2, ALT2);
  pinModeAlt(_gre3, ALT2);
  pinModeAlt(_gre4, ALT2);
  pinModeAlt(_gre5, ALT2);
  pinModeAlt(_gre6, ALT2);
  pinModeAlt(_gre7, ALT2);
  pinModeAlt(_red3, ALT2);
  pinModeAlt(_red4, ALT2);
  pinModeAlt(_red5, ALT2);
  pinModeAlt(_red6, ALT2);
  pinModeAlt(_red7, ALT2);
  
  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }
  
  writecommand(ILI9340_SWRESET);
  delay(1);
  
  writecommand(ILI9340_DISPOFF);
  
  /*
  writecommand(0xCA);
  writedata(0xC3);
  writedata(0x08);
  writedata(0x50);
  */
  
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);
  
  writecommand(0xCF); // LCD_POWERB
  writedata(0x00);
  writedata(0XC1);
  writedata(0X30);
  
  writecommand(0xED); // LCD_POWER_SEQ
  writedata(0x64);
  writedata(0x03);
  writedata(0X12);
  writedata(0X81);
  
  writecommand(0xE8); // LCD_DTCA
  writedata(0x85);
  writedata(0x00);
  writedata(0x78);
  
  writecommand(0xCB); // LCD_POWERA
  writedata(0x39);
  writedata(0x2C);
  writedata(0x00);
  writedata(0x34);
  writedata(0x02);
  
  writecommand(0xF7); // LCD_PRC
  writedata(0x20);
  
  writecommand(0xEA); // LCD_DTCB
  writedata(0x00);
  writedata(0x00);
  
  /*
  writecommand(0xB1); // LCD_FRC // Frame Rate Control
  writedata(0x00);
  writedata(0b00010000); // 0x1B=70 FPS
  */
  // LCD_DFC
  
  writecommand(ILI9340_PWCTR1);    //Power control // 0xC0
  writedata(0x23);
  
  writecommand(ILI9340_PWCTR2);    //Power control // 0xC1
  writedata(0x10);
  
  writecommand(ILI9340_VMCTR1);    //VCM control // 0xC5
  writedata(0x3e); // 0x31?
  writedata(0x28); // 0x3C?
  
  writecommand(ILI9340_VMCTR2);    //VCM control2 // 0xC7
  writedata(0x86); // 0xC0? 0x86?
  
  writecommand(ILI9340_MADCTL);    // Memory Access Control  // 0x36
  writedata(0b01001000); // MY=1, MX=0, MV=1 (0b01000000), BGR=1 (0b01001000)
  
  writecommand(ILI9340_PIXFMT); // 0x3A
  writedata(0x55); // 16-bit: DPI=101 // 18-bit=0x66
  
  writecommand(0xF2); // 3Gamma Function Disable
  writedata(0x02);
  
  writecommand(0xB0); // RGB Interface Signal Control
  writedata(0b11000011); // memory=1 RCM=10, 0, VSPL=0 (low level sync clock), HSPL=0, DPL=1 (dotclock fetched at rising time), EPL=0 (high enable)
  // 0xC2? DEFAULT: 0x40, 0b01000000
  
  writecommand(0xB6); // LCD_DFC // Display Function Control
  writedata(0x0A);
  writedata(0b10000010); // =0x82
  writedata(0x27);
  writedata(0x08); // PCDIV = 4
  
  writecommand(0x2A); // column set
  writedata(0x00);
  writedata(0x00);
  writedata(0x00);
  writedata(0xEF);
  
  writecommand(0x2B); // Page set
  writedata(0x00);
  writedata(0x00);
  writedata(0x01);
  writedata(0x3F);
  
  writecommand(0xF6); // Interface Control
  writedata(0x01);
  writedata(0b00000000); // EPF=11
  writedata(0b00001010); // 16-bit RGB=0b00000110, DM=01, RM=1, RIM=0
  
  writecommand(0x2C); // GRAM register
  delay(1);
  
  writecommand(ILI9340_GAMMASET);    //Gamma curve selected // 0x26
  writedata(0x01);
  
  writecommand(ILI9340_GMCTRP1);    //Set Gamma // 0xE0
  writedata(0x0F);
  writedata(0x31);
  writedata(0x2B);
  writedata(0x0C);
  writedata(0x0E);
  writedata(0x08);
  writedata(0x4E);
  writedata(0xF1);
  writedata(0x37);
  writedata(0x07);
  writedata(0x10);
  writedata(0x03);
  writedata(0x0E);
  writedata(0x09);
  writedata(0x00);
  
  writecommand(ILI9340_GMCTRN1);    //Set Gamma // 0xE1
  writedata(0x00);
  writedata(0x0E);
  writedata(0x14);
  writedata(0x03);
  writedata(0x11);
  writedata(0x07);
  writedata(0x31);
  writedata(0xC1);
  writedata(0x48);
  writedata(0x08);
  writedata(0x0F);
  writedata(0x0C);
  writedata(0x31);
  writedata(0x36);
  writedata(0x0F);
  
  writecommand(0xB5); // Blanking Porch Control
  writedata(0x04); // VFP
  writedata(0x02); // VBP
  writedata(0x0A); // HFP
  writedata(0x14); // 0x14? 0x16? HBP
  
  writecommand(0x2D); // Color Set LUT
  for (unsigned char i = 0; i < 32; i++) {
    writedata(63*i/31);
  }
  for (unsigned char i = 0; i < 64; i++) {
    writedata(i);
  }
  for (unsigned char i = 0; i < 32; i++) {
    writedata(63*i/31);
  }
  
  writecommand(ILI9340_SLPOUT);    //Exit Sleep
  delay(120);
  writecommand(ILI9340_DISPON);    //Display on
  delay(120);
  writecommand(0x2C); // memory write
  
  // Initialise backlight
//  pinMode(_backlight, PWM_OUTPUT);
//  pwmWrite(_backlight, 1023);
  pinMode(_backlight, OUTPUT);
  digitalWrite(_backlight, HIGH);
  
  /*
  pwmSetMode  (PWM_MODE_BAL) ;	// Pi default mode
  pwmSetClock (32) ;		// 19.2 / 32 = 600KHz - Also starts the PWM
  */
  
  /*
  *(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE ;
  *(pwm + PWM0_RANGE) = 1024;
  
  uint32_t pwm_control = *(pwm + PWM_CONTROL); // preserve PWM_CONTROL
  *(pwm + PWM_CONTROL) = 0; // Stop PWM
  
  
  pwmWrite(_backlight, 1023);
  */
  
  // Initialise audio
//  const int ALT5 = 0b010;
//  pinModeAlt(_audio, ALT5);
}

int main() {
	wiringPiSetupGpio();
	
	begin();
}

