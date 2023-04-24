#include <Arduino.h>
#include <Network/NetworkManager.h>

/*
  Steps to setup a new ESP32:
  1. Upload the content of the data dir to SPIFFS
    -> see https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/
  2. Set the correct COM Port in platformio.ini
    -> Quick Access -> PIO Home -> Devices lists all connected devices and COM ports
  3. Build and Upload this project
  4. Use smartphone and list all network devices
    -> connect to your device (ESP32-XXXXXXX) and open 192.168.4.1 via browser
  5. Set Wifi credentials and submit. The ESP32 will reboot.
  6. Go to platformio.ini and uncomment these lines to enable OTA uploads:
    -> upload_protocol
    -> extra_scripts
    -> upload_url (make sure you set the correct WIFI IP of your ESP32 here)
  7. You can access the netsettings of your ESP32 under the given WIFI IP again and change settings

*/

/*
GPI EXPANDER https://wolles-elektronikkiste.de/en/port-expander-mcp23017-2
*/
#include <NeoPixelBus.h>
#include <SPI.h>
#include <MCP23S18.h>
#include <U8g2lib.h> //display
#include <Wire.h> //i2c für display

#define CS_PIN 16   // Chip Select - Pin 16 on gateway - 4 on wt32 //TODO: CS wofür?
/* A hardware reset is performed during init(). If you want to save a pin you can define a dummy 
 * reset pin >= 99 and connect the reset pin to HIGH. This will trigger a software reset instead 
 * of a hardware reset. 
 */
#define RESET_PIN 99
#define MCP_SPI_CTRL_BYTE 0x20 // Do not change

#define SCK  14
#define MISO 12
#define MOSI  15
#define EXP_CS 16
#define DAC_CS 4 //DAC SYNC Pin
#define I2C_SDA 2   //Display
#define I2C_SCL 13     //Display
 
#define colorSaturation 128

const uint16_t PixelCount = 16; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 33;  // make sure to set this to the correct pin, ignored for Esp8266

unsigned int one;
unsigned int two;
unsigned int three;
unsigned int four;

unsigned long message;

U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // DISPLAY


// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(PixelCount, PixelPin);

NetworkManager networkManager;

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

HslColor hslRed(red);
HslColor hslGreen(green);
HslColor hslBlue(blue);
HslColor hslWhite(white);
HslColor hslBlack(black);

/* There are two ways to create your MCP23S18 object:
 * MCP23S18 myMCP = MCP23S18(CS_PIN, RESET_PIN, MCP_CTRL_BYTE);
 * MCP23S18 myMCP = MCP23S18(&SPI, CS_PIN, RESET_PIN, MCP_CTRL_BYTE);
 * The second option allows you to pass SPI objects, e.g. in order to
 * use two SPI interfaces on the ESP32.
 */

MCP23S18 myMCP = MCP23S18(&SPI, EXP_CS, RESET_PIN, MCP_SPI_CTRL_BYTE);
int wT = 500; // wT = waiting time
byte buttons = 0b00000000;


void DAC8568Write(unsigned int prefix, unsigned int control, unsigned int address, unsigned int data, unsigned int feature) {
 
  message = (prefix << 28) | (control << 24) | (address << 20) | (data << 4) | (feature);
  
  digitalWrite(DAC_CS,LOW);
  SPI.transfer32(message);
  digitalWrite(DAC_CS,HIGH); 
}

void DAC8568Reset()
{
  DAC8568Write(0b0001, 0b1100, 0x00, 0x00, 0x00) ;
}

void DAC8568InternalRef()
{
  DAC8568Write(0x00, 0x08, 0x00, 0x0000, 0x01) ;
}

void DAC8568SetVoltage(unsigned int channel,unsigned int voltage)
{
  DAC8568Write(0, 3, channel, voltage, 0) ;
}

void setup()
{
  pinMode(EXP_CS, OUTPUT);
  SPI.begin(SCK, MISO, MOSI, DAC_CS); //EXP_CS

  pinMode (DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH);
  SPI.beginTransaction(SPISettings(48000000, MSBFIRST, SPI_MODE1));
  delay(200);
  DAC8568Reset();
  delay(200);
  DAC8568InternalRef();
  delay(200);
  SPI.endTransaction();


  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
    u8g2.begin();

  
  delay(5000);

  networkManager.begin();

  Serial.println();
    Serial.println("Initializing...");
    Serial.flush();

    // this resets all the neopixels to an off state
    strip.Begin();
    strip.Show();

    Serial.println("Colors R, G, B, W...");

    // set the colors, 
    // if they don't match in order, you need to use NeoGrbFeature feature
    strip.SetPixelColor(0, red);
    strip.SetPixelColor(1, green);
    strip.SetPixelColor(2, blue);
    strip.SetPixelColor(3, white);
    strip.SetPixelColor(4, red);
    strip.SetPixelColor(5, green);
    strip.SetPixelColor(6, blue);
    strip.SetPixelColor(7, white);
    strip.SetPixelColor(8, red);
    strip.SetPixelColor(9, green);
    strip.SetPixelColor(10, blue);
    strip.SetPixelColor(11, white);
    strip.SetPixelColor(12, red);
    strip.SetPixelColor(13, green);
    strip.SetPixelColor(14, blue);
    strip.SetPixelColor(15, white);
    // the following line demonstrates rgbw color support
    // if the NeoPixels are rgbw types the following line will compile
    // if the NeoPixels are anything else, the following line will give an error
    //strip.SetPixelColor(3, RgbwColor(colorSaturation));
    strip.Show();

    Serial.println();
    Serial.println("Running...");
    SPI.begin();
    if(!myMCP.Init()){
      Serial.println("Not connected!");
        while(1){} 
    }
  // myMCP.setSPIClockSpeed(8000000); // Choose SPI clock speed (after Init()!)
  delay(wT);
  myMCP.setPortMode(0b11111111, A);   // Port A: all pins are OUTPUT - DAC SDN is output
  delay(wT);
  myMCP.setPortMode(0b00000000, B);   // Port B: all pins are INPUT - Buttons are inputs
  myMCP.setPortPullUp(0b11111111, B);  // Port B: Pin 4 - 7 are pulled up

  delay(wT);
  myMCP.setAllPins(A, LOW);            // Port A: all pins are LOW - DACs activated (or deactivated?)
}

void loop()
{
  unsigned int i;
  unsigned int j;
  networkManager.loop();
  delay(1000);
  buttons = myMCP.getPort( B );
  Serial.print("Buttons: ");
  Serial.println(buttons, BIN);

   u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_u8glib_4_tf);	// choose a suitable font
  u8g2.drawStr(0, 5, "Hello,");	 // write something to the internal memory
  u8g2.drawStr(0, 10, "World…");
  u8g2.drawStr(0, 15, "I'm tiny…");
  u8g2.drawStr(0, 20, "So tiny!");
  u8g2.drawStr(0, 25, "However you can");
  u8g2.drawStr(0, 30, "have six lines");
  u8g2.sendBuffer();					// transfer internal memory to the display
  
  SPI.beginTransaction(SPISettings(48000000, MSBFIRST, SPI_MODE1));
    digitalWrite(DAC_CS, HIGH);
for (j=0;j<8;++j) {
  for (i=0;i<8;++i)
  {
    DAC8568SetVoltage(i,500*j);
  }
    delay(500);

}
    SPI.endTransaction();

  digitalWrite(DAC_CS, LOW);




  

}
