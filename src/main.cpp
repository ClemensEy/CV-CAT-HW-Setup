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
#include <MCP3x6x.h>


#define RESET_PIN 99
#define MCP_SPI_CTRL_BYTE 0x20 // Do not change

#define SCK  2 //14
#define MISO 36 //12
#define MOSI  5 //15
#define EXP_CS 16
#define DAC_CS 4 //DAC SYNC Pin

 
#define colorSaturation 128


//ADC
#define MCP3x6x_DEBUG 0
#define ADC_pinCS 32
#define ADC_IRQ 34

const uint16_t PixelCount = 16; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 33;  // make sure to set this to the correct pin, ignored for Esp8266

unsigned int one;
unsigned int two;
unsigned int three;
unsigned int four;

unsigned long message;



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

//ADC
MCP3464 mcp(ADC_pinCS, &SPI); //only ADC_CS as argument?

//Buttons
/* There are two ways to create your MCP23S18 object:
 * MCP23S18 myMCP = MCP23S18(EXP_CS, RESET_PIN, MCP_CTRL_BYTE);
 * MCP23S18 myMCP = MCP23S18(&SPI, EXP_CS, RESET_PIN, MCP_CTRL_BYTE);
 * The second option allows you to pass SPI objects, e.g. in order to
 * use two SPI interfaces on the ESP32.
 */

MCP23S18 myMCP = MCP23S18(&SPI, EXP_CS, RESET_PIN, MCP_SPI_CTRL_BYTE);
int wT = 500; // wT = waiting time
byte buttons = 0b00000000;

/// The Message that is eventually serialized and transmitted to the DAC
/// The input shift register (SR) of the DAC7568, DAC8168, and DAC8568
/// is 32 bits wide, and consists of four Prefix bits (DB31 to DB28),
/// four control bits (DB27 to DB24), 16 databits (DB23 to DB4),
/// and four additional feature bits. The 16 databits comprise the 16-, 14-, or 12-bit input code
void DAC8568Write(unsigned int prefix, unsigned int control, unsigned int address, unsigned int data, unsigned int feature) {
 
  message = (prefix << 28) | (control << 24) | (address << 20) | (data << 4) | (feature);
  
  digitalWrite(DAC_CS,LOW);
  SPI.transfer32(message);
  digitalWrite(DAC_CS,HIGH); 
}


/// Get software reset message
    /// 8.2.10 Software Reset Function
    /// The DAC7568, DAC8168, and DAC8568 contain a software reset feature.
    /// If the software reset feature is executed, all registers inside the device are reset to default settings; that is,
    /// all DAC channels are reset to the power-on reset code (power on reset to zero scale for grades A and C; power on reset to midscale for grades B and D).
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

void mcp_wrapper() { 

  mcp.IRQ_handler(); }


void setup()
{
  unsigned int i;
  unsigned int j;
  pinMode(EXP_CS, INPUT);
  pinMode(ADC_IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(ADC_IRQ), mcp_wrapper, FALLING);

  pinMode(EXP_CS, OUTPUT);
  SPI.begin(SCK, MISO, MOSI, DAC_CS); //EXP_CS
  pinMode(ADC_pinCS, OUTPUT);
  digitalWrite(ADC_pinCS, HIGH);
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
  myMCP.setPortPullUp(0b11111111, A);  // Port A: Pin 0 - 3 are pulled up

  delay(wT);
  myMCP.setPortMode(0b00000000, B);   // Port B: all pins are INPUT - Buttons are inputs
  myMCP.setPortPullUp(0b11111111, B);  // Port B: Pin 4 - 7 are pulled up

  delay(wT);
  myMCP.setAllPins(A, HIGH);            // Port A: all pins are LOW - all DAC amplifiers activated 



  //setup DAC
    SPI.beginTransaction(SPISettings(48000000, MSBFIRST, SPI_MODE1));
    digitalWrite(DAC_CS, HIGH);
    for (int f=0;f<10;++f) {
for (j=0;j<70;++j) {
  for (i=0;i<8;++i)
  {
    DAC8568SetVoltage(i,1000*j);
    //Serial.println(1000*j);
  }
    delay(2);
}

for (j=70;j>0;--j) {
  for (i=0;i<8;++i)
  {
    DAC8568SetVoltage(i,1000*j);
    //Serial.println(1000*j);
  }
    delay(2);
   
}
    }
 
  
 SPI.endTransaction();
digitalWrite(DAC_CS, LOW);


//check for ADC
  if (!mcp.begin()) {
    Serial.println("failed to initialize MCP");
    while (1)    ;
  }
  delay(500);
  Serial.println("MCP setup done");
}

void loop()
{
  unsigned int i;
  unsigned int j;

//ADC

int32_t adcdata4 = mcp.analogRead(MCP_CH4);
  delay(100);
  


int32_t adcdata5 = mcp.analogRead(MCP_CH5);
  delay(100);
int32_t adcdata6 = mcp.analogRead(MCP_CH6);
  delay(100);
int32_t adcdata7 = mcp.analogRead(MCP_CH7);
  delay(100);
// Convert the analog reading

    double voltage4 = adcdata4 * mcp.getReference() / mcp.getMaxValue();
    double voltage5 = adcdata5 * mcp.getReference() / mcp.getMaxValue();
    double voltage6 = adcdata6 * mcp.getReference() / mcp.getMaxValue();
    double voltage7 = adcdata7 * mcp.getReference() / mcp.getMaxValue();

    // print out the value you read:
  

    Serial.print("voltage4: ");
    Serial.println(voltage4, 10);
    
    Serial.print("voltage5: ");
    Serial.println(voltage5, 10);
    
    Serial.print("voltage6: ");
    Serial.println(voltage6, 10);
    Serial.print("voltage7: ");
    Serial.println(voltage7, 10);


//ADC END
  //networkManager.loop();
  delay(1000);
  buttons = myMCP.getPort( B );
  Serial.print("Buttons: ");
  Serial.println(buttons, BIN);

  



//DAC 3x Dreieck output 

  SPI.beginTransaction(SPISettings(48000000, MSBFIRST, SPI_MODE1));
  for (int f=0;f<3;++f) {
for (j=0;j<70;++j) {
  for (i=0;i<8;++i)
  {
    DAC8568SetVoltage(i,1000*j);
  }
    delay(2);
}

for (j=70;j>0;--j) {
  for (i=0;i<8;++i)
  {
    DAC8568SetVoltage(i,1000*j);
  }
    delay(2);
   
}
  }
  SPI.endTransaction();

  



  

}
