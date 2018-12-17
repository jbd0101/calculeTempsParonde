/*

    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND
   screen oled :
        vcc: 5v
        gnd: gnd
        data: d2
        clk: d3
        rst: d4
 */
 #include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 433.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      5
  #define RFM69_INT     7
  #define RFM69_RST     9
  #define LED           13
#endif

#define SCREEN true

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

char creditText[ ] = "Jean-Christ. bauduin";
int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
long base_dist=0;
long duration, cm, inches;
float pourcentage = 0;
unsigned long distance =0;


RH_RF69 rf69(RFM69_CS, RFM69_INT);
const unsigned long ECHO_INTERVAL = 250;
unsigned long previousMillis = 0;
unsigned long startMillis = 0;
unsigned long endMillis = 0;
boolean enCourse = false;
int16_t packetnum = 0; // packet counter, we increment per xmission
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  if(SCREEN==true){
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  }
    delay(2000);

  // Clear the buffer.
  printscreen(F("\n Demarrage"),creditText);
  delay(500);
 

  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println(F("Feather RFM69 TX Test!"));
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  printscreen(F("\n Radio"),creditText);

  if (!rf69.init()) {
    Serial.println(F("RFM69 radio init failed"));
      printscreen(F("\n ERREUR RADIO"),creditText);

    while (1);
  }
  printscreen(F("radio 433mhz"),creditText);
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println(F("setFrequency failed"));
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  printscreen(F("radio OK"),creditText);
  delay(250);
  
  
  printscreen(F("\n Calibrage"),creditText);
  delay(500);
  int tmp = 0;
  for(int i=0;i<10;i++){
    printscreen((String)"\nCalibr. "+i,creditText);

    calcule_distance();
    delay(500);
    tmp += cm;
    Serial.println(cm);

  }
  base_dist = (tmp / 10);
  delay(500);
  printscreen(F("\n Detect B2"),creditText);
  if(SCREEN ==true){
    while (true){
      printscreen("\n Test Base",creditText);
  
      if(sendPassage() == true){
  
        break;
      }else{
       printscreen("\n Base Not F.",creditText);
       delay(350);
  
      }
    }
  
  }
}
 
void loop() {

 unsigned long currentMillis = millis();
  
  // Si ECHO ou plus millisecondes se sont écoulés
  if(currentMillis - previousMillis >= ECHO_INTERVAL) {
    
    // Garde en mémoire la valeur actuelle de millis()
    previousMillis = currentMillis;
    
    // Inverse l'état de la LED
    calcule_distance();
    if(enCourse ==false){
      printscreen((String)"\n READY",(String)"Dist: "+cm+"cm | "+pourcentage+"%");
    }else{
      printscreen((String)"\n "+(millis()-startMillis)/1000.0+"s",(String)"Dist: "+cm+"cm");

    }
    if(pourcentage < 30){
        printscreen((String)"\n Passage",(String)"Dist: "+cm+"cm | "+pourcentage+"%");
        
        // uniquement l arduino qui n a pas d ecran envoit des donnees de passage !
        if(SCREEN == false){
          sendPassage();
        }else{
          enCourse = true;
          startMillis = millis();
        }
        delay(3000);
    }
  }

  //test_passage();
  //verificaiton si on a des des passages
   if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      endMillis = millis();

      Serial.println((char*)buf);
      Serial.print(F("RSSI: "));
      Serial.println(rf69.lastRssi(), DEC);
      if(SCREEN==true){
        printscreen(F("\n PASSAGE B2"),F("Passage balise 2"));
      }
      if (strstr((char *)buf, "PASSAGE")) {
        // Send a reply!
        uint8_t data[] = "OK";
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
        Serial.println(F("Sent a reply"));
        delay(2000);
      }
      if(SCREEN==true){
        printscreen(F("\n PASSAGE B2"),F("Passage balise 2"));
        delay(250);
        printscreen((String)"\n "+(endMillis-startMillis)/1000.0+"s",F("Passage balise 2"));
        delay(5000);
        enCourse=false;
        startMillis=0;
        endMillis=0;
      }
    } else {
      Serial.println(F("Receive failed"));
    }
  }
}

