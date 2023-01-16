#include <lmic.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "Adafruit_NeoPixel.h"
#ifdef __AVR__
#include <avr/power.h>
#endif

//Nouveau include


#include <SPI.h> //Import SPI librarey 

#include <RH_RF95.h> // RF95 from RadioHead Librarey 

#define RFM95_CS 10 //CS if Lora connected to pin 10

#define RFM95_RST 9 //RST of Lora connected to pin 9

#define RFM95_INT 2 //INT of Lora connected to pin 2

// Change to 434.0 or other frequency, must match RX's freq!

#define RF95_FREQ 434.0

// Singleton instance of the radio driver

RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define LEDSTRIP        5
#define NUMPIXELS      10

const int MIC = A0; //the microphone amplifier output is connected to pin A0
int adc,i;
int dB;
static int PdB=0; //the variable that will hold the value read from the microphone each time
int delayval = 50;// 500 origine
int red[]={0,111,152,182,205,223,238,248,254,255};
int green[]={255,237,219,199,178,155,130,102,68,0};
char value = 40;





Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDSTRIP, NEO_GRB + NEO_KHZ800);



// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x22, 0x2F, 0x8D, 0x7B, 0x07, 0x37, 0xA6, 0xC4, 0x2C, 0x56, 0xFA, 0xE8, 0x3E, 0x7D, 0x7A, 0x08  };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x20, 0xD7, 0xD5, 0xA5, 0x22, 0x5A, 0xB6, 0x0A, 0x80, 0x37, 0xD2, 0x9E, 0x48, 0xBE, 0x27, 0xE7 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR  = 0x260B3CBA;


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// payload to send to TTN gateway
uint8_t payload[6];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;// origine 10

// Pin mapping for Adafruit Feather M0 LoRa
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
    
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);// rajouté  Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
            
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
       
        for (i=0;i<5;i++){
        adc= analogRead(A0); //Read the ADC value from amplifer 
        Serial.print("adc"); //rajouté
        Serial.println(adc);
//        dB = (adc-89.32827) / 10.000140; //Convert ADC value to dB using Regression values
//        PdB=adc-500;
          dB = (adc / 10.00); //remplace les 2 lignes du dessus.
          PdB=dB+30;
//        //PdB+=dB;//origine
//        PdB=byte(dB);//mise en commentaire
//        Serial.print("PdB_test:");//rajouté commentee new
        Serial.println(dB);//rajouté
        Serial.println(PdB);//rajouté
        byte dBbyte = byte(dB);// a revoir 
//        Serial.print("dByte:");
//        Serial.println(dBbyte);// rajouté
//        Serial.print(dBbyte);  // partie commentee new
//        Serial.print("->");
//        Serial.println(dBbyte,HEX);
        payload[i] = dBbyte;
        delay(2);  // delay en milliseconds 10000 => 100 pour test
        }
           
//        PdB/=5;  mis en commentaire
        Serial.println("TurnLED");
//        Serial.print("PdB=");//rajouter
//        Serial.println(PdB);
//        Serial.println(dB);
        TurnLED(PdB);
        
        LMIC_setTxData2(1,payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
       
      }
    // Next TX is scheduled after TX_COMPLETE event.
}

void TurnLED(int PdB)
{
//  pixels.clear();// rajouté
//  if( PdB <= 30){//if( PdB <= 60){
//          Serial.println("60");//rajouté
//          Serial.println(PdB);
//    for(int j=0;j<5;j++){
//          pixels.setPixelColor(j, pixels.Color(red[j],green[j],0)); }
//          // green red 
//          pixels.show(); 
//
//          delay(delayval); 
//          }
//  else if (( PdB > 30) and (PdB <= 90)){//(( PdB > 60) and (PdB < 90))
//          Serial.println("75");
//          Serial.println(PdB);
//    for(int j=0;j<8;j++){
//          pixels.setPixelColor(j, pixels.Color(red[j],green[j],0));} 
//          pixels.show(); 
//
//          delay(delayval); 
//          }
//  else if (PdB > 90) {//>=
//          Serial.println("90");
//          Serial.println(PdB);
//    for(int j=0;j<10;j++){
//          pixels.setPixelColor(j, pixels.Color(red[j],green[j],0)); }
//          pixels.show(); 
//          delay(delayval); 
//          }
}
/*for(int j=0;j<NUMPIXELS;j++){
          if( PdB <= 60){
          pixels.setPixelColor(j, pixels.Color(0,255,0)); 
          pixels.show(); 
          delay(delayval); 
          }
          else if (( PdB > 60) and (PdB < 90)){
          pixels.setPixelColor(j, pixels.Color(255,165,0)); 
          pixels.show(); 
          delay(delayval); 
          }
          else if (PdB >= 90) {
          pixels.setPixelColor(j, pixels.Color(255,0,0)); 
          pixels.show(); 
          delay(delayval); 
          }
        }*/
void setup() {
    Serial.begin(115200);
    pixels.setBrightness(255);
    pixels.begin(); // This initializes the NeoPixel library.
    pinMode(A0, INPUT);
    Serial.println(F("Starting"));
    pixels.setBrightness(64);






//"""New code"""

    //Initialize Serial Monitor
  
    //Serial.begin(9600);
  
    
  
    // Reset LoRa Module 
  
    pinMode(RFM95_RST, OUTPUT); 
  
    digitalWrite(RFM95_RST, LOW);
  
    delay(10);
  
    digitalWrite(RFM95_RST, HIGH);
  
    delay(10);
  
    //Initialize LoRa Module
  
    while (!rf95.init()) {
  
      Serial.println("LoRa radio init failed");
  
      while (1);
  
    }
  
    
  
   //Set the default frequency 434.0MHz
  
    if (!rf95.setFrequency(RF95_FREQ)) {
  
      Serial.println("setFrequency failed");
  
      while (1);
  
    }
  
    rf95.setTxPower(18); //Transmission power of the Lora Module



    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {

  os_runloop_once();
  
  Serial.print("Send: ");

  char radiopacket[1] = "ar";

  rf95.send((uint8_t *)radiopacket, 1);

    

  //delay(1000);

 

  Serial.print("Sonlu:");
  Serial.println(analogRead(A0));
  Serial.println("Pdb = ");
  Serial.println(PdB);
  {//partie déplacéee 
  pixels.clear();// rajouté


       adc= analogRead(A0); //Read the ADC value from amplifer 
        Serial.print("adc"); //rajouté
        Serial.println(adc);
//        dB = (adc-89.32827) / 10.000140; //Convert ADC value to dB using Regression values
//        PdB=adc-500;
          dB = (adc / 10.00); //remplace les 2 lignes du dessus.
          PdB=dB+30;
//        //PdB+=dB;//origine
//        PdB=byte(dB);//mise en commentaire
//        Serial.print("PdB_test:");//rajouté commentee new
        Serial.println(dB);//rajouté
        Serial.println(PdB);//rajouté
        byte dBbyte = byte(dB);// a revoir 
//        Serial.print("dByte:");
//        Serial.println(dBbyte);// rajouté
//        Serial.print(dBbyte);  // partie commentee new
//        Serial.print("->");
//        Serial.println(dBbyte,HEX);
  if( PdB <= 35){//if( PdB <= 60){
          Serial.println("60");//rajouté
          Serial.println(PdB);
    for(int j=0;j<4;j++){
          pixels.setPixelColor(j, pixels.Color(red[j],green[j],0)); }
          // green red 
          pixels.show(); 
          delay(delayval); 
          }
  else if (( PdB > 35) and (PdB <= 45)){//(( PdB > 60) and (PdB < 90))
          Serial.println("75");
          Serial.println(PdB);
    for(int j=0;j<6;j++){
          pixels.setPixelColor(j, pixels.Color(red[j],green[j],0));} 
          pixels.show(); 
          delay(delayval); 
          }
  else if (( PdB > 45) and (PdB <= 55)){//(( PdB > 60) and (PdB < 90))
          Serial.println("75");
          Serial.println(PdB);
          Serial.println("Test de db");
    for(int j=0;j<8;j++){
          pixels.setPixelColor(j, pixels.Color(red[j],green[j],0));} 
          pixels.show(); 
          delay(delayval); 
          }
  else if (PdB > 55) {//>=
          Serial.println("90");
          Serial.println(PdB);
    for(int j=0;j<10;j++){
          pixels.setPixelColor(j, pixels.Color(red[j],green[j],0)); }
          pixels.show(); 
          delay(delayval); 
          }
}
}
