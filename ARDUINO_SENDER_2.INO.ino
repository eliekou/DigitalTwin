#include <SPI.h> //Import SPI librarey
#include <RH_RF95.h> // RF95 from RadioHead Librarey

#define RFM95_CS 10 //CS if Lora connected to pin 10
#define RFM95_RST 9 //RST of Lora connected to pin 9
#define RFM95_INT 2 //INT of Lora connected to pin 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

static int adc;
static int pdB = 0;
const int MIC =0;
void listen(){}

void listen_(){
  adc = analogRead(MIC);
  pdB = (adc-89.32827) / 10.000140;
  
}



struct Envoi{
  String Capteur;
  
  char donne;
  int mesure;

  uint8_t data1[];
};

Envoi A = {"Capteur 1","son",23,"Capteur de son"};



void setup()
{
//Initialize Serial Monitor
  Serial.begin(9600);

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
}
char value = 48;
void loop()
{



  char Donne_[3] = {A.donne};

  Serial.println(A.mesure);
  int mesure_1 = {A.mesure};

  String Nom = {A.Capteur};

  
  
  //Lecture du son avec le capteur branché
  adc = analogRead(MIC);
  pdB = (adc-89.32827) / 10.000140;
  Serial.println(pdB);//On fait les conversions nécéssaires pour récupérer l'information en décibel

  A.mesure = pdB;

  
  char Textout[20] = "The measure is";
  char Numberout[30];

  itoa(A.mesure,Numberout,10);//Le son en db pdB est convertit en char pour pouvoir l'envoyer après

 

  rf95.send((uint8_t*)Nom.c_str(),Nom.length()+1);
  //Cette ligne envoie le nom du capteur sur la raspberry
  delay(2000);

  rf95.send((uint8_t*)Numberout,10);
  //Cette ligne envoie la valeur du son en db sur la raspberry

  //Les deux informations vont défiler sur la raspberry

  //delay(2000);

  
}
