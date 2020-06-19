#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include "heltec.h"
//#include "TheThingsNetwork.h"


//DHT
#define DHTPIN 13 // Pino 13 do Hletec
#define DHTTYPE DHT11 // DHT 11
 
// Instancia DHT
DHT dht(DHTPIN, DHTTYPE);



//Por padrão, o código usa dragino. Se usar Heltec, descomentar linah abaixo.
#define heltec

//Define qual dos dispositivos será compilado. Somente um por vez pode ser compilado
//#define ttn_dragino 
#define ttn_heltec_forte 
//#define ttn_heltec_fraco 

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//static const PROGMEM u1_t NWKSKEY[16] = { 0x62, 0x2A, 0x51, 0x71, 0xE1, 0xD8, 0x20, 0x1B, 0x45, 0x60, 0x11, 0x37, 0xFE, 0x9F, 0x1B, 0x63 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//static const u1_t PROGMEM APPSKEY[16] = { 0x11, 0xC7, 0xF4, 0xF7, 0x43, 0xB2, 0xE1, 0x31, 0x69, 0x84, 0x6E, 0xE9, 0xEE, 0x9F, 0x87, 0xD0 };

//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
//static const u4_t DEVADDR = 0x2603149D;

#ifdef ttn_dragino
  static const PROGMEM u1_t NWKSKEY[16] = { 0x8B, 0x19, 0x63, 0xA9, 0x98, 0x9C, 0xEC, 0x9B, 0xC6, 0x7D, 0xF7, 0xD8, 0xD3, 0x8A, 0x40, 0xE2 };
  static const u1_t PROGMEM APPSKEY[16] = { 0x7B, 0x2D, 0x93, 0xEA, 0x74, 0x34, 0x9C, 0x65, 0x55, 0x2A, 0xB7, 0xA7, 0x15, 0xB9, 0x3D, 0x4C };
  static const u4_t DEVADDR = 0x2603167B;
#endif
#ifdef ttn_heltec_forte
  static const PROGMEM u1_t NWKSKEY[16] = { 0x2C, 0xB4, 0xDB, 0xFF, 0x01, 0xB8, 0x46, 0x2E, 0xEF, 0x82, 0x5F, 0xFC, 0x89, 0x83, 0xB0, 0xD0 };
  static const u1_t PROGMEM APPSKEY[16] = { 0xBF, 0xEE, 0x49, 0x92, 0x7E, 0x7F, 0x4C, 0x88, 0x79, 0xE7, 0xA7, 0x71, 0x0F, 0x40, 0xD5, 0x64 };
  static const u4_t DEVADDR = 0x26031448;
#endif
#ifdef ttn_heltec_fraco
  static const PROGMEM u1_t NWKSKEY[16] = { 0xF7, 0x0F, 0x22, 0x84, 0x5C, 0x07, 0x50, 0x84, 0x31, 0x5C, 0x0D, 0x5B, 0x54, 0x96, 0xBC, 0x3F };
  static const u1_t PROGMEM APPSKEY[16] = { 0x1C, 0xF6, 0xAC, 0x56, 0x01, 0x38, 0xA7, 0xCC, 0xD7, 0xC1, 0x2E, 0xF9, 0x0B, 0x1A, 0x54, 0x52 };
  static const u4_t DEVADDR = 0x26031DAA;
#endif

int contador = 0;
//Usado para diferenciar par e ímpar
int resto = 0;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//Limite 51 bytes
int tamanho_vetor = 51;
static uint8_t  mydata[51];

//Talvez deixar só o sendjob
//static osjob_t initjob, sendjob, blinkjob;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 3; //Padrão 60

#ifdef heltec
//Pin mapping heltec
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 35, 34}, //antigo 26,33,32
};
#else
// Pin mapping Dragino
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
}; 
#endif

void do_send(osjob_t* j) {

  // Parâmetros DHT
  // Cria string que armazenará os dados de temperatura e umidade
  String myString;
  //Delay para leitura dos dados
  //delay(TX_INTERVAL * 1000);
  
  //Limpa display
  Heltec.display->clear();
  // A leitura da temperatura e umidade pode levar 250ms!
  // O atraso do sensor pode chegar a 2 segundos.
  
  // Armazena dados da temperatura e umidade
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  // testa se retorno é valido, caso contrário algo está errado.
  if (isnan(t) || isnan(h)) 
  {
    Serial.println("Failed to read from DHT");
  } 
  else
  {
   //Mostra os dados no display
    myString = String(t);
    myString = myString + " ºC";
    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->drawString(0, 0, "Temperatura");
    Heltec.display->setFont(ArialMT_Plain_24);
    Heltec.display->drawString(0, 15, myString);
    myString = "Umidade: ";
    myString = myString + String(h);
    myString = myString + " %";
    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->drawString(0, 50, myString);
    Heltec.display->display();
    
  }
  if (t > 15){
    //digitalWrite(2, HIGH);
    //digitalWrite(3,LOW);
  }else if(t >= 15 || t < 20){
    //digitalWrite(2, HIGH);
    //digitalWrite(3,HIGH);
  }else{
    //digitalWrite(2, LOW);
    //digitalWrite(3,HIGH);
  }


  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    resto = contador % 2;
    
    //Usa Mystring para formar um único texto para escrita
    //na console e transmissão para LoRa
    myString = "Temp: ";
    myString = myString + String(t);
    myString = myString + " ºC - ";
    myString = myString + "Umid: ";
    myString = myString + String(h);
    myString = myString + " %";
    
    //Printa na Serial
    Serial.println(myString);

    //Converte para const void para copair para memória do LoRa
    const void * text = myString.c_str();

    //Copia dados para memória do LoRa
    memcpy(mydata, text, sizeof(mydata));

    // Condicional abaixo utilziado para variar dados entre pares e ímpares
    // if (resto == 1) {
    //   //dtostrf(0, 5, 2, (char*)mydata);
    //   memcpy(mydata, text, sizeof(mydata));
    //   //for (int i = 0; i < tamanho_vetor; i++){
    //   //    mydata[i] = (uint8_t) "2";
    //   //}
    //   //Serial.println(mydata);
    // }
    // else{
    //   //dtostrf(1, 5, 2, (char*)mydata);
    //   memcpy(mydata, text, sizeof(mydata));
    //   //for (int i = 0; i < tamanho_vetor; i++){
    //   //    mydata[i] = (uint8_t) "1";
    //   //}
    // } 
   
    // Prepare transmission at the next possible time.
    LMIC_setTxData2(1, mydata, strlen((char*) mydata), 0);
    //LMIC_setTxData();
    Serial.println(); 
    Serial.println("Packet queued");
    Serial.print("TX nº: ");
    Serial.println(contador);
    Serial.println((char*)mydata);
    contador++;
    Serial.println(LMIC.freq);
    //Serial.print("Temperatura = ");
    //Serial.print(temperatura);
   //Serial.println(" *C");
  }
}
void onEvent (ev_t ev) {
  //sleep(5);
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.print("EV: ");
  Serial.println(ev);
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println("EV_BEACON_FOUND");
      //LMIC_sendAlive();
      break;
    case EV_BEACON_MISSED:
      Serial.println("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println("EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println("EV_JOINED");
      //LMIC_setPingable(1);
      //Serial.println("SCANNING...");
      break;
    case EV_RFU1:
      Serial.println("EV_RFU1");
      break;
    case EV_JOIN_FAILED:
      Serial.println("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println("EV_REJOIN_FAILED");
      break;
    case EV_TXCOMPLETE:
      Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print("Data Received: ");
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }else{
        Serial.println("Nada recebido");
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      Serial.println("Agendando nova transmissão...");
      break;
    case EV_LOST_TSYNC:
      Serial.println("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println("EV_LINK_ALIVE");
      break;
    default:
      Serial.println("Unknown event");
      break;
  }
}

void setup() {

  //Inicialização Display
  Heltec.begin(true, false, true);
 
  Heltec.display->setContrast(255);
  Heltec.display->clear();
  
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "Ligando sensor...");
  Heltec.display->display();

  //Inicialização console
  Serial.begin(9600);
  Serial.println("DHTxx test!");
  
  //Inicialização DHT
  dht.begin();

  //Inicialização Lora
  //SPI.begin(5, 19, 27);
  Serial.begin(115200); //Talvez alterar para 115200
  
  //analogReference(INTERNAL); // Talvez comentar
  while (!Serial);
  delay(5000);
  Serial.println("Starting...");
  
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
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
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

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF10, 18); // Ver se GW está no 10; 14 é 14dBM

  //Deixa canal único
  // for (int i = 1; i < 64; i++)
  // {
  //   LMIC_disableChannel(i);  // only the first channel 902.3Mhz works now.
  //   Serial.println("Desabilitando canal ");
  // }

  //Desabilita os canais desnecessários dos Gateways de Caxias.
  
  for (int i = 0; i < 7; i++)
  {
    LMIC_disableChannel(i);  
    Serial.println("Desabilitando canal ");
  }
  
  for (int i = 16; i < 63; i++) //alterado i de 15 para 16 para fechar envios a cada 10 min
  {
    LMIC_disableChannel(i);  
    Serial.println("Desabilitando canal ");
  }
  

  //Original. Tem que alterar lorabase.h para usar essa sequencia
  //for (int i = 0; i < 8; i++)
  //{
  //  LMIC_disableChannel(i);  
  //  Serial.println("Desabilitando canal ");
  //}

  //for (int i = 16; i < 71; i++)
  //{
  //  LMIC_disableChannel(i);  
  //  Serial.println("Desabilitando canal ");
  //}

  // Start job
  do_send(&sendjob);
  
}

void loop() {
  
  #ifndef heltec
  //Aguarda resposta para TTN retransmitir ao dispositivo
  os_runloop_once();
  #else
  //Heltec não abre a janela de RX. Causa desconehcida. BW 500
  os_runloop_once();
  //delay(TX_INTERVAL * 1000);
  //LMIC_clrTxData();
  //do_send(&sendjob);
  #endif
  //Serial.print("Frequencia: ");
  //Serial.println(LMIC.freq);
  
  //Serial.println("Teste");
  //Retorna valores
  //temp_lida = analogRead(LM35);
  //temperatura = temp_lida * 0.1075268817204301;


  
}
