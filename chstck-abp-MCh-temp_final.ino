#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS.h> //inclusión de fichero necesario para adecuar la data gps en formato NMEA

//////////////////// Ingreso de parámetros para validación ABP LORAWAN //////////////////////////

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb). PROGMEM: Store data in flash (program) memory instead of SRAM
static const PROGMEM u1_t NWKSKEY[16] = { 0x4C, 0x59, 0xCD, 0xE7, 0x8D, 0x58, 0x69, 0xA7, 0x8B, 0xFB, 0x3D, 0x56, 0xB2, 0x4E, 0xC7, 0xCC };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0x93, 0x0C, 0x62, 0x0B, 0xAF, 0xA0, 0x1B, 0x09, 0x45, 0x9B, 0x02, 0x97, 0x4B, 0x31, 0x3C, 0x76 };

// LoRaWAN end-device address (DevAddr)
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x26011B9F ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/////////////////////// Declaración de variables //////////////////////////////////

static uint8_t mydata[6]; // Variable para almacenar el valor obtenido del sensor (0 a 1023)
static osjob_t sendjob; // Struct tipo osjob_t

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 8;

TinyGPS gps;

//////////////////// Mapeado de pines ///////////////////////////////////////
int pinLM35 = 1; // Variable del pin de entrada del sensor (A0)
// Adapted for Dragino Lora/GPS Shield
const lmic_pinmap lmic_pins = {
    .nss = 10,                       // chip select CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,                       // reset pin
    .dio = {2, 6, 7}, // assumes external jumpers // DIO1 is on JP1-1: is io1 - we connect to GPO6 // DIO2 is on JP5-3: is D2 - we connect to GPO7     
};
 
//////////////////// Funciones ////////////////////////////////////////
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    byte payload[13]; //array en la que se enviará la data a través de un enlace LoRaWAN
    bool newData;
    unsigned long chars;
    unsigned short sentences, failed;
    float flat, flon, falt; //4 bytes
    long flat2, flon2; //4 bytes
    long falt2; // 4 bytes
    int16_t temperature;
    
    unsigned long age;

    for (unsigned long start = millis(); millis() - start < 2000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read(); //varía el buffer del puerto serial 1 y termina el bucle while.
        if (gps.encode(c)) // Did a new valid sentence come in? Arroja un falso o un verdadero.
        { 
          newData = true;
          //gps.f_get_position(&flat, &flon, &age);
          //falt = gps.f_altitude();
        }
      }
    }
//    }
//    
    if (newData)
    {
      gps.f_get_position(&flat, &flon, &age);
      falt = gps.f_altitude();
    }

    flat2 = flat * 1000000; // seis decimales
    flon2 = flon * 1000000; // seis decimales
    falt2 = falt * 100; // dos decimales
    temperature = temp(); //Llamo a la función temp()

    //Construcción del paquete de datos LoRaWAN
    payload[0] = (byte) ((temperature & 0x0000FF00) >> 8  );
    payload[1] = (byte) ((temperature & 0X000000FF)       );     
    payload[2] = (byte) ((flat2 & 0xFF000000) >> 24 );
    payload[3] = (byte) ((flat2 & 0x00FF0000) >> 16 );
    payload[4] = (byte) ((flat2 & 0x0000FF00) >> 8  );
    payload[5] = (byte) ((flat2 & 0X000000FF)       );
    payload[6] = (byte) ((flon2 & 0xFF000000) >> 24 );
    payload[7] = (byte) ((flon2 & 0x00FF0000) >> 16 );
    payload[8] = (byte) ((flon2 & 0x0000FF00) >> 8  );
    payload[9] = (byte) ((flon2 & 0X000000FF)       );
    payload[10] = (byte) ((falt2 & 0x00FF0000) >> 16 );
    payload[11] = (byte) ((falt2 & 0x0000FF00) >> 8  );
    payload[12] = (byte) ((falt2 & 0X000000FF)       );    
  
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        Serial.print("LAT=");
        Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
        Serial.print(" LON=");
        Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
        Serial.print(" ALT=");
        Serial.print(falt == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : falt, 2);
        Serial.print(" TEMP=");      
        Serial.println(((float)temperature)/100);
        LMIC_setTxData2(1,payload,sizeof(payload),0);
        Serial.println(F("Packet queued"));
        Serial.println(LMIC.freq);                                       
      }
    // Next TX is scheduled after TX_COMPLETE event.
}

float temp() {
  // Con analogRead leemos el sensor, recuerda que es un valor de 0 a 1023
  float tempC;
  tempC = analogRead(pinLM35);   
  tempC = (5.0 * tempC * 100.0)/1024.0; // Calculamos la temperatura con la fórmula
  int temp100 = tempC * 100;
  return temp100;
}

//esta función se activa ante un evento o interrupción, por ejemplo cuando el dispositivo escucha
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
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
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
              Serial.println(LMIC.dataLen);
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
            Serial.println(F("Unknown event"));
            break;
    }
}

/////////////// Cuerpo del programa////////////////////
void setup() {
//    pinMode(13, OUTPUT);
    
    while (!Serial); // wait for Serial to be initialized, mientras sea False permanece en el bucle
    Serial.println("Iniciando Puerto Serial 0");
    Serial.begin(115200);
    delay(100);     // 
    Serial.println(F("Iniciando Puerto Serial 1"));
    Serial1.begin(9600);
    
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
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));//memcpy copia el contenido del segundo al primero
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
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);
    
    //Bucle para no enviar data mientras el gps fija su localización
    bool newData0 = false;
    while (!newData0)
    { for (unsigned long start = millis(); millis() - start < 1000;)
      { while (Serial1.available())
        {
          char c0 = Serial1.read(); //vacía el buffer del puerto serial 1 y termina el bucle while.
          if (gps.encode(c0)) // Did a new valid sentence come in? Arroja un falso o un verdadero.
            newData0 = true;    
        }
      }
      Serial.println("Alineando satélites...");
    }

    // Start job
    do_send(&sendjob);
}

void loop() {
  os_runloop_once();
 }
