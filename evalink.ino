//////////////////////////////////////////////////////////////////////////////////
// EVALINK - esp8266 based ppm and serial bridge for quadcopters
//////////////////////////////////////////////////////////////////////////////////

const boolean   CFG_IS_RECEIVER = false;     //true=receiver, false=transmitter
const int       CFG_PROTOCOL = 0;           //0=PPM/Timer0, 1=PPM/bitbang, 2=MSP (Multiwii)
const int       CFG_PPM_IN = 14;            //pin number for PPM input, can be same as CFG_PPM_OUT
const int       CFG_PPM_OUT = 16;           //pin number for PPM output

const byte      CFG_WIFI_CHANNEL = 14;      //wifi channel to use
const int       CFG_LED_PIN = 2;            //0=off, 1=ESP-01, 2=ESP-07/12/etc

const int       CFG_MAX_CHANNELS = 12;      //limit channels in ppm output, cleanflight=12
const int       CFG_RSSI_CHANNEL = -1;      //send rssi thru servo channel (-1=off)
const boolean   CFG_SERIAL_BRIDGE = true;   //activate serial bridge on UART0?
const int       CFG_SERIAL_SPEED = 9600;    //default 115200, use 9600 for frsky telemetry

const boolean   CFG_CHANNEL_HOPPING = false;//true=use channel hopping, false=always use channel CFG_WIFI_CHANNEL
const int       CFG_HOPPING_SEED = 19385;   //change this to a random number

// 1=network 2=pulses 4=rssi 8=serialin 16=serialout 32=
const uint8_t   CFG_DEBUG = 0;

/* some notes:
  - ppm in/out are interrupt based (timer0)
  - no wifi peering/bind needed, we are are using raw beacon packets
  - set chip speed to 160mhz to avoid 1us jitter from isr
  - add heavy filtering to esp8266 power rail to avoid strange crashes
  - some protocols (SBUS, DMS2, SXBL...) are implemented but untested
  - api limits management packet size to 112 bytes, so servo bytes are packed to save space
  - servo byte packing: short to 3 bytes, so 12 bits per short, msb in the middle byte
  - limited serial buffer because of packet size, rx uses servo data space as additional
    serial buffer space, in most cases you get more data from fc than sending to fc
*/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#include <ESP8266WiFi.h>
#include <Servo.h>

#ifdef ESP8266
extern "C" {
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "user_interface.h"
}
#endif

volatile unsigned short pulses[16] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1000, 2000 } ;
volatile int pulseIndex = 0;
volatile unsigned long pulseStart = 0;
volatile boolean pulseFlag = false;

volatile unsigned int errorCount = 0;
volatile unsigned int framesLostRx = 0;
volatile unsigned int framesLostTx = 0;
const int errorLimit = 20;

unsigned long ledTimeout = 0;

os_timer_t errTimer;

os_timer_t txTimer;
static byte txBuffer[64];
volatile boolean txFlag = false;

os_timer_t rxTimer;
int8_t rssiTx = 0;
int8_t rssiRx = 0;
uint16_t seqNumber = 0;
uint16_t lastSeqNumber = 0;

//sdk limits beacon packet size to 112 bytes :(
byte packet[] = { 0x80, 0x00, 0x00, 0x00,
                  /*4*/   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //rec mac
                  /*10*/  0xfe, 0xfe, 0xfe, 0x00, 0x00, 0x01, //src mac
                  /*16*/  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, //bss id
                  /*22*/  0x00, 0x00, //sequence number
                  /*24*/  0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00, //fixed parms: timestamp
                  /*32*/  0x32, 0x00, //fp: beacon interval
                  /*34*/  0x00, 0x04, //fp: caps 0104
                  /* tags start here */
                  /*36*/  0x00, 0x02, 'R', 'C', //SSID
                  /*40*/ 0xE9, 70,
                  /* pkt-type rssi pktloss */
                  /*42*/ 0x00, 0x00,
                  /* packed servo data */
                  /*44*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  /*60*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  /* serial buffer */
                  /*68*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  /*84*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  /*100*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                };

const int OFFSET_CONFIG = 42;
const int OFFSET_SERVO = 44;
const int OFFSET_SERIAL = 68;
const int SERIAL_BUFSIZE = 40;
const int SERIAL_BUFSIZE_RX = 40 + 24;

struct RxControl {
  signed rssi: 8;
  unsigned rate: 4;
  unsigned is_group: 1;
  unsigned: 1;
  unsigned sig_mode: 2;
  unsigned legacy_length: 12;
  unsigned damatch0: 1;
  unsigned damatch1: 1;
  unsigned bssidmatch0: 1;
  unsigned bssidmatch1: 1;
  unsigned MCS: 7;
  unsigned CWB: 1;
  unsigned HT_length: 16;
  unsigned Smoothing: 1;
  unsigned Not_Sounding: 1;
  unsigned: 1;
  unsigned Aggregation: 1;
  unsigned STBC: 2;
  unsigned FEC_CODING: 1;
  unsigned SGI: 1;
  unsigned rxend_state: 8;
  unsigned ampdu_cnt: 8;
  unsigned channel: 4;
  unsigned: 12;
};

struct LenSeq {
  uint16_t length;
  uint16_t seq;
  uint8_t address3[6];
};

struct sniffer_buf {
  struct RxControl rx_ctrl;
  uint8_t buf[36];
  uint16_t cnt;
  struct LenSeq lenseq[1];
};

struct sniffer_buf2 {
  struct RxControl rx_ctrl;
  uint8_t buf[320];
  uint16_t cnt;
  uint16_t len;
};

void setSequenceBuf(uint16_t seq) {
  packet[22] = seq % 255;
  packet[23] = seq / 255;
}

uint16_t getSequenceBuf() {
  return packet[22] + (int16_t)(packet[23] << 8);
}

void setSequenceSnif(struct sniffer_buf2 *sniffer, uint16_t seq) {
  sniffer->buf[22] = seq % 255;
  sniffer->buf[23] = seq / 255;
}

uint16_t getSequenceSnif(struct sniffer_buf2 *sniffer) {
  return sniffer->buf[22] + (int16_t)(sniffer->buf[23] << 8);
}



void errTimerCb(void *pArg) {
  if (errorCount < 200) errorCount++;
  if (CFG_SERIAL_BRIDGE) {
    if (Serial.available() > SERIAL_BUFSIZE / 1.5) {
      if (CFG_IS_RECEIVER) {
        sendRx(NULL);
      } else {
        txFlag = true;
      }
    }
  }
}

uint8_t nextChannel(uint16_t seq) {
  randomSeed(seq + CFG_HOPPING_SEED + 1);
  return random(13);
}

uint8_t currChannel(uint16_t seq) {
  randomSeed(seq + CFG_HOPPING_SEED);
  return random(13);
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// RECEIVER
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void ICACHE_FLASH_ATTR packetCbRx(uint8 *buf, uint16 len) {
  if (len == 128) {
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2*) buf;

    if ((sniffer->buf[OFFSET_CONFIG] & 1) != 1) {
      return;
    }

    //correct SSID?
    for (int i = 0; i < 4; i++) {
      if (sniffer->buf[i + 37] != packet[i + 37]) {
        return;
      }
    }

    if (CFG_SERIAL_BRIDGE) {
      uint8_t ser = sniffer->buf[OFFSET_SERIAL];
      if (ser > 0) {
        for (uint8_t i = 1; i <= ser; i++) {
          if (CFG_DEBUG & 16) {
            Serial.print(sniffer->buf[OFFSET_SERIAL + i], HEX); Serial.print(" ");
          } else {
            Serial.write(sniffer->buf[OFFSET_SERIAL + i]);
          }
        }
      }
    }

    errorCount = 1;
    rssiRx = sniffer->rx_ctrl.rssi;
    rssiTx = sniffer->buf[OFFSET_CONFIG + 1];

    uint16_t seq = getSequenceSnif(sniffer);
    if (lastSeqNumber + 1 != seq && (seq != 0 && lastSeqNumber != 255)) {
      framesLostRx++;
      if (CFG_DEBUG & 1) {
        Serial.print("frame lost. now:"); Serial.print(seq); Serial.print(" last:"); Serial.println(lastSeqNumber);
      }
    }
    lastSeqNumber = seq;

    uint8_t off = OFFSET_SERVO;
    for (int j = 0; j < 16 ; j += 2) {
      pulses[j + 0] = (short)sniffer->buf[off + 0] | (short)((sniffer->buf[off + 1] & 15) << 8);
      pulses[j + 1] = (short)sniffer->buf[off + 2] | (short)((sniffer->buf[off + 1] & 240) << 4);
      off += 3;
    }

    if (CFG_RSSI_CHANNEL != 0) {
      pulses[CFG_RSSI_CHANNEL] = map(rssiRx, 1000, 2000, 0, -48);
    }

    sendRx(NULL);
  }
}

inline void timer0_rearm(uint32_t us) {
  us = clockCyclesPerMicrosecond() * us;
  timer0_write(ESP.getCycleCount() + us);
}

void initRx() {
  os_timer_disarm(&rxTimer);

  switch (CFG_PROTOCOL) {
    case 0: //PPM OUT, timer0
      pinMode(CFG_PPM_OUT, OUTPUT);
      timer0_isr_init();
      timer0_attachInterrupt(sendPpmRxInt0);
      timer0_rearm(10000);
      break;

    case 2: //MSP
      os_timer_setfn(&rxTimer, sendMspRx, NULL);
      os_timer_arm(&rxTimer, 20, 1);
      break;
  }

  os_timer_disarm(&errTimer);
  os_timer_setfn(&errTimer, errTimerCb, NULL);
  os_timer_arm(&errTimer, 200, 1);

  system_phy_set_max_tpw(82);
  wifi_set_phy_mode(PHY_MODE_11N);
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(CFG_WIFI_CHANNEL);
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(packetCbRx);
  wifi_promiscuous_enable(1);
}

void runRx() {
  if (CFG_PROTOCOL == 1) {
    sendPpmRxBitbang();
  }

  if (CFG_DEBUG & 2) {
    for (uint8_t i = 0; i < 16; i++) {
      Serial.print(pulses[i]);
      Serial.print(",");
    } Serial.println("-1");
  }
  if (CFG_DEBUG & 4) {
    Serial.print("rssi:"); Serial.println(rssiRx);
  }
}

void sendRx(void *pArg) {
  packet[OFFSET_CONFIG] = 2;
  packet[OFFSET_CONFIG + 1] = rssiRx;

  packet[OFFSET_SERVO] = 0;
  if (CFG_SERIAL_BRIDGE) {
    uint8_t lim = 1;
    while (Serial.available()) {
      packet[OFFSET_SERVO + lim] = Serial.read();
      if (CFG_DEBUG & 8) {
        Serial.print(packet[OFFSET_SERVO + lim], HEX); Serial.print(" ");
      }
      packet[OFFSET_SERVO]++;
      lim++;
      if (lim > SERIAL_BUFSIZE_RX) break;
    }
  }

  uint8_t chan = CFG_CHANNEL_HOPPING ? nextChannel(seqNumber) : CFG_WIFI_CHANNEL;
  wifi_set_channel(chan);
  wifi_send_pkt_freedom(packet, sizeof(packet), 0);
}


/*
   timing diagram of ppm pulse train
         0.3ms                 1-2ms              GAP 30ms
   PPM """||"""""""""||""""||"""""""""||""""""""""""""""""""""""""""""""||"
   S1 ____|""""""""""|__________________________________________________|""
   S2 _______________|"""""|_______________________________________________
   S3 _____________________|""""""""""|____________________________________
*/
void sendPpmRxInt0() {
  if (errorCount > errorLimit) {
    digitalWrite(CFG_PPM_OUT, HIGH);
    timer0_rearm(50);
    return;
  }

  //raising edge?
  if (!pulseFlag) {
    timer0_rearm(300);
    digitalWrite(CFG_PPM_OUT, LOW);
    pulseFlag = true;
    return;
  }

  //reached gap?
  if (pulseIndex >= CFG_MAX_CHANNELS) {
    timer0_rearm(6640);
    digitalWrite(CFG_PPM_OUT, HIGH);
    pulseFlag = false;
    pulseIndex = 0;
    return;
  }

  //prepare next pulse
  if (pulses[pulseIndex] > 300) {
    timer0_rearm(pulses[pulseIndex] - 300);
  } else {
    timer0_rearm(1200);
  }

  digitalWrite(CFG_PPM_OUT, HIGH);
  pulseFlag = false;
  pulseIndex++;
}

// timer1 crashes? probably hardware, but here is a software fallback
void sendPpmRxBitbang() {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(CFG_PPM_OUT, LOW);
    delayMicroseconds(300);
    digitalWrite(CFG_PPM_OUT, HIGH);
    delayMicroseconds(pulses[i] - 300);
  }
  digitalWrite(CFG_PPM_OUT, LOW);
  delayMicroseconds(300);
  digitalWrite(CFG_PPM_OUT, HIGH);
  delayMicroseconds(30000);
}

//MSP http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
void sendMspRx(void *pArg) {
  txBuffer[0] = '$';
  txBuffer[1] = 'M';
  txBuffer[2] = '<';
  txBuffer[3] = 32; //16*uint16
  txBuffer[4] = 200; //MSP_SET_RAW_RC

  for (uint8_t i = 0; i < 16; i++) {
    txBuffer[5 + (i * 2)] = (byte)(pulses[i] & 0xff);
    txBuffer[6 + (i * 2)] = (byte)((pulses[i] >> 8) & 0xff);
  }
  uint8_t crc = txBuffer[5];
  for (uint8_t i = 1; i < 32; i++) {
    crc = crc ^ txBuffer[5 + i];
  }
  txBuffer[5 + 32] = crc;
  Serial.write(txBuffer, 32 + 6);
}

/*
  //DSM2 https://www.spektrumrc.com/ProdInfo/Files/Remote%20Receiver%20Interfacing%20Rev%20A.pdf
  void sendDsm2Rx(void *pArg) {
  Serial.write((unsigned short)errorCount);
  unsigned short v;
  for (uint8_t i = 0; i < 7; i++) {
    v = (i & 16) << 11;
    v = v | (pulses[i] & 0x1111111111);
    Serial.write(v);
  }
  }

  //https://developer.mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
  void sendSbusRx(void *pArg) {
  uint8_t i;
  txBuffer[0] = 0xf0;
  for (i = 1; i < 24; i++) {
    txBuffer[i] = 0;
  }

  uint8_t ch = 0;
  uint8_t bit_in_servo = 0;
  uint8_t byte_in_sbus = 1;
  uint8_t bit_in_sbus = 0;

  // store servo data
  for (i = 0; i < 176; i++) {
    //uint16_t pulse = pulses[ch];

    if (pulses[ch] & (1 << bit_in_servo)) {
      txBuffer[byte_in_sbus] |= (1 << bit_in_sbus);
    }
    bit_in_sbus++;
    bit_in_servo++;

    if (bit_in_sbus == 8) {
      bit_in_sbus = 0;
      byte_in_sbus++;
    }
    if (bit_in_servo == 11) {
      bit_in_servo = 0;
      ch++;
    }
  }

  if (framesLostRx > 0) {
    txBuffer[23] |= (1 << 2); //frame lost
  }
  if (errorCount > errorLimit) {
    txBuffer[23] |= (1 << 3); //failsafe active
  }

  Serial.write(txBuffer, 25);
  }

  uint16_t CRC16(u16 crc, u8 value) {
  crc = crc ^ (int16_t)value << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = crc << 1 ^ 0x1021;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
  }

  // SXRL mode B
  // 0xA2 CH1MSB CH2LSB ... C16MSB C16LSB CRCMSG CRCLSB
  // 0    1      2          31     32     33     34      = 35 bytes
  void sendSxrlRx(void *pArg) {
  txBuffer[0] = 0xa2; //version 2, 16ch

  for (uint8_t i = 0; i < 16; i++) {
    txBuffer[1 + (i * 2)] = pulses[i] / 255;
    txBuffer[2 + (i * 2)] = pulses[i] % 255;
  }

  uint16_t crc = 0;
  for (uint8_t i = 1; i < 33; i++) {
    crc = CRC16(crc, txBuffer[i]);
  }
  txBuffer[33] = crc / 255;
  txBuffer[34] = crc & 255;

  Serial.write(txBuffer, 35);
  }
*/
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// TRANSMITTER
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void ICACHE_FLASH_ATTR packetCbTx(uint8 *buf, uint16 len) {
  if (len == 128) {
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2*) buf;

    if ((sniffer->buf[OFFSET_CONFIG] & 2) != 2) {
      return;
    }

    //correct SSID?
    for (int i = 0; i < 4; i++) {
      if (sniffer->buf[i + 37] != packet[i + 37]) {
        return;
      }
    }

    if (CFG_SERIAL_BRIDGE) {
      uint8_t ser = sniffer->buf[OFFSET_SERVO];
      if (ser > 0) {
        for (uint8_t i = 1; i <= ser; i++) {
          if (CFG_DEBUG & 16) {
            Serial.print(sniffer->buf[OFFSET_SERVO + i], HEX); Serial.print(" ");
          } else {
            Serial.write(sniffer->buf[OFFSET_SERVO + i]);
          }
        }
      }
    }

    errorCount = 1;
    rssiTx = sniffer->rx_ctrl.rssi;//negative!
    rssiRx = sniffer->buf[OFFSET_CONFIG + 1];

    uint16_t seq = getSequenceSnif(sniffer);
    if (lastSeqNumber + 1 != seq) {
      framesLostTx++;
      if (CFG_DEBUG & 1) {
        Serial.print("frame lost. now:"); Serial.print(seq); Serial.print(" last:"); Serial.println(lastSeqNumber);
      }
    }
    lastSeqNumber = seq;
  }
}

void initTx() {
  pinMode(CFG_PPM_IN, INPUT);
  attachInterrupt(CFG_PPM_IN, ppmInterruptTx, RISING);

  system_phy_set_max_tpw(82);
  wifi_set_phy_mode(PHY_MODE_11N);
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(CFG_WIFI_CHANNEL);
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(packetCbTx);
  wifi_promiscuous_enable(1);

  os_timer_disarm(&errTimer);
  os_timer_setfn(&errTimer, errTimerCb, NULL);
  os_timer_arm(&errTimer, 200, 1);

  os_timer_disarm(&txTimer);
  os_timer_setfn(&txTimer, sendTx, NULL);
  os_timer_arm(&txTimer, 2, 1);
}

void sendTx(void *pArg) {
  uint8_t chan;

  if (txFlag == false) return;
  txFlag = false;

  uint8_t off = OFFSET_SERVO;
  for (uint8_t j = 0; j < 16; j += 2) {
    packet[off + 0] = (byte)(pulses[j + 0] & 255);
    packet[off + 1] = (byte)((pulses[j + 0] >> 8) & 15) | (byte)(((pulses[j + 1] >> 8) & 15) << 4);
    packet[off + 2] = (byte)(pulses[j + 1] & 255);
    off += 3;
  }
  packet[OFFSET_CONFIG] = 1;
  packet[OFFSET_CONFIG + 1] = rssiTx;

  packet[OFFSET_SERIAL] = 0;
  if (CFG_SERIAL_BRIDGE) {
    uint8_t lim = 1;
    while (Serial.available()) {
      packet[OFFSET_SERIAL + lim] = Serial.read();
      if (CFG_DEBUG & 8) {
        Serial.print(packet[OFFSET_SERIAL + lim], HEX); Serial.print(" ");
      }
      packet[OFFSET_SERIAL]++;
      lim++;
      if (lim >= SERIAL_BUFSIZE) break;
    }
  }

  setSequenceBuf(seqNumber);

  chan = CFG_CHANNEL_HOPPING ? currChannel(seqNumber) : CFG_WIFI_CHANNEL;
  wifi_set_channel(chan);
  wifi_send_pkt_freedom(packet, sizeof(packet), 0);
  chan = CFG_CHANNEL_HOPPING ? nextChannel(seqNumber) : CFG_WIFI_CHANNEL;
  wifi_set_channel(chan);

  seqNumber++;
}

void runTx() {
  if (CFG_DEBUG & 2) {
    for (uint8_t i = 0; i < 16; i++) {
      Serial.print(pulses[i]); Serial.print(" ");
    }
    Serial.println();
  }
  if (CFG_DEBUG & 4) {
    Serial.print("rssi:"); Serial.println(rssiTx);
  }
}

void ppmInterruptTx() {
  unsigned long diff = 0;

  os_intr_lock();

  if (pulseStart > 0) {
    diff = (unsigned long)(micros() - pulseStart);
    pulseStart = micros();
  } else {
    pulseStart = micros();
    goto leave;
  }

  //SYNC
  if (diff > 3000) {
    pulseIndex = 0;
    errorCount = 0;
    txFlag = true;
    goto leave;
  }

  //SERVO
  if (diff > 400 && diff < 2600) {
    if (pulseIndex < 16) {
      pulses[pulseIndex++] = diff;
    }
    errorCount = 0;
    goto leave;
  }

  errorCount++;

leave:
  os_intr_unlock();
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// setup/loop
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void initLed() {
  pinMode(CFG_LED_PIN, OUTPUT);
  digitalWrite(CFG_LED_PIN, LOW);
}

void runLed() {
  signed long limit = errorCount < errorLimit ? 300 : 25;
  if ((signed long)(millis() - ledTimeout) > limit) {
    ledTimeout = millis() + limit;
    digitalWrite(CFG_LED_PIN, digitalRead(CFG_LED_PIN) == LOW ? HIGH : LOW);
  }
}

void setup() {
  delay(500);
  if (CFG_SERIAL_BRIDGE) {
    Serial.begin(CFG_SERIAL_SPEED);
  }

  byte mac[6];
  WiFi.macAddress(mac);
  for (uint8_t i = 0; i < 6; i++) {
    packet[16 + i] = mac[i];
    packet[10 + i] = mac[i];
  }

  if (CFG_LED_PIN) {
    initLed();
  }
  if (CFG_IS_RECEIVER) {
    initRx();
  } else {
    initTx();
  }

  os_timer_disarm(&errTimer);
  os_timer_setfn(&errTimer, errTimerCb, NULL);
  os_timer_arm(&errTimer, 20, 1);
}

void loop() {
  if (CFG_LED_PIN) {
    runLed();
  }
  if (CFG_IS_RECEIVER) {
    runRx();
  } else {
    runTx();
  }

  ESP.wdtFeed();
  yield();
}
