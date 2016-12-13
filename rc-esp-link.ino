//////////////////////////////////////////////////////////////////////////////////
// RC-ESP-LINK - esp8266 based ppm and serial bridge for quadcopters
//////////////////////////////////////////////////////////////////////////////////

const boolean IS_RX = false; //true=receiver, false=transmitter

const int PROTOCOL = 0; //0=PPM/Timer0, 1=PPM/bitbang, 2=MSP (Multiwii)
const int PPM_IN = 14; //pin number, can be same as PPM_OUT
const int PPM_OUT = 16;

byte WIFI_CHANNEL = 7; //wifi channel to use

const int LED_PIN = 2; //0=off, 1=ESP-01, 2=ESP-07/12/etc

/* some notes:
  - ppm in/out are interrupt based (timer0)
  - no wifi peering/bind needed, we are are using raw beacon packets
  - set chip speed to 160mhz to avoid 1us jitter from isr
  - add heavy filtering to esp8266 power rail to avoid strange crashes
  - some protocols (SBUS, DMS2, SXBL...) are implemented but untested

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
volatile unsigned int framesLost = 0;
const int errorLimit = 20;

unsigned long ledTimeout = 0;

os_timer_t errTimer;

os_timer_t txTimer;
static byte txBuffer[64];
volatile boolean txFlag = false;

os_timer_t rxTimer;
long rssi = 0;
uint16_t seqNumber = 0;
uint16_t lastSeqNumber = 0;

uint8_t packet[] = { 0x80, 0x00, 0x00, 0x00,
                     /*4*/   0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                     /*10*/  0xfe, 0xfe, 0xfe, 0x00, 0x00, 0x01, //dest mac
                     /*16*/  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, //src mac
                     /*22*/  0x00, 0x00, //sequence number
                     /*24*/  0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00, //fixed parms
                     /*32*/  0x32, 0x00, //beacon interval
                     /*34*/  0x01, 0x04, //caps
                     /*36*/  0x00, 0x06, 'R', 'C', 'E', 'S', 'P', '1', //SSID
                     0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c, //rates
                     0x03, 0x01, 0x07, //channel
                     /*57*/ 0xe9, 144,
                     /*59*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*75*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*s 91*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*107*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*113*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*139*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*155*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*171*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     /*187*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                   };

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
  uint8_t buf[300];
  uint16_t cnt;
  uint16_t len;
};

void errTimerCb(void *pArg) {
  if (errorCount < 200) errorCount++;
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

    if (sniffer->buf[15]!=1) {
        return;
    }

    //correct SSID?
    for (int i = 0; i < 6; i++) {
      if (sniffer->buf[i + 38] != packet[i + 38]) {
        return;
      }
    }

    uint8_t ser = sniffer->buf[91];
    if (ser>0) {
        for (uint8_t i=0;i<ser;i++) {
            Serial.write(sniffer->buf[92+i]);
        }
    }

    errorCount = 1;
    rssi = sniffer->rx_ctrl.rssi;//negative!

    seqNumber = sniffer->buf[22] | (sniffer->buf[23] << 8);
    if (lastSeqNumber + 1 != seqNumber) {
      framesLost++;
    }
    lastSeqNumber = seqNumber;

    for (int j = 0; j < 16 ; j++) {
      pulses[j] = (short)(sniffer->buf[59 + (j * 2)] << 8) | (short)sniffer->buf[60 + (j * 2)];
    }
  }
}

inline void timer0_rearm(uint32_t us) {
  us = clockCyclesPerMicrosecond() * us;
  timer0_write(ESP.getCycleCount() + us);
}

void initRx() {
  os_timer_disarm(&rxTimer);

  switch (PROTOCOL) {
    case 0: //PPM OUT, timer0
      pinMode(PPM_OUT, OUTPUT);
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

  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(WIFI_CHANNEL);
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(packetCbRx);
  wifi_promiscuous_enable(1);
}

void runRx() {
  if (PROTOCOL == 1) {
    sendPpmRxBitbang();
  }
  if (Serial.available()) {
    sendRx(NULL);
  }
}

void sendRx(void *pArg) {
  packet[56] = WIFI_CHANNEL;
  packet[15] = 2;

  packet[22] = seqNumber % 255;
  packet[23] = seqNumber / 255;
  seqNumber++;

  uint8_t lim = Serial.available();
  if (lim > 0) {
    if (lim > 111) lim = 111;
    packet[91] = lim;
    for (uint8_t i = 0; i < lim; i++) {
      packet[i + 92] = Serial.read();
    }
  } else {
    packet[91] = 0;
  }

  wifi_set_channel(WIFI_CHANNEL);
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
    digitalWrite(PPM_OUT, HIGH);
    timer0_rearm(50);
    return;
  }

  //raising edge?
  if (!pulseFlag) {
    timer0_rearm(300);
    digitalWrite(PPM_OUT, LOW);
    pulseFlag = true;
    return;
  }

  //reached gap?
  if (pulseIndex >= 8) {
    timer0_rearm(30000);
    digitalWrite(PPM_OUT, HIGH);
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

  digitalWrite(PPM_OUT, HIGH);
  pulseFlag = false;
  pulseIndex++;
}

// timer1 crashes? probably hardware, but here is a software fallback
void sendPpmRxBitbang() {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(PPM_OUT, LOW);
    delayMicroseconds(300);
    digitalWrite(PPM_OUT, HIGH);
    delayMicroseconds(pulses[i] - 300);
  }
  digitalWrite(PPM_OUT, LOW);
  delayMicroseconds(300);
  digitalWrite(PPM_OUT, HIGH);
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
    txBuffer[5 + (i * 2)] = pulses[i] & 0xff;
    txBuffer[6 + (i * 2)] = pulses[i] % 0xff;
  }

  uint8_t crc = txBuffer[5];
  for (uint8_t i = 1; i < 32; i++) {
    crc = crc ^ txBuffer[5 + i];
  }
  txBuffer[5 + 32] = crc;

  Serial.write(txBuffer, 32 + 6);
}

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
  uint16_t pulse = 0;

  // store servo data
  for (i = 0; i < 176; i++) {
    pulse = pulses[ch];

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

  if (framesLost > 0) {
    txBuffer[23] |= (1 << 2); //frame lost
  }
  if (errorCount > errorLimit) {
    txBuffer[23] |= (1 << 3); //failsafe active
  }

  Serial.write(txBuffer, 25);
}

uint16_t CRC16(u16 crc, u8 value) {
  uint8_t i;
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

    if (sniffer->buf[15]!=2) {
        return;
    }

    //correct SSID?
    for (int i = 0; i < 6; i++) {
      if (sniffer->buf[i + 38] != packet[i + 38]) {
        return;
      }
    }

    uint8_t ser = sniffer->buf[91];
    if (ser>0) {
        for (uint8_t i=0;i<ser;i++) {
            Serial.write(sniffer->buf[92+i]);
        }
    }

    errorCount = 1;
    rssi = sniffer->rx_ctrl.rssi;//negative!

    seqNumber = sniffer->buf[22] | (sniffer->buf[23] << 8);
    if (lastSeqNumber + 1 != seqNumber) {
      framesLost++;
    }
    lastSeqNumber = seqNumber;
  }
}

void initTx() {
  pinMode(PPM_IN, INPUT);
  attachInterrupt(PPM_IN, ppmInterruptTx, RISING);

  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(WIFI_CHANNEL);
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
  if (txFlag == false) return;
  txFlag = false;

  packet[56] = WIFI_CHANNEL;
  packet[15] = 1;

  for (int j = 0; j < 16 ; j++) {
    packet[59 + (j * 2)] = (byte)(pulses[j] >> 8);
    packet[60 + (j * 2)] = (byte)(pulses[j]);
  }

  packet[22] = seqNumber % 255;
  packet[23] = seqNumber / 255;
  seqNumber++;

  uint8_t lim = Serial.available();
  if (lim > 0) {
    if (lim > 111) lim = 111;
    packet[91] = lim;
    for (uint8_t i = 0; i < lim; i++) {
      packet[i + 92] = Serial.read();
    }
  } else {
    packet[91] = 0;
  }

  wifi_set_channel(WIFI_CHANNEL);
  wifi_send_pkt_freedom(packet, sizeof(packet), 0);
}

void runTx() {

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
    for (uint8_t i = 0; i < 16; i++) {
      pulses[i] = 0;
    }
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void runLed() {
  signed long limit = errorCount < errorLimit ? 300 : 25;
  if ((signed long)(millis() - ledTimeout) > limit) {
    ledTimeout = millis() + limit;
    digitalWrite(LED_PIN, digitalRead(LED_PIN) == LOW ? HIGH : LOW);
  }
}

void setup() {
  delay(500);
  Serial.begin(115200);

  byte mac[6];
  WiFi.macAddress(mac);
  for (uint8_t i = 0; i < 6; i++) {
    packet[16 + i] = mac[i];
  }

  if (LED_PIN) {
    initLed();
  }
  if (IS_RX) {
    initRx();
  } else {
    initTx();
  }

  os_timer_disarm(&errTimer);
  os_timer_setfn(&errTimer, errTimerCb, NULL);
  os_timer_arm(&errTimer, 20, 1);
}

void loop() {
  if (LED_PIN) {
    runLed();
  }
  if (IS_RX) {
    runRx();
  } else {
    runTx();
  }

  ESP.wdtFeed();
  yield();
}
