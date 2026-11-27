/*
  ESP32-WROOM / QUAD-X — RECEIVER + WATCHDOG BENCH DIAGNOSTICS

  BENCH-ONLY, NOT A FLIGHT CONTROLLER. ARMING IS DISABLED.
  ALL FOUR PROPELLERS MUST BE REMOVED.

  Diagnostic stages (change ONE option, re-upload, and repeat >= 10 min):
  Stage A: WIFI_ON=1, IMU_STRESS=0, ESC_STRESS=0 (default).
  Stage B: WIFI_ON=1, IMU_STRESS=1, ESC_STRESS=0.
  Stage C: WIFI_ON=1, IMU_STRESS=1, ESC_STRESS=1.
  ESC_STRESS always commands 1000 us to all motors. NEVER attempts arming.

  RX: CH1=34, CH2=35, CH3=32, CH4=33, CH5=25, CH6=26
  AP: ESP32-DRONE | password drone1234 | http://192.168.4.1

  Purpose:
  - Remove shared cross-core RX spinlock and complicated ISR filtering.
  - Keep ISRs extremely short. Do not call Wi-Fi/Serial/String there.
  - Count valid pulse updates, invalid widths, orphan edges, and age.
  - Detect group dropouts, report reset reason and uptime.
  - Check whether Wi-Fi alone, IMU I2C load or ESC PWM contributes.

  Last-reset reason is recorded at BOOT. A watchdog code there only means
  the previous run ended with watchdog reset.
  No watchdog disabled, relaxed, or artificially fed.
*/

#define WIFI_ON      1
#define IMU_STRESS   0
#define ESC_STRESS   0

#include <Arduino.h>
#include <esp_system.h>
#include <soc/gpio_struct.h>
#include <WiFi.h>
#include <WebServer.h>
#if IMU_STRESS
#include <Wire.h>
#endif
#if ESC_STRESS
#include <ESP32Servo.h>
#endif

static constexpr int RX_COUNT = 6;
static const uint8_t RX_PIN[RX_COUNT] = {34,35,32,33,25,26};
static constexpr uint32_t RX_TIMEOUT_US = 100000UL;
static constexpr uint32_t RX_MIN_US = 900UL;
static constexpr uint32_t RX_MAX_US = 2100UL;
static constexpr uint32_t TICK_US = 4000UL;

// Writes to aligned 32-bit words are atomic on ESP32.
// All GPIO callbacks are installed from Arduino setup()/loop() core.
// No portMUX and no shared critical section inside any receiver ISR.
volatile uint32_t risingUs[RX_COUNT] = {};
volatile uint32_t pulseUs[RX_COUNT] = {};
volatile uint32_t rawUs[RX_COUNT] = {};
volatile uint32_t acceptedAtUs[RX_COUNT] = {};
volatile uint32_t edges[RX_COUNT] = {};
volatile uint32_t accepted[RX_COUNT] = {};
volatile uint32_t badWidths[RX_COUNT] = {};
volatile uint32_t orphans[RX_COUNT] = {};
volatile uint8_t sawRise[RX_COUNT] = {};

struct ChannelSnap {
  uint32_t pulse, raw, age, edgeCount, goodCount, badCount, orphanCount;
  bool valid;
};
ChannelSnap ch[RX_COUNT] = {};
uint8_t badMask = 0x3F;
bool receiverHealthy = false;
bool everHealthy = false;
uint32_t dropouts = 0;
uint32_t lastDropoutMs = 0;
uint8_t lastDropoutMask = 0;
uint32_t lastRxPollMs = 0;
uint32_t maxRxPollGapMs = 0;
uint32_t loopTicks = 0;
uint32_t maxTickGapUs = 0;
uint32_t priorTickUs = 0;
uint32_t bootReason = 0;

#if IMU_STRESS
static constexpr uint8_t IMU_ADDR = 0x68;
bool imuAvailable = false;
uint8_t whoAmI = 0;
uint32_t imuReads = 0;
uint32_t imuErrors = 0;
#endif

#if ESC_STRESS
Servo esc1, esc2, esc3, esc4;
#endif

#if WIFI_ON
WebServer server(80);
const char* WIFI_SSID = "ESP32-DRONE";
const char* WIFI_PASS = "drone1234";
#endif

// Direct GPIO register read; no extra HAL or lock from an interrupt.
static inline bool IRAM_ATTR readLevel(uint8_t pin) {
  if (pin < 32) return (GPIO.in & (1UL << pin)) != 0;
  return (GPIO.in1.val & (1UL << (pin-32))) != 0;
}

void IRAM_ATTR onReceiverEdge(uint8_t channel) {
  const uint32_t t = micros();
  const uint8_t pin = RX_PIN[channel];
  const bool isHigh = readLevel(pin);
  edges[channel]++;

  if (isHigh) {
    risingUs[channel] = t;  // resynchronize whenever a rise occurs
    sawRise[channel] = 1;
    return;
  }
  if (!sawRise[channel]) {
    orphans[channel]++;
    return;
  }
  sawRise[channel] = 0;
  const uint32_t w = (uint32_t)(t-risingUs[channel]);
  rawUs[channel] = w;
  if (w >= RX_MIN_US && w <= RX_MAX_US) {
    pulseUs[channel] = w;
    acceptedAtUs[channel] = t; // MUST update timestamp after pulse width
    accepted[channel]++;
  } else {
    badWidths[channel]++;
  }
}
void IRAM_ATTR rx1(){onReceiverEdge(0);}
void IRAM_ATTR rx2(){onReceiverEdge(1);}
void IRAM_ATTR rx3(){onReceiverEdge(2);}
void IRAM_ATTR rx4(){onReceiverEdge(3);}
void IRAM_ATTR rx5(){onReceiverEdge(4);}
void IRAM_ATTR rx6(){onReceiverEdge(5);}

void initializeReceiver() {
  void (*handlers[RX_COUNT])() = {rx1,rx2,rx3,rx4,rx5,rx6};
  for (int i=0;i<RX_COUNT;i++) {
    pinMode(RX_PIN[i], INPUT); // 34/35 have no internal pulls
    attachInterrupt(digitalPinToInterrupt(RX_PIN[i]), handlers[i], CHANGE);
  }
}

void pollReceiver() {
  uint32_t p[RX_COUNT], raw[RX_COUNT], ts[RX_COUNT];
  uint32_t nEdge[RX_COUNT], nGood[RX_COUNT], nBad[RX_COUNT], nOrphan[RX_COUNT];
  // The callback and this snapshot run on the Arduino loop core. Only a
  // short interrupt mask (< tens of us) is used; there is NO spinlock.
  noInterrupts();
  for (int i=0;i<RX_COUNT;i++) {
    p[i]=pulseUs[i]; raw[i]=rawUs[i]; ts[i]=acceptedAtUs[i];
    nEdge[i]=edges[i]; nGood[i]=accepted[i];
    nBad[i]=badWidths[i]; nOrphan[i]=orphans[i];
  }
  interrupts();
  // Capture now AFTER the snapshot to avoid unsigned underflow from a new pulse.
  uint32_t nowUs=micros();
  uint8_t currentBad=0;
  for(int i=0;i<RX_COUNT;i++) {
    ch[i].pulse=p[i]; ch[i].raw=raw[i];
    ch[i].edgeCount=nEdge[i]; ch[i].goodCount=nGood[i];
    ch[i].badCount=nBad[i]; ch[i].orphanCount=nOrphan[i];
    ch[i].age = nGood[i] ? (uint32_t)(nowUs-ts[i]) : UINT32_MAX;
    ch[i].valid = nGood[i] && p[i]>=RX_MIN_US && p[i]<=RX_MAX_US && ch[i].age<=RX_TIMEOUT_US;
    if(!ch[i].valid) currentBad |= (1U<<i);
  }
  const bool nowHealthy=(currentBad==0);
  if(nowHealthy) everHealthy=true;
  if(everHealthy && receiverHealthy && !nowHealthy) {
    dropouts++;
    lastDropoutMs=millis();
    lastDropoutMask=currentBad;
    Serial.printf("RX DROP %lu at uptime=%lums mask=0x%02X\n",
      (unsigned long)dropouts, (unsigned long)lastDropoutMs, (unsigned int)currentBad);
  }
  badMask=currentBad;
  receiverHealthy=nowHealthy;
  uint32_t currentMs=millis();
  if(lastRxPollMs) {
    uint32_t gap=(uint32_t)(currentMs-lastRxPollMs);
    if(gap>maxRxPollGapMs) maxRxPollGapMs=gap;
  }
  lastRxPollMs=currentMs;
}

#if IMU_STRESS
bool checkImu() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)1) != 1) return false;
  int b=Wire.read();
  if(b < 0) return false;
  whoAmI=(uint8_t)b;
  return whoAmI==0x68 || whoAmI==0x70;
}
void imuStressRead() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(IMU_ADDR, (uint8_t)14)!=14) {
    imuErrors++; return;
  }
  for(int i=0;i<14;i++) if(Wire.read()<0){imuErrors++;return;}
  imuReads++;
}
#endif

#if ESC_STRESS
void initESCs() {
  ESP32PWM::allocateTimer(0);ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);ESP32PWM::allocateTimer(3);
  esc1.setPeriodHertz(500);esc2.setPeriodHertz(500);
  esc3.setPeriodHertz(500);esc4.setPeriodHertz(500);
  esc1.attach(13,1000,1800);esc2.attach(12,1000,1800);
  esc3.attach(14,1000,1800);esc4.attach(27,1000,1800);
  esc1.writeMicroseconds(1000);esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);esc4.writeMicroseconds(1000);
  // No other ESC writes in this sketch. Never arms.
}
#endif

const char* resetName(uint32_t r) {
  switch(r) {
    case 1:return "POWER_ON"; case 2:return "EXTERNAL";
    case 3:return "SOFTWARE"; case 4:return "PANIC";
    case 5:return "INT_WDT"; case 6:return "TASK_WDT";
    case 7:return "WDT"; case 8:return "DEEP_SLEEP";
    case 9:return "BROWNOUT"; default:return "OTHER";
  }
}

#if WIFI_ON
const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="en"><head><meta name="viewport" content="width=device-width,initial-scale=1"><meta charset="utf-8"><title>ESP32 RX/WDT Bench</title><style>
:root{color-scheme:dark}body{font:15px system-ui;background:#081321;color:#e4efff;margin:0;padding:18px}.wrap{max-width:1000px;margin:auto}h1{font-size:23px;margin-bottom:6px}.muted{color:#9cb4d4}.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-top:16px}.card{background:#15243a;border:1px solid #2d415b;border-radius:12px;padding:16px}.card h2{color:#abc8ec;font-size:12px;letter-spacing:.08em;margin:0 0 9px}.big{font-size:24px;font-weight:700}.ok{color:#2dd9a2}.bad{color:#ff647f}.warn{color:#ffca74}table{width:100%;border-collapse:collapse;margin-top:10px;font-variant-numeric:tabular-nums;font-size:13px}th,td{padding:10px 7px;text-align:right;border-bottom:1px solid #34455e}th:first-child,td:first-child{text-align:left}.wide{grid-column:1/-1}.note{line-height:1.55;font-size:13px}@media(max-width:700px){.grid{grid-template-columns:repeat(2,1fr)}.big{font-size:20px}.tablewrap{overflow:auto}table{min-width:620px}} </style></head><body><div class="wrap"><h1>ESP32 Receiver + Watchdog Bench</h1><div class="muted">Read-only diagnostic · Motor arming permanently disabled · Refresh 4 Hz</div><div class="grid">
<div class="card"><h2>RECEIVER</h2><div id="health" class="big">--</div></div><div class="card"><h2>DROPOUTS</h2><div id="dropouts" class="big">--</div></div><div class="card"><h2>UPTIME</h2><div id="uptime" class="big">--</div></div><div class="card"><h2>PREVIOUS RESET</h2><div id="reset" class="big">--</div></div>
<div class="card wide"><h2>CHANNEL SIGNALS (100 MS FRESHNESS LIMIT)</h2><div class="tablewrap"><table><thead><tr><th>Channel</th><th>Pulse µs</th><th>Age ms</th><th>Last raw µs</th><th>Valid pulses</th><th>Edges</th><th>Bad width</th><th>Orphans</th></tr></thead><tbody id="rxbody"></tbody></table></div><p id="badmask" class="note muted"></p></div>
<div class="card wide"><h2>BENCH DIAGNOSTICS</h2><div class="note">Mode: <span id="mode">--</span> · Last dropout: <span id="lastdrop">--</span> · Max 250 Hz work gap: <span id="tickgap">--</span> µs · Max receiver poll gap: <span id="pollgap">--</span> ms · IMU reads/errors: <span id="imu">--</span> · Connection: <span id="conn">Connecting</span></div><p class="note muted">An INT_WDT reset reason refers to the last boot, not the current receiver dropout. If uptime resets, the ESP32 restarted. All propellers must remain removed.</p></div></div></div><script>
const $=id=>document.getElementById(id);let prevUptime=-1;
const names=['Roll','Pitch','Throttle','Yaw','Arm','Aux'];for(let i=0;i<6;i++)$('rxbody').insertAdjacentHTML('beforeend',`<tr><td>CH${i+1} ${names[i]}</td><td id="p${i}"></td><td id="a${i}"></td><td id="r${i}"></td><td id="g${i}"></td><td id="e${i}"></td><td id="b${i}"></td><td id="o${i}"></td></tr>`);
async function update(){try{let response=await fetch('/data',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);let d=await response.json();$('conn').textContent='LIVE';$('conn').className='ok';$('health').textContent=d.receiverOK?'HEALTHY':'INVALID';$('health').className='big '+(d.receiverOK?'ok':'bad');$('dropouts').textContent=d.dropouts;$('uptime').textContent=(d.uptimeMs/1000).toFixed(1)+' s';$('reset').textContent=d.reset; $('reset').className='big '+(d.reset==='INT_WDT'?'warn':'');$('badmask').textContent='Invalid channels bitmask: 0x'+Number(d.badMask).toString(16).toUpperCase();$('lastdrop').textContent=d.lastDropMs?d.lastDropMs+' ms (mask 0x'+d.lastDropMask.toString(16)+')':'None';$('tickgap').textContent=d.maxTickGapUs;$('pollgap').textContent=d.maxRxPollGapMs;$('mode').textContent=d.stage;$('imu').textContent=d.imuReads+'/'+d.imuErrors;
for(let i=0;i<6;i++){let c=d.channels[i];$('p'+i).textContent=c.pulse;$('p'+i).className=c.ok?'ok':'bad';$('a'+i).textContent=c.age===4294967295?'NEVER':(c.age/1000).toFixed(0);$('r'+i).textContent=c.raw;$('g'+i).textContent=c.good;$('e'+i).textContent=c.edges;$('b'+i).textContent=c.bad;$('o'+i).textContent=c.orphan;}
if(prevUptime>=0&&d.uptimeMs<prevUptime){$('conn').textContent='REBOOT DETECTED';$('conn').className='bad';}prevUptime=d.uptimeMs;
}catch(error){$('conn').textContent='OFFLINE';$('conn').className='bad';}}
setInterval(update,250);update();
</script></body></html>
)HTML";

void serveData() {
  // Runs in same Arduino loop task. No cross-core telemetry mutex.
  String s;
  s.reserve(1700);
  s="{\"uptimeMs\":"+String(millis())+",\"reset\":\""+resetName(bootReason)+"\"";
  s+=",\"receiverOK\":"; s+=(receiverHealthy?"true":"false");
  s+=",\"badMask\":"+String(badMask)+",\"dropouts\":"+String(dropouts);
  s+=",\"lastDropMs\":"+String(lastDropoutMs)+",\"lastDropMask\":"+String(lastDropoutMask);
  s+=",\"maxTickGapUs\":"+String(maxTickGapUs)+",\"maxRxPollGapMs\":"+String(maxRxPollGapMs);
  s+=",\"stage\":\"WiFi ";s+=WIFI_ON?"ON":"OFF";
  s+=", IMU ";s+=IMU_STRESS?"ON":"OFF";
  s+=", ESC ";s+=ESC_STRESS?"1000us":"OFF";s+="\"";
#if IMU_STRESS
  s+=",\"imuReads\":"+String(imuReads)+",\"imuErrors\":"+String(imuErrors);
#else
  s+=",\"imuReads\":0,\"imuErrors\":0";
#endif
  s+=",\"channels\":[";
  for(int i=0;i<RX_COUNT;i++){
    if(i)s+=',';
    s+="{\"pulse\":"+String(ch[i].pulse)+",\"raw\":"+String(ch[i].raw);
    s+=",\"age\":"+String(ch[i].age)+",\"good\":"+String(ch[i].goodCount);
    s+=",\"edges\":"+String(ch[i].edgeCount)+",\"bad\":"+String(ch[i].badCount);
    s+=",\"orphan\":"+String(ch[i].orphanCount)+",\"ok\":";
    s+=(ch[i].valid?"true":"false");s+='}';
  }
  s+="]}";
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",s);
}
void startWeb(){
  WiFi.mode(WIFI_AP);
  if(!WiFi.softAP(WIFI_SSID,WIFI_PASS)){
    Serial.println("WiFi AP failed");return;
  }
  server.on("/",HTTP_GET,[](){server.send_P(200,"text/html",PAGE);});
  server.on("/data",HTTP_GET,serveData);
  server.begin();
  Serial.print("Dashboard: http://");Serial.println(WiFi.softAPIP());
}
#endif

void setup() {
  Serial.begin(115200);
  delay(200);
  bootReason=(uint32_t)esp_reset_reason();
  Serial.printf("\n==== RECEIVER + WDT BENCH ONLY ====\nLast reset: %s (%lu)\n",
    resetName(bootReason),(unsigned long)bootReason);
  Serial.printf("WIFI_ON=%d IMU_STRESS=%d ESC_STRESS=%d\n",WIFI_ON,IMU_STRESS,ESC_STRESS);
  Serial.println("Motor arming is DISABLED. All propellers must be removed.");
  initializeReceiver();
#if IMU_STRESS
  if(Wire.begin(21,22,400000)){
    Wire.setTimeOut(5);
    imuAvailable=checkImu();
  }
  Serial.printf("IMU available=%d WHO_AM_I=0x%02X\n",imuAvailable,whoAmI);
#endif
#if ESC_STRESS
  initESCs();
  Serial.println("Four ESC signals held permanently at 1000us");
#endif
#if WIFI_ON
  startWeb();
#endif
  priorTickUs=micros();
}

void loop() {
  uint32_t now=micros();
  if((uint32_t)(now-priorTickUs)>=TICK_US) {
    const uint32_t gap=(uint32_t)(now-priorTickUs);
    priorTickUs=now;
    if(gap>maxTickGapUs) maxTickGapUs=gap;
    loopTicks++;
    pollReceiver();
#if IMU_STRESS
    if(imuAvailable) imuStressRead();
#endif
  }
#if WIFI_ON
  server.handleClient();
#endif
  static uint32_t lastPrint=0;
  if((uint32_t)(millis()-lastPrint)>=1000) {
    lastPrint=millis();
    Serial.printf("up=%lus RX=%s drops=%lu badmask=0x%02X maxTick=%luus",
      (unsigned long)(millis()/1000),receiverHealthy?"OK":"BAD",
      (unsigned long)dropouts,(unsigned int)badMask,(unsigned long)maxTickGapUs);
    for(int i=0;i<RX_COUNT;i++)
      Serial.printf(" | %d:%lu/%lums g%lu b%lu o%lu",i+1,
        (unsigned long)ch[i].pulse,
        (unsigned long)(ch[i].age==UINT32_MAX?9999:(ch[i].age/1000)),
        (unsigned long)ch[i].goodCount,
        (unsigned long)ch[i].badCount,
        (unsigned long)ch[i].orphanCount);
#if IMU_STRESS
    Serial.printf(" imu=%lu/%lu",(unsigned long)imuReads,(unsigned long)imuErrors);
#endif
    Serial.println();
  }
  delay(1); // always yield to WiFi/FreeRTOS; no spin wait
}