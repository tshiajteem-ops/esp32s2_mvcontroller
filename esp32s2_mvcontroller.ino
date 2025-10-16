/*
  Esp32 S2 MVSilicon Controller - Optimized
  - WiFi AP: "TSHIAJTEEM Studio" / "emquenroi"
  - Fixed SoftAP IP: 192.168.1.198
  - Delay control pin: GPIO35
  - Supports models: K10, K10Plus, K18, XY-Sound, YX5300 (selectable in UI)
  - Packet format (default MVSilicon generic): [0xAA,0x55][CMD][LEN_L][LEN_H][PAYLOAD...][CHK]
    where CHK = (sum of CMD + payload bytes) & 0xFF

  Notes:
  - This sketch provides ready-to-use packet builders (stubs) for each model.
    You should verify bytes with your actual board logs; if you provide real packet dumps
    I will adapt them exactly.
  - Libraries: AsyncTCP, ESPAsyncWebServer, ArduinoJson, LittleFS_esp32
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// --- CONFIG ---
const char* AP_SSID = "TSHIAJTEEM Studio";
const char* AP_PWD  = "emquenroi";
const IPAddress AP_IP(192,168,1,198);
const IPAddress AP_GATEWAY(192,168,1,1);
const IPAddress AP_SUBNET(255,255,255,0);

// UART to MVSilicon board
const int MV_RX_PIN = 37; // ESP32-S2 RX (to MVSilicon TX)
const int MV_TX_PIN = 38; // ESP32-S2 TX (to MVSilicon RX)
const unsigned long MV_BAUD = 115200;

// Delay control pin (changed to GPIO35 per request)
const int DELAY_PIN = 35;

AsyncWebServer server(80);
Preferences prefs;

// --- Params (kept compact) ---
struct Params {
  bool delay_enabled;
  int delay_time; // ms
  float delay_speed;
  float delay_rhythm;
  float echo_volume, echo_delay, echo_repeat; int echo_cutoff;
  float echo_eq[12];
  float reverb_volume, reverb_highcut, reverb_predelay, reverb_diffusion, reverb_decay, reverb_damping, reverb_wetdrymix;
  float reverb_eq[15];
  float music_bass, music_mid, music_treble;
  float mic_volume, mic_anti_feedback;
  float audio_eq[15];
  float mic_eq[15];
  String device_name;
  String chip_code; // model selection (K10, K10Plus, K18, XYS, YX5300)
  String bt_name;
} params;

// --- Helpers ---
uint8_t calc_chk(const uint8_t *buf, size_t len){
  uint32_t s = 0; for(size_t i=0;i<len;i++) s += buf[i];
  return (uint8_t)(s & 0xFF);
}

void send_raw_packet(const uint8_t *payload, size_t payloadLen){
  // payload already includes header/cmd/len/.../chk
  Serial1.write(payload, payloadLen);
  Serial1.flush();
}

// Build packet: header(AA55) + cmd + lenL + lenH + payload + chk
void build_and_send(uint8_t cmd, const uint8_t *data, uint16_t dataLen){
  size_t total = 2 + 1 + 2 + dataLen + 1;
  uint8_t *buf = (uint8_t*)malloc(total);
  if(!buf) return;
  size_t p = 0;
  buf[p++] = 0xAA; buf[p++] = 0x55;
  buf[p++] = cmd;
  buf[p++] = (uint8_t)(dataLen & 0xFF);
  buf[p++] = (uint8_t)((dataLen>>8) & 0xFF);
  if(dataLen) memcpy(buf + p, data, dataLen);
  p += dataLen;
  // checksum: sum of cmd + payload
  uint32_t s = cmd;
  for(uint16_t i=0;i<dataLen;i++) s += data[i];
  buf[p++] = (uint8_t)(s & 0xFF);
  send_raw_packet(buf, total);
  free(buf);
}

// Convert float gain (-12..12) to int16 centered representation (gain*100)
int16_t f2s100(float v){ return (int16_t)round(v * 100.0f); }

// --- Model-specific packet builders (stubs) ---
// Each sends a compact binary packet representing main parameters.

void sendPacket_K10_all(){
  // Example: CMD 0x10 = set all params
  // Payload layout (example): [delay_on(1)][delay_ms(2)][echo_vol(1)][echo_delay_ms(2)]...[eq counts...]
  uint8_t payload[64]; memset(payload,0,sizeof(payload)); size_t p=0;
  payload[p++] = params.delay_enabled ? 1 : 0;
  payload[p++] = (uint8_t)(params.delay_time & 0xFF);
  payload[p++] = (uint8_t)((params.delay_time>>8)&0xFF);
  payload[p++] = (uint8_t)(uint8_t)(params.echo_volume*255.0f);
  uint16_t ed = (uint16_t)params.echo_delay; payload[p++]=ed & 0xFF; payload[p++]=(ed>>8)&0xFF;
  payload[p++] = (uint8_t)min(max((int)params.echo_repeat,0),255);
  payload[p++] = (uint8_t)(params.echo_cutoff>>8); // coarse
  // pack first 8 audio EQ bands as int16
  for(int i=0;i<8;i++){ int16_t g = f2s100(params.audio_eq[i]); payload[p++] = g & 0xFF; payload[p++] = (g>>8)&0xFF; }
  build_and_send(0x10, payload, p);
}

void sendPacket_K10Plus_all(){
  // K10Plus uses CMD 0x11
  uint8_t payload[80]; size_t p=0; memset(payload,0,sizeof(payload));
  payload[p++] = params.delay_enabled?1:0;
  payload[p++] = (uint8_t)(params.delay_time & 0xFF); payload[p++]=(uint8_t)((params.delay_time>>8)&0xFF);
  // include mic volume
  int16_t mg = f2s100(params.mic_volume);
  payload[p++] = mg & 0xFF; payload[p++] = (mg>>8)&0xFF;
  // include full 15-band audio EQ (packed int16)
  for(int i=0;i<15;i++){ int16_t gg = f2s100(params.audio_eq[i]); payload[p++] = gg & 0xFF; payload[p++] = (gg>>8)&0xFF; }
  build_and_send(0x11, payload, p);
}

void sendPacket_K18_all(){
  // K18: CMD 0x12 with simpler payload
  uint8_t payload[48]; size_t p=0; memset(payload,0,sizeof(payload));
  payload[p++] = params.delay_enabled?1:0;
  payload++[0]= (uint8_t)(params.delay_time & 0xFF); payload++[0]=(uint8_t)((params.delay_time>>8)&0xFF);
  payload[p++] = (uint8_t)(params.reverb_volume*255);
  // pack 12-band echo EQ as int8 (-12..12 mapped to -120..120 -> clamp to -128..127)
  for(int i=0;i<12;i++){ int v = (int)round(params.echo_eq[i]*10.0f); if(v<-128) v=-128; if(v>127) v=127; payload[p++]=(uint8_t)(v & 0xFF); }
  build_and_send(0x12, payload, p);
}

void sendPacket_XYS_all(){
  // XY-Sound custom: CMD 0x20
  uint8_t payload[64]; size_t p=0; memset(payload,0,sizeof(payload));
  payload[p++]=params.delay_enabled?1:0; payload[p++]=(uint8_t)params.delay_time; payload[p++]=(uint8_t)(params.delay_time>>8);
  // minimal example: send three music bands
  payload[p++]= (int8_t)round(params.music_bass*10);
  payload[p++]= (int8_t)round(params.music_mid*10);
  payload[p++]= (int8_t)round(params.music_treble*10);
  build_and_send(0x20, payload, p);
}

void sendPacket_YX5300_all(){
  // YX5300 often has its own protocol; we send a generic SET JSON command as fallback on CMD 0x30
  DynamicJsonDocument d(512);
  d["cmd"] = "set_all";
  d["delay_enabled"] = params.delay_enabled;
  d["delay_time"] = params.delay_time;
  d["mic_volume"] = params.mic_volume;
  String s; serializeJson(d, s);
  build_and_send(0x30, (const uint8_t*)s.c_str(), s.length());
}

// Dispatcher
void applyAllParamsToMV(){
  String m = params.chip_code;
  if(m.equalsIgnoreCase("K10")) sendPacket_K10_all();
  else if(m.equalsIgnoreCase("K10Plus") || m.equalsIgnoreCase("K10 Plus")) sendPacket_K10Plus_all();
  else if(m.equalsIgnoreCase("K18") || m.equalsIgnoreCase("K8")) sendPacket_K18_all();
  else if(m.equalsIgnoreCase("XY-Sound") || m.equalsIgnoreCase("XYS") || m.equalsIgnoreCase("XY")) sendPacket_XYS_all();
  else if(m.equalsIgnoreCase("YX5300")) sendPacket_YX5300_all();
  else {
    // generic fallback: small JSON
    DynamicJsonDocument d(512); d["cmd"]="apply_all"; d["delay_enabled"]=params.delay_enabled; d["delay_time"]=params.delay_time;
    String s; serializeJson(d,s); build_and_send(0x3F, (const uint8_t*)s.c_str(), s.length());
  }
}

// --- Web UI (lightweight) ---
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>TSHIAJTEEM Studio</title>
<style>body{font-family:Arial;margin:0}header{background:#FFD700;padding:12px;text-align:center;font-weight:bold}main{padding:10px}label{display:block;margin:6px 0}input[type=range]{width:100%}select,input[type=text]{width:100%;padding:6px}section{border:1px solid #ddd;padding:8px;margin:8px 0;border-radius:6px}button{padding:8px;margin:6px 0}</style>
</head><body><header>&gt;&gt;&gt;&gt;&gt;TSHIAJ TEEM Studio&lt;&lt;&lt;&lt;&lt;</header><main>
<section><label>Model<br><select id="chip_select"><option>K10</option><option>K10Plus</option><option>K18</option><option>XY-Sound</option><option>YX5300</option></select></label>
<label>Bluetooth Name<input id="bt_name" type="text"></label></section>

<section><h4>Delay</h4><label>Enable <input id="delay_enable" type="checkbox"></label>
<label>Time <input id="delay_time" type="range" min="1" max="2000" value="120"></label></section>

<section><h4>Echo</h4><label>Volume <input id="echo_volume" type="range" min="0" max="1" step="0.01" value="0.5"></label>
<label>Delay <input id="echo_delay" type="range" min="1" max="2000" value="120"></label></section>

<section><h4>Music</h4><label>Bass <input id="music_bass" type="range" min="-12" max="12" step="0.1" value="0"></label>
<label>Mid <input id="music_mid" type="range" min="-12" max="12" step="0.1" value="0"></label>
<label>Treble <input id="music_treble" type="range" min="-12" max="12" step="0.1" value="0"></label></section>

<section><button onclick="applyAll()">Gửi tất cả</button><button onclick="toggleDelay()">Bật/Tắt Delay (pin35)</button></section>

<section><h4>Utilities</h4>
<form id="fw" method="POST" enctype="multipart/form-data" action="/update"><input type="file" name="update_file"><button type="submit">Nâng cấp Firmware</button></form>
</section>

<section><h4>Giới thiệu</h4><p>Chào mừng đến với TSHIAJTEEM STUDIO<br>Liên hệ: 0869.898.931 / 0869.898.737</p></section>

</main>
<script>
function getEl(id){return document.getElementById(id)}
function collect(){return{
  chip:getEl('chip_select').value,
  bt_name:getEl('bt_name').value,
  delay_enabled:getEl('delay_enable').checked,
  delay_time:parseInt(getEl('delay_time').value),
  echo_volume:parseFloat(getEl('echo_volume').value),
  echo_delay:parseInt(getEl('echo_delay').value),
  music_bass:parseFloat(getEl('music_bass').value),music_mid:parseFloat(getEl('music_mid').value),music_treble:parseFloat(getEl('music_treble').value)
}}
function applyAll(){fetch('/api/apply_all',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(collect())}).then(r=>r.json()).then(a=>alert(JSON.stringify(a))).catch(e=>alert(e))}
function toggleDelay(){fetch('/api/toggle_delay',{method:'POST'}).then(r=>r.json()).then(a=>alert(JSON.stringify(a))).catch(e=>alert(e))}
</script>
</body></html>
)rawliteral";

// --- HTTP handlers ---
void handleGetParams(AsyncWebServerRequest *req){
  DynamicJsonDocument d(512);
  d["chip"] = params.chip_code;
  d["bt_name"] = params.bt_name;
  d["delay_enabled"] = params.delay_enabled;
  d["delay_time"] = params.delay_time;
  String out; serializeJson(d,out);
  req->send(200, "application/json", out);
}

void handleApplyAll(AsyncWebServerRequest *req){
  if(req->method()!=HTTP_POST){ req->send(405); return; }
  if(!req->hasParam("body", true)){ req->send(400); return; }
  String b = req->getParam("body", true)->value();
  DynamicJsonDocument d(2048); if(deserializeJson(d,b)){ req->send(400); return; }
  params.chip_code = String((const char*)d["chip"].as<const char*>() ?: "K10");
  params.bt_name = String((const char*)d["bt_name"].as<const char*>() ?: "MV_BT");
  params.delay_enabled = d["delay_enabled"] | params.delay_enabled;
  params.delay_time = d["delay_time"] | params.delay_time;
  params.echo_volume = d["echo_volume"] | params.echo_volume;
  params.echo_delay = d["echo_delay"] | params.echo_delay;
  params.music_bass = d["music_bass"] | params.music_bass; params.music_mid = d["music_mid"] | params.music_mid; params.music_treble = d["music_treble"] | params.music_treble;
  // apply
  applyAllParamsToMV();
  req->send(200, "application/json", "{\"ok\":true}");
}

void handleToggleDelay(AsyncWebServerRequest *req){
  digitalWrite(DELAY_PIN, !digitalRead(DELAY_PIN));
  DynamicJsonDocument d(128); d["pin"] = digitalRead(DELAY_PIN);
  String out; serializeJson(d,out); req->send(200, "application/json", out);
}

// Firmware upload endpoints handled by server.on /update

void setup(){
  Serial.begin(115200);
  delay(50);
  LittleFS.begin();
  prefs.begin("tshiaj", false);

  // Serial1 for MVSilicon
  Serial1.begin(MV_BAUD, SERIAL_8N1, MV_RX_PIN, MV_TX_PIN);

  pinMode(DELAY_PIN, OUTPUT); digitalWrite(DELAY_PIN, LOW);

  // init defaults
  params.delay_enabled = false; params.delay_time = 120; params.echo_volume = 0.5; params.echo_delay = 120;
  params.music_bass = params.music_mid = params.music_treble = 0.0;
  params.chip_code = "K10"; params.bt_name = "MV_BT";

  // WiFi AP with fixed IP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PWD);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  Serial.printf("SoftAP started. IP: %s\n", AP_IP.toString().c_str());

  // Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });
  server.on("/api/get_params", HTTP_GET, handleGetParams);
  server.on("/api/apply_all", HTTP_POST, handleApplyAll);
  server.on("/api/toggle_delay", HTTP_POST, handleToggleDelay);

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    bool shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
    if (shouldReboot) { delay(1000); ESP.restart(); }
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if (!index) { Serial.printf("Update Start: %s\n", filename.c_str()); if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial); }
    if (Update.write(data, len) != len) Update.printError(Serial);
    if (final) { if (Update.end(true)) Serial.printf("Update Success: %u bytes\n", index+len); else Update.printError(Serial); }
  });

  server.begin();
}

void loop(){
  // forward debug data from MVSilicon to USB serial
  while(Serial1.available()){ Serial.write(Serial1.read()); }
  delay(10);
}
