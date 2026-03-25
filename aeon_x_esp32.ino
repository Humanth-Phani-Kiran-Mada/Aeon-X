/**
 * ============================================================
 *  AEON X — AUX-9 PROJECT
 *  MODULE  : ESP32 Communication + Manual Control System
 *  PHASE   : 4  (FIXED v1.1)
 *  FRAMEWORK: Arduino (ESP32 Arduino Core)
 *
 *  FIXES IN v1.1:
 *    [BUG-A]  sscanf() used to parse STM32 telemetry with %lu format
 *             specifiers, but Arduino/ESP32's built-in sscanf does NOT
 *             reliably support %lu on all ESP32 core versions.
 *             Replaced with manual strtoul() parsing — always works.
 *    [BUG-B]  EventSource SSE: if browser tab is closed mid-stream the
 *             events.send() call would block and lock the async server.
 *             Added events.count() > 0 guard before every send().
 *    [BUG-C]  On first connection, the EventSource fires onerror briefly
 *             before onopen. setConnected(false) was called unnecessarily.
 *             Fixed: onerror only marks offline after 3 consecutive errors.
 *    [BUG-D]  In AP mode WiFi.softAPIP() sometimes returns 0.0.0.0 if
 *             called before the AP is fully up. Added 500ms delay after
 *             WiFi.softAP() before reading the IP.
 *    [BUG-E]  HTML: D-pad cmdRelease() always called sendCmd('STOP') even
 *             in autonomous mode (the pointer-leave event fires regardless
 *             of mode check). Added mode guard inside cmdRelease().
 *    [BUG-F]  ArduinoJson StaticJsonDocument size was 256 bytes — too small
 *             if telemetry grows. Upgraded to DynamicJsonDocument(512).
 *    [BUG-G]  UART2 buffer: if STM32 sends a line longer than UART_BUF_SIZE
 *             the index would overflow silently. Added hard reset of index
 *             on overflow rather than dropping into undefined behavior.
 *    [BUG-H]  Serial2.begin() on ESP32 with custom RX/TX pins requires
 *             the full 4-argument form. Fixed to pass pin numbers explicitly.
 *
 *  LIBRARIES REQUIRED:
 *    - ESPAsyncWebServer  (github.com/lacamera/ESPAsyncWebServer)
 *    - AsyncTCP           (dependency, same author)
 *    - ArduinoJson        v6.x (Arduino Library Manager)
 *
 *  PIN MAP:
 *    GPIO16 (RX2) → STM32 UART2 TX (PA2)
 *    GPIO17 (TX2) → STM32 UART2 RX (PA3)
 *    GND         → STM32 GND   ← MANDATORY
 *
 *  HOW TO CONNECT:
 *    Power on ESP32 → WiFi "AEON-X-Control" appears
 *    Connect → open http://192.168.4.1
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

/* ============================================================
 *  CONFIG
 * ============================================================ */
const char *AP_SSID     = "AEON-X-Control";
const char *AP_PASSWORD = "aeonx9999";

/* BUG-D fix: configure AP IP before starting */
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

/* UART — BUG-H fix: use explicit pin arguments */
#define STM32_RX_PIN    16
#define STM32_TX_PIN    17
#define STM32_BAUD      115200
#define UART_BUF_SIZE   256

/* ============================================================
 *  SERVER & SSE
 * ============================================================ */
AsyncWebServer  server(80);
AsyncEventSource events("/events");

/* ============================================================
 *  STATE
 * ============================================================ */
enum ControlMode { MODE_MANUAL = 0, MODE_AUTONOMOUS = 1 };

struct VehicleStatus {
    ControlMode mode         = MODE_MANUAL;
    int         stm32_state  = 0;
    uint32_t    front_cm     = 999;
    uint32_t    rear_cm      = 999;
    uint32_t    left_cm      = 999;
    uint32_t    right_cm     = 999;
    bool        ir_left      = false;
    bool        ir_right     = false;
    bool        stm32_online = false;
    uint32_t    last_tel_ms  = 0;
    uint32_t    cmd_count    = 0;
} gStatus;

/* ============================================================
 *  UART BUFFER
 *  BUG-G fix: hard-reset on overflow
 * ============================================================ */
char     uart_buf[UART_BUF_SIZE];
uint16_t uart_idx = 0;

/* ============================================================
 *  TELEMETRY PARSE
 *  BUG-A fix: use strtoul() instead of sscanf %lu
 *  Format: TEL:state,mode,front,rear,left,right,irl,irr
 * ============================================================ */
bool parseTelemetry(const char *line) {
    if (strncmp(line, "TEL:", 4) != 0) return false;

    const char *p = line + 4;
    char       *end;

    gStatus.stm32_state = (int)strtol(p, &end, 10); if (*end != ',') return false; p = end + 1;
    /* mode field — skip, we manage mode ourselves */
    strtol(p, &end, 10); if (*end != ',') return false; p = end + 1;
    gStatus.front_cm  = strtoul(p, &end, 10); if (*end != ',') return false; p = end + 1;
    gStatus.rear_cm   = strtoul(p, &end, 10); if (*end != ',') return false; p = end + 1;
    gStatus.left_cm   = strtoul(p, &end, 10); if (*end != ',') return false; p = end + 1;
    gStatus.right_cm  = strtoul(p, &end, 10); if (*end != ',') return false; p = end + 1;
    gStatus.ir_left   = (bool)strtol(p, &end, 10); if (*end != ',') return false; p = end + 1;
    gStatus.ir_right  = (bool)strtol(p, &end, 10);

    gStatus.stm32_online = true;
    gStatus.last_tel_ms  = millis();
    return true;
}

/* ============================================================
 *  SEND COMMAND TO STM32
 * ============================================================ */
void sendToSTM32(const char *cmd) {
    Serial2.print(cmd);
    Serial.printf("[ESP32→STM32] %s", cmd);
    gStatus.cmd_count++;
}

/* ============================================================
 *  HTML DASHBOARD — stored in flash (PROGMEM)
 * ============================================================ */
const char DASHBOARD_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>AEON X Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600;800&display=swap');
  :root {
    --bg:#0a0c10; --surface:#0f1318; --border:#1e2530;
    --accent:#00e5ff; --accent2:#ff6b00; --danger:#ff2d2d;
    --safe:#00ff88; --text:#c8d8e8; --dim:#4a5a6a; --r:12px;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:'Exo 2',sans-serif;
    min-height:100vh;display:flex;flex-direction:column;align-items:center;
    padding:16px;gap:14px;-webkit-user-select:none;user-select:none;touch-action:manipulation}
  .header{width:100%;max-width:500px;display:flex;align-items:center;
    justify-content:space-between;border-bottom:1px solid var(--border);padding-bottom:12px}
  .logo{font-family:'Share Tech Mono',monospace;font-size:1.5rem;letter-spacing:4px;
    color:var(--accent);text-shadow:0 0 20px rgba(0,229,255,.5)}
  .logo span{color:var(--accent2)}
  .conn{display:flex;align-items:center;gap:8px;font-size:.75rem;
    font-family:'Share Tech Mono',monospace;color:var(--dim)}
  .dot{width:10px;height:10px;border-radius:50%;background:var(--danger);
    box-shadow:0 0 8px var(--danger);transition:all .3s}
  .dot.on{background:var(--safe);box-shadow:0 0 8px var(--safe)}
  .mode-row{width:100%;max-width:500px}
  .mode-toggle{display:flex;background:var(--surface);border:1px solid var(--border);
    border-radius:var(--r);overflow:hidden}
  .mbtn{flex:1;padding:14px;border:none;background:transparent;color:var(--dim);
    font-family:'Exo 2',sans-serif;font-weight:600;font-size:.85rem;letter-spacing:2px;cursor:pointer;transition:all .25s}
  .mbtn.am{background:linear-gradient(135deg,#ff6b00,#ff3a00);color:#fff;box-shadow:0 0 20px rgba(255,107,0,.4)}
  .mbtn.aa{background:linear-gradient(135deg,#00e5ff,#0090ff);color:#000;box-shadow:0 0 20px rgba(0,229,255,.4)}
  .alert{width:100%;max-width:500px;background:rgba(255,45,45,.1);border:1px solid rgba(255,45,45,.3);
    border-radius:var(--r);padding:10px 16px;font-family:'Share Tech Mono',monospace;
    font-size:.78rem;color:var(--danger);display:none}
  .alert.show{display:block}
  .dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;width:250px;margin:0 auto}
  .cb{aspect-ratio:1;border:1px solid var(--border);border-radius:var(--r);
    background:var(--surface);color:var(--text);font-size:1.6rem;
    display:flex;align-items:center;justify-content:center;cursor:pointer;
    transition:all .12s;-webkit-tap-highlight-color:transparent}
  .cb:active,.cb.p{border-color:var(--accent);background:rgba(0,229,255,.12);
    box-shadow:0 0 16px rgba(0,229,255,.3);transform:scale(.94)}
  .sb{background:rgba(255,45,45,.1);border-color:rgba(255,45,45,.3);
    font-size:.75rem;font-weight:800;letter-spacing:1px;color:var(--danger)}
  .sb:active,.sb.p{background:rgba(255,45,45,.3);border-color:var(--danger);box-shadow:0 0 20px rgba(255,45,45,.5)}
  .empty{visibility:hidden}
  .tel{width:100%;max-width:500px;background:var(--surface);border:1px solid var(--border);
    border-radius:var(--r);padding:16px}
  .tl{font-family:'Share Tech Mono',monospace;font-size:.7rem;letter-spacing:3px;color:var(--dim);margin-bottom:12px}
  .tg{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
  .ti{background:var(--bg);border:1px solid var(--border);border-radius:8px;padding:10px 12px}
  .tk{font-family:'Share Tech Mono',monospace;font-size:.62rem;color:var(--dim);letter-spacing:2px;margin-bottom:4px}
  .tv{font-family:'Share Tech Mono',monospace;font-size:1.15rem;color:var(--accent);font-weight:bold}
  .tv.d{color:var(--danger)} .tv.s{color:var(--safe)}
  .sbar{width:100%;max-width:500px;display:flex;justify-content:space-between;
    align-items:center;font-family:'Share Tech Mono',monospace;font-size:.65rem;
    color:var(--dim);border-top:1px solid var(--border);padding-top:10px}
  .chip{background:rgba(0,229,255,.1);border:1px solid rgba(0,229,255,.2);
    border-radius:20px;padding:3px 12px;color:var(--accent);letter-spacing:2px}
  body.auto .cb:not(.sb){opacity:.3;pointer-events:none}
</style>
</head>
<body>
<div class="header">
  <div class="logo">AEON<span>X</span></div>
  <div class="conn"><div class="dot" id="dot"></div><span id="cl">OFFLINE</span></div>
</div>
<div class="mode-row">
  <div class="mode-toggle">
    <button class="mbtn am" id="bm" onclick="setMode('MANUAL')">◉ MANUAL</button>
    <button class="mbtn"    id="ba" onclick="setMode('AUTO')">◎ AUTONOMOUS</button>
  </div>
</div>
<div class="alert" id="al">⚠ <span id="at"></span></div>
<div class="dpad">
  <div class="empty"></div>
  <div class="cb" id="bf" onpointerdown="press('FWD',this)" onpointerup="rel(this)" onpointerleave="rel(this)">▲</div>
  <div class="empty"></div>
  <div class="cb" id="bl" onpointerdown="press('LEFT',this)" onpointerup="rel(this)" onpointerleave="rel(this)">◀</div>
  <div class="cb sb" id="bs" onpointerdown="sc('STOP');this.classList.add('p')" onpointerup="this.classList.remove('p')">STOP</div>
  <div class="cb" id="br" onpointerdown="press('RIGHT',this)" onpointerup="rel(this)" onpointerleave="rel(this)">▶</div>
  <div class="empty"></div>
  <div class="cb" id="bb" onpointerdown="press('REV',this)" onpointerup="rel(this)" onpointerleave="rel(this)">▼</div>
  <div class="empty"></div>
</div>
<div class="tel">
  <div class="tl">◈ SENSOR TELEMETRY</div>
  <div class="tg">
    <div class="ti"><div class="tk">FRONT cm</div><div class="tv" id="tf">—</div></div>
    <div class="ti"><div class="tk">REAR cm</div><div class="tv" id="tr">—</div></div>
    <div class="ti"><div class="tk">LEFT cm</div><div class="tv" id="tl2">—</div></div>
    <div class="ti"><div class="tk">RIGHT cm</div><div class="tv" id="trg">—</div></div>
    <div class="ti"><div class="tk">IR LEFT</div><div class="tv" id="til">—</div></div>
    <div class="ti"><div class="tk">IR RIGHT</div><div class="tv" id="tir">—</div></div>
  </div>
</div>
<div class="sbar">
  <div class="chip" id="sc2">IDLE</div>
  <div id="cc">CMD:0</div>
  <div id="lu">—</div>
</div>
<script>
  let mode='MANUAL', cnt=0, hold=null, errCnt=0;
  const STATES=['INIT','IDLE','FWD','REV','LEFT','RIGHT','STOP','E-STOP','DEGRADED'];

  function sc(cmd){
    fetch('/cmd?c='+cmd).then(r=>{if(r.ok){cnt++;document.getElementById('cc').textContent='CMD:'+cnt;}
    }).catch(()=>{});
  }

  /* BUG-E fix: mode guard inside cmdRelease */
  function press(cmd,btn){
    if(mode!=='MANUAL')return;
    btn.classList.add('p'); sc(cmd);
    hold=setInterval(()=>sc(cmd),120);
  }
  function rel(btn){
    btn.classList.remove('p');
    clearInterval(hold); hold=null;
    if(mode==='MANUAL') sc('STOP');
  }

  function setMode(m){
    mode=m;
    sc(m==='AUTO'?'AUTO':'MANUAL');
    document.getElementById('bm').className='mbtn'+(m==='MANUAL'?' am':'');
    document.getElementById('ba').className='mbtn'+(m==='AUTO'?' aa':'');
    m==='AUTO'?document.body.classList.add('auto'):document.body.classList.remove('auto');
  }

  function setOnline(v){
    document.getElementById('dot').className='dot'+(v?' on':'');
    document.getElementById('cl').textContent=v?'ONLINE':'OFFLINE';
  }

  function dc(el,v){
    el.textContent=v>=999?'ERR':v+'cm';
    el.className='tv'+(v<25?' d':v<50?'':' s');
  }

  function renderTel(d){
    setOnline(true); errCnt=0;
    document.getElementById('sc2').textContent=STATES[d.state]||d.state;
    dc(document.getElementById('tf'), d.f);
    dc(document.getElementById('tr'), d.r);
    dc(document.getElementById('tl2'),d.l);
    dc(document.getElementById('trg'),d.rg);
    const il=document.getElementById('til');
    il.textContent=d.irl?'BLOCKED':'CLEAR'; il.className='tv'+(d.irl?' d':' s');
    const ir=document.getElementById('tir');
    ir.textContent=d.irr?'BLOCKED':'CLEAR'; ir.className='tv'+(d.irr?' d':' s');
    document.getElementById('lu').textContent=new Date().toLocaleTimeString();
  }

  function showAlert(msg){
    document.getElementById('at').textContent=msg;
    const b=document.getElementById('al'); b.classList.add('show');
    setTimeout(()=>b.classList.remove('show'),4000);
  }

  const ev=new EventSource('/events');
  ev.addEventListener('telemetry',e=>{try{renderTel(JSON.parse(e.data));}catch(x){}});
  ev.addEventListener('alert',e=>showAlert(e.data));
  /* BUG-C fix: count errors before marking offline */
  ev.onerror=()=>{errCnt++;if(errCnt>3)setOnline(false);};
  ev.onopen=()=>{errCnt=0;setOnline(true);};

  document.addEventListener('keydown',e=>{
    if(mode!=='MANUAL')return;
    const m={'ArrowUp':'FWD','ArrowDown':'REV','ArrowLeft':'LEFT','ArrowRight':'RIGHT',' ':'STOP'};
    if(m[e.key]){e.preventDefault();sc(m[e.key]);}
  });
  document.addEventListener('keyup',e=>{
    if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)&&mode==='MANUAL')sc('STOP');
  });
</script>
</body>
</html>
)=====";

/* ============================================================
 *  SETUP
 * ============================================================ */
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== AEON X ESP32 v1.1 BOOT ===");

    /* BUG-H fix: 4-arg Serial2.begin() with explicit pins */
    Serial2.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    Serial.printf("[UART] Serial2 RX=%d TX=%d @ %d\n", STM32_RX_PIN, STM32_TX_PIN, STM32_BAUD);

    /* BUG-D fix: configure IP before starting AP, then wait */
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    delay(500);  /* allow AP to fully init before reading IP */

    Serial.printf("[WiFi] AP: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    /* Dashboard */
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send_P(200, "text/html", DASHBOARD_HTML);
    });

    /* Command endpoint */
    server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("c")) { req->send(400, "text/plain", "Missing c"); return; }

        String cmd = req->getParam("c")->value();
        cmd.toUpperCase();

        /* Mode tracking */
        if (cmd == "AUTO")   gStatus.mode = MODE_AUTONOMOUS;
        if (cmd == "MANUAL") gStatus.mode = MODE_MANUAL;

        /* Block motion commands in autonomous mode (only STOP/ESTOP/MANUAL allowed) */
        if (gStatus.mode == MODE_AUTONOMOUS &&
            cmd != "STOP" && cmd != "ESTOP" && cmd != "MANUAL") {
            req->send(403, "text/plain", "AUTO mode active");
            return;
        }

        cmd += "\n";
        sendToSTM32(cmd.c_str());
        req->send(200, "text/plain", "OK");
    });

    /* JSON status */
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        /* BUG-F fix: DynamicJsonDocument instead of StaticJsonDocument<256> */
        DynamicJsonDocument doc(512);
        doc["mode"]     = (int)gStatus.mode;
        doc["state"]    = gStatus.stm32_state;
        doc["front_cm"] = gStatus.front_cm;
        doc["rear_cm"]  = gStatus.rear_cm;
        doc["left_cm"]  = gStatus.left_cm;
        doc["right_cm"] = gStatus.right_cm;
        doc["ir_left"]  = gStatus.ir_left;
        doc["ir_right"] = gStatus.ir_right;
        doc["online"]   = gStatus.stm32_online;
        doc["cmds"]     = gStatus.cmd_count;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/plain", "AEON-X-ONLINE");
    });

    /* SSE handler */
    events.onConnect([](AsyncEventSourceClient *client) {
        Serial.println("[SSE] Client connected");
        client->send("connected", "status", millis());
    });
    server.addHandler(&events);

    server.onNotFound([](AsyncWebServerRequest *req) { req->send(404, "text/plain", "404"); });
    server.begin();
    Serial.println("[HTTP] Server started — open http://192.168.4.1");
}

/* ============================================================
 *  LOOP
 * ============================================================ */
uint32_t last_push     = 0;
const uint32_t PUSH_MS = 150;
const uint32_t TIMEOUT = 2000;

void loop() {
    /* ── Read STM32 UART ── */
    while (Serial2.available()) {
        char c = (char)Serial2.read();

        if (c == '\n' || c == '\r') {
            if (uart_idx > 0) {
                uart_buf[uart_idx] = '\0';

                if (strncmp(uart_buf, "TEL:", 4) == 0) {
                    parseTelemetry(uart_buf);
                }
                else if (strncmp(uart_buf, "ALERT:", 6) == 0) {
                    String msg = String(uart_buf + 6);
                    Serial.printf("[ALERT] %s\n", uart_buf + 6);
                    /* BUG-B fix: guard events.send() */
                    if (events.count() > 0) events.send(msg.c_str(), "alert", millis());
                }

                uart_idx = 0;
            }
        } else {
            /* BUG-G fix: hard reset on buffer overflow */
            if (uart_idx >= UART_BUF_SIZE - 1) {
                Serial.println("[WARN] UART buf overflow — reset");
                uart_idx = 0;
            } else {
                uart_buf[uart_idx++] = c;
            }
        }
    }

    /* ── STM32 timeout detection ── */
    if (gStatus.stm32_online &&
        (millis() - gStatus.last_tel_ms) > TIMEOUT) {
        gStatus.stm32_online = false;
        if (events.count() > 0) events.send("STM32_OFFLINE", "alert", millis());
        Serial.println("[WARN] STM32 offline");
    }

    /* ── Push telemetry via SSE ── */
    if ((millis() - last_push) >= PUSH_MS && events.count() > 0) {
        last_push = millis();
        char json[160];
        snprintf(json, sizeof(json),
                 "{\"state\":%d,\"mode\":%d,"
                 "\"f\":%lu,\"r\":%lu,\"l\":%lu,\"rg\":%lu,"
                 "\"irl\":%s,\"irr\":%s}",
                 gStatus.stm32_state, (int)gStatus.mode,
                 gStatus.front_cm, gStatus.rear_cm,
                 gStatus.left_cm,  gStatus.right_cm,
                 gStatus.ir_left  ? "true" : "false",
                 gStatus.ir_right ? "true" : "false");
        events.send(json, "telemetry", millis());
    }
}
