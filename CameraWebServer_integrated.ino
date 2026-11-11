#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include <WiFi.h>
#include "esp_wifi.h"
#include <TinyGPSPlus.h>
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "sauron_poster.h"
#include "elst_map.h"

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "BosbrandDrone";
const char *password = "12345678";

// GPS: GEP-M10-DQ TX -> ESP32 GPIO45, GND -> GND.
// HardwareSerial(1) uses UART1; TX is unused (-1).
static HardwareSerial gpsSerial(1);
static TinyGPSPlus gps;
static bool gpsDataSeen = false;
static bool gpsSentenceSeen = false;
static bool gpsConnectedMessagePrinted = false;
static uint32_t lastGpsStatusPrint = 0;
static uint32_t gpsBaudStartedAt = 0;
static uint8_t gpsBaudIndex = 0;
static uint32_t gpsRawBytesPrinted = 0;
static uint8_t gpsRawWindow[32] = {};
static uint8_t gpsRawWindowSize = 0;
static constexpr uint32_t GPS_PRIMARY_BAUD = 115200;
static constexpr uint32_t GPS_BAUDS[] = {115200, 9600, 38400, 4800, 57600};
static bool gpsUbxDetected = false;
static uint32_t gpsUbxMessages = 0;
static uint32_t gpsUbxChecksumFailures = 0;

struct GpsState {
  bool hasFix = false;
  uint8_t fixType = 0;
  uint8_t satellites = 0;
  int32_t latitudeE7 = 0;
  int32_t longitudeE7 = 0;
  int32_t altitudeMm = 0;
  int32_t groundSpeedMmS = 0;
  int32_t headingE5 = 0;
  uint16_t hdopHundredths = 0;
  bool hdopValid = false;
};

static GpsState gpsState;

static uint8_t gpsUbxBuffer[256] = {};
static uint16_t gpsUbxLength = 0;
static uint16_t gpsUbxPayloadIndex = 0;
static uint8_t gpsUbxClass = 0;
static uint8_t gpsUbxId = 0;
static uint8_t gpsUbxChecksumA = 0;
static uint8_t gpsUbxChecksumB = 0;
static enum class GpsUbxParserState : uint8_t {
  WaitingForSync,
  GotSyncB5,
  ReadingClass,
  ReadingId,
  ReadingLengthLow,
  ReadingLengthHigh,
  ReadingPayload,
  ReadingChecksumA,
  ReadingChecksumB
} gpsUbxParserState = GpsUbxParserState::WaitingForSync;
static httpd_handle_t cameraHttpServer = nullptr;
static httpd_handle_t cameraStreamServer = nullptr;
static bool cameraReady = false;

static constexpr framesize_t STREAM_FRAME_SIZE = FRAMESIZE_QQVGA;
static constexpr uint8_t STREAM_JPEG_QUALITY = 40;
static constexpr uint8_t STREAM_JPEG_QUALITY_MIN = 40;
static constexpr uint8_t STREAM_JPEG_QUALITY_MAX = 40;
static constexpr uint32_t STREAM_SEND_SLOW_US = 150000;
static constexpr uint32_t STREAM_SEND_FAST_US = 80000;
static constexpr uint32_t STREAM_NO_FRAME_RETRY_MS = 1;

static volatile uint32_t streamFramesSent = 0;
static volatile uint32_t streamFramesCaptured = 0;
static volatile uint32_t streamFramesSkipped = 0;
static volatile uint32_t streamCaptureErrors = 0;
static volatile uint32_t streamSendErrors = 0;
static volatile uint32_t streamLastFrameBytes = 0;
static volatile uint32_t streamTotalFrameBytes = 0;
static volatile uint32_t streamTotalCaptureUs = 0;
static volatile uint32_t streamTotalSendUs = 0;
static volatile uint32_t streamLastFrameAtMs = 0;
static volatile uint32_t streamLastFrameIntervalUs = 0;
static volatile uint32_t streamLastCaptureUs = 0;
static volatile uint32_t streamLastSendUs = 0;
static volatile uint32_t streamStartedAtMs = 0;
static volatile uint32_t streamFpsWindowStartedMs = 0;
static volatile uint32_t streamFpsWindowFrames = 0;
static volatile uint8_t streamJpegQuality = STREAM_JPEG_QUALITY;
static uint8_t streamSlowSendSamples = 0;
static uint8_t streamFastSendSamples = 0;
static sensor_t *streamCameraSensor = nullptr;

static bool startWiFiAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    Serial.println("Warning: WiFi power-save could not be disabled");
  }

  Serial.println("Starting WiFi access point...");
  if (!WiFi.softAP(ssid, password, 6, false, 1)) {
    Serial.println("Failed to start WiFi AP!");
    return false;
  }

  Serial.println("WiFi AP started!");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("' to connect");
  return true;
}

static void configureStreamSocket(httpd_req_t *request) {
  const int socketFd = httpd_req_to_sockfd(request);
  if (socketFd < 0) {
    return;
  }
  const int enabled = 1;
  if (setsockopt(socketFd, IPPROTO_TCP, TCP_NODELAY,
                 &enabled, sizeof(enabled)) != 0) {
    Serial.println("Warning: TCP_NODELAY kon niet worden ingesteld");
  }
}

static void adaptStreamQuality(uint32_t sendUs, bool sendFailed) {
  if (sendFailed || sendUs >= STREAM_SEND_SLOW_US) {
    streamFastSendSamples = 0;
    if (streamSlowSendSamples < 3) {
      ++streamSlowSendSamples;
    }
    if (streamSlowSendSamples >= 2 &&
        streamJpegQuality < STREAM_JPEG_QUALITY_MAX) {
      streamJpegQuality = static_cast<uint8_t>(
        min(static_cast<int>(STREAM_JPEG_QUALITY_MAX),
            static_cast<int>(streamJpegQuality) + 2));
      streamSlowSendSamples = 0;
      if (streamCameraSensor != nullptr) {
        streamCameraSensor->set_quality(
          streamCameraSensor, streamJpegQuality);
      }
    }
    return;
  }

  streamSlowSendSamples = 0;
  if (sendUs <= STREAM_SEND_FAST_US) {
    if (streamFastSendSamples < 6) {
      ++streamFastSendSamples;
    }
    if (streamFastSendSamples >= 6 &&
        streamJpegQuality > STREAM_JPEG_QUALITY_MIN) {
      streamJpegQuality = static_cast<uint8_t>(
        max(static_cast<int>(STREAM_JPEG_QUALITY_MIN),
            static_cast<int>(streamJpegQuality) - 1));
      streamFastSendSamples = 0;
      if (streamCameraSensor != nullptr) {
        streamCameraSensor->set_quality(
          streamCameraSensor, streamJpegQuality);
      }
    }
  } else {
    streamFastSendSamples = 0;
  }
}

static uint32_t gpsUbxU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

static int32_t gpsUbxI32(const uint8_t *data) {
  return static_cast<int32_t>(gpsUbxU32(data));
}

static uint16_t gpsUbxU16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

static void gpsHandleUbxMessage() {
  ++gpsUbxMessages;
  gpsUbxDetected = true;
  Serial.printf("GPS UBX bericht: class=0x%02X id=0x%02X lengte=%u\n",
                gpsUbxClass, gpsUbxId, gpsUbxLength);

  if (gpsUbxClass != 0x01 || gpsUbxId != 0x07 || gpsUbxLength < 92) {
    return;
  }

  sauronMarkGpsFresh();
  const uint8_t *payload = gpsUbxBuffer;
  gpsState.fixType = payload[20];
  gpsState.hasFix = (payload[21] & 0x01) != 0 &&
                    gpsState.fixType >= 2;
  gpsState.satellites = payload[23];
  gpsState.longitudeE7 = gpsUbxI32(payload + 24);
  gpsState.latitudeE7 = gpsUbxI32(payload + 28);
  gpsState.altitudeMm = gpsUbxI32(payload + 36);
  gpsState.groundSpeedMmS = gpsUbxI32(payload + 60);
  gpsState.headingE5 = gpsUbxI32(payload + 64);
  gpsState.hdopHundredths = gpsUbxU16(payload + 76);
  gpsState.hdopValid = true;
  Serial.printf("GPS fix: type=%u valid=%s sats=%u lat=%.7f lon=%.7f "
                "alt=%.1f m speed=%.2f m/s course=%.2f deg HDOP=%.2f\n",
                gpsState.fixType, gpsState.hasFix ? "yes" : "no",
                gpsState.satellites, gpsState.latitudeE7 / 10000000.0,
                gpsState.longitudeE7 / 10000000.0,
                gpsState.altitudeMm / 1000.0,
                gpsState.groundSpeedMmS / 1000.0,
                gpsState.headingE5 / 100000.0,
                gpsState.hdopHundredths / 100.0);
}

static void gpsUbxReset() {
  gpsUbxParserState = GpsUbxParserState::WaitingForSync;
  gpsUbxLength = 0;
  gpsUbxPayloadIndex = 0;
  gpsUbxChecksumA = 0;
  gpsUbxChecksumB = 0;
}

static void gpsUbxChecksumAdd(uint8_t value) {
  gpsUbxChecksumA = static_cast<uint8_t>(gpsUbxChecksumA + value);
  gpsUbxChecksumB = static_cast<uint8_t>(gpsUbxChecksumB + gpsUbxChecksumA);
}

static void gpsParseUbxByte(uint8_t value) {
  switch (gpsUbxParserState) {
    case GpsUbxParserState::WaitingForSync:
      if (value == 0xB5) {
        gpsUbxParserState = GpsUbxParserState::GotSyncB5;
      }
      break;
    case GpsUbxParserState::GotSyncB5:
      if (value == 0x62) {
        Serial.println("GPS UBX sync gedetecteerd (0xB5 0x62)");
        gpsUbxChecksumA = 0;
        gpsUbxChecksumB = 0;
        gpsUbxParserState = GpsUbxParserState::ReadingClass;
      } else {
        gpsUbxParserState = value == 0xB5
                              ? GpsUbxParserState::GotSyncB5
                              : GpsUbxParserState::WaitingForSync;
      }
      break;
    case GpsUbxParserState::ReadingClass:
      gpsUbxClass = value;
      gpsUbxChecksumAdd(value);
      gpsUbxParserState = GpsUbxParserState::ReadingId;
      break;
    case GpsUbxParserState::ReadingId:
      gpsUbxId = value;
      gpsUbxChecksumAdd(value);
      gpsUbxParserState = GpsUbxParserState::ReadingLengthLow;
      break;
    case GpsUbxParserState::ReadingLengthLow:
      gpsUbxLength = value;
      gpsUbxChecksumAdd(value);
      gpsUbxParserState = GpsUbxParserState::ReadingLengthHigh;
      break;
    case GpsUbxParserState::ReadingLengthHigh:
      gpsUbxLength |= static_cast<uint16_t>(value) << 8;
      gpsUbxChecksumAdd(value);
      if (gpsUbxLength > sizeof(gpsUbxBuffer)) {
        Serial.printf("GPS UBX bericht te groot: %u bytes\n", gpsUbxLength);
        gpsUbxReset();
      } else if (gpsUbxLength == 0) {
        gpsUbxParserState = GpsUbxParserState::ReadingChecksumA;
      } else {
        gpsUbxPayloadIndex = 0;
        gpsUbxParserState = GpsUbxParserState::ReadingPayload;
      }
      break;
    case GpsUbxParserState::ReadingPayload:
      gpsUbxBuffer[gpsUbxPayloadIndex++] = value;
      gpsUbxChecksumAdd(value);
      if (gpsUbxPayloadIndex == gpsUbxLength) {
        gpsUbxParserState = GpsUbxParserState::ReadingChecksumA;
      }
      break;
    case GpsUbxParserState::ReadingChecksumA:
      if (value != gpsUbxChecksumA) {
        ++gpsUbxChecksumFailures;
        Serial.printf("GPS UBX checksum A ongeldig: verwacht=%02X ontvangen=%02X\n",
                      gpsUbxChecksumA, value);
        gpsUbxReset();
      } else {
        gpsUbxParserState = GpsUbxParserState::ReadingChecksumB;
      }
      break;
    case GpsUbxParserState::ReadingChecksumB:
      if (value == gpsUbxChecksumB) {
        gpsHandleUbxMessage();
      } else {
        ++gpsUbxChecksumFailures;
        Serial.printf("GPS UBX checksum B ongeldig: verwacht=%02X ontvangen=%02X\n",
                      gpsUbxChecksumB, value);
      }
      gpsUbxReset();
      break;
  }
}

void setupLedFlash();

static esp_err_t sendCors(httpd_req_t *request) {
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(request, "Access-Control-Allow-Methods", "GET, OPTIONS");
  httpd_resp_set_hdr(request, "Access-Control-Allow-Headers", "Content-Type");
  return ESP_OK;
}

static esp_err_t optionsHandler(httpd_req_t *request) {
  sendCors(request);
  httpd_resp_send(request, nullptr, 0);
  return ESP_OK;
}

static esp_err_t indexHandler(httpd_req_t *request) {
  static const char page[] =
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>BosbrandDrone</title><style>"
    "*{box-sizing:border-box}body{margin:0;background:#101419;color:#e8edf2;font:14px Arial,sans-serif}"
    "header{height:46px;padding:12px 16px;background:#182129;border-bottom:1px solid #35404a;font-weight:bold}"
    "header span{float:right;color:#83d98b;font-weight:normal}.grid{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(280px,.65fr);grid-template-rows:minmax(360px,58vh) minmax(260px,1fr) auto;gap:10px;padding:10px;min-height:calc(100vh - 46px)}"
    ".panel{min-height:0;background:#182129;border:1px solid #35404a;border-radius:6px;padding:10px;position:relative;overflow:hidden}.title{color:#9fb1c0;font-size:12px;text-transform:uppercase;letter-spacing:.08em;margin-bottom:7px}"
    "#rgb{width:100%;height:calc(100% - 25px);object-fit:contain;background:#080b0e;transform:scaleX(-1)}canvas{width:100%;height:calc(100% - 25px);display:block;background:#0b1014;object-fit:contain}.rgb-panel{grid-column:1/-1;min-height:58vh}"
    "#telemetry{display:grid;grid-template-columns:1fr 1fr;gap:8px;align-content:start}.metric{background:#11181e;border-radius:4px;padding:9px}.metric b{display:block;color:#8fa4b5;font-size:11px;margin-bottom:4px}.metric strong{font-size:18px;color:#fff}.wide{grid-column:1/-1}.raw-panel{grid-column:1/-1;min-height:170px}.raw{margin:0;max-height:240px;overflow:auto;white-space:pre-wrap;word-break:break-word;color:#b9d1df;font:11px monospace;background:#0b1014;padding:8px;border-radius:4px}"
    "#message{position:absolute;right:14px;top:12px;color:#f0b56b;font-size:12px}@media(max-width:700px){.grid{grid-template-columns:1fr;grid-template-rows:auto;min-height:0}.panel{min-height:220px}.rgb-panel{min-height:300px}.raw-panel{min-height:170px}}"
    "</style></head><body><header>BOSBRANDDRONE <span id=state>connecting...</span></header>"
    "<main class=grid><section class=\"panel rgb-panel\"><div class=title>RGB camera</div><img id=rgb alt=\"RGB camera stream\"></section>"
    "<section class=panel><div class=title>Thermal camera - hottest point</div><div id=message></div><canvas id=thermal width=320 height=240></canvas></section>"
    "<section class=panel><div class=title>GPS flight map</div><canvas id=map width=640 height=360></canvas></section>"
    "<section class=panel><div class=title>Flight information</div><div id=telemetry>"
    "<div class=metric><b>Altitude MSL</b><strong id=alt>-</strong></div><div class=metric><b>Speed</b><strong id=speed>-</strong></div>"
    "<div class=metric><b>Hotspot</b><strong id=hot>-</strong></div><div class=metric><b>Thermal range</b><strong id=range>-</strong></div>"
    "<div class=\"metric\"><b>Wi-Fi to laptop</b><strong id=wifi>-</strong></div><div class=\"metric\"><b>Wi-Fi RSSI</b><strong id=wifi_rssi>-</strong></div>"
    "<div class=\"metric\"><b>GPS satellites</b><strong id=sats>-</strong></div><div class=\"metric\"><b>GPS signal</b><strong id=gps_quality>-</strong></div>"
    "<div class=\"metric wide\"><b>GPS position</b><strong id=pos>-</strong></div><div class=\"metric wide\"><b>Fix / satellites / HDOP</b><strong id=fix>-</strong></div>"
    "<div class=\"metric\"><b>Camera FPS</b><strong id=camera_fps>-</strong></div><div class=\"metric\"><b>Ontvangst-FPS</b><strong id=receive_fps>-</strong></div>"
    "<div class=\"metric\"><b>Weergave-FPS (browser)</b><strong id=display_fps>-</strong></div><div class=\"metric\"><b>Browser ontvangst</b><strong id=browser_receive_fps>niet meetbaar</strong></div>"
    "<div class=\"metric\"><b>JPEG-grootte</b><strong id=jpeg_size>-</strong></div>"
    "<div class=\"metric\"><b>Capture + JPEG</b><strong id=capture_time>-</strong></div><div class=\"metric\"><b>JPEG verwerking</b><strong id=jpeg_time>niet meetbaar</strong></div>"
    "<div class=\"metric\"><b>Tijd tussen frames</b><strong id=frame_interval>-</strong></div><div class=\"metric\"><b>Streamtijd</b><strong id=stream_time>-</strong></div>"
    "<div class=\"metric wide\"><b>End-to-end latency</b><strong id=latency>niet meetbaar zonder gesynchroniseerde klok</strong></div>"
    "</div></section><section class=\"panel raw-panel\"><div class=title>Raw data GPS / thermal / stream</div><pre id=raw class=raw>waiting for data...</pre></section></main><script>"
    "const rgb=document.getElementById('rgb'),tc=document.getElementById('thermal'),tx=tc.getContext('2d'),mc=document.getElementById('map'),mx=mc.getContext('2d'),trail=[];let latestGpsData=null,latestThermalJson='';"
    "let displayFrames=0,displayWindowStarted=performance.now(),displayFps=0,lastStatusAt=0;"
    "let rgbRetryTimer=0,rgbRetryDelay=100;"
    "function connectRgb(){clearTimeout(rgbRetryTimer);document.getElementById('state').textContent='camera verbinden...';rgb.src='http://'+location.hostname+':81/stream?t='+Date.now();}"
    "function retryRgb(){clearTimeout(rgbRetryTimer);document.getElementById('state').textContent='camera opnieuw verbinden...';rgbRetryTimer=setTimeout(connectRgb,rgbRetryDelay);rgbRetryDelay=Math.min(rgbRetryDelay*2,500);}"
    "rgb.onload=()=>{rgbRetryDelay=100;displayFrames++;let now=performance.now(),elapsed=now-displayWindowStarted;if(elapsed>=1000){displayFps=displayFrames*1000/elapsed;displayFrames=0;displayWindowStarted=now;}document.getElementById('display_fps').textContent=displayFps.toFixed(1);document.getElementById('state').textContent='online'};"
    "rgb.onerror=retryRgb;"
    "window.addEventListener('online',connectRgb);"
    "connectRgb();"
    "function val(v,u){return v==null||!isFinite(v)?'-':Number(v).toFixed(1)+' '+u}"
    "function wifiLabel(rssi){if(rssi==null||!isFinite(rssi))return 'no client';if(rssi>=-55)return 'excellent';if(rssi>=-67)return 'good';if(rssi>=-75)return 'fair';if(rssi>=-90)return 'weak';return 'very weak';}"
    "function wifiPercent(rssi){if(rssi==null||!isFinite(rssi))return 0;let q=Math.max(0,Math.min(100,Math.round((rssi+100)*1.6)));return q;}"
    "function colorRgb(t,a,b){let x=(t-a)/(b-a||1);x=Math.max(0,Math.min(1,x));return [Math.round(255*Math.min(1,2*x)),Math.round(255*Math.min(1,2-2*x)),Math.round(255*Math.max(0,1-2*x))]}"
    "function thermal(d){if(!d.frame){document.getElementById('message').textContent='thermal sensor unavailable';return}"
    "let a=d.thermal.min_c,b=d.thermal.max_c,w=tc.width,h=tc.height,img=tx.createImageData(w,h),src=d.frame;"
    "for(let y=0;y<h;y++){let sy=y*23/(h-1),y0=Math.floor(sy),y1=Math.min(23,y0+1),fy=sy-y0;"
    "for(let x=0;x<w;x++){let sx=x*31/(w-1),x0=Math.floor(sx),x1=Math.min(31,x0+1),fx=sx-x0,i00=y0*32+x0,i10=y0*32+x1,i01=y1*32+x0,i11=y1*32+x1;"
    "let v00=src[i00],v10=src[i10],v01=src[i01],v11=src[i11],v=v00==null?v10:v10==null?v00:v00*(1-fx)+v10*fx;"
    "let lower=v01==null?v:v11==null?v01:v01*(1-fx)+v11*fx;if(v!=null&&lower!=null)v=v*(1-fy)+lower*fy;"
    "let c=colorRgb(v==null?a:v,a,b),o=(y*w+x)*4;img.data[o]=c[0];img.data[o+1]=c[1];img.data[o+2]=c[2];img.data[o+3]=255;}}"
    "tx.putImageData(img,0,0);latestThermalJson=JSON.stringify(d);document.getElementById('raw').textContent='THERMAL\\n'+latestThermalJson;let p=d.thermal.hotspot,fire=d.thermal.fire===true||b>=300;tx.strokeStyle=fire?'#ff2020':'#fff';tx.lineWidth=fire?4:2;tx.strokeRect(p.x*w/32,p.y*h/24,w/32,h/24);document.getElementById('hot').textContent=fire?'VUUR':val(b,'C');document.getElementById('range').textContent=val(a,'C')+' - '+(fire?'VUUR':val(b,'C'));document.getElementById('message').textContent=fire?'VUUR: meetbereik overschreden':'hotspot x'+p.x+' y'+p.y}"
    "const mapBounds={minLat:51.90544,maxLat:51.91454,minLon:5.81623,maxLon:5.84168};"
    "const elstMap=" ELST_MAP_DATA ";"
    "function mapPoint(lat,lon){return {x:(lon-mapBounds.minLon)/(mapBounds.maxLon-mapBounds.minLon)*mc.width,y:mc.height-(lat-mapBounds.minLat)/(mapBounds.maxLat-mapBounds.minLat)*mc.height};}"
    "function mapLine(points,color,width){mx.beginPath();points.forEach((q,i)=>{let z=mapPoint(q[0],q[1]);if(i)mx.lineTo(z.x,z.y);else mx.moveTo(z.x,z.y)});mx.strokeStyle=color;mx.lineWidth=width;mx.stroke();}"
    "function mapLabel(text,lat,lon){let z=mapPoint(lat,lon);mx.fillStyle='#314350';mx.font='bold 11px Arial';mx.fillText(text,z.x+4,z.y-4);}"
    "function map(p){"
    "mx.clearRect(0,0,mc.width,mc.height);mx.fillStyle='#dce8d2';mx.fillRect(0,0,mc.width,mc.height);"
    "const roadStyle={M:['#d18b35',3],T:['#e4ad55',2.5],R:['#ffffff',2],L:['#ffffff',1.5],U:['#ffffff',1.5],S:['#f5f5f5',1],C:['#82a96b',1.2]};"
    "elstMap.forEach(w=>{let s=roadStyle[w.t]||roadStyle.R;mapLine(w.p.map(q=>[mapBounds.minLat+q[0]/100000,mapBounds.minLon+q[1]/100000]),s[0],s[1]);});"
    "elstMap.forEach(w=>{if(w.n&&(w.t==='M'||w.t==='T')){let q=w.p[Math.floor(w.p.length/2)];mapLabel(w.n,mapBounds.minLat+q[0]/100000,mapBounds.minLon+q[1]/100000);}});"
    "mapLabel('Elst',51.9108,5.8265);mx.fillStyle='#536b77';mx.font='11px Arial';mx.fillText('offline OpenStreetMap · Elst',10,17);"
    "if(!p||p.lat==null||p.lon==null){mx.fillStyle='#314350';mx.font='14px Arial';mx.fillText('Waiting for GPS fix...',15,38);return}"
    "trail.push([Number(p.lat),Number(p.lon)]);if(trail.length>200)trail.shift();"
    "mx.setLineDash([5,4]);mapLine(trail,'#1685c7',2);mx.setLineDash([]);"
    "let z=mapPoint(Number(p.lat),Number(p.lon));mx.fillStyle='#e3342f';mx.beginPath();mx.arc(z.x,z.y,8,0,7);mx.fill();mx.strokeStyle='#fff';mx.lineWidth=2;mx.stroke();"
    "mx.fillStyle='#1f2c33';mx.font='bold 11px Arial';mx.fillText(Number(p.lat).toFixed(6)+', '+Number(p.lon).toFixed(6),z.x+10,z.y+4);}"
    "let updateBusy=false;"
    "async function update(){if(updateBusy)return;updateBusy=true;"
    "try{let r=await fetch('/api/data',{cache:'no-store'});if(r.ok)thermal(await r.json());}"
    "catch(e){document.getElementById('message').textContent='thermal data unavailable';}"
    "try{let g=await fetch('/api/gps',{cache:'no-store'});if(!g.ok)throw Error('GPS HTTP '+g.status);let p=await g.json();"
    "latestGpsData=p;map(p);document.getElementById('alt').textContent=val(p.alt_msl,'m');"
    "document.getElementById('speed').textContent=val(p.speed_mps,'m/s');"
    "document.getElementById('pos').textContent=p.lat==null?'-':Number(p.lat).toFixed(6)+', '+Number(p.lon).toFixed(6);"
    "document.getElementById('fix').textContent=(p.fix==null?'-':p.fix)+' / '+(p.sats==null?'-':p.sats)+' / '+val(p.hdop,'');"
    "document.getElementById('sats').textContent=p.sats==null?'-':String(p.sats);"
    "document.getElementById('gps_quality').textContent=(p.hdop==null||!isFinite(p.hdop))?'-':(p.hdop<2?'excellent':p.hdop<5?'good':p.hdop<8?'fair':'weak');}"
    "catch(e){map(null);document.getElementById('alt').textContent='-';document.getElementById('speed').textContent='-';"
    "document.getElementById('pos').textContent='-';document.getElementById('fix').textContent='-';"
    "document.getElementById('sats').textContent='-';document.getElementById('gps_quality').textContent='-';}"
    "if(Date.now()-lastStatusAt>=500){lastStatusAt=Date.now();let s=await fetch('/api/status',{cache:'no-store'});if(!s.ok)throw Error('status HTTP '+s.status);let st=await s.json();"
    "let rssi=st&&st.wifi&&st.wifi.rssi_dbm!=null?Number(st.wifi.rssi_dbm):-Infinity;"
    "let wifiQuality=wifiLabel(rssi),wifiSignalPercent=wifiPercent(rssi);"
    "document.getElementById('wifi').textContent=wifiQuality+' ('+wifiSignalPercent+'%)';"
    "document.getElementById('wifi_rssi').textContent=rssi===-Infinity?'-':rssi+' dBm';let stream=st.stream||{};document.getElementById('camera_fps').textContent=Number(stream.current_fps||0).toFixed(1);document.getElementById('receive_fps').textContent=Number(stream.average_fps||0).toFixed(1)+' (ESP32)';document.getElementById('browser_receive_fps').textContent=stream.browser_receive_fps==null?'niet meetbaar':Number(stream.browser_receive_fps).toFixed(1);document.getElementById('jpeg_size').textContent=(stream.average_frame_bytes||0)+' bytes';document.getElementById('capture_time').textContent=stream.average_capture_us?Math.round(stream.average_capture_us/1000)+' ms':'-';document.getElementById('jpeg_time').textContent=stream.jpeg_processing_measurable?'gemeten':'niet meetbaar';document.getElementById('frame_interval').textContent=stream.last_frame_interval_us?Math.round(stream.last_frame_interval_us/1000)+' ms':'-';document.getElementById('stream_time').textContent=stream.stream_time_ms?Math.round(stream.stream_time_ms/1000)+' s':'-';document.getElementById('raw').textContent='THERMAL\\n'+latestThermalJson+'\\n\\nGPS\\n'+JSON.stringify(latestGpsData)+'\\n\\nSTATUS\\n'+JSON.stringify(st);"
    "document.getElementById('state').textContent=(rgb.complete&&rgb.naturalWidth)?'online · Wi-Fi '+wifiSignalPercent+'%':'offline';}"
    "catch(e){document.getElementById('wifi').textContent='no client';document.getElementById('wifi_rssi').textContent='-';"
    "document.getElementById('state').textContent=(rgb.complete&&rgb.naturalWidth)?'online':'offline';}"
    "updateBusy=false;}"
    "setInterval(update,125);update();</script></body></html>";
  sendCors(request);
  httpd_resp_set_type(request, "text/html; charset=utf-8");
  httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static void addNoCacheHeaders(httpd_req_t *request) {
  httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");
  httpd_resp_set_hdr(request, "Pragma", "no-cache");
  httpd_resp_set_hdr(request, "Connection", "keep-alive");
  httpd_resp_set_hdr(request, "X-Accel-Buffering", "no");
}

static esp_err_t streamHandler(httpd_req_t *request) {
  static const char *contentType =
    "multipart/x-mixed-replace;boundary=frame";
  static const char *boundary = "\r\n--frame\r\n";
  static const char *part = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  sendCors(request);
  httpd_resp_set_type(request, contentType);
  addNoCacheHeaders(request);
  httpd_resp_set_hdr(request, "Access-Control-Expose-Headers", "Cache-Control");
  configureStreamSocket(request);

  const uint32_t streamStartMs = millis();
  if (streamStartedAtMs == 0) {
    streamStartedAtMs = streamStartMs;
  }
  streamFpsWindowStartedMs = streamStartMs;
  streamFpsWindowFrames = 0;
  int64_t lastFrameSentUs = 0;
  while (true) {
    const int64_t captureStartedUs = esp_timer_get_time();
    camera_fb_t *frame = esp_camera_fb_get();
    const uint32_t captureUs =
      static_cast<uint32_t>(esp_timer_get_time() - captureStartedUs);
    if (frame == nullptr) {
      ++streamCaptureErrors;
      ++streamSendErrors;
      delay(STREAM_NO_FRAME_RETRY_MS);
      continue;
    }

    ++streamFramesCaptured;
    ++streamFpsWindowFrames;
    const size_t frameLength = frame->len;
    char header[128];
    int headerLength = snprintf(header, sizeof(header), "%s%s", boundary, "");
    if (headerLength >= 0) {
      headerLength += snprintf(header + headerLength,
                               sizeof(header) - static_cast<size_t>(headerLength),
                               part,
                                static_cast<unsigned int>(frameLength));
    }
    const int64_t sendStartedUs = esp_timer_get_time();
    esp_err_t result = headerLength < 0 ||
                       static_cast<size_t>(headerLength) >= sizeof(header)
                         ? ESP_ERR_INVALID_SIZE
                         : httpd_resp_send_chunk(request, header, headerLength);
    if (result == ESP_OK) {
      result = httpd_resp_send_chunk(
        request, reinterpret_cast<const char *>(frame->buf), frameLength);
    }
    esp_camera_fb_return(frame);
    const uint32_t sendUs =
      static_cast<uint32_t>(esp_timer_get_time() - sendStartedUs);
    adaptStreamQuality(sendUs, result != ESP_OK);
    if (result != ESP_OK) {
      ++streamSendErrors;
      return result;
    }

    ++streamFramesSent;
    streamLastFrameBytes = static_cast<uint32_t>(frameLength);
    streamTotalFrameBytes += static_cast<uint32_t>(frameLength);
    streamTotalCaptureUs += captureUs;
    streamTotalSendUs += sendUs;
    streamLastCaptureUs = captureUs;
    streamLastSendUs = sendUs;
    if (lastFrameSentUs != 0) {
      streamLastFrameIntervalUs =
        static_cast<uint32_t>(sendStartedUs - lastFrameSentUs);
    }
    lastFrameSentUs = sendStartedUs;
    streamLastFrameAtMs = millis();
    if (sendUs >= STREAM_SEND_SLOW_US) {
      // A congested socket would otherwise make the browser display an old
      // queue of JPEGs. Reconnect so CAMERA_GRAB_LATEST can provide a fresh frame.
      ++streamFramesSkipped;
      ++streamSendErrors;
      return ESP_FAIL;
    }
  }
}

static esp_err_t dataHandler(httpd_req_t *request) {
  const char *json = "{\"frame\":null,\"thermal_available\":false}";
  if (sauronFrameIsFresh(1000UL) &&
      sauronBodyMutex != nullptr &&
      xSemaphoreTake(sauronBodyMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    json = sauronLaatsteJson();
    sendCors(request);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");
    esp_err_t result = httpd_resp_send(request, json, HTTPD_RESP_USE_STRLEN);
    xSemaphoreGive(sauronBodyMutex);
    return result;
  }
  sendCors(request);
  httpd_resp_set_type(request, "application/json");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");
  if (json == nullptr) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Thermal frame is not available");
    return ESP_FAIL;
  }
  httpd_resp_send(request, json, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t gpsHandler(httpd_req_t *request) {
  const char *json = sauronLaatsteGpsJson(gpsDataSeen);
  if (json == nullptr) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "GPS response is too large");
    return ESP_FAIL;
  }
  sendCors(request);
  httpd_resp_set_type(request, "application/json");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  httpd_resp_send(request, json, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t statusHandler(httpd_req_t *request) {
  char json[1280];
  const uint32_t frames = streamFramesSent;
  const uint32_t capturedFrames = streamFramesCaptured;
  const uint32_t nowMs = millis();
  const uint32_t averageBytes = frames == 0
                                  ? 0
                                  : streamTotalFrameBytes / frames;
  const uint32_t averageCaptureUs = capturedFrames == 0
                                      ? 0
                                      : streamTotalCaptureUs / capturedFrames;
  const uint32_t averageSendUs = frames == 0
                                   ? 0
                                   : streamTotalSendUs / frames;
  const uint32_t streamTimeMs = streamStartedAtMs == 0
                                  ? 0
                                  : nowMs - streamStartedAtMs;
  const uint32_t averageFps = streamTimeMs == 0
                                ? 0
                                : static_cast<uint32_t>(
                                    (static_cast<uint64_t>(capturedFrames) * 1000ULL) /
                                    streamTimeMs);
  const uint32_t fpsWindowMs = streamFpsWindowStartedMs == 0
                                ? 0
                                : nowMs - streamFpsWindowStartedMs;
  const uint32_t currentFps = fpsWindowMs == 0
                                ? 0
                                : static_cast<uint32_t>(
                                    (static_cast<uint64_t>(streamFpsWindowFrames) *
                                     1000ULL) / fpsWindowMs);
  int8_t rssiDbm = -127;
  uint8_t stationCount = WiFi.softAPgetStationNum();
  if (stationCount > 0) {
    wifi_sta_list_t stations = {};
    if (esp_wifi_ap_get_sta_list(&stations) == ESP_OK) {
      int32_t sum = 0;
      uint8_t activeStations = 0;
      for (uint8_t i = 0; i < stations.num; ++i) {
        const int8_t stationRssi = stations.sta[i].rssi;
        if (stationRssi != 0) {
          sum += stationRssi;
          ++activeStations;
        }
      }
      if (activeStations > 0) {
        rssiDbm = static_cast<int8_t>(sum / static_cast<int32_t>(activeStations));
      }
    }
  }

  int signalPercent = 0;
  if (rssiDbm > -127) {
    signalPercent = static_cast<int>(constrain((rssiDbm + 100.0f) * 1.6f, 0.0f, 100.0f));
  }
  const char *wifiQuality = "no client";
  if (rssiDbm > -127) {
    if (rssiDbm >= -55) {
      wifiQuality = "excellent";
    } else if (rssiDbm >= -67) {
      wifiQuality = "good";
    } else if (rssiDbm >= -75) {
      wifiQuality = "fair";
    } else if (rssiDbm >= -90) {
      wifiQuality = "weak";
    } else {
      wifiQuality = "very weak";
    }
  }

  int length = snprintf(
    json, sizeof(json),
    "{\"wifi\":{\"mode\":\"AP\",\"ssid\":\"%s\",\"ip\":\"%s\",\"clients\":%u,\"rssi_dbm\":%d,\"signal_percent\":%d,\"quality\":\"%s\"},"
    "\"camera\":%s,\"thermal\":{\"available\":%s,\"status\":\"%s\","
    "\"frame_errors\":%lu},\"gps\":{\"data_seen\":%s,"
    "\"location_valid\":%s,\"baud\":%lu,\"rx_gpio\":45,"
    "\"bytes_seen\":%lu,\"ubx_detected\":%s,\"ubx_messages\":%lu,"
    "\"checksum_failures\":%lu,\"protocol\":\"%s\","
    "\"fix_type\":%u,\"satellites\":%u},"
    "\"stream\":{\"frames_sent\":%lu,\"frames_received\":%lu,"
    "\"frames_captured\":%lu,\"capture_errors\":%lu,"
    "\"frames_skipped\":%lu,\"stream_time_ms\":%lu,"
    "\"average_fps\":%lu,\"current_fps\":%lu,"
    "\"send_errors\":%lu,\"last_frame_bytes\":%lu,"
    "\"average_frame_bytes\":%lu,\"average_capture_us\":%lu,"
    "\"average_send_us\":%lu,\"last_capture_us\":%lu,"
    "\"last_send_us\":%lu,\"last_frame_interval_us\":%lu,"
    "\"jpeg_quality\":%u,\"jpeg_processing_us\":null,"
    "\"jpeg_processing_measurable\":false,\"browser_receive_fps\":null,"
    "\"browser_display_fps_measurable\":false,"
    "\"last_frame_at_ms\":%lu}}",
    ssid, WiFi.softAPIP().toString().c_str(),
    static_cast<unsigned int>(stationCount),
    rssiDbm,
    signalPercent,
    wifiQuality,
    cameraReady ? "true" : "false",
    sauronHeeftFrame() ? "true" : "false",
    sauronStatus(), static_cast<unsigned long>(sauronFrameErrorCount()),
    gpsDataSeen ? "true" : "false",
    gpsState.hasFix ? "true" : "false",
    static_cast<unsigned long>(GPS_BAUDS[gpsBaudIndex]),
    static_cast<unsigned long>(gpsRawBytesPrinted),
    gpsUbxDetected ? "true" : "false",
    static_cast<unsigned long>(gpsUbxMessages),
    static_cast<unsigned long>(gpsUbxChecksumFailures),
    gpsUbxDetected ? "UBX" : (gpsSentenceSeen ? "NMEA" : "unknown"),
    gpsState.fixType,
    gpsState.satellites,
    static_cast<unsigned long>(frames),
    static_cast<unsigned long>(streamFramesCaptured),
    static_cast<unsigned long>(streamFramesCaptured),
    static_cast<unsigned long>(streamCaptureErrors),
    static_cast<unsigned long>(streamFramesSkipped),
    static_cast<unsigned long>(streamTimeMs),
    static_cast<unsigned long>(averageFps),
    static_cast<unsigned long>(currentFps),
    static_cast<unsigned long>(streamSendErrors),
    static_cast<unsigned long>(streamLastFrameBytes),
    static_cast<unsigned long>(averageBytes),
    static_cast<unsigned long>(averageCaptureUs),
    static_cast<unsigned long>(averageSendUs),
    static_cast<unsigned long>(streamLastCaptureUs),
    static_cast<unsigned long>(streamLastSendUs),
    static_cast<unsigned long>(streamLastFrameIntervalUs),
    static_cast<unsigned int>(streamJpegQuality),
    static_cast<unsigned long>(streamLastFrameAtMs));
  if (length < 0 || static_cast<size_t>(length) >= sizeof(json)) {
    httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Status response is too large");
    return ESP_FAIL;
  }
  sendCors(request);
  httpd_resp_set_type(request, "application/json");
  httpd_resp_send(request, json, length);
  return ESP_OK;
}

static void registerRoute(httpd_handle_t server, const char *uri,
                          httpd_method_t method, esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t route = {};
  route.uri = uri;
  route.method = method;
  route.handler = handler;
  httpd_register_uri_handler(server, &route);
}

static void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 16;
  config.stack_size = 8192;
  config.send_wait_timeout = 1;
  config.recv_wait_timeout = 1;
  config.lru_purge_enable = true;

  if (httpd_start(&cameraHttpServer, &config) != ESP_OK) {
    Serial.println("Failed to start HTTP server");
    return;
  }
  registerRoute(cameraHttpServer, "/", HTTP_GET, indexHandler);
  registerRoute(cameraHttpServer, "/api/data", HTTP_GET, dataHandler);
  registerRoute(cameraHttpServer, "/api/gps", HTTP_GET, gpsHandler);
  registerRoute(cameraHttpServer, "/api/status", HTTP_GET, statusHandler);
  registerRoute(cameraHttpServer, "/api/data", HTTP_OPTIONS, optionsHandler);
  registerRoute(cameraHttpServer, "/api/gps", HTTP_OPTIONS, optionsHandler);
  registerRoute(cameraHttpServer, "/api/status", HTTP_OPTIONS, optionsHandler);

  config.server_port = 81;
  config.ctrl_port = 32769;
  if (httpd_start(&cameraStreamServer, &config) == ESP_OK) {
    registerRoute(cameraStreamServer, "/stream", HTTP_GET, streamHandler);
  } else {
    Serial.println("Failed to start camera stream server");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = STREAM_FRAME_SIZE;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = STREAM_JPEG_QUALITY;
  // Keep the sensor capturing while a frame is sent. LATEST discards older
  // buffers so the stream never displays a queued/stale frame.
  config.fb_count = 2;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = STREAM_JPEG_QUALITY;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Keep the no-PSRAM fallback small enough for low latency and DRAM.
      config.frame_size = STREAM_FRAME_SIZE;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x; WiFi-only mode\n", err);
  } else {
    cameraReady = true;
    sensor_t *s = esp_camera_sensor_get();
    streamCameraSensor = s;
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);        // flip it back
      s->set_brightness(s, 1);   // up the brightness just a bit
      s->set_saturation(s, -2);  // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    if (config.pixel_format == PIXFORMAT_JPEG) {
      s->set_framesize(s, STREAM_FRAME_SIZE);
    }
#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
    s->set_vflip(s, 1);
#endif

  // Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
    setupLedFlash();
#endif
  }

  if (!startWiFiAccessPoint()) {
    return;
  }

  startCameraServer();

  // Start GPS UART after camera and Wi-Fi initialization.
  gpsSerial.begin(GPS_PRIMARY_BAUD, SERIAL_8N1, 45, -1);
  gpsBaudStartedAt = millis();
  Serial.println("GPS UART gestart (115200 baud, RX GPIO45)");

  // Initialize thermal sensor and laptop sender. RGB streaming remains intact.
  sauronBegin();
}

void loop() {
  // Thermal capture has priority over GPS parsing and optional dashboard POST.
  // This keeps the local HTTP thermal endpoint current even without a GPS fix.
  sauronLoop();

  // Parse all bytes currently available from the GPS receiver.
  while (gpsSerial.available() > 0) {
    const int c = gpsSerial.read();
    if (c >= 0) {
      gpsDataSeen = true;
      const char gpsCharacter = static_cast<char>(c);
      gpsParseUbxByte(static_cast<uint8_t>(c));
      if (gps.encode(gpsCharacter)) {
        gpsSentenceSeen = true;
      }
      gpsRawWindow[gpsRawBytesPrinted % sizeof(gpsRawWindow)] =
        static_cast<uint8_t>(c);
      if (gpsRawWindowSize < sizeof(gpsRawWindow)) {
        ++gpsRawWindowSize;
      }
      ++gpsRawBytesPrinted;
    }

  }

  if (!gpsSentenceSeen && !gpsUbxDetected &&
      gpsBaudIndex + 1 < sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0]) &&
      millis() - gpsBaudStartedAt >= 8000UL) {
    ++gpsBaudIndex;
    gpsSerial.updateBaudRate(GPS_BAUDS[gpsBaudIndex]);
    gpsBaudStartedAt = millis();
    Serial.printf("Geen geldige GPS-data; probeer %lu baud.\n",
                  static_cast<unsigned long>(GPS_BAUDS[gpsBaudIndex]));
  }

  if (gpsSentenceSeen && !gpsConnectedMessagePrinted) {
    Serial.println("GPS NMEA-data ontvangen.");
    gpsConnectedMessagePrinted = true;
  }

  if (millis() - lastGpsStatusPrint >= 5000UL) {
    lastGpsStatusPrint = millis();
    Serial.printf("GPS status: baud=%lu, bytes=%lu, NMEA=%s, UBX=%s, "
                  "NMEA-tekens=%lu, NMEA-fixes=%lu, UBX-berichten=%lu, "
                  "fix=%s, sats=%d\n",
                  static_cast<unsigned long>(GPS_BAUDS[gpsBaudIndex]),
                  static_cast<unsigned long>(gpsRawBytesPrinted),
                  gpsSentenceSeen ? "yes" : "no",
                  gpsUbxDetected ? "yes" : "no",
                  static_cast<unsigned long>(gps.charsProcessed()),
                  static_cast<unsigned long>(gps.sentencesWithFix()),
                  static_cast<unsigned long>(gpsUbxMessages),
                  gps.location.isValid() || gpsState.hasFix ? "yes" : "no",
                  gps.location.isValid()
                    ? static_cast<int>(gps.satellites.isValid()
                                         ? gps.satellites.value()
                                         : 0)
                    : static_cast<int>(gpsState.satellites));
    Serial.print("GPS raw latest: ");
    const uint32_t firstRawByte =
      gpsRawBytesPrinted >= gpsRawWindowSize
        ? gpsRawBytesPrinted - gpsRawWindowSize
        : 0;
    for (uint8_t i = 0; i < gpsRawWindowSize; ++i) {
      const uint8_t index =
        static_cast<uint8_t>((firstRawByte + i) % sizeof(gpsRawWindow));
      Serial.printf("%02X ", static_cast<unsigned int>(gpsRawWindow[index]));
    }
    Serial.println();
  }

  // Send the latest UBX NAV-PVT values to sauron_poster.h.
  if (gps.location.isValid()) {
    const float lat = static_cast<float>(gps.location.lat());
    const float lon = static_cast<float>(gps.location.lng());
    const float altitude = gps.altitude.isValid()
                             ? static_cast<float>(gps.altitude.meters())
                             : NAN;
    const float heading = gps.course.isValid()
                            ? static_cast<float>(gps.course.deg())
                            : NAN;
    const float speed = gps.speed.isValid()
                          ? static_cast<float>(gps.speed.mps())
                          : NAN;
    const int satellites = gps.satellites.isValid()
                             ? static_cast<int>(gps.satellites.value())
                             : -1;
    const float hdop = gps.hdop.isValid()
                         ? static_cast<float>(gps.hdop.hdop())
                         : NAN;
    sauronSetGps(lat, lon, altitude, heading, speed, 3, satellites, hdop);
  } else if (gpsState.hasFix) {
    sauronSetGps(
      gpsState.latitudeE7 / 10000000.0f,
      gpsState.longitudeE7 / 10000000.0f,
      gpsState.altitudeMm / 1000.0f,
      gpsState.headingE5 / 100000.0f,
      gpsState.groundSpeedMmS / 1000.0f,
      gpsState.fixType,
      gpsState.satellites,
      gpsState.hdopValid ? gpsState.hdopHundredths / 100.0f : NAN);
  } else {
    // Keep status and satellite count visible until a valid position is available.
    sauronSetGps(
      NAN, NAN, NAN, NAN, NAN, 0,
      gps.satellites.isValid() ? static_cast<int>(gps.satellites.value()) : -1,
      gps.hdop.isValid() ? static_cast<float>(gps.hdop.hdop()) : NAN);
  }

  // Avoid a busy loop while keeping GPS parsing responsive.
  delay(1);
}
