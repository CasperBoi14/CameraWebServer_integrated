
#pragma once

// sauron_poster.h
// MLX90640 thermische camera + GPS-data naar het dashboard.
// Geschikt als toevoeging aan de ESP32-S3 CameraWebServer.
//
// De MLX90640 Melexis API en ESP32 I2C-driver staan naast dit bestand.
//
// Aansluitingen MLX90640:
//   VIN -> 3V3
//   GND -> GND
//   SDA -> GPIO47
//   SCL -> GPIO48
//
// GPS wordt vanuit CameraWebServer_integrated.ino aangeleverd via:
//   sauronSetGps(...)
//
// Gebruik:
//   setup(): sauronBegin();
//   loop():  sauronLoop();

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <Adafruit_MLX90640.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

// ============================================================
// CONFIGURATIE
// ============================================================

#ifndef SAURON_SDA
#define SAURON_SDA 47
#endif

#ifndef SAURON_SCL
#define SAURON_SCL 48
#endif

#define SAURON_I2C_ADDR 0x33

// Lees de sensor op de ingestelde refresh rate; verzend hoogstens eenmaal
// per seconde naar het optionele externe dashboard.
#define SAURON_READ_INTERVAL_MS 62UL
#define SAURON_POST_INTERVAL_MS 1000UL

// UDP-poort voor ontdekking van de laptop.
#define SAURON_UDP_POORT 8001

// 768 pixels, plus GPS-gegevens en JSON-overhead.
#define SAURON_BUF (768 * 12 + 2048)

// Oriëntatie aanpassen als het thermische beeld verkeerd staat.
#define SAURON_TRANSPOSE 0
#define SAURON_MIRROR_X 1
#define SAURON_MIRROR_Y 1

// ============================================================
// STATUS EN DATA
// ============================================================

static Adafruit_MLX90640 sauronMlx;
static WiFiUDP sauronUdp;

static float sauronRuw[768];
static float sauronFrame[768];

static char sauronBodies[2][SAURON_BUF];
static char *sauronBuildBody = sauronBodies[0];
static char sauronPostBody[SAURON_BUF];
static volatile uint8_t sauronActiveBody = 0;
static SemaphoreHandle_t sauronBodyMutex = nullptr;

static bool sauronLive = false;
static bool sauronFrameReady = false;
static bool sauronInitAttempted = false;
static uint32_t sauronLastInitAttempt = 0;
static uint32_t sauronFrameErrors = 0;
static uint32_t sauronConsecutiveFrameErrors = 0;
static const char *sauronError = "not_initialized";
static uint32_t sauronLaatsteUitlezing = 0;
static uint32_t sauronFrameAtMs = 0;
static uint32_t sauronLaatsteVerzending = 0;
static String sauronDoel;
static TaskHandle_t sauronPostTaskHandle = nullptr;

// GPS-gegevens
static float sauronLat = NAN;
static float sauronLon = NAN;
static float sauronAltMsl = NAN;
static float sauronHeading = NAN;
static float sauronSpeedMps = NAN;
static float sauronHdop = NAN;

static int sauronFix = -1;
static int sauronSats = -1;
static uint32_t sauronGpsAtMs = 0;

static float sauronMinTemp = NAN;
static float sauronMaxTemp = NAN;
static int sauronHotspotX = -1;
static int sauronHotspotY = -1;

static bool sauronGpsIsFresh(uint32_t maxAgeMs);
static void sauronMarkGpsFresh();
static void sauronPostTask(void *);

// ============================================================
// VEILIG JSON OPBOUWEN
// ============================================================

static bool sauronAppend(size_t &used, const char *format, ...) {
  if (used >= SAURON_BUF) {
    return false;
  }

  va_list args;
  va_start(args, format);

  const size_t remaining = SAURON_BUF - used;

  int written = vsnprintf(
    sauronBuildBody + used,
    remaining,
    format,
    args
  );

  va_end(args);

  if (written < 0 || (size_t)written >= remaining) {
    return false;
  }

  used += (size_t)written;
  return true;
}

// ============================================================
// LAPTOP ONTDEKKEN VIA UDP
// ============================================================

static bool sauronVindDoel() {
  uint8_t buf[256];

  IPAddress lokaal =
    (WiFi.getMode() & WIFI_MODE_AP)
      ? WiFi.softAPIP()
      : WiFi.localIP();

  IPAddress bcast(
    lokaal[0],
    lokaal[1],
    lokaal[2],
    255
  );

  for (uint8_t poging = 0; poging < 3; poging++) {

    // Broadcast op het huidige lokale subnet.
    sauronUdp.beginPacket(bcast, SAURON_UDP_POORT);
    sauronUdp.write(
      (const uint8_t *)"SAURON?",
      7
    );
    sauronUdp.endPacket();

    // Ook een algemene broadcast proberen bij verbinding
    // met een bestaand WiFi-netwerk.
    if (WiFi.status() == WL_CONNECTED) {
      sauronUdp.beginPacket(
        IPAddress(255, 255, 255, 255),
        SAURON_UDP_POORT
      );

      sauronUdp.write(
        (const uint8_t *)"SAURON?",
        7
      );

      sauronUdp.endPacket();
    }

    uint32_t start = millis();

    while (millis() - start < 300) {
      int packetSize = sauronUdp.parsePacket();

      if (packetSize > 0) {
        int len = sauronUdp.read(
          buf,
          sizeof(buf) - 1
        );

        if (len > 0) {
          buf[len] = '\0';

          String antwoord = String((char *)buf);
          antwoord.trim();

          if (antwoord.startsWith("http://") ||
              antwoord.startsWith("https://")) {

            sauronDoel = antwoord;

            Serial.printf(
              "[sauron] Dashboard gevonden: %s\n",
              sauronDoel.c_str()
            );

            return true;
          }
        }
      }

      delay(5);
    }
  }

  Serial.println(
    "[sauron] Dashboard niet gevonden via UDP."
  );

  return false;
}

// ============================================================
// THERMISCH FRAME OMBOUWEN
// ============================================================

static void sauronOmbouwen() {
  for (int rij = 0; rij < 24; rij++) {
    for (int kolom = 0; kolom < 32; kolom++) {

      int bron;

#if SAURON_TRANSPOSE
      bron = kolom * 24 + rij;
#else
      bron = rij * 32 + kolom;
#endif

      const int doelRij =
#if SAURON_MIRROR_Y
        23 - rij;
#else
        rij;
#endif
      const int doelKolom =
#if SAURON_MIRROR_X
        31 - kolom;
#else
        kolom;
#endif
      int doel = doelRij * 32 + doelKolom;

      if (bron >= 0 && bron < 768 &&
          doel >= 0 && doel < 768) {
        sauronFrame[doel] = sauronRuw[bron];
      }
    }
  }
}

// ============================================================
// JSON MET THERMISCHE DATA EN GPS
// ============================================================

static bool sauronBouwBody() {
  size_t n = 0;
  sauronMinTemp = INFINITY;
  sauronMaxTemp = -INFINITY;
  sauronHotspotX = -1;
  sauronHotspotY = -1;
  uint16_t geldigePixels = 0;

  for (int i = 0; i < 768; i++) {
    if (!isnan(sauronFrame[i]) && !isinf(sauronFrame[i])) {
      geldigePixels++;
      if (sauronFrame[i] < sauronMinTemp) {
        sauronMinTemp = sauronFrame[i];
      }
      if (sauronFrame[i] > sauronMaxTemp) {
        sauronMaxTemp = sauronFrame[i];
        sauronHotspotX = i % 32;
        sauronHotspotY = i / 32;
      }
    }
  }

  if (geldigePixels == 0) {
    sauronError = "mlx90640_invalid_frame";
    return false;
  }

  if (!sauronAppend(n, "{\"thermal_available\":true,\"frame\":[")) {
    return false;
  }

  for (int i = 0; i < 768; i++) {
    if (i > 0) {
      if (!sauronAppend(n, ",")) {
        return false;
      }
    }

    // JSON ondersteunt geen NaN of Infinity.
    if (isnan(sauronFrame[i]) ||
        isinf(sauronFrame[i])) {

      if (!sauronAppend(n, "null")) {
        return false;
      }

    } else {
      if (!sauronAppend(
            n,
            "%.1f",
            (double)sauronFrame[i])) {
        return false;
      }
    }
  }

  if (!sauronAppend(n, "]")) {
    return false;
  }

  const bool fireDetected = sauronMaxTemp >= 300.0f;
  if (!sauronAppend(n, ",\"thermal\":{\"min_c\":%.1f,\"max_c\":%.1f,"
                       "\"fire\":%s,\"hotspot\":{\"x\":%d,\"y\":%d}}",
                    (double)sauronMinTemp, (double)sauronMaxTemp,
                    fireDetected ? "true" : "false",
                    sauronHotspotX, sauronHotspotY)) {
    return false;
  }

  // GPS-object alleen opnemen bij geldige coördinaten.
  if (!isnan(sauronLat) &&
      !isnan(sauronLon) &&
      !isinf(sauronLat) &&
      !isinf(sauronLon)) {

    if (!sauronAppend(
          n,
          ",\"drone\":{\"lat\":%.6f,\"lon\":%.6f",
          (double)sauronLat,
          (double)sauronLon)) {
      return false;
    }

    if (!isnan(sauronAltMsl) &&
        !isinf(sauronAltMsl)) {

      if (!sauronAppend(
            n,
            ",\"alt_msl\":%.1f",
            (double)sauronAltMsl)) {
        return false;
      }
    }

    if (!isnan(sauronHeading) &&
        !isinf(sauronHeading)) {

      if (!sauronAppend(
            n,
            ",\"heading_deg\":%.1f",
            (double)sauronHeading)) {
        return false;
      }
    }

    if (!isnan(sauronSpeedMps) &&
        !isinf(sauronSpeedMps)) {
      if (!sauronAppend(n, ",\"speed_mps\":%.1f",
                        (double)sauronSpeedMps)) {
        return false;
      }
    }

    if (sauronFix >= 0) {
      if (!sauronAppend(
            n,
            ",\"fix\":%d",
            sauronFix)) {
        return false;
      }
    }

    if (sauronSats >= 0) {
      if (!sauronAppend(
            n,
            ",\"sats\":%d",
            sauronSats)) {
        return false;
      }
    }

    if (!isnan(sauronHdop) &&
        !isinf(sauronHdop)) {

      if (!sauronAppend(
            n,
            ",\"hdop\":%.1f",
            (double)sauronHdop)) {
        return false;
      }
    }

    if (!sauronAppend(n, "}")) {
      return false;
    }
  }

  if (!sauronAppend(n, "}")) {
    return false;
  }

  return true;
}

// ============================================================
// INITIALISATIE
// ============================================================

static void sauronBegin() {
  if (sauronLive) {
    return;
  }

  sauronInitAttempted = true;
  sauronLastInitAttempt = millis();
  Serial.printf(
    "[sauron] MLX90640 initialiseren (SDA=%d, SCL=%d)...\n",
    SAURON_SDA,
    SAURON_SCL
  );

  // De bestaande camera blijft op zijn eigen pinconfiguratie.
  pinMode(SAURON_SDA, INPUT_PULLUP);
  pinMode(SAURON_SCL, INPUT_PULLUP);
  Wire.begin(SAURON_SDA, SAURON_SCL);
  // Start conservatively so the initial probe also works with longer wires or
  // weaker pull-ups. Increase only after the device acknowledges.
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  uint8_t foundDevices = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      if (foundDevices == 0) {
        Serial.print("[sauron] I2C apparaten gevonden op: ");
      } else {
        Serial.print(", ");
      }
      Serial.printf("0x%02X", address);
      ++foundDevices;
    }
  }
  if (foundDevices > 0) {
    Serial.println();
  } else {
    Serial.println("[sauron] Geen I2C-apparaten gevonden op deze pinnen.");
  }

  Wire.beginTransmission(SAURON_I2C_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(
      "[sauron] MLX90640 NIET gevonden."
    );

    Serial.println(
      "[sauron] Controleer 3V3, GND, SDA GPIO47 en SCL GPIO48."
    );

    Serial.println(
      "[sauron] CameraWebServer blijft beschikbaar."
    );

    sauronError = "mlx90640_not_found";
    return;
  }

  Wire.setClock(400000);

  if (!sauronMlx.begin(SAURON_I2C_ADDR, &Wire)) {
    Serial.println("[sauron] Adafruit MLX90640 initialisatie mislukt.");
    sauronError = "mlx90640_initialization_error";
    return;
  }

  sauronMlx.setRefreshRate(MLX90640_16_HZ);
  sauronMlx.setMode(MLX90640_CHESS);

  if (sauronMlx.getRefreshRate() != MLX90640_16_HZ ||
      sauronMlx.getMode() != MLX90640_CHESS) {
    Serial.printf("[sauron] Refresh rate instellen mislukt (status %d).\n",
                  -1);
    sauronError = "mlx90640_configuration_error";
    return;
  }

  // UDP-listener starten voor antwoorden van de laptop.
  if (!sauronUdp.begin(SAURON_UDP_POORT)) {
    Serial.println(
      "[sauron] Waarschuwing: UDP-poort kon niet worden gestart."
    );
  }

  sauronLive = true;
  sauronError = "ok";

  Serial.println(
    "[sauron] MLX90640 klaar."
  );

  sauronBodyMutex = xSemaphoreCreateMutex();
  if (sauronBodyMutex == nullptr) {
    Serial.println("[sauron] JSON-buffer mutex kon niet worden aangemaakt.");
    sauronLive = false;
    sauronError = "json_mutex_error";
    return;
  }

  Serial.println(
    "[sauron] Sensor: 32 x 24 pixels, refresh rate 16 Hz."
  );

  Serial.println(
    "[sauron] Dashboard-verzending: maximaal 1 frame/s."
  );

  xTaskCreatePinnedToCore(
    sauronPostTask, "sauron-post", 8192, nullptr, 1,
    &sauronPostTaskHandle, 0);
}

// ============================================================
// HOOFDLOOP
// ============================================================

static void sauronLoop() {
  if (!sauronLive) {
    if (!sauronInitAttempted ||
        millis() - sauronLastInitAttempt >= 5000UL) {
      sauronBegin();
    }
    return;
  }

  uint32_t nu = millis();

  if (nu - sauronLaatsteUitlezing < SAURON_READ_INTERVAL_MS) {
    return;
  }

  sauronLaatsteUitlezing = nu;

  // Lees op de sensor-refresh rate zodat /api/data steeds een recent frame
  // bevat, ook wanneer het externe dashboard tijdelijk niet bereikbaar is.
  int status = sauronMlx.getFrame(sauronRuw);
  if (status < 0) {
    sauronFrameErrors++;
    sauronConsecutiveFrameErrors++;
    sauronError = "mlx90640_frame_error";
    if (sauronConsecutiveFrameErrors == 1 ||
        sauronConsecutiveFrameErrors % 20 == 0) {
      Serial.printf("[sauron] MLX90640 frame error status=%d (%lu achter elkaar).\n",
                    status,
                    static_cast<unsigned long>(sauronConsecutiveFrameErrors));
    }
    return;
  }
  sauronConsecutiveFrameErrors = 0;

  sauronOmbouwen();

  const uint8_t nextBody = sauronActiveBody == 0 ? 1 : 0;
  sauronBuildBody = sauronBodies[nextBody];
  if (xSemaphoreTake(sauronBodyMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    sauronError = "json_buffer_busy";
    return;
  }
  if (!sauronBouwBody()) {
    xSemaphoreGive(sauronBodyMutex);
    Serial.println(
      "[sauron] JSON-buffer onvoldoende groot; frame overgeslagen."
    );

    return;
  }

  sauronActiveBody = nextBody;
  xSemaphoreGive(sauronBodyMutex);

  sauronFrameReady = true;
  sauronFrameAtMs = nu;

}

static void sauronPostLoop() {
  if (!sauronLive ||
      millis() - sauronLaatsteVerzending < SAURON_POST_INTERVAL_MS) {
    return;
  }
  sauronLaatsteVerzending = millis();

  if (sauronDoel.length() == 0 && !sauronVindDoel()) {
    return;
  }

  size_t postLength = 0;
  if (xSemaphoreTake(sauronBodyMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    const char *activeBody = sauronBodies[sauronActiveBody];
    postLength = strnlen(activeBody, SAURON_BUF - 1);
    memcpy(sauronPostBody, activeBody, postLength);
    sauronPostBody[postLength] = '\0';
    xSemaphoreGive(sauronBodyMutex);
  }
  if (postLength == 0) {
    return;
  }

  HTTPClient http;
  http.setTimeout(1000);
  if (!http.begin(sauronDoel)) {
    sauronDoel = "";
    return;
  }
  http.addHeader("Content-Type", "application/json");
  const int code = http.POST((uint8_t *)sauronPostBody, postLength);
  http.end();
  if (code < 200 || code >= 300) {
    sauronDoel = "";
  }
}

static void sauronPostTask(void *) {
  for (;;) {
    sauronPostLoop();
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

// ============================================================
// GPS-API
// ============================================================

static void sauronSetGps(
  float lat,
  float lon,
  float altMsl,
  float heading,
  float speedMps,
  int fix,
  int sats,
  float hdop
) {
  sauronLat = lat;
  sauronLon = lon;
  sauronAltMsl = altMsl;
  sauronHeading = heading;
  sauronSpeedMps = speedMps;
  sauronFix = fix;
  sauronSats = sats;
  sauronHdop = hdop;
  if (isfinite(lat) && isfinite(lon)) {
    sauronMarkGpsFresh();
  }
}

static bool sauronHeeftFrame() {
  return sauronFrameReady;
}

static const char *sauronLaatsteJson() {
  if (sauronFrameReady) {
    return sauronBodies[sauronActiveBody];
  }

  static const char unavailable[] =
    "{\"frame\":null,\"thermal_available\":false}";
  return unavailable;
}

static bool sauronFrameIsFresh(uint32_t maxAgeMs) {
  return sauronFrameReady &&
         sauronFrameAtMs != 0 &&
         millis() - sauronFrameAtMs <= maxAgeMs;
}

static const char *sauronLaatsteGpsJson(bool dataSeen) {
  static char json[320];
  const bool fresh = dataSeen && sauronGpsIsFresh(2000UL);
  const bool locationValid = fresh &&
    isfinite(sauronLat) && isfinite(sauronLon);
  const int length = snprintf(
    json,
    sizeof(json),
    "{\"data_seen\":%s,\"location_valid\":%s,\"lat\":%s,\"lon\":%s,"
    "\"alt_msl\":%s,\"heading_deg\":%s,\"speed_mps\":%s,"
    "\"fix\":%d,\"sats\":%d,\"hdop\":%s}",
    dataSeen ? "true" : "false",
    locationValid ? "true" : "false",
    fresh && isfinite(sauronLat) ? String(sauronLat, 6).c_str() : "null",
    fresh && isfinite(sauronLon) ? String(sauronLon, 6).c_str() : "null",
    fresh && isfinite(sauronAltMsl) ? String(sauronAltMsl, 1).c_str() : "null",
    fresh && isfinite(sauronHeading) ? String(sauronHeading, 1).c_str() : "null",
    fresh && isfinite(sauronSpeedMps) ? String(sauronSpeedMps, 1).c_str() : "null",
    fresh ? sauronFix : -1,
    fresh ? sauronSats : -1,
    fresh && isfinite(sauronHdop) ? String(sauronHdop, 1).c_str() : "null");

  if (length < 0 || static_cast<size_t>(length) >= sizeof(json)) {
    return nullptr;
  }
  return json;
}

static const char *sauronStatus() {
  return sauronError;
}

static void sauronMarkGpsFresh() {
  sauronGpsAtMs = millis();
}

static bool sauronGpsIsFresh(uint32_t maxAgeMs) {
  return sauronGpsAtMs != 0 &&
         millis() - sauronGpsAtMs <= maxAgeMs;
}

static uint32_t sauronFrameErrorCount() {
  return sauronFrameErrors;
}
