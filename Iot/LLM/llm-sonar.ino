#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

// This sketch turns inbound ESP-NOW beacon traffic into an audible sonar cue.
// It tracks the beacon's visibility, smooths RSSI data, and uses that state
// to generate changing buzzer tones that indicate proximity and signal quality.

const char *QA_BEACON_NAME = "ACEBOTT-Beacon-01";
const uint8_t QA_BEACON_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const int BEACON_BUZZER_PIN = 33;
const uint16_t BEACON_SIGNAL_STALE_MS = 1400;
const int BEACON_NOT_FOUND_RSSI = -127;
const int BEACON_MIN_USABLE_RSSI = -96;
const int BEACON_WEAK_RSSI = -92;
const int BEACON_SONAR_NEAR_RSSI = -60;
const unsigned long BEACON_SONAR_MIN_INTERVAL_MS = 80;
const unsigned long BEACON_SONAR_MAX_INTERVAL_MS = 1600;
const unsigned long BEACON_SONAR_MIN_PULSE_MS = 22;
const unsigned long BEACON_SONAR_MAX_PULSE_MS = 80;
const unsigned int BEACON_SONAR_MIN_FREQ_HZ = 700;
const unsigned int BEACON_SONAR_MAX_FREQ_HZ = 3200;
const unsigned long BEACON_FOUND_DOUBLE_CHIRP_GAP_MS = 110;
const float BEACON_SONAR_RSSI_EMA_ALPHA = 0.28f;
const uint8_t BEACON_RSSI_MEDIAN_WINDOW = 5;
const float BEACON_SONAR_PROX_ALPHA_UP = 0.22f;
const float BEACON_SONAR_PROX_ALPHA_DOWN = 0.32f;
const uint8_t BEACON_CADENCE_BUCKETS = 10;
const unsigned long BEACON_CADENCE_BUCKET_DWELL_MS = 650;
const unsigned long BEACON_RATE_WINDOW_MS = 1000;
const float BEACON_RATE_EMA_ALPHA = 0.45f;
const float BEACON_LINK_QUALITY_EMA_ALPHA = 0.30f;
const float BEACON_EXPECTED_PPS = 4.0f;
const int BEACON_SYNTH_RSSI_MIN = -98;
const int BEACON_SYNTH_RSSI_MAX = -62;
const float BEACON_FALLBACK_PROX_ALPHA_UP = 0.35f;
const float BEACON_FALLBACK_PROX_ALPHA_DOWN = 0.75f;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
const bool BEACON_HAS_NATIVE_RSSI_CALLBACK = true;
#else
const bool BEACON_HAS_NATIVE_RSSI_CALLBACK = false;
#endif
const bool BEACON_HARD_MODE_RSSI_ONLY = false;

// Global state for the most recent beacon packet, the filtered RSSI estimates,
// and the current sonar session behavior.
volatile int beaconLatestRssi = BEACON_NOT_FOUND_RSSI;
volatile unsigned long beaconLatestPacketMs = 0;
volatile uint8_t beaconLatestSequence = 0;
volatile uint32_t beaconLatestUptimeMs = 0;
volatile int beaconPromiscRssi = BEACON_NOT_FOUND_RSSI;
volatile unsigned long beaconPromiscMs = 0;
volatile uint8_t beaconPromiscSrcMac[6] = {0, 0, 0, 0, 0, 0};
uint8_t beaconLastSenderMac[6] = {0, 0, 0, 0, 0, 0};
int beaconLastRssi = BEACON_NOT_FOUND_RSSI;
bool beaconVisible = false;
unsigned long beaconLastSeenMs = 0;
bool beaconSonarSessionActive = false;
uint8_t beaconSonarIntroPulses = 0;
unsigned long beaconNextSonarMs = 0;
bool beaconBuzzerToneActive = false;
float beaconSonarRssiEma = (float)BEACON_NOT_FOUND_RSSI;
bool beaconSonarRssiEmaReady = false;
bool beaconUsingNativeRssi = false;
float beaconFallbackProximity = 0.0f;
int beaconRssiWindow[BEACON_RSSI_MEDIAN_WINDOW] = {BEACON_NOT_FOUND_RSSI, BEACON_NOT_FOUND_RSSI, BEACON_NOT_FOUND_RSSI, BEACON_NOT_FOUND_RSSI, BEACON_NOT_FOUND_RSSI};
uint8_t beaconRssiWindowCount = 0;
uint8_t beaconRssiWindowIndex = 0;
float beaconSonarProximityEma = 0.0f;
bool beaconSonarProximityEmaReady = false;
int beaconCadenceBucket = -1;
unsigned long beaconCadenceBucketAtMs = 0;
bool beaconFlowReady = false;
uint8_t beaconFlowPrevSequence = 0;
uint32_t beaconFlowPrevUptimeMs = 0;
unsigned long beaconFlowPrevLocalMs = 0;
float beaconLinkQualityEma = 0.0f;
unsigned long beaconRateWindowStartMs = 0;
uint16_t beaconRateWindowPackets = 0;
float beaconPacketsPerSecondEma = 0.0f;
bool beaconPacketsPerSecondReady = false;
unsigned long beaconFallbackLastUpdateMs = 0;
unsigned long lastStatusPrintMs = 0;
struct EspNowBeaconPacket {
  char name[16];
  uint32_t uptimeMs;
  uint8_t sequence;
};
typedef struct {
  uint16_t frameCtrl;
  uint16_t durationId;
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  uint16_t seqCtrl;
} wifi_ieee80211_mac_hdr_t;
typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0];
} wifi_ieee80211_packet_t;

// Clamp a float into a standard [0, 1] range for proximity and quality calculations.
float clamp01(float x) {
  return constrain(x, 0.0f, 1.0f);
}
// Stop the buzzer safely, but only if a tone is currently active.
void stopBeaconToneSafe() {
  if (!beaconBuzzerToneActive) {
    return;
  }
  noTone(BEACON_BUZZER_PIN);
  beaconBuzzerToneActive = false;
}
// Play a short buzzer tone and mark the tone state so other routines can stop it cleanly.
void playBeaconTone(unsigned int freqHz, unsigned long pulseMs) {
  tone(BEACON_BUZZER_PIN, freqHz, pulseMs);
  beaconBuzzerToneActive = true;
}
// Keep a sliding window of recent RSSI values and return the median value.
// The median helps reduce single-packet spikes and makes the sonar feel more stable.
int medianBeaconRssi(int latestRssi) {
  beaconRssiWindow[beaconRssiWindowIndex] = latestRssi;
  beaconRssiWindowIndex = (beaconRssiWindowIndex + 1) % BEACON_RSSI_MEDIAN_WINDOW;
  if (beaconRssiWindowCount < BEACON_RSSI_MEDIAN_WINDOW) {
    beaconRssiWindowCount++;
  }
  int sorted[BEACON_RSSI_MEDIAN_WINDOW];
  for (uint8_t i = 0; i < beaconRssiWindowCount; ++i) {
    sorted[i] = beaconRssiWindow[i];
  }
  for (uint8_t i = 0; i < beaconRssiWindowCount; ++i) {
    for (uint8_t j = (uint8_t)(i + 1); j < beaconRssiWindowCount; ++j) {
      if (sorted[j] < sorted[i]) {
        int tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }
  return sorted[beaconRssiWindowCount / 2];
}
// When native RSSI is unavailable, synthesize a usable RSSI estimate from packet
// timing, sequence continuity, and observed packet rate. This keeps the beacon
// experience useful even on cores that do not provide a native RSSI callback.
void updateSyntheticBeaconRssi(const EspNowBeaconPacket &packet, unsigned long nowMs) {
  if (beaconRateWindowStartMs == 0) {
    beaconRateWindowStartMs = nowMs;
  }
  beaconRateWindowPackets++;
  unsigned long rateWindowElapsedMs = nowMs - beaconRateWindowStartMs;
  if (rateWindowElapsedMs >= BEACON_RATE_WINDOW_MS) {
    float packetsPerSecond = ((float)beaconRateWindowPackets * 1000.0f) / (float)rateWindowElapsedMs;
    if (!beaconPacketsPerSecondReady) {
      beaconPacketsPerSecondEma = packetsPerSecond;
      beaconPacketsPerSecondReady = true;
    } else {
      beaconPacketsPerSecondEma = (BEACON_RATE_EMA_ALPHA * packetsPerSecond) +
                                  ((1.0f - BEACON_RATE_EMA_ALPHA) * beaconPacketsPerSecondEma);
    }
    beaconRateWindowPackets = 0;
    beaconRateWindowStartMs = nowMs;
  }
  float lossQuality = 1.0f;
  float timingQuality = 1.0f;
  if (beaconFlowReady) {
    uint8_t seqDelta = (uint8_t)((packet.sequence - beaconFlowPrevSequence) & 0xFF);
    if (seqDelta == 0) {
      seqDelta = 1;
    }
    lossQuality = 1.0f / (float)seqDelta;
    uint32_t uptimeDeltaMs = packet.uptimeMs - beaconFlowPrevUptimeMs;
    unsigned long localDeltaMs = nowMs - beaconFlowPrevLocalMs;
    if (uptimeDeltaMs == 0) {
      uptimeDeltaMs = 1;
    }
    if (localDeltaMs == 0) {
      localDeltaMs = 1;
    }
    float larger = (uptimeDeltaMs > localDeltaMs) ? (float)uptimeDeltaMs : (float)localDeltaMs;
    float mismatch = fabsf((float)uptimeDeltaMs - (float)localDeltaMs) / larger;
    timingQuality = 1.0f - clamp01(mismatch);
  }
  float rateQuality = 0.0f;
  if (beaconPacketsPerSecondReady) {
    rateQuality = clamp01(beaconPacketsPerSecondEma / BEACON_EXPECTED_PPS);
  }
  // Keep fallback quality conservative so it doesn't peg near 1.0 in normal 1Hz traffic.
  float instantQuality = (0.25f * lossQuality) + (0.35f * timingQuality) + (0.40f * rateQuality);
  instantQuality = clamp01(instantQuality);
  if (!beaconFlowReady) {
    beaconLinkQualityEma = instantQuality;
    beaconFlowReady = true;
  } else {
    beaconLinkQualityEma = (BEACON_LINK_QUALITY_EMA_ALPHA * instantQuality) +
                           ((1.0f - BEACON_LINK_QUALITY_EMA_ALPHA) * beaconLinkQualityEma);
  }
  beaconFlowPrevSequence = packet.sequence;
  beaconFlowPrevUptimeMs = packet.uptimeMs;
  beaconFlowPrevLocalMs = nowMs;
  float proxTarget = clamp01((beaconLinkQualityEma - 0.35f) / 0.55f);
  if (beaconFallbackLastUpdateMs != 0) {
    unsigned long dtMs = nowMs - beaconFallbackLastUpdateMs;
    float passiveDecay = 0.06f * ((float)dtMs / 1000.0f);
    beaconFallbackProximity = clamp01(beaconFallbackProximity - passiveDecay);
  }
  beaconFallbackLastUpdateMs = nowMs;
  float proxAlpha = (proxTarget >= beaconFallbackProximity) ?
                        BEACON_FALLBACK_PROX_ALPHA_UP :
                        BEACON_FALLBACK_PROX_ALPHA_DOWN;
  beaconFallbackProximity = (proxAlpha * proxTarget) + ((1.0f - proxAlpha) * beaconFallbackProximity);
  beaconFallbackProximity = clamp01(beaconFallbackProximity);
  int synthRssi = (int)roundf((float)BEACON_SYNTH_RSSI_MIN +
                              (beaconLinkQualityEma * (float)(BEACON_SYNTH_RSSI_MAX - BEACON_SYNTH_RSSI_MIN)));
  beaconLatestRssi = synthRssi;
}
// Return true when a specific beacon MAC address has been configured.
// If no MAC is configured, the receiver accepts packets from any sender.
bool beaconMacConfigured() {
  for (uint8_t i = 0; i < 6; ++i) {
    if (QA_BEACON_MAC[i] != 0x00) {
      return true;
    }
  }
  return false;
}
// Compare two MAC addresses for an exact match and avoid dereferencing null pointers.
bool beaconSenderMatchesRuntime(const uint8_t *candidateMac, const uint8_t *expectedMac) {
  if (candidateMac == nullptr || expectedMac == nullptr) {
    return false;
  }
  return memcmp(candidateMac, expectedMac, 6) == 0;
}
// Decide whether an incoming sender should be accepted.
// This allows a fixed target MAC to be used when desired or a permissive mode otherwise.
bool beaconSenderMatches(const uint8_t *macAddr) {
  if (macAddr == nullptr) {
    return false;
  }
  if (!beaconMacConfigured()) {
    return true;
  }
  return memcmp(macAddr, QA_BEACON_MAC, 6) == 0;
}
// Capture packets seen in promiscuous mode so the sketch can inspect sender MACs
// and RSSI even when the main ESP-NOW callback does not expose those details.
void onPromiscuousRx(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (buf == nullptr) {
    return;
  }
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) {
    return;
  }
  const wifi_promiscuous_pkt_t *ppkt = (const wifi_promiscuous_pkt_t *)buf;
  if (ppkt->rx_ctrl.sig_len < sizeof(wifi_ieee80211_mac_hdr_t)) {
    return;
  }
  const wifi_ieee80211_packet_t *ipkt = (const wifi_ieee80211_packet_t *)ppkt->payload;
  const uint8_t *srcMac = ipkt->hdr.addr2;
  if (!beaconSenderMatches(srcMac)) {
    return;
  }
  beaconPromiscRssi = ppkt->rx_ctrl.rssi;
  beaconPromiscMs = millis();
  memcpy((void *)beaconPromiscSrcMac, srcMac, 6);
}
// Verify that the payload looks like the expected beacon format and name.
// This prevents unrelated traffic from being accepted as a target packet.
bool beaconPacketMatches(const uint8_t *data, int dataLen) {
  if (data == nullptr || dataLen < (int)sizeof(EspNowBeaconPacket)) {
    return false;
  }
  EspNowBeaconPacket packet = {};
  memcpy(&packet, data, sizeof(packet));
  packet.name[sizeof(packet.name) - 1] = '\0';
  String packetName(packet.name);
  packetName.trim();
  String expectedName(QA_BEACON_NAME);
  if (packetName.equalsIgnoreCase(expectedName)) {
    return true;
  }
  if (packetName.length() == (sizeof(packet.name) - 1) &&
      expectedName.length() >= packetName.length()) {
    return expectedName.substring(0, packetName.length()).equalsIgnoreCase(packetName);
  }
  return false;
}
// Store the latest beacon information and choose either the native RSSI or the
// synthetic fallback depending on what the platform exposes.
void rememberBeaconPacket(const uint8_t *macAddr, int rssi, const uint8_t *data, int dataLen) {
  if (!beaconSenderMatches(macAddr) || !beaconPacketMatches(data, dataLen)) {
    return;
  }
  EspNowBeaconPacket packet = {};
  memcpy(&packet, data, sizeof(packet));
  unsigned long nowMs = millis();
  int effectiveRssi = rssi;
  unsigned long promiscMs = beaconPromiscMs;
  if (effectiveRssi <= BEACON_NOT_FOUND_RSSI &&
      promiscMs > 0 &&
      (nowMs - promiscMs) <= 250UL &&
      beaconSenderMatchesRuntime((const uint8_t *)beaconPromiscSrcMac, macAddr)) {
    effectiveRssi = beaconPromiscRssi;
  }
  if (effectiveRssi > BEACON_NOT_FOUND_RSSI) {
    beaconUsingNativeRssi = true;
    beaconLatestRssi = effectiveRssi;
  } else {
    updateSyntheticBeaconRssi(packet, nowMs);
  }
  beaconLatestPacketMs = nowMs;
  beaconLatestSequence = packet.sequence;
  beaconLatestUptimeMs = packet.uptimeMs;
  memcpy(beaconLastSenderMac, macAddr, 6);
}
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void onBeaconDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  if (recvInfo == nullptr || recvInfo->src_addr == nullptr) {
    return;
  }
  int rssi = BEACON_NOT_FOUND_RSSI;
  if (recvInfo->rx_ctrl != nullptr) {
    rssi = recvInfo->rx_ctrl->rssi;
  }
  rememberBeaconPacket(recvInfo->src_addr, rssi, data, dataLen);
}
#else
void onBeaconDataRecv(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
  rememberBeaconPacket(macAddr, BEACON_NOT_FOUND_RSSI, data, dataLen);
}
#endif
// Initialize ESP-NOW plus promiscuous monitoring so beacons can be received and inspected.
void initBeaconReceiver() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onBeaconDataRecv);
  wifi_promiscuous_filter_t filter = {};
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(&onPromiscuousRx);
  esp_wifi_set_promiscuous(true);
  Serial.println("ESP-NOW beacon receiver ready");
}
// Convert the current beacon state into a normalized proximity value in the range [0, 1].
// A value near 0 means weak/far, near 1 means strong/near, and negative means unavailable.
float computeBeaconProximityFactor() {
  if (beaconUsingNativeRssi) {
    if (!beaconSonarRssiEmaReady) {
      return -1.0f;
    }
    float rssiForSonar = (0.50f * beaconSonarRssiEma) + (0.50f * (float)beaconLastRssi);
    if (rssiForSonar <= (float)BEACON_MIN_USABLE_RSSI) {
      return -1.0f;
    }
    if (rssiForSonar <= (float)BEACON_WEAK_RSSI) {
      return 0.0f;
    }
    if (rssiForSonar >= (float)BEACON_SONAR_NEAR_RSSI) {
      return 1.0f;
    }
    return (rssiForSonar - (float)BEACON_WEAK_RSSI) /
           (float)(BEACON_SONAR_NEAR_RSSI - BEACON_WEAK_RSSI);
  }
  if (!beaconFlowReady) {
    return -1.0f;
  }
  return clamp01(beaconFallbackProximity);
}
// Main state machine for beacon visibility.
// If packets are fresh and usable, the beacon is considered locked; if they go stale,
// the lock is released and the sonar state is reset.
void updateBeaconTracking(unsigned long nowMs) {
  unsigned long latestPacketMs = beaconLatestPacketMs;
  int latestRssi = beaconLatestRssi;
  bool beaconFresh = (latestPacketMs > 0) && ((nowMs - latestPacketMs) <= BEACON_SIGNAL_STALE_MS);
  if (beaconFresh && latestRssi > BEACON_NOT_FOUND_RSSI) {
    if (beaconUsingNativeRssi) {
      beaconLastRssi = medianBeaconRssi(latestRssi);
    } else {
      beaconLastRssi = latestRssi;
    }
    if (!beaconSonarRssiEmaReady) {
      beaconSonarRssiEma = (float)beaconLastRssi;
      beaconSonarRssiEmaReady = true;
    } else {
      beaconSonarRssiEma = (BEACON_SONAR_RSSI_EMA_ALPHA * (float)beaconLastRssi) +
                           ((1.0f - BEACON_SONAR_RSSI_EMA_ALPHA) * beaconSonarRssiEma);
    }
    bool wasVisible = beaconVisible;
    beaconVisible = true;
    beaconLastSeenMs = latestPacketMs;
    if (!wasVisible) {
      beaconSonarIntroPulses = 2;
      beaconNextSonarMs = nowMs;
      Serial.print("Beacon lock mac=");
      for (uint8_t i = 0; i < 6; ++i) {
        if (beaconLastSenderMac[i] < 16) {
          Serial.print('0');
        }
        Serial.print(beaconLastSenderMac[i], HEX);
        if (i < 5) {
          Serial.print(':');
        }
      }
      Serial.print(" seq=");
      Serial.println((unsigned int)beaconLatestSequence);
    }
    return;
  }
  if (beaconVisible && (nowMs - beaconLastSeenMs) > BEACON_SIGNAL_STALE_MS) {
    beaconVisible = false;
    beaconSonarRssiEmaReady = false;
    beaconSonarSessionActive = false;
    beaconSonarIntroPulses = 0;
    beaconFlowReady = false;
    beaconRateWindowPackets = 0;
    beaconRateWindowStartMs = 0;
    beaconPacketsPerSecondReady = false;
    beaconFallbackLastUpdateMs = 0;
    beaconFallbackProximity = 0.0f;
    beaconRssiWindowCount = 0;
    beaconRssiWindowIndex = 0;
    beaconSonarProximityEmaReady = false;
    beaconCadenceBucket = -1;
    beaconCadenceBucketAtMs = 0;
    stopBeaconToneSafe();
    Serial.println("Beacon lock lost");
  }
}
// Generate the audible sonar cue from the current beacon proximity estimate.
// The sketch uses a short intro chirp when a lock is first acquired, then switches
// to a repeating tone whose interval and pitch change with proximity.
void updateBeaconSonarCue(unsigned long nowMs) {
  if (!beaconVisible || (nowMs - beaconLastSeenMs) > BEACON_SIGNAL_STALE_MS) {
    if (beaconSonarSessionActive) {
      stopBeaconToneSafe();
    }
    beaconSonarSessionActive = false;
    beaconSonarIntroPulses = 0;
    return;
  }
  if (!beaconSonarSessionActive) {
    beaconSonarSessionActive = true;
  }
  // Optional hard mode can enforce real-RSSI-only beeps.
  if (BEACON_HARD_MODE_RSSI_ONLY && !beaconUsingNativeRssi) {
    stopBeaconToneSafe();
    beaconSonarIntroPulses = 0;
    beaconNextSonarMs = nowMs + 300UL;
    return;
  }
  if (beaconSonarIntroPulses > 0) {
    if (nowMs >= beaconNextSonarMs) {
      unsigned int introFreq = (beaconSonarIntroPulses == 2) ? 1500U : 2100U;
      playBeaconTone(introFreq, 70);
      beaconSonarIntroPulses--;
      beaconNextSonarMs = nowMs + BEACON_FOUND_DOUBLE_CHIRP_GAP_MS;
    }
    return;
  }
  if (nowMs < beaconNextSonarMs) {
    return;
  }
  float proximity = computeBeaconProximityFactor();
  if (proximity < 0.0f) {
    stopBeaconToneSafe();
    beaconNextSonarMs = nowMs + 280UL;
    return;
  }
  float pRaw = constrain(proximity, 0.0f, 1.0f);
  if (!beaconSonarProximityEmaReady) {
    beaconSonarProximityEma = pRaw;
    beaconSonarProximityEmaReady = true;
  } else {
    float proxAlpha = (pRaw >= beaconSonarProximityEma) ?
                          BEACON_SONAR_PROX_ALPHA_UP :
                          BEACON_SONAR_PROX_ALPHA_DOWN;
    beaconSonarProximityEma = (proxAlpha * pRaw) + ((1.0f - proxAlpha) * beaconSonarProximityEma);
  }
  int targetBucket = (int)roundf(beaconSonarProximityEma * (float)BEACON_CADENCE_BUCKETS);
  if (beaconCadenceBucket < 0) {
    beaconCadenceBucket = targetBucket;
    beaconCadenceBucketAtMs = nowMs;
  } else if (targetBucket != beaconCadenceBucket) {
    int bucketDelta = targetBucket - beaconCadenceBucket;
    if (bucketDelta < 0) {
      bucketDelta = -bucketDelta;
    }
    bool dwellExpired = (nowMs - beaconCadenceBucketAtMs) >= BEACON_CADENCE_BUCKET_DWELL_MS;
    if (bucketDelta >= 2 || dwellExpired) {
      beaconCadenceBucket = targetBucket;
      beaconCadenceBucketAtMs = nowMs;
    }
  }
  float stagedP = (float)beaconCadenceBucket / (float)BEACON_CADENCE_BUCKETS;
  float cadenceGain = powf(stagedP, 1.15f);
  float timbreGain = powf(stagedP, 1.0f);
  unsigned long intervalMs = (unsigned long)(BEACON_SONAR_MAX_INTERVAL_MS - (cadenceGain * (float)(BEACON_SONAR_MAX_INTERVAL_MS - BEACON_SONAR_MIN_INTERVAL_MS)));
  unsigned long pulseMs = (unsigned long)(BEACON_SONAR_MAX_PULSE_MS - (cadenceGain * (float)(BEACON_SONAR_MAX_PULSE_MS - BEACON_SONAR_MIN_PULSE_MS)));
  unsigned int freqHz = (unsigned int)(BEACON_SONAR_MIN_FREQ_HZ + (timbreGain * (float)(BEACON_SONAR_MAX_FREQ_HZ - BEACON_SONAR_MIN_FREQ_HZ)));
  playBeaconTone(freqHz, pulseMs);
  beaconNextSonarMs = nowMs + intervalMs;
}
// Print a compact status line so the device can be tested and tuned over Serial.
void printStatus(unsigned long nowMs) {
  if ((nowMs - lastStatusPrintMs) < 1000UL) {
    return;
  }
  lastStatusPrintMs = nowMs;
  if (!beaconVisible) {
    Serial.println("lock=NO rssi=-127 prox=-1 seq=0");
    return;
  }
  float prox = computeBeaconProximityFactor();
  Serial.print("lock=YES rssi=");
  Serial.print(beaconLastRssi);
  Serial.print(" src=");
  Serial.print(beaconUsingNativeRssi ? "RSSI" : "SYNTH");
  Serial.print(" beep=");
  if (BEACON_HARD_MODE_RSSI_ONLY && !beaconUsingNativeRssi) {
    Serial.print("MUTED");
  } else {
    Serial.print("ON");
  }
  Serial.print(" prox=");
  Serial.print(prox, 2);
  Serial.print(" seq=");
  Serial.println((unsigned int)beaconLatestSequence);
}
// Setup runs once at boot to configure the serial console, buzzer pin, and receiver.
void setup() {
  Serial.begin(115200);
  pinMode(BEACON_BUZZER_PIN, OUTPUT);
  initBeaconReceiver();
  Serial.print("Beacon sonar test target name: ");
  Serial.println(QA_BEACON_NAME);
  if (!BEACON_HAS_NATIVE_RSSI_CALLBACK) {
    if (BEACON_HARD_MODE_RSSI_ONLY) {
      Serial.println("Note: native RX RSSI unavailable on this Arduino core; hard mode mutes buzzer for SYNTH data");
    } else {
      Serial.println("Note: native RX RSSI unavailable on this Arduino core; using SYNTH proximity for audible homing");
    }
  }
  Serial.println("Manual test mode: no drive, no AI, sonar beep only");
}
// The main loop updates beacon state, generates the sonar cue, and reports status.
void loop() {
  unsigned long nowMs = millis();
  updateBeaconTracking(nowMs);
  updateBeaconSonarCue(nowMs);
  printStatus(nowMs);
  delay(20);
}
