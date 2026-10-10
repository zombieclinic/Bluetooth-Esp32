


#include <Arduino.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <FastLED.h>
#include <math.h>

// ----------------------- CONFIG ---------------------------

#define NUM_LEDS           60
#define LED_PIN            27
#define BUTTON_PIN         33
#define VOLUME_PIN         34

#define I2S_BCLK_PIN       26
#define I2S_WS_PIN         25
#define I2S_DATA_PIN       22

#define MAX_BT_VOLUME      114
#define LED_BRIGHTNESS     76
#define LED_POWER_MA       600

#define MODE_COUNT         12
#define LONG_PRESS_MS      1500
#define DEBOUNCE_MS        35
#define LED_FRAME_MS       40

const char *BLUETOOTH_NAME = "Bowies Speaker";

// Optional music sensitivity tuning

const float BASS_GAIN = 13.0f;
const float MID_GAIN = 15.0f;
const float TREBLE_GAIN = 20.0f;
const float LEVEL_GAIN = 10.0f;

// ----------------------- OBJECTS --------------------------

I2SStream i2s;
BluetoothA2DPSink a2dpSink(i2s);

CRGB leds[NUM_LEDS];

// ------------------ AUDIO SHARED STATE --------------------

portMUX_TYPE audioLock = portMUX_INITIALIZER_UNLOCKED;

float sharedBass = 0.0f;
float sharedMid = 0.0f;
float sharedHigh = 0.0f;
float sharedTotal = 0.0f;

uint32_t sharedAudioAt = 0;

// Filter history touched only by A2DP callback

float filterLow = 0.0f;
float filterBroad = 0.0f;

// Main loop analysis values

float bass = 0.0f;
float mids = 0.0f;
float highs = 0.0f;
float level = 0.0f;

float rollingBass = 0.03f;
float beatGlow = 0.0f;

bool beatNow = false;
bool musicActive = false;

uint32_t lastBeatAt = 0;

// ----------------------- LED STATE ------------------------

bool lightsOn = true;

uint8_t currentMode = 0;
uint8_t baseHue = 0;

uint32_t animationFrame = 0;
uint32_t lastFrameAt = 0;

uint8_t fireHeat[NUM_LEDS] = {0};

const char *modeNames[MODE_COUNT] = {
  "Bass Bloom",
  "Rainbow Equalizer",
  "Treble Sparkles",
  "Dancing Rainbow",
  "Bass Lightning",
  "Red Blue Heartbeat",
  "Three-Band Meter",
  "Music Comet",
  "Beat Explosion",
  "Disco Confetti",
  "Ocean Waves",
  "Fire and Embers"
};

// --------------------- BUTTON STATE -----------------------

bool previousRawButton = HIGH;
bool stableButton = HIGH;
bool longPressUsed = false;

uint32_t rawButtonChangedAt = 0;
uint32_t buttonPressedAt = 0;

// --------------------- VOLUME STATE -----------------------

float filteredKnob = 0.0f;
int lastKnobVolume = -1;

uint32_t lastKnobAt = 0;

// ------------------------ HELPERS -------------------------

float limit01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

uint8_t power8(float value) {
  return (uint8_t)(255.0f * limit01(value));
}

void whiteSparkle(uint8_t chance) {
  if (random8() < chance) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

void clearModeState() {
  FastLED.clear();
  animationFrame = 0;
  memset(fireHeat, 0, sizeof(fireHeat));
}

// -------------------- BLUETOOTH PCM TAP -------------------

/*
  The ESP32-A2DP library invokes this on decoded
  interleaved 16-bit stereo PCM.

  With set_stream_reader(callback, true),
  the same audio is ALSO forwarded to I2S output.

  Never call FastLED.show() inside this callback.
*/

void onBluetoothAudio(const uint8_t *data, uint32_t len) {
  if (!data || len < 4) return;

  const int16_t *samples = (const int16_t *)data;

  uint32_t stereoFrames = len / 4;

  float sumLow = 0.0f;
  float sumMid = 0.0f;
  float sumHigh = 0.0f;
  float sumAll = 0.0f;

  uint32_t count = 0;

  // Every 2nd stereo frame
  // Effective sample rate approximately 22 kHz

  for (uint32_t f = 0; f < stereoFrames; f += 2) {
    int32_t l = samples[f * 2];
    int32_t r = samples[f * 2 + 1];

    float x = ((float)(l + r)) * (1.0f / 65536.0f);

    // Approximate bass and frequency separation

    filterLow += 0.042f * (x - filterLow);
    filterBroad += 0.39f * (x - filterBroad);

    float low = filterLow;
    float middle = filterBroad - filterLow;
    float high = x - filterBroad;

    sumLow += low * low;
    sumMid += middle * middle;
    sumHigh += high * high;
    sumAll += x * x;

    count++;
  }

  if (!count) return;

  float b = sqrtf(sumLow / count);
  float m = sqrtf(sumMid / count);
  float h = sqrtf(sumHigh / count);
  float all = sqrtf(sumAll / count);

  portENTER_CRITICAL(&audioLock);

  sharedBass = b;
  sharedMid = m;
  sharedHigh = h;
  sharedTotal = all;
  sharedAudioAt = millis();

  portEXIT_CRITICAL(&audioLock);
}

// ---------------------- BEAT TRACKING ---------------------

void updateMusicLevels() {
  float rawBass, rawMid, rawHigh, rawTotal;
  uint32_t latest;

  portENTER_CRITICAL(&audioLock);

  rawBass = sharedBass;
  rawMid = sharedMid;
  rawHigh = sharedHigh;
  rawTotal = sharedTotal;
  latest = sharedAudioAt;
  // Gentle blue standby when music stops
  float targetBass = limit01(rawBass * BASS_GAIN);
  float targetMid = limit01(rawMid * MID_GAIN);
  float targetHigh = limit01(rawHigh * TREBLE_GAIN);
  float targetLevel = limit01(rawTotal * LEVEL_GAIN);

  bass += (targetBass - bass) *
          (targetBass > bass ? 0.62f : 0.18f);

  mids += (targetMid - mids) *
          (targetMid > mids ? 0.60f : 0.20f);

  highs += (targetHigh - highs) *
           (targetHigh > highs ? 0.65f : 0.24f);

  level += (targetLevel - level) *
           (targetLevel > level ? 0.64f : 0.18f);

  // Detect sudden bass transients

  beatNow =
    musicActive &&
    (bass > 0.11f) &&
    (bass > rollingBass * 1.40f) &&
    (millis() - lastBeatAt > 170);

  if (beatNow) {
    beatGlow = bass;
    lastBeatAt = millis();
  }

  beatGlow *= 0.84f;

  rollingBass =
    rollingBass * 0.965f +
    bass * 0.035f;
}

// -------------------------- BUTTON -----------------------

void updateButton() {
  uint32_t now = millis();
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != previousRawButton) {
    rawButtonChangedAt = now;
  }

  if (now - rawButtonChangedAt >= DEBOUNCE_MS &&
      reading != stableButton) {

    stableButton = reading;

    if (stableButton == LOW) {
      buttonPressedAt = now;
      longPressUsed = false;
    }
    else if (!longPressUsed && lightsOn) {

      currentMode = (currentMode + 1) % MODE_COUNT;

      clearModeState();

      Serial.print("LED mode ");
      Serial.print(currentMode + 1);
      Serial.print("/12: ");
      Serial.println(modeNames[currentMode]);
    }
  }

  if (stableButton == LOW &&
      !longPressUsed &&
      now - buttonPressedAt >= LONG_PRESS_MS) {

    longPressUsed = true;
    lightsOn = !lightsOn;

    if (!lightsOn) {
      FastLED.clear(true);
      Serial.println("LEDs OFF");
    }
    else {
      clearModeState();

      Serial.print("LEDs ON - ");
      Serial.println(modeNames[currentMode]);
    }
  }

  previousRawButton = reading;
}

// ----------------------- VOLUME KNOB ---------------------

void updateVolume() {
  uint32_t now = millis();

  if (now - lastKnobAt < 65) return;

  lastKnobAt = now;

  filteredKnob +=
    0.18f * (analogRead(VOLUME_PIN) - filteredKnob);

  int desired =
    map(constrain((int)filteredKnob, 0, 4095),
        0, 4095, 0, MAX_BT_VOLUME);

  if (lastKnobVolume < 0 ||
      abs(desired - lastKnobVolume) >= 2) {

    a2dpSink.set_volume((uint8_t)desired);
    lastKnobVolume = desired;
  }

  // Prevent Bluetooth volume exceeding software limit

  if (a2dpSink.get_volume() > MAX_BT_VOLUME) {
    a2dpSink.set_volume(MAX_BT_VOLUME);
  }
}

// ========================= LED MODES ======================

// 1 - BASS BLOOM

void bassBloom() {
  fadeToBlackBy(leds, NUM_LEDS, 90);

  int center = NUM_LEDS / 2;
  int radius = (int)(bass * center);

  for (int d = 0; d <= radius; d++) {
    CRGB color = CHSV(
      baseHue + d * 5,
      255,
      power8(bass * (1.0f - d * 0.025f))
    );

    if (center - d >= 0)
      leds[center - d] = color;

    if (center + d < NUM_LEDS)
      leds[center + d] = color;
  }

  if (beatNow) {
    for (int i = 0; i < 5; i++)
      whiteSparkle(255);
  }
}

// 2 - RAINBOW EQUALIZER

void rainbowEqualizer() {
  fadeToBlackBy(leds, NUM_LEDS, 95);

  float values[6] = {
    bass,
    bass * 0.7f,
    mids,
    mids * 0.8f,
    highs,
    highs * 0.7f
  };

  for (int band = 0; band < 6; band++) {
    int start = band * NUM_LEDS / 6;
    int end = (band + 1) * NUM_LEDS / 6;

    int lit =
      (int)(values[band] * (end - start) + 0.5f);

    for (int i = 0; i < lit && start + i < end; i++) {
      leds[start + i] =
        CHSV(baseHue + band * 36, 255, 255);
    }
  }
}

// 3 - TREBLE SPARKLES

void trebleSparkles() {
  fadeToBlackBy(leds, NUM_LEDS, 45);

  int attempts = 1 + (int)(highs * 9.0f);

  for (int i = 0; i < attempts; i++) {
    if (random8() < power8(highs)) {
      leds[random16(NUM_LEDS)] =
        CHSV(random8(), 115, 255);
    }
  }

  if (beatNow) {
    for (int i = 0; i < 8; i++)
      whiteSparkle(255);
  }
}

// 4 - DANCING RAINBOW

void dancingRainbow() {
  uint8_t bright =
    15 + (uint8_t)(level * 240.0f);

  fill_rainbow(leds, NUM_LEDS, baseHue, 8);

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8_video(bright);
  }

  if (beatNow) {
    for (int i = 0; i < NUM_LEDS; i += 3)
      leds[i] = CRGB::White;
  }
}

// 5 - BASS LIGHTNING

void bassLightning() {
  fadeToBlackBy(leds, NUM_LEDS, 115);

  if (beatNow ||
      random8() < power8(bass * 0.25f)) {

    int first = random16(NUM_LEDS);
    int count = random8(2, 9);

    for (int n = 0; n < count; n++) {
      leds[(first + n) % NUM_LEDS] =
        CRGB(90, 170, 255);
    }
  }

  whiteSparkle(power8(highs * 0.65f));
}

// 6 - RED BLUE HEARTBEAT

void redBlueHeartbeat() {
  uint8_t strength =
    power8(bass * 0.8f + beatGlow * 0.2f);

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] =
      (i < NUM_LEDS / 2)
      ? CHSV(0, 255, strength)
      : CHSV(160, 255, strength);
  }

  if (beatNow) {
    leds[NUM_LEDS / 2 - 1] = CRGB::White;
    leds[NUM_LEDS / 2] = CRGB::White;
  }
}

// 7 - THREE BAND METER

void threeBandMeter() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  float bands[3] = {bass, mids, highs};
  uint8_t colors[3] = {0, 95, 160};

  for (int band = 0; band < 3; band++) {
    int start = (NUM_LEDS * band) / 3;
    int end = (NUM_LEDS * (band + 1)) / 3;

    int count =
      (int)(bands[band] * (end - start) + 0.5f);

    for (int i = start;
         i < start + count && i < end;
         i++) {

      leds[i] = CHSV(colors[band], 255, 255);
    }
  }
}

// 8 - MUSIC COMET

void musicComet() {
  fadeToBlackBy(leds, NUM_LEDS, 55);

  uint32_t cycle =
    (animationFrame * (1 + (int)(bass * 5))) %
    (NUM_LEDS * 2);

  int pos =
    (cycle < NUM_LEDS)
    ? (int)cycle
    : (int)(NUM_LEDS * 2 - 1 - cycle);

  leds[pos] =
    CHSV(baseHue, 255, power8(level));

  whiteSparkle(power8(highs * 0.6f));
}

// 9 - BEAT EXPLOSION

void beatExplosion() {
  fadeToBlackBy(leds, NUM_LEDS, 75);

  if (beatNow) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] =
        CHSV(baseHue + random8(110), 255, 255);
    }
  }
  else {
    int count = (int)(mids * NUM_LEDS / 2);

    for (int j = 0; j < count; j++) {
      leds[random16(NUM_LEDS)] +=
        CHSV(baseHue, 255, 70);
    }
  }
}

// 10 - DISCO CONFETTI

void discoConfetti() {
  fadeToBlackBy(leds, NUM_LEDS, 100);

  int flashes = (int)(level * 10.0f);

  for (int i = 0; i < flashes; i++) {
    leds[random16(NUM_LEDS)] =
      CHSV(random8(), 255, 255);
  }

  if (beatNow) {
    for (int i = 0; i < 8; i++)
      whiteSparkle(255);
  }
}

// 11 - OCEAN WAVES

void oceanWaves() {
  uint8_t power =
    power8(bass * 0.45f + mids * 0.55f);

  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t wave =
      sin8(i * 14 + animationFrame * 4);

    leds[i] =
      CHSV(130 + wave / 12,
           240,
           scale8(wave, power));
  }

  whiteSparkle(power8(highs * 0.6f));
}

// 12 - FIRE AND EMBERS

void fireEmbers() {
  for (int i = 0; i < NUM_LEDS; i++) {
    fireHeat[i] =
      qsub8(fireHeat[i], random8(0, 34));
  }

  for (int i = NUM_LEDS - 1; i >= 2; i--) {
    fireHeat[i] =
      (fireHeat[i - 1] +
       fireHeat[i - 2] +
       fireHeat[i - 2]) / 3;
  }

  if (random8() < power8(bass)) {
    int nearStart =
      random8((uint8_t)min(NUM_LEDS, 5));

    fireHeat[nearStart] =
      qadd8(fireHeat[nearStart],
            random8(150, 255));
  }

  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = HeatColor(fireHeat[i]);

  if (beatNow) {
    for (int i = 0; i < 4; i++)
      whiteSparkle(255);
  }
}

// ---------------------- LED FRAME -------------------------

void updateLEDs() {
  uint32_t now = millis();

  if (now - lastFrameAt < LED_FRAME_MS) return;

  lastFrameAt = now;

  updateMusicLevels();

  if (!lightsOn) return;

  // Gentle blue standby when music stops

  if (!musicActive && level < 0.012f) {
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, 12));
    FastLED.show();
    return;
  }

  switch (currentMode) {
    case 0:
      bassBloom();
      break;

    case 1:
      rainbowEqualizer();
      break;

    case 2:
      trebleSparkles();
      break;

    case 3:
      dancingRainbow();
      break;

    case 4:
      bassLightning();
      break;

    case 5:
      redBlueHeartbeat();
      break;

    case 6:
      threeBandMeter();
      break;

    case 7:
      musicComet();
      break;

    case 8:
      beatExplosion();
      break;

    case 9:
      discoConfetti();
      break;

    case 10:
      oceanWaves();
      break;

    case 11:
      fireEmbers();
      break;
  }

  baseHue += 2;
  animationFrame++;

  FastLED.show();
}

// ------------------------ STARTUP -------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VOLUME_PIN, INPUT);

  analogReadResolution(12);

  filteredKnob = analogRead(VOLUME_PIN);

  // Initialize LED strip

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(
    leds, NUM_LEDS
  );

  FastLED.setBrightness(LED_BRIGHTNESS);

  FastLED.setMaxPowerInVoltsAndMilliamps(
    5, LED_POWER_MA
  );

  lightsOn = true;
  currentMode = 0;

  fill_solid(
    leds, NUM_LEDS, CRGB(0, 0, 12)
  );

  FastLED.show();

  // Initialize I2S amplifier output

  auto audioConfig = i2s.defaultConfig();

  audioConfig.pin_bck = I2S_BCLK_PIN;
  audioConfig.pin_ws = I2S_WS_PIN;
  audioConfig.pin_data = I2S_DATA_PIN;

  i2s.begin(audioConfig);

  // Keep audio output active while analyzing samples

  a2dpSink.set_stream_reader(
    onBluetoothAudio, true
  );

  // Initial volume knob reading

  int startingVolume =
    map(constrain((int)filteredKnob, 0, 4095),
        0, 4095, 0, MAX_BT_VOLUME);

  a2dpSink.set_volume(
    (uint8_t)startingVolume
  );

  lastKnobVolume = startingVolume;

  // Start Bluetooth speaker

  a2dpSink.start(BLUETOOTH_NAME, false);

  Serial.println("=================================");
  Serial.println("      BOWIES SPEAKER READY");
  Serial.println("=================================");

  Serial.println("Bluetooth: Bowies Speaker");
  Serial.println("LED strip: GPIO27, 60 LEDs");
  Serial.println("Button: GPIO33 -> GND");
  Serial.println("Tap button: next music LED mode");
  Serial.println("Hold 1.5 seconds: LED on/off");
  Serial.println("Volume knob: GPIO34; cap ~90%");
  Serial.println("Startup: LEDs ON, mode 1");

  Serial.println("=================================");
}

// ------------------------ MAIN LOOP -----------------------

void loop() {
  updateButton();
  updateVolume();
  updateLEDs();

  delay(2);
}
