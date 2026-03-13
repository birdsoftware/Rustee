/*
  Sleep Meditation LEDs
  ---------------------
  Designed for 10 discrete LEDs on pins 2-11.
  Works on Arduino Nano / Uno.

  Features:
  - Soft breathing glow
  - Slow drifting light waves
  - Gentle star-like twinkles
  - Smooth fades using software PWM on ANY digital pin
  - No harsh flashes or sudden patterns

  Wiring:
  - LEDs on pins 2..11
  - Each LED should have its own resistor

  Notes:
  - This uses software PWM so fading works even on non-PWM pins.
  - Best for a dark room / sleep meditation / calming visual rhythm.
*/

#include <Arduino.h>
#include <math.h>

const uint8_t leds[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const uint8_t LED_COUNT = sizeof(leds) / sizeof(leds[0]);

// Brightness buffer: 0-255 for each LED
uint8_t level[LED_COUNT];

// ---------- Timing / render config ----------
const uint8_t PWM_STEPS = 32;          // More steps = smoother fade
const uint16_t FRAME_MS = 16;          // ~60 FPS
const uint16_t REST_BETWEEN_SCENES_MS = 1200;

// ---------- Twinkle state ----------
struct Twinkle {
  bool active;
  uint8_t led;
  unsigned long startMs;
  unsigned long durationMs;
  uint8_t peak;
};

const uint8_t MAX_TWINKLES = 3;
Twinkle twinkles[MAX_TWINKLES];

// ---------- Utility ----------
float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

float smoothstep(float x) {
  x = clamp01(x);
  return x * x * (3.0f - 2.0f * x);
}

// Simple gamma correction so fades look softer to the eye
uint8_t gamma8(uint8_t v) {
  uint16_t x = v;
  return (uint8_t)((x * x) / 255); // perceptual-ish curve
}

void clearLevels() {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    level[i] = 0;
  }
}

void allOffImmediate() {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    digitalWrite(leds[i], LOW);
  }
}

void renderFrame(uint16_t frameMs) {
  // Software PWM across all LEDs so we get fading on any digital pin
  unsigned long sliceUs = ((unsigned long)frameMs * 1000UL) / PWM_STEPS;

  for (uint8_t step = 0; step < PWM_STEPS; step++) {
    uint8_t threshold = map(step, 0, PWM_STEPS - 1, 0, 255);

    for (uint8_t i = 0; i < LED_COUNT; i++) {
      digitalWrite(leds[i], (level[i] > threshold) ? HIGH : LOW);
    }

    delayMicroseconds(sliceUs);
  }
}

void renderFor(unsigned long durationMs, void (*sceneFn)(unsigned long nowMs)) {
  unsigned long start = millis();

  while (millis() - start < durationMs) {
    unsigned long now = millis() - start;
    sceneFn(now);
    renderFrame(FRAME_MS);
  }
}

float ledDistance(uint8_t a, float b) {
  float d = fabs((float)a - b);
  return d;
}

// Adds a soft glow centered at "center" with gaussian-like falloff
void addGlow(float center, float width, float brightness01) {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    float d = ledDistance(i, center);
    float influence = expf(-(d * d) / (2.0f * width * width));
    int added = (int)(255.0f * brightness01 * influence);

    int v = level[i] + added;
    level[i] = (v > 255) ? 255 : v;
  }
}

void addBaseAmbient(uint8_t value) {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    if (value > level[i]) level[i] = value;
  }
}

// ---------- Twinkle system ----------
void resetTwinkles() {
  for (uint8_t i = 0; i < MAX_TWINKLES; i++) {
    twinkles[i].active = false;
  }
}

void maybeSpawnTwinkle(unsigned long nowMs, uint16_t chancePerFrame) {
  if (random(0, chancePerFrame) != 0) return;

  for (uint8_t i = 0; i < MAX_TWINKLES; i++) {
    if (!twinkles[i].active) {
      twinkles[i].active = true;
      twinkles[i].led = random(0, LED_COUNT);
      twinkles[i].startMs = nowMs;
      twinkles[i].durationMs = random(1800, 3200);
      twinkles[i].peak = random(35, 85); // keep twinkles dim and gentle
      return;
    }
  }
}

void applyTwinkles(unsigned long nowMs) {
  for (uint8_t i = 0; i < MAX_TWINKLES; i++) {
    if (!twinkles[i].active) continue;

    unsigned long age = nowMs - twinkles[i].startMs;
    if (age >= twinkles[i].durationMs) {
      twinkles[i].active = false;
      continue;
    }

    float t = (float)age / (float)twinkles[i].durationMs;

    // Soft in / soft out
    float envelope;
    if (t < 0.5f) {
      envelope = smoothstep(t * 2.0f);
    } else {
      envelope = smoothstep((1.0f - t) * 2.0f);
    }

    uint8_t value = (uint8_t)(twinkles[i].peak * envelope);
    uint8_t idx = twinkles[i].led;

    int v = level[idx] + value;
    level[idx] = (v > 255) ? 255 : v;
  }
}

// ---------- Scenes ----------

// Scene 1: Center breathing, like calm inhale/exhale
void sceneCenterBreath(unsigned long nowMs) {
  clearLevels();

  // 12 second breath cycle
  float cycle = fmod(nowMs / 12000.0f, 1.0f);
  float breath = 0.5f - 0.5f * cosf(cycle * TWO_PI); // 0..1
  breath = smoothstep(breath);

  // Slightly shifts around center to keep it organic
  float center = 4.5f + 0.35f * sinf(nowMs / 7000.0f);
  float width = 0.9f + 2.2f * breath;
  float bright = 0.08f + 0.55f * breath;

  addGlow(center, width, bright);
  addBaseAmbient(2);

  for (uint8_t i = 0; i < LED_COUNT; i++) {
    level[i] = gamma8(level[i]);
  }
}

// Scene 2: Very slow moon-wave drifting left/right
void sceneMoonWave(unsigned long nowMs) {
  clearLevels();

  for (uint8_t i = 0; i < LED_COUNT; i++) {
    float phase = (float)i * 0.52f - (nowMs / 4200.0f);
    float wave = 0.5f + 0.5f * sinf(phase);
    wave = smoothstep(wave);

    float secondary = 0.5f + 0.5f * sinf((float)i * 0.23f + nowMs / 9000.0f);
    float combined = 0.10f + 0.40f * wave + 0.14f * secondary;

    level[i] = gamma8((uint8_t)(255.0f * clamp01(combined)));
  }
}

// Scene 3: Dim ambient with rare, gentle stars
void sceneNightTwinkle(unsigned long nowMs) {
  clearLevels();

  // Very dim background wave
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    float bg = 0.03f + 0.05f * (0.5f + 0.5f * sinf((float)i * 0.6f + nowMs / 8000.0f));
    level[i] = gamma8((uint8_t)(255.0f * bg));
  }

  maybeSpawnTwinkle(nowMs, 22);
  applyTwinkles(nowMs);

  for (uint8_t i = 0; i < LED_COUNT; i++) {
    level[i] = gamma8(level[i]);
  }
}

// Scene 4: Long exhale into darkness
void sceneDeepExhale(unsigned long nowMs) {
  clearLevels();

  // 20 sec fade down
  float t = clamp01((float)nowMs / 20000.0f);
  float fade = 1.0f - smoothstep(t);

  float center = 4.5f;
  float width = 3.4f;
  float brightness = 0.38f * fade;

  addGlow(center, width, brightness);

  // Tiny residual shimmer while fading out
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    float shimmer = 0.015f * (0.5f + 0.5f * sinf(nowMs / 5000.0f + i));
    int v = level[i] + (int)(255.0f * shimmer * fade);
    level[i] = (v > 255) ? 255 : v;
  }

  for (uint8_t i = 0; i < LED_COUNT; i++) {
    level[i] = gamma8(level[i]);
  }
}

void darkPause(unsigned long durationMs) {
  clearLevels();
  unsigned long start = millis();

  while (millis() - start < durationMs) {
    renderFrame(FRAME_MS);
  }
}

void setup() {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  randomSeed(analogRead(A0));
  resetTwinkles();
  clearLevels();
  allOffImmediate();
}

void loop() {
  resetTwinkles();
  renderFor(180000UL, sceneCenterBreath);  // 3 min slow breath
  darkPause(REST_BETWEEN_SCENES_MS);

  resetTwinkles();
  renderFor(180000UL, sceneMoonWave);      // 3 min drifting wave
  darkPause(REST_BETWEEN_SCENES_MS);

  resetTwinkles();
  renderFor(180000UL, sceneNightTwinkle);  // 3 min gentle stars
  darkPause(REST_BETWEEN_SCENES_MS);

  resetTwinkles();
  renderFor(60000UL, sceneDeepExhale);     // 1 min exhale into near-dark
  darkPause(10000UL);                      // 10 sec darkness before repeating
}