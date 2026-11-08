/* Host unit tests for Core/Src/audio.c (not part of the firmware build).
 * Build and run from the project root:
 *   gcc -Wall -Wextra -ICore/Inc Tests/audio_test.c Core/Src/audio.c -lm -o audio_test && ./audio_test
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "audio.h"

static int fails;
static Audio_Spectrum spec;

/* Run one spectrum of n samples from gen(); returns the number of steps taken. */
static int run_spectrum(double (*gen)(uint32_t))
{
  Audio_SpectrumStart(&spec);
  for (uint32_t i = 0; i < SPEC_N; i++)
  {
    Audio_SpectrumAdd(&spec, (int32_t)lround(gen(i) * 8388607.0));
  }
  int steps = 1;
  while (!Audio_SpectrumStep(&spec))
  {
    steps++;
  }
  return steps;
}

static double gen_half_scale_bin64(uint32_t n) { return 0.5 * sin(2 * M_PI * 64 * n / SPEC_N); }
static double gen_full_scale_bin10(uint32_t n) { return 0.999 * sin(2 * M_PI * 10 * n / SPEC_N); }
static double gen_mix(uint32_t n)
{
  /* off-bin tones and a little deterministic "noise" */
  return 0.3 * sin(2 * M_PI * 37.3 * n / SPEC_N) + 0.01 * cos(2 * M_PI * 150.7 * n / SPEC_N)
         + 0.001 * sin(n * 12.9898) - 0.05;
}
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

int main(void)
{
  /* 24-bit conversion both ways */
  CHECK(Audio_FromSai24(0x00000001) == 1);
  CHECK(Audio_FromSai24(0x007FFFFF) == 8388607);
  CHECK(Audio_FromSai24(0x00800000) == -8388608);
  CHECK(Audio_FromSai24(0x00FFFFFF) == -1);
  CHECK(Audio_FromSai24(0xAB000005) == 5);
  CHECK(Audio_ToSai24(-1) == 0x00FFFFFF);
  CHECK(Audio_ToSai24(9000000) == 0x007FFFFF);
  CHECK(Audio_ToSai24(-9000000) == 0x00800000);
  for (int32_t v = -8388608; v < 8388608; v += 4099)
  {
    CHECK(Audio_FromSai24(Audio_ToSai24(v)) == v);
  }

  /* Meter: DC offset of 1000 with a +/-4194304 (50%) square wave on top */
  Audio_Meter m;
  memset(&m, 0, sizeof m);
  for (uint32_t i = 0; i < 2 * AUDIO_METER_WINDOW; i++)
  {
    Audio_MeterAdd(&m, 1000 + ((i & 1) ? 4194304 : -4194304));
  }
  CHECK(m.dc == 1000);
  CHECK(m.peak == 4194304);
  CHECK(m.peak_pct == 50);
  memset(&m, 0, sizeof m);
  for (uint32_t i = 0; i < AUDIO_METER_WINDOW; i++)
  {
    Audio_MeterAdd(&m, 0);
  }
  CHECK(m.peak == 0 && m.peak_pct == 0);
  memset(&m, 0, sizeof m);
  for (uint32_t i = 0; i < AUDIO_METER_WINDOW; i++)
  {
    Audio_MeterAdd(&m, -8388608);
  }
  CHECK(m.peak == 8388608 && m.peak_pct == 100);

  /* Clip at ratio 2: doubles the rate with midpoints, loops at the end */
  const int16_t d[3] = { 100, -100, 32767 };
  Audio_Clip c = { d, 3, 0, 0, 2 };
  int32_t want[] = { 25600, 0, -25600, 4181248, 8388352, 4207104, 25600, 0 };
  for (unsigned i = 0; i < sizeof want / sizeof want[0]; i++)
  {
    int32_t got = Audio_ClipNext(&c);
    if (got != want[i]) { printf("FAIL clip[%u] = %d, want %d\n", i, got, want[i]); fails++; }
  }

  /* Recorder at ratio 2: pairs averaged into 16-bit, stops when full */
  int16_t rb[3];
  Audio_Recorder r = { rb, 3, 0, 0, 0, 2 };
  Audio_RecStart(&r);
  int32_t in[] = { 256, 768, -25600, -25600, 8388607, 8388607 };
  uint8_t done = 0;
  for (int i = 0; i < 6; i++)
  {
    CHECK(done == 0);
    done = Audio_RecAdd(&r, in[i]);
  }
  CHECK(done == 1 && r.len == 3);
  CHECK(rb[0] == 2 && rb[1] == -100 && rb[2] == 32767);
  CHECK(Audio_RecAdd(&r, 0) == 1 && r.len == 3);
  Audio_RecStart(&r);
  CHECK(r.len == 0);
  Audio_Clip pc = { rb, 3, 0, 0, 2 };
  CHECK(Audio_ClipNext(&pc) == 512);

  /* Ratio 3 (48 kHz <-> 16 kHz): groups of 3 averaged, played back as 3 interpolated frames */
  int16_t rb3[2];
  Audio_Recorder r3 = { rb3, 2, 0, 0, 0, 3 };
  int32_t in3[] = { 0, 768, 1536, 7680, 7680, 7680 };
  for (int i = 0; i < 6; i++)
  {
    done = Audio_RecAdd(&r3, in3[i]);
  }
  CHECK(done == 1 && rb3[0] == 3 && rb3[1] == 30);
  Audio_Clip c3 = { rb3, 2, 0, 0, 3 };
  int32_t want3[] = { 768, 3072, 5376, 7680, 5376, 3072, 768 };
  for (unsigned i = 0; i < sizeof want3 / sizeof want3[0]; i++)
  {
    int32_t got = Audio_ClipNext(&c3);
    if (got != want3[i]) { printf("FAIL clip3[%u] = %d, want %d\n", i, got, want3[i]); fails++; }
  }

  /* Button: 20 ms debounce, one event per press, bounces ignored */
  Audio_Button b;
  memset(&b, 0, sizeof b);
  int presses = 0;
  for (int i = 0; i < 100; i++)
  {
    presses += Audio_ButtonPressed(&b, i & 1);
  }
  CHECK(presses == 0);
  for (unsigned i = 0; i < AUDIO_BUTTON_FRAMES + 5; i++)
  {
    presses += Audio_ButtonPressed(&b, 1);
  }
  CHECK(presses == 1);
  for (unsigned i = 0; i < 5000; i++)
  {
    presses += Audio_ButtonPressed(&b, 1);
  }
  CHECK(presses == 1);
  for (unsigned i = 0; i < AUDIO_BUTTON_FRAMES + 5; i++)
  {
    presses += Audio_ButtonPressed(&b, 0);
  }
  for (unsigned i = 0; i < AUDIO_BUTTON_FRAMES + 5; i++)
  {
    presses += Audio_ButtonPressed(&b, 1);
  }
  CHECK(presses == 2);

  /* Spectrum: on-bin tones land in the right bin at the right level */
  Audio_SpectrumInit();
  int steps = run_spectrum(gen_half_scale_bin64);
  /* log2(N) FFT stages of N/2 butterflies, then N/2 magnitudes, 16 per step */
  int stages = 0;
  while ((1U << stages) < SPEC_N) stages++;
  CHECK(steps == stages * (int)(SPEC_N / 2U / 16U) + (int)(SPEC_BINS / 16U));
  int peak = 0;
  for (int k = 1; k < (int)SPEC_BINS; k++)
  {
    if (spec.db[k] < spec.db[peak]) peak = k;
  }
  CHECK(peak == 64);
  CHECK(spec.db[64] == 6);                             /* 0.5 of full scale = -6 dBFS */
  for (int k = 0; k < (int)SPEC_BINS; k++)
  {
    if (k < 63 || k > 65) CHECK(spec.db[k] >= 100);   /* Hann leaves only bins 63..65 */
  }
  run_spectrum(gen_full_scale_bin10);
  CHECK(spec.db[10] == 0);

  /* Spectrum vs. a direct DFT in double precision, for an off-bin mixture */
  run_spectrum(gen_mix);
  int worst = 0;
  for (int k = 0; k < (int)SPEC_BINS; k++)
  {
    double re = 0, im = 0;
    for (uint32_t n = 0; n < SPEC_N; n++)
    {
      double x = round(gen_mix(n) * 8388607.0) / 8388608.0;
      double w = 0.5 - 0.5 * cos(2 * M_PI * n / SPEC_N);
      re += x * w * cos(2 * M_PI * k * n / SPEC_N);
      im -= x * w * sin(2 * M_PI * k * n / SPEC_N);
    }
    double ref = -10 * log10((re * re + im * im) * 16.0 / ((double)SPEC_N * SPEC_N));
    if (ref < 100)
    {
      int diff = abs((int)lround(ref) - (int)spec.db[k]);
      if (diff > worst) worst = diff;
    }
  }
  CHECK(worst <= 1);
  printf("spectrum: %d steps per run, worst error vs. direct DFT %d dB\n", steps, worst);

  /* Tone: 5 kHz at -12 dBFS, checked by zero crossings and by the spectrum */
  Audio_Tone t;
  Audio_ToneSet(&t, 5000.0f, 48000.0f, 0.25f);
  int crossings = 0, max_abs = 0;
  int32_t prev = Audio_ToneNext(&t);
  for (int i = 1; i < 48000; i++)
  {
    int32_t v = Audio_ToneNext(&t);
    if ((prev < 0) != (v < 0)) crossings++;
    if (abs(v) > max_abs) max_abs = abs(v);
    prev = v;
  }
  CHECK(crossings >= 9998 && crossings <= 10002);      /* 2 per cycle, 5000 cycles in 1 s */
  CHECK(max_abs > 2097000 && max_abs <= 2097152);      /* 0.25 of full scale */
  Audio_ToneSet(&t, 5000.0f, 48000.0f, 0.25f);
  Audio_SpectrumStart(&spec);
  for (uint32_t i = 0; i < SPEC_N; i++) Audio_SpectrumAdd(&spec, Audio_ToneNext(&t));
  while (!Audio_SpectrumStep(&spec)) {}
  int tpeak = 2;
  for (int k = 2; k < (int)SPEC_BINS; k++) if (spec.db[k] < spec.db[tpeak]) tpeak = k;
  int tbin = (int)lround(5000.0 * SPEC_N / 48000.0);  /* nearest bin to 5 kHz */
  CHECK(tpeak == tbin);
  CHECK(spec.db[tbin] >= 12 && spec.db[tbin] <= 14);   /* -12 dBFS less up to 1.4 dB off-bin loss */
  printf("tone: %d zero crossings in 1 s, peak bin %d at -%d dBFS\n", crossings, tpeak, spec.db[tpeak]);

  printf("audio tests: %s (%d failures)\n", fails ? "FAILED" : "passed", fails);
  return fails != 0;
}
