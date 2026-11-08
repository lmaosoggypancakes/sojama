/**
  * @file    audio.c
  * @brief   Sample-level helpers for the codec audio path (no hardware access).
  */
#include "audio.h"
#include <math.h>

#define AUDIO_FULL_SCALE 8388608  /* 2^23 */

int32_t Audio_FromSai24(uint32_t word)
{
  word &= 0x00FFFFFFU;
  if (word & 0x00800000U)
  {
    word |= 0xFF000000U;
  }
  return (int32_t)word;
}

uint32_t Audio_ToSai24(int32_t sample)
{
  if (sample > AUDIO_FULL_SCALE - 1)
  {
    sample = AUDIO_FULL_SCALE - 1;
  }
  else if (sample < -AUDIO_FULL_SCALE)
  {
    sample = -AUDIO_FULL_SCALE;
  }
  return (uint32_t)sample & 0x00FFFFFFU;
}

void Audio_MeterAdd(Audio_Meter *m, int32_t sample)
{
  int32_t dev = sample - m->dc;
  uint32_t mag = (dev < 0) ? (uint32_t)(-(int64_t)dev) : (uint32_t)dev;

  m->sum += sample;
  if (mag > m->max_dev)
  {
    m->max_dev = mag;
  }
  if (++m->count == AUDIO_METER_WINDOW)
  {
    m->dc = (int32_t)(m->sum / (int64_t)AUDIO_METER_WINDOW);
    m->peak = m->max_dev;
    m->peak_pct = (uint32_t)(((uint64_t)m->max_dev * 100U) / AUDIO_FULL_SCALE);
    m->sum = 0;
    m->max_dev = 0;
    m->count = 0;
  }
}

int32_t Audio_ClipNext(Audio_Clip *c)
{
  uint32_t next_pos = (c->pos + 1U == c->len) ? 0U : c->pos + 1U;
  int32_t cur = c->data[c->pos];
  int32_t next = c->data[next_pos];
  int32_t out = (cur + ((next - cur) * (int32_t)c->phase) / (int32_t)c->ratio) * 256;

  if (++c->phase == c->ratio)
  {
    c->phase = 0U;
    c->pos = next_pos;
  }
  return out;
}

void Audio_RecStart(Audio_Recorder *r)
{
  r->len = 0;
  r->acc = 0;
  r->n = 0;
}

uint8_t Audio_RecAdd(Audio_Recorder *r, int32_t sample)
{
  if (r->len >= r->cap)
  {
    return 1;
  }
  r->acc += sample;
  if (++r->n < r->ratio)
  {
    return 0;
  }
  r->buf[r->len++] = (int16_t)((r->acc / (int32_t)r->ratio) / 256);
  r->acc = 0;
  r->n = 0;
  return (r->len >= r->cap) ? 1U : 0U;
}

uint8_t Audio_ButtonPressed(Audio_Button *b, uint8_t pressed)
{
  if (pressed != b->raw)
  {
    b->raw = pressed;
    b->count = 0;
    return 0;
  }
  if (b->count < AUDIO_BUTTON_FRAMES)
  {
    b->count++;
    return 0;
  }
  if (b->stable != b->raw)
  {
    b->stable = b->raw;
    return b->stable;
  }
  return 0;
}

/* ---- Tone generator ------------------------------------------------------ */

#define TONE_TABLE_BITS 8U
#define TONE_TABLE_SIZE (1U << TONE_TABLE_BITS)

/* One sine cycle plus a wrap-around entry for interpolation. */
static float tone_table[TONE_TABLE_SIZE + 1U];
static uint8_t tone_table_ready;

void Audio_ToneSet(Audio_Tone *t, float hz, float fs, float level)
{
  if (!tone_table_ready)
  {
    for (uint32_t i = 0; i <= TONE_TABLE_SIZE; i++)
    {
      tone_table[i] = sinf(6.28318530718f * (float)i / (float)TONE_TABLE_SIZE);
    }
    tone_table_ready = 1;
  }
  t->phase = 0;
  t->inc = (uint32_t)((double)hz / (double)fs * 4294967296.0);
  t->amp = level * 8388607.0f;
}

int32_t Audio_ToneNext(Audio_Tone *t)
{
  /* Top bits pick the table entry, the rest interpolate to the next one. */
  uint32_t idx = t->phase >> (32U - TONE_TABLE_BITS);
  float frac = (float)(t->phase & ((1U << (32U - TONE_TABLE_BITS)) - 1U))
               * (1.0f / (float)(1U << (32U - TONE_TABLE_BITS)));
  float v = tone_table[idx] + (tone_table[idx + 1U] - tone_table[idx]) * frac;
  t->phase += t->inc;
  return (int32_t)(v * t->amp);
}

/* ---- Spectrum ------------------------------------------------------------ */

#define SPEC_BUTTERFLIES_PER_STEP 16U
#define SPEC_BINS_PER_STEP        16U

static float spec_window[SPEC_N];
static uint16_t spec_rev[SPEC_N];
static float spec_cos[SPEC_N / 2U];
static float spec_sin[SPEC_N / 2U];

void Audio_SpectrumInit(void)
{
  const float two_pi = 6.28318530718f;
  uint32_t bits = 0;
  while ((1U << bits) < SPEC_N)
  {
    bits++;
  }
  for (uint32_t n = 0; n < SPEC_N; n++)
  {
    spec_window[n] = 0.5f - 0.5f * cosf(two_pi * (float)n / (float)SPEC_N);
    uint32_t r = 0;
    for (uint32_t b = 0; b < bits; b++)
    {
      r |= ((n >> b) & 1U) << (bits - 1U - b);
    }
    spec_rev[n] = (uint16_t)r;
  }
  for (uint32_t k = 0; k < SPEC_N / 2U; k++)
  {
    spec_cos[k] = cosf(two_pi * (float)k / (float)SPEC_N);
    spec_sin[k] = -sinf(two_pi * (float)k / (float)SPEC_N);
  }
}

void Audio_SpectrumStart(Audio_Spectrum *s)
{
  s->pos = 0;
  s->size = 2;
  s->phase = 0;
}

uint8_t Audio_SpectrumAdd(Audio_Spectrum *s, int32_t sample)
{
  if (s->phase != 0U)
  {
    return 1;
  }
  /* Windowed sample, scaled to +/-1.0, stored in bit-reversed order for the FFT. */
  uint32_t i = spec_rev[s->pos];
  s->re[i] = (float)sample * (1.0f / 8388608.0f) * spec_window[s->pos];
  s->im[i] = 0.0f;
  if (++s->pos == SPEC_N)
  {
    s->pos = 0;
    s->phase = 1;
    return 1;
  }
  return 0;
}

/* log2(x) for x > 0, accurate to about 0.005 (0.015 dB in the result). */
static float fast_log2(float x)
{
  union { float f; uint32_t i; } u = { x };
  float e = (float)((int32_t)((u.i >> 23) & 0xFFU) - 128);   /* the polynomial adds the 1 back */
  u.i = (u.i & 0x007FFFFFU) | 0x3F800000U;
  float m = u.f;
  return e + (-0.34484843f * m + 2.02466578f) * m - 0.67487759f;
}

uint8_t Audio_SpectrumStep(Audio_Spectrum *s)
{
  if (s->phase == 1U)
  {
    /* Radix-2 butterflies, stage by stage; SPEC_N / 2 butterflies per stage. */
    uint32_t half = s->size / 2U;
    uint32_t tw_step = SPEC_N / s->size;
    for (uint32_t n = 0; n < SPEC_BUTTERFLIES_PER_STEP && s->pos < SPEC_N / 2U; n++, s->pos++)
    {
      uint32_t j = s->pos % half;
      uint32_t a = (s->pos / half) * s->size + j;
      uint32_t b = a + half;
      float wr = spec_cos[j * tw_step];
      float wi = spec_sin[j * tw_step];
      float tr = s->re[b] * wr - s->im[b] * wi;
      float ti = s->re[b] * wi + s->im[b] * wr;
      s->re[b] = s->re[a] - tr;
      s->im[b] = s->im[a] - ti;
      s->re[a] += tr;
      s->im[a] += ti;
    }
    if (s->pos == SPEC_N / 2U)
    {
      s->pos = 0;
      if (s->size == SPEC_N)
      {
        s->phase = 2;
      }
      else
      {
        s->size *= 2U;
      }
    }
    return 0;
  }

  if (s->phase == 2U)
  {
    /* A full-scale sine gives |X| = SPEC_N / 4 with the Hann window, so
     * amplitude = |X| * 4 / SPEC_N and dBFS = 10 * log10(power). */
    const float scale = 16.0f / ((float)SPEC_N * (float)SPEC_N);
    for (uint32_t n = 0; n < SPEC_BINS_PER_STEP && s->pos < SPEC_BINS; n++, s->pos++)
    {
      float p = (s->re[s->pos] * s->re[s->pos] + s->im[s->pos] * s->im[s->pos]) * scale;
      uint8_t v = (uint8_t)SPEC_FLOOR;
      if (p > 0.0f)
      {
        float neg_db = -3.01029996f * fast_log2(p);   /* -10 * log10(p) */
        if (neg_db < 0.0f)
        {
          v = 0;
        }
        else if (neg_db < (float)SPEC_FLOOR)
        {
          v = (uint8_t)(neg_db + 0.5f);
        }
      }
      s->db[s->pos] = v;
    }
    if (s->pos == SPEC_BINS)
    {
      s->phase = 3;
    }
    return (s->phase == 3U) ? 1U : 0U;
  }

  return (s->phase == 3U) ? 1U : 0U;
}
