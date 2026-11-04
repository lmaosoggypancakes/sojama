/**
  * @file    audio.c
  * @brief   Sample-level helpers for the codec audio path (no hardware access).
  */
#include "audio.h"

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
  int32_t cur = c->data[c->pos];
  int32_t out;

  if (c->half == 0U)
  {
    out = cur * 256;
    c->half = 1U;
  }
  else
  {
    uint32_t next_pos = (c->pos + 1U == c->len) ? 0U : c->pos + 1U;
    out = ((cur + c->data[next_pos]) / 2) * 256;
    c->pos = next_pos;
    c->half = 0U;
  }
  return out;
}

void Audio_RecStart(Audio_Recorder *r)
{
  r->len = 0;
  r->odd = 0;
}

uint8_t Audio_RecAdd(Audio_Recorder *r, int32_t sample)
{
  if (r->len >= r->cap)
  {
    return 1;
  }
  if (r->odd == 0U)
  {
    r->pending = sample;
    r->odd = 1U;
    return 0;
  }
  r->odd = 0U;
  r->buf[r->len++] = (int16_t)(((r->pending + sample) / 2) / 256);
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
