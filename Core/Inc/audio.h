/**
  * @file    audio.h
  * @brief   Sample-level helpers for the codec audio path (no hardware access).
  *
  * Samples are signed 24-bit values held in int32_t, matching the SAI's 24-bit
  * data in 32-bit TDM slots.
  */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>

/* Meter window: 4800 frames = 100 ms at 48 kHz. */
#define AUDIO_METER_WINDOW 4800U

typedef struct
{
  /* Results, updated once per window. */
  int32_t  dc;          /* average sample value (offset) */
  uint32_t peak;        /* largest |sample - dc| in the window, 0..8388608 */
  uint32_t peak_pct;    /* peak as a percentage of full scale */
  /* Running state. */
  int64_t  sum;
  uint32_t max_dev;
  uint32_t count;
} Audio_Meter;

typedef struct
{
  const int16_t *data;  /* 16-bit clip at half the output rate */
  uint32_t len;
  uint32_t pos;
  uint8_t  half;        /* 0 = output the sample, 1 = output the midpoint to the next */
} Audio_Clip;

typedef struct
{
  int16_t *buf;         /* recording at half the input rate */
  uint32_t cap;         /* buffer size in samples */
  uint32_t len;         /* samples recorded so far */
  int32_t  pending;     /* first sample of the current pair */
  uint8_t  odd;         /* 1 when holding the first sample of a pair */
} Audio_Recorder;

typedef struct
{
  uint8_t  stable;      /* debounced state, 1 = pressed */
  uint8_t  raw;
  uint16_t count;       /* frames the raw state has been unchanged */
} Audio_Button;

/* Debounce time in frames: 960 = 20 ms at 48 kHz. */
#define AUDIO_BUTTON_FRAMES 960U

/* Raw 32-bit SAI receive word -> signed 24-bit sample. */
int32_t Audio_FromSai24(uint32_t word);

/* Signed 24-bit sample -> 32-bit SAI transmit word. Out-of-range values are clipped. */
uint32_t Audio_ToSai24(int32_t sample);

void Audio_MeterAdd(Audio_Meter *m, int32_t sample);

/* Next 24-bit output sample of the clip, doubling its rate by linear interpolation.
 * Loops back to the start at the end. */
int32_t Audio_ClipNext(Audio_Clip *c);

void Audio_RecStart(Audio_Recorder *r);

/* Add one 24-bit sample at the full rate. Pairs are averaged into one 16-bit sample.
 * Returns 1 once the buffer is full. */
uint8_t Audio_RecAdd(Audio_Recorder *r, int32_t sample);

/* Call once per frame with the raw pin state (1 = pressed).
 * Returns 1 on the frame a debounced press starts. */
uint8_t Audio_ButtonPressed(Audio_Button *b, uint8_t pressed);

#endif /* AUDIO_H */
