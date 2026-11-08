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
  const int16_t *data;  /* 16-bit clip at 1/ratio of the output rate */
  uint32_t len;
  uint32_t pos;
  uint8_t  phase;       /* output frames already produced from data[pos], 0..ratio-1 */
  uint8_t  ratio;       /* output frames per clip sample */
} Audio_Clip;

typedef struct
{
  int16_t *buf;         /* recording at 1/ratio of the input rate */
  uint32_t cap;         /* buffer size in samples */
  uint32_t len;         /* samples recorded so far */
  int32_t  acc;         /* sum of the current group of input samples */
  uint8_t  n;           /* input samples in the current group */
  uint8_t  ratio;       /* input samples averaged into each stored sample */
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

/* Next 24-bit output sample of the clip, raising its rate by c->ratio with linear
 * interpolation. Loops back to the start at the end. */
int32_t Audio_ClipNext(Audio_Clip *c);

void Audio_RecStart(Audio_Recorder *r);

/* Add one 24-bit sample at the full rate. Each group of r->ratio samples is averaged
 * into one 16-bit sample. Returns 1 once the buffer is full. */
uint8_t Audio_RecAdd(Audio_Recorder *r, int32_t sample);

/* Call once per frame with the raw pin state (1 = pressed).
 * Returns 1 on the frame a debounced press starts. */
uint8_t Audio_ButtonPressed(Audio_Button *b, uint8_t pressed);

/* ---- Tone generator: sine at any frequency (phase accumulator + table) ---- */
typedef struct
{
  uint32_t phase;   /* 0..2^32 is one cycle */
  uint32_t inc;     /* phase step per sample */
  float    amp;     /* peak amplitude in 24-bit units */
} Audio_Tone;

/* hz = tone frequency, fs = sample rate, level = peak amplitude as a fraction of
 * full scale (0..1). */
void Audio_ToneSet(Audio_Tone *t, float hz, float fs, float level);

/* Next 24-bit sample of the tone. */
int32_t Audio_ToneNext(Audio_Tone *t);

/* ---- Spectrum: 512-point FFT with a Hann window, split into small steps ----
 * Each Audio_SpectrumStep() call does a bounded amount of work (a few us on the
 * STM32) so it can run once per audio frame without missing samples. */
#define SPEC_N     512U
#define SPEC_BINS  (SPEC_N / 2U)   /* bin k is k * fs / SPEC_N Hz */
#define SPEC_FLOOR 140U            /* db[] values are clamped to -140 dBFS */

typedef struct
{
  float    re[SPEC_N];
  float    im[SPEC_N];
  uint8_t  db[SPEC_BINS];  /* result: level of each bin in -dBFS (0 = full-scale sine) */
  uint32_t pos;            /* samples collected, butterflies done in the stage, or bins done */
  uint32_t size;           /* current FFT stage size, 2..SPEC_N */
  uint8_t  phase;          /* 0 = collecting, 1 = FFT, 2 = magnitudes, 3 = done */
} Audio_Spectrum;

/* Build the window, bit-reversal and twiddle tables. Call once before use. */
void Audio_SpectrumInit(void);

void Audio_SpectrumStart(Audio_Spectrum *s);

/* Add one 24-bit sample. Returns 1 once SPEC_N samples have been collected. */
uint8_t Audio_SpectrumAdd(Audio_Spectrum *s, int32_t sample);

/* Do the next slice of work. Returns 1 when s->db[] holds the finished spectrum. */
uint8_t Audio_SpectrumStep(Audio_Spectrum *s);

#endif /* AUDIO_H */
