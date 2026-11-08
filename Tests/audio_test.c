/* Host unit tests for Core/Src/audio.c (not part of the firmware build).
 * Build and run from the project root:
 *   gcc -Wall -Wextra -ICore/Inc Tests/audio_test.c Core/Src/audio.c -o audio_test && ./audio_test
 */
#include <stdio.h>
#include <string.h>
#include "audio.h"

static int fails;
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

  printf("audio tests: %s (%d failures)\n", fails ? "FAILED" : "passed", fails);
  return fails != 0;
}
