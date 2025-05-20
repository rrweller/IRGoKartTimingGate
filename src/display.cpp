#include "display.h"
#include <SPI.h>

/* ──────────────────  SPI / latch  ────────────────── */
static const uint8_t LATCH_PIN = 10;                // TPIC6B595 RCK
static const SPISettings SPI_CFG(4000000, MSBFIRST, SPI_MODE0);

/* ──────────────────  Segment maps  ──────────────────
   RED digits wiring (first 3 shift registers)
     D0→E  D1→D  D2→C  D3→DP  D4→B  D5→A  D6→F  D7→G          */
static const uint8_t R_A=5, R_B=4, R_C=2, R_D=1, R_E=0, R_F=6, R_G=7, R_DP=3;

/* GREEN digits wiring (next 4 shift registers)
     D0→E  D1→D  D2→C  D3→B  D4→A  D5→DP  D6→F  D7→G          */
static const uint8_t G_A=4, G_B=3, G_C=2, G_D=1, G_E=0, G_F=6, G_G=7, G_DP=5;

/* Shift-register byte order: RED0,RED1,RED2,GREEN0,GREEN1,GREEN2,GREEN3 */
enum DigitIdx { RED0=0, RED1, RED2, GREEN0, GREEN1, GREEN2, GREEN3 };

/* ───────── helpers to build bit-masks ───────── */
#define RMASK(a,b,c,d,e,f,g,dp) \
  ((a?1<<R_A:0)|(b?1<<R_B:0)|(c?1<<R_C:0)|(d?1<<R_D:0)| \
   (e?1<<R_E:0)|(f?1<<R_F:0)|(g?1<<R_G:0)|(dp?1<<R_DP:0))

#define GMASK(a,b,c,d,e,f,g,dp) \
  ((a?1<<G_A:0)|(b?1<<G_B:0)|(c?1<<G_C:0)|(d?1<<G_D:0)| \
   (e?1<<G_E:0)|(f?1<<G_F:0)|(g?1<<G_G:0)|(dp?1<<G_DP:0))

/* digit patterns (DP off) */
static const uint8_t RDIGIT[10]={
  RMASK(1,1,1,1,1,1,0,0), RMASK(0,1,1,0,0,0,0,0),
  RMASK(1,1,0,1,1,0,1,0), RMASK(1,1,1,1,0,0,1,0),
  RMASK(0,1,1,0,0,1,1,0), RMASK(1,0,1,1,0,1,1,0),
  RMASK(1,0,1,1,1,1,1,0), RMASK(1,1,1,0,0,0,0,0),
  RMASK(1,1,1,1,1,1,1,0), RMASK(1,1,1,1,0,1,1,0)
};

static const uint8_t GDIGIT[10]={
  GMASK(1,1,1,1,1,1,0,0), GMASK(0,1,1,0,0,0,0,0),
  GMASK(1,1,0,1,1,0,1,0), GMASK(1,1,1,1,0,0,1,0),
  GMASK(0,1,1,0,0,1,1,0), GMASK(1,0,1,1,0,1,1,0),
  GMASK(1,0,1,1,1,1,1,0), GMASK(1,1,1,0,0,0,0,0),
  GMASK(1,1,1,1,1,1,1,0), GMASK(1,1,1,1,0,1,1,0)
};

static const uint8_t SEG_BLANK = 0;

/* ===== low-level shift (no inversion!) ===== */
static void shiftBytes(const uint8_t *b, uint8_t n)
{
  digitalWrite(LATCH_PIN, LOW);
  for (int8_t i=n-1; i>=0; --i) SPI.transfer(b[i]);
  digitalWrite(LATCH_PIN, HIGH);
}

/* ───────── PUBLIC API ───────── */
namespace DisplayDriver {

void begin()
{
  pinMode(LATCH_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, LOW);
  SPI.begin();
  SPI.beginTransaction(SPI_CFG);
}

/* ---------- RED 0-100 % ---------- */
void showStrength(uint8_t pct)
{
  if (pct>100) pct=100;
  uint8_t h=pct/100, t=(pct/10)%10, o=pct%10;

  uint8_t s[7]={SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK};
  if (h){ s[RED0]=RDIGIT[h]; s[RED1]=RDIGIT[t]; s[RED2]=RDIGIT[o]; }
  else if (t){ s[RED1]=RDIGIT[t]; s[RED2]=RDIGIT[o]; }
  else       { s[RED2]=RDIGIT[o]; }
  shiftBytes(s,7);
}

/* ---------- RED SS.t ---------- */
void showCurrentTime(unsigned long us)
{
  unsigned long tenths=(us+50000UL)/100000UL;
  if (tenths>999) tenths=999;
  uint8_t secs=tenths/10, tenth=tenths%10;
  uint8_t tens=secs/10, ones=secs%10;

  uint8_t s[7]={SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK};
  s[RED0]=(tens?RDIGIT[tens]:SEG_BLANK);
  s[RED1]=RDIGIT[ones]|(1<<R_DP);      // DP on middle red
  s[RED2]=RDIGIT[tenth];
  shiftBytes(s,7);
}

/* ---------- GREEN SS.hh ---------- */
void showBestTime(unsigned int centis)
{
  if (centis>9999) centis=9999;
  uint8_t secs=centis/100, hh=centis%100;
  uint8_t tens=secs/10, ones=secs%10, h1=hh/10, h2=hh%10;

  uint8_t s[7]={SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK};
  s[GREEN0]=(tens?GDIGIT[tens]:SEG_BLANK);
  s[GREEN1]=GDIGIT[ones]|(1<<G_DP);    // DP between seconds & hundredths
  s[GREEN2]=GDIGIT[h1];
  s[GREEN3]=GDIGIT[h2];
  shiftBytes(s,7);
}

/* ---------- Blank helpers ---------- */
void blankRedDigits()
{
  uint8_t s[7]={SEG_BLANK,SEG_BLANK,SEG_BLANK,
                0,0,0,0};              // leave greens unchanged
  shiftBytes(s,7);
}

void blankAll()
{
  uint8_t s[7]={SEG_BLANK,SEG_BLANK,SEG_BLANK,
                SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK};
  shiftBytes(s,7);
}

} // namespace
