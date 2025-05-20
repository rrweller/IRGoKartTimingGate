#include "display.h"
#include <SPI.h>

/* ──────────────────  SPI / latch  ────────────────── */
static const uint8_t LATCH_PIN = 10;
static const SPISettings SPI_CFG(4000000, MSBFIRST, SPI_MODE0);

/* ──────────────────  Segment maps  ──────────────────
   RED digits wiring (first 3 shift registers)
     D0→E  D1→D  D2→C  D3→DP  D4→B  D5→A  D6→F  D7→G
   GREEN digits wiring (next 4 shift registers)
     D0→E  D1→D  D2→C  D3→B  D4→A  D5→DP  D6→F  D7→G           */
static const uint8_t R_A=5, R_B=4, R_C=2, R_D=1, R_E=0, R_F=6, R_G=7, R_DP=3;
static const uint8_t G_A=4, G_B=3, G_C=2, G_D=1, G_E=0, G_F=6, G_G=7, G_DP=5;

/* Shift-register byte order */
enum DigitIdx { RED0=0, RED1, RED2, GREEN0, GREEN1, GREEN2, GREEN3 };

/* helpers to build patterns */
#define RMASK(a,b,c,d,e,f,g,dp) \
  ((a?1<<R_A:0)|(b?1<<R_B:0)|(c?1<<R_C:0)|(d?1<<R_D:0)| \
   (e?1<<R_E:0)|(f?1<<R_F:0)|(g?1<<R_G:0)|(dp?1<<R_DP:0))

#define GMASK(a,b,c,d,e,f,g,dp) \
  ((a?1<<G_A:0)|(b?1<<G_B:0)|(c?1<<G_C:0)|(d?1<<G_D:0)| \
   (e?1<<G_E:0)|(f?1<<G_F:0)|(g?1<<G_G:0)|(dp?1<<G_DP:0))

/* digit tables (DP off) */
static const uint8_t RDIG[10]={
  RMASK(1,1,1,1,1,1,0,0), RMASK(0,1,1,0,0,0,0,0),
  RMASK(1,1,0,1,1,0,1,0), RMASK(1,1,1,1,0,0,1,0),
  RMASK(0,1,1,0,0,1,1,0), RMASK(1,0,1,1,0,1,1,0),
  RMASK(1,0,1,1,1,1,1,0), RMASK(1,1,1,0,0,0,0,0),
  RMASK(1,1,1,1,1,1,1,0), RMASK(1,1,1,1,0,1,1,0)
};
static const uint8_t GDIG[10]={
  GMASK(1,1,1,1,1,1,0,0), GMASK(0,1,1,0,0,0,0,0),
  GMASK(1,1,0,1,1,0,1,0), GMASK(1,1,1,1,0,0,1,0),
  GMASK(0,1,1,0,0,1,1,0), GMASK(1,0,1,1,0,1,1,0),
  GMASK(1,0,1,1,1,1,1,0), GMASK(1,1,1,0,0,0,0,0),
  GMASK(1,1,1,1,1,1,1,0), GMASK(1,1,1,1,0,1,1,0)
};
static const uint8_t RDASH      = RMASK(0,0,0,0,0,0,1,0);   // segment-G only, for ready mode
static const uint8_t SEG_BLANK = 0;

/* ---------- persistent frame buffer ---------- */
static uint8_t frame[7] = { SEG_BLANK,SEG_BLANK,SEG_BLANK,
                            SEG_BLANK,SEG_BLANK,SEG_BLANK,SEG_BLANK };

/* low-level shift-out */
static void pushFrame()
{
  digitalWrite(LATCH_PIN, LOW);
  for (int8_t i=6; i>=0; --i) SPI.transfer(frame[i]);
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
  pushFrame();                           // all blank at power-up
}

/* ------ RED 0-100% ------ */
void showStrength(uint8_t pct)
{
  if (pct>100) pct=100;
  uint8_t h=pct/100, t=(pct/10)%10, o=pct%10;

  frame[RED0]= (h?RDIG[h]:SEG_BLANK);
  frame[RED1]= (h?RDIG[t]:(t?RDIG[t]:SEG_BLANK));
  frame[RED2]= RDIG[o];
  pushFrame();
}

void showReady(bool on)
{
  frame[RED0]=frame[RED1]=frame[RED2]= (on? RDASH : SEG_BLANK);
  pushFrame();
}

/* ------ RED SS.t ------ */
void showCurrentTime(unsigned long us)
{
  unsigned long tenths=(us+50000UL)/100000UL;
  if (tenths>999) tenths=999;
  uint8_t secs=tenths/10, tenth=tenths%10;
  uint8_t tens=secs/10, ones=secs%10;

  frame[RED0]= (tens?RDIG[tens]:SEG_BLANK);
  frame[RED1]= RDIG[ones] | (1<<R_DP);
  frame[RED2]= RDIG[tenth];
  pushFrame();
}

/* ------ GREEN SS.hh ------ */
void showBestTime(unsigned int centis)
{
  if (centis>9999) centis=9999;
  uint8_t secs=centis/100, hh=centis%100;
  uint8_t tens=secs/10, ones=secs%10, h1=hh/10, h2=hh%10;

  frame[GREEN0]= (tens?GDIG[tens]:SEG_BLANK);
  frame[GREEN1]= GDIG[ones] | (1<<G_DP);
  frame[GREEN2]= GDIG[h1];
  frame[GREEN3]= GDIG[h2];
  pushFrame();
}

/* ------ Blank helpers ------ */
void blankRedDigits()
{
  frame[RED0]=frame[RED1]=frame[RED2]=SEG_BLANK;
  pushFrame();
}

void blankAll()
{
  for (uint8_t &b : frame) b=SEG_BLANK;
  pushFrame();
}

} // namespace
