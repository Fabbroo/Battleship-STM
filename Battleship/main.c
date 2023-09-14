#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "ssd1306.h"
#include "stdio.h"
#include "time.h"
#include "stdlib.h"
#include "stdbool.h"
#include <string.h>
#include "notes.h"

#define BUFF_SIZE   20
char buff[BUFF_SIZE];

// map sizes
#define MAP_HEIGHT 4
#define MAP_WIDTH 5
#define SQUARE_LENGTH 11
#define DISTANCE 10

// player map delimitations
#define UPPER_LIMIT 16
#define LEFT_LIMIT 0
#define BOTTOM_LIMIT UPPER_LIMIT+MAP_HEIGHT*SQUARE_LENGTH
#define RIGHT_LIMIT LEFT_LIMIT+MAP_WIDTH*SQUARE_LENGTH

// enemy map delimitations
#define ENEMY_UPPER_LIMIT 16
#define ENEMY_LEFT_LIMIT RIGHT_LIMIT+DISTANCE
#define ENEMY_BOTTOM_LIMIT BOTTOM_LIMIT
#define ENEMY_RIGHT_LIMIT ENEMY_LEFT_LIMIT+MAP_WIDTH*SQUARE_LENGTH

// player
#define PLAYER 0

// enemy
#define ENEMY 1

// numero barche
#define BOATS_NUMBER 5

// punto
typedef struct{
  bool occupied;
  bool checked;
  uint8_t x;
  uint8_t y;
}Point;

// map
typedef struct{
  Point map[MAP_HEIGHT][MAP_WIDTH];
}Map;

// cursor
typedef struct{
  Point position;
  bool display;
}Cursor;

// mutex resources
typedef struct{
  mutex_t mutex;
  Map maps[2];
  Cursor cursor;
  bool initialized;
  uint8_t myScore;
  uint8_t enemyScore;
}MutexPlayers;


// cursor length
#define CURSOR_LENGTH 7

// CASSE
#define PORTAB_LINE_LED1            LINE_LED
#define PORTAB_LED_OFF              PAL_LOW
#define PORTAB_LED_ON               PAL_HIGH
#define PORTAB_LINE_BUTTON          LINE_BUTTON
#define PORTAB_BUTTON_PRESSED       PAL_HIGH
#define PORTAB_SD1                  LPSD1
#define PORTAB_DAC_TRIG             7

size_t nx = 0, ny = 0, nz = 0;
static void end_cb1(DACDriver *dacp) {

  nz++;
  if (dacIsBufferComplete(dacp)) {
    nx += DAC_BUFFER_SIZE / 4;
  }
  else {
    ny += DAC_BUFFER_SIZE / 4;
  }

  if ((nz % 1000) == 0) {
    palToggleLine(PORTAB_LINE_LED1);
  }
}

/*
 * DAC error callback.
 */
static void error_cb1(DACDriver *dacp, dacerror_t err) {

  (void)dacp;
  (void)err;

  chSysHalt("DAC failure");
}

static const DACConfig dac1cfg1 = {
  .init         = 2047U,
  .datamode     = DAC_DHRM_12BIT_RIGHT,
  .cr           = 0
};

static const DACConversionGroup dacgrpcfg1 = {
  .num_channels = 1U,
  .end_cb       = end_cb1,
  .error_cb     = error_cb1,
  .trigger      = DAC_TRG(PORTAB_DAC_TRIG)
};

/*
 * GPT6 configuration.
 */
static GPTConfig SI = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_B4,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,    /* MMS = 010 = TRGO on Update Event.    */
  .dier         = 0U
};
static GPTConfig MI = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_E5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig FA = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_F5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig LA = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_AS4,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig SOL = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_G5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig RE = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_D5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig DO = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_C5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig LA1 = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_A4,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig RES = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_DS5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig SOLS = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_GS5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig FAS = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_FS5,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};
static GPTConfig SOLS1 = {
  .frequency    = 2*DAC_BUFFER_SIZE*NOTE_GS4,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,
  .dier         = 0U
};


// SENSOR
#define IR_SENSOR_PIN   PAL_LINE(GPIOB, 3)

// JOYPAD
#define VOLTAGE_RES            ((float)3.3/4096) //4095 : 3.3 = sample : x

#define MSG_ADC_OK               0x1337
#define MSG_ADC_KO               0x7331

static thread_reference_t trp = NULL;
/*
 * ADC streaming callback.
 */

static void adccallback(ADCDriver *adcp) {
  if (adcIsBufferComplete(adcp)) {
    chSysLockFromISR();
    chThdResumeI(&trp, (msg_t) MSG_ADC_OK );  /* Resuming the thread with message 0x1337.*/
    chSysUnlockFromISR();
  }
}

/*
 * ADC error callback.
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
  chSysLockFromISR();
  chThdResumeI(&trp, (msg_t) MSG_ADC_KO );  /* Resuming the thread with message 0x7331.*/
  chSysUnlockFromISR();
}

#define ADC_GRP_NUM_CHANNELS        2
#define ADC_GRP_BUF_DEPTH           16
static adcsample_t samples[ADC_GRP_NUM_CHANNELS * ADC_GRP_BUF_DEPTH];

/*
 * ADC conversion group.
 * Mode:        Continuous on 2 channels, SW triggered.
 * Channels:    IN1 (GPIOA4), IN2 (GPIOA5)
 */

static const ADCConversionGroup adcgrpcfg = {
          .circular     = false,
          .num_channels = ADC_GRP_NUM_CHANNELS,
          .end_cb       = adccallback,
          .error_cb     = adcerrorcallback,
          .cfgr         = ADC_CFGR_CONT,
          .cfgr2        = 0U,
          .tr1          = ADC_TR_DISABLED,
          .tr2          = ADC_TR_DISABLED,
          .tr3          = ADC_TR_DISABLED,
          .awd2cr       = 0U,
          .awd3cr       = 0U,
          .smpr         = {
            ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_247P5) |
            ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_247P5),0U
          },
          .sqr          = {
            ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN2),
            0U,
            0U,
            0U
          }
        };

static float converted[ADC_GRP_NUM_CHANNELS];

#define LINE_X PAL_LINE(GPIOA, 0)
#define LINE_Y PAL_LINE(GPIOA, 1)
#define LINE_joystickBut PAL_LINE(GPIOB, 7)
#define LINE_BLUE PAL_LINE(GPIOB, 4)

// DISPLAY
static const I2CConfig i2ccfg = {
  // I2C_TIMINGR address offset
  .timingr = 0x10,
  .cr1 = 0,
  .cr2 = 1,
};

static const SSD1306Config ssd1306cfg = {
  &I2CD1,
  &i2ccfg,
  SSD1306_SAD_0X78,
};

static SSD1306Driver SSD1306D1;

// MOSTRA MAPPA PLAYER
void displayPlayerMap(Point map[][MAP_WIDTH]){

  for(int i = 0; i<MAP_HEIGHT; i++){

    for(int j = 0; j<MAP_WIDTH; j++){

      ssd1306DrawRectangle(&SSD1306D1, map[i][j].x, map[i][j].y, SQUARE_LENGTH, SQUARE_LENGTH, SSD1306_COLOR_WHITE);

      if(map[i][j].occupied == true){

        ssd1306DrawRectangleFill(&SSD1306D1, map[i][j].x + 4, map[i][j].y + 4, 3, 3, SSD1306_COLOR_WHITE);
      }

    }
  }
}

// MOSTRA MAPPA NEMICA
void displayEnemyMap(Point map[][MAP_WIDTH]){
  for(int i = 0; i<MAP_HEIGHT; i++){
    for(int j = 0; j<MAP_WIDTH; j++){

      ssd1306DrawRectangle(&SSD1306D1, map[i][j].x, map[i][j].y, SQUARE_LENGTH, SQUARE_LENGTH, SSD1306_COLOR_WHITE);

      if(map[i][j].occupied == false && map[i][j].checked == true){

        ssd1306DrawLine(&SSD1306D1, map[i][j].x + 3, map[i][j].y + 3, map[i][j].x + SQUARE_LENGTH - 3, map[i][j].y + SQUARE_LENGTH - 3, SSD1306_COLOR_WHITE);
        ssd1306DrawLine(&SSD1306D1, map[i][j].x + SQUARE_LENGTH - 3, map[i][j].y + 3, map[i][j].x + 3, map[i][j].y + SQUARE_LENGTH - 3, SSD1306_COLOR_WHITE);

      }else if(map[i][j].occupied == true && map[i][j].checked == true){

        ssd1306DrawRectangleFill(&SSD1306D1, map[i][j].x + 4, map[i][j].y + 4, 3, 3, SSD1306_COLOR_WHITE);

      }

    }
  }
}

// MOSTRA CURSORE
void displayCursor(Point position){

  Point drawingPosition;
  drawingPosition.x = position.x + 2;
  drawingPosition.y = position.y + 2;

  // cursor square
  ssd1306DrawRectangle(&SSD1306D1, drawingPosition.x, drawingPosition.y, CURSOR_LENGTH, CURSOR_LENGTH, SSD1306_COLOR_WHITE);

  // up
  ssd1306DrawRectangle(&SSD1306D1, drawingPosition.x + 2, drawingPosition.y, 3, 1, SSD1306_COLOR_BLACK);
  // down
  ssd1306DrawRectangle(&SSD1306D1, drawingPosition.x + 2, drawingPosition.y + CURSOR_LENGTH, 3, 1, SSD1306_COLOR_BLACK);
  // left
  ssd1306DrawRectangle(&SSD1306D1, drawingPosition.x, drawingPosition.y + 2, 1, 3, SSD1306_COLOR_BLACK);
  //right
  ssd1306DrawRectangle(&SSD1306D1, drawingPosition.x + CURSOR_LENGTH, drawingPosition.y + 2, 1, 3, SSD1306_COLOR_BLACK);
}

static THD_WORKING_AREA(waSpeaker, 1024);
static THD_FUNCTION(Speaker, arg){
  (void)arg;

  while(true){
    dacStartConversion(&DACD1, &dacgrpcfg1, (dacsample_t *)dac_buffer, DAC_BUFFER_SIZE);

    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(400);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(200);

    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(700);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(200);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &SOLS);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &SOL);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(600);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(200);
    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(400);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(500);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(500);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &SOLS);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(300);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &SOL);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(300);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(200);
    gptStopTimer(&GPTD6);
    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(70);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(400);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(700);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(200);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(500);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &SOLS1);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(400);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &SOL);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(300);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(500);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(200);

    gptStart(&GPTD6, &RE);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(100);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(200);

    gptStart(&GPTD6, &RE);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(80);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RE);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(80);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RE);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(80);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RE);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(80);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &DO);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(400);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(300);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &DO);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(800);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(200);

    gptStart(&GPTD6, &SOLS1);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(400);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(600);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &LA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(400);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(600);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(80);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(300);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &SOL);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(750);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(400);

    gptStart(&GPTD6, &FA);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(350);
    gptStopTimer(&GPTD6);

    gptStart(&GPTD6, &RES);
    gptStartContinuous(&GPTD6, 2U);
    chThdSleepMilliseconds(350);
    gptStopTimer(&GPTD6);
    chThdSleepMilliseconds(400);

    chThdSleepMilliseconds(500);
  }
}


// THREAD DISPLAY
static THD_WORKING_AREA(waOledDisplay, 2048);
static THD_FUNCTION(OledDisplay, arg) {

  MutexPlayers* players = (MutexPlayers*) arg;

  chRegSetThreadName("OledDisplay");

  /*
   * Initialize SSD1306 Display Driver Object.
   */
  ssd1306ObjectInit(&SSD1306D1);

  /*
   * Start the SSD1306 Display Driver Object with
   * configuration.
   */
  ssd1306Start(&SSD1306D1, &ssd1306cfg);

  while (true) {

    ssd1306FillScreen(&SSD1306D1, 0x00);

    chMtxLock(&players->mutex);

    if(players->initialized==false){

      ssd1306GotoXy(&SSD1306D1, 43, 1);
      chsnprintf(buff, BUFF_SIZE, "MENU");
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

      ssd1306GotoXy(&SSD1306D1, 15, 40);
      chsnprintf(buff, BUFF_SIZE, "INSERT TO START");
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_7x10, SSD1306_COLOR_WHITE);

    }else if(players->myScore < BOATS_NUMBER && players->enemyScore < BOATS_NUMBER){

      ssd1306GotoXy(&SSD1306D1, 15, 1);
      chsnprintf(buff, BUFF_SIZE, "YOU: %u  ENEMY: %u", players->myScore, players->enemyScore);
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_7x10, SSD1306_COLOR_WHITE);

      displayPlayerMap(players->maps[PLAYER].map);

      displayEnemyMap(players->maps[ENEMY].map);

      if(players->cursor.display == true)
        displayCursor(players->cursor.position);

      players->cursor.display = !(players->cursor.display);

    }else if(players->myScore < BOATS_NUMBER){
      ssd1306GotoXy(&SSD1306D1, 15, 40);
      chsnprintf(buff, BUFF_SIZE, "YOU LOST :(");
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);
    }else if(players->myScore == BOATS_NUMBER){
      ssd1306GotoXy(&SSD1306D1, 15, 40);
      chsnprintf(buff, BUFF_SIZE, "YOU WON :)");
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);
    }

    chMtxUnlock(&players->mutex);

    ssd1306UpdateScreen(&SSD1306D1);

    chThdSleepMilliseconds(500);
  }
}

void clearMap(Point map[][MAP_WIDTH]){
  for(int i=0; i<MAP_HEIGHT; i++){
    for(int j=0; j<MAP_WIDTH; j++){
      map[i][j].occupied = false;
      map[i][j].checked = false;
    }
  }
}

// GENAZIONE MAPPA
void generateMap(Point map[][MAP_WIDTH], uint8_t x, uint8_t y){

  uint8_t start = x;

  // CREAZIONE VERTICI DEI QUADRATI PER LA MAPPA
  for(int i=0; i<MAP_HEIGHT; i++){
    for(int j=0; j<MAP_WIDTH; j++){
      map[i][j].x = x;
      map[i][j].y = y;
      map[i][j].occupied = false;
      map[i][j].checked = false;
      x += SQUARE_LENGTH;
    }

    x = start;
    y += SQUARE_LENGTH;
  }
}

// GENERAZIONE BARCHE
void generateBoats(Point map[][MAP_WIDTH]){
  for(int n = 0; n<BOATS_NUMBER; n++){

    uint8_t indexX = 0;
    uint8_t indexY = 0;

    do{
      indexY = (uint8_t) (rand() % MAP_HEIGHT);
      indexX = (uint8_t) (rand() % MAP_WIDTH);
    }while(map[indexY][indexX].occupied == true);

    map[indexY][indexX].occupied = true;
  }
}

int newX = 0;
int newY = 0;

void joypadInput(MutexPlayers* players){

  bool selected = false;
  while( !selected ) {

    msg_t msg;
    float chX, chY;

    chSysLock();
    adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP_BUF_DEPTH);
    msg = chThdSuspendS(&trp);
    chSysUnlock();

      /* Check if acquisition is KO */
      if( msg == MSG_ADC_KO ) {
        continue;
      }

      /*
       * Clean the buffer
       */
      for(int i = 0; i < ADC_GRP_NUM_CHANNELS; i++ ) {
        converted[i] = 0.0f;
      }

      for(int i = 0; i < ADC_GRP_NUM_CHANNELS * ADC_GRP_BUF_DEPTH; i++ ) {
        converted[ i % ADC_GRP_NUM_CHANNELS] += (float) samples[i] * VOLTAGE_RES; //voltage res = 3.3/4095
      }

      for(int i = 0; i < ADC_GRP_NUM_CHANNELS; i++ ) {
        converted[i] = converted[i] / ADC_GRP_BUF_DEPTH;
      }

      /* Copy converted values into a new variable */
      chX = converted[0];
      chY = converted[1];

      /* Update output! */

      //destra
      if( chX < 0.035f && chY > 0.25f ) {
        newX = (newX + 1) % MAP_WIDTH;
      }

      //sotto
      if( chX > 0.1f && chY > 3.27f)  {
        newY = (newY + 1) % MAP_HEIGHT;
      }

      //sinistra
      if( chX > 3.259f && chY > 0.5f) {

        newX--;

        if(newX < 0){
          newX = MAP_WIDTH - 1;
        }

      }

      //sopra
      if( chX > 0.15f && chY < 0.035f) {
        newY--;

        if(newY < 0){
          newY = MAP_HEIGHT - 1;
        }
      }

      chMtxLock(&players->mutex);
      players->cursor.position = players->maps[ENEMY].map[newY][newX];
      chMtxUnlock(&players->mutex);

      if(palReadLine(LINE_BLUE) == PAL_HIGH){
        selected = true;
      }

      chThdSleepMilliseconds(1000);
    }
}

void myTurn(MutexPlayers* players, SerialDriver* sd1){
  joypadInput(players);

  uint8_t targetBool = 0;
  uint8_t sendX = (uint8_t) newX;
  uint8_t sendY = (uint8_t) newY;

  sdWrite(sd1, &sendX, sizeof(uint8_t));
  sdWrite(sd1, &sendY, sizeof(uint8_t));

  sdRead(sd1, &targetBool, sizeof(uint8_t));


  chMtxLock(&players->mutex);

  if(targetBool == 1 && players->maps[ENEMY].map[newY][newX].checked == false){
    (players->myScore)++;
    players->maps[ENEMY].map[newY][newX].occupied = true;
  }

  players->maps[ENEMY].map[newY][newX].checked = true;

  chMtxUnlock(&players->mutex);
}

void enemyTurn(MutexPlayers* players, SerialDriver* sd1){

  uint8_t enemyX = 0;
  uint8_t enemyY = 0;

  sdRead(sd1, &enemyX, sizeof(uint8_t));
  sdRead(sd1, &enemyY, sizeof(uint8_t));

  uint8_t sendBool = (uint8_t) players->maps[PLAYER].map[enemyY][enemyX].occupied;

  chMtxLock(&players->mutex);

  if(sendBool == 1 && players->maps[PLAYER].map[enemyY][enemyX].checked == false){
    (players->enemyScore)++;
  }

  players->maps[PLAYER].map[enemyY][enemyX].checked = true;

  chMtxUnlock(&players->mutex);

  sdWrite(sd1, &sendBool, sizeof(uint8_t));
}

int main(void) {

  halInit();
  chSysInit();

  /* Configuring I2C related PINs */
   palSetLineMode(PAL_LINE(GPIOB, 8U), PAL_MODE_ALTERNATE(4) |
                  PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST |
                  PAL_STM32_PUPDR_PULLUP);
   palSetLineMode(PAL_LINE(GPIOB, 9U), PAL_MODE_ALTERNATE(4) |
                  PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST |
                  PAL_STM32_PUPDR_PULLUP);

   // start joystick
   adcStart(&ADCD1, NULL);
   dacStart(&DACD1, &dac1cfg1);

   palSetPadMode( GPIOA, 2, PAL_MODE_ALTERNATE(7) );
   palSetPadMode( GPIOA, 3, PAL_MODE_ALTERNATE(7) );
   palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);


   //Setting input mode joystick
   palSetLineMode(LINE_X, PAL_MODE_INPUT_ANALOG);
   palSetLineMode(LINE_Y, PAL_MODE_INPUT_ANALOG);
   palSetLineMode(LINE_joystickBut, PAL_MODE_INPUT_PULLUP);
   palSetLineMode(LINE_BLUE, PAL_MODE_INPUT_PULLDOWN);
   palSetLineMode(IR_SENSOR_PIN, PAL_MODE_INPUT_PULLUP);

   //Comunication
   palSetPadMode( GPIOA, 9, PAL_MODE_ALTERNATE(7) );//TX
   palSetPadMode( GPIOA, 10, PAL_MODE_ALTERNATE(7) );//RX

   SerialDriver * sd1 = &SD1;

   sdStart(sd1, NULL);

   uint32_t tick = 0;
   MutexPlayers players;
   players.cursor.display = true;
   players.initialized = false;

   chMtxObjectInit(&players.mutex);

   chThdCreateStatic(waOledDisplay, sizeof(waOledDisplay), NORMALPRIO, OledDisplay, (void*) &players);
   chThdCreateStatic(waSpeaker, sizeof(waSpeaker), NORMALPRIO + 1, Speaker, NULL);

   while(true){


     chMtxLock(&players.mutex);
     players.initialized = false;
     players.myScore = 0;
     players.enemyScore = 0;
     chMtxUnlock(&players.mutex);

     uint8_t player = 0;
     uint8_t message = 2;

     // ATTENDI MONETA BUTTON
     while(player == 0){
       sdReadTimeout(sd1, &player, sizeof(player), TIME_IMMEDIATE);
       tick = (tick + 1) % 1000;
       chThdSleepMilliseconds(20);

       if(palReadLine(IR_SENSOR_PIN) == PAL_LOW){
         sdWrite(sd1, &message, sizeof(message));
         player = 1;
         chThdSleepMilliseconds(20);
       }

       // ATTENDI RILASCIO
       while(palReadLine(IR_SENSOR_PIN) == PAL_LOW){
         chThdSleepMilliseconds(20);
       }
     }

     // PULISCI MAPPA PLAYER
     clearMap(players.maps[PLAYER].map);

     // GENERA I PUNTI DELLA MAPPA
     generateMap(players.maps[PLAYER].map, LEFT_LIMIT, UPPER_LIMIT);
     generateMap(players.maps[ENEMY].map, ENEMY_LEFT_LIMIT, ENEMY_UPPER_LIMIT);

     players.cursor.position.x = players.maps[ENEMY].map[newY][newX].x;
     players.cursor.position.y = players.maps[ENEMY].map[newY][newX].y;

     // GENERAZIONE BARCHE PLAYER E NEMICO
     srand(tick);
     generateBoats(players.maps[PLAYER].map);

     // INIZIALIZZAZIONE COMPLETATA, IL THREAD INIZIERA' A STAMPARE
     chMtxLock(&players.mutex);
     players.initialized = true;
     chMtxUnlock(&players.mutex);


     // INIZIA LA PARTITA

     if(player==1){

       while(players.myScore<BOATS_NUMBER && players.enemyScore<BOATS_NUMBER){

         /*while(true){
            joypadInput(&players);

            chThdSleepMilliseconds(2000);
          }*/

         // TURNO INVIO
         myTurn(&players, sd1);

         // TURNO RICEZIONE
         if(players.myScore<BOATS_NUMBER){
           enemyTurn(&players, sd1);
         }

       }

     }else if(player == 2){

      while(players.myScore<BOATS_NUMBER && players.enemyScore<BOATS_NUMBER){

          // TURNO RICEZIONE
          enemyTurn(&players, sd1);

          // TURNO INVIO
          if(players.enemyScore<BOATS_NUMBER){
            myTurn(&players, sd1);
          }
      }
    }

     chThdSleepMilliseconds(5000);
  }

   sdStop(sd1);
}

