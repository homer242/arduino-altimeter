#include <stdarg.h>
#include <stdio.h>

#include <HP20x_dev.h>
#include <KalmanFilter.h>
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

static const int buttonUp = 2; // (D2) buttonUp pin
static const int buttonDown = 3; // (D2) buttonDown pin

static int calibAbsAltiCmOffset;
KalmanFilter t_filter;    //temperature filter
KalmanFilter p_filter;    //pressure filter
KalmanFilter a_filter;    //altitude filter
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0,
                                         /* clock=*/ SCL,
                                         /* data=*/ SDA,
                                         /* reset=*/ U8X8_PIN_NONE);    //Software I2C

enum FsmState {
  state_idle,
  state_acq,
  state_calib,
  state_error,
};

struct acq_result {
  float alti_cm;
  float alti_cm_filt;
  float temper;
  float temper_filt;
  float pressure_hpa;
  float pressure_hpa_filt;
};

static enum FsmState prgState;

static int screen_print_lines(const char *line1, const char *line2,
                              const char *line3, const char *line4)
{
  u8g2.firstPage();
  do {
    /* all graphics commands have to appear within the loop body. */    
    u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font
  
    if (line1 != NULL) {
      u8g2.drawStr(0, 10, line1);
    }
    if (line2 != NULL) {
      u8g2.drawStr(0, 20, line2);
    }
    if (line3 != NULL) {
      u8g2.drawStr(0, 30, line3);
    }
    if (line4 != NULL) {
      u8g2.drawStr(0, 40, line4);
    }
  } while (u8g2.nextPage());

  return 0;
}

static void log_text(const char *fmt, ...)
{
  char buf[128];
  va_list args;

  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  Serial.print(buf);
}

static int do_acq(struct acq_result *result)
{
  result->temper = HP20x.ReadTemperature() / 100.0f;
  result->pressure_hpa = HP20x.ReadPressure() / 100.0f;
  result->alti_cm = HP20x.ReadAltitude();

  result->temper_filt = t_filter.Filter(result->temper);
  result->pressure_hpa_filt = p_filter.Filter(result->pressure_hpa);
  result->alti_cm_filt = p_filter.Filter(result->alti_cm);

  return 0;
}

static int screen_setup(void)
{
  u8g2.begin();

  return 0;
}

static int hp20x_setup(void)
{
  int res;

  /* Reset HP20x_dev*/
  HP20x.begin();
  delay(100);

  return 0;
}

static void print_acq_result(const struct acq_result *result)
{
  char line1[128],
    line2[128],
    line3[128]/* ,
    line4[128] */;

  Serial.print(F("Temper: "));
  Serial.print(result->temper);
  Serial.println(F("C."));
  /*Serial.println(F("Filter:"));
  Serial.print(result->temper_filt);
  Serial.println(F("C.\n"));*/

  Serial.print(F("Pressure: "));
  Serial.print(result->pressure_hpa);
  Serial.println(F("hPa."));
  /*Serial.println(F("Filter:"));
  Serial.print(result->pressure_hpa_filt);
  Serial.println(F("hPa\n"));*/

  Serial.print(F("Altitude: "));
  Serial.print(result->alti_cm);
  Serial.println(F("cm."));
  /*Serial.println(F("Filter:"));
  Serial.print(result->alti_cm_filt);
  Serial.println(F("cm.\n"));
  Serial.println(F("Filter with calib offset:"));
  Serial.print(result->alti_cm_filt + calibAbsAltiCmOffset);
  Serial.println(F("cm.\n"));*/
  Serial.println(F("------------------"));

  snprintf(line1, sizeof(line1), "temper: %f C", result->temper_filt);
  snprintf(line2, sizeof(line2), "pressure: %f hPa", result->pressure_hpa_filt);
  snprintf(line3, sizeof(line3), "altitude: %f cm", result->alti_cm_filt + calibAbsAltiCmOffset);

  screen_print_lines(line1, line2, line3, NULL);
}

static void print_calib(void) {
  char line2[128];

  snprintf(line2, sizeof(line2), "alti_cm_offset: %d\n", calibAbsAltiCmOffset);
  screen_print_lines("Calib settings:", line2, NULL, NULL);
}

void setup() {
  int res;

  Serial.begin(9600);
  log_text("booting...\r\n");

  prgState = state_idle;

  /* Control buttons */
  pinMode(buttonUp, INPUT);
  pinMode(buttonDown, INPUT);

  /* screen */
  screen_setup();
  screen_print_lines("Booting!", "Hello!", NULL, NULL);

  /* HP20x (Altitude, pressure, temperature)
   * Power up,delay 150ms,until voltage is stable
   */
  delay(150);

  res = hp20x_setup();
  if (res != 0) {
    log_text("hp20x_setup() failed!\r\n");
    screen_print_lines("Error!", "cannot setup hp20x device!", NULL, NULL);
    prgState = state_error;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static int BtnUpPushed,
    BtnDownPushed;
  static unsigned int changeGuardMs;
  static unsigned long prevTimeMs,
    last_acq_time_ms;
  struct acq_result result;
  unsigned long newTimeMs,
    ElapsedTimeMs;
  int newBtnUpPushed,
    newBtnDownPushed,
    processNewBtnStates,
    res;

  /* Read button states and delay the handling of new PUSHED states */
  newBtnUpPushed = (digitalRead(buttonUp) == 0);
  newBtnDownPushed = (digitalRead(buttonDown) == 0);
  //log_text("newBtnUpPushed:%d newBtnDownPushed:%d...\r\n", newBtnUpPushed, newBtnDownPushed);

  if (newBtnUpPushed != BtnUpPushed) {
    BtnUpPushed = newBtnUpPushed;
    changeGuardMs = 500;
  } else if (newBtnDownPushed != BtnDownPushed) {
    BtnDownPushed = newBtnDownPushed;
    changeGuardMs = 500;
  }

  newTimeMs = millis();
  ElapsedTimeMs = newTimeMs - prevTimeMs;
  prevTimeMs = millis();

  processNewBtnStates = 0;

  if (changeGuardMs > 0) {
    if (ElapsedTimeMs >= changeGuardMs) {
      changeGuardMs = 0;
      processNewBtnStates = 1;
    } else {
      changeGuardMs -= ElapsedTimeMs;
    }
  }

  if (!processNewBtnStates) {
    delay(50 /* ms */);
    return;
  }

  log_text("BtnUpPushed:%d BtnDownPushed:%d\r\n", BtnUpPushed, BtnDownPushed);

  switch(prgState) {
  case state_error:
    break;
  case state_idle:
    /* Enter calib mode */ 
    if (BtnUpPushed && BtnDownPushed) {
      prgState = state_calib;
    } else if (BtnUpPushed) {
      prgState = state_acq;
    }
    break;
  case state_acq:
    res = do_acq(&result);
    if (res == 0) {
      print_acq_result(&result);
    }
    prgState = state_idle;
    break;
  case state_calib:
    /* Exit calib mode */
    if (BtnUpPushed && BtnDownPushed) {
      prgState = state_idle;
    } else if (BtnUpPushed) {
      ++calibAbsAltiCmOffset;
      print_calib();
    } else if (BtnDownPushed) {
      --calibAbsAltiCmOffset;
      print_calib();
    }
  }

  BtnUpPushed = 0;
  BtnDownPushed = 0;
}
