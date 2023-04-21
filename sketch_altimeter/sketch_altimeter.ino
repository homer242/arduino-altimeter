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
  state_calib,
  state_error,
};

enum PrgEvent {
  event_loop,
  event_btn_pushed,
  event_print_acq,
  event_do_acq,
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
    u8g2.setFont(u8g2_font_luBIS08_tf);
    /* all graphics commands have to appear within the loop body. */ 
    if (line1 != NULL) {
      u8g2.drawStr(0, 15, line1);
    }
    if (line2 != NULL) {
      u8g2.drawStr(0, 30, line2);
    }
    if (line3 != NULL) {
      u8g2.drawStr(0, 45, line3);
    }
    if (line4 != NULL) {
      u8g2.drawStr(0, 60, line4);
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
  result->alti_cm_filt = a_filter.Filter(result->alti_cm);

  return 0;
}

static int screen_setup(void)
{
  u8g2.begin();

  return 0;
}

static int hp20x_setup(void)
{
  /* Reset HP20x_dev*/
  HP20x.begin();
  delay(100);

  return 0;
}

static void print_acq_result(const struct acq_result *result)
{
  char line1[32],
    line2[32],
    line3[32]/* ,
    line4[128] */;

  Serial.print(F("Temper: "));
  Serial.print(result->temper);
  Serial.print(F("C."));
  Serial.print(F(" - filtered:"));
  Serial.print(result->temper_filt);
  Serial.println(F("C."));

  Serial.print(F("Pressure: "));
  Serial.print(result->pressure_hpa);
  Serial.print(F("hPa."));
  Serial.print(F(" - filtered:"));
  Serial.print(result->pressure_hpa_filt);
  Serial.println(F("hPa"));

  Serial.print(F("Altitude: "));
  Serial.print(result->alti_cm);
  Serial.print(F("cm."));
  Serial.print(F(" - filtered and off:"));
  Serial.print(result->alti_cm_filt + calibAbsAltiCmOffset);
  Serial.println(F("cm."));
  
  Serial.println(F("------------------"));

  snprintf(line1, sizeof(line1), "%f C", result->temper_filt);
  snprintf(line2, sizeof(line2), "%f hPa", result->pressure_hpa_filt);
  snprintf(line3, sizeof(line3), "%f cm", result->alti_cm_filt + calibAbsAltiCmOffset);

  screen_print_lines(line1, line2, line3, NULL);
}

static void print_calib(void) {
  char line2[128];

  snprintf(line2, sizeof(line2), "cm_offset: %d\n", calibAbsAltiCmOffset);
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
    screen_print_lines("Error!", "hp20x_setup", NULL, NULL);
    prgState = state_error;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  static int BtnUpPushed,
    BtnDownPushed;
  static unsigned int changeGuardMs;
  static unsigned long last_btn_push_time_ms,
    last_acq_print_time_ms,
    last_acq_time_ms;
  static struct acq_result last_acq_result;
  static bool acq_result_avl;
  unsigned long newTimeMs;
  int newBtnUpPushed,
    newBtnDownPushed,
    res;
  enum PrgEvent prgEvent;

  do {
    newTimeMs = millis();
  } while (newTimeMs == 0); /* we want a value different to 0 */
  
  /* Read button states and delay the handling of new PUSHED states */
  newBtnUpPushed = (digitalRead(buttonUp) == 0);
  newBtnDownPushed = (digitalRead(buttonDown) == 0);
  //log_text("newBtnUpPushed:%d newBtnDownPushed:%d...\r\n", newBtnUpPushed, newBtnDownPushed);

  if (newBtnUpPushed != BtnUpPushed) {
    BtnUpPushed = newBtnUpPushed;
    last_btn_push_time_ms = newTimeMs;
  }
  
  if (newBtnDownPushed != BtnDownPushed) {
    BtnDownPushed = newBtnDownPushed;
    last_btn_push_time_ms = newTimeMs;
  }
  
  /* generate an event? */
  if (last_btn_push_time_ms != 0 && newTimeMs - last_btn_push_time_ms >= 500 /* ms */) {
    prgEvent = event_btn_pushed;
    last_btn_push_time_ms = 0;
  } else if (newTimeMs - last_acq_print_time_ms >= 1000 /* ms */) {
    prgEvent = event_print_acq;
    last_acq_print_time_ms = newTimeMs;
  } else if (newTimeMs - last_acq_time_ms >= 50 /* ms */) {
    prgEvent = event_do_acq;
    last_acq_time_ms = newTimeMs;
  } else {
    prgEvent = event_loop;
  }

  /* handling event */
  switch(prgState) {
  case state_error:
    break;
  case state_idle:
    if (prgEvent == event_do_acq) {
      res = do_acq(&last_acq_result);
      if (res == 0) {
        acq_result_avl = true;
      }
    } else if (prgEvent == event_print_acq) {
      if (acq_result_avl)
        print_acq_result(&last_acq_result);
    } else if (prgEvent == event_btn_pushed) {
      if (BtnUpPushed && BtnDownPushed) {
        /* Enter calib mode */ 
        prgState = state_calib;
      }
    }
    break;
  case state_calib:
    /* Exit calib mode */
    if (prgEvent == event_btn_pushed) {
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
  }

  /* reset for next loop */
  if (prgEvent == event_btn_pushed) {
    BtnUpPushed = 0;
    BtnDownPushed = 0;
  }
}
