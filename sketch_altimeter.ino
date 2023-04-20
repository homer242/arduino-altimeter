#include <stdarg.h>
#include <stdio.h>

static int buttonUp = 2; // buttonUp pin
static int buttonDown = 3; // buttonDown pin
static int calibAbsAltiCmOffset;
static bool hp20x_avlb;
KalmanFilter t_filter;    //temperature filter
KalmanFilter p_filter;    //pressure filter
KalmanFilter a_filter;    //altitude filter
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0,
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
  u8g2.clearBuffer();                   // clear the internal memory
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

  u8g2.sendBuffer();                    // transfer internal memory to the display
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
}

static int hp20x_setup(void)
{
  int res;

  /* Reset HP20x_dev*/
  HP20x.begin();
  delay(100);

  /* Determine HP20x_dev is available or not*/
  res = HP20x.isAvailable();

  return (res == OK_HP20X_DEV ? 0 : -1);
}

static void print_acq_result(const struct acq_result *result)
{
  char line1[128],
    line2[128],
    line3[128]/* ,
    line4[128] */;

  Serial.println("Temper:");
  Serial.print(result->temper);
  Serial.println("C.\n");
  Serial.println("Filter:");
  Serial.print(result->temper_filt);
  Serial.println("C.\n");

  Serial.println("Pressure:");
  Serial.print(result->pressure_hpa);
  Serial.println("hPa.\n");
  Serial.println("Filter:");
  Serial.print(result->pressure_hpa_filt);
  Serial.println("hPa\n");

  Serial.println("Altitude:");
  Serial.print(result->alti_cm);
  Serial.println("cm.\n");
  Serial.println("Filter:");
  Serial.print(result->alti_cm_filt);
  Serial.println("cm.\n");
  Serial.println("Filter with calib offset:");
  Serial.print(result->alti_cm_filt + calibAbsAltiCmOffset);
  Serial.println("cm.\n");
  Serial.println("------------------\n");

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

  prgState = state_idle;

  /* Control buttons */
  pinMode(buttonUp, INPUT);
  pinMode(buttonDown, INPUT);

  /* screen */
  screen_setup();

  /* HP20x (Altitude, pressure, temperature)
   * Power up,delay 150ms,until voltage is stable
   */
  delay(150);

  res = hp20x_setup();
  hp20x_avlb = (res == OK_HP20X_DEV);
  if (hp20x_avlb) {
    log_text("HP20x_dev is available.\n");
  } else {
    log_text("HP20x_dev isn't available.\n");
    screen_print_lines("Error!", "HP20x_dev isn't available!", NULL, NULL);
    prgState = state_error;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static int BtnStateUp,
    BtnStateDown;
  static unsigned int changeGuardMs;
  static unsigned long prevTimeMs;
  unsigned long newTimeMs,
    ElapsedTimeMs;
  int newBtnStateUp,
    newBtnStateDown;

  /* Read button states and delay the handling of new states */
  newBtnStateUp = digitalRead(buttonUp);
  newBtnStateDown = digitalRead(buttonDown);

  if (newBtnStateUp != BtnStateUp || newBtnStateDown != BtnStateDown) {
    changeGuardMs = 500;
  } else {
    changeGuardMs = 0;
  }

  newTimeMs = millis();
  ElapsedTimeMs = newTimeMs - prevTimeMs;
  prevTimeMs = millis();

  if (ElapsedTimeMs > changeGuardMs) {
    changeGuardMs = 0;
  } else {
    changeGuardMs -= ElapsedTimeMs;
  }

  if (changeGuardMs != 0) {
    delay(50 /* ms */);
    return;
  }

  BtnStateUp = newBtnStateUp;
  BtnStateDown = newBtnStateDown;

  switch(prgState) {
  case state_error:
    break;
  case state_idle:
    /* Enter calib mode */ 
    if (BtnStateUp == 0 && BtnStateDown == 0) {
      prgState = state_calib;
    } else if (BtnStateUp == 0 && BtnStateDown != 0) {
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
    if (BtnStateUp == 0 && BtnStateDown == 0) {
      prgState = state_idle;
    } else if (BtnStateUp == 0 && BtnStateDown != 0) {
      ++calibAbsAltiCmOffset;
      print_calib();
    } else if (BtnStateUp != 0 && BtnStateDown == 0) {
      --calibAbsAltiCmOffset;
      print_calib();
    }
  }
}
