#include <Arduino.h>
#include <sbus.h> // Bolder Flight Systems SBUS
#include <Servo_megaTinyCore.h>
#include <SoftwareSerial.h>

#define RFWD PIN_PA5
#define RREV PIN_PA4

#define LFWD PIN_PB0
#define LREV PIN_PB1

#define BLUE PIN_PA6
#define RED PIN_PA7

#define TXDEBUG PIN_PC3
#define RXDEBUG PIN_PC2

#define CONTROL_MOTION_TIMEOUT_MS 3000

bfs::SbusRx sbus_rx(&Serial);
bfs::SbusData data;

SoftwareSerial dbgSerial(RXDEBUG, TXDEBUG);

unsigned long control_checksum;
unsigned long last_changed_at;
boolean startup = true;

Servo motor;

void setup() {
  dbgSerial.begin(9600);

  delay(50);

  dbgSerial.println("setup begin");

  motor.attach(PIN_PC0);

  pinMode(RFWD, OUTPUT);
  pinMode(RREV, OUTPUT);

  pinMode(LFWD, OUTPUT);
  pinMode(LREV, OUTPUT);

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RED, HIGH);

  analogWrite(LREV, 0);
  analogWrite(RREV, 0);
  analogWrite(LFWD, 0);
  analogWrite(RFWD, 0);

  sbus_rx.Begin();

  dbgSerial.println("setup done");
}

bool rc_poll() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    return true;
  }

  return false;
}

int get_ch(int ch) {
  // 11-bit channel, range from 172-1811
  int pulse_len = data.ch[ch];

  // reduce to 0..1639
  pulse_len -= 172;

  // reduce to 0..512
  float norm = pulse_len * (512.0/1639.0);

  // and normalize to -255-255
  pulse_len = norm - 255;

  return pulse_len;
}

int lim_8(int in) {
  if (in > 255) {
    return 255;
  }

  if (in < -255) {
    return -255;
  }

  // center deadzone
  if(in < 10 && in > -10) {
    return 0;
  }

  return in;
}

unsigned long compute_checksum() {
  return data.ch[0]*64 + data.ch[1]*16 + data.ch[2]*4 + data.ch[3];
}

bool rc_signal_is_healthy() {
  unsigned long new_checksum = compute_checksum();
  unsigned long now = millis();
  bool pass = false;
  if (new_checksum != control_checksum) {
    last_changed_at = now;
    control_checksum = new_checksum;
    pass = true;
    startup = false;
  } else if(!startup) {
    pass = ((now - last_changed_at) < CONTROL_MOTION_TIMEOUT_MS);
  }
  return pass;
}

void loop() {
  rc_poll();

  // default to: Neutral drive controls, minimum (off) weapon throttle
  int forback = 0;
  int leftright = 0;
  int throttle = 0;

  if(rc_signal_is_healthy()) {
    forback = get_ch(1);
    leftright = get_ch(0);
    throttle = get_ch(2);
    digitalWrite(BLUE, HIGH);
  }
  else {
    digitalWrite(BLUE, LOW);
  }

  // mixing time!
  int ldrv = lim_8(forback + leftright);
  int rdrv = lim_8(forback - leftright);

  if (ldrv > 0) {
    analogWrite(LFWD, ldrv);
    analogWrite(LREV, 0);
  } else {
    analogWrite(LFWD, 0);
    analogWrite(LREV, ldrv*-1);
  }

  if (rdrv > 0) {
    analogWrite(RFWD, rdrv);
    analogWrite(RREV, 0);
  } else {
    analogWrite(RFWD, 0);
    analogWrite(RREV, rdrv*-1);
  }

  int wthrtl = map(throttle, -255, 255, 1000, 2000);
  
  if(!startup) {
    // only send weapon signal once we know we have radio signal
    motor.writeMicroseconds(wthrtl);
  }

  if(millis() % 512 < 20) {
    dbgSerial.print("Throttle setting ms: ");
    dbgSerial.println(wthrtl);
  }

  delay(10);
}
