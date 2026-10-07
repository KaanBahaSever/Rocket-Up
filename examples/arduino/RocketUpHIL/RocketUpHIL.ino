/*
  Rocket-Up hardware-in-the-loop flight computer example.

  Receives simulated sensor data from the Rocket-Up simulator over USB serial and answers
  with deployment commands, exactly as a real flight computer would drive pyro channels.

    rocketup hil data/projects/orbit_chaser.project.json --port COM3
    (or: hil_flight_computer COM3)

  In the project/design, set the parachute triggers to external channels:
    "deploy": {"type": "external", "channel": "drogue"}
    "deploy": {"type": "external", "channel": "main"}

  Protocol (see docs/HIL.md):
    sim -> board  $RUSEN,t_ms,p_Pa,T_C,ax,ay,az,gx,gy,gz,baroAlt,fix,lat,lon,gpsAlt*CS
    board -> sim  $RUCMD,<channel>*CS     $RULOG,<text>

  Works on any Arduino (Uno, Nano, Mega, ESP32, Teensy...). Pins 5/6 light up as "pyro"
  outputs so you can see the deployments, pin 13 blinks while data is received.
*/

const long BAUD = 115200;
const int PIN_DROGUE = 5;
const int PIN_MAIN = 6;
const int PIN_LED = 13;

const float LAUNCH_ACCEL = 3.0 * 9.81;  // m/s^2 along the rocket axis
const float APOGEE_DROP = 4.0;          // m below the maximum filtered altitude
const float MAIN_ALTITUDE = 600.0;      // m above the pad
const float LOCKOUT_S = 5.0;            // no apogee detection right after launch
const float FILTER_ALPHA = 0.3;         // baro low-pass

enum State { PAD, ASCENT, DROGUE, MAIN_DEPLOYED };
State state = PAD;

char line[200];
int lineLen = 0;
float filtered = 0, maxAlt = -1e9, launchTime = 0;
bool filterInit = false;

byte checksum(const char* s) {
  byte c = 0;
  while (*s) c ^= (byte)*s++;
  return c;
}

void sendFrame(const char* body) {
  char buf[64];
  snprintf(buf, sizeof(buf), "$%s*%02X", body, checksum(body));
  Serial.println(buf);
}

void logMessage(const char* text, float value) {
  // $RULOG lines may omit the checksum.
  Serial.print("$RULOG,");
  Serial.print(text);
  Serial.print(' ');
  Serial.println(value, 1);
}

void handleSensors(char* body) {
  // body = "RUSEN,t_ms,p,T,ax,ay,az,gx,gy,gz,baroAlt,fix,lat,lon,gpsAlt"
  float f[15];
  int n = 0;
  char* tok = strtok(body, ",");  // "RUSEN"
  while ((tok = strtok(NULL, ",")) != NULL && n < 15) f[n++] = atof(tok);
  if (n < 10) return;
  const float t = f[0] / 1000.0;
  const float ax = f[3];
  const float alt = f[9];

  if (!filterInit) {
    filtered = alt;
    filterInit = true;
  }
  filtered += FILTER_ALPHA * (alt - filtered);
  if (filtered > maxAlt) maxAlt = filtered;

  switch (state) {
    case PAD:
      if (ax > LAUNCH_ACCEL) {
        state = ASCENT;
        launchTime = t;
        logMessage("launch detected at t =", t);
      }
      break;
    case ASCENT:
      if (t - launchTime > LOCKOUT_S && maxAlt - filtered > APOGEE_DROP) {
        state = DROGUE;
        digitalWrite(PIN_DROGUE, HIGH);
        sendFrame("RUCMD,drogue");
        logMessage("apogee, max altitude", maxAlt);
      }
      break;
    case DROGUE:
      if (filtered < MAIN_ALTITUDE) {
        state = MAIN_DEPLOYED;
        digitalWrite(PIN_MAIN, HIGH);
        sendFrame("RUCMD,main");
        logMessage("main at", filtered);
      }
      break;
    case MAIN_DEPLOYED:
      break;
  }
}

void handleLine(char* s) {
  if (s[0] != '$') return;
  char* star = strchr(s, '*');
  if (star) {
    *star = '\0';
    const byte expected = (byte)strtol(star + 1, NULL, 16);
    if (checksum(s + 1) != expected) return;  // corrupted line
  }
  char* body = s + 1;
  if (strncmp(body, "RUSEN,", 6) == 0) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    handleSensors(body);
  } else if (strncmp(body, "RUHELLO", 7) == 0) {
    state = PAD;
    filterInit = false;
    maxAlt = -1e9;
    digitalWrite(PIN_DROGUE, LOW);
    digitalWrite(PIN_MAIN, LOW);
    Serial.println("$RULOG,flight computer ready");
  }
}

void setup() {
  pinMode(PIN_DROGUE, OUTPUT);
  pinMode(PIN_MAIN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(BAUD);
}

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        line[lineLen] = '\0';
        handleLine(line);
        lineLen = 0;
      }
    } else if (lineLen < (int)sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow: drop the line
    }
  }
}
