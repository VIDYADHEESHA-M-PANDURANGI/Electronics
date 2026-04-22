/*
 * =============================================================================
 * PROJECT  : IoT Smart Helmet – Accident Detection & Emergency Alert
 * MCU      : ESP32
 * SENSORS  : ADXL345 (Accelerometer), Neo-6M (GPS)
 * COMMS    : SIM800L (GSM)
 * =============================================================================
 *
 * WORKING LOGIC:
 *   1. Power ON  → System starts monitoring (buckle assumed worn)
 *   2. ADXL345   → Continuously reads acceleration + vibration
 *   3. Threshold → Impact detected → Acquire GPS → Send VERIFICATION SMS to rider
 *   4. Rider replies:
 *        YES          → Accident confirmed →
 *                       "Accident Detected! The rider needs help.
 *                        Contact him and the emergency services immediately.
 *                        Location: <Google Maps link>"
 *
 *        NO           → Rider is safe → No message sent to emergency contacts
 *                       → Resume monitoring
 *
 *        No reply 30s → Assumed accident →
 *                       "The rider met with an accident and did not reply to
 *                        the verification. Contact the emergency services and
 *                        him immediately. Location: <Google Maps link>"
 *
 * HARDWARE CONNECTIONS
 * ─────────────────────────────────────────────────────────────────────────────
 *  ADXL345  (I2C)
 *      VCC  → 3.3V
 *      GND  → GND
 *      SDA  → GPIO 21
 *      SCL  → GPIO 22
 *      SDO  → GND        (sets I2C address to 0x53)
 *
 *  Neo-6M GPS
 *      VCC  → 3.3V
 *      GND  → GND
 *      TX   → GPIO 16   (ESP32 RX2)
 *      RX   → GPIO 17   (ESP32 TX2)
 *
 *  SIM800L GSM
 *      VCC  → 4.1V  *** Use dedicated power supply (NOT ESP32 3.3V) ***
 *      GND  → Common GND
 *      TXD  → GPIO 13   (ESP32 RX1)
 *      RXD  → GPIO 14   (ESP32 TX1)
 *
 * LIBRARIES (install via Arduino Library Manager)
 * ─────────────────────────────────────────────────────────────────────────────
 *  1. Adafruit ADXL345        (Adafruit)
 *  2. Adafruit Unified Sensor (Adafruit)
 *  3. TinyGPS++               (Mikal Hart)
 *
 * BOARD: ESP32 Dev Module
 * =============================================================================
 */

#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// =============================================================================
//  PIN DEFINITIONS
// =============================================================================

// SIM800L on UART1
#define GSM_RX_PIN     13
#define GSM_TX_PIN     14

// Neo-6M on UART2
#define GPS_RX_PIN     16
#define GPS_TX_PIN     17

// =============================================================================
//  USER CONFIGURATION  –  EDIT THESE
// =============================================================================

// Rider's own SIM number (receives YES/NO verification SMS)
const char* RIDER_NUMBER       = "+918428573108";

// Emergency contacts (receive accident alert + GPS location)
const char* EMERGENCY_NUMBER_1 = "+918428573108";
const char* EMERGENCY_NUMBER_2 = "+918144063456";  // set "" to disable

// =============================================================================
//  THRESHOLDS & TIMING
// =============================================================================

#define CRASH_THRESHOLD_MS2   49.0f   // m/s²  (~3.5g) – tune as needed
#define VIBRATION_THRESHOLD   49.0f   // m/s²  for vibration burst detection
#define VIBRATION_COUNT_LIMIT 2       // hits within VIBRATION_WINDOW_MS
#define VIBRATION_WINDOW_MS   500     // ms rolling window

#define REPLY_TIMEOUT_MS      30000UL // 30 seconds reply window
#define GPS_TIMEOUT_MS        20000UL // 20 seconds GPS fix window

// =============================================================================
//  GLOBAL OBJECTS
// =============================================================================
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

HardwareSerial gsmSerial(1);   // UART1 → SIM800L
HardwareSerial gpsSerial(2);   // UART2 → Neo-6M
TinyGPSPlus    gps;

// =============================================================================
//  STATE MACHINE
// =============================================================================
enum State {
  STATE_MONITORING,
  STATE_CRASH_DETECTED,
  STATE_AWAITING_REPLY,
  STATE_ALERT_YES,       // Rider replied YES
  STATE_ALERT_NOREPLY,   // No reply within 30s
  STATE_ABORTED          // Rider replied NO
};
State currentState = STATE_MONITORING;

// =============================================================================
//  RUNTIME VARIABLES
// =============================================================================
float         lastAccelMag   = 9.81f;
unsigned long vibWindowStart = 0;
int           vibCount       = 0;

double        savedLat       = 0.0;
double        savedLng       = 0.0;
bool          gpsFixed       = false;

// =============================================================================
//  FORWARD DECLARATIONS
// =============================================================================
bool   detectCrash();
bool   acquireGPS();
String buildLocationStr();
void   sendVerificationSMS();
int    waitForReply();
void   sendAlertYES();
void   sendAlertNoReply();
void   sendSMS(const char* number, String message);
String readGSM(unsigned long timeoutMs);
void   gsmCommand(const char* cmd, unsigned long waitMs);
void   initGSM();
void   log(const char* msg);

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  log("=== Smart Helmet Starting ===");

  // I2C for ADXL345
  Wire.begin(21, 22);
  if (!adxl.begin()) {
    log("[ADXL345] NOT FOUND – check wiring!");
    while (1) { delay(500); }
  }
  adxl.setRange(ADXL345_RANGE_16_G);
  adxl.setDataRate(ADXL345_DATARATE_100_HZ);
  log("[ADXL345] OK");

  // GSM SIM800L – UART1
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(3000);
  initGSM();

  // GPS Neo-6M – UART2
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  log("[GPS] UART started");

  log("System ACTIVE – Monitoring started");
}

// =============================================================================
//  MAIN LOOP
// =============================================================================
void loop() {

  // Always feed GPS parser
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  switch (currentState) {

    // ── NORMAL MONITORING ─────────────────────────────────────────────────
    case STATE_MONITORING:
      if (detectCrash()) {
        log(">>> CRASH DETECTED <<<");
        currentState = STATE_CRASH_DETECTED;
      }
      break;

    // ── CRASH: Acquire GPS → Send verification SMS to rider ───────────────
    case STATE_CRASH_DETECTED:
      log("Acquiring GPS location...");
      gpsFixed = acquireGPS();
      savedLat = gps.location.lat();
      savedLng = gps.location.lng();

      sendVerificationSMS();
      currentState = STATE_AWAITING_REPLY;
      log("Verification SMS sent. Waiting for rider reply (30s)...");
      break;

    // ── WAIT FOR RIDER REPLY ──────────────────────────────────────────────
    case STATE_AWAITING_REPLY: {
      int reply = waitForReply();
      if      (reply ==  1) currentState = STATE_ALERT_YES;
      else if (reply ==  0) currentState = STATE_ABORTED;
      else                  currentState = STATE_ALERT_NOREPLY;
      break;
    }

    // ── RIDER REPLIED YES → Send accident confirmed alert ─────────────────
    case STATE_ALERT_YES:
      sendAlertYES();
      log("Alert sent (YES). Resuming monitoring...");
      delay(5000);
      vibCount     = 0;
      lastAccelMag = 9.81f;
      currentState = STATE_MONITORING;
      break;

    // ── NO REPLY IN 30s → Send no-reply alert ────────────────────────────
    case STATE_ALERT_NOREPLY:
      sendAlertNoReply();
      log("Alert sent (no reply). Resuming monitoring...");
      delay(5000);
      vibCount     = 0;
      lastAccelMag = 9.81f;
      currentState = STATE_MONITORING;
      break;

    // ── RIDER REPLIED NO → False alarm, no alert sent ─────────────────────
    case STATE_ABORTED:
      log("Rider replied NO – Rider is safe. No alert sent. Resuming monitoring...");
      vibCount     = 0;
      lastAccelMag = 9.81f;
      delay(2000);
      currentState = STATE_MONITORING;
      break;
  }
}

// =============================================================================
//  CRASH DETECTION
// =============================================================================
bool detectCrash() {
  sensors_event_t event;
  adxl.getEvent(&event);

  float ax  = event.acceleration.x;
  float ay  = event.acceleration.y;
  float az  = event.acceleration.z;
  float mag = sqrtf(ax*ax + ay*ay + az*az);

  float delta  = fabsf(mag - lastAccelMag);
  lastAccelMag = mag;

  if (mag > VIBRATION_THRESHOLD) {
    if (millis() - vibWindowStart > VIBRATION_WINDOW_MS) {
      vibWindowStart = millis();
      vibCount       = 0;
    }
    vibCount++;
  }

  bool impactDetected    = (delta    >= CRASH_THRESHOLD_MS2);
  bool vibrationDetected = (vibCount >= VIBRATION_COUNT_LIMIT);

  // Debug – comment out after testing
  Serial.printf("[ADXL345] mag=%.2f delta=%.2f vib=%d\n", mag, delta, vibCount);

  return (impactDetected || vibrationDetected);
}

// =============================================================================
//  GPS – Try to get a valid fix within GPS_TIMEOUT_MS
// =============================================================================
bool acquireGPS() {
  unsigned long start = millis();
  log("[GPS] Waiting for fix...");

  while (millis() - start < GPS_TIMEOUT_MS) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid() && gps.location.age() < 2000) {
      Serial.printf("[GPS] Fix OK: %.6f, %.6f\n",
                    gps.location.lat(), gps.location.lng());
      return true;
    }
    delay(100);
  }

  log("[GPS] Timeout – alert will be sent without accurate location.");
  return false;
}

// =============================================================================
//  BUILD LOCATION STRING
// =============================================================================
String buildLocationStr() {
  if (gpsFixed && gps.location.isValid()) {
    String loc = "Location: https://maps.google.com/?q=";
    loc += String(savedLat, 6);
    loc += ",";
    loc += String(savedLng, 6);
    return loc;
  }
  return "Location: GPS unavailable. Please track the rider manually.";
}

// =============================================================================
//  SEND VERIFICATION SMS TO RIDER
// =============================================================================
void sendVerificationSMS() {
  log("[GSM] Sending verification SMS to rider...");
  String msg =
    "SMART HELMET ALERT!\n"
    "A possible accident has been detected.\n"
    "Reply YES if you are in an accident.\n"
    "Reply NO if you are safe.\n"
    "(No reply in 30 sec = emergency alert sent automatically)";
  sendSMS(RIDER_NUMBER, msg);
}

// =============================================================================
//  WAIT FOR RIDER REPLY  (30 second window)
//  Returns:  1 = YES received
//            0 = NO  received
//           -1 = timeout (no reply)
// =============================================================================
int waitForReply() {
  unsigned long deadline = millis() + REPLY_TIMEOUT_MS;

  while (millis() < deadline) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    String incoming = readGSM(800);

    if (incoming.length() > 0) {
      Serial.print("[GSM] Incoming: "); Serial.println(incoming);
      String upper = incoming;
      upper.toUpperCase();

      if (upper.indexOf("YES") != -1) {
        log("[GSM] Rider replied YES – Accident confirmed.");
        return 1;
      }
      if (upper.indexOf("NO") != -1) {
        log("[GSM] Rider replied NO – Rider is safe.");
        return 0;
      }
    }

    delay(100);
  }

  log("[GSM] No reply received in 30s.");
  return -1;
}

// =============================================================================
//  ALERT – Rider replied YES
//  SMS text:
//    "Accident Detected! The rider needs help.
//     Contact him and the emergency services immediately.
//     Location: <Google Maps link>"
// =============================================================================
void sendAlertYES() {
  String msg =
    String("*** ACCIDENT ALERT ***\n") +
    "Accident Detected! The rider needs help.\n" +
    "Contact him and the emergency services immediately.\n" +
    buildLocationStr();

  log("[GSM] Sending alert to emergency contacts (rider replied YES)...");

  if (strlen(EMERGENCY_NUMBER_1) > 0) {
    sendSMS(EMERGENCY_NUMBER_1, msg);
    delay(3000);
  }
  if (strlen(EMERGENCY_NUMBER_2) > 0) {
    sendSMS(EMERGENCY_NUMBER_2, msg);
    delay(3000);
  }

  log("[GSM] Alert sent to all emergency contacts.");
}

// =============================================================================
//  ALERT – No reply within 30 seconds
//  SMS text:
//    "The rider met with an accident and did not reply to the verification.
//     Contact the emergency services and him immediately.
//     Location: <Google Maps link>"
// =============================================================================
void sendAlertNoReply() {
  String msg =
    String("*** ACCIDENT ALERT ***\n") +
    "The rider met with an accident and did not reply to the verification.\n" +
    "Contact the emergency services and him immediately.\n" +
    buildLocationStr();

  log("[GSM] Sending alert to emergency contacts (no reply from rider)...");

  if (strlen(EMERGENCY_NUMBER_1) > 0) {
    sendSMS(EMERGENCY_NUMBER_1, msg);
    delay(3000);
  }
  if (strlen(EMERGENCY_NUMBER_2) > 0) {
    sendSMS(EMERGENCY_NUMBER_2, msg);
    delay(3000);
  }

  log("[GSM] Alert sent to all emergency contacts.");
}

// =============================================================================
//  LOW-LEVEL: SEND SMS VIA SIM800L
// =============================================================================
void sendSMS(const char* number, String message) {
  Serial.printf("[GSM] SMS to %s\n", number);

  String cmd = "AT+CMGS=\"";
  cmd += number;
  cmd += "\"";
  gsmSerial.println(cmd);
  delay(1000);

  gsmSerial.print(message);
  gsmSerial.write(26);   // Ctrl+Z to send
  delay(5000);

  String result = readGSM(5000);
  Serial.print("[GSM] Send result: "); Serial.println(result);
}

// =============================================================================
//  LOW-LEVEL: READ FROM GSM SERIAL WITH TIMEOUT
// =============================================================================
String readGSM(unsigned long timeoutMs) {
  String response = "";
  unsigned long t = millis();
  while (millis() - t < timeoutMs) {
    while (gsmSerial.available()) {
      char c = (char)gsmSerial.read();
      response += c;
    }
  }
  response.trim();
  return response;
}

// =============================================================================
//  LOW-LEVEL: SEND AT COMMAND TO GSM
// =============================================================================
void gsmCommand(const char* cmd, unsigned long waitMs) {
  gsmSerial.println(cmd);
  delay(waitMs);
  String r = readGSM(waitMs);
  Serial.print("[GSM] "); Serial.print(cmd);
  Serial.print(" → "); Serial.println(r);
}

// =============================================================================
//  GSM INIT SEQUENCE
// =============================================================================
void initGSM() {
  log("[GSM] Initializing SIM800L...");
  gsmCommand("AT",                   1000);
  gsmCommand("ATE0",                  500);
  gsmCommand("AT+CMGF=1",             500);
  gsmCommand("AT+CNMI=2,2,0,0,0",     500);
  gsmCommand("AT+CMGDA=\"DEL ALL\"", 3000);
  log("[GSM] SIM800L Ready");
}

// =============================================================================
//  SERIAL LOG HELPER
// =============================================================================
void log(const char* msg) {
  Serial.println(msg);
}
