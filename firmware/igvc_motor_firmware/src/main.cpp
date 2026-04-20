// =============================================================================
// igvc_motor_firmware  —  AVL IGVC 2026  —  Teensy 4.1
// =============================================================================
//
// Thin USB-Serial <-> CAN bridge. The Jetson (ROS2 igvc_control) owns
// diff-drive kinematics and streams per-wheel RPM setpoints; this firmware
// forwards them to each SparkMAX's built-in velocity PID and echoes encoder
// feedback back over serial.
//
// Hardware:
//   Teensy 4.1  (CAN1: CTX1=pin22, CRX1=pin23)
//   SN65HVD230 or TJA1051T/3 CAN transceiver, 120 ohm termination at each end
//   REV SparkMAX x 2 -- CAN ID 1 = left, ID 2 = right
//
// Host -> Teensy (115200 baud, newline-terminated):
//   L<rpm> R<rpm>     set wheel setpoints (e.g. "L500 R-500")
//   S                 stop both wheels
//   D                 print one DIAG line
//
// Teensy -> Host:
//   E L<rpm> <pos> R<rpm> <pos>   50 Hz wheel feedback (RPM + rotations)
//   OK  ...                       command acknowledgement
//   ERR ...                       parse failure
//   DIAG ...                      response to 'D'
//   # ...                         informational log line
//
// Safety:
//   300 ms host watchdog -- if no L/R/S arrives in that window, both wheels
//   are forced to 0 RPM. SparkMAX 100 ms heartbeat timeout is a second layer.
//   MAX_RPM clamp (3000, well below NEO free speed).
// =============================================================================

#include <FlexCAN_T4.h>
#include <string.h>
#include <ctype.h>

// ---------- Configuration ----------------------------------------------------
static constexpr uint8_t  LEFT_ID        = 1;
static constexpr uint8_t  RIGHT_ID       = 2;
static constexpr uint32_t CAN_BAUD       = 1000000;
static constexpr uint32_t CTRL_DT_MS     = 20;    // 50 Hz control loop
static constexpr uint32_t FEEDBACK_DT_MS = 20;    // 50 Hz E-line to host
static constexpr uint32_t WATCHDOG_MS    = 300;
static constexpr float    MAX_RPM        = 3000.0f;

// ---------- SparkMAX CAN protocol constants ----------------------------------
static constexpr uint8_t  SPARK_DEV_TYPE = 2;
static constexpr uint8_t  SPARK_MFG      = 5;
static constexpr uint8_t  CLS_VELOCITY   = 1;   // velocity setpoint
static constexpr uint8_t  IDX_VELOCITY   = 2;
static constexpr uint8_t  CLS_STATUS     = 46;  // periodic status frames (FW 25+)
static constexpr uint8_t  IDX_STATUS_2   = 2;
static constexpr uint8_t  CLS_HB         = 11;  // REV secondary heartbeat
static constexpr uint8_t  IDX_HB         = 2;
static constexpr uint32_t UNIVERSAL_HB   = 0x01011840;  // roboRIO heartbeat (FW 26+)

// ---------- State ------------------------------------------------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

struct Wheel {
    float          cmd_rpm  = 0.0f;
    volatile float meas_rpm = 0.0f;
    volatile float meas_pos = 0.0f;
};
static Wheel left, right;

static uint32_t t_ctrl = 0, t_fb = 0, t_last_host = 0;
static uint32_t tx_count = 0, rx_count = 0;
static bool     wdt_tripped = false;

// ---------- CAN helpers ------------------------------------------------------
static inline uint32_t sparkId(uint8_t cls, uint8_t idx, uint8_t dev) {
    return ((uint32_t)SPARK_DEV_TYPE << 24)
         | ((uint32_t)SPARK_MFG      << 16)
         | ((uint32_t)(cls & 0x3F)   << 10)
         | ((uint32_t)(idx & 0x0F)   <<  6)
         | ((uint32_t)(dev & 0x3F));
}

static void canSend(uint32_t id, const uint8_t *data, uint8_t len) {
    CAN_message_t m;
    m.flags.extended = 1;
    m.id  = id;
    m.len = len;
    memcpy(m.buf, data, len);
    if (Can.write(m) > 0) tx_count++;
}

static inline void fromFloat(float f, uint8_t *b) { memcpy(b, &f, sizeof(float)); }
static inline float toFloat(const uint8_t *b)     { float v; memcpy(&v, b, sizeof(float)); return v; }

// ---------- SparkMAX commands ------------------------------------------------
static void sendHeartbeats() {
    // Universal heartbeat (required by SparkMAX FW 26+)
    static const uint8_t uni[8] = {0x78, 0x01, 0x00, 0x12, 0x59, 0x04, 0x00, 0x60};
    canSend(UNIVERSAL_HB, uni, 8);
    // REV Secondary heartbeat (enables all devices as fallback)
    static const uint8_t sec[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    canSend(sparkId(CLS_HB, IDX_HB, 0), sec, 8);
}

static void setVelocity(uint8_t dev, float rpm) {
    if (rpm >  MAX_RPM) rpm =  MAX_RPM;
    if (rpm < -MAX_RPM) rpm = -MAX_RPM;
    uint8_t d[8] = {0};
    fromFloat(rpm, d);
    canSend(sparkId(CLS_VELOCITY, IDX_VELOCITY, dev), d, 8);
}

// ---------- CAN RX -- decode STATUS_2 (encoder feedback) ---------------------
static void onCanRx(const CAN_message_t &msg) {
    if (!msg.flags.extended || msg.len < 8) return;
    rx_count++;

    uint8_t dev = msg.id & 0x3F;
    uint8_t cls = (msg.id >> 10) & 0x3F;
    uint8_t idx = (msg.id >>  6) & 0x0F;
    if (cls != CLS_STATUS || idx != IDX_STATUS_2) return;

    float vel = toFloat(msg.buf);
    float pos = toFloat(msg.buf + 4);
    if (dev == LEFT_ID) {
        left.meas_rpm = vel;  left.meas_pos = pos;
    } else if (dev == RIGHT_ID) {
        right.meas_rpm = vel; right.meas_pos = pos;
    }
}

// ---------- Serial parser ----------------------------------------------------
static void handleLine(char *line) {
    if (!line[0]) return;
    char cmd = toupper((unsigned char)line[0]);

    switch (cmd) {
        case 'S':
            left.cmd_rpm = right.cmd_rpm = 0.0f;
            t_last_host = millis();
            wdt_tripped = false;
            Serial.println("OK S");
            return;

        case 'D':
            Serial.printf("DIAG tx=%lu rx=%lu wdt=%d L=%.0f/%.0f R=%.0f/%.0f\n",
                          tx_count, rx_count, wdt_tripped ? 1 : 0,
                          left.meas_rpm, left.cmd_rpm,
                          right.meas_rpm, right.cmd_rpm);
            return;

        default: {
            // "L500 R-500" / "L500" / "R-500"
            char *lp = strchr(line, 'L'); if (!lp) lp = strchr(line, 'l');
            char *rp = strchr(line, 'R'); if (!rp) rp = strchr(line, 'r');
            if (!lp && !rp) { Serial.println("ERR unknown"); return; }
            if (lp) left.cmd_rpm  = atof(lp + 1);
            if (rp) right.cmd_rpm = atof(rp + 1);
            t_last_host = millis();
            wdt_tripped = false;
            Serial.printf("OK L=%.0f R=%.0f\n", left.cmd_rpm, right.cmd_rpm);
            return;
        }
    }
}

static void processSerial() {
    static char buf[96];
    static uint8_t n = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (n > 0) { buf[n] = '\0'; handleLine(buf); n = 0; }
        } else if (n < sizeof(buf) - 1) {
            buf[n++] = c;
        }
    }
}

// ---------- setup / loop -----------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("# avros diff-drive bridge ready");
    Serial.println("# proto: L<rpm> R<rpm> | S | D");

    Can.begin();
    Can.setBaudRate(CAN_BAUD);
    Can.setMaxMB(16);
    Can.enableFIFO();
    Can.enableFIFOInterrupt();
    Can.setFIFOFilter(ACCEPT_ALL);
    Can.onReceive(onCanRx);

    // Wake SparkMAXes before commanding
    delay(100);
    sendHeartbeats();
    delay(50);

    t_last_host = millis();
}

void loop() {
    Can.events();
    processSerial();

    uint32_t now = millis();

    // 50 Hz control tick
    if (now - t_ctrl >= CTRL_DT_MS) {
        t_ctrl = now;

        if (now - t_last_host > WATCHDOG_MS) {
            if (!wdt_tripped) {
                Serial.println("# WDT host-timeout stop");
                wdt_tripped = true;
            }
            left.cmd_rpm = right.cmd_rpm = 0.0f;
        }

        sendHeartbeats();
        setVelocity(LEFT_ID,  left.cmd_rpm);
        setVelocity(RIGHT_ID, right.cmd_rpm);
    }

    // 50 Hz wheel feedback to host
    if (now - t_fb >= FEEDBACK_DT_MS) {
        t_fb = now;
        if (Serial.availableForWrite() >= 64) {
            Serial.printf("E L%.0f %.4f R%.0f %.4f\n",
                          left.meas_rpm, left.meas_pos,
                          right.meas_rpm, right.meas_pos);
        }
    }
}
