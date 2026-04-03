// =============================================================================
// igvc_motor_firmware  —  AVL IGVC 2026  —  Teensy 4.1
// =============================================================================
//
// Reads motor commands from the ROS igvc_motor_driver/serial_bridge node over
// USB serial, then sends velocity setpoints to both SPARK MAX motor controllers
// via CAN.  The SPARK MAX internal PID keeps both motors locked to the target
// RPM, giving motor synchronisation that duty-cycle control cannot.
//
// Serial protocol (USB, 115200 baud):
//   IN  (ROS → Teensy):
//     "M <duty1> <duty2>\n"   duty ±1.0 → mapped to ±MAX_RPM velocity setpoint
//     "S\n"                   stop both motors immediately (0 RPM + duty = 0)
//
//   OUT (Teensy → ROS):
//     "E <rpm_a> <rpm_b>\n"   encoder velocity feedback at ~20 Hz
//
// CAN wiring:
//   CAN ID 1 → SPARK MAX A  (left  motor)
//   CAN ID 2 → SPARK MAX B  (right motor)
//
// Hardware:
//   Teensy 4.1  CAN1_TX = pin 22   CAN1_RX = pin 23
//   Adafruit TJA1051T/3 CAN transceiver (SLNT pin must be tied to GND)
//   SPARK MAX default CAN bus speed: 1 Mbit/s
//   Motors: REV NEO Vortex (REV-21-1650) — free speed ~6784 RPM
//
// Safety:
//   If no valid serial command arrives within SERIAL_TIMEOUT_MS the firmware
//   zeros both motors.  Catches USB disconnects and ROS crashes.
// =============================================================================

#include <Arduino.h>
#include <FlexCAN_T4.h>

// ── CAN bus instance ──────────────────────────────────────────────────────────
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

// ── SPARK MAX CAN ID helpers ─────────────────────────────────────────────────
// REV Robotics 29-bit extended CAN ID encoding:
//   Bits 28:24  Device Type   = 2  (Motor Controller)
//   Bits 23:16  Manufacturer  = 5  (REV Robotics)
//   Bits 15:10  API Class
//   Bits  9:6   API Index
//   Bits  5:0   Device ID
static inline uint32_t sparkId(uint8_t apiClass, uint8_t apiIndex, uint8_t devId)
{
    return (2UL << 24) | (5UL << 16)
         | ((uint32_t)apiClass << 10) | ((uint32_t)apiIndex << 6)
         | devId;
}

// API IDs
static const uint32_t DUTY_BASE      = sparkId(0,  2, 0);  // 0x02050080 — percent output
static const uint32_t VELOCITY_BASE  = sparkId(1,  2, 0);  // 0x02050480 — velocity setpoint
static const uint32_t HEARTBEAT_ID   = sparkId(11, 2, 0);  // 0x02052C80 — non-roboRIO enable
static const uint32_t PARAM_BASE     = sparkId(48, 0, 0);  // 0x0205C000 — parameter access

// ── Motor CAN IDs ────────────────────────────────────────────────────────────
static const uint8_t CAN_ID_MOTOR_A = 1;   // left  motor
static const uint8_t CAN_ID_MOTOR_B = 2;   // right motor

// ── Velocity mapping ─────────────────────────────────────────────────────────
// NEO Vortex free speed ~6784 RPM.  Use a conservative max for loaded operation.
static const float MAX_RPM = 5700.0f;

// ── PID gains (SPARK MAX velocity PID slot 0) ────────────────────────────────
// kFF ≈ 1/free_speed so that kFF * target_RPM ≈ required duty at that RPM.
// kP provides correction for load disturbances.
static const float PID_KP  = 0.0002f;
static const float PID_KI  = 0.000001f;
static const float PID_KD  = 0.0f;
static const float PID_KFF = 0.000175f;   // ~1/5700

// SPARK MAX parameter IDs for PID slot 0
static const uint8_t PARAM_KP  = 1;
static const uint8_t PARAM_KI  = 2;
static const uint8_t PARAM_KD  = 3;
static const uint8_t PARAM_KFF = 5;

// ── Timing ────────────────────────────────────────────────────────────────────
static const uint32_t SERIAL_TIMEOUT_MS   = 500;   // zero motors if ROS goes silent
static const uint32_t CAN_INTERVAL_MS     = 20;    // 50 Hz CAN output
static const uint32_t FEEDBACK_INTERVAL_MS = 50;   // 20 Hz encoder feedback to ROS

// ── State ─────────────────────────────────────────────────────────────────────
static float    g_duty_a    = 0.0f;     // commanded duty [-1, 1]
static float    g_duty_b    = 0.0f;
static float    g_rpm_a     = 0.0f;     // measured RPM from SPARK MAX feedback
static float    g_rpm_b     = 0.0f;
static uint32_t g_last_cmd  = 0;
static uint32_t g_last_can  = 0;
static uint32_t g_last_fb   = 0;
static bool     g_stopped   = true;     // send duty=0 when stopped for instant brake

static char g_buf[64];
static int  g_buf_len = 0;

// ── CAN send helpers ─────────────────────────────────────────────────────────

static void sendHeartbeat()
{
    CAN_message_t msg;
    msg.id             = HEARTBEAT_ID;
    msg.flags.extended = 1;
    msg.len            = 8;
    uint8_t hb[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
    memcpy(msg.buf, hb, 8);
    Can.write(msg);
}

static void sendVelocity(uint8_t deviceId, float rpm)
{
    CAN_message_t msg;
    msg.id             = VELOCITY_BASE | deviceId;
    msg.flags.extended = 1;
    msg.len            = 8;
    memset(msg.buf, 0, 8);
    memcpy(msg.buf, &rpm, 4);
    Can.write(msg);
}

static void sendDuty(uint8_t deviceId, float duty)
{
    CAN_message_t msg;
    msg.id             = DUTY_BASE | deviceId;
    msg.flags.extended = 1;
    msg.len            = 8;
    memset(msg.buf, 0, 8);
    memcpy(msg.buf, &duty, 4);
    Can.write(msg);
}

static void setParameter(uint8_t deviceId, uint8_t paramId, float value)
{
    CAN_message_t msg;
    msg.id             = PARAM_BASE | deviceId;
    msg.flags.extended = 1;
    msg.len            = 8;
    memset(msg.buf, 0, 8);
    msg.buf[0] = paramId;       // parameter ID low byte
    msg.buf[1] = 0;             // parameter ID high byte
    msg.buf[2] = 0x01;          // type: float
    memcpy(&msg.buf[3], &value, 4);
    Can.write(msg);
}

static void configStatusFramePeriod(uint8_t deviceId, uint8_t apiIndex, uint16_t periodMs)
{
    CAN_message_t msg;
    msg.id             = sparkId(6, apiIndex, deviceId);
    msg.flags.extended = 1;
    msg.len            = 2;
    msg.buf[0]         = periodMs & 0xFF;
    msg.buf[1]         = (periodMs >> 8) & 0xFF;
    Can.write(msg);
}

// ── Configure one SPARK MAX for velocity PID ─────────────────────────────────
static void configureMotor(uint8_t deviceId)
{
    // Set PID gains (slot 0)
    setParameter(deviceId, PARAM_KP,  PID_KP);
    delay(5);
    setParameter(deviceId, PARAM_KI,  PID_KI);
    delay(5);
    setParameter(deviceId, PARAM_KD,  PID_KD);
    delay(5);
    setParameter(deviceId, PARAM_KFF, PID_KFF);
    delay(5);

    // Configure velocity feedback status frame to 50ms
    configStatusFramePeriod(deviceId, 3, 50);   // api_index 3 = velocity
    delay(5);
}

// ── Read velocity feedback from CAN RX ───────────────────────────────────────
static void pollCanRx()
{
    CAN_message_t msg;
    while (Can.read(msg)) {
        if (!msg.flags.extended) continue;

        uint8_t devId    =  msg.id        & 0x3F;
        uint8_t apiIndex = (msg.id >> 6)  & 0x0F;
        uint8_t apiClass = (msg.id >> 10) & 0x3F;

        // Velocity feedback: api_class=6, api_index=3
        if (apiClass == 6 && apiIndex == 3 && msg.len >= 4) {
            float rpm;
            memcpy(&rpm, msg.buf, 4);
            if (devId == CAN_ID_MOTOR_A) g_rpm_a = rpm;
            if (devId == CAN_ID_MOTOR_B) g_rpm_b = rpm;
        }
    }
}

// ── Process one \n-terminated line from serial ────────────────────────────────
static void processLine(const char *line)
{
    if (line[0] == 'S' || line[0] == 's') {
        g_duty_a   = 0.0f;
        g_duty_b   = 0.0f;
        g_stopped  = true;
        g_last_cmd = millis();
        return;
    }

    if ((line[0] == 'M' || line[0] == 'm') && line[1] == ' ') {
        float d1, d2;
        if (sscanf(line + 2, "%f %f", &d1, &d2) == 2) {
            g_duty_a   = constrain(d1, -1.0f, 1.0f);
            g_duty_b   = constrain(d2, -1.0f, 1.0f);
            g_stopped  = (d1 == 0.0f && d2 == 0.0f);
            g_last_cmd = millis();
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    Can.begin();
    Can.setBaudRate(1000000);

    // Give SPARK MAXes time to boot after power-on
    delay(500);

    // Send heartbeats first to enable the controllers
    for (int i = 0; i < 10; i++) {
        sendHeartbeat();
        delay(20);
    }

    // Configure PID gains and status frame periods
    configureMotor(CAN_ID_MOTOR_A);
    configureMotor(CAN_ID_MOTOR_B);

    g_last_cmd = millis();
    g_last_can = millis();
    g_last_fb  = millis();
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop()
{
    // Read serial commands from ROS
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n') {
            g_buf[g_buf_len] = '\0';
            if (g_buf_len > 0) {
                processLine(g_buf);
            }
            g_buf_len = 0;
        } else if (g_buf_len < (int)(sizeof(g_buf) - 1)) {
            g_buf[g_buf_len++] = c;
        }
    }

    // Read velocity feedback from SPARK MAXes
    pollCanRx();

    // Safety watchdog — zero motors if ROS stops sending
    if (millis() - g_last_cmd > SERIAL_TIMEOUT_MS) {
        g_duty_a  = 0.0f;
        g_duty_b  = 0.0f;
        g_stopped = true;
    }

    // CAN output at 50 Hz
    if (millis() - g_last_can >= CAN_INTERVAL_MS) {
        g_last_can = millis();
        sendHeartbeat();

        if (g_stopped) {
            // When stopped, send duty=0 for immediate brake (no PID windup)
            sendDuty(CAN_ID_MOTOR_A, 0.0f);
            sendDuty(CAN_ID_MOTOR_B, 0.0f);
        } else {
            // Map duty [-1, 1] to RPM and send velocity setpoint
            sendVelocity(CAN_ID_MOTOR_A, g_duty_a * MAX_RPM);
            sendVelocity(CAN_ID_MOTOR_B, g_duty_b * MAX_RPM);
        }
    }

    // Send encoder feedback to ROS at 20 Hz
    if (millis() - g_last_fb >= FEEDBACK_INTERVAL_MS) {
        g_last_fb = millis();
        char fb[64];
        snprintf(fb, sizeof(fb), "E %.1f %.1f\n", g_rpm_a, g_rpm_b);
        Serial.print(fb);
    }
}
