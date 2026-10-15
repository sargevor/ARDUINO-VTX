#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>

/*
 * STM32F103C8T6 ("Blue Pill") + Arduino IDE
 *
 * ------------------------------------------------------------------
 * CHANGE LOG vs. the original STM32G0 version
 * ------------------------------------------------------------------
 *  - MCU:            STM32G0  ->  STM32F103C8T6
 *  - Command/debug link: was native USB Serial -> now USART2 on
 *                    PA2 (TX) / PA3 (RX), for an external USB-TTL
 *                    adapter, 115200 baud (same framing as before).
 *  - VTX link:       was PB6 single-wire half-duplex -> now PA9
 *                    (USART1), still single-wire half-duplex, same
 *                    9600-baud IRC Tramp protocol as before.
 *                    ASSUMPTION: I've kept this a true single-wire
 *                    hardware half-duplex link (only PA9 is driven;
 *                    PA10 is not used by this code) because that's
 *                    how Tramp VTXs are normally wired, and it's the
 *                    most direct port of your original PB6 design.
 *                    If you actually have two separate wires running
 *                    to the VTX (TX on PA9, RX on PA10, e.g. through
 *                    an external combining resistor), tell me and
 *                    I'll switch this to a real two-pin driver
 *                    instead - it's a different code path.
 *  - Thermometer:    PA5, unchanged (1-Wire DS18B20, still needs the
 *                    4.7k pull-up to 3.3V).
 *  - PSU link:       Ruideng DPH5005 digital power supply, wired to
 *                    USART3 on PB10 (TX) / PB11 (RX), Modbus-RTU,
 *                    8N1. New serial commands: PS, PS_SET_<v>_<i>,
 *                    PS_ON, PS_OFF (see below).
 *  - NEW: PSU retry: PSU queries/writes are now retried a few times
 *                    before being reported as failed. A single-byte
 *                    glitch or a request landing while the PSU is
 *                    busy with its own housekeeping (display/keypad
 *                    scan) can occasionally corrupt or delay one
 *                    reply; retrying a couple of times recovers from
 *                    that without materially slowing down a genuine
 *                    "PSU not connected" case. See PSU_MAX_ATTEMPTS /
 *                    PSU_RETRY_DELAY_MS below.
 *
 * ------------------------------------------------------------------
 * BUGFIX (this revision): OK/INVALID_INPUT sometimes wrong, V
 * sometimes stale
 * ------------------------------------------------------------------
 * Two related bugs caused "VTX changes but sketch reports
 * INVALID_INPUT" and "V occasionally shows a wrong value":
 *
 *  1. F_/P_/OFF used a single fixed delay() (300ms) before reading
 *     the setting back exactly once. But F/P/pit-mode are NOT
 *     acknowledged by this VTX - it applies them internally (PLL
 *     relock, RF amp ramp/calibration) over some VTX-dependent
 *     amount of time that 300ms doesn't reliably cover. A too-early
 *     single check reads the VTX's still-old cached value and
 *     misreports a real, successful change as INVALID_INPUT.
 *     FIX: confirmVtxFreqPower()/confirmVtxPitMode() now POLL with
 *     repeated 'v' queries for up to VTX_CONFIRM_TIMEOUT_MS,
 *     returning OK as soon as a match is seen instead of guessing a
 *     single wait time.
 *
 *  2. queryVtxRaw() (used by every 'v' read, including the plain V
 *     command) only tried once. On a bare single-wire link, an
 *     occasional dropped/corrupted reply is normal - PSU queries
 *     already accounted for this with retries, but the VTX side
 *     didn't. FIX: queryVtxRaw() now retries up to
 *     VTX_QUERY_MAX_ATTEMPTS times before giving up, same idea as
 *     the PSU retry wrappers.
 *
 * All three UARTs (USART1/PA9 half-duplex, USART2/PA2-PA3,
 * USART3/PB10-PB11) are independent peripherals on the F103, so
 * there's no pin/peripheral conflict between VTX, debug and PSU.
 *
 * ------------------------------------------------------------------
 * Serial commands (newline terminated, case-insensitive)
 * ------------------------------------------------------------------
 *   F_<freq>_P_<power>   Set VTX frequency (MHz) and power (mW).
 *                        Automatically exits Pit Mode first, then reads
 *                        back the VTX's actual settings to confirm they
 *                        took. Replies with exactly one of:
 *                          OK             - value applied and confirmed
 *                          INVALID_INPUT  - VTX rejected the value (out
 *                                           of its supported range) and
 *                                           kept its previous setting
 *                          ERR            - couldn't talk to the VTX
 *   V                    Query VTX status. One line, fields separated
 *                        by ';', single trailing newline, e.g.:
 *                          VTX_ALIVE=1;FREQUENCY_MHZ=3330;CONFIGURED_POWER_MW=100;
 *                          ACTUAL_POWER_MW=100;CONTROL_MODE=0;PIT_MODE=0;
 *                          RACE_LOCK=0;TEMP=24.31
 *                        (TEMP=ERROR if the sensor read failed.) If the
 *                        VTX doesn't respond at all, just VTX_ALIVE=0.
 *   OFF                  Put the VTX into Pit Mode, then reads back pit
 *                        mode to confirm. Replies OK or ERR.
 *
 *   PS                    Query the DPH5005 PSU. One line, e.g.:
 *                          PSU_ALIVE=1;VSET=12.00;ISET=2.000;VOUT=12.01;
 *                          IOUT=1.520;POWER_W=18.30;VIN=24.05;LOCK=1;
 *                          CVCC=0;ONOFF=1;PROTECT=0
 *                        (CVCC: 0=CV, 1=CC. PROTECT: 0=normal, 1=OVP,
 *                        2=OCP, 3=OPP. LOCK: 1=keypad locked.)
 *                        If the PSU doesn't respond after retrying:
 *                        PSU_ALIVE=0.
 *   PS_SET_<v>_<i>        Set the PSU's voltage/current setpoints, in
 *                        the device's native register units: <v> is
 *                        volts x100 (1200 = 12.00V), <i> is amps x1000
 *                        (2000 = 2.000A). Replies OK or ERR.
 *   PS_ON / PS_OFF        Turn the PSU output on/off. Replies OK or ERR.
 *
 * Requires the "OneWire" and "DallasTemperature" libraries (Library
 * Manager). Temperature is read in the background once per second and
 * exposed via the TEMP= field in the V command's output above - it is
 * not printed on its own.
 *
 * Requires an STM32 Arduino core version that supports HardwareSerial
 * hardware half-duplex (enableHalfDuplexRx()) - core >= 1.7.0 of
 * stm32duino's Arduino_Core_STM32. Select a Generic STM32F1 board /
 * "BluePill F103C8" in the Boards Manager.
 *
 * NOTE: PIT_MODE_COMMAND below ('I') is the commonly documented Tramp
 * command letter for setting Pit Mode. Confirm this against your VTX's
 * own Tramp implementation - neither original version of this sketch
 * ever sent a pit-mode SET frame and confirmed the ack, only read pit
 * mode back passively via the 'v' status response.
 *
 * NOTE on the PSU: this uses the register map from the DPH5005's own
 * "CNC Power Communication Protocol" manual:
 *   0x0000 U-SET   (volts x100, R/W)
 *   0x0001 I-SET   (amps  x1000, R/W)
 *   0x0002 UOUT    (volts x100)
 *   0x0003 IOUT    (amps  x1000)
 *   0x0004 POWER   (watts x100)
 *   0x0005 UIN     (volts x100)
 *   0x0006 LOCK    (0/1, R/W)
 *   0x0007 PROTECT (0 normal, 1 OVP, 2 OCP, 3 OPP)
 *   0x0008 CVCC    (0 CV, 1 CC)
 *   0x0009 ONOFF   (0/1, R/W)
 * PSU_BAUD below is set to 9600, the DPH5005's common default, but
 * this is USER-CONFIGURABLE on the unit itself (2400/4800/9600/19200)
 * - check the BAUD field on the PSU's screen/menu and change PSU_BAUD
 * to match if it's been changed from default. Likewise PSU_SLAVE_ADDR
 * defaults to 1 (shown as "001" on the unit) but is also changeable.
 * Confirmed working wiring for this build: STM32 GND/PB11(RX)/PB10(TX)
 * to the mainboard's bare GND/TXD/RXD pads (crossed: PSU TXD -> PB11,
 * PSU RXD -> PB10), NOT the adjacent V/G/C/D display header.
 */

/* ============================================================
 * VTX link (IRC Tramp over single-wire half-duplex USART1/PA9)
 * ============================================================ */

constexpr uint8_t  VTX_PIN  = PA9;
constexpr uint32_t VTX_BAUD = 9600;

constexpr size_t   TRAMP_PACKET_SIZE            = 16;
constexpr uint32_t TRAMP_RESPONSE_TIMEOUT_MS     = 400;
constexpr uint32_t TRAMP_MIN_REQUEST_INTERVAL_MS = 200;

// Retry/confirmation handling: an occasional dropped or corrupted
// reply on the single-wire link is normal, and F/P/pit-mode changes
// aren't acknowledged - the VTX just applies them internally over
// some VTX-dependent amount of time. So queries are retried a few
// times, and post-change verification POLLS (rather than trusting a
// single fixed delay) until either the expected value shows up or
// VTX_CONFIRM_TIMEOUT_MS elapses.
constexpr uint8_t  VTX_QUERY_MAX_ATTEMPTS   = 3;
constexpr uint32_t VTX_QUERY_RETRY_DELAY_MS = 50;
constexpr uint32_t VTX_CONFIRM_TIMEOUT_MS   = 1500;

// Pit mode command letter/values - verify against your VTX's Tramp spec.
// NOTE: confirmed by testing on this unit that the field is inverted from
// the commonly documented convention: 0 = pit mode ON, 1 = pit mode OFF.
constexpr char     PIT_MODE_COMMAND = 'I';
constexpr uint16_t PIT_MODE_ON      = 0;
constexpr uint16_t PIT_MODE_OFF     = 1;

// Single-pin constructor = hardware half-duplex mode on the STM32 core.
HardwareSerial VTXSerial(VTX_PIN);

uint32_t lastRequestTimeMs = 0;

// --- DS18B20 temperature sensor (1-Wire on PA5) ---
constexpr uint8_t  TEMP_SENSOR_PIN       = PA5;
constexpr uint32_t TEMP_READ_INTERVAL_MS = 1000; // how often to start a new reading
constexpr uint32_t DS18B20_CONVERSION_MS = 750;  // 12-bit conversion time, per datasheet

OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

enum class TempState { Idle, Converting };

TempState tempState             = TempState::Idle;
uint32_t  lastTempRequestTimeMs = 0;
uint32_t  tempConversionStartMs = 0;
float     lastKnownTempC        = DEVICE_DISCONNECTED_C; // updated once per second

/* ============================================================
 * Ruideng DPH5005 PSU link (Modbus-RTU over USART3, PB10/PB11)
 * ============================================================ */

constexpr uint8_t  PSU_SLAVE_ADDR              = 0x01;  // default "001" on the unit - verify on-screen
constexpr uint32_t PSU_BAUD                    = 9600;  // DPH5005 common default - VERIFY against the
                                                          // unit's on-screen BAUD setting before relying on this
constexpr uint32_t PSU_RESPONSE_TIMEOUT_MS     = 800;
constexpr uint32_t PSU_INTERBYTE_TIMEOUT_MS    = 100;
constexpr size_t   MODBUS_MAX_FRAME            = 64;

// Retry handling: an occasional single-bit glitch or a request that
// lands while the PSU's own MCU is busy (display/keypad scan) can
// corrupt or delay a single reply. Retry a few times before giving
// up, rather than reporting PSU_ALIVE=0 / ERR on the very first miss.
constexpr uint8_t  PSU_MAX_ATTEMPTS            = 3;
constexpr uint32_t PSU_RETRY_DELAY_MS          = 100;

// Enable to print raw Modbus bytes received from the PSU on every PS
// query, for debugging a dead link. Leave off for normal use.
constexpr bool     PSU_DEBUG_RAW_BYTES         = false;

// Holding-register map, per the DPH5005's "CNC Power Communication
// Protocol" manual. Register : meaning : scaling.
constexpr uint16_t PSU_REG_VSET    = 0x0000; // U-SET,  volts   x100,  R/W
constexpr uint16_t PSU_REG_ISET    = 0x0001; // I-SET,  amps    x1000, R/W
constexpr uint16_t PSU_REG_VOUT    = 0x0002; // UOUT,   volts   x100
constexpr uint16_t PSU_REG_IOUT    = 0x0003; // IOUT,   amps    x1000
constexpr uint16_t PSU_REG_POWER   = 0x0004; // POWER,  watts   x100
constexpr uint16_t PSU_REG_VIN     = 0x0005; // UIN,    volts   x100
constexpr uint16_t PSU_REG_LOCK    = 0x0006; // LOCK: 0=unlocked, 1=locked, R/W
constexpr uint16_t PSU_REG_PROTECT = 0x0007; // PROTECT: 0 normal, 1 OVP, 2 OCP, 3 OPP
constexpr uint16_t PSU_REG_CVCC    = 0x0008; // CVCC: 0=CV, 1=CC
constexpr uint16_t PSU_REG_ONOFF   = 0x0009; // ONOFF: 0=off, 1=on, R/W
constexpr uint16_t PSU_QUERY_COUNT = 0x000A; // read VSET + ISET

HardwareSerial PsuSerial(PB11, PB10); // RX, TX

/* ============================================================
 * Helpers
 * ============================================================ */

uint16_t readU16LE(const uint8_t *data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint8_t trampChecksum(const uint8_t *packet)
{
  uint8_t checksum = 0;

  // IRC Tramp: bytes 1..13 are summed.
  for (size_t i = 1; i < 14; i++) {
    checksum = static_cast<uint8_t>(checksum + packet[i]);
  }

  return checksum;
}

void buildTrampFrame(char command, uint16_t value, uint8_t frame[TRAMP_PACKET_SIZE])
{
  memset(frame, 0, TRAMP_PACKET_SIZE);

  frame[0] = 0x0F;
  frame[1] = static_cast<uint8_t>(command);
  frame[2] = value & 0xFF;
  frame[3] = (value >> 8) & 0xFF;
  frame[14] = trampChecksum(frame);
}

void printHexPacket(const char *prefix, const uint8_t packet[TRAMP_PACKET_SIZE])
{
  Serial.print(prefix);

  for (size_t i = 0; i < TRAMP_PACKET_SIZE; i++) {
    if (packet[i] < 0x10) {
      Serial.print('0');
    }

    Serial.print(packet[i], HEX);

    if (i + 1 < TRAMP_PACKET_SIZE) {
      Serial.print(' ');
    }
  }

  Serial.println();
}

void clearVtxRx()
{
  while (VTXSerial.available() > 0) {
    VTXSerial.read();
  }
}

/* ============================================================
 * Response validation
 *
 * Header/command/checksum match is necessary but, in theory, could
 * also be satisfied by our own request looping back. As a defensive
 * check (belt-and-braces alongside the proper RX switch below), the
 * 'v' response is additionally required to carry a non-zero
 * frequency field, since a real VTX status reply always has one.
 * ============================================================ */

bool hasPlausiblePayload(const uint8_t packet[TRAMP_PACKET_SIZE], char expectedCommand)
{
  switch (expectedCommand) {
    case 'v':
      return readU16LE(&packet[2]) != 0;

    default:
      return true;
  }
}

bool isValidTrampResponse(const uint8_t packet[TRAMP_PACKET_SIZE], char expectedCommand)
{
  if (packet[0] != 0x0F) {
    return false;
  }

  if (packet[1] != static_cast<uint8_t>(expectedCommand)) {
    return false;
  }

  if (packet[14] != trampChecksum(packet)) {
    return false;
  }

  return hasPlausiblePayload(packet, expectedCommand);
}

/* ============================================================
 * Byte-synchronized response reader.
 *
 * Rather than blindly reading a fixed 16 bytes back-to-back (which
 * breaks if there's any stray byte or timing gap on the single-wire
 * link), this resyncs on 0x0F followed by the expected command byte,
 * the same approach as the known-working query sketch.
 * ============================================================ */

bool readTrampResponse(char expectedCommand, uint8_t response[TRAMP_PACKET_SIZE], uint32_t timeoutMs)
{
  size_t position = 0;
  uint32_t startedAt = millis();

  memset(response, 0, TRAMP_PACKET_SIZE);

  while (millis() - startedAt < timeoutMs) {
    while (VTXSerial.available() > 0) {
      uint8_t receivedByte = static_cast<uint8_t>(VTXSerial.read());

      if (position == 0) {
        if (receivedByte == 0x0F) {
          response[0] = receivedByte;
          position = 1;
        }

        continue;
      }

      if (position == 1) {
        if (receivedByte == static_cast<uint8_t>(expectedCommand)) {
          response[1] = receivedByte;
          position = 2;
        }
        else if (receivedByte == 0x0F) {
          // Could be the start of a new frame.
          response[0] = 0x0F;
          position = 1;
        }
        else {
          position = 0;
        }

        continue;
      }

      response[position++] = receivedByte;

      if (position == TRAMP_PACKET_SIZE) {
        if (isValidTrampResponse(response, expectedCommand)) {
          return true;
        }

        // Corrupt frame or a false-positive sync - keep looking.
        memset(response, 0, TRAMP_PACKET_SIZE);
        position = 0;
      }
    }

    delay(1);
  }

  return false;
}

/* ============================================================
 * Low-level frame transmit, shared by both the fire-and-forget
 * "set" path and the "query" path below.
 *
 * After transmitting, we explicitly switch the shared PA9 pin back
 * to RX with enableHalfDuplexRx() - without this the pin stays
 * driven as TX and no reply is ever captured.
 * ============================================================ */

bool trampSendFrame(char command, uint16_t value, bool silent = false)
{
  uint8_t frame[TRAMP_PACKET_SIZE];
  buildTrampFrame(command, value, frame);

  // Don't hammer the VTX faster than it can handle.
  uint32_t elapsed = millis() - lastRequestTimeMs;

  if (elapsed < TRAMP_MIN_REQUEST_INTERVAL_MS) {
    delay(TRAMP_MIN_REQUEST_INTERVAL_MS - elapsed);
  }

  clearVtxRx();

  if (!silent) {
    //printHexPacket("TX: ", frame);
  }

  /*
   * write() drives PA9 as TX. flush() waits for the last bit to
   * finish shifting out before we touch the pin direction again.
   */
  size_t written = VTXSerial.write(frame, TRAMP_PACKET_SIZE);
  VTXSerial.flush();

  VTXSerial.enableHalfDuplexRx();

  lastRequestTimeMs = millis();

  return written == TRAMP_PACKET_SIZE;
}

/*
 * trampSet: for 'F', 'P', 'I' - this VTX does not acknowledge these,
 * it just applies them silently. Don't wait for (or expect) a reply;
 * waiting here would just burn the full timeout on every call. Use
 * a 'v' query afterward if you need to confirm the change stuck.
 */
bool trampSet(char command, uint16_t value)
{
  return trampSendFrame(command, value);
}

/*
 * trampQuery: for 'v' (and 'r'/'s' if added later) - these commands
 * do get a data-bearing reply, so send and then wait for it.
 */
bool trampQuery(char command, uint8_t response[TRAMP_PACKET_SIZE], bool silent = false)
{
  if (!trampSendFrame(command, 0, silent)) {
    return false;
  }

  bool ok = readTrampResponse(command, response, TRAMP_RESPONSE_TIMEOUT_MS);

  if (ok && !silent) {
    //printHexPacket("RX: ", response);
  }

  return ok;
}

/*
 * queryVtxRaw: fetch the 'v' fields without printing anything itself.
 * Used both by the V command (which formats and prints the result)
 * and internally, after F_/P_/OFF, to verify a change actually stuck.
 *
 * Retries up to VTX_QUERY_MAX_ATTEMPTS times: a single dropped or
 * corrupted reply on the shared single-wire link is normal and
 * shouldn't by itself be reported as VTX_ALIVE=0 / ERR, or (worse)
 * silently leave the caller working from a stale value.
 */
bool queryVtxRaw(
    uint16_t &frequency,
    uint16_t &configuredPower,
    uint8_t &controlMode,
    uint8_t &pitMode,
    uint16_t &actualPower,
    bool silent
)
{
  uint8_t response[TRAMP_PACKET_SIZE];

  for (uint8_t attempt = 0; attempt < VTX_QUERY_MAX_ATTEMPTS; attempt++) {
    if (attempt > 0) {
      delay(VTX_QUERY_RETRY_DELAY_MS);
    }

    if (trampQuery('v', response, silent)) {
      frequency       = readU16LE(&response[2]);
      configuredPower = readU16LE(&response[4]);
      controlMode     = response[6];
      pitMode         = response[7];
      actualPower     = readU16LE(&response[8]);

      return true;
    }
  }

  return false;
}

/* ============================================================
 * V: full status query - doubles as the "is VTX alive" check.
 * ============================================================ */

bool queryVtxStatus()
{
  uint16_t frequency;
  uint16_t configuredPower;
  uint8_t  controlMode;
  uint8_t  pitMode;
  uint16_t actualPower;

  if (!queryVtxRaw(frequency, configuredPower, controlMode, pitMode, actualPower, false)) {
    Serial.println("VTX_ALIVE=0");
    return false;
  }

  Serial.print("VTX_ALIVE=1;FREQUENCY_MHZ=");
  Serial.print(frequency);
  Serial.print(";CONFIGURED_POWER_MW=");
  Serial.print(configuredPower);
  Serial.print(";ACTUAL_POWER_MW=");
  Serial.print(actualPower);
  Serial.print(";CONTROL_MODE=");
  Serial.print(controlMode);
  Serial.print(";PIT_MODE=");
  Serial.print(pitMode);
  Serial.print(";RACE_LOCK=");
  Serial.print((controlMode & 0x01U) ? 1 : 0);
  Serial.print(";TEMP=");

  if (lastKnownTempC == DEVICE_DISCONNECTED_C) {
    Serial.println("ERROR");
  }
  else {
    Serial.println(lastKnownTempC, 2);
  }

  return true;
}

/* ============================================================
 * Pit mode + frequency/power setters
 * ============================================================ */

void exitPitMode()
{
  trampSet(PIT_MODE_COMMAND, PIT_MODE_OFF);
}

void setVtxFrequency(uint16_t freqMHz)
{
  trampSet('F', freqMHz);
}

void setVtxPower(uint16_t powerMw)
{
  trampSet('P', powerMw);
}

/* ============================================================
 * Confirmation polling
 *
 * F/P/pit-mode changes aren't acknowledged by this VTX - it applies
 * them internally (PLL relock, RF amp ramp/calibration, etc.) over
 * some VTX-dependent amount of time. A single fixed delay() before
 * reading the setting back is a guess: too short, and a real,
 * successful change gets misreported as INVALID_INPUT/ERR because we
 * looked before the VTX finished applying it. Instead, poll with
 * repeated 'v' queries (each already internally retried, see
 * queryVtxRaw above) until either the expected value shows up or
 * timeoutMs elapses.
 * ============================================================ */

bool confirmVtxFreqPower(
    uint16_t expectedFreq,
    uint16_t expectedPower,
    uint32_t timeoutMs,
    uint16_t &readFrequency,
    uint16_t &readConfiguredPower,
    uint8_t &readControlMode,
    uint8_t &readPitMode,
    uint16_t &readActualPower
)
{
  uint32_t startedAt = millis();

  do {
    if (queryVtxRaw(readFrequency, readConfiguredPower, readControlMode,
                     readPitMode, readActualPower, true)) {
      if (readFrequency == expectedFreq && readConfiguredPower == expectedPower) {
        return true;
      }
    }
    // trampSendFrame() already enforces TRAMP_MIN_REQUEST_INTERVAL_MS
    // between requests, so no extra delay() is needed here.
  } while (millis() - startedAt < timeoutMs);

  return false; // out-params hold whatever the last read-back was
}

bool confirmVtxPitMode(
    uint16_t expectedPitMode,
    uint32_t timeoutMs,
    uint16_t &readFrequency,
    uint16_t &readConfiguredPower,
    uint8_t &readControlMode,
    uint8_t &readPitMode,
    uint16_t &readActualPower
)
{
  uint32_t startedAt = millis();

  do {
    if (queryVtxRaw(readFrequency, readConfiguredPower, readControlMode,
                     readPitMode, readActualPower, true)) {
      if (readPitMode == expectedPitMode) {
        return true;
      }
    }
  } while (millis() - startedAt < timeoutMs);

  return false;
}

void handleOffCommand()
{
  if (!trampSet(PIT_MODE_COMMAND, PIT_MODE_ON)) {
    Serial.println("ERR");
    return;
  }

  uint16_t frequency;
  uint16_t configuredPower;
  uint8_t  controlMode;
  uint8_t  pitMode;
  uint16_t actualPower;

  bool matched = confirmVtxPitMode(PIT_MODE_ON, VTX_CONFIRM_TIMEOUT_MS,
                                    frequency, configuredPower, controlMode,
                                    pitMode, actualPower);

  Serial.println(matched ? "OK" : "ERR");
}

/* ============================================================
 * DS18B20 temperature sensor - non-blocking, runs every loop().
 *
 * A DS18B20 conversion takes ~750ms, and we don't want that to
 * block the VTX command handling. So this is a small state machine:
 * kick off a conversion, remember when, and only read the result
 * back once enough time has passed - all without ever calling
 * delay() for the full second.
 * ============================================================ */

void handleTemperatureSensor()
{
  uint32_t now = millis();

  switch (tempState) {
    case TempState::Idle:
      if (now - lastTempRequestTimeMs >= TEMP_READ_INTERVAL_MS) {
        tempSensor.requestTemperatures(); // non-blocking: setWaitForConversion(false)
        tempConversionStartMs = now;
        tempState = TempState::Converting;
      }
      break;

    case TempState::Converting:
      if (now - tempConversionStartMs >= DS18B20_CONVERSION_MS) {
        lastKnownTempC = tempSensor.getTempCByIndex(0);

        lastTempRequestTimeMs = now;
        tempState = TempState::Idle;
      }
      break;
  }
}

/* ============================================================
 * Modbus-RTU master (Ruideng DPH5005 PSU on USART3, PB10/PB11)
 *
 * The PSU exposes 0x03 (read holding registers), 0x06 (write single
 * register) and 0x10 (write multiple registers) only. This is a
 * plain point-to-point TTL link (no shared bus/other master), so a
 * simple send-then-wait-for-reply model is enough - no bus
 * arbitration is needed.
 * ============================================================ */

uint16_t modbusCRC16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFF;

  for (size_t pos = 0; pos < length; pos++) {
    crc ^= data[pos];

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      }
      else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void modbusFlushRx()
{
  while (PsuSerial.available() > 0) {
    PsuSerial.read();
  }
}

/*
 * Reads whatever the PSU sends back, stopping either once we've
 * filled the buffer or once there's been a short silence after at
 * least one byte arrived (Modbus RTU frames are delimited by an
 * inter-frame gap, not a fixed length known in advance to us here).
 */
size_t modbusReadFrame(uint8_t *buffer, size_t maxLen, uint32_t timeoutMs)
{
  size_t received = 0;
  uint32_t startedAt = millis();
  uint32_t lastByteAt = startedAt;

  while (millis() - startedAt < timeoutMs) {
    if (PsuSerial.available() > 0) {
      uint8_t receivedByte = static_cast<uint8_t>(PsuSerial.read());

      if (received < maxLen) {
        buffer[received++] = receivedByte;
      }

      lastByteAt = millis();
    }
    else if (received > 0 && (millis() - lastByteAt > PSU_INTERBYTE_TIMEOUT_MS)) {
      break; // frame looks complete
    }
  }

  return received;
}

bool modbusFrameCrcOk(const uint8_t *frame, size_t length)
{
  if (length < 4) {
    return false;
  }

  uint16_t computedCrc = modbusCRC16(frame, length - 2);
  uint16_t receivedCrc = readU16LE(&frame[length - 2]); // CRC is little-endian on the wire

  return computedCrc == receivedCrc;
}

void debugPrintPsuFrame(const uint8_t *frame, size_t length)
{
  if (!PSU_DEBUG_RAW_BYTES) {
    return;
  }

  Serial.print("PSU raw bytes: ");
  Serial.println(length);

  for (size_t i = 0; i < length; i++) {
    if (frame[i] < 0x10) {
      Serial.print('0');
    }

    Serial.print(frame[i], HEX);
    Serial.print(' ');
  }

  Serial.println();
}

bool modbusReadHoldingRegisters(uint8_t slaveAddr, uint16_t startReg, uint16_t count, uint16_t *outValues, uint32_t timeoutMs)
{
  uint8_t request[8];

  request[0] = slaveAddr;
  request[1] = 0x03;
  request[2] = highByte(startReg);
  request[3] = lowByte(startReg);
  request[4] = highByte(count);
  request[5] = lowByte(count);

  uint16_t crc = modbusCRC16(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  modbusFlushRx();
  delay(10);
  PsuSerial.write(request, sizeof(request));
  PsuSerial.flush();

  uint8_t response[MODBUS_MAX_FRAME];
  size_t responseLen = modbusReadFrame(response, sizeof(response), timeoutMs);

  debugPrintPsuFrame(response, responseLen);

  size_t expectedLen = 5 + static_cast<size_t>(count) * 2; // addr+func+bytecount+data+crc(2)

  if (responseLen != expectedLen || !modbusFrameCrcOk(response, responseLen)) {
    return false;
  }

  if (response[0] != slaveAddr || response[1] != 0x03 || response[2] != count * 2) {
    return false; // also catches an 0x83 exception reply
  }

  for (uint16_t i = 0; i < count; i++) {
    // Register bytes are big-endian in the Modbus payload, unlike
    // the little-endian CRC - build the value explicitly:
    outValues[i] = (static_cast<uint16_t>(response[3 + i * 2]) << 8) |
                   static_cast<uint16_t>(response[4 + i * 2]);
  }

  return true;
}

bool modbusWriteSingleRegister(uint8_t slaveAddr, uint16_t reg, uint16_t value, uint32_t timeoutMs)
{
  uint8_t request[8];

  request[0] = slaveAddr;
  request[1] = 0x06;
  request[2] = highByte(reg);
  request[3] = lowByte(reg);
  request[4] = highByte(value);
  request[5] = lowByte(value);

  uint16_t crc = modbusCRC16(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  modbusFlushRx();
  PsuSerial.write(request, sizeof(request));
  PsuSerial.flush();

  uint8_t response[MODBUS_MAX_FRAME];
  size_t responseLen = modbusReadFrame(response, sizeof(response), timeoutMs);

  debugPrintPsuFrame(response, responseLen);

  if (responseLen != 8 || !modbusFrameCrcOk(response, responseLen)) {
    return false;
  }

  // A successful 0x06 reply echoes the request's address/function/
  // register/value exactly; anything else (including an 0x86
  // exception reply) is treated as a failure.
  return memcmp(response, request, 6) == 0;
}

bool modbusWriteMultipleRegisters(uint8_t slaveAddr, uint16_t startReg, const uint16_t *values, uint16_t count, uint32_t timeoutMs)
{
  uint8_t request[7 + 2 * 8 + 2]; // header(7) + up to 8 registers + crc(2)
  uint8_t byteCount = static_cast<uint8_t>(count * 2);

  request[0] = slaveAddr;
  request[1] = 0x10;
  request[2] = highByte(startReg);
  request[3] = lowByte(startReg);
  request[4] = highByte(count);
  request[5] = lowByte(count);
  request[6] = byteCount;

  for (uint16_t i = 0; i < count; i++) {
    request[7 + i * 2]     = highByte(values[i]);
    request[7 + i * 2 + 1] = lowByte(values[i]);
  }

  size_t requestLen = 7 + byteCount;
  uint16_t crc = modbusCRC16(request, requestLen);
  request[requestLen]     = lowByte(crc);
  request[requestLen + 1] = highByte(crc);
  requestLen += 2;

  modbusFlushRx();
  PsuSerial.write(request, requestLen);
  PsuSerial.flush();

  uint8_t response[MODBUS_MAX_FRAME];
  size_t responseLen = modbusReadFrame(response, sizeof(response), timeoutMs);

  debugPrintPsuFrame(response, responseLen);

  if (responseLen != 8 || !modbusFrameCrcOk(response, responseLen)) {
    return false;
  }

  // A successful 0x10 reply echoes address/function/start-reg/count
  // (not the data itself).
  return memcmp(response, request, 6) == 0;
}

/* ============================================================
 * Retry wrappers
 *
 * Point-to-point Modbus link over a bare UART: an occasional single
 * corrupted or missed reply is normal (electrical noise, or the PSU
 * being briefly busy with its own display/keypad scan). Rather than
 * reporting a failure on the very first miss, each PSU transaction
 * is retried a few times (PSU_MAX_ATTEMPTS) with a short pause
 * (PSU_RETRY_DELAY_MS) between attempts before giving up.
 * ============================================================ */

bool modbusReadHoldingRegistersRetry(uint8_t slaveAddr, uint16_t startReg, uint16_t count, uint16_t *outValues, uint32_t timeoutMs)
{
  for (uint8_t attempt = 0; attempt < PSU_MAX_ATTEMPTS; attempt++) {
    if (attempt > 0) {
      delay(PSU_RETRY_DELAY_MS);
    }

    if (modbusReadHoldingRegisters(slaveAddr, startReg, count, outValues, timeoutMs)) {
      return true;
    }
  }

  return false;
}

bool modbusWriteSingleRegisterRetry(uint8_t slaveAddr, uint16_t reg, uint16_t value, uint32_t timeoutMs)
{
  for (uint8_t attempt = 0; attempt < PSU_MAX_ATTEMPTS; attempt++) {
    if (attempt > 0) {
      delay(PSU_RETRY_DELAY_MS);
    }

    if (modbusWriteSingleRegister(slaveAddr, reg, value, timeoutMs)) {
      return true;
    }
  }

  return false;
}

bool modbusWriteMultipleRegistersRetry(uint8_t slaveAddr, uint16_t startReg, const uint16_t *values, uint16_t count, uint32_t timeoutMs)
{
  for (uint8_t attempt = 0; attempt < PSU_MAX_ATTEMPTS; attempt++) {
    if (attempt > 0) {
      delay(PSU_RETRY_DELAY_MS);
    }

    if (modbusWriteMultipleRegisters(slaveAddr, startReg, values, count, timeoutMs)) {
      return true;
    }
  }

  return false;
}

/* ============================================================
 * PS: full PSU status query - doubles as the "is PSU alive" check.
 * ============================================================ */

bool queryPsuStatus()
{
  uint16_t regs[PSU_QUERY_COUNT];

  if (!modbusReadHoldingRegistersRetry(PSU_SLAVE_ADDR, 0x0000, PSU_QUERY_COUNT, regs, PSU_RESPONSE_TIMEOUT_MS)) {
    Serial.println("PSU_ALIVE=0");
    return false;
  }

  Serial.print("PSU_ALIVE=1;VSET=");
  Serial.print(regs[PSU_REG_VSET] / 100.0f, 2);
  Serial.print(";ISET=");
  Serial.print(regs[PSU_REG_ISET] / 1000.0f, 3);
  Serial.print(";VOUT=");
  Serial.print(regs[PSU_REG_VOUT] / 100.0f, 2);
  Serial.print(";IOUT=");
  Serial.print(regs[PSU_REG_IOUT] / 1000.0f, 3);
  Serial.print(";POWER_W=");
  Serial.print(regs[PSU_REG_POWER] / 100.0f, 2);
  Serial.print(";VIN=");
  Serial.print(regs[PSU_REG_VIN] / 100.0f, 2);
  Serial.print(";LOCK=");
  Serial.print(regs[PSU_REG_LOCK]);
  Serial.print(";CVCC=");
  Serial.print(regs[PSU_REG_CVCC]);
  Serial.print(";ONOFF=");
  Serial.print(regs[PSU_REG_ONOFF]);
  Serial.print(";PROTECT=");
  Serial.println(regs[PSU_REG_PROTECT]);

  return true;
}

/*
 * handlePsuSetCommand: parses "PS_SET_<v>_<i>" in the device's native
 * register units - <v> is volts x100 (1200 = 12.00V), <i> is amps
 * x1000 (2000 = 2.000A) - and writes U-SET/I-SET together in a
 * single 0x10 request (they're adjacent registers, 0x0000/0x0001).
 */
bool handlePsuSetCommand(const String& cmd)
{
    constexpr int PREFIX_LEN = 7; // "PS_SET_"
    int splitIndex = cmd.indexOf('_', PREFIX_LEN);
    if (splitIndex < 0) {
        Serial.println("ERR");
        return false;
    }

    uint16_t vCenti = static_cast<uint16_t>(cmd.substring(PREFIX_LEN, splitIndex).toInt());
    uint16_t iMilli = static_cast<uint16_t>(cmd.substring(splitIndex + 1).toInt());

    uint16_t values[2] = { vCenti, iMilli }; // VSET (0x0000), ISET (0x0001)

    if (!modbusWriteMultipleRegistersRetry(PSU_SLAVE_ADDR, PSU_REG_VSET, values, 2, PSU_RESPONSE_TIMEOUT_MS)) {
        Serial.println("ERR");
        return false;
    }

    delay(200);

    uint16_t readBack[2] = {0};
    if (!modbusReadHoldingRegistersRetry(PSU_SLAVE_ADDR, PSU_REG_VSET, 2, readBack, PSU_RESPONSE_TIMEOUT_MS)) {
        Serial.println("ERR");
        return false;
    }

    if (readBack[0] != vCenti || readBack[1] != iMilli) {
        Serial.println("ERR");
        return false;
    }

    Serial.println("OK");
    return true;
}
void handlePsuOnOffCommand(bool turnOn)
{
  if (!modbusWriteSingleRegisterRetry(PSU_SLAVE_ADDR, PSU_REG_ONOFF, turnOn ? 1 : 0, PSU_RESPONSE_TIMEOUT_MS)) {
    Serial.println("ERR");
    return;
  }

  Serial.println("OK");
}

/* ============================================================
 * Command parsing
 * ============================================================ */

void handleFreqPowerCommand(const String &cmd)
{
  int pIndex = cmd.indexOf("_P_");

  uint16_t freqMHz = cmd.substring(2, pIndex).toInt();
  uint16_t powerMw = cmd.substring(pIndex + 3).toInt();

  if (freqMHz == 0 || powerMw == 0) {
    Serial.println("ERR");
    return;
  }

  // A new frequency/power command always brings the VTX out of Pit Mode.
  exitPitMode();
  delay(50);

  setVtxFrequency(freqMHz);
  delay(300);

  setVtxPower(powerMw);
  delay(150); // small settle before we start polling for confirmation

  uint16_t readFrequency;
  uint16_t readConfiguredPower;
  uint8_t  readControlMode;
  uint8_t  readPitMode;
  uint16_t readActualPower;

  // Poll (rather than a single fixed-delay check) because how long the
  // VTX takes to actually apply F/P internally (PLL relock, RF amp
  // ramp/calibration) isn't something this fire-and-forget protocol
  // tells us - a too-short single check misreports a real, successful
  // change as INVALID_INPUT.
  bool matched = confirmVtxFreqPower(freqMHz, powerMw, VTX_CONFIRM_TIMEOUT_MS,
                                      readFrequency, readConfiguredPower,
                                      readControlMode, readPitMode, readActualPower);

  // If the VTX rejected an out-of-range value, it just keeps its old
  // setting - so no match within the timeout means the requested value
  // didn't stick.
  if (!matched) {
    Serial.println("INVALID_INPUT");
    return;
  }

  Serial.println("OK");
}

void processCommand(String cmd)
{
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "V") {
    queryVtxStatus();
    return;
  }

  if (cmd == "OFF") {
    handleOffCommand();
    return;
  }

  if (cmd.startsWith("F_") && cmd.indexOf("_P_") > 0) {
    handleFreqPowerCommand(cmd);
    return;
  }

  if (cmd == "PS") {
    queryPsuStatus();
    return;
  }

  if (cmd == "PS_ON") {
    handlePsuOnOffCommand(true);
    return;
  }

  if (cmd == "PS_OFF") {
    handlePsuOnOffCommand(false);
    return;
  }

  if (cmd.startsWith("PS_SET_")) {
    handlePsuSetCommand(cmd);
    return;
  }

  Serial.println("ERR");
}

/* ============================================================
 * Arduino
 * ============================================================ */

void setup()
{
  VTXSerial.begin(VTX_BAUD);

  // Idle state should be RX so we're ready for any unsolicited data
  // and so the very first request's reply isn't missed.
  VTXSerial.enableHalfDuplexRx();

  // Redirect the console/debug link from its default pins onto
  // USART2 (PA2=TX, PA3=RX) for an external USB-TTL adapter - must be
  // called before begin().
  Serial.setRx(PA3);
  Serial.setTx(PA2);
  Serial.begin(115200);

  PsuSerial.begin(PSU_BAUD);

  tempSensor.begin();
  tempSensor.setWaitForConversion(false); // non-blocking: we poll for it ourselves

  delay(200);
}

void loop()
{
  handleTemperatureSensor();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}
