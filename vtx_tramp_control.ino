#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * STM32G0 + Arduino IDE
 *
 * VTX control over the IRC Tramp protocol.
 * PB6 = single-wire hardware half-duplex link to the VTX, 9600 baud.
 * PA5 = DS18B20 temperature sensor (1-Wire, needs a 4.7k pull-up to 3.3V).
 * Serial (USB/console) = command/debug interface, 115200 baud.
 *
 * Requires the "OneWire" and "DallasTemperature" libraries (Library
 * Manager). Temperature is read in the background once per second and
 * exposed via the TEMP_<val> line in the V command's output below -
 * it is not printed on its own.
 *
 * Serial commands (newline terminated, case-insensitive):
 *   F_<freq>_P_<power>   Set frequency (MHz) and power (mW).
 *                        Automatically exits Pit Mode first.
 *   V                    Query VTX status (freq/power/pit mode/etc) plus
 *                        the last known temperature reading (TEMP_<val>).
 *                        Use this as an "is VTX alive" check:
 *                        prints VTX_ALIVE=1 with full status if a
 *                        valid reply came back, VTX_ALIVE=0 if the
 *                        request timed out or the reply was invalid.
 *   OFF                  Put the VTX into Pit Mode.
 *
 * Requires STM32 Arduino Core version that supports HardwareSerial
 * hardware half-duplex (enableHalfDuplexRx). Confirm PB6 maps to a
 * USART/LPUART on your specific STM32G0 part/board that supports the
 * half-duplex peripheral feature - it's an MCU/pin capability, not
 * something the sketch can fake.
 *
 * NOTE: PIT_MODE_COMMAND below ('I') is the commonly documented Tramp
 * command letter for setting Pit Mode. Confirm this against your VTX's
 * own Tramp implementation - neither original version of this sketch
 * ever sent a pit-mode SET frame and confirmed the ack, only read pit
 * mode back passively via the 'v' status response.
 */

constexpr uint8_t  VTX_PIN  = PB6;
constexpr uint32_t VTX_BAUD = 9600;

constexpr size_t   TRAMP_PACKET_SIZE            = 16;
constexpr uint32_t TRAMP_RESPONSE_TIMEOUT_MS     = 400;
constexpr uint32_t TRAMP_MIN_REQUEST_INTERVAL_MS = 200;

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
 * After transmitting, we explicitly switch the shared PB6 pin back
 * to RX with enableHalfDuplexRx() - without this the pin stays
 * driven as TX and no reply is ever captured.
 * ============================================================ */

bool trampSendFrame(char command, uint16_t value)
{
  uint8_t frame[TRAMP_PACKET_SIZE];
  buildTrampFrame(command, value, frame);

  // Don't hammer the VTX faster than it can handle.
  uint32_t elapsed = millis() - lastRequestTimeMs;

  if (elapsed < TRAMP_MIN_REQUEST_INTERVAL_MS) {
    delay(TRAMP_MIN_REQUEST_INTERVAL_MS - elapsed);
  }

  clearVtxRx();
  printHexPacket("TX: ", frame);

  /*
   * write() drives PB6 as TX. flush() waits for the last bit to
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
bool trampQuery(char command, uint8_t response[TRAMP_PACKET_SIZE])
{
  if (!trampSendFrame(command, 0)) {
    return false;
  }

  bool ok = readTrampResponse(command, response, TRAMP_RESPONSE_TIMEOUT_MS);

  if (ok) {
    printHexPacket("RX: ", response);
  }

  return ok;
}

/* ============================================================
 * V: full status query - doubles as the "is VTX alive" check.
 * ============================================================ */

bool queryVtxStatus()
{
  uint8_t response[TRAMP_PACKET_SIZE];

  if (!trampQuery('v', response)) {
    Serial.println("VTX_ALIVE=0");
    return false;
  }

  uint16_t frequency       = readU16LE(&response[2]);
  uint16_t configuredPower = readU16LE(&response[4]);
  uint8_t  controlMode     = response[6];
  uint8_t  pitMode         = response[7];
  uint16_t actualPower     = readU16LE(&response[8]);

  Serial.println("VTX_ALIVE=1");

  Serial.print("FREQUENCY_MHZ=");
  Serial.println(frequency);

  Serial.print("CONFIGURED_POWER_MW=");
  Serial.println(configuredPower);

  Serial.print("ACTUAL_POWER_MW=");
  Serial.println(actualPower);

  Serial.print("CONTROL_MODE=");
  Serial.println(controlMode);

  Serial.print("PIT_MODE=");
  Serial.println(pitMode);

  Serial.print("RACE_LOCK=");
  Serial.println((controlMode & 0x01U) ? 1 : 0);

  Serial.print("TEMP_");

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

void enterPitMode()
{
  trampSet(PIT_MODE_COMMAND, PIT_MODE_ON);
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
    enterPitMode();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("F_") && cmd.indexOf("_P_") > 0) {
    handleFreqPowerCommand(cmd);
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

  Serial.begin(115200);

  tempSensor.begin();
  tempSensor.setWaitForConversion(false); // non-blocking: we poll for it ourselves

  delay(200);
  Serial.println("VTX_READY");
  Serial.println("SEND: F_<freq>_P_<power>, V, or OFF");
}

void loop()
{
  handleTemperatureSensor();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}
