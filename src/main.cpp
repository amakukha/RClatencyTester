#include <Arduino.h>
#include "SoftwareSerial.h"

#define NUM_OF_TESTS                500

#define RX_THRESHOLD_HIGH           1900
#define RX_THRESHOLD_LOW            1100

#define TRIGGER_WAIT_RAND_MIN_MS    200000
#define TRIGGER_WAIR_RAND_MAX_MS    350000

#define START_BUTTON_PIN            0
#define GPIO_OUTPUT_PIN             14
#define RX_SERIAL_PIN               15

#define USB_SERIAL_PIN              1
#define USB_SERIAL_BAUDRATE         19200

SoftwareSerial usbSerial;
auto &testSerial = Serial;

//#define USE_GHST
#define USE_CRSF
//#define USE_SBUS
//#define USE_IBUS
////#define USE_SRXL2

bool testRunning = false;
bool valueSentHigh = false;
uint32_t testCount = 0;
uint32_t invalidValueCount = 0;
uint16_t ChannelData[16];

uint32_t latencyResult[NUM_OF_TESTS] = {0};

#ifdef USE_GHST
#include "ghst.h"
GHST ghst(Serial);
#endif

#ifdef USE_CRSF
#include "CRSF.h"
CRSF crsf(Serial);
#endif

#ifdef USE_SBUS
#include "SBUS.h"
SBUS sbus(Serial);
bool failSafe;
bool lostFrame;
#endif

#ifdef USE_IBUS
#include "IBUS.h"
IBUS ibus(Serial);
#endif

#ifdef USE_SRXL2
#include "spm_srxl.h"
#include "spm_srxl_config.h"

void uartInit(uint8_t uartNum, uint32_t baudRate)
{
  Serial.begin(baudRate, SERIAL_8N1, SERIAL_FULL, 1, false);
}

void uartSetBaud(uint8_t uartNum, uint32_t baudRate)
{
  Serial.begin(baudRate, SERIAL_8N1, SERIAL_FULL, 1, false);
}

uint8_t uartReceiveBytes(uint8_t uartNum, uint8_t *pBuffer, uint8_t bufferSize, uint8_t timeout_ms)
{
  Serial.setTimeout(timeout_ms);
  Serial.readBytes(pBuffer, bufferSize);
}

uint8_t uartTransmit(uint8_t uartNum, uint8_t *pBuffer, uint8_t bytesToSend)
{
  Serial.write(pBuffer, bytesToSend);
}

// Forward definitions of app-specific handling of telemetry and channel data -- see examples below
void userProvidedFillSrxlTelemetry(SrxlTelemetryData *pTelemetry)
{
  Serial.println("TLM");
}

void userProvidedReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafeData)
{
  Serial.println("Data");
}

void userProvidedHandleVtxData(SrxlVtxData *pVtxData)
{
  Serial.println("Data");
}

#endif

uint32_t TriggerBeginTime;

uint32_t BeginTriggerMicros;

uint32_t lastRCdataMicros;
uint32_t currRCdataMicros;

enum {
    STATE_IDLE,
    STATE_SCHEDULED,
    STATE_TRIGGERED
};

uint8_t CurrState;

void IRAM_ATTR clear_array(uint32_t *array, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
    array[i] = 0;
}

double IRAM_ATTR average(uint32_t *array, uint32_t len)
{
  double sum = 0; // sum will be larger than an item, long for safety.
  for (uint32_t i = 0; i < len; i++)
  {
    sum += array[i];
  }
  return ((double)sum / (double)len); // average will be fractional, so float may be appropriate.
}

void IRAM_ATTR RCcallback(volatile uint16_t *data)
{
  uint32_t now = micros();
  if (testRunning && CurrState == STATE_TRIGGERED)
  {
    uint16_t chanVal = data[2];

    usbSerial.print((int)chanVal);
    usbSerial.println(" received");

    if (( valueSentHigh && chanVal >= RX_THRESHOLD_HIGH) ||
        (!valueSentHigh && chanVal <= RX_THRESHOLD_LOW))
    {
      CurrState = STATE_IDLE;
      uint32_t result = now - BeginTriggerMicros;

      // Record result
      latencyResult[testCount] = result;
      usbSerial.print(testCount);
      usbSerial.print(",");
      usbSerial.print(result);
      usbSerial.print(",");
      usbSerial.println(now - lastRCdataMicros);

      // Check if test finished
      if (testCount++ >= NUM_OF_TESTS)
      {
        testRunning = false;
        testCount = 0;
        usbSerial.println("===== FINISHED =====");
        usbSerial.print("AVERAGE: ");
        usbSerial.print(average(latencyResult, NUM_OF_TESTS));
        usbSerial.println(" microSeconds");
        if (invalidValueCount > 0) {
          usbSerial.print("INVALID VALUES: ");
          usbSerial.println(invalidValueCount);
          invalidValueCount = 0;
        }
        clear_array(latencyResult, sizeof(latencyResult));
      }
    }
    else
    {
      invalidValueCount++;
    }
  }
  lastRCdataMicros = now;
}

void inline CRSF_GHST_RC_CALLBACK()
{
#if defined(USE_CRSF)
  RCcallback(crsf.ChannelDataIn);
#elif defined(USE_GHST)
  RCcallback(ghst.ChannelDataIn);
#elif defined(USE_IBUS)
  RCcallback(ibus.ChannelDataIn);
#endif
}

void inline PreTrigger()
{
  TriggerBeginTime = random(TRIGGER_WAIT_RAND_MIN_MS, TRIGGER_WAIR_RAND_MAX_MS) + micros();
  CurrState = STATE_SCHEDULED;
}

void inline DoTrigger()
{
  // Toggle GPIO value
  valueSentHigh = !valueSentHigh;
  usbSerial.print((int)valueSentHigh);
  usbSerial.println(" sent");
  digitalWrite(GPIO_OUTPUT_PIN, valueSentHigh ? HIGH : LOW);
  BeginTriggerMicros = micros();
  CurrState = STATE_TRIGGERED;
}

void setup()
{
  pinMode(GPIO_OUTPUT_PIN, OUTPUT);
  pinMode(D1, OUTPUT);
  digitalWrite(D1, LOW);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  CurrState = STATE_IDLE;
  clear_array(latencyResult, NUM_OF_TESTS);

#if defined(USE_CRSF)
  crsf.RCdataCallback = &CRSF_GHST_RC_CALLBACK;
  crsf.Begin();
  Serial.begin(CRSF_RX_BAUDRATE, SERIAL_8N1, SERIAL_FULL, RX_SERIAL_PIN, false);
#elif defined(USE_GHST)
  ghst.RCdataCallback = &CRSF_GHST_RC_CALLBACK;
  ghst.Begin();
  Serial.begin(GHST_RX_BAUDRATE, SERIAL_8N1, SERIAL_FULL, RX_SERIAL_PIN, false);
#elif defined(USE_SBUS)
  sbus.begin();
#elif defined(USE_IBUS)
  ibus.RCdataCallback = &CRSF_GHST_RC_CALLBACK;
#endif

  Serial.swap();

  usbSerial.begin(USB_SERIAL_BAUDRATE, SWSERIAL_8N1, 3, USB_SERIAL_PIN, false, 256);
  usbSerial.enableIntTx(false);
  usbSerial.println("Softserial Mon Started");
  usbSerial.println("Press Button to Begin Test");
}

#ifdef USE_SBUS
void loop_sbus()
{
  if (sbus.read(&ChannelData[0], &failSafe, &lostFrame))
  {
    RCcallback(ChannelData);
  }
}
#endif

#ifdef USE_SRXL2
void loop_srxl2()
{
  while (Serial.available())
  {
    // Try to receive UART bytes, or timeout after 5 ms
    uint8_t bytesReceived = Serial.read();
    Serial.println(bytesReceived);
    if (bytesReceived)
    {
      rxBufferIndex += bytesReceived;
      if (rxBufferIndex < 5)
        continue;

      if (rxBuffer[0] == SPEKTRUM_SRXL_ID)
      {
        uint8_t packetLength = rxBuffer[2];
        if (rxBufferIndex > packetLength)
        {
          // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
          if (srxlParsePacket(0, rxBuffer, packetLength))
          {
            // Move any remaining bytes to beginning of buffer (usually 0)
            rxBufferIndex -= packetLength;
            memmove(rxBuffer, &rxBuffer[packetLength], rxBufferIndex);
          }
          else
          {
            rxBufferIndex = 0;
          }
        }
      }
    }
    else
    {
      // Tell SRXL state machine that 5 more milliseconds have passed since packet received
      srxlRun(0, 5);
      rxBufferIndex = 0;
    }
    // Check a bind button, and if pressed enter bind mode
    // if (bindButtonPressed)
    // {
    //   srxlEnterBind(DSMX_11MS);
    // }
  }
}
#endif

void loop()
{
  if (testRunning)
  {
    if (CurrState == STATE_IDLE)
    {
      PreTrigger();
    }
    else if (CurrState == STATE_SCHEDULED)
    {
      if (micros() > TriggerBeginTime)
      {
        DoTrigger();
      }
    }
  }

  // Handle serial to receiver
#if defined(USE_CRSF)
  crsf.handleUARTin();
#elif defined(USE_GHST)
  ghst.handleUARTin();
#elif defined(USE_SRXL2)
  loop_srxl2();
#elif defined(USE_SBUS)
  loop_sbus();
#elif defined(USE_IBUS)
  ibus.loop();
#endif

  if (!testRunning)
  {
    if (digitalRead(START_BUTTON_PIN) == 0)
    {
      usbSerial.println("Begin Test");
      usbSerial.print("Test will run:");
      usbSerial.print(NUM_OF_TESTS);
      usbSerial.println(" times");
      testRunning = true;
      testCount = 0;
      invalidValueCount = 0;
      clear_array(latencyResult, NUM_OF_TESTS);
      usbSerial.println("===== BEGIN =====");
    }
  }
}
