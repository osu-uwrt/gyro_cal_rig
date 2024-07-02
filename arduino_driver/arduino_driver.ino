/**
 * Gyro calibration rig Arduino driver.
 * 
 * Serial message format:
 * In:
 * byte number: 1           2           3-6
 * description: Sync (';')  Status     Speed
 * 
 * Out:
 * byte number: 1           2           3-6
 * description: Sync (';')  Status      Speed
 * 
 * Fields:
 * - Sync: ';'
 * - Enabled: 0 for disabled, nonzero for enabled
 * - Status: Contains various status flags
 *      bit:    7-2     2               1           0
 *      desc:   Unused  Stalled (RO)    Heat (RW)   Enabled (RW)
 * - Speed: Desired speed
 * - Heat: Desired heater state (on/off)
 */

#include <TMC2209.h>

#define ENABLE_PIN 2
#define SEND_INTERVAL 100

SoftwareSerial stepperSerial(10, 11);
TMC2209 stepper;
char
    inBuf[32],
    outBuf[6];

size_t inBufCursor = 0;
const char sync = ';';

bool enabled;
uint16_t speed;
bool heat;
long lastSendTime = 0;

void setStepperEnabled(bool enabled);
void receiveSerial();
void sendSerial();

void setup() {
    //initialize stepper, disabled
    stepper.setup(stepperSerial);
    stepper.enableAutomaticCurrentScaling();
    pinMode(ENABLE_PIN, OUTPUT);
    setStepperEnabled(false);

    //initialize serial
    Serial.begin(9600);
    Serial.setTimeout(100);

    //blink leds to indicate successful setup
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i = 0; i < 2; i++)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(300);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(300);
    }

    lastSendTime = millis();
}


void loop() {
    long currentTime = millis();
    receiveSerial();

    if(currentTime - lastSendTime > SEND_INTERVAL)
    {
        sendSerial();
        lastSendTime = currentTime;
    }
}


void setStepperEnabled(bool enabled)
{
    digitalWrite(ENABLE_PIN, (enabled ? LOW : HIGH));
}


void receiveSerial()
{
    if(Serial.available())
    {
        inBufCursor += Serial.readBytes(&inBuf[inBufCursor], sizeof(inBuf) - inBufCursor);
    }

    char *sync1 = strchr(inBuf, sync);
    if(!sync1)
    {
        return;
    }

    char *sync2 = strchr(sync1 + 1, sync);
    if(!sync2)
    {
        return;
    }

    if(sync2 - sync1 == 6)
    {
        //get enabled
        enabled = sync1[1] & B00000001;

        //get temperature
        heat = sync1[1] & B00000010;

        //get speed
        speed = sync1[2];
        speed = speed << 8;
        speed |= sync1[3] & 0xFF;
        speed = speed << 8;
        speed |= sync1[4] & 0xFF;
        speed = speed << 8;
        speed |= sync1[5] & 0xFF;

        //clear buffer (could handle better but dont really need to)
        memset(inBuf, 0, sizeof(inBuf));
        inBufCursor = 0;
    }
}


void sendSerial()
{
    //sync
    outBuf[0] = sync;

    //status buffer
    if(enabled)
    {
        outBuf[1] |= B00000001;
    } else
    {
        outBuf[1] &= ~B00000001;
    }

    if(heat)
    {
        outBuf[1] |= B00000010;
    } else
    {
        outBuf[1] &= ~B00000010;
    }

    //TODO implement stall detection

    //speed
    outBuf[2] = speed >> 24 & 0xFF;
    outBuf[3] = speed >> 16 & 0xFF;
    outBuf[4] = speed >> 8 & 0xFF;
    outBuf[5] = speed & 0xFF;

    Serial.write(outBuf, sizeof(outBuf));
}