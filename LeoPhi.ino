#include <Wire.h>
#include <avr/eeprom.h>

long int _digitDelay;   // How much time spent per display during multiplexing.
long int _digitOnDelay;   // How much on-time per display (used for dimming), i.e. it could be on only 40% of digitDelay
long int _digitOffDelay;    // digitDelay minus digitOnDelay
int _dutyCycle;   // The duty cycle (digitOnDelay/digitDelay, here in percent)
long int _timerCounter;
long int _fadeCounter = 0;
long int _timerCounterOnEnd;
long int _timerCounterOffEnd;
int _timerPhase;

enum {
  I2C_CMD_GET_PH = 1,
  I2C_CMD_SET_PH4 = 2,
  I2C_CMD_SET_PH7 = 3,
  I2C_CMD_SET_PH10 = 4,
  I2C_CMD_SET_DEFAULT = 5,
  I2C_CMD_SET_PULSE_ON = 6,
  I2C_CMD_SET_GREEN_PULSE = 7,
  I2C_CMD_SET_BLUE_PULSE = 8,
  I2C_CMD_SET_RED_PULSE = 9,
  I2C_CMD_SET_PULSE_OFF = 10,
  GREEN_LED = 6,
  BLUE_LED = 11,
  RED_LED = 12
};

#define BUFFER_SIZE       16
#define NUM_PASSES        20
#define I2C_ADDR          0x5a
#define TWI_FREQ_SETTING  400000L       // 400KHz for I2C
#define CPU_FREQ          16000000L     // 16MHz

// EEPROM trigger check
#define Write_Check      0x1234

int requestedCmd = 0;    // which command was requested (if any)

uint8_t i2cResponse[2];  // array to store response
int i2cResponseLen = 0;  // response length

volatile uint32_t result[BUFFER_SIZE];
volatile int i = 0;
volatile uint32_t sum = 0;

// LED fade effects
unsigned long lastBlink;
uint8_t brightness = 0;
uint8_t fadeAmount = 2.55;
uint8_t fadeInterval = 30; // interval at which to change led brightness

// smoothing
int passes[NUM_PASSES]; // Our storage array
uint8_t passIndex = 0;  // what pass are we on? this one
long total = 0;         // running total

int pHRaw;
int pHSmooth = 0;       // Our smoothed pHRaw number
int intPh = 0;          // Our pH float as an int with the decimal moved right 2 places

float temp;
float miliVolts;
float pH;

unsigned long lastCalc;
uint8_t calcInterval = 30; // interval at which to calculate the pH

// Our parameter, for ease of use and eeprom access lets use a struct
struct parameters_T {
    unsigned int WriteCheck;
    int pH7Cal, pH4Cal, pH10Cal, statusColor;
    bool statusPulse;
    float pHStep;
} params;

void resetParams(void) {
  // check if we're not already defaulted so a runaway I2C call to reset doesn't burn up our eeprom
  //if (params.WriteCheck != Write_Check || !params.statusPulse || params.statusColor != GREEN_LED || params.pH7Cal != 2048
   //   || params.pH4Cal != 1286 || params.pH10Cal != 2810 || params.pHStep != 59.16) {
    //Restore to default set of parameters!
    params.WriteCheck = Write_Check;
    params.statusPulse = true;
    params.statusColor = GREEN_LED;
    params.pH7Cal = 2048; //assume ideal probe and amp conditions 1/2 of 4096
    params.pH4Cal = 1286; //using ideal probe slope we end up this many 12bit units away on the 4 scale
    params.pH10Cal = 2810; //using ideal probe slope we end up this many 12bit units away on the 10 scale
    params.pHStep = 59.16; //ideal probe slope
    eeprom_write_block(&params, (void *) 0, sizeof(params)); //write these settings back to eeprom
  //}
}

void updDelay() {
  // On-time for each display is total time spent per digit times the duty cycle. The
  // off-time is the rest of the cycle for the given display.

  long int temp = _digitDelay;    // Stored into long int since temporary variable gets larger than 32767
  temp *= _dutyCycle;     // Multiplication in this way to prevent multiplying two "shorter" ints.
  temp /= 100;        // Division after multiplication to minimize round-off errors.
  _digitOnDelay = temp;
  _digitOffDelay = _digitDelay - _digitOnDelay;

  cli();
  _timerCounterOnEnd=(_digitOnDelay/16)-1;
  _timerCounterOffEnd=(_digitOffDelay/16)-1;
  if(_digitOnDelay==0) _timerCounterOnEnd=0;
  if(_digitOffDelay==0) _timerCounterOffEnd=0;
  _timerCounter=0;
  sei();
}

void setDutyCycle(int dc){
  _dutyCycle=dc;
  updDelay();
}

void setDigitDelay(long int delay){
  _digitDelay=delay;
  updDelay();
}

void setRefreshRate(int freq) {
  long int period = 1000000L / freq;
  long int digitDelay = period/1;
  setDigitDelay(digitDelay);
}

void setupADC(uint8_t channel, int frequency) {
  cli();
  ADMUX = channel | _BV(REFS0);
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADATE) | _BV(ADIE);
  ADCSRB |= _BV(ADTS2) | _BV(ADTS0);  //Compare Match B on counter 1
  TCCR1A = _BV(COM1B1);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
  uint32_t clock = 250000;
  uint16_t counts = clock / (BUFFER_SIZE * frequency);
  OCR1B = counts;
  TIMSK1 = _BV(OCIE1B);
  ADCSRA |= _BV(ADEN) | _BV(ADSC);
  sei();
}

void setupFadeTimer()
{
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 3;
  TCCR3B = ((1 << WGM32)|(1 << CS31));
  TIMSK3 = (1 << OCIE3A);
  sei();
  updDelay();
  _timerCounter = 0;
}

ISR(ADC_vect) {
  result[i] = ADC;
  i = ++i & (BUFFER_SIZE - 1);

  for (uint8_t j = 0; j < BUFFER_SIZE; j++) {
    sum += result[j];
  }

  if (i == 0) {
    pHRaw = sum >> 2;
  }

  sum = 0;
  TCNT1 = 0;
}

void calcpH() {
  if (millis() - lastCalc > calcInterval || intPh == 0) {
    lastCalc = millis();
    miliVolts = (((float) pHSmooth / 4096) * 5) * 1000;
    temp = ((((5 * (float) params.pH7Cal) / 4096) * 1000) - miliVolts) / 5.25; //5.25 is the gain of our amp stage we need to remove it
    pH = 7 - (temp / params.pHStep);
    intPh = pH * 100;
  }
}

void requestEvent() {
  Wire.write(i2cResponse, i2cResponseLen);
  requestedCmd = 0;
}

// function that executes when master sends data (begin-end transmission)
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  if (Wire.available()) {
    requestedCmd = Wire.read(); // receive first byte - command assumed
  }
}

void smoothPh() {
  //Our smoothing portion
  //subtract the last pass
  total = total - passes[passIndex];

  //grab our pHRaw this should pretty much always be updated due to our Oversample ISR
  //and place it in our passes array this mimics an analogRead on a pin
  passes[passIndex] = pHRaw;
  total = total + passes[passIndex];
  passIndex = passIndex + 1;

  //Now handle end of array and make our rolling average portion
  if (passIndex >= NUM_PASSES) {
    passIndex = 0;
  }

  pHSmooth = total / NUM_PASSES;
}

void ledPulse() {
  _timerCounter++;

  // Finished with on-part. Turn off digit, and switch to the off-phase (_timerPhase=0)
  if ((_timerCounter >= _timerCounterOnEnd) && (_timerPhase == 1)) {
    _timerCounter = 0;
    _timerPhase = 0;
    digitalWrite(params.statusColor, HIGH);
  }

  // Finished with the off-part. Switch to next digit and turn it on.
  if ((_timerCounter >= _timerCounterOffEnd) && (_timerPhase == 0)) {
    _timerCounter = 0;
    _timerPhase = 1;
    digitalWrite(params.statusColor, LOW);
  }

  if (millis() - lastBlink > fadeInterval) {
    lastBlink = millis();
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;
    setDutyCycle(brightness);

    if (brightness == 0 || brightness == 100)
    {
      fadeAmount = -fadeAmount;
   }
  }
}

void getPhResponse() {
  requestedCmd = 0;
  i2cResponseLen = 0;
  calcpH();
  i2cResponse[i2cResponseLen++] = intPh >> 8;
  i2cResponse[i2cResponseLen++] = intPh & 0xff;
}

void setPH4Response() {
  requestedCmd = 255;
  i2cResponseLen = 0;
  if (params.pH4Cal != pHSmooth) {
    params.pH4Cal = pHSmooth;
    //RefVoltage * our deltaRawpH / 12bit steps *mV in V / OP-Amp gain /pH step difference 7-4
    params.pHStep = ((((5 * (float) (params.pH7Cal - params.pH4Cal)) / 4096) * 1000) / 5.25) / 3;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = params.pH4Cal >> 8;
  i2cResponse[i2cResponseLen++] = params.pH4Cal & 0xff;
  Serial.print("ph4 calibrated to ");
  Serial.println(params.pH4Cal);
}

void setPH7Response() {
  requestedCmd = 255;
  i2cResponseLen = 0;
  if (params.pH7Cal != pHSmooth) {
    params.pH7Cal = pHSmooth;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = params.pH7Cal >> 8;
  i2cResponse[i2cResponseLen++] = params.pH7Cal & 0xff;
  Serial.print("pH7 calibrated to ");
  Serial.println(params.pH7Cal);
}

void setPH10Response() {
  requestedCmd = 255;
  i2cResponseLen = 0;
  if (params.pH10Cal != pHSmooth) {
    params.pH10Cal = pHSmooth;
    //RefVoltage * our deltaRawpH / 12bit steps *mV in V / OP-Amp gain /pH step difference 10-7
    params.pHStep = ((((5 * (float) (params.pH10Cal - params.pH7Cal)) / 4096) * 1000) / 5.25) / 3;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = params.pH10Cal >> 8;
  i2cResponse[i2cResponseLen++] = params.pH10Cal & 0xff;
  Serial.print("pH10 calibrated to ");
  Serial.println(params.pH10Cal);
}

void setDefaultResponse() {
  requestedCmd = 255;
  i2cResponseLen = 0;
  Serial.println("Reseting to default settings");
  resetParams();
  i2cResponse[i2cResponseLen++] = true >> 8;
}

void setpulseOnResponse()
{
  requestedCmd = 255;
  i2cResponseLen = 0;
  if(!params.statusPulse)
  {
    Serial.println("Turning status pulse on");
    params.statusPulse = true;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = true >> 8;
}

void setpulseOffResponse()
{
  requestedCmd = 255;
  i2cResponseLen = 0;
  if(params.statusPulse)
  {
    Serial.println("Turning status pulse off");
    params.statusPulse = false;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = true >> 8;
}

void setGreenPulseResponse()
{
  requestedCmd = 255;
  i2cResponseLen = 0;
  if(params.statusColor != GREEN_LED)
  {
    Serial.println("Setting status led to GREEN");
    params.statusColor = GREEN_LED;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = true >> 8;
}

void setBluePulseResponse()
{
  requestedCmd = 255;
  i2cResponseLen = 0;
  if(params.statusColor != BLUE_LED)
  {
    Serial.println("Setting status led to BLUE");
    params.statusColor = BLUE_LED;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = true >> 8;
}

void setRedPulseResponse()
{
  requestedCmd = 255;
  i2cResponseLen = 0;
  if(params.statusColor != RED_LED)
  {
    Serial.println("Setting status led to RED");
    params.statusColor = RED_LED;
    eeprom_write_block(&params, (void *) 0, sizeof(params));
  }
  i2cResponse[i2cResponseLen++] = true >> 8;
}

void setI2cResponse() {
  if (requestedCmd > 0 && requestedCmd < 255) {
    switch (requestedCmd) {

      case I2C_CMD_GET_PH:
        getPhResponse();
        break;

      case I2C_CMD_SET_PH4:
        setPH4Response();
        break;

      case I2C_CMD_SET_PH7:
        setPH7Response();
        break;

      case I2C_CMD_SET_PH10:
        setPH10Response();
        break;

      case I2C_CMD_SET_DEFAULT:
        setDefaultResponse();
        break;

      case I2C_CMD_SET_PULSE_ON:
        setpulseOnResponse();
        break;

      case I2C_CMD_SET_PULSE_OFF:
        setpulseOffResponse();
        break;

      case I2C_CMD_SET_GREEN_PULSE:
        setGreenPulseResponse();
        break;

      case I2C_CMD_SET_BLUE_PULSE:
        setBluePulseResponse();
        break;

      case I2C_CMD_SET_RED_PULSE:
        setRedPulseResponse();
        break;

      default:
        requestedCmd = 0;
    }
  }
  // just set the response with the pH value
  else if(requestedCmd != 255){
    getPhResponse();
  }
}

void setup() {
  _digitDelay = 0;
  _digitOnDelay = 0;
  _digitOffDelay = 0;
  _dutyCycle = 100;

  _timerPhase = 1;
  _timerCounterOnEnd = 0;
  _timerCounterOffEnd = 0;

  setupADC(0, 100);
  setupFadeTimer();

  setRefreshRate(150);

  //eeprom_read_block(&params, (void *) 0, sizeof(params));
  if (params.WriteCheck != Write_Check) {
    resetParams();
  }
  Serial.begin(57600);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  // >> starting i2c
  TWBR = ((CPU_FREQ / TWI_FREQ_SETTING) - 16) / 2;
  Wire.begin(I2C_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // initialize smoothing variables to 0:
  for (uint8_t thisPass = 0; thisPass < NUM_PASSES; thisPass++) {
    passes[thisPass] = 0;
  }
}

void loop() {
  smoothPh();
  setI2cResponse();
}

ISR(TIMER3_COMPA_vect)
{
  ledPulse();
}

ISR(TIMER1_COMPB_vect) {

}

