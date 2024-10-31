/*
    library for the h2nose project
*/
#include "Arduino.h"
#include "Nose3.h"

#define DHTLIB_DHT_WAKEUP 1
#define DHTLIB_DHT_LEADING_ZEROS 6
#ifndef F_CPU
#define DHTLIB_TIMEOUT 1000  // should be approx. clock/40000
#else
#define DHTLIB_TIMEOUT (F_CPU/40000)
#endif

Nose::Nose(int pin1, int pin2, int pin3, bool isPPB, float b, float m, float ratioInCleanAir, bool isMG811, String gasType, float rl, bool comm) //standard mode
{ 
    _pin1 = pin1; //1st sensor 
    _pin2 = pin2; //2nd sensor 
    _pin3 = pin3; //3rd sensor
    _isPPB = isPPB; //for MQ3 
    _b = b; //V400 in MG811 
    _m = m; //V4000 in MG811 
    _RSR0 = ratioInCleanAir; //Clean air to R0 ratio 
    _isMG811 = isMG811; //sets sensor mode
    _gasType = gasType; //H2, CH4, C3H8, etc. 
    _RL = rl; //load resistance 
    _com = comm; //separate print with comma 
}

Nose::Nose(int pin[3], float v[2]) //MG811 mode
{
    _pin1 = pin[0]; //declare pins
    _pin2 = pin[1]; //declare pins
    _pin3 = pin[2]; //declare pins
    _b = v[0]; //V400
    _m = v[1]; //V4000
    _isMG811 = true; //set mode to MG811 mode
    _gasType = "MG811_CO2"; //identifier
}

Nose::Nose(int pin[3], int gastype, bool comm) //simple mode
{ 
    _pin1 = pin[0]; //declare pins
    _pin2 = pin[1]; //declare pins
    _pin3 = pin[2]; //declare pins
    _RL = 5;
    _isPPB = false; //for later
    _isMG811 = false; //set mode to MQ mode
    switch (gastype) //switch between the different sensors
    { 
    case 0: 
        _m = m2_h2; 
        _b = b2_h2; 
        _RSR0 = rsr02; 
        _gasType = "MQ2_H2"; 
        _RL = 5;
        break; 
    case 1: 
        _m = m4_h2; 
        _b = b4_h2; 
        _RSR0 = rsr04; 
        _gasType = "MQ4_H2"; 
        _RL = 20;
        break;
    case 2: 
        _m = m6_h2; 
        _b = b6_h2; 
        _RSR0 = rsr06; 
        _gasType = "MQ6_H2"; 
        _RL = 20;
        break; 
    case 3: 
        _m = m7_h2; 
        _b = b7_h2; 
        _RSR0 = rsr08; 
        _gasType = "MQ7_H2"; 
        _RL = 10;
        break; 
    case 4: 
        _m = m8_h2; 
        _b = b8_h2; 
        _RSR0 = rsr08; 
        _gasType = "MQ8_H2"; 
        _RL = 10;
        break; 
    case 5: 
        _m = m2_co; 
        _b = b2_co; 
        _RSR0 = rsr02; 
        _gasType = "MQ2_CO"; 
        _RL = 5;
        break;     
    case 6: 
        _m = m7_co; 
        _b = b7_co; 
        _RSR0 = rsr07; 
        _gasType = "MQ7_CO"; 
        _RL = 10;
        break;     
    case 7: 
        _m = m2_ch4; 
        _b = b2_ch4; 
        _RSR0 = rsr02; 
        _gasType = "MQ2_CH4"; 
        _RL = 5;
        break;     
    case 8: 
        _m = m4_ch4; 
        _b = b4_ch4; 
        _RSR0 = rsr04; 
        _gasType = "MQ4_CH4"; 
        _RL = 20;
        break; 
    case 9: 
        _m = m9_ch4; 
        _b = b9_ch4; 
        _RSR0 = rsr09; 
        _gasType = "MQ9_CH4"; 
        _RL = 10;
        break; 
    case 10: 
        _m = m3_benzene; 
        _b = b3_benzene; 
        _RSR0 = rsr03; 
        _gasType = "MQ3_C6H6"; 
        _RL = 200;
        break; 
    case 11: 
        _m = m2_propane; 
        _b = b2_propane; 
        _RSR0 = rsr02; 
        _gasType = "MQ2_C3H8"; 
        _RL = 5;
        break; 
    case 12: 
        _m = m3_alcohol; 
        _b = b3_alcohol; 
        _RSR0 = rsr03; 
        _gasType = "MQ3_OH"; 
        _RL = 200;
        break; 
    case 13: 
        _m = m2_lpg; 
        _b = b2_lpg; 
        _RSR0 = rsr02; 
        _gasType = "MQ2_LPG"; 
        _RL = 5;
        break; 
    case 14:
        _m = m214_ch4;
        _b = b214_ch4;
        _RSR0 = rsr0214;
        _gasType = "MQ214_CH4";
        _RL = 1;
        break;
    case 15: 
        _b = _R01; 
        _m = _R02; 
        _RSR0 = 0; 
        _isMG811 = true; 
        _gasType = "MG811_CO2"; 
        break;
    default: 
        break; 
    } 
    _com = comm; //add comma in front of the printout or no
}

void Nose::setRatioInCleanAir(float x)
{
    _RSR0 = x;
}

void Nose::setPin(int x[3])
{
    _pin1 = x[0];
    _pin2 = x[1];
    _pin3 = x[2];
}

void Nose::setPin1(int x)
{
    _pin1 = x;
}

void Nose::setPin2(int x)
{
    _pin2 = x;
}

void Nose::setPin3(int x)
{
    _pin3 = x;
}

void Nose::setR0(float x[3])
{
    _R01 = x[0];
    _R02 = x[1];
    _R03 = x[2];
    _R0A = (_R01 + _R02 + _R03)/3;
}

void Nose::setRL(float x)
{
    _RL = x;
}

void Nose::setB(float x)
{
    _b = x;
}

void Nose::setM(float x)
{
    _m = x;
}

void Nose::start(bool prt)
{
    float RS_air1;
    float RS_air2;
    float RS_air3;
    getVoltage();
    RS_air1 = ((5.0*_RL)/_volt1)-_RL;
    _R01 = RS_air1/_RSR0;
    RS_air2 = ((5.0*_RL)/_volt2)-_RL;
    _R02 = RS_air2/_RSR0;
    RS_air3 = ((5.0*_RL)/_volt3)-_RL;
    _R03 = RS_air3/_RSR0;
    _R0A = (_R01 + _R02 + _R03) / 3;
    if(prt)
    {
        Serial.println(_gasType+" R0 1 :"+String(_R01)+" | "+String(_volt1)+" | "+String(_readout1));
        Serial.println(_gasType+" R0 2 :"+String(_R02)+" | "+String(_volt2)+" | "+String(_readout2));
        Serial.println(_gasType+" R0 3 :"+String(_R03)+" | "+String(_volt3)+" | "+String(_readout3));
        Serial.println(_gasType+" R0 AVG :"+String(_R0A)+" | "+String(_volt)+" | "+String(_readout));
        Serial.println("------------------------------------------------------------");
    }
}

float Nose::getRL()
{
    return _RL;
}

float Nose::getR0(float outs[3])
{
    outs[0] = _R01;
    outs[1] = _R02;
    outs[2] = _R03;
    return _R0A;
}

float Nose::getB()
{
    return _b;
}

float Nose::getM()
{
    return _m;
}

float Nose::getVoltage()
{
    _readout1 = analogRead(_pin1);
    _volt1 = (_readout1 * (5.0/1023.0));
    _readout2 = analogRead(_pin2);
    _volt2 = (_readout2 * (5.0/1023.0));
    _readout3 = analogRead(_pin3);
    _volt3 = (_readout3 * (5.0/1023.0));
    _volt = (_volt1 + _volt2 + _volt3) / 3;
    _readout = (_readout1 + _readout2 + _readout3) / 3;
    return _volt;
}

float Nose::getOutput(bool inject, float volt, float r0)
{
    _volt = getVoltage();
    if (!inject){
        if(_isMG811){
            _buffer = 0;
            _buffer = (_b - _m)/(log10(400) - log10(40000)); // Delta V
            _buffer = (_volt - _b)/_buffer; 
            _buffer += log10(400);
            _buffer_final = pow(10, _buffer);
            return _buffer_final; //return in ppm
        } else {
            _RS_gas = ((5.0*_RL)/_volt)-_RL; //Get value of RS in a gas
            _ratio = _RS_gas/_R0A;  // Get ratio RS_gas/RS_air
            _ppm_log = (log10(_ratio)-_b)/_m; //Get ppm value in linear scale according to the the ratio value  
            _ppm = pow(10, _ppm_log); //Convert ppm value to log scale 
            if (_isPPB){
                _ppb = _ppm*1000;
                return _ppb; //return conversion value (parts per billion)
            } else {
                return _ppm; //return conversion value (in parts per million)
            }
        }
    } else {
        if(_isMG811){
            _buffer = 0;
            _buffer = (_b - _m)/(log10(400) - log10(40000)); // Delta V
            _buffer = (volt - _b)/_buffer; 
            _buffer += log10(400);
            _buffer_final = pow(10, _buffer);
            return _buffer_final; //return in ppm
        } else {
            _RS_gas = ((5.0*_RL)/volt)-_RL; //Get value of RS in a gas
            _ratio = _RS_gas/r0;  // Get ratio RS_gas/RS_air
            _ppm_log = (log10(_ratio)-_b)/_m; //Get ppm value in linear scale according to the the ratio value  
            _ppm = pow(10, _ppm_log); //Convert ppm value to log scale 
            if (_isPPB){
                _ppb = _ppm*1000;
                return _ppb; //return conversion value (parts per billion)
            } else {
                return _ppm; //return conversion value (in parts per million)
            }
        }
    }
}

void Nose::setPPM(float x, float y, float z)
{
    _ppm1 = x;
    _ppm2 = y;
    _ppm3 = z;
}

void Nose::setPPM(float x)
{
    _ppm = x;
}

void Nose::printOutput()
{
    getOutput();
    if(_isMG811){
        Serial.print(",\""+_gasType+"\""+":"+_buffer_final);
    } else {
        if (_isPPB){
            Serial.print("\""+_gasType+"\""+":"+_ppb);
        } else {
            if (_com){
                Serial.print(",\""+_gasType+"\""+":"+_ppm); 
            } else {
                Serial.print("\""+_gasType+"\""+":"+_ppm);   
            }
        }
    }
}

void Nose::printOutputAll(bool inject)
{
    getVoltage();
    if(!inject){
        _ppm1 = getOutput(true, _volt1, _R01);
        _ppm2 = getOutput(true, _volt2, _R02);
        _ppm3 = getOutput(true, _volt3, _R03);
    }
    if(_isMG811){
        Serial.print(",\""+_gasType+"_1"+"\""+":"+_ppm1+",\""+_gasType+"_2"+"\""+":"+_ppm2+",\""+_gasType+"_3"+"\""+":"+_ppm3);
    } else {
        if (_isPPB){
            Serial.print(",\""+_gasType+"_1"+"\""+":"+_ppm1+",\""+_gasType+"_2"+"\""+":"+_ppm2+",\""+_gasType+"_3"+"\""+":"+_ppm3);
        } else {
            if (!_com){
                Serial.print(",\""+_gasType+"_1"+"\""+":"+_ppm1+",\""+_gasType+"_2"+"\""+":"+_ppm2+",\""+_gasType+"_3"+"\""+":"+_ppm3); 
            } else {
                Serial.print("\""+_gasType+"_1"+"\""+":"+_ppm1+",\""+_gasType+"_2"+"\""+":"+_ppm2+",\""+_gasType+"_3"+"\""+":"+_ppm3);   
            }
        }
    }
}

void Nose::returnToArray(float outs[3])
{
    outs[0] = getOutput(true, _volt1, _R01);
    outs[1] = getOutput(true, _volt2, _R02);
    outs[2] = getOutput(true, _volt3, _R03);
}

float Nose::calibrate()
{
    _readout = (analogRead(_pin1) + analogRead(_pin2) + analogRead(_pin3)) / 3; //Read analog values of sensors
    _volt = _readout*(5.0/1023); //Convert to voltage
    float RS_air = ((5.0*_RL)/_volt)-_RL; //Get value of RS in a gas
    float _R0A = RS_air/_RSR0;
    return _R0A;
}

void Nose::addUpAll()
{
    _total += getOutput();
    _total1 += getOutput(true, _volt1, _R01);
    _total2 += getOutput(true, _volt2, _R02);
    _total3 += getOutput(true, _volt3, _R03);
}

void Nose::averageOut(int dataCount)
{
    _total /= dataCount;
    _total1 /= dataCount;
    _total2 /= dataCount;
    _total3 /= dataCount;
    setPPM(_total1, _total2, _total3);
    printOutputAll(true);
    _total = 0.0;
    _total1 = 0.0;
    _total2 = 0.0;
    _total3 = 0.0;
}

Thermocouple::Thermocouple(int8_t SCLK, int8_t CS, int8_t MISO, String identifier)
{
    _sclk = SCLK;
    _cs = CS;
    _miso = MISO;
    _identifier = identifier;

    // define pin modes
    pinMode(_cs, OUTPUT);
    pinMode(_sclk, OUTPUT);
    pinMode(_miso, INPUT);

    digitalWrite(_cs, HIGH);
}

float Thermocouple::readTemps(void)
{
    uint16_t v;

    digitalWrite(_cs, LOW);
    delayMicroseconds(10);

    v = spiRead();
    v <<= 8;
    v |= spiRead();

    digitalWrite(_cs, HIGH);

    if (v & 0x4) {
        return 33.5;
    }

    v >>= 3;
    if(v == 0){
        v = 134.0;
    }
    return v * 0.25;
}

byte Thermocouple::spiRead(void)
{
    int i;
    byte d = 0;

    for (i =7; i >= 0; i--){
        digitalWrite(_sclk, LOW);
        delayMicroseconds(10);
        if (digitalRead(_miso)){
            d |= (1 << i);
        }
        digitalWrite(_sclk, HIGH);
        delayMicroseconds(10);
    }
    return d;
}

void Thermocouple::enable(bool status)
{
    _cs = status;
}

void Thermocouple::printTemps()
{
    float t = readTemps();
    if (t == NAN){
        t = 31.5;
    }
    Serial.print(",\"Thermocouple_"+_identifier+"\":"+t);
}

DHT22::DHT22(uint8_t pin, bool disableIRQ)
{
    _pin = pin;
    _disableIRQ = disableIRQ;
}

int8_t DHT22::read()
{
    // READ VALUES
    if (_disableIRQ) noInterrupts();
    int8_t result = _readSensor(_pin, DHTLIB_DHT_WAKEUP, DHTLIB_DHT_LEADING_ZEROS);
    if (_disableIRQ) interrupts();

    // these bits are always zero, masking them reduces errors.
    bits[0] &= 0x03;
    bits[2] &= 0x83;

    // CONVERT AND STORE
    humidity  = (bits[0] * 256 + bits[1]) * 0.1;
    int16_t t = ((bits[2] & 0x7F) * 256 + bits[3]);
    if (t == 0)
    {
      temperature = 0.0;     // prevent -0.0;
    }
    else
    {
      temperature = t * 0.1;
      if((bits[2] & 0x80) == 0x80 )
      {
        temperature = -temperature;
      }
    }

    // TEST CHECKSUM
    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if (bits[4] != sum)
    {
        return DHTLIB_ERROR_CHECKSUM;
    }
    return result;
}

int8_t DHT22::_readSensor(uint8_t pin, uint8_t wakeupDelay, uint8_t leadingZeroBits)
{
    // INIT BUFFERVAR TO RECEIVE DATA
    uint8_t mask = 128;
    uint8_t idx = 0;

    uint8_t data = 0;
    uint8_t state = LOW;
    uint8_t pstate = LOW;
    uint16_t zeroLoop = DHTLIB_TIMEOUT;
    uint16_t delta = 0;

    leadingZeroBits = 40 - leadingZeroBits; // reverse counting...

    // replace digitalRead() with Direct Port Reads.
    // reduces footprint ~100 bytes => portability issue?
    // direct port read is about 3x faster
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *PIR = portInputRegister(port);

    // REQUEST SAMPLE
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // T-be
    if (wakeupDelay > 8) delay(wakeupDelay);
    else delayMicroseconds(wakeupDelay * 1000UL);
    digitalWrite(pin, HIGH); // T-go
    pinMode(pin, INPUT);

    uint16_t loopCount = DHTLIB_TIMEOUT * 2;  // 200uSec max
    // while(digitalRead(pin) == HIGH)
    while ((*PIR & bit) != LOW )
    {
        if (--loopCount == 0) 
        {
          return DHTLIB_ERROR_CONNECT;
        }
    }

    // GET ACKNOWLEDGE or TIMEOUT
    loopCount = DHTLIB_TIMEOUT;
    // while(digitalRead(pin) == LOW)
    while ((*PIR & bit) == LOW )  // T-rel
    {
        if (--loopCount == 0) 
        {
          return DHTLIB_ERROR_ACK_L;
        }
    }

    loopCount = DHTLIB_TIMEOUT;
    // while(digitalRead(pin) == HIGH)
    while ((*PIR & bit) != LOW )  // T-reh
    {
        if (--loopCount == 0)
        {
          return DHTLIB_ERROR_ACK_H;
        }
    }

    loopCount = DHTLIB_TIMEOUT;

    // READ THE OUTPUT - 40 BITS => 5 BYTES
    for (uint8_t i = 40; i != 0; )
    {
        // WAIT FOR FALLING EDGE
        state = (*PIR & bit);
        if (state == LOW && pstate != LOW)
        {
            if (i > leadingZeroBits) // DHT22 first 6 bits are all zero !!   DHT11 only 1
            {
                zeroLoop = min(zeroLoop, loopCount);
                delta = (DHTLIB_TIMEOUT - zeroLoop)/4;
            }
            else if ( loopCount <= (zeroLoop - delta) ) // long -> one
            {
                data |= mask;
            }
            mask >>= 1;
            if (mask == 0)   // next byte
            {
                mask = 128;
                bits[idx] = data;
                idx++;
                data = 0;
            }
            // next bit
            --i;

            // reset timeout flag
            loopCount = DHTLIB_TIMEOUT;
        }
        pstate = state;
        // Check timeout
        if (--loopCount == 0)
        {
          return DHTLIB_ERROR_TIMEOUT;
        }

    }
    // pinMode(pin, OUTPUT);
    // digitalWrite(pin, HIGH);
    return DHTLIB_OK;
}

void DHT22::printOutput()
{
    read();
    Serial.print(",\"DHT22_H\":"+String(humidity)+",\"DHT22_T\":"+String(temperature));
}