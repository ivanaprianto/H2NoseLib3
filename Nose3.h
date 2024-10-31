#ifndef Nose3_H
#define Nose3_H

#define MAXLENGTH 9

#define DHTLIB_OK                        0
#define DHTLIB_ERROR_CHECKSUM           -1
#define DHTLIB_ERROR_TIMEOUT            -2
#define DHTLIB_ERROR_CONNECT            -3
#define DHTLIB_ERROR_ACK_L              -4
#define DHTLIB_ERROR_ACK_H              -5

#include <Arduino.h>

class Nose
{
    public:
        Nose(int pin1, int pin2, int pin3, bool isPPB, float b, float m, float ratioInCleanAir, bool isMG811, String gasType, float rl, bool comm = false);
        Nose(int pin[3], float v[2]);
        Nose(int pin[3], int gastype, bool comm = false);

        //general functions
        void printOutput();
        void printOutputAll(bool inject = false);
        void setRatioInCleanAir(float x);
        void setPin(int x[3]);
        void setPin1(int x);
        void setPin2(int x);
        void setPin3(int x);
        void setRL(float x);
        void setR0(float x[3]);
        void setB(float x);
        void setM(float x);
        void setPPM(float x, float y, float z);
        void setPPM(float x);
        void addUpAll();
        void averageOut(int dataCount);
        void start(bool prt = false);
        void returnToArray(float outs[3]);

        //get main data
        float getOutput(bool inject = false, float volt = 0.0, float rl = 0.0);

        //other data
        float getRL();
        float getR0(float outs[3]);
        float getB();
        float getM();
        float getVoltage();

        //tools
        float calibrate();

    private:
        int _pin1;
        int _pin2;
        int _pin3;
        bool _isPPB;
        float _b;
        float _m;
        float _RSR0;
        float _R0A;
        float _R01;
        float _R02;
        float _R03;
        bool _isMG811;
        String _gasType;
        float _readout;
        float _readout1;
        float _readout2;
        float _readout3;
        float _buffer;
        float _buffer_final;
        float _volt;
        float _volt1;
        float _volt2;
        float _volt3;
        float _RS_gas;
        float _ratio;
        float _ppm_log;
        float _ppm;
        float _ppm1;
        float _ppm2;
        float _ppm3;
        float _ppb;
        float _RL;
        bool _com;
        float _total;
        float _total1;
        float _total2;
        float _total3;
        //MQ2-H2
        const double b2_h2 = 3.99626591;
        const double m2_h2 = -1.1690194965;
        //MQ7-CO
        const double b7_co = 1.4441096915;
        const double m7_co = -0.6705969873;
        //MQ4-CH4
        const double b4_ch4 = 1.1001300621;
        const double m4_ch4 = -0.3613714269;
        //MQ3-C6H6
        const double b3_benzene = 4.8387;
        const double m3_benzene = -2.68;
        //MQ3-OH
        const double b3_alcohol = 0.3934;
        const double m3_alcohol = -1.504;
        //MQ2-LPG
        const double m2_lpg = -0.4900080111;
        const double b2_lpg = 1.3688274357;
        // MQ2-C3H8
        const double m2_propane = -0.4756496415;
        const double b2_propane = 1.3504783634;
        // MQ4-H2 
        const double m4_h2 = -0.1528128737;
        const double b4_h2 = 0.88540934406;
        // MQ6-H2 
        const double m6_h2 = -0.2596918708;
        const double b6_h2 = 1.10129490713;
        // MQ7-H2 
        const double m7_h2 = -0.7106886988;
        const double b7_h2 = 1.3473178615;
        // MQ8-H2 
        const double m8_h2 = -1.4418066958;
        const double b8_h2 = 4.23409440355;
        // MQ2-CO
        const double m2_co = -0.3128241909;
        const double b2_co = 1.42738802266;
        // MQ2-CH4
        const double m2_ch4 = -0.3720037523;
        const double b2_ch4 = 1.33311304481;
        // MQ6-CH4
        const double m6_ch4 = -0.4017608236;
        const double b6_ch4 = 1.33943705422;
        // MQ9-CH4
        const double m9_ch4 = -0.3803855584;
        const double b9_ch4 = 1.36664027361;
        //MQ214-CH4
        const double m214_ch4 = -0.3692872167;
        const double b214_ch4 = 1.477148867;
        const float rsr02 = 9.83;
        const float rsr03 = 60.0;
        const float rsr04 = 4.4;
        const float rsr06 = 10.0;
        const float rsr07 = 27.5;
        const float rsr08 = 70.0;
        const float rsr09 = 9.6;
        const float rsr0214 = 7.5;
};

class Thermocouple
{
    public:
        Thermocouple(int8_t SCLK, int8_t CS, int8_t MISO, String identifier);
        void printTemps();
        void enable(bool status);
        float readTemps(void);

    private:
        int8_t _sclk, _cs, _miso;
        String _identifier;
        uint8_t spiRead(void);
};

class DHT22
{
    public:
        DHT22(uint8_t pin, bool disableIRQ = false);
        void printOutput();
        int8_t read();
        float humidity;
        float temperature;

    private:
        uint8_t bits[5];
        int8_t _readSensor(uint8_t pin, uint8_t wakeupDelay, uint8_t leadingZeroBits);
        bool   _disableIRQ;
        int    _pin;
};
#endif