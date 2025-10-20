/** Project CPP includes. */
#include "TLx493D_inc.hpp"
/** Define which sensors should be activated */
#define SENSOR_1_ACTIVE
//#define SENSOR_2_ACTIVE  
//#define SENSOR_3_ACTIVE
//#define SENSOR_4_ACTIVE 
#define numavg 50
using namespace ifx::tlx493d;

/** Definition of the power and chip select pins for four sensors. */
const uint8_t CHIP_SELECT_PIN_1 = 3;
const uint8_t CHIP_SELECT_PIN_2 = 4;
const uint8_t CHIP_SELECT_PIN_3 = 5;
const uint8_t CHIP_SELECT_PIN_4 = 6;

const uint8_t POWER_PIN = 4;

/** Declaration of the sensor objects. */
TLx493D_P3I8 sensor1(SPI);
TLx493D_P3I8 sensor2(SPI);
TLx493D_P3I8 sensor3(SPI);
TLx493D_P3I8 sensor4(SPI);




void average_sensor_value(uint8_t sensor, double *sumTemp, double *sumX, double *sumY, double *sumZ)
{
    double sumTemp1 = 0.0, sumTemp2 = 0.0, sumTemp3 = 0.0, sumTemp4 = 0.0;
    double sumX1 = 0, sumY1 = 0, sumZ1 = 0;
    double sumX2 = 0, sumY2 = 0, sumZ2 = 0;
    double sumX3 = 0, sumY3 = 0, sumZ3 = 0;
    double sumX4 = 0, sumY4 = 0, sumZ4 = 0;
    double temp, valX, valY, valZ;

    unsigned long startTime = millis(); // Start time

    for (int i = 0; i < numavg; ++i) {
        if(sensor == 1)
        {
        sensor1.getTemperature(&temp);
        sensor1.getMagneticField(&valX, &valY, &valZ);
        sumTemp1 += temp;
        sumX1 += valX;
        sumY1 += valY;
        sumZ1 += valZ;
        if(i == numavg-1)
        {
            *sumTemp = sumTemp1/numavg;
            *sumX = sumX1/numavg;
            *sumY = sumY1/numavg;
            *sumZ = sumZ1/numavg;
        }
        }
        else if(sensor == 2)
        {
        sensor2.getTemperature(&temp);
        sensor2.getMagneticField(&valX, &valY, &valZ);
        sumTemp2 += temp;
        sumX2 += valX;
        sumY2 += valY;
        sumZ2 += valZ;
        if (i == numavg-1)
        {
            *sumTemp = sumTemp2/numavg;
            *sumX = sumX2/numavg;
            *sumY = sumY2/numavg;
            *sumZ = sumZ2/numavg;
        }
        
        }

        else if(sensor == 3)
        {
        sensor3.getTemperature(&temp);
        sensor3.getMagneticField(&valX, &valY, &valZ);
        sumTemp3 += temp;
        sumX3 += valX;
        sumY3 += valY;
        sumZ3 += valZ;
        if (i == numavg-1)
        {
            *sumTemp = sumTemp3/numavg;
            *sumX = sumX3/numavg;
            *sumY = sumY3/numavg;
            *sumZ = sumZ3/numavg;
        }
        }
        else if(sensor == 4)
        {
        sensor4.getTemperature(&temp);
        sensor4.getMagneticField(&valX, &valY, &valZ);
        sumTemp4 += temp;
        sumX4 += valX;
        sumY4 += valY;
        sumZ4 += valZ;
        if (i == numavg-1)
        {
           *sumTemp = sumTemp4/numavg;
            *sumX = sumX4/numavg;
            *sumY = sumY4/numavg;
            *sumZ = sumZ4/numavg;
        }
        }
    }

    // End time
    unsigned long endTime = millis();
    unsigned long loopDuration = endTime - startTime;
    Serial.print("Loop duration: ");
    Serial.println(loopDuration);
}


void setup() {
    Serial.begin(115200);
    delay(3000);

    
#ifdef SENSOR_1_ACTIVE
    sensor1.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 1000, 250000);
    sensor1.setSelectPin(CHIP_SELECT_PIN_1, OUTPUT, INPUT, LOW, HIGH, 0, 0,0,5);
    sensor1.begin(true, true);
#endif

#ifdef SENSOR_2_ACTIVE
    // Setup for sensor 2
    sensor2.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 1000, 250000);
    sensor2.setSelectPin(CHIP_SELECT_PIN_2, OUTPUT, INPUT, LOW, HIGH, 0, 0,0,5);
    sensor2.begin(true, true);

#endif

#ifdef SENSOR_3_ACTIVE
    // Setup for sensor 3
    sensor3.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 1000, 250000);
    sensor3.setSelectPin(CHIP_SELECT_PIN_3, OUTPUT, INPUT, LOW, HIGH, 0, 0,0,5);
    sensor3.begin(true, true);
#endif

#ifdef SENSOR_4_ACTIVE
    // Setup for sensor 4
    sensor4.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 1000, 250000);
    sensor4.setSelectPin(CHIP_SELECT_PIN_4, OUTPUT, INPUT, LOW, HIGH, 0, 0,0,5);
    sensor4.begin(true, true);
#endif

    Serial.print("setup done.\n");
}

/** In the loop we're reading out the temperature value (in C) as well as the magnetic fields values in X, Y, Z-direction (in mT).
 *  We're also reading out the raw temperature value (in LSB).
 */
void loop() {
    double avg_temp=0; double avg_x=0; double avg_y=0; double avg_z=0;

    
#ifdef SENSOR_1_ACTIVE
    // Read and print data from sensor 1
    sensor1.printRegisters();
    average_sensor_value(1, &avg_temp, &avg_x, &avg_y, &avg_z);
     Serial.print("Sensor 1:");
    Serial.print("Sensor 1 - Temp: "); Serial.print(avg_temp); Serial.print(" C, ");
    Serial.print("Magnetic Field - X: "); Serial.print(avg_x); Serial.print(" mT, ");
    Serial.print("Y: "); Serial.print(avg_y); Serial.print(" mT, ");
    Serial.print("Z: "); Serial.println(avg_z); Serial.print(" mT\n");
#endif

#ifdef SENSOR_2_ACTIVE

    average_sensor_value(2, &avg_temp, &avg_x, &avg_y, &avg_z);
    Serial.print("Sensor 2:");
    Serial.print("Sensor 2 - Temp: "); Serial.print(avg_temp); Serial.print(" C, ");
    Serial.print("Magnetic Field - X: "); Serial.print(avg_x); Serial.print(" mT, ");
    Serial.print("Y: "); Serial.print(avg_y); Serial.print(" mT, ");
    Serial.print("Z: "); Serial.println(avg_z); Serial.print(" mT\n");
#endif

#ifdef SENSOR_3_ACTIVE

    average_sensor_value(3, &avg_temp, &avg_x, &avg_y, &avg_z);
    Serial.print("Sensor 3:");
    Serial.print("Sensor 3 - Temp: "); Serial.print(avg_temp); Serial.print(" C, ");
    Serial.print("Magnetic Field - X: "); Serial.print(avg_x); Serial.print(" mT, ");
    Serial.print("Y: "); Serial.print(avg_y); Serial.print(" mT, ");
    Serial.print("Z: "); Serial.println(avg_z); Serial.print(" mT\n");
#endif

#ifdef SENSOR_4_ACTIVE

    average_sensor_value(4, &avg_temp, &avg_x, &avg_y, &avg_z);
    Serial.print("Sensor 4:");
    Serial.print("Sensor 4 - Temp: "); Serial.print(avg_temp); Serial.print(" C, ");
    Serial.print("Magnetic Field - X: "); Serial.print(avg_x); Serial.print(" mT, ");
    Serial.print("Y: "); Serial.print(avg_y); Serial.print(" mT, ");
    Serial.print("Z: "); Serial.println(avg_z); Serial.print(" mT\n");
#endif

}