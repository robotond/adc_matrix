/*
 * tempcalc.h
 *
 *  Created on: Aug 6, 2023
 *      Author: tjp
 */

#ifndef INC_TEMPCALC_H_
#define INC_TEMPCALC_H_

#include <math.h>
#define DIVIDER_RESISTANCE 2400     // voltage divider resistance (ohms)
#define NOMINAL_RESISTANCE 10000    // NTC nominal resistance (ohms)
#define VREF 3.072                   // reference voltage
#define NOMINAL_TEMPERATURE 298.15  // °K
#define BETA 3450                   // °K (Beta25/85)
#define DIODE_FV 0.286				//Serial diode forward voltage drop

/////////////////////////////////////////////
// gets °C data from raw adc data
int16_t get_temperature_data(uint16_t value){

    double voltage = VREF / (double)4096 * value;
 //   double ntc_resistance = voltage / (VREF-DIODE_FV - voltage) * DIVIDER_RESISTANCE;
//ezzel nem ok    double ntc_resistance =(VREF-voltage)/((voltage-DIODE_FV)/DIVIDER_RESISTANCE)-DIVIDER_RESISTANCE;
//majdnem jó    double ntc_resistance = 1.0 / ((voltage -DIODE_FV)/ (VREF-DIODE_FV- voltage) / DIVIDER_RESISTANCE);
    double ntc_resistance = 1.0 / ((voltage )/ (VREF- voltage) / DIVIDER_RESISTANCE);
    double temperature = (double)ntc_resistance / (double)NOMINAL_RESISTANCE;
    temperature = log(temperature);
    temperature /= BETA;
    temperature += 1.0 / NOMINAL_TEMPERATURE;
    temperature = 1.0 / temperature;
    temperature -= 273.15;
    return (int16_t)round(temperature);
}



#endif /* INC_TEMPCALC_H_ */
