/*
 * tempcalc.h
 *
 *  Created on: Aug 6, 2023
 *      Author: tjp
 */

#ifndef INC_TEMPCALC_H_
#define INC_TEMPCALC_H_

#include <math.h>
#define DIVIDER_RESISTANCE 2200     // voltage divider resistance (ohms)
#define NOMINAL_RESISTANCE 10000    // NTC nominal resistance (ohms)
#define VREF 3.072                   // reference voltage
#define NOMINAL_TEMPERATURE 298.15  // °K
#define BETA 3450                   // °K (Beta25/85)
#define DIODE_FV 0.180				//Serial diode forward voltage drop

#define MAX_PROBES_NUM	20
#define MAX_SENSORS_NUM	16

static const uint16_t nominal_resistances[MAX_PROBES_NUM][MAX_SENSORS_NUM] = {
	{8914, 10128, 10279, 8876, 8897, 9277, 9164, 9709, 8843, 8802, 9105, 8937, 11494, 8796, 9503, 9148},
	{10911, 25910, 14549, 9752, 9219, 13539, 10602, 9280, 14207, 9252, 9176, 9342, 13479, 9186, 13086, 12515},
	{11371, 10135, 13459, 9473, 9535, 9235, 9665, 9836, 9445, 13805, 9182, 9439, 9286, 12798, 9300, 13893},
	{9584, 11414, 10807, 9401, 10639, 8969, 9105, 12343, 9453, 9018, 9055, 9734, 9075, 11701, 11177, 9109},
	{8930, 8980, 8964, 8933, 8924, 9435, 12408, 11357, 8985, 8899, 8993, 8888, 8831, 8930, 8876, 8965},
	{9516, 12924, 9829, 9871, 9822, 9527, 10666, 9710, 9584, 9698, 9641, 9967, 10770, 9632, 9571, 12389},
	{9499, 9878, 12896, 9580, 12128, 10203, 9652, 9715, 9696, 10986, 9442, 9539, 11442, 10343, 9356, 9433},
	{9389, 9420, 9358, 9432, 10205, 9202, 9391, 9352, 10467, 9238, 10139, 10157, 9632, 9239, 10932, 9480},
	{9604, 11860, 10254, 9363, 10383, 11587, 10037, 9981, 10009, 10417, 9369, 11558, 9792, 9239, 9383, 9275},
	{10417, 9757, 9676, 12908, 9724, 10088, 10770, 9808, 10261, 10400, 10218, 10173, 11025, 9787, 9569, 10957},
	{9498, 9503, 9454, 9525, 9576, 10681, 9599, 9534, 9378, 9465, 9379, 9447, 9458, 9324, 9407, 9498},
	{10128, 14646, 10177, 10020, 10315, 10092, 10228, 10234, 10123, 10130, 10158, 10073, 10130, 13673, 10096, 10095},
	{10320, 9949, 10899, 11080, 10018, 9883, 12539, 9998, 11682, 10896, 12405, 11334, 9896, 9821, 9926, 11035},
	{9290, 10406, 10934, 9217, 9415, 11002, 9242, 10381, 12794, 9232, 9020, 14778, 9969, 10363, 13127, 9203},
	{9492, 10222, 9580, 12142, 9772, 12478, 9982, 10685, 9596, 11104, 9537, 9875, 10106, 11113, 10617, 13153},
	{9870, 9908, 9845, 10747, 9892, 9852, 11108, 10256, 9923, 11324, 10356, 10888, 11761, 11670, 12750, 10600},
	{9290, 9405, 12854, 10156, 11828, 9554, 11849, 11006, 9623, 10917, 9863, 12353, 11559, 9662, 11093, 11122},
	{10135, 10244, 11360, 10134, 10138, 15886, 11629, 10240, 10193, 10293, 10249, 11576, 10171, 10157, 10046, 10072},
	{10158, 11620, 9861, 9844, 9808, 9820, 9882, 9913, 10488, 11068, 11328, 9702, 10875, 9942, 10142, 11823},
	{9504, 10239, 10156, 9703, 9462, 12403, 9980, 11106, 9409, 10970, 10880, 12597, 9398, 9350, 9478, 9846}
};

/////////////////////////////////////////////
// gets °C data from raw adc data
// probe_num is "1 indexed", sensor_num is "0 indexed"
int16_t get_temperature_data(uint16_t value, int probe_num, int sensor_num){

	uint16_t nominal_resistance = NOMINAL_RESISTANCE; // default

	if (probe_num > 0 && probe_num <= MAX_PROBES_NUM){
		if (sensor_num >= 0 && sensor_num < MAX_SENSORS_NUM){
			nominal_resistance = nominal_resistances[probe_num - 1][sensor_num];
		}
	}
    double voltage = VREF / (double)4096 * value;

 // original calculation
 //   double ntc_resistance = voltage / (VREF - voltage) * DIVIDER_RESISTANCE;
    double ntc_resistance = 1.0 / (voltage/ (VREF - voltage) / DIVIDER_RESISTANCE);
    double temperature = (double)ntc_resistance / (double)nominal_resistance;
    temperature = log(temperature);
    temperature /= BETA;
    temperature += 1.0 / NOMINAL_TEMPERATURE;
    temperature = 1.0 / temperature;
    temperature -= 273.15;
    return (int16_t)round(temperature);
}



#endif /* INC_TEMPCALC_H_ */
