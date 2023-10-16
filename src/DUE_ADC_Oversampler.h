#ifndef DUE_ADC_Oversampler_h
#define DUE_ADC_Oversampler_h

#include "Arduino.h"

#if !defined(ARDUINO_ARCH_SAM)
#error "This libraries is for arduino DUE only !!!"
#endif

#define ADC_MR_STARTUP(startupTime)  ( ( startupTime << ADC_MR_STARTUP_Pos ) & ADC_MR_STARTUP_Msk )
#define TC_CMR_LDRA(TCCLKS) ( ( TCCLKS << TC_CMR_LDRA_Pos ) & TC_CMR_LDRA_Msk )
#define TC_CMR_LDRB(TCCLKS) ( ( TCCLKS << TC_CMR_LDRB_Pos ) & TC_CMR_LDRB_Msk )

#define ADC_MR_TRACKTIM_MAX 15
#define ADC_MR_PRESCAL_MAX  255
#define ADC_MR_TRANSFER_MAX 3

#define INTERNAL_TEMP 15
#define PIN52_Pos 14

#define ADC_CLOCK_PER_CONVERSION 21

#define SEQR1_PIN_SEQUENCE 0x01234567
#define SEQR2_PIN_SEQUENCE 0xfedcba98

enum ADC_OS_Resolution : uint8_t
{
	ADC_OS_12BITS,
	ADC_OS_13BITS,
	ADC_OS_14BITS,
	ADC_OS_15BITS,
	ADC_OS_16BITS
};

#define ANALOG_PINS_DEFAULT 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
#define ANALOG_PINS_ALL     0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 20, 21, 52, INTERNAL_TEMP
#define PIN_NOT_ENABLED    -1
//Internal temp
#define INTERNAL_TEMP_OFFSET 27
/* DATASHEET P"1410
VO_TS equals 0.8V at TA 27°C, with a ±15% accuracy.
The VO_TS slope versus temperature dVO_TS/dT = 2.65 mV/°C 
only shows a ±5% slight variation over process, mismatch and supply voltage.
The user needs to calibrate it (offset calibration) at ambient temperature to 
eliminate the VO_TS spread at ambient temperature (±15%).
*/

/***
Found Eyes ball that offset is better at 727. but may vary by devise....
***/
#define INTERNAL_MV_OFFSET 727
#define INTERNAL_TEMP_MV_PER_C 2.65f
#define AREF_mV 3300.

#define ADC_MAX_VALUE(RES) ( (1 << (12 + RES) )-1) 

/*
**********Timing descriptions:

Oversample number = ( 4^(num added bits) )
	ex: -12 bits oversample to 16 =  4^(16-12) = 256
		-12 bits oversample to 14 = 4^(14-12) = 16

Tc counter = MCK / 2 / ( sample frequency * Oversampler number )

ADC clock = MCK / ( (Prescaler+1) * 2 )

*** Prescaler must be set at a speed that can handle -> Sample frequency * oversample number:
Prescaler = 
(
	MCK / 2 / 
	(
		sample frequency
		* oversample number
		* number of ADC channels 
		* ADC_clock per conversion
	) 
	-1
 )

 Narrow down to: 
 Prescaler = Tc counter / number of ADC channels / ADC_clock per conversion -1
 **********
*/


class ADC_OS_class {
private:
	void TIAO_setup(uint32_t counter);
	const uint32_t OS_TimerCounter(double sampleFrequency) const;
	const uint8_t OS_GetNumSamplesPerConversion(ADC_OS_Resolution addedBits) const;
	void ADC_init(uint32_t trigSel); //OK
	const uint8_t numEnabledCh() const; //OK
	void prescalerAdjust(float tcCount);
	void bufferConfig();//OK
	const uint8_t getArrayPos( uint8_t pin ) const;
	const int8_t analogPinToChannelPos(int8_t pin) const;

	volatile uint16_t *dmaBuffer;
	volatile uint32_t *dataSum;
	volatile uint16_t *result;
	volatile uint16_t count;
	uint8_t OS_NumSamplesPerConversion;
	uint8_t numChannels;
	ADC_OS_Resolution OS_Res;
	
	void enableChX(uint8_t pin);
	template<typename Pin, typename ... PinX>
	void enableChX(Pin pin, PinX ... pinX) { enableChX((uint8_t)pin); enableChX((uint8_t)pinX...); };
	friend const float internalTemp();
public:
	void ADC_Handler();
	const int32_t read(uint8_t pin);
	void start();
	
	void begin(float sampleFrequency, ADC_OS_Resolution nOSBits, uint8_t pin)
	{
		OS_Res = nOSBits;
		OS_NumSamplesPerConversion = OS_GetNumSamplesPerConversion(OS_Res);
		TIAO_setup( OS_TimerCounter(sampleFrequency) );
		ADC_init(ADC_MR_TRGSEL_ADC_TRIG1);
		enableChX(pin);
		numChannels = numEnabledCh();
		bufferConfig();
		prescalerAdjust( OS_TimerCounter(sampleFrequency) );
		start();
	};

	template<typename ... PinX>
	void begin(float sampleFrequency, ADC_OS_Resolution nOSBits, PinX ... pinX)
	{
		OS_Res = nOSBits;
		OS_NumSamplesPerConversion = OS_GetNumSamplesPerConversion(OS_Res);
		TIAO_setup( OS_TimerCounter(sampleFrequency) );
		ADC_init(ADC_MR_TRGSEL_ADC_TRIG1);
		enableChX(pinX...);
		numChannels = numEnabledCh();
		bufferConfig();
		prescalerAdjust( OS_TimerCounter(sampleFrequency) );
		start();
	};
	void begin(float f, ADC_OS_Resolution OSBits)
	{
		begin(f, OSBits, ANALOG_PINS_DEFAULT);
	}
};

extern ADC_OS_class ADC_OS;

const float internalTemp();

#endif //DUE_ADC_Oversampler_h

