#include "DUE_ADC_Oversampler.h"


void ADC_OS_class::TIAO_setup(uint32_t counter)
{
	pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3+0) ;  // clock the TC0 channel 0

	TcChannel * tc0Ch0 = &(TC0->TC_CHANNEL)[TC0_CHA0] ;    // pointer to TC0 registers for its channel 0
	tc0Ch0->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
	tc0Ch0->TC_IDR = 0xFFFF ;     // disable interrupts
	tc0Ch0->TC_SR ;                   // read int status reg to clear pending
	tc0Ch0->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (prescale by 2, = 42MHz)
	        	TC_CMR_WAVE |                  // waveform mode
	        	TC_CMR_WAVSEL_UP_RC |          // count-up PWM using RC as threshold
	        	TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
	        	TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
	        	TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;

	tc0Ch0->TC_RC =  counter ;     // counter resets on RC, so sets period in terms of 42MHz clock---952 for 44.1kHz
	tc0Ch0->TC_RA =  counter/2 ;     // roughly square wave
	tc0Ch0->TC_CMR = ( (tc0Ch0->TC_CMR & ~(TC_CMR_LDRA(-1ul)) ) & (tc0Ch0->TC_CMR & ~(TC_CMR_LDRB(-1ul)) ) ) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares

	tc0Ch0->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.

	PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
	PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;   // disable PIO interrupts
	PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral

}

void ADC_OS_class::enableChX(uint8_t pin)
{ 
	pin = analogPinToChannelPos(pin);
	if( pin > 15 ) return;  //ignore input bigger than analog pins
	if( pin == INTERNAL_TEMP ) ADC->ADC_ACR |= ADC_ACR_TSON; //enable internal temp sensor
	ADC->ADC_CHER|=( 1 << pin );
};

const uint32_t ADC_OS_class::OS_TimerCounter(double sampleFrequency) const
{
	return VARIANT_MCK / 2 / ( sampleFrequency * OS_NumSamplesPerConversion );
}

const uint8_t ADC_OS_class::OS_GetNumSamplesPerConversion(ADC_OS_Resolution addedBits) const
{
	switch (addedBits) {
		case ADC_OS_12BITS: return pow(4, 0)-1;//-1 because 0 count
		case ADC_OS_13BITS: return pow(4, 1)-1;
		case ADC_OS_14BITS: return pow(4, 2)-1;
		case ADC_OS_15BITS: return pow(4, 3)-1;
		case ADC_OS_16BITS: return pow(4, 4)-1;
	}
	return 0;
}

void ADC_OS_class::ADC_init(uint32_t trigSel)
{
	pmc_enable_periph_clk(ID_ADC);
	ADC->ADC_CR = ADC_CR_SWRST; //reset the adc
	ADC->ADC_IDR = -1ul ;   // disable interrupts
	NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector

	ADC->ADC_CHDR = -1ul ;      // disable all channels*/
	ADC->ADC_MR = 0; //reset register
	ADC->ADC_MR |= ADC_MR_USEQ
				|  ADC_MR_TRGEN
				|  trigSel
				|  ADC_MR_TRACKTIM(ADC_MR_TRACKTIM_MAX)
				|  ADC_MR_SETTLING_AST3
				|  ADC_MR_STARTUP(ADC_STARTUP_FAST)
				|  ADC_MR_PRESCAL(ADC_MR_PRESCAL_MAX)
				|  ADC_MR_TRANSFER(ADC_MR_TRANSFER_MAX);
	ADC->ADC_SEQR1 = SEQR1_PIN_SEQUENCE;// use A0 to A7 in order into array 
	ADC->ADC_SEQR2 = SEQR2_PIN_SEQUENCE;//use A8 to A11, 20, 21, 52 and INTERNAL_TEMP
}

const uint8_t ADC_OS_class::numEnabledCh() const
{
	uint8_t Enabled = 0;
	for ( int i = 0; i <= 15; i++) {
		if ( 1<<i & ADC->ADC_CHSR ) Enabled++;
	}
	return Enabled;
}

void ADC_OS_class::prescalerAdjust(float tcCount)
{
	float prescaler = tcCount /numChannels /ADC_CLOCK_PER_CONVERSION -1;
	if (prescaler < 0) {
		Serial.println("SAMPLE FREQUENCY TO HIGH, ADC CLOCK CAN'T HANDLE IT !!! Reduce number of active channel or chose a lower frequency.");
		return;
	}
	prescaler = constrain(prescaler, 0, 255);
	ADC->ADC_MR &= ~(ADC_MR_PRESCAL(-1)); //reset prescaler
	ADC->ADC_MR |= ADC_MR_PRESCAL(static_cast<int>(prescaler));	//set prescaler
}

void ADC_OS_class::bufferConfig()
{
	//if buffer exist, delete it and reallocate it
	if ( dmaBuffer ) delete dmaBuffer;
	dmaBuffer = new uint16_t[numChannels]{};
	if ( dataSum ) delete dataSum;
	dataSum = new uint32_t[numChannels]{};
	if ( result ) delete result;
	result = new uint16_t[numChannels]{};
	count = 0;

	ADC->ADC_IER =  ADC_IDR_ENDRX;   // interrupt enable register, enables only ENDRX
 // following are the DMA controller registers for this peripheral
 // "receive buffer address" 
	ADC->ADC_RPR = reinterpret_cast<uint32_t>(dmaBuffer);   // DMA receive pointer register  points to beginning of dmaBuffer
	// "receive count" 
	ADC->ADC_RCR = numChannels;  //  receive counter set

//NOTE: RNPR seem useless here...	
	// "next-buffer address"
	ADC->ADC_RNPR = reinterpret_cast<uint32_t>(dmaBuffer);
	// and "next count"
	ADC->ADC_RNCR = numChannels;   //  and next counter is set
	// "transmit control register"
	ADC->ADC_PTCR = ADC_PTCR_RXTEN;  // transfer control register for the DMA is set to enable receiver channel requests
}

const int32_t ADC_OS_class::read(uint8_t pin)
{
	pin = analogPinToChannelPos(pin);
	if ( !(ADC->ADC_CHSR & (1<<pin)) ) return PIN_NOT_ENABLED; 
	return result[getArrayPos(pin)];
}

const uint8_t ADC_OS_class::getArrayPos( uint8_t pin ) const
{
  uint8_t pos = 0;
  for (uint8_t i=0; i<pin; i++){  //loop add a position in buffer for each active pin in ADC_CHSR
    if ( ADC->ADC_CHSR & (1<<i) ) pos++; 
  }
  return pos;
}

void ADC_OS_class::start()
{
	ADC->ADC_CR = ADC_CR_START;
}

void ADC_OS_class::ADC_Handler() {     // for the ATOD: re-initialize DMA pointers and count	
	//   read the interrupt status register 
	if (ADC->ADC_ISR & ADC_ISR_ENDRX); /// check the bit "endrx"  in the status register

	for ( uint16_t i = 0; i < numChannels; i++ ) { //add dma result into adder
		dataSum[i] += dmaBuffer[i];
	}
	
	ADC->ADC_RNCR = numChannels;  // "receive next" counter

	if ( count < ( OS_NumSamplesPerConversion ) ) { //return when oversampling not complete
		count++;
		return;
	}
	for ( uint16_t i = 0; i <= numChannels; i++ ) { //shift added data to result
		result[i] = dataSum[i] >> OS_Res;
		dataSum[i] = 0;
	}
	count = 0;	
}

const int8_t ADC_OS_class::analogPinToChannelPos(int8_t pin) const
{
	switch (pin) { //handle 0-11 and A0 to A11 pin, if invalid return -2
		case 0:;
		case 1:;
		case 2:;
		case 3:;
		case 4:;
		case 5:;
		case 6:;
		case 7: break;
		case 8: pin = 0xa; break;
		case 9:  pin = 0xb; break;
		case 10: pin = 0xc; break;
		case 11: pin = 0xd; break;
		case INTERNAL_TEMP: pin = INTERNAL_TEMP; break;
		case 20: pin = 8; break;
		case 21: pin = 9; break;		
		case 52: pin = PIN52_Pos; break;
		case A0:;
		case A1:;
		case A2:;
		case A3:;
		case A4:;
		case A5:;
		case A6:;
		case A7:;
		case A8:;
		case A9:;
		case A10:;
		case A11: pin = analogPinToChannelPos(pin -= A0); break;
		default: pin = -2;
		
	}
	return pin;
}


ADC_OS_class ADC_OS;

const float internalTemp()
{ // convert the sam3x internal sensor temp
	if ( !(ADC->ADC_CHSR & ADC_CHSR_CH15) ) return -265;  // return 0 if internal temps ch is disable
	float tVolt = static_cast<float>(ADC_OS.read(INTERNAL_TEMP)) * AREF_mV / ADC_MAX_VALUE(ADC_OS.OS_Res);
	return (tVolt-INTERNAL_MV_OFFSET) / INTERNAL_TEMP_MV_PER_C + INTERNAL_TEMP_OFFSET;
}

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

void ADC_Handler (void)
{
	ADC_OS.ADC_Handler();
}

#ifdef __cplusplus
}
#endif //__cplusplus