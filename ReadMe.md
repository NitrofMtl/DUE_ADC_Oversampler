# DUE_ADC_Oversampler

### Use the Sam3x8e analog PDC to oversample analog inputs up to 16 bits.

# Usage
- Import DUE_ADC_Oversampler from library manager
- Include header:
````
#include "DUE_ADC_Oversampler.h"
````
-The sample frequency that the Due can scan is divided by the number of anable channel and the oversampling level.
  Theoretically, the ADC could sample 1 channel at 4Mh.
  If you enable 4 channel, you must devide by 4.
  Oversample take many reading by conversion:</br>
  12 bits = 1 sample (no oversample)</br>
  13 bits = 4 samples</br>
  14 bits = 16 samples</br>
  15 bits = 64 samples</br>
  16 bits = 256 samples</br>

  So the bigger the oversampler, the slower the sampler will be.
  The programmer should enable only the needed pins.

  The available paramter for chosing resolution are:

````
ADC_OS_12BITS
ADC_OS_13BITS
ADC_OS_14BITS
ADC_OS_15BITS
ADC_OS_16BITS
````

- In setup, start the sampler with the frequency, the oversample bitrage and add the channel you want to enable:
````
void setup()
{
  Serial.begin (115200) ; 
  // ADC_OS.begin() Parameter:
  //  -(float)sample frequency in Hz,
  //  -oversampler resolution(12 to 16),
  //  - ... list of channel to enabled
  ADC_OS.begin(50, ADC_OS_16BITS);  //Enable all standar DUE analog pins (A0 to A11)

  //ADC_OS.begin(50, ADC_OS_16BITS, A0,A2,A3,A5,A6,A7,A9,A10,A11, 52);  //Enable selected pin

  //ADC_OS.begin(50, ADC_OS_16BITS, ANALOG_PINS_ALL); //Enable all 15 analog pins and internal temp channel
}
````

use ADC_OS.read(), like analogRead(), to get the latest converted value:</br>
If an disabled channel is read, it return -1
````
//Usual arduino DUE analog pins
  Serial.print("Pin  A0: "); Serial.println(ADC_OS.read(0));
  Serial.print("Pin  A1: "); Serial.println(ADC_OS.read(A1));
  Serial.print("Pin  A2: "); Serial.println(ADC_OS.read(A2));
  Serial.print("Pin  A3: "); Serial.println(ADC_OS.read(A3));
  Serial.print("Pin  A4: "); Serial.println(ADC_OS.read(A4));
  Serial.print("Pin  A5: "); Serial.println(ADC_OS.read(A5));
  Serial.print("Pin  A6: "); Serial.println(ADC_OS.read(A6));
  Serial.print("Pin  A7: "); Serial.println(ADC_OS.read(A7));
  Serial.print("Pin  A8: "); Serial.println(ADC_OS.read(A8));
  Serial.print("Pin  A9: "); Serial.println(ADC_OS.read(A9));
  Serial.print("Pin A10: "); Serial.println(ADC_OS.read(A10));
  Serial.print("Pin A11: "); Serial.println(ADC_OS.read(A11));
//Others analog pins
  Serial.print("Pin  20: "); Serial.println(ADC_OS.read(20)); 
  Serial.print("Pin  21: "); Serial.println(ADC_OS.read(21));
  Serial.print("Pin  52: "); Serial.println(ADC_OS.read(52));
//Internal temp sensor, converted in celcius  
  Serial.print("internal temp : "); Serial.print(internalTemp()); Serial.println("°C");
````