v1.0.0
-Initial commit
v1.0.1
-Add pins D20(ad8), D21(AD9) and D52(AD14) in analog pin map for full 16 analogs channels
v1.0.2
-On IRQ:
    -Fix bad comparaizon on end oversampling sum condition
    -Guard on ADC_ISR_ENDRX
    -Pointer semantic for better optimization
-faster OS_GetNumSamplesPerConversion()
