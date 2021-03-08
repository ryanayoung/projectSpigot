# Dual Touchpad Example Code

Copyright (c) 2018 Cirque Corp. Restrictions apply. See: www.cirque.com/sw-license

### Overview

This project shows how to use the TM040040 (Sensor0) and TM035035 (Sensor1)
devices at the same time on the DK-000013 demo kit. This is the code that ships
with the device, so if you would like to return your demo kit to it's original
state upload this code to the device. 

Curved Operation:
    For use with the curved overlay optimization, define the OVERLAY_CURVE
    definitions as shown below.

    #define SENSE0_OVERLAY_CURVE  1
    #define SENSE1_OVERLAY_CURVE  1

Flat Operation:
    For use with flat overlay operation, define the OVERLAY_CURVE definitions as
    shown below.

    #define SENSE0_OVERLAY_CURVE  0
    #define SENSE1_OVERLAY_CURVE  0

### Example output from serial terminal:

    Pinnacle Initialized...
    Pinnacle Initialized...

    Setting ADC gain...
    ADC gain set to:	40 (X/2)

    Setting xAxis.WideZMin...
    Current value:	4
    New value:	4

    Setting yAxis.WideZMin...
    Current value:	3
    New value:	3

    Setting ADC gain...
    ADC gain set to:	40 (X/2)

    Setting xAxis.WideZMin...
    Current value:	4
    New value:	4

    Setting yAxis.WideZMin...
    Current value:	3
    New value:	3

            X	Y	Z	       X	Y	Z
                        SENS_1 361	666	22-V
                        SENS_1 361	666	27-V
                        SENS_1 361	666	27-V
    SENS_0 0	0	0-L
    SENS_0 0	0	0-L
    SENS_0 30	116	7-V
                        SENS_1 408	760	34-V
    SENS_0 30	116	13-V
                        SENS_1 403	765	33-V
    SENS_0 30	116	4-V
                        SENS_1 397	771	33-V
    SENS_0 30	116	7-V
                        SENS_1 390	778	32-V
    SENS_0 32	104	6-V
                        SENS_1 383	784	28-V
    SENS_0 34	72	7-V
                        SENS_1 375	789	24-V
    SENS_0 0	0	0-L
                        SENS_1 366	794	15-V
    SENS_0 0	0	0-L
                        SENS_1 0	0	0-L
    SENS_0 0	0	0-L
                        SENS_1 0	0	0-L
    SENS_0 0	0	0-L
                        SENS_1 0	0	0-L
    SENS_0 0	0	0-L
