TODO:
- Clean the input throttle voltage so the ADC value doesn't jump everywhere (add a required change of ~3 or 4). Probably best to add to primary controls a utility function that can be passed 1: adcValue, 2: requiredDelta, and then will only return a new value if it's different from the previous value by requiredDelta or more
- Write primary teensy's state to the CAN bus when it changes
