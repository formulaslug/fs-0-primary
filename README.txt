TODO:
- Clean the input throttle voltage so the ADC value doesn't jump everywhere (add a required change of ~3 or 4). Probably best to add to primary controls a utility function that can be passed 1: adcValue, 2: requiredDelta, and then will only return a new value if it's different from the previous value by requiredDelta or more
- Implement function canPrimary2Secondary() that queues all primary->secondary communication to CAN bus. This only includes information not already being written to the bus for Primary->Master communication
  - Add primary teensy's state (of fsm) to the packet in canPrimary2Secondary()
