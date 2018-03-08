# STM32F103-Code
Code that runs on the STM32F103 (BluePill Micro onboard the quad)



Writing custom code to run on the Naze32 Flight controller alongside betaflight sounded difficult and slightly dodgy. Instead, onboard the quad there's a STM32F103 that recieves CPPM from the Rx and Outputs it to the FC like a little bridge. In manual mode this is all that happens and it's as thought the Rx is directly connected to the FC like in any other drone.

But when a Tx toggle switch is flicked, the STM enters an Auto mode. Either altitude hold mode or position+altitude hold mode (not implimented yet). In the former, Pitch/Roll/Yaw are alled passed unmodified from Rx to FC but the throttle is modified very slightly based on readings from the onboard and connected sonar.

A PID algorithm runs taking sonar readings as the input and producing throttle values for output. Theoretically, if tuned correctly this should keep the quad flying at a constant altitude...

The code runs a little real time OS so it initially starts up some tasks to do things such as 
- Read the px4Flow every 50ms (which includes getting sonar readings and filtering them a bit)
- Blink an LED on the STM board as an indicator it's running
- Set Manual/Auto mode based on whether Ch6 of the CPPM signal is low or high
- Send UART over telemetry for debugging purposes. Eg. send sonar readings and PID outputs
- Pass through the CPPM vaalues from Rx to FC unmodified/modified based on Mode

A Periodic function runs every 55ms (slightly slower than the 50ms sonar reading updates) to compute the PID output. Look through the code, it's simple enough.

Perhaps a few other things to mention
- The difference in PWM throttle values between a slow decent and a slow ascent is frikin tiny like 10-20us. (From a 1000us range)
- As battery voltage falls a higher PWM throttle value is needed for hover
- We do PID output averaging (eg. 10 sample) to avoid any sudden jumpy changes in throttle which could be catastrophic!
- We filter the sonar readings (don't know if this is neccessary) with a 5 sample median filter and a slew rate limiter...?
- The sonar outputs in millimetres, it can't measure <300mm
- We only allow Alt_hold mode to become active above a certain height (~500mm, with hysterisis) to prevent integral windup in the PID??

Stuff on PIDs
 - D is essential I think. Since there is no natural damping in the system.
