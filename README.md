# STM32F103-Code
Code that runs on the STM32F103 (aka BluePill microcontroller) onboard the quad)

Writing custom code to run on the Naze32 Flight controller alongside betaflight sounded difficult and dodgy. Instead, onboard the quadcopter there's a STM32F103 that recieves CPPM from the Receiver and Outputs it to the Flight contrller like a bridge. In manual mode this is all that happens and it's as though the Receiver is directly connected to the FC like in any other drone.

But when a Transmitter 3-position toggle switch is flicked, the STM enters an Auto mode:
 - Altitude hold mode -> Just throttle PPM modified to maintain a height setpoint
 - Altitude + Position hold mode -> Throttle, Roll & Pitch modified to maintain height and position setpoint

Separate PID algorithms run to control for x, y & z. Height is measured using the TFmini LIDAR module, x,y position is measured using the Px4Flow sensor.

The code runs a real time operating system. Initially it configures peripherals before starting up some tasks to do: 
- Read the px4Flow every 50ms 
- Blink an LED on the STM board as an indicator it's running
- Set Manual/Height/Auto mode based on whether Ch6 of the CPPM signal is 1000, 1500 or 2000us.
- Send UART over telemetry for debugging purposes. Eg. send LIDAR or flow readings and PID outputs etc
- Pass through the CPPM vaalues from Rx to FC unmodified/modified based on Mode

PID updates are run via periodic functions.

Perhaps a few other things to mention
- We do PID output averaging (eg. 20 samples) to avoid any sudden jumpy changes in throttle. (Important to do this!)
- We prevent PID windup when sat on the floor by zeroing the integral PID parts when throttle is < 1100us.
- Having non-zero D gain is essential in all the PIDs. Since there is no natural damping in the system.
- Our pitch and roll angles are small so they won't effect the height readings from the LIDAR much.

To Do:
 - Add position (x,y) global variables that PX4flow readings update
 - Test px4flow x,y readings and drift in position over time
 - Tune X & Y PIDs for position hold

 - Get UART from Pi working
 - Allow Pi to send X,Y,Yaw corrections
 - Allow for Yaw correction
