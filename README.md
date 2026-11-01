This project showcases how a lane can be given a priority based on vehicular density so that the timers given to the green signal in each lane
is according to how much traffic is there per lane, which solves the problem of fixed timers, so that the empty and filled lanes both
do not get the same green time signal.
The vehicular density is calculated by |Number of incoming vehicles - Number of Outgoing vehicles|, fow which each lane has 2 IR sensors,
one for incoming vehicle count,and one for outgoing.
We have also included pedestrian priority , where the push buttons are used to activate pedestrian mode. 
If the push button for pedestrian mode is pressed thrice within 1-2 seconds, it switches to elderly mode, where we get prolonged time pedestrian.
Emergency mode for ambulance, fire trucks has also been initiated, where push buttons and buzzers are used. When the emergency button is 
pressed for that particular lane, the signals for all other lanes turn red and only that particular lane is given green signal.

Register level C programming is used to implement this in STM32CubeIDE, and it was flashed using STM32CubePrg
Hardware: STM32F103C8T6(Blue pill), ST Link v2,12 LEDs(Red, Yellow Green), 4 7-segment displays TM1637(for timer of each lane), 8 IR sensors, I2C expanders,4 Buzzers, 8 push buttons(2 per lane- one for emergency, one for pedestrian)
Software: STM32CubeIDE,STM32CubePrg, CMSIS device headers(Register definitions for STM32F103), UART Terminal(to help check counts, and fault messages while testing)
