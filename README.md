# MEMS_bluetooth_nrf

## Description

Code for nRF54l15 SoC on an nRF54l15dk board for low power logging and data transfer of a MEMS microphone.

## To Do
MTU and connection parameters
Long data read
Data circular buffer (file)
Persistent BLE characteristics

## Notes

When BLE connects it tells the sensor what the current time is. The value is stored both in a BLE characteristic in flash, and as the first value of the sensor data octet array. The timer is also reset to 0. Each sensor readout stores the sensor value in the array, and resets the timer. The sensor values are reconstructed as the sum of previous timer deltas + current delta + first timestamp.


Button 0 simulates BLE wakeup cycle (BLE core wakeup, advertising, connecting, transfering data, ...) Later an RTC will be used each n minutes
Button 1 resets the bonded devices
Button 2 simulates sensor data in category 2
Button 3 simulates sensor data in category 3

Upon reset, the sensor loads the BLE configuration from flash.
The buttons act analogue to the sensor, they are external interrupts.
Each sensor event gets timestamped and parsed through a computational dummy function, then stored into:
* Memory
* External flash (later)
The characteristic holding sensor data holds the pointer to the data.
The sensor data is a FIFO buffer with a circular pointer. Data sent isn't deleted, instead the old pointer is stored in case of failure.
Data is overwritten if buffer reaches the end.
Timestamps are held as differences from previous ones. After reset and transfer, a new timestamp is stored to ease working with data on the client's side.
If possible, timestamp with data is held in a single characteristic for characteristic retrieval efficiency. In case this is not possible, a timestamp characteristic will be introduced.
All sensor data is sent as binary octet data for data transfer advantages.
A characteristic for time synchronization will be introduced.

Authenticated devices are saved in the whitelist. For now 1 device, later n devices could connect to fetch data. Authentication process is TODO as I'd like for devices to be addable at runtime, not only during setup.


LED0 indicates BLE core power mode
LED1 indicates pairing mode


## TODO
Add read and write handlers for characteristics.
Handle new characteristic values at runtime.
Create sensor dummy data creator.
Add dynamic advertising thread.
Add sleep modes.



