Tasks:
1. Write an STM32 program to get the temperature from DHT22 sensor and send it over
CAN bus. (IDE- Keil/Make, Language- c/c++, STM32 version- any)
2. Write an ESP32 program to receive a string from BLE and push it to MQTT and send
the string received from MQTT over BLE. (IDE- PlatformIO, Language- c/c++, Mobile
Application- SerialBluetooth, ESP32 version- any)
3. Write a pseudo code for a battery swapping station.
Note:
● Please feel free to use all resources.
● Please attach resources used while performing the above tasks.

Solutions:
1. The solution for this is inside the folder named "batterypool_STM_CAN". I have used STM32MxCube-IDE to generate compatible files for the project and included the required functions in the can.h and can.c files. Also, I created dht22.h and dht22.c files taking some reference from the tutorial https://controllerstech.com/temperature-measurement-using-dht22-in-stm32/
2. The solution for this is inside the folder named "esp32doit-devkit-v1_MQTT_BLE". I have referred to the example codes available for PubSubClient and BluetoothSerial in the Arduino-IDE and tried creating the required algorithm for the same.
3. The solution for this task is written in the file named "PsuedoCode_BatterySwapping.txt".
Note:
● I have not tested the solution for any of the tasks because of inaccessibility to hardware. And there will be a need for a lot of revisions for each of the firmware (at least I like building firmware step by step for each function). I have rarely seen firmware made in a single go that works perfectly fine.
● I have assumed many things while defining the functions for CAN, DHT22, MQTT-topic, WIFI - Bluetooth settings, and even in pseudo-code.(mentioned in detail in PsuedoCode_BatterySwapping.txt), so please feel free to ask if there is a need of clarification.