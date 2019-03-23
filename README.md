# ESP32 Arduino IDE Template
This is an ESP32 template with some (hopefully) useful functions:  
- ESP DeepSleep Mode
- Watchdog
- OTA Updates through Arduino IDE
- MQTT
- Monitoring VCC (-> doesn't work!)

This sketch does actually nothing than blinking the LED, publish VCC to an MQTT topic and providing the functions mentioned above, so you can use it as a template for whatever you want to do.  

I think the code is well documented, some hints in advance:  
- You will *require* a MQTT broker (auth not implemented, so you will probably want to use a local broker)
- Code included for reading Vcc, however i can't get this working as the ADC doesn't seem to have a valid reference voltage (maybe an issue of my DevKitC board).

Have fun,  
Juergen
