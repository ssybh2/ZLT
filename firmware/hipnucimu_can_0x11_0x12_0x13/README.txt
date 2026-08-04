Target: STM32G431 CAN IMU module matching AIMEtherCAT/hipnucimu hardware
CAN standard IDs: packet1=0x11, packet2=0x12, packet3=0x13
CAN bitrate: 1 Mbit/s
Use for the SECOND IMU only.
H750 config app_4 must listen to 0x11, 0x12, 0x13.
Do not flash into the H750 EtherCAT slave.
Before flashing, back up the original firmware if possible and confirm ST-LINK detects STM32G431.
