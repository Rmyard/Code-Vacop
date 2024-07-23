# Code-Vacop

3 nodes of CAN bus on the 3 raspberry pi:

-Brake_node
  BRAKE.py
  CAN_List.txt 
  
-Steer_node
  STEER.py
  CAN_List.txt
  
-OBU_master
  OBU.py #pilot with the graphical interface
  OBU2.py #pilot with the controler
  CAN_List.txt 

CAN_List.txt is the file with all the can adresses. Its a translation of human readable messages to hexadecimal
This file MUST be the same in all 3 nodes. 

Other folders : 
-Test_code
  motor_test.py #control the torque aplied to the brushless motors (from 0 to 100%)
  brake_openloop_control.py #control the speed of the electric actuator (from -100% to +100%)
  post_processor.py #plot graphs of the sensor_log.csv file
