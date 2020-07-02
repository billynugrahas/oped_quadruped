#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

//#define USE_SIMULATION_ACTUATOR
#define USE_DYNAMIXEL_ACTUATOR
// #define USE_SERVO_ACTUATOR
// #define USE_BRUSHLESS_ACTUATOR

#define USE_SIMULATION_IMU
// #define USE_BNO0809DOF_IMU

#define USE_ROS
// #define USE_ROS_RF

#ifdef USE_ROS_RF
    #define ELE_PIN 16
    #define AIL_PIN 21
    #define RUD_PIN 17
    #define THR_PIN 20
    #define AUX1_PIN 22
    #define AUX2_PIN 23
    #define RF_INV_LX false
    #define RF_INV_LY false
    #define RF_INV_AZ false
    #define RF_INV_ROLL true
    #define RF_INV_PITCH false
    #define RF_INV_YAW false
#endif

#ifdef USE_DYNAMIXEL_ACTUATOR
    #define LFH_SERVO_ID 3
    #define LFU_SERVO_ID 4
    #define LFL_SERVO_ID 5

    #define RFH_SERVO_ID 0
    #define RFU_SERVO_ID 1
    #define RFL_SERVO_ID 2

    // #define RFU_SERVO_ID 2
    // #define RFL_SERVO_ID 3

    //nanti dibalik lagi, rfh rfu harusnya 3 4

    #define LHH_SERVO_ID 6
    #define LHU_SERVO_ID 7
    #define LHL_SERVO_ID 8

    #define RHH_SERVO_ID 9
    #define RHU_SERVO_ID 10
    #define RHL_SERVO_ID 11


    #define LFH_INV false
    #define LFU_INV false
    #define LFL_INV true

    #define RFH_INV false
    #define RFU_INV true
    #define RFL_INV false

    #define LHH_INV false
    #define LHU_INV false
    #define LHL_INV true

    #define RHH_INV false
    #define RHU_INV true
    #define RHL_INV false


    // Control table address
    #define ADDR_AX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
    #define ADDR_AX_GOAL_POSITION           30
    #define ADDR_AX_PRESENT_POSITION        36
    #define ADDR_AX_MOVING_                 46
    
    // Data Byte Length
    #define LEN_AX_MOVE_WITH_SPEED          4
    
    // Protocol version
    #define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
    
    #define BAUDRATE                        1000000          //try to change baudrate
    #define DEVICENAME                      "/dev/U2D2"      // Check which port is being used on your controller
                                                                // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        
    
    #define TORQUE_ENABLE                   1                   // Value for enabling the torque
    #define TORQUE_DISABLE                  0                   // Value for disabling the torque
    
    #define DXL_MOVING_SPEED                600 // X ORIENTATION
    // #define DXL_MOVING_SPEED                500 // ORI

    #define DEG_TO_PULSE                    3.41 //3.413333333
    #define PULSE_TO_DEG                    0.29296875
    #define HALF_RANGE                      150                 // to re-map -HALF_RANGE TO +HALF_RANGE as 0 TO +(HALF_RANGE*2). dynamixel AX-12 Range: 300 degree. so it convert -150~150 into 0~300.

#endif 

#ifdef USE_SERVO_ACTUATOR
    #define LFH_PIN 2
    #define LFU_PIN 3
    #define LFL_PIN 4

    #define RFH_PIN 23
    #define RFU_PIN 22
    #define RFL_PIN 21

    #define LHH_PIN 6
    #define LHU_PIN 7
    #define LHL_PIN 8

    #define RHH_PIN 17
    #define RHU_PIN 16
    #define RHL_PIN 14

    #define LFH_INV false
    #define LFU_INV false
    #define LFL_INV true

    #define RFH_INV false
    #define RFU_INV true
    #define RFL_INV false

    #define LHH_INV false
    #define LHU_INV false
    #define LHL_INV true

    #define RHH_INV false
    #define RHU_INV true
    #define RHL_INV false
#endif

#endif
