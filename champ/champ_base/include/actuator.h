/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <hardware_config.h>

#define RAD_TO_DEG 57.2958
#define DEG_TO_RAD 0.0174533


namespace champ
    {
        class Actuator
        {
            float thetas_[12];
            float prev_angle_;

            float leg_joint_position[4][3];

            #ifdef USE_DYNAMIXEL_ACTUATOR

            /**
             *  LEG DEFINITION
             * 2-1-0  3[   ]2   0-1-2
             * 2-1-0  1[ v ]0   0-1-2   
             * 
             **/
            /**
             * 
                legs[total_legs++] = &lf;
                legs[total_legs++] = &rf;
                legs[total_legs++] = &lh;
                legs[total_legs++] = &rh;
            */
            // int dxl_quadruped_servo_address[4][3] = {
            //                                         LFH_SERVO_ID, LFU_SERVO_ID, LFL_SERVO_ID,
            //                                         RFH_SERVO_ID, RFU_SERVO_ID, RFL_SERVO_ID,
            //                                         LHH_SERVO_ID, LHU_SERVO_ID, LHL_SERVO_ID,
            //                                         RHH_SERVO_ID, RHU_SERVO_ID, RHL_SERVO_ID
            //                                         } ;
            int dxl_quadruped_servo_address[4][3] = {
                                                    // // ORI
                                                    // LFH_SERVO_ID, LFU_SERVO_ID, LFL_SERVO_ID,
                                                    // RFH_SERVO_ID, RFU_SERVO_ID, RFL_SERVO_ID,
                                                    // LHH_SERVO_ID, LHU_SERVO_ID, LHL_SERVO_ID,
                                                    // RHH_SERVO_ID, RHU_SERVO_ID, RHL_SERVO_ID

                                                    // ORI
                                                    LFH_SERVO_ID, LFU_SERVO_ID, LFL_SERVO_ID,
                                                    RFH_SERVO_ID, RFU_SERVO_ID, RFL_SERVO_ID,
                                                    RHH_SERVO_ID, RHU_SERVO_ID, RHL_SERVO_ID,
                                                    LHH_SERVO_ID, LHU_SERVO_ID, LHL_SERVO_ID
                                                    } ;

//WIP
            int16_t leg_joint_offsett[4][3] = {
                                        // //ukuran besar X
                                        // 9, 52, 227,
                                        // -11, 259, -204,
                                        // -11, 259, -186,
                                        // 9, 46, 173

                                        // //ukuran besar X
                                        9, 46, 251,
                                        -11, 259, -250,
                                        -11, 259, -250,
                                        9, 46, 251

                                        // // ukuran mini
                                        // 512 + 15, 512 + 79,   512 +  182, //ok
                                        // 512 -20,  512 + 220,  512 - 165, //-217
                                        // 512 + 10, 512 - 105,   512 +  149, //ok 270
                                        // 512 -10,  512 + 402,  512 - 161

                                        //OK ori
                                        // 15, 79, 182, //ok
                                        // -20, 220, -165, //-217
                                        // 10, 92, 208, //ok 270
                                        // -10, 204, -202

                                        // 0, 79, 237, //ok
                                        // -5, 240, -260, //-217
                                        // 5, 82, 263, //ok 270
                                        // 5, 234, -267

                                        //miring
                                        // 0, 79, 237, //ok
                                        // -5, 250, -237, //-217
                                        // -5, 82, 263, //ok 270
                                        // 5, 234, -247

                                        //terbaru
                                        // 0, 79, 237, //ok
                                        // -15, 250, -217,
                                        // 0, 82, 243, //ok
                                        // 0, 234, -232

                                        //awal banget
                                        // 0, 82, 243, //ok
                                        // 0, 224, -217,
                                        // 0, 82, 243, //ok
                                        // 0, 234, -232
                                       };

                      
            // int16_t leg_joint_offsett[4][3] = {
            //                             0, 82, 243, //ok
            //                             0, -60, 217, //176
            //                             0, 82, 243, //ok
            //                             0, -60, 232
            //                            };

            // int16_t leg_joint_offsett[4][3] = {
            //                             0, 60, 201,
            //                             0, -234, 217, //176
            //                             0, 27, 194,
            //                             0, -224, 232
            //                            };

            //dynamixel definition
            int dxl_comm_result = COMM_TX_FAIL;             // Communication result
            bool dxl_addparam_result = false;               // addParam result
            int dxl_moving_speed = DXL_MOVING_SPEED;

            uint8_t dxl_error = 0;                          // Dynamixel error
            uint8_t param_goal_position[2];
            uint8_t param_goal_position_moving_speed[4];
            uint16_t leg_present_position[4][3];  // Present position in leg in dynamixel pulse system : 0 ~ 1023 for 0 ~ 300 degree
            uint8_t leg_present_moving = 0;  // Present position in leg in dynamixel pulse system : 0 ~ 1023 for 0 ~ 300 degree
            uint16_t dxl_goal_position = 512;     // Goal position


            // Initialize PortHandler instance
            // Set the port path
            // Get methods and members of PortHandlerLinux or PortHandlerWindows
            dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

            // Initialize PacketHandler instance
            // Set the protocol version
            // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
            dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

            int setup_dynamixel()
            {
                // Open port
                if (portHandler->openPort())
                {
                  printf("Succeeded to open the port!\n");
                }
                else
                {
                  printf("Failed to open the port!\n");
                  return 0;
                }

                // Set port baudrate
                if (portHandler->setBaudRate(BAUDRATE))
                {
                  printf("Succeeded to change the baudrate!\n");
                }
                else
                {
                  printf("Failed to change the baudrate!\n");
                  return 0;
                }

                // //enable torque
                for (int leg_number = 0; leg_number < 4; leg_number++)
                {
                    for(int joint_number = 0; joint_number < 3; joint_number++)
                    {
                      //   ROS_INFO("Dynamixel leg %d - joint %d - ID: %d", leg_number, joint_number, dxl_quadruped_servo_address[leg_number][joint_number]);
                        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_quadruped_servo_address[leg_number][joint_number], ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
                        if (dxl_comm_result != COMM_SUCCESS)
                        {
                          printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                        }
                        else if (dxl_error != 0)
                        {
                          printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                        }
                        else if (dxl_comm_result == COMM_SUCCESS)
                        {
                          printf("Dynamixel#%d has been successfully connected \n", dxl_quadruped_servo_address[leg_number][joint_number]);
                        }
                    }
                }
            }
            #endif

            

            public:
                Actuator()
                {
                  #ifdef USE_DYNAMIXEL_ACTUATOR
                    setup_dynamixel();
                  #endif
                }

                void moveJoints(float joint_positions[12])
                {
                    // for(unsigned int i = 0; i < 12; i++)
                    // {
                    //   printf("joint %d joint_pos: %.02f \n", i, joint_positions[i]);
                    // }
                  #ifdef USE_DYNAMIXEL_ACTUATOR
                    for(unsigned int i = 0; i < 4; i++)
                    {
                      if (i == 0 || i == 2)
                      {
                        // printf("leg %d joint_pos: %.02f - %.02f - %.02f \n", i, joint_positions[i*3], joint_positions[(i*3) + 1], joint_positions[(i*3) + 2]);
                        leg_joint_position[i][0] = joint_positions[i*3];
                        leg_joint_position[i][1] = joint_positions[(i*3) + 1];
                        leg_joint_position[i][2] = joint_positions[(i*3) + 2];
                        // printf("leg_j_p %d joint_pos: %.02f - %.02f - %.02f \n", i, leg_joint_position[i][0], leg_joint_position[i][1],leg_joint_position[i][2]);
                      }
                      else
                      {
                        // printf("leg %d joint_pos: %.02f - %.02f - %.02f \n", i, joint_positions[i*3], joint_positions[(i*3) + 1], joint_positions[(i*3) + 2]);
                        leg_joint_position[i][0] = -joint_positions[i*3];
                        leg_joint_position[i][1] = -joint_positions[(i*3) + 1];
                        leg_joint_position[i][2] = -joint_positions[(i*3) + 2];
                        // printf("leg_j_p %d joint_pos: %.02f - %.02f - %.02f \n", i, leg_joint_position[i][0], leg_joint_position[i][1],leg_joint_position[i][2]);                      
                      }
                      
                    }

                    syncWriteJoints();
                  #endif

                    // thetas_[leg_id] = joint_position;            


                    // //add dynamixel moving speed value to the syncwrite storage
                    // for (int leg_number = 0; leg_number < 4; leg_number++)
                    // {
                    //     for(int joint_number = 0; joint_number < 3; joint_number++)
                    //     {
                    //         // Add Dynamixel#1 moving speed value to the Syncwrite storage
                    //         dxl_addparam_result = groupSyncWrite.addParam(dxl_quadruped_servo_address[leg_number][joint_number], param_goal_position_moving_speed);
                    //         if (dxl_addparam_result != true)
                    //         {
                    //           fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_quadruped_servo_address[leg_number][joint_number]);
                    //         //   return 0;
                    //         }
                    //         // else 
                    //         // {
                    //         //   printf("[ID:%03d] groupSyncWrite addparam success \n", dxl_quadruped_servo_address[leg_number][joint_number]);
                    //         // }
                    //     }
                    // }
                    
                    
                    for(unsigned int i = 0; i < 12; i++)
                    {
                        moveJoint(i, joint_positions[i]);
                    }
                }

                void moveJoint(unsigned int leg_id, float joint_position)
                {
                    //until a proper hardware interface is done, hardware integration can be done here
                    
                    //joint_position can be passed to the hardware api to set the joint position of the actuator

                    //this stores the set joint_position by the controller for pseudo feedback.
                    float delta = (joint_position - thetas_[leg_id]);
                    //the random number is just to add noise to the reading
                    thetas_[leg_id] = thetas_[leg_id] + (delta * (((rand() % 80) + 70) / 100.0)); 
                    // thetas_[leg_id] = joint_position;              
                }

                void getJointPositions(float joint_position[12])
                {
                  // float delta_joint_position[4][3];
                  // readAllJoints(joint_position); //MASIH BINGUNG

                  // for (int leg_number = 0; leg_number < 4; leg_number++)
                  // {
                  //     for (int joint_number = 0; joint_number < 3; joint_number++)
                  //     {
                  //       delta_joint_position[leg_number][joint_number] = leg_present_position[leg_number][joint_number];
                  //     }
                  // }
                  
                  // return thetas_[leg_id];

                    for(unsigned int i = 0; i < 12; i++)
                    {
                        joint_position[i] = getJointPosition(i);
                    }
                }

                float getJointPosition(unsigned int leg_id)
                {

                    //virtually this returns the stored joint_position set by the controller
                    //until a proper hardware interface is done, this can be used to return
                    //real actuator feedback

                    //simulation
                    return thetas_[leg_id];
                }
                  #ifdef USE_DYNAMIXEL_ACTUATOR
                  int syncWriteJoints() // the dynamixel function to  move all the joints at the sametime / simultaneously
                  {
                      // Initialize GroupSyncWrite instance
                      dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_MOVE_WITH_SPEED);

                      for (int leg_number = 0; leg_number < 4; leg_number++)
                      {
                          for (int joint_number = 0; joint_number < 3; joint_number++)
                          {
                          
                              //   ROS_INFO("Dynamixel leg %d - joint %d - ID: %d", leg_number, joint_number, dxl_quadruped_servo_address[leg_number][joint_number]);
                              // convert dynamixel goal position value into byte array
                              // if (leg_number == 0 || leg_number == 2)
                              // {
                              // fprintf(stderr, "leg_number : %d joint : %d dxl_goal_position = %.02f \n", leg_number, joint_number, leg_joint_position[leg_number][joint_number]);
                              //  dxl_goal_position = (((leg_joint_position[leg_number][joint_number]*RAD_TO_DEG) * DEG_TO_PULSE) + leg_joint_offsett[leg_number][joint_number]); //normal
                              
                              // ukuran kecil
                              //  if ( (leg_number == 3) || (leg_number == 2) )
                              //  {
                              //    dxl_goal_position = leg_joint_offsett[leg_number][joint_number] - ((leg_joint_position[leg_number][joint_number]*RAD_TO_DEG) * DEG_TO_PULSE);
                              //    fprintf(stderr, "leg_number : %d joint : %d leg_joint = %0.2f dxl_goal = %d \n", leg_number, joint_number, leg_joint_position[leg_number][joint_number]*RAD_TO_DEG*DEG_TO_PULSE, dxl_goal_position);
                              //  }
                              //  else 
                              //  dxl_goal_position = leg_joint_offsett[leg_number][joint_number] + ((leg_joint_position[leg_number][joint_number]*RAD_TO_DEG) * DEG_TO_PULSE); //normal
                            //  fprintf(stderr, "leg: %d joint: %d ID: %02d pos: %d \n", leg_number, joint_number, dxl_quadruped_servo_address[leg_number][joint_number], dxl_goal_position);
                              
                               if (joint_number == 0 && (leg_number == 1 || leg_number == 2))
                               {
                                 dxl_goal_position = 512 + leg_joint_offsett[leg_number][joint_number] - ((leg_joint_position[leg_number][joint_number]*RAD_TO_DEG) * DEG_TO_PULSE);
                                //  fprintf(stderr, "leg_number : %d jtrueoint : %d leg_joint = %0.2f dxl_goal = %d \n", leg_number, joint_number, leg_joint_position[leg_number][joint_number]*RAD_TO_DEG*DEG_TO_PULSE, dxl_goal_position);
                               }
                               else
                               dxl_goal_position = 512 + leg_joint_offsett[leg_number][joint_number] + ((leg_joint_position[leg_number][joint_number]*RAD_TO_DEG) * DEG_TO_PULSE); //normal
                              //  if (leg_number == 2 && joint_number == 1) fprintf(stderr, "leg_number : %d joint : %d leg_joint = %0.2f dxl_goal = %d \n", leg_number, joint_number, leg_joint_position[leg_number][joint_number]*RAD_TO_DEG*DEG_TO_PULSE, dxl_goal_position);
                              
                              
                              // fprintf(stderr, "leg_number : %d joint : %d dxl_goal_position = %04d \n", leg_number, joint_number, dxl_goal_position);
                              // }
                              // else
                              // {
                              // // fprintf(stderr, "leg_number : %d joint : %d dxl_goal_position = %.02f \n", leg_number, joint_number, leg_joint_position[leg_number][joint_number]);
                              //  dxl_goal_position = (((leg_joint_position[leg_number][joint_number]*RAD_TO_DEG) * DEG_TO_PULSE) + (512 + leg_joint_offsett[leg_number][joint_number]));
                              // fprintf(stderr, "leg_number : %d joint : %d dxl_goal_position = %d \n", leg_number, joint_number, dxl_goal_position);
                              // }
                              
                              // dxl_goal_position = (((leg_joint_position[leg_number][joint_number]*RAD_TO_DEG) * DEG_TO_PULSE + 512) + leg_joint_offsett[leg_number][joint_number]);

                              // fprintf(stderr, "dxl_goal_position = %d \n", dxl_goal_position);
                              param_goal_position_moving_speed[0] = DXL_LOBYTE(dxl_goal_position);
                              param_goal_position_moving_speed[1] = DXL_HIBYTE(dxl_goal_position);
                              param_goal_position_moving_speed[2] = DXL_LOBYTE(dxl_moving_speed);
                              param_goal_position_moving_speed[3] = DXL_HIBYTE(dxl_moving_speed);

                              dxl_addparam_result = groupSyncWrite.addParam(dxl_quadruped_servo_address[leg_number][joint_number], param_goal_position_moving_speed);
                              // if (leg_number == 2) printf("id2: %02d dxl_goal_pos: %03d\n", dxl_quadruped_servo_address[leg_number][joint_number], dxl_goal_position);
                              // if (leg_number == 1) printf("id1: %02d dxl_goal_pos: %03d\n", dxl_quadruped_servo_address[leg_number][joint_number], dxl_goal_position);
                              // printf("id: %02d dxl_goal_pos: %03d\n", dxl_quadruped_servo_address[leg_number][joint_number], dxl_goal_position);
                              // if (dxl_addparam_result != true)
                              // {
                              //   // fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_quadruped_servo_address[leg_number][joint_number]);
                              //   // return 0;
                              // }
                          }
                      }

                      // Syncwrite goal position
                      dxl_comm_result = groupSyncWrite.txPacket();
                      // if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                      // else printf("Sukses syncwrite goal !\n");

                      groupSyncWrite.clearParam();

                      dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, dxl_quadruped_servo_address[1][2], ADDR_AX_MOVING_, &leg_present_moving, &dxl_error);
                      if (dxl_comm_result != COMM_SUCCESS)
                      {
                        // printf("sukses %s\n", packetHandler->getTxRxResult(dxl_comm_result));
                      }
                      else if (dxl_error != 0)
                      {
                        // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                      }
                      // else printf("status  %d\n", leg_present_moving);



                  }

                  void readAllJoints(float joint_position[12]) // the dynamixel function to  read all joints
                  {
                    int leg_id = 0;
                      for (int leg_number = 0; leg_number < 4; leg_number++)
                      {
                          for (int joint_number = 0; joint_number < 3; joint_number++)
                          {
                              // Read present position
                              dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_quadruped_servo_address[leg_number][joint_number], ADDR_AX_PRESENT_POSITION, &leg_present_position[leg_number][joint_number], &dxl_error);
                              if (dxl_comm_result != COMM_SUCCESS)
                              {
                                // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                              }
                              else if (dxl_error != 0)
                              {
                                // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                              }

                              joint_position[leg_id] = ((((leg_present_position[leg_number][joint_number] - leg_joint_offsett[leg_number][joint_number]) * PULSE_TO_DEG) - HALF_RANGE) * DEG_TO_RAD); //leg_present_position is on dynamixel pulse system
                              printf("[ID:%03d] PresPos:%03d Rad:%.03f\n",  dxl_quadruped_servo_address[leg_number][joint_number], leg_present_position[leg_number][joint_number], thetas_[leg_id]);

                              leg_id++;
                          }
                      }
                  }
                  #endif


                // int cobaTest()
                // {
                  // uint16_t dxl_goal_position2 = 512;     // Goal position

                    // Initialize GroupSyncWrite instance
                    // dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_MOVE_WITH_SPEED);
                //     dxl_goal_position = (((joint_positions[4]*RAD_TO_DEG) + HALF_RANGE) * DEG_TO_PULSE) + 471);
                //     param_goal_position[0] = DXL_LOBYTE(dxl_goal_position2);
                //     param_goal_position[1] = DXL_HIBYTE(dxl_goal_position2);
                //     param_goal_position_moving_speed[0] = param_goal_position[0];
                //     param_goal_position_moving_speed[1] = param_goal_position[1];
                //     param_goal_position_moving_speed[2] = DXL_LOBYTE(dxl_moving_speed);
                //     param_goal_position_moving_speed[3] = DXL_HIBYTE(dxl_moving_speed);


                //     dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position_moving_speed);
                //     if (dxl_addparam_result != true)
                //     {
                //       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 3);
                //     //   return 0;
                //     }
                //     dxl_goal_position2 = (((joint_positions[5]*RAD_TO_DEG) + HALF_RANGE) * DEG_TO_PULSE) + 471);
                //     param_goal_position[0] = DXL_LOBYTE(dxl_goal_position2);
                //     param_goal_position[1] = DXL_HIBYTE(dxl_goal_position2);
                //     param_goal_position_moving_speed[0] = param_goal_position[0];
                //     param_goal_position_moving_speed[1] = param_goal_position[1];
                //     param_goal_position_moving_speed[2] = DXL_LOBYTE(dxl_moving_speed);
                //     param_goal_position_moving_speed[3] = DXL_HIBYTE(dxl_moving_speed);


                //     dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position_moving_speed);
                //     if (dxl_addparam_result != true)
                //     {
                //       fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 3);
                //     //   return 0;
                //     }

                //     // Syncwrite goal position
                //     dxl_comm_result = groupSyncWrite.txPacket();
                //     if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                //     // else printf("Sukses syncwrite goal !\n");


                //     fprintf(stderr, "%03d %03d - %.03f %.03f \n", dxl_goal_position, dxl_goal_position2,joint_positions[4],joint_positions[5]); //kaki 2
                //     groupSyncWrite.clearParam();

                // }
        };
    }
#endif
