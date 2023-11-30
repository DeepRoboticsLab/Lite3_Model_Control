/**
 * @file robot_types.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef SEND_TO_ROBOT_H_
#define SEND_TO_ROBOT_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include "command_list.h"
#include "udp_socket.hpp"
#include "robot_types.h"

#define ROBOT_IP "192.168.1.120"
#define ROBOT_PORT 43893

#define ABLE 2
#define UNABLE 1

/**
 * @brief A class used to send robot data.
 */
class SendToRobot{
  private:
    RobotCmd robot_cmd; /**< View robot_tppes.h.*/
    UDPSocket *udpSocket; /**< What udpSocket needs*/

    /**
    * @brief Send instructions.
    * @param Command
    */
    void cmd_done(Command& command);
  public:
    /**
    * @brief Create the send thread using the work()
    */
    void startWork();

    /**
    * @brief Send data to robot.
    */   
    void work();

    /**
    * @brief Initialization timer for receiving data
    */ 
    void init(void);

    /**
    * @brief Send the set joint target status to the robot.Input:RobotCmd
    * @param RobotCmd
    */ 
    void set_send(RobotCmd&);

    /**
    * @brief Initialization timer for receiving data
    */ 
    void all_joint_back_zero(void); /**< Set all joint motors of the robot to zero.*/

    /**
    * @brief Initialize the robot for the first time after powering on
    */ 
    void robot_state_init(void);

    /**
    * @brief Set the command code and command value to send
    * @param code
    * @param value
    */ 
    void set_cmd(uint32_t code , uint32_t value);

    /**
    * @brief Select the control right
    * @param  1:Original robot algorithm control  2: SDK control.
    */ 
    void control_get(int mode){ 
      if(mode == UNABLE){
        RobotCmd cmd;
        for(int i = 0; i < 12; i++){
          cmd.joint_cmd[i].pos = 0.0;
          cmd.joint_cmd[i].tor = 0.0;
          cmd.joint_cmd[i].vel = 0.0;
          cmd.joint_cmd[i].kp = 0.0;
          cmd.joint_cmd[i].kd = 5.0;
        }
        set_send(cmd);
        sleep(2);
        Command command_temp(0x0113,0, 0);
       cmd_done(command_temp);
      }else if (mode == ABLE){
        Command command_temp(0x0114,0, 0);
  cmd_done(command_temp);
      }
    }// void control_get(void);
};

/**
 * @brief A class used to get time.
 */
class Time_Tool{
  private:
    int tfd;    /**< Timer descriptor.*/
    int efd;    /**< Epoll descriptor.*/
    int fds, ret; /**< Variables used to initialize the timer.*/
    uint64_t value; /**< Variables used to initialize the timer.*/
    struct epoll_event ev, *evptr; /**< Variables used to initialize the timer.*/
    struct itimerspec time_intv;  /**< Variables used to initialize the timer.*/
  public:
    timespec system_time; /**< A class for accurately obtaining time.*/
    /**
    * @brief Initialize timer, input cycle(ms).
    * @param Cycle time unit: ms
    */ 
    void time_init(int ms); 

    /**
    * @brief Acquire interrupt signal
    * @return 1:Enter interrupt 0:no
    */ 
    int time_interrupt(); /**< Acquire interrupt signal.*/

    /**
    * @brief How long has it been
    * @param Initial time
    */ 
    double get_now_time(double start_time);

    /**
    * @brief Get current time
    */ 
    double get_start_time(); /**< Get start time.*/
};


#endif  // PARSE_CMD_H_