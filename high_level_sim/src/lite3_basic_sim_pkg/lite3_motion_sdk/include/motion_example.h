#include "command_list.h"
#include "parse_cmd.h"
#include "robot_types.h"
// #include <iostream>
#include <time.h>
#include <string.h>
#include "Eigen/Dense"
using namespace std;
using namespace Eigen;

typedef Matrix< double, 3, 1> Vec3;

const double kDegree2Radian = 3.1415926 / 180;

class MotionExample{
  private:

  public:
    void setCmd(RobotCmd &cmd){  
    }
    /**
     * @brief Spend 1 s putting the robot's legs away and preparing to stand
     * @param cmd Issue control command
     * @param time Current timestamp
     * @param data_state Real-time status data of robot
     */
    void PreStandUp(RobotCmd &cmd, double time,RobotState &data_state);
    
    /**
     * @brief Spend 1.5s standing
     * @param cmd Issue control command
     * @param time Current timestamp
     * @param data_state Real-time status data of robot
     */
    void StandUp(RobotCmd &cmd, double time,RobotState &data_state);

    /**
     * @brief Specifically achieve swinging one leg of the robot to a specified position within a specified time
     * @param initial_angle 
     * @param final_angle
     * @param total_time
     * @param run_time
     * @param cycle_time Control cycle, default is 1ms
     * @param side Control which leg, FL is the left front leg, FR is the right front leg, HL is the left and right leg, and HR is the right rear leg
     * @param cmd Issue control command
     * @param data Real-time status data of robot
     */
    void SwingToAngle(Vec3 initial_angle, Vec3 final_angle, double total_time, double run_time, double cycle_time, string side, RobotCmd &cmd,  RobotState &data);

    /**
     * @brief Interpolation to find the path point, i.e. the target angle for each control cycle
     * @param init_pos 
     * @param init_vel 
     * @param goal_pos 
     * @param goal_vel 
     * @param run_time 
     * @param cycle_time Control cycle, default is 1ms
     * @param total_time 
     * @param sub_goal_pos Target angle for the control cycle
     * @param sub_goal_pos_next Target angle for the next control cycle
     * @param sub_goal_pos_next2 Target angle for the next and next control cycle
     */
    void CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time, double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next, double &sub_goal_pos_next2);
    
   /**
    * @brief Only the current moment and angle are recorded
    * @param data Current joint data
    * @param time Current timestamp
    */
    void GetInitData(RobotData data, double time);
};

