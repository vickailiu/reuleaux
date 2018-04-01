#ifndef KINEMATICS
#define IKFAST_HAS_LIBRARY  // Build IKFast with API functions
#define IKFAST_NO_MAIN

#define IK_VERSION 61
//#include "ikfast_ur3.cpp"
//#include "abb_irb2400_manipulator_ikfast_solver.cpp"
//#include "ikfast_ur3_finger.cpp"
//#include "ikfast_ur3_joint_limited.cpp"

//TODO: MX:
// create a new header: joint_limit.h and define joint limit in the header
// include the header here.


#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal  // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal  // for IKFast 54
#endif

namespace kinematics
{
class Kinematics
{
public:
  // kinematics();
  //~kinematics();

  float SIGN(float x);
  float NORM(float a, float b, float c, float d);
  void getPoseFromFK(const std::vector< double > joint_values, std::vector< double >& pose);

  bool isIKSuccess(const std::vector<double> &pose, std::vector<double> &joints, int& numOfSolns);



  const std::string getRobotName();

  bool isIkSuccesswithTransformedBase(const geometry_msgs::Pose& base_pose, const geometry_msgs::Pose& grasp_pose, std::vector<double>& joint_soln,
                                      int& numOfSolns);


};
}

#endif  // KINEMATICS
