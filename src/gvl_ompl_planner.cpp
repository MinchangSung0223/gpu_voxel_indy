// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*

#include <iostream>
using namespace std;
#include <signal.h>

#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <unistd.h> 


#include "gvl_ompl_planner_helper.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <thread>
#include <memory>

#include <Python.h>
#include <stdlib.h>
#include <vector>

#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
using namespace std;

#include <iostream>
#include <fstream> 
#include <jsoncpp/json/json.h>

#pragma comment(lib, "jsoncpp.lib")

// initial quaternion 0.49996,0.86605,0.00010683,0

std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr;
bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}

int main(int argc, char **argv)
{ 



//LOAD JSON data
  cout<<argv[1]<<endl;

  const char* JSON_FILE= argv[1];
  std::string robot_name = argv[2];
  
  const int BufferLength = 102400;
  char readBuffer[BufferLength] = {0,};
  if (false == ReadFromFile(JSON_FILE, readBuffer, BufferLength)) 
      return 0;
  std::string config_doc = readBuffer;
  Json::Value rootr;
  Json::Reader reader;
  bool parsingSuccessful = reader.parse(config_doc,rootr);
  if ( !parsingSuccessful ) { 
    std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
    return 0; 
  }
  std::cout << rootr[robot_name]["joint_names"]<<endl;
  for(int i = 0;i<jointnum;i++){
    joint_names.push_back(rootr[robot_name]["joint_names"][i].asString());
    cout<<joint_names.at(i)<<endl;
  }

  std::cout << rootr["camera"]["BaseToCamera"]<<endl;
  for(int i= 0;i<4;i++)
    for(int j=0;j<4;j++)
      TBaseToCamera(i,j) = rootr["camera"]["BaseToCamera"][i][j].asFloat();
  std::cout<<TBaseToCamera<<std::endl;
  std::cout<<tf<<std::endl;
   voxel_side_length= rootr["camera"]["voxel_size"].asFloat();
   coll_threshold = rootr["camera"]["collision_threshold"].asInt();
  urdf_name = rootr[robot_name]["urdf_location"].asString();
  colilsion_urdf_name = rootr["robot_collision"]["urdf_location"].asString();
  point_topic_name = rootr["camera"]["topic_name"].asString();
  base_link_name =rootr[robot_name]["base_link_name"].asString();
  tcp_link_name =rootr[robot_name]["tcp_link_name"].asString();


  jointnum = rootr[robot_name]["JOINTNUM"].asInt();
  base_x=rootr[robot_name]["base_position"][0].asFloat();
  base_y=rootr[robot_name]["base_position"][1].asFloat();
  base_z=rootr[robot_name]["base_position"][2].asFloat();




  KDL::JntArray q_min_(jointnum);
  KDL::JntArray q_max_(jointnum);
  KDL::JntArray joint_states_(jointnum);

  for(int j = 0;j<jointnum;j++){
      q_min_(j) = rootr[robot_name]["lower_limit"][j].asFloat();
      q_max_(j) = rootr[robot_name]["upper_limit"][j].asFloat();
      joint_states_(j)=rootr[robot_name]["joint_init"][j].asFloat();
      cout<<"qmin : "<<q_min_.data(j)<<endl;
      cout<<"qmax : "<<rootr[robot_name]["upper_limit"][j]<<endl;
         
  }
  q_min = q_min_;
  q_max = q_max_;
  joint_states = joint_states_;
  prev_joint_states = joint_states;






  usleep(100);

  signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

  icl_core::logging::initialize(argc, argv);

  PERF_MON_INITIALIZE(100, 1000);
  PERF_MON_ENABLE("planning");

  // construct the state space we are planning in
  auto space(std::make_shared<ob::RealVectorStateSpace>(jointnum));
  //We then set the bounds for the R3 component of this state space:
  ob::RealVectorBounds bounds(jointnum);
  for(int j = 0;j<jointnum;j++){
      bounds.setLow(j,q_min(j));
      bounds.setHigh(j,q_max(j));
  }
  space->setBounds(bounds);

  //Create an instance of ompl::base::SpaceInformation for the state space
  auto si(std::make_shared<ob::SpaceInformation>(space));
  //Set the state validity checker
  std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));
  si->setStateValidityChecker(my_class_ptr->getptr());
  si->setMotionValidator(my_class_ptr->getptr());
  si->setup();




  my_class_ptr->doVis();


  thread t1{&GvlOmplPlannerHelper::rosIter ,my_class_ptr};  
  thread t2{&GvlOmplPlannerHelper::dcpIter ,my_class_ptr};  


  while(1){
        usleep(3000000);
  }
//----------------------------------------------------//
    t1.join();
    t2.join();
    return 1;
}
