#include <stdio.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/frames_io.hpp>
#include <jsoncpp/json/json.h>
#pragma comment(lib, "jsoncpp.lib")

#define JOINTNUM 6


using namespace std;
bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
void printCatesianKDLFrame(KDL::Frame frame,char* str ){
    std::cout<<"======="<<str<<"=======\n\n"<<endl;
    for(int i =0;i<4;i++){
        for(int j=0;j<4;j++)
            std::cout<<frame(i,j)<<"\t";
        std::cout<<"\n"<<std::endl;
    }
}


int  main(int argc, char **argv){

KDL::Tree my_tree;
KDL::Chain my_chain;

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


  std::string base_link_name =rootr[robot_name]["base_link_name"].asString();
  std::string tcp_link_name =rootr[robot_name]["tcp_link_name"].asString();


  int jointnum = rootr[robot_name]["JOINTNUM"].asInt();




  KDL::JntArray q_min(jointnum);
  KDL::JntArray q_max(jointnum);

  for(int j = 0;j<jointnum;j++){
      q_min(j) = rootr[robot_name]["lower_limit"][j].asFloat();
      q_max(j) = rootr[robot_name]["upper_limit"][j].asFloat();         
  }

  float target[7];
  target[0]=rootr[robot_name]["target"][0].asFloat();
  target[1]=rootr[robot_name]["target"][1].asFloat();
  target[2]=rootr[robot_name]["target"][2].asFloat();
  target[3]=rootr[robot_name]["target"][3].asFloat();
  target[4]=rootr[robot_name]["target"][4].asFloat();
  target[5]=rootr[robot_name]["target"][5].asFloat();
  target[6]=rootr[robot_name]["target"][6].asFloat();
  
   std::cout<<"target: "<<target[0]<<","<<target[1]<<","<<target[2]<<","<<target[3]<<"\n"<<target[4]<<","<<target[5]<<","<<target[6]<<endl;
	std::string urdf_name = rootr[robot_name]["urdf_location"].asString();
	if (!kdl_parser::treeFromFile(urdf_name, my_tree)){
            // LOGGING_INFO(Gpu_voxels,"Failed to construct kdl tree");
		 std::cout<<"Failed to construct kdl tree"<<std::endl;
    }
    std::cout<< "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl;

    std::cout<< "\n\nKDL Chain load : "<<base_link_name<<","<<tcp_link_name<<my_tree.getChain(base_link_name,tcp_link_name,my_chain) <<"\n"<< std::endl;

 	KDL::JntArray q_start(jointnum);  
    KDL::JntArray q_result(jointnum);  
     for(int i=0;i<jointnum;i++){
        q_start(i) = rootr[robot_name]["joint_init"][i].asFloat();
        q_result(i) = 0.0;
    }
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    KDL::Frame cartesian_pos;
    KDL::Frame cartesian_pos_result;
    
    KDL::Frame goal_pose(KDL::Rotation::Quaternion(target[0],target[1],target[2],target[3]),KDL::Vector(target[4],target[5],target[6]));

    fk_solver.JntToCart(q_start, cartesian_pos);
    printCatesianKDLFrame(cartesian_pos,"catesian pos start");
    
    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.01);



    bool ret = iksolver1.CartToJnt(q_start,goal_pose,q_result);
    std::cout<<"ik ret : "<<ret<<std::endl;
    std::cout<<"ik q : "<<q_result(0)<<","<<q_result(1)<<","<<q_result(2)
                        <<","<<q_result(3)<<","<<q_result(4)<<","
                        <<q_result(5)<<std::endl;

    fk_solver.JntToCart(q_result, cartesian_pos_result);

    printCatesianKDLFrame(goal_pose,"goal pos");

    printCatesianKDLFrame(cartesian_pos_result,"pos result");


return 0;
}
