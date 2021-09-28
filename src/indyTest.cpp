
#include "../indydcp/IndyDCPConnector.h"
#include <iostream>
#include <signal.h>
#include <time.h>

using namespace NRMKIndy::Service::DCP;
using namespace std;
IndyDCPConnector connector("192.168.0.7", ROBOT_INDY7);;
void WaitFinish(IndyDCPConnector& connector) {
    // Wait for motion finishing
    bool fin = false;
    do {
            sleep(0.5);
            connector.isMoveFinished(fin); // check if motion finished
    } while (!fin);
}
void ctrlchandler(int)
{
  exit(EXIT_SUCCESS);
}

void killhandler(int)
{
  exit(EXIT_SUCCESS);
}


int main(void){
  clock_t start, end;
  double result=0;
  connector.connect();
  bool ready;
  connector.isRobotReady(ready);
  std::cout<<"robot ready : "<<ready<<std::endl;
  if (ready) {
        cout << "---------------------Indy7 Robot is ready-------------------" << endl;
        bool ishome;
        connector.isHome(ishome);
        if (!ishome){
        	connector.moveJointHome();
       		WaitFinish(connector);	

        } else{
        	connector.moveJointZero();
       		WaitFinish(connector);	
	       }
	while(1){
      start = clock();
      double jointValue[6];
		  connector.getJointPosition(jointValue);
      cout<<"jointValue : "<< jointValue[0]<<","<< jointValue[1]<<","<< jointValue[2]<<","<< jointValue[3]<<","<< jointValue[4]<<","<< jointValue[5]<<endl;
      end = clock();
      result = (double)(end - start);
      cout<<result<<"ms"<<endl;
	}

  }
}

