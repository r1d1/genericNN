/*
	@created : 3/02/2015
	@author : Erwan
	@description : data sender to neural network (genericNN) 
*/

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <numeric>
#include <getopt.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "genericNN/NeuralActivity.h"

class DataSender
{
	private :
		ros::NodeHandle nh_;
		ros::Publisher data_pub;

		ros::Timer controlTimer;

		std::vector<float> output;
		

	public :
		//DataSender(ros::NodeHandle & n, int insize, int outsize);
		DataSender(ros::NodeHandle & n, std::string datafile);
		~DataSender();

		bool run();
		void timerCallback(const ros::TimerEvent &);
};
