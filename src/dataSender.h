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

#define PI 3.14159265359

class DataSender
{
	private :
		ros::NodeHandle nh_;
		ros::Publisher data_pub;

		ros::Timer controlTimer;

		std::vector<float> output;

		// Parameters on how to send data :
		int nbGenFunc;
		int dimNb;
		std::string mixtureOfFunc;
		// rythm of publication
		// variable --
		// this should be a more complex data structure array:
		std::string functionType;
		std::vector<std::vector<float> > means;
		std::vector<std::vector<float> > variances;
		

	public :
		//DataSender(ros::NodeHandle & n, int insize, int outsize);
		DataSender(ros::NodeHandle & n, std::string datafile);
		~DataSender();

		bool run();
		void timerCallback(const ros::TimerEvent &);
};
