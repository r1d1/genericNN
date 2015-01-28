/*
	@created : 30/12/2014
	@author : Erwan
	@description : generic configurable neural network
*/

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <numeric>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "genericNN/NeuralActivity.h"

class NeuralNet
{
	private :
		ros::NodeHandle nh_;
		ros::Subscriber input_sub;
		ros::Subscriber learning_sub;
		ros::Publisher output_pub;
		ros::Publisher weights_pub;

		ros::Timer controlTimer;

		std::vector<float> input;
		std::vector<float> output;
		std::vector<std::vector<float> > weights;
		bool learning;
		int learningRule;

		float learningRate;

	public :
		NeuralNet(ros::NodeHandle & n, int insize, int outsize);
		~NeuralNet();

		bool run();
		void learn();
		void computeOutput();
		//void activityCallback(genericNN::NeuralActivity & msg);
		void activityCallback(genericNN::NeuralActivity msg);
		//void learningCallback(std_msgs::Bool & msg);
		void learningCallback(std_msgs::Bool msg);
		void timerCallback(const ros::TimerEvent &);

		float learningRule_Hebb(); // hebbian learning
		float learningRule_LMS(); // Least mean square

};
