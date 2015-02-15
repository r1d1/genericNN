/*
	@created : 8/02/2015
	@author : Erwan
	@description : image to neural activity data 
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
#include <sensor_msgs/Image.h>
#include "genericNN/NeuralActivity.h"

class Image2NN
{
	private :
		ros::NodeHandle nh_;
		ros::Publisher imageNN_pub;
		ros::Publisher imageProc_pub;
		ros::Subscriber image_sub;

		int outputWidth;
		int outputHeight;
		sensor_msgs::Image previousImg;
		// Parameters on how to send data :
//		int nbGenFunc;
//		int dimNb;
//		std::string mixtureOfFunc;

	public :
		//Image2NN(ros::NodeHandle & n, int insize, int outsize);
		Image2NN(ros::NodeHandle & n, int widthNN, int heightNN);
		~Image2NN();

		void imageCallback(const sensor_msgs::Image &);
		bool run();
};
