#include "image2NN.h"

//Image2NN::Image2NN(ros::NodeHandle & n, int inputSize=5, int outputSize=3)
Image2NN::Image2NN(ros::NodeHandle & n, int NN_w=80, int NN_h=60)
{
	nh_ = n;
	imageNN_pub = nh_.advertise<genericNN::NeuralActivity>("mappingNN", 1);
	imageProc_pub = nh_.advertise<sensor_msgs::Image>("imgproc", 1);
	image_sub = nh_.subscribe("inputImage", 1, &Image2NN::imageCallback, this);
	outputWidth = NN_w;
	outputHeight = NN_h;

//	previousImg;

}
Image2NN::~Image2NN(){}

void Image2NN::imageCallback(const sensor_msgs::Image & msg)
{
	genericNN::NeuralActivity dataToSend;
	sensor_msgs::Image tempImage;
	
	std::cout << "callback, image fata : " << msg.header.frame_id << " size: " << msg.width << " " << msg.height << " " << msg.width * msg.height << " " << msg.encoding << " " << msg.data.size() << " divised : " << msg.width / outputWidth << " "  << msg.height / outputHeight << std::endl;

	int buffer;
	float mean_R = 0, mean_G = 0, mean_B = 0;

	int goalWidth=msg.width / outputWidth, goalHeight=msg.height / outputHeight;
	tempImage = msg;
	//tempImage.encoding = "rgb8";
	//tempImage.encoding = "bgr8";
	/*tempImage.header = msg.header;
	tempImage.height = msg.height / 3;
	tempImage.width = msg.width;

	tempImage.encoding = msg.encoding;
	tempImage.is_bigendian = msg.is_bigendian;
	tempImage.step = msg.step;

	tempImage.data = msg.data;
*/

	for(int j = 0 ; j < tempImage.height ; j++)
	{
		for(int i = 0 ; i < tempImage.width ; i++)
		{
			tempImage.data[3*(j*tempImage.width+i)] = msg.data[3*(j*tempImage.width+i)] ;
			tempImage.data[3*(j*tempImage.width+i)+1] = msg.data[3*(j*tempImage.width+i)+1];
			tempImage.data[3*(j*tempImage.width+i)+2] = msg.data[3*(j*tempImage.width+i)+2];
		}
	}

	for(int k = 0 ; k < outputHeight ; k++)
	{
		for(int l = 0 ; l < outputWidth ; l++)
		{
			for(int j = k * tempImage.height / outputHeight ; j < (k+1)*tempImage.height / outputHeight ; j++)
			{
				for(int i = l * tempImage.width / outputWidth ; i < (l+1)*tempImage.width / outputWidth ; i++)
				{
					mean_B += tempImage.data[3*(j*tempImage.width+i)];
					mean_G += tempImage.data[3*(j*tempImage.width+i)+1];
					mean_R += tempImage.data[3*(j*tempImage.width+i)+2];

				}
			}

			mean_B /= tempImage.width / outputWidth  * tempImage.height / outputHeight;  
			mean_G /= tempImage.width / outputWidth  * tempImage.height / outputHeight;  
			mean_R /= tempImage.width / outputWidth  * tempImage.height / outputHeight;  
			for(int j = k * tempImage.height / outputHeight ; j < (k+1)*tempImage.height / outputHeight ; j++)
			{
				for(int i = l * tempImage.width / outputWidth ; i < (l+1)*tempImage.width / outputWidth ; i++)
				{
					tempImage.data[3*(j*tempImage.width+i)] = mean_B;
					tempImage.data[3*(j*tempImage.width+i)+1] = mean_G;
					tempImage.data[3*(j*tempImage.width+i)+2] = mean_R;
				}
			}
		}
	}


//	dataToSend.activityLevels.push_back(0.4);
//	dataToSend.activityLevels.push_back(0.2);
//	dataToSend.activityLevels.push_back(0.9);
//	imageNN_pub.publish(dataToSend);
	imageProc_pub.publish(tempImage);
}

bool Image2NN::run()
{
        while(nh_.ok()){ ros::spinOnce(); }
        return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "datasender");
	ros::NodeHandle nh;

	int datasize=0, opt, W=80, H=60;
	std::string datafile;
	
	const struct option availableOptions[] = {
	{"datafile", optional_argument, NULL, 'f'},
	{"outwidth", optional_argument, NULL, 'W'},
	{"outheight", optional_argument, NULL, 'H'},
	{0,0,0,0}
	};

	while( (opt = getopt_long(argc, argv, "fW:H:h", availableOptions, NULL)) != -1)
	switch(opt)
	{
		case 'f':
			datafile = argv[optind];
		break;
		case 'W':
			W = atoi(argv[optind]);
		break;
		case 'H':
			H = atoi(argv[optind]);
		break;
		default :
			std::cout << "Unknown argument !" << std::endl;
			exit(EXIT_FAILURE);
		break;
	}

	//Image2NN sender(nh, datafile);
	Image2NN sender(nh, W, H);
	sender.run();

	std::cout << "Done." << std::endl;
	return EXIT_SUCCESS;
}
