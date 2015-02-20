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

	std::cout << "Sizes : " << outputWidth << " " << outputHeight << std::endl;

}
Image2NN::~Image2NN(){}

void Image2NN::imageCallback(const sensor_msgs::Image & msg)
{
	genericNN::NeuralActivity dataToSend;
	sensor_msgs::Image tempImage;
	
//	std::cout << "callback, image data : " << msg.header.frame_id << " size: " << msg.width << " " << msg.height << " " << msg.width * msg.height << " " << msg.encoding << " " << msg.data.size() << " divised : " << msg.width / outputWidth << " "  << msg.height / outputHeight << " " << msg.step << std::endl;

	int buffer;
	float mean_pix=0;

	int goalWidth=msg.width / outputWidth, goalHeight=msg.height / outputHeight;
	tempImage.header = msg.header;
	tempImage.width = msg.width;
	tempImage.height = msg.height;
	tempImage.step = msg.step / 3;
	tempImage.encoding = "mono8";
	tempImage.is_bigendian = msg.is_bigendian;
	tempImage.data.resize(tempImage.width * tempImage.height);

	// luminosity method for converting rgb to grayscale :
	for(int j = 0 ; j < tempImage.height ; j++){ for(int i = 0 ; i < tempImage.width ; i++){ tempImage.data[(j*tempImage.width+i)] = 0.21 * msg.data[3*(j*tempImage.width+i)] + 0.72 * msg.data[3*(j*tempImage.width+i)+1] + 0.07 * msg.data[3*(j*tempImage.width+i)+2]; } }

	// Downsampling to parameter size
//	std::cout << "Size : " << outputWidth << " " << outputHeight << std::endl;
	for(int k = 0 ; k < outputHeight ; k++)
	{
		for(int l = 0 ; l < outputWidth ; l++)
		{
			for(int j = k * tempImage.height / outputHeight ; j < (k+1)*tempImage.height / outputHeight ; j++){ for(int i = l * tempImage.width / outputWidth ; i < (l+1)*tempImage.width / outputWidth ; i++){ mean_pix += tempImage.data[(j*tempImage.width+i)]; } }

			mean_pix /= tempImage.width / outputWidth  * tempImage.height / outputHeight;  
			for(int j = k * tempImage.height / outputHeight ; j < (k+1)*tempImage.height / outputHeight ; j++){ for(int i = l * tempImage.width / outputWidth ; i < (l+1)*tempImage.width / outputWidth ; i++){ tempImage.data[(j*tempImage.width+i)] = mean_pix; } }
		}
	}


	std::vector<float> denom(tempImage.data.size(), 255.0);
//	dataToSend.activityLevels.assign(tempImage.data.begin(), tempImage.data.end());
	dataToSend.activityLevels.resize(tempImage.data.size());
	std::transform(tempImage.data.begin(), tempImage.data.end(), denom.begin(), dataToSend.activityLevels.begin(), std::divides<float>());
	imageNN_pub.publish(dataToSend);
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
