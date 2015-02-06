#include "dataSender.h"

//DataSender::DataSender(ros::NodeHandle & n, int inputSize=5, int outputSize=3)
DataSender::DataSender(ros::NodeHandle & n, std::string dataf)
{
	nh_ = n;
	data_pub = nh_.advertise<genericNN::NeuralActivity>("/data", 1);
//	controlTimer = nh_.createTimer(ros::Duration(1.0), &DataSender::timerCallback, this);
	std::ifstream dataFile;
	if(dataf.empty()){ std::cout << "No data model given ; generating uniform data." << std::endl; dataf = "default.txt"; }
	std::cout << "Data file : " << dataf << std::endl;
	dataFile.open(dataf.c_str());
	if( !dataFile.is_open() ){ std::cerr << "File not open"<< std::endl; abort(); }

	std::cout << dataFile << std::endl;
	dataFile.close();
}
DataSender::~DataSender(){}

void DataSender::timerCallback(const ros::TimerEvent &)
{
	for(int i=0 ; i < output.size() ; i++)
	{
		std::cout << " | " <<  output.at(i) << " "  << std::endl;
	}
	std::cout << std::endl;

}

bool DataSender::run()
{
        while(nh_.ok()){ ros::spinOnce(); }
        return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "neuralnetwork");
	ros::NodeHandle nh;

	int datasize=0, opt;
	std::string datafile;
	
	const struct option availableOptions[] = {
	{"inputsize", optional_argument, NULL, 'i'},
	{"outputsize", optional_argument, NULL, 'o'},
	{"datafile", optional_argument, NULL, 'f'},
	{0,0,0,0}
	};

	while( (opt = getopt_long(argc, argv, "fi:o:h", availableOptions, NULL)) != -1)
	switch(opt)
	{
		case 'f':
			datafile = argv[optind];
		break;
		case 'i':
			datasize = atoi(argv[optind]);
			//insize = atoi(argv[optind]);
		break;
		case 'o':
//			outsize = atoi(argv[optind]);
		break;
		default :
			std::cout << "Unknown argument !" << std::endl;
			exit(EXIT_FAILURE);
		break;
	}

	DataSender sender(nh, datafile);
	sender.run();

	std::cout << "Done." << std::endl;
	return EXIT_SUCCESS;
}
