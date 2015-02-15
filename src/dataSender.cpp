#include "dataSender.h"

//DataSender::DataSender(ros::NodeHandle & n, int inputSize=5, int outputSize=3)
DataSender::DataSender(ros::NodeHandle & n, std::string dataf)
{
	nh_ = n;
	data_pub = nh_.advertise<genericNN::NeuralActivity>("data", 1);
	controlTimer = nh_.createTimer(ros::Duration(1.0), &DataSender::timerCallback, this);
	std::ifstream dataFile;
	if(dataf.empty()){ std::cout << "No data model given ; generating uniform data." << std::endl; dataf = "default.txt"; }
	std::cout << "Data file : " << dataf << std::endl;
	dataFile.open(dataf.c_str());
	if( !dataFile.is_open() ){ std::cerr << "File not open"<< std::endl; abort(); }

	dataFile >> nbGenFunc >> mixtureOfFunc;
	std::cout << "Metadata: " << nbGenFunc << " " << mixtureOfFunc << std::endl;
	for(int function=0 ; function < nbGenFunc ; function++)
	{
		dimNb=1;
		dataFile >> functionType >> dimNb;
		std::cout << "type of function : " << functionType << " " << dimNb << std::endl;
		std::vector<float> mus, sigmas;
		for(int field=0 ; field < dimNb ; field++)
		{
			float mu=0.0, sigma=1.0;
			dataFile >> mu >> sigma;
			std::cout << "mean : " << mu << " variance : " << sigma << std::endl;
		}
		means.push_back(mus);
		variances.push_back(sigmas);
		std::cout << "sizes : " << means.size() << " " << variances.size() << std::endl;
	}
	dataFile.close();
}
DataSender::~DataSender(){}

void DataSender::timerCallback(const ros::TimerEvent &)
{
	genericNN::NeuralActivity dataToSend;
	
/*	for(int i=0 ; i < output.size() ; i++)
	{
		std::cout << " | " <<  output.at(i) << " "  << std::endl;
	}
	std::cout << std::endl;
*/

	for(int dim=0 ; dim < dimNb ; dim++)
	{
//		float gaussianvalue = 1.0/(sqrt(2*PI)*variances.at(dim))*exp(-0.5*pow((rand() - means.at(dim))/variances.at(dim),2));
	//	std::cout << "val : " << gaussianvalue << " " << std::flush;
	}
	dataToSend.activityLevels.push_back(0.4);
	dataToSend.activityLevels.push_back(0.2);
	dataToSend.activityLevels.push_back(0.9);
	data_pub.publish(dataToSend);
}

bool DataSender::run()
{
        while(nh_.ok()){ ros::spinOnce(); }
        return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "datasender");
	ros::NodeHandle nh;

	int datasize=0, opt;
	std::string datafile;
	
	const struct option availableOptions[] = {
//	{"inputsize", optional_argument, NULL, 'i'},
//	{"outputsize", optional_argument, NULL, 'o'},
	{"datafile", optional_argument, NULL, 'f'},
	{0,0,0,0}
	};

	while( (opt = getopt_long(argc, argv, "fi:o:h", availableOptions, NULL)) != -1)
	switch(opt)
	{
		case 'f':
			datafile = argv[optind];
		break;
	/*	case 'i':
			datasize = atoi(argv[optind]);
			//insize = atoi(argv[optind]);
		break;
		case 'o':
			outsize = atoi(argv[optind]);
		break;
	*/
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
