#include "neuralNet.h"

NeuralNet::NeuralNet(ros::NodeHandle & n, int inputSize=5, int outputSize=3)
{
	nh_ = n;
	input_sub = nh_.subscribe("inputNN", 1, &NeuralNet::activityCallback, this);
	desired_sub = nh_.subscribe("desiredOutputNN", 1, &NeuralNet::desiredCallback, this);
	learning_sub = nh_.subscribe("learningNN", 1, &NeuralNet::learningCallback, this);
	output_pub = nh_.advertise<genericNN::NeuralActivity>("outputNN",1);
	controlTimer = nh_.createTimer(ros::Duration(1.0), &NeuralNet::timerCallback, this);
	learning = false;
	inputReceived = desiredOutputReceived = false;
	learningRate = 0.05;

	input.assign(inputSize, 0.0);
	output.assign(outputSize, 0.0);
	desiredOutput.assign(outputSize, 0.0);
	weights.resize(outputSize);
	//weights.resize(inputSize);
	for(int i=0 ; i < outputSize ; i++){ weights[i].assign(inputSize, 1.0/(outputSize)); }
	//for(int i=0 ; i < inputSize ; i++){ weights[i].assign(outputSize, 0.0); }
	//for(int i=0 ; i < inputSize ; i++){ weights.at(i).resize(outputSize); }
}
NeuralNet::~NeuralNet(){}

void NeuralNet::learn()
{
	// Perceptron rule :
//	delta_W = alpha * x[j] * (d[i] - y_d[i]);
	if( inputReceived && desiredOutputReceived )
	{
		for(int j=0 ; j< output.size() ; j++)
		{
			for (int i=0 ; i < input.size() ; i++)
			{
				weights[j].at(i) += learningRate * input.at(i) * (desiredOutput.at(j) - output.at(j));
				std::cout << weights[j].at(i) << " " << std::flush;
			}
			std::cout << std::endl;
		}
		inputReceived = desiredOutputReceived = false;
	}
}

void NeuralNet::computeOutput()
{
//	ROS_INFO("Computing network output !");
	for(int outindex=0 ; outindex < output.size() ; outindex++ )
	{
		//std::transform(input.begin(), input.end(), weights.begin(), output.begin(), std::multiplies<float>());
		std::cout << " data : " << input.size() << " " << weights[outindex].size() << std::endl;
		output[outindex]=float(std::inner_product(input.begin(), input.end(), weights[outindex].begin(), 0.0));
	}
	genericNN::NeuralActivity output_msg;
	output_msg.activityLevels = output;
	output_pub.publish(output_msg);
}
void NeuralNet::activityCallback(genericNN::NeuralActivity msg)
{
	for(int neuron=0 ; neuron < input.size() ; neuron++){ input[neuron] = float(msg.activityLevels[neuron]); }
	inputReceived = true;
	std::cout << "input size : " << input.size() << std::endl;
}

void NeuralNet::desiredCallback(genericNN::NeuralActivity msg)
{
	for(int neuron=0 ; neuron < desiredOutput.size() ; neuron++){ desiredOutput[neuron] = float(msg.activityLevels[neuron]); }
	desiredOutputReceived = true;
	std::cout << "desired output size : " << desiredOutput.size() << std::endl;
}

void NeuralNet::learningCallback(std_msgs::Bool msg){ learning = msg.data; std::cout << ((learning )? "true" : "false") << std::endl; }
//void NeuralNet::activityCallback(genericNN::NeuralActivity & msg){}
/*NeuralNet::NeuralNet(){}
NeuralNet::NeuralNet(){}
*/
void NeuralNet::timerCallback(const ros::TimerEvent &)
{
	if(input.size() > 0)
	{
		for(int i=0 ; i < input.size() ; i++){ std::cout << input.at(i) << " " << std::flush; }
		std::cout << std::endl;
		computeOutput();
		if(learning){ learn(); }
		//for(int i=0 ; i < output.size() ; i++){ std::cout << output.at(i) << " " << std::flush; }
		//std::cout << std::endl;
		
		for(int i=0 ; i < output.size() ; i++)
		{
			/*for(int j=0 ; j < input.size() ; j++)
			{
				std::cout << weights[i].at(j) << " " << std::flush;
			}*/
			std::cout << " | " <<  output.at(i) << " " << desiredOutput.at(i) <<std::endl;
		}
		std::cout << std::endl;
	}
}

bool NeuralNet::run()
{
        while(nh_.ok()){ ros::spinOnce(); }
        return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "neuralnetwork");
	ros::NodeHandle nh;

	int insize=0, outsize=0, opt;
	
	const struct option availableOptions[] = {
	{"inputsize", optional_argument, NULL, 'i'},
	{"outputsize", optional_argument, NULL, 'o'},
	{0,0,0,0}
	};

	while( (opt = getopt_long(argc, argv, "i:o:h", availableOptions, NULL)) != -1)
	switch(opt)
	{
		case 'i':
			insize = atoi(argv[optind]);
		break;
		case 'o':
			outsize = atoi(argv[optind]);
		break;
		default :
			std::cout << "Unknown argument !" << std::endl;
			exit(EXIT_FAILURE);
		break;
	}

	NeuralNet network(nh, insize, outsize);
	network.run();

	std::cout << "Done." << std::endl;
	return EXIT_SUCCESS;
}
