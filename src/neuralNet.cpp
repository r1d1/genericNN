#include "neuralNet.h"

NeuralNet::NeuralNet(ros::NodeHandle & n, int inputSize=5, int outputSize=3)
{
	nh_ = n;
	input_sub = nh_.subscribe("/inputNN", 1, &NeuralNet::activityCallback, this);
	learning_sub = nh_.subscribe("/learningNN", 1, &NeuralNet::learningCallback, this);
	output_pub = nh_.advertise<genericNN::NeuralActivity>("/outputNN",1);
	controlTimer = nh_.createTimer(ros::Duration(1.0), &NeuralNet::timerCallback, this);
	learning = false;
	
	input.assign(inputSize, 0.0);
	output.assign(outputSize, 0.0);
	weights.resize(outputSize);
	//weights.resize(inputSize);
	for(int i=0 ; i < outputSize ; i++){ weights[i].assign(inputSize, 1.0/(outputSize)); }
	//for(int i=0 ; i < inputSize ; i++){ weights[i].assign(outputSize, 0.0); }
	//for(int i=0 ; i < inputSize ; i++){ weights.at(i).resize(outputSize); }
}
NeuralNet::~NeuralNet(){}

void NeuralNet::learn(){}
void NeuralNet::computeOutput()
{
//	ROS_INFO("Computing network output !");
	for(int outindex=0 ; outindex < output.size() ; outindex++ )
	{
		//std::transform(input.begin(), input.end(), weights.begin(), output.begin(), std::multiplies<float>());
		std::cout << " data : " << input.size() << " " << weights[outindex].size() << std::endl;
		output[outindex]=float(std::inner_product(input.begin(), input.end(), weights[outindex].begin(), 0.0));
	}
}
void NeuralNet::activityCallback(genericNN::NeuralActivity msg)
{
	for(int neuron=0 ; neuron < input.size() ; neuron++){ input[neuron] = float(msg.activityLevels[neuron]); }
	std::cout << "input size : " << input.size() << std::endl;
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
		//for(int i=0 ; i < output.size() ; i++){ std::cout << output.at(i) << " " << std::flush; }
		//std::cout << std::endl;
		
		for(int i=0 ; i < output.size() ; i++)
		{
			for(int j=0 ; j < input.size() ; j++)
			{
				std::cout << weights[i].at(j) << " " << std::flush;
			}
			std::cout << " | " <<  output.at(i) << std::endl;
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
	
	NeuralNet network(nh);
	network.run();

	std::cout << "Done." << std::endl;
	return EXIT_SUCCESS;
}