
// Library includes
#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>

// Project includes
#include "parseCSV_CIS_pointCloud.hpp"
#include "parseCommandLineOptions.hpp"


namespace po = boost::program_options;

struct AlgorithmParams {
	std::string inputPointCloudFile1;
	std::string inputPointCloudFile2;
};

bool readCommandLine(int argc, char* argv[], AlgorithmParams & algorithmParams){

  
    static const bool optional = false; // false means parameters are not required, and therefore optional
    static const bool required = true;  // true means parameters are required
	

  
    po::options_description generalOptions("General Options");
    generalOptions.add_options()
    ("responseFile", po::value<std::string>(), "File containing additional command line parameters")
    CLO_HELP
    ;
	

    po::options_description algorithmOptions("Algorithm Options");
  
    algorithmOptions.add_options()
		  ("inputPointCloudFile",po::value<std::string>(),"PointCloudFile.txt")
		  ("inputPointCloudFile2",po::value<std::string>(),"PointCloudFile2")
			  ;

    po::options_description allOptions;
    allOptions.add(generalOptions).add(algorithmOptions);
  
    po::variables_map vmap;
	
	parseResponseFiles(vmap,allOptions);
	
	po::readOption(vmap,"inputPointCloudFile",algorithmParams.inputPointCloudFile1,optional);
	
}



/**************************************************************************/
/**
 * @brief Main function 
 * 
 * @param argc  Number of input arguments
 * @param argv  Pointer to input arguments
 * 
 * @return int 
 */
int main(int argc,char**argv) {
	AlgorithmParams ap;
	readCommandLine(argc,argv,ap);
	
	parseCSV_CIS_pointCloud(ap.inputPointCloudFile1);
	
	return 0;
}
