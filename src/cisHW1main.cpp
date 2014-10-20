
// Library includes
#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <sstream>
#include <fstream>
#include <iostream>

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

    try
    {
      po::store(po::command_line_parser(argc, argv).options(allOptions).run(), vmap);
      po::notify(vmap);
    }
    catch (std::exception& e)
    {
      std::cerr << "[Error] " << BOOST_CURRENT_FUNCTION << std::endl
      << "    " << e.what() << std::endl
      << std::endl
      << allOptions << std::endl;
      return false;
    }
  
    if (vmap.count(CLO_GET_ARG_STR2(CLO_HELP)) || argc < 2)
    {
      std::cout << allOptions << std::endl;
      return false;
    }
  
	parseResponseFiles(vmap,allOptions);
	
	po::readOption(vmap,"inputPointCloudFile",algorithmParams.inputPointCloudFile1,optional);
	
    if (!algorithmParams.inputPointCloudFile1.empty() && !boost::filesystem::exists(algorithmParams.inputPointCloudFile1) ) {
      throw std::runtime_error("File "+algorithmParams.inputPointCloudFile1 + " does not exist!");
    }
	
    return false;
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
	
	std::stringstream ss;
	ss << std::ifstream( ap.inputPointCloudFile1.c_str() ).rdbuf();
    
	csvCIS_pointCloudData firstPointCloud = parseCSV_CIS_pointCloud(ss.str(),true);
	
	return 0;
}
