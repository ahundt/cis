
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

struct AlgorithmData {
	csvCIS_pointCloudData calbody;
	csvCIS_pointCloudData calreadings;
	csvCIS_pointCloudData empivot;
	csvCIS_pointCloudData optPivot;
	csvCIS_pointCloudData output1;
};

struct AlgorithmParams {
	std::string  calbodyPath;
	std::string  calreadingsPath;
	std::string  empivotPath;
	std::string  optpivotPath;
	std::string  output1Path;
};

void assmblePathIfFullPathNotSupplied(std::string dataFolderPath, std::string dataFilenamePrefix, std::string suffix, std::string& dataFilePath){
	
	if(boost::filesystem::exists(dataFilePath)) return;
	
	if (dataFilePath.empty() || !boost::filesystem::exists(dataFilePath) ) {
		dataFilePath = dataFolderPath+dataFilenamePrefix+suffix;
    }
	
	if (dataFilePath.empty() || !boost::filesystem::exists(dataFilePath) ) {
      throw std::runtime_error("File "+dataFilePath + " does not exist!");
    }
	
}

bool readCommandLine(int argc, char* argv[], AlgorithmParams & algorithmParams){

  
    static const bool optional = false; // false means parameters are not required, and therefore optional
    static const bool required = true;  // true means parameters are required
	

  
    po::options_description generalOptions("General Options");
    generalOptions.add_options()
    ("responseFile", po::value<std::string>(), "File containing additional command line parameters")
    CLO_HELP
    ;
	

    po::options_description algorithmOptions("Algorithm Options");
    std::string currentPath(boost::filesystem::path( boost::filesystem::current_path() ).string());
  
    algorithmOptions.add_options()
		    ("dataFolderPath"                ,po::value<std::string>()->default_value(currentPath)       ,"folder containing data files, defaults to current working directory"   ) 
		  	("dataFilenamePrefix"            ,po::value<std::string>()->default_value("pa1-debug-a"     ),"constant prefix of data filename path"   )
		  	("dataFileNameSuffix_calbody"    ,po::value<std::string>()->default_value("-calbody.txt"    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_calreadings",po::value<std::string>()->default_value("-calreadings.txt"),"suffix of data filename path"   )
		  	("dataFileNameSuffix_empivot"    ,po::value<std::string>()->default_value("-empivot.txt"    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_optpivot"   ,po::value<std::string>()->default_value("-optpivot.txt"   ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_output1"    ,po::value<std::string>()->default_value("-output1.txt"    ),"suffix of data filename path"   )
		  	("calbodyPath"    ,po::value<std::string>() )
		  	("calreadingsPath",po::value<std::string>() )
		  	("empivotPath"    ,po::value<std::string>() )
		  	("optpivotPath"   ,po::value<std::string>() )
		  	("output1Path"    ,po::value<std::string>() )
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
	
	std::string  dataFolderPath
				,dataFilenamePrefix
				,dataFileNameSuffix_calbody 
				,dataFileNameSuffix_calreadings
				,dataFileNameSuffix_empivot    
				,dataFileNameSuffix_optpivot   
				,dataFileNameSuffix_output1;   
		
	po::readOption(vmap,"dataFolderPath",dataFolderPath,optional);
	po::readOption(vmap,"dataFilenamePrefix",dataFilenamePrefix,optional);
	po::readOption(vmap, "dataFileNameSuffix_calbody"      ,dataFileNameSuffix_calbody         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calreadings"  ,dataFileNameSuffix_calreadings     ,optional);
	po::readOption(vmap, "dataFileNameSuffix_empivot"      ,dataFileNameSuffix_empivot         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_optpivot"     ,dataFileNameSuffix_optpivot        ,optional);
	po::readOption(vmap, "dataFileNameSuffix_output1"      ,dataFileNameSuffix_output1         ,optional);
	                                                                                           
    
    po::readOption(vmap,"calbodyPath",algorithmParams.calbodyPath,optional);
    po::readOption(vmap,"calreadingsPath",algorithmParams.calreadingsPath,optional);
	po::readOption(vmap,"empivotPath",algorithmParams.empivotPath,optional);
	po::readOption(vmap,"optpivotPath",algorithmParams.optpivotPath,optional);
	po::readOption(vmap,"output1Path",algorithmParams.output1Path,optional);
	
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_calbody       ,algorithmParams.calbodyPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_calreadings   ,algorithmParams.calreadingsPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_empivot       ,algorithmParams.empivotPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_optpivot      ,algorithmParams.optpivotPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_output1       ,algorithmParams.output1Path);
	
	
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
	ss << std::ifstream( ap.calbodyPath ).rdbuf();
    
	csvCIS_pointCloudData firstPointCloud = parseCSV_CIS_pointCloud(ss.str(),true);
	
	return 0;
}
