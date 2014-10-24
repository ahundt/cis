
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
#include "hornRegistration.hpp"


namespace po = boost::program_options;

struct AlgorithmData {
	csvCIS_pointCloudData calbody;
	csvCIS_pointCloudData calreadings;
	csvCIS_pointCloudData empivot;
	csvCIS_pointCloudData optpivot;
	csvCIS_pointCloudData output1;
};

struct ParsedCommandLineCommands {
	std::string  calbodyPath;
	std::string  calreadingsPath;
	std::string  empivotPath;
	std::string  optpivotPath;
	std::string  output1Path;
};

/// The user may specify an exact path for a data source, or simply the folder path and filename prefix.
/// This funciton will detect which option and select the correctly assembled files.
///
/// @throws std::runtime_error if none of the options correspond to an actual existing file.
void assmblePathIfFullPathNotSupplied(std::string dataFolderPath, std::string dataFilenamePrefix, std::string suffix, std::string& dataFilePath){
	
	if(boost::filesystem::exists(dataFilePath)) return;
	
	if (dataFilePath.empty() || !boost::filesystem::exists(dataFilePath) ) {
		dataFilePath = dataFolderPath+dataFilenamePrefix+suffix;
    }
	
	if (dataFilePath.empty() || !boost::filesystem::exists(dataFilePath) ) {
      throw std::runtime_error("File "+dataFilePath + " does not exist!");
    }
	
}

/// read the command line options from argc,argv and load them into the params object
bool readCommandLine(int argc, char* argv[], ParsedCommandLineCommands & pclp){

  
    static const bool optional = false; // false means parameters are not required, and therefore optional
    static const bool required = true;  // true means parameters are required
	

  
    po::options_description generalOptions("General Options");
    generalOptions.add_options()
    ("responseFile", po::value<std::string>(), "File containing additional command line parameters")
    CLO_HELP
    ;
	

    po::options_description algorithmOptions("Algorithm Options");
  
  
    // create algorithm command line options
    //algorithmOptions.add_options()
	// todo, add options for configuring algorithms
	
    po::options_description dataOptions("Data Options");
  

    std::string currentPath(boost::filesystem::path( boost::filesystem::current_path() ).string());
    // create algorithm command line options
    dataOptions.add_options()
		    ("dataFolderPath"                ,po::value<std::string>()->default_value(currentPath)       ,"folder containing data files, defaults to current working directory"   ) 
		  	("dataFilenamePrefix"            ,po::value<std::string>()->default_value("pa1-debug-a"     ),"constant prefix of data filename path"   )
		  	("dataFileNameSuffix_calbody"    ,po::value<std::string>()->default_value("-calbody.txt"    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_calreadings",po::value<std::string>()->default_value("-calreadings.txt"),"suffix of data filename path"   )
		  	("dataFileNameSuffix_empivot"    ,po::value<std::string>()->default_value("-empivot.txt"    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_optpivot"   ,po::value<std::string>()->default_value("-optpivot.txt"   ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_output1"    ,po::value<std::string>()->default_value("-output1.txt"    ),"suffix of data filename path"   )
		  	("calbodyPath"                   ,po::value<std::string>() , "full path to data txt file")
		  	("calreadingsPath"               ,po::value<std::string>() , "full path to data txt file")
		  	("empivotPath"                   ,po::value<std::string>() , "full path to data txt file")
		  	("optpivotPath"                  ,po::value<std::string>() , "full path to data txt file")
		  	("output1Path"                   ,po::value<std::string>() , "full path to data txt file")
			  ;

    po::options_description allOptions;
    allOptions.add(generalOptions).add(algorithmOptions).add(dataOptions);
  
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
	
	// initalize string params
	std::string  dataFolderPath
				,dataFilenamePrefix
				,dataFileNameSuffix_calbody 
				,dataFileNameSuffix_calreadings
				,dataFileNameSuffix_empivot    
				,dataFileNameSuffix_optpivot   
				,dataFileNameSuffix_output1;   
		
	// load up parameter values from the variable map
	po::readOption(vmap,"dataFolderPath"                   ,dataFolderPath                     ,optional);
	po::readOption(vmap,"dataFilenamePrefix"               ,dataFilenamePrefix                 ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calbody"      ,dataFileNameSuffix_calbody         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calreadings"  ,dataFileNameSuffix_calreadings     ,optional);
	po::readOption(vmap, "dataFileNameSuffix_empivot"      ,dataFileNameSuffix_empivot         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_optpivot"     ,dataFileNameSuffix_optpivot        ,optional);
	po::readOption(vmap, "dataFileNameSuffix_output1"      ,dataFileNameSuffix_output1         ,optional);
	                                                                                           
    
    po::readOption(vmap,"calbodyPath"                      ,pclp.calbodyPath        ,optional);
    po::readOption(vmap,"calreadingsPath"                  ,pclp.calreadingsPath    ,optional);
	po::readOption(vmap,"empivotPath"                      ,pclp.empivotPath        ,optional);
	po::readOption(vmap,"optpivotPath"                     ,pclp.optpivotPath       ,optional);
	po::readOption(vmap,"output1Path"                      ,pclp.output1Path        ,optional);
	
	
	// check if the user supplied a full path, if not assemble a path 
	// from the default paths and the defualt prefix/suffix combos
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_calbody       ,pclp.calbodyPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_calreadings   ,pclp.calreadingsPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_empivot       ,pclp.empivotPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_optpivot      ,pclp.optpivotPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,dataFilenamePrefix,dataFileNameSuffix_output1       ,pclp.output1Path);
	
	
    return false;
}

void loadPointCloud(std::string fullFilePath, csvCIS_pointCloudData& pointCloud){
    
    std::stringstream ss;
    ss << std::ifstream( fullFilePath ).rdbuf();
    
    pointCloud = parseCSV_CIS_pointCloud(ss.str(),true);
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
	ParsedCommandLineCommands pclp;
	readCommandLine(argc,argv,pclp);
    
    AlgorithmData ad;
    loadPointCloud(pclp.calbodyPath       ,ad.calbody                    );
    loadPointCloud(pclp.calreadingsPath   ,ad.calreadings                );
    loadPointCloud(pclp.empivotPath       ,ad.empivot                    );
    loadPointCloud(pclp.optpivotPath      ,ad.optpivot                   );
    loadPointCloud(pclp.output1Path       ,ad.output1                    );
	
    
    Eigen::Matrix4d F = hornRegistration(ad.calreadings.clouds[1],ad.calbody.clouds[1]);
    
    std::cout << F;
    
	return 0;
}
