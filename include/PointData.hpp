#ifndef _POINT_DATA_HPP_
#define _POINT_DATA_HPP_

// library includes
#include <boost/filesystem.hpp>
#include <sstream>
#include <fstream>
#include <iostream>

// project includes
#include "hwDataConstants.hpp"
#include "parseCSV_CIS_pointCloud.hpp"

struct AlgorithmData {
	csvCIS_pointCloudData calbody;
	csvCIS_pointCloudData calreadings;
	csvCIS_pointCloudData empivot;
	csvCIS_pointCloudData optpivot;
	csvCIS_pointCloudData output1;
};

struct DataSource {
    // filename prefix that goes in front of the filename string
    std::string  filenamePrefix;
    // full paths to the files
    std::string  calbodyPath;
    std::string  calreadingsPath;
    std::string  empivotPath;
    std::string  optpivotPath;
    std::string  output1Path;
	std::string  ct_fiducials;
	std::string  em_fiducials;
	std::string  em_nav      ;
	std::string  output2     ;
	
};

struct ParsedCommandLineCommands {
    bool         debug;
    std::vector<DataSource> dataSources;
};

/// The user may specify an exact path for a data source, or simply the folder path and filename prefix.
/// This funciton will detect which option and select the correctly assembled files.
///
/// @throws std::runtime_error if none of the options correspond to an actual existing file.
void assemblePathIfFullPathNotSupplied(std::string dataFolderPath, std::string dataFilenamePrefix, std::string suffix, std::string& dataFilePath,bool requiredFile = true){
	
	if(boost::filesystem::exists(dataFilePath)) return;
	
	if (dataFilePath.empty() || !boost::filesystem::exists(dataFilePath) ) {
		dataFilePath = dataFolderPath+dataFilenamePrefix+suffix;
    }
    
    if(!boost::filesystem::exists(dataFilePath) && !requiredFile){
        dataFilePath = "";
        return;
    }
    
	if (dataFilePath.empty() || !boost::filesystem::exists(dataFilePath) ) {
      throw std::runtime_error("File "+dataFilePath + " does not exist!");
    }
	
}


AlgorithmData assembleHW1AlgorithmData(std::string datafolderpath, std::string dataFilenamePrefix, bool debug = false){
    DataSource pclp;
    const bool required = true;
    // optional == not required == false
    const bool optional = false;
    // check if the user supplied a full path, if not assemble a path
    // from the default paths and the defualt prefix/suffix combos
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_calbody       ,pclp.calbodyPath,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_calreadings   ,pclp.calreadingsPath,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_empivot       ,pclp.empivotPath,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_optpivot      ,pclp.optpivotPath,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_output1       ,pclp.output1Path,optional);
    
    
    AlgorithmData ad;
    loadPointCloudFromFile(pclp.calbodyPath       ,ad.calbody                    ,debug);
    loadPointCloudFromFile(pclp.calreadingsPath   ,ad.calreadings                ,debug);
    loadPointCloudFromFile(pclp.empivotPath       ,ad.empivot                    ,debug);
    loadPointCloudFromFile(pclp.optpivotPath      ,ad.optpivot                   ,debug);
    loadPointCloudFromFile(pclp.output1Path       ,ad.output1                    ,debug);
    
    return ad;
    
}

#endif // _POINT_DATA_HPP_