#ifndef _POINT_DATA_HPP_
#define _POINT_DATA_HPP_

// library includes
#include <boost/filesystem.hpp>
#include <sstream>
#include <fstream>
#include <iostream>

// project includes
#include "parseCSV_CIS_pointCloud.hpp"

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

static const std::string dataFileNameSuffix_calbody("calbody.txt");
static const std::string dataFileNameSuffix_calreadings("calreadings.txt");
static const std::string dataFileNameSuffix_empivot("empivot.txt");
static const std::string dataFileNameSuffix_optpivot("optpivot.txt");
static const std::string dataFileNameSuffix_output1("output1.txt");

AlgorithmData assembleHW1AlgorithmData(std::string datafolderpath, std::string dataFilenamePrefix){
    ParsedCommandLineCommands pclp;
    // check if the user supplied a full path, if not assemble a path
    // from the default paths and the defualt prefix/suffix combos
    assmblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_calbody       ,pclp.calbodyPath);
    assmblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_calreadings   ,pclp.calreadingsPath);
    assmblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_empivot       ,pclp.empivotPath);
    assmblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_optpivot      ,pclp.optpivotPath);
    assmblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_output1       ,pclp.output1Path);
    
    
    AlgorithmData ad;
    loadPointCloudFromFile(pclp.calbodyPath       ,ad.calbody                    );
    loadPointCloudFromFile(pclp.calreadingsPath   ,ad.calreadings                );
    loadPointCloudFromFile(pclp.empivotPath       ,ad.empivot                    );
    loadPointCloudFromFile(pclp.optpivotPath      ,ad.optpivot                   );
    loadPointCloudFromFile(pclp.output1Path       ,ad.output1                    );
    
    return ad;
    
}

#endif // _POINT_DATA_HPP_