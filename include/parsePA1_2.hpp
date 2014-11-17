#ifndef _POINT_DATA_HPP_
#define _POINT_DATA_HPP_

// library includes
#include <boost/filesystem.hpp>
#include <sstream>
#include <fstream>
#include <iostream>

// project includes
#include "PA1_2_DataConstants.hpp"
#include "parseCSV_CIS_pointCloud.hpp"

/// Source data after loading into memory
struct AlgorithmData {
	csvCIS_pointCloudData calbody     ;
	csvCIS_pointCloudData calreadings ;
	csvCIS_pointCloudData empivot     ;
	csvCIS_pointCloudData optpivot    ;
	csvCIS_pointCloudData output1     ;
	csvCIS_pointCloudData ct_fiducials;
	csvCIS_pointCloudData em_fiducials;
	csvCIS_pointCloudData em_nav      ;
	csvCIS_pointCloudData output2     ;
};

/// Paths to the files containing source data
struct DataSource {
    // filename prefix that goes in front of the filename string
    std::string  filenamePrefix   ;
    // full paths to the files    
    std::string  calbodyPath      ;
    std::string  calreadingsPath  ;
    std::string  empivotPath      ;
    std::string  optpivotPath     ;
    std::string  output1Path      ;
	std::string  ct_fiducialsPath ;
	std::string  em_fiducialsPath ;
	std::string  em_navPath       ;
	std::string  output2Path      ;
	
};

struct ParsedCommandLineCommands {
    bool                    debug;
    bool                    debugParser;
    bool                    threads;
    std::string             outputDataFolderPath;
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

/// Initialize an AlgorithmData object utilizing the data files stored in the specified path and file prefix,
/// utilizing the hard coded default filename suffixes. Used for unit testing.
AlgorithmData initAlgorithmData(std::string datafolderpath, std::string dataFilenamePrefix, bool debug = false){
    DataSource pclp;
    const bool required = true;
    // optional == not required == false
    const bool optional = false;
    // check if the user supplied a full path, if not assemble a path
    // from the default paths and the defualt prefix/suffix combos
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_calbody       ,pclp.calbodyPath      ,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_calreadings   ,pclp.calreadingsPath  ,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_empivot       ,pclp.empivotPath      ,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_optpivot      ,pclp.optpivotPath     ,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_output1       ,pclp.output1Path      ,optional);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_ct_fiducials  ,pclp.ct_fiducialsPath ,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_em_fiducials  ,pclp.em_fiducialsPath ,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_em_nav        ,pclp.em_navPath       ,required);
    assemblePathIfFullPathNotSupplied(datafolderpath,dataFilenamePrefix,dataFileNameSuffix_output2       ,pclp.output2Path      ,optional);
    
    
    AlgorithmData ad;
    loadPointCloudFromFile(pclp.calbodyPath       ,ad.calbody                    ,debug);
    loadPointCloudFromFile(pclp.calreadingsPath   ,ad.calreadings                ,debug);
    loadPointCloudFromFile(pclp.empivotPath       ,ad.empivot                    ,debug);
    loadPointCloudFromFile(pclp.optpivotPath      ,ad.optpivot                   ,debug);
    loadPointCloudFromFile(pclp.output1Path       ,ad.output1                    ,debug);
    loadPointCloudFromFile(pclp.ct_fiducialsPath  ,ad.ct_fiducials               ,debug);
    loadPointCloudFromFile(pclp.em_fiducialsPath  ,ad.em_fiducials               ,debug);
    loadPointCloudFromFile(pclp.em_navPath        ,ad.em_nav                     ,debug);
    loadPointCloudFromFile(pclp.output2Path       ,ad.output2                    ,debug);
    
    return ad;
    
}

#endif // _POINT_DATA_HPP_