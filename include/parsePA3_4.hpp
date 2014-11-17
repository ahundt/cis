#ifndef _PARSE_CSV_CIS_POINTCLOUD_HPP_
#define _PARSE_CSV_CIS_POINTCLOUD_HPP_

#include "parse.hpp"

struct cisMesh {
	Eigen::MatrixXd vertices;
	Eigen::MatrixXd vertexIndexTriangleIndex;
};

csvCIS_pointCloudData parseCSV_CIS_pointCloud(std::string csv, bool debug = false){
    csvCIS_pointCloudData outputData;
    if(csv.empty()) return outputData;
       
    std::vector<std::string> strs;
    
    // by default if no frame count is specified, there is only one frame
    int numFrames = 1;
    // split into separate strings by newlines
	boost::split(strs, csv, boost::is_any_of("\n"));
    
    if(debug) {
        //std::cout << "\n\n FULL FILE CONTENTS:\n\n" << csv;  // original file string
        std::cout << "\n\nNewly split vector of strings:\n\n"; // file string post split on newlines
        printStringVector(strs);
    }
    
    
	// parse first line differently
    std::vector<std::string>::iterator currentStringLineIterator = strs.begin();
	std::vector<std::string> firstLineStrings;
}



#endif // _PARSE_CSV_CIS_POINTCLOUD_HPP_