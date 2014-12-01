#ifndef _PARSE_HPP_
#define _PARSE_HPP_

// system includes
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <ostream>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

// project includes
#include "matrixOperations.hpp"

template<typename T>
void Print (const std::vector<T>& v, bool newline = true, std::string description = std::string("")){
    if(!description.empty()) std::cout << description;
    if(newline) std::cout << "\n";
    
    for (int i=0; i<v.size();i++){
        std::cout << v[i] << std::endl;
        if(newline) std::cout << "\n"; // this will print all the contents of *features*
    }
}

/// print std::vector<std::string> for debugging
void printStringVector(std::vector<std::string> sv, bool newline = true, std::string description = std::string("")){
    if(!description.empty()) std::cout << description;
    if(newline) std::cout << "\n";
    
    for(std::vector<std::string>::const_iterator i = sv.begin(); i != sv.end(); ++i) {
        // process i
        std::cout << *i << " ";
        if(newline) std::cout << "\n"; // this will print all the contents of *features*
    }
}

/// trim whitespace on the ends of a string and convert the text to a double
double trimAndConvertToDouble(std::string str, bool debug = false){
    
    // start by removing spaces and tabs
    boost::trim(str);
    // debug output
    if(debug) std::cout << "afterTrim:" << str << "\n";
    // convert string to double
    return boost::lexical_cast<double>(str);
}


Eigen::VectorXd readPointString(const std::string& pointString, bool debug = false){
    std::string mutablePointString = pointString;
    // start by removing spaces and tabs from the ends
    boost::trim(mutablePointString);
    // there is data here, load the point in
    std::vector<std::string> pointStrings;
    boost::split( pointStrings, mutablePointString, boost::is_any_of(" \t,"), boost::token_compress_on);
    
    if(debug) printStringVector(pointStrings,true, "point:");
	
	Eigen::VectorXd point(pointStrings.size());
	
	for(int i = 0; i < pointStrings.size(); ++i){
		point(i) = trimAndConvertToDouble(pointStrings[i]);
	}
    
    return point;
}




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
        throw std::runtime_error("File "+ dataFilePath + " does not exist!");
    }
    
}


#endif