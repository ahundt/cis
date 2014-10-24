#ifndef _PARSE_CSV_CIS_POINTCLOUD_HPP_
#define _PARSE_CSV_CIS_POINTCLOUD_HPP_

// system includes
#include <Eigen/Dense>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

// project includes
#include "matrixOperations.hpp"

/// print std::vector<std::string> for debugging
void printStringVector(std::vector<std::string> sv,std::string description = std::string("")){
    if(!description.empty()) std::cout << description << " \n";
    for(std::vector<std::string>::const_iterator i = sv.begin(); i != sv.end(); ++i) {
        // process i
        std::cout << *i << " \n"; // this will print all the contents of *features*
    }
}

/// trim whitespace on the ends of a string and convert the text to a double
double trimAndConvertToDouble(std::string str){
    
    // start by removing spaces and tabs
    boost::trim(str);
    // debug output
    // std::cout << "afterTrim:" << str << "\n";
    // convert string to double
    return boost::lexical_cast<double>(str);
}

/// Stores the parsed data from the csv text files
struct csvCIS_pointCloudData {
    
    /// filename of the file
	std::string title;
    
    /// the number of elements in each "cloud"
	std::vector<int> firstLine;
    
    int numFrames;
    
    /// vector containing each cloud
    std::vector<Eigen::MatrixXd> clouds;
};

/// parse the CIS HW1 point cloud string that was loaded directly from a txt file into a simpler struct
csvCIS_pointCloudData parseCSV_CIS_pointCloud(std::string csv, bool debug = false){
    csvCIS_pointCloudData outputData;
	std::vector<std::string> strs;
    // split into separate strings by newlines
	boost::split(strs, csv, boost::is_any_of("\n"));
    
    if(debug) {
        std::cout << csv << "\n\n FULL FILE CONTENTS PRINTED, now printing newly split vector of strings:\n\n";
        printStringVector(strs);
    }
	
    
	// parse first line differently
    std::vector<std::string>::iterator begin = strs.begin();
	std::vector<std::string> firstLineStrings;

	boost::split(firstLineStrings, *begin, boost::is_any_of(", "), boost::token_compress_on);
    int firstLineIndex = 0;
    for(std::vector<std::string>::iterator beginFL = firstLineStrings.begin();
        beginFL != firstLineStrings.end(); ++beginFL, ++firstLineIndex ){
		if(boost::algorithm::contains(*beginFL, std::string("txt"))){
            // the first 3 numbers are the number of trackers on each object
            // there are expected to be 3 of these numbers (as opposed to trackers) as of HW1
			outputData.title = *beginFL;
        } else if(firstLineIndex == 4) {
            // the number of Frames containing all the tracker markers,
            // essentially like timesteps, in each file
            std::cout << *beginFL <<"\n";
            outputData.numFrames = boost::lexical_cast<int>(*beginFL);
            
        } else {
            std::cout << *beginFL <<"\n";
			outputData.firstLine.push_back(boost::lexical_cast<int>(*beginFL));
		}
	}
    
    // move to second line
    ++begin;
	// parse remaining lines
	
	// need to decrement each point cloud in the file using the number of points specified in the first line
	std::vector<int> flCounter = outputData.firstLine;
	std::vector<int>::iterator FirstLineDoubleIterator = flCounter.begin();
    std::size_t cloudIndex = 0;
    // create a matrix large enough to store all the points for this set
    // and insert it into the vector
    outputData.clouds.push_back(Eigen::MatrixXd(*FirstLineDoubleIterator,3));
    
	// go through every string
	/// @todo there is a potential code safety issue here where if the sizes specified in the file don't match up with the length then we could run over the end of arrays.
	for(int vectorIndex = 0; begin != strs.end(); ++begin, ++vectorIndex){
		if(flCounter.size() == 0){
            throw std::out_of_range("csvCIS_pointCloudData() mismatch between specified number of points and actual");
        }
        
        if(begin->empty()){
            // no data on this line, skip it
            *FirstLineDoubleIterator--; // decrement point cloud counter for this cloud
            
        }else if(*FirstLineDoubleIterator > 0 ){
            // there is data here, load the point in
            std::vector<std::string> pointStrings;
            boost::split( pointStrings, *begin, boost::is_any_of(","), boost::token_compress_on);
            
            if(debug) printStringVector(pointStrings, "point:");
            
            BOOST_VERIFY(pointStrings.size() == 3);
            
            Eigen::Vector3d point(trimAndConvertToDouble(pointStrings[0]), trimAndConvertToDouble(pointStrings[1]), trimAndConvertToDouble(pointStrings[2]));
            outputData.clouds[cloudIndex].block<1,3>(vectorIndex,0)= point;
        
            *FirstLineDoubleIterator--; // decrement point cloud counter for this cloud
        } else {
			// go to next point cloud
	      	++FirstLineDoubleIterator;
            // create a cloud pre-allocated to the correct size
            Eigen::MatrixXd nextCloud(*FirstLineDoubleIterator,3);
			outputData.clouds.push_back(nextCloud);
			++cloudIndex;
            vectorIndex = 0;
	    }
		
	}
	
	return outputData;
}

#endif // _PARSE_CSV_CIS_POINTCLOUD_HPP_