#ifndef _PARSE_CSV_CIS_POINTCLOUD_HPP_
#define _PARSE_CSV_CIS_POINTCLOUD_HPP_

// system includes
#include <Eigen/Dense>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

// project includes
#include "matrixOperations.hpp"

void Print (const std::vector<int>& v){
    //vector<int> v;
    for (int i=0; i<v.size();i++){
        std::cout << v[i] << std::endl;
    }
}

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
    typedef Eigen::MatrixXd TrackerPoints;
    typedef std::vector<TrackerPoints> TrackerDevices;
    typedef std::vector<TrackerDevices> TrackerFrames;
    
    /// filename of the file
	std::string title;
    
    /// the number of elements in each "cloud"
	std::vector<int> firstLine;
    
    /// vector containing each cloud
    TrackerFrames frames;
    
};

/// parse the CIS HW1 point cloud string that was loaded directly from a txt file into a simpler struct
/// @see DataFileFormatDescription.svg for a visual explanation of the file format
csvCIS_pointCloudData parseCSV_CIS_pointCloud(std::string csv, bool debug = false){
    csvCIS_pointCloudData outputData;
    std::vector<std::string> strs;
    
    // by default if no frame count is specified, there is only one frame
    int numFrames = 1;
    // split into separate strings by newlines
	boost::split(strs, csv, boost::is_any_of("\n"));
    
    if(debug) {
        std::cout << csv << "\n\n FULL FILE CONTENTS PRINTED, now printing newly split vector of strings:\n\n";
        printStringVector(strs);
    }
	
    
	// parse first line differently
    std::vector<std::string>::iterator currentStringLineIterator = strs.begin();
	std::vector<std::string> firstLineStrings;

	boost::split(firstLineStrings, *currentStringLineIterator, boost::is_any_of(", "), boost::token_compress_on);
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
            numFrames = boost::lexical_cast<int>(*beginFL);
            
        } else {
            std::cout << *beginFL <<"\n";
			outputData.firstLine.push_back(boost::lexical_cast<int>(*beginFL));
		}
	}
    
    // move to second line
    ++currentStringLineIterator;
	// parse remaining lines
	
	// need to decrement each point cloud in the file using the number of points specified in the first line
    /// @bug this isn't accounted for correctly, somehow the values end up huge or negative
	std::vector<int> trackerCounterVec;
    // start out on a new frame
	std::vector<int>::iterator trackerCounterVecPointsRemainingIterator = trackerCounterVec.end();
    std::size_t currentPointIndexInThisTracker = 0;
    std::size_t trackerDeviceIndex = 0;
    csvCIS_pointCloudData::TrackerFrames::iterator currentFrameIterator(outputData.frames.end());
    
    /// @todo remove isNewFrame, it is confusing
    bool isNewFrame = true;
    
    
	// go through every string
	/// @todo there is a potential code safety issue here where if the sizes specified in the file don't match up with the length then we could run over the end of arrays.
	for(int stringVectorIndex = 0; currentStringLineIterator != strs.end(); ++currentStringLineIterator, ++stringVectorIndex){
        
        ////////////////////
        // Skip Empty lines
        ////////////////////
        if(currentStringLineIterator->empty()){
            // no data on this line, skip it
            continue; // skip empty lines, consider finding a way to remove the continue
            
        }
        
        /////////////////////////////////////////
        // Move to next Frame if needed
        /////////////////////////////////////////
        
        if(trackerCounterVecPointsRemainingIterator == trackerCounterVec.end()){
            // done with this frame, move on to next one
            // back to the first tracker device for this new frame
            trackerDeviceIndex = 0;
            
            // create the counters for the next set of trackers
            trackerCounterVec = outputData.firstLine;
            trackerCounterVecPointsRemainingIterator = trackerCounterVec.begin();
            
            // add the next frame to the vector, which starts out empty
            csvCIS_pointCloudData::TrackerDevices tf;
            outputData.frames.push_back(tf);
            currentFrameIterator = outputData.frames.end()-1;
            isNewFrame = true;
            
            // add the next tracker point cloud to fill out, currently empty
            Eigen::MatrixXd nextCloud(*trackerCounterVecPointsRemainingIterator,3);
            currentFrameIterator->push_back(nextCloud);
        }
        
        /////////////////////////////////////////
        // move to next Tracker if needed
        /////////////////////////////////////////
        
        if(!isNewFrame && *trackerCounterVecPointsRemainingIterator == 0 ) {
            BOOST_VERIFY(trackerCounterVecPointsRemainingIterator!=trackerCounterVec.end());
			// go to next tracker
	      	++trackerCounterVecPointsRemainingIterator;
            // create a matrix large enough to store all the points for this tracker
            // and insert it into the vector
            /// @bug the length isn't accounted for correctly, somehow the values in *currentTrackerPointsRemainingIterato end up huge or negative
            Eigen::MatrixXd nextCloud(*trackerCounterVecPointsRemainingIterator,3);
			currentFrameIterator->push_back(nextCloud);
			++trackerDeviceIndex;
            currentPointIndexInThisTracker = 0;
	    }
        
        /////////////////////////////////////////
        // Add point to this tracker
        /////////////////////////////////////////
        if(*trackerCounterVecPointsRemainingIterator > 0 ){
            // there is data here, load the point in
            std::vector<std::string> pointStrings;
            boost::split( pointStrings, *currentStringLineIterator, boost::is_any_of(","), boost::token_compress_on);
            
            if(debug) printStringVector(pointStrings, "point:");
            
            BOOST_VERIFY(pointStrings.size() == 3);
            
            Eigen::Vector3d point(trimAndConvertToDouble(pointStrings[0]), trimAndConvertToDouble(pointStrings[1]), trimAndConvertToDouble(pointStrings[2]));
            (*currentFrameIterator)[trackerDeviceIndex].block<1,3>(currentPointIndexInThisTracker,0)= point;
            
            (*trackerCounterVecPointsRemainingIterator)--; // decrement point cloud counter for this cloud
            currentPointIndexInThisTracker++;
        }
		
        // s
        isNewFrame = false;
	}
	
    std::cout << "numframes: " <<numFrames << " outputdata.frames.size(): " << outputData.frames.size() << std::endl;
    BOOST_VERIFY(numFrames==outputData.frames.size());
    
	return outputData;
}

#endif // _PARSE_CSV_CIS_POINTCLOUD_HPP_