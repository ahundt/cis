#include <vector>
#include <boost/algorithm/string.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <boost/lexical_cast.hpp>


struct csvCIS_pointCloudData {
	std::string title;
	std::vector<int> firstLine;
	std::vector<std::vector<pcl::PointXYZ> > clouds;
};

csvCIS_pointCloudData parseCSV_CIS_pointCloud(std::string csv){
    csvCIS_pointCloudData outputData;
	std::vector<std::string> strs;
	boost::split(strs, csv, boost::is_any_of("\n"), boost::token_compress_on);
	
	// parse first line differently
    std::vector<std::string>::iterator begin = strs.begin();
	std::vector<std::string> firstLineStrings;

	boost::split(firstLineStrings, *begin, boost::is_any_of(","), boost::token_compress_on);
	for(std::vector<std::string>::iterator beginFL = firstLineStrings.begin(); beginFL != firstLineStrings.end(); ++beginFL ){
		if(boost::algorithm::contains(*beginFL, std::string("txt"))){
			outputData.title = *beginFL;
		} else {
			outputData.firstLine.push_back(boost::lexical_cast<double>(*beginFL));
		}
	}
	
	// parse remaining lines
	
	// need to decrement each point cloud in the file using the number of points specified in the first line
	std::vector<int> flCounter = outputData.firstLine;
	std::vector<int>::iterator FirstLineDoubleIterator = flCounter.begin();
	std::size_t cloudIndex = 0;
		
	// go through every string
	/// @todo there is a potential code safety issue here where if the sizes specified in the file don't match up with the length then we could run over the end of arrays.
	for(; begin != strs.end(); ++begin){
		if(flCounter.size() == 0){
            throw std::out_of_range("csvCIS_pointCloudData() mismatch between specified number of points and actual");
		}
		
		if(*FirstLineDoubleIterator > 0){
		  std::vector<std::string> pointStrings;
		  boost::split( pointStrings, *begin, boost::is_any_of(","), boost::token_compress_on);

  		  outputData.clouds[cloudIndex].push_back (pcl::PointXYZ (boost::lexical_cast<double>(pointStrings[0]), boost::lexical_cast<double>(pointStrings[1]), boost::lexical_cast<double>(pointStrings[2])));
		  
		  // decrement point cloud counter for this cloud
		  *FirstLineDoubleIterator = *FirstLineDoubleIterator-1;
	    } else {
			// go to next point cloud
	      	++FirstLineDoubleIterator;
		  	std::vector<pcl::PointXYZ> emptyCloud;
			outputData.clouds.push_back(emptyCloud);
			++cloudIndex;
	    }
		
	}
	
	return outputData;
}