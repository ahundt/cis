#ifndef _HW_CONSTANTS_H_
#define _HW_CONSTANTS_H_

#include <string>
#include <vector>


static const std::string relativeDataPath("PA1-2/");

/////////////////
// PA 1 Constants
/////////////////

static const std::string pa1debuga("pa1-debug-a");
static const std::string pa1debugb("pa1-debug-b");
static const std::string pa1debugc("pa1-debug-c");
static const std::string pa1debugd("pa1-debug-d");
static const std::string pa1debuge("pa1-debug-e");
static const std::string pa1debugf("pa1-debug-f");
static const std::string pa1debugg("pa1-debug-g");
static const std::string pa1unknownh("pa1-unknown-h");
static const std::string pa1unknowni("pa1-unknown-i");
static const std::string pa1unknownj("pa1-unknown-j");
static const std::string pa1unknownk("pa1-unknown-k");


static const std::string dataFileNameSuffix_calbody("-calbody.txt");
static const std::string dataFileNameSuffix_calreadings("-calreadings.txt");
static const std::string dataFileNameSuffix_empivot("-empivot.txt");
static const std::string dataFileNameSuffix_optpivot("-optpivot.txt");
static const std::string dataFileNameSuffix_output1("-output1.txt");


std::vector<std::string> HW1DataFilePrefixes(){
    
    std::vector<std::string> defaultPrefixes;
    defaultPrefixes.push_back(pa1debuga);
    defaultPrefixes.push_back(pa1debugb);
    defaultPrefixes.push_back(pa1debugc);
    defaultPrefixes.push_back(pa1debugd);
    defaultPrefixes.push_back(pa1debuge);
    defaultPrefixes.push_back(pa1debugf);
    defaultPrefixes.push_back(pa1debugg);
    defaultPrefixes.push_back(pa1unknownh);
    defaultPrefixes.push_back(pa1unknowni);
    defaultPrefixes.push_back(pa1unknownj);
    defaultPrefixes.push_back(pa1unknownk);
	
	return defaultPrefixes;
}

/////////////////
// PA 2 Constants
/////////////////
static const std::string pa2debuga("pa2-debug-a");
static const std::string pa2debugb("pa2-debug-b");
static const std::string pa2debugc("pa2-debug-c");
static const std::string pa2debugd("pa2-debug-d");
static const std::string pa2debuge("pa2-debug-e");
static const std::string pa2debugf("pa2-debug-f");
static const std::string pa2unknowng("pa2-unknown-g");
static const std::string pa2unknownh("pa2-unknown-h");
static const std::string pa2unknowni("pa2-unknown-i");
static const std::string pa2unknownj("pa2-unknown-j");


static const std::string dataFileNameSuffix_ct_fiducials("-ct-fiducials.txt");
// [sic] fiducialss spelling error is as provided in assignment data
static const std::string dataFileNameSuffix_em_fiducials("-em-fiducialss.txt");
static const std::string dataFileNameSuffix_em_nav("-EM-nav.txt");
static const std::string dataFileNameSuffix_output2("-output2.txt");


std::vector<std::string> HW2DataFilePrefixes(){
    
    std::vector<std::string> defaultPrefixes;
    defaultPrefixes.push_back(pa2debuga);
    defaultPrefixes.push_back(pa2debugb);
    defaultPrefixes.push_back(pa2debugc);
    defaultPrefixes.push_back(pa2debugd);
    defaultPrefixes.push_back(pa2debuge);
    defaultPrefixes.push_back(pa2debugf);
    defaultPrefixes.push_back(pa2unknowng);
    defaultPrefixes.push_back(pa2unknownh);
    defaultPrefixes.push_back(pa2unknowni);
    defaultPrefixes.push_back(pa2unknownj);
	
	return defaultPrefixes;
}

#endif // _HW_CONSTANTS_H_