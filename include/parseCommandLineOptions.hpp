/*
 *  @file parseCommandLineOptions.hpp
 *  @author ahundt
 */

#ifndef PARSECOMMANDLINEOPTIONS_HPP_
#define PARSECOMMANDLINEOPTIONS_HPP_

#include <fstream>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/token_functions.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/tuple/elem.hpp>

#define CLO_GET_ARG_STR3(data) BOOST_PP_TUPLE_ELEM(3, 0, data)
#define CLO_GET_DESCRIPTION_STR3(data) BOOST_PP_TUPLE_ELEM(3, 2, data)

#define CLO_PARAM_FILE_STR           ("paramFile", boost::program_options::value<std::string>(), "path to existing parameters file")
#define CLO_MAKE_PARAM_FILE_STR      ("makeParamFile", boost::program_options::value<std::string>(), "path to a new parameters file")

#define CLO_GET_ARG_STR2(data) BOOST_PP_TUPLE_ELEM(2, 0, data)

#define CLO_HELP               ("help", "produce help message")
#define CLO_DEBUG              ("debug",po::value<bool>()->default_value(false), "enable debug output")
#define CLO_RESPONSE_FILES     ("responseFiles", boost::program_options::value<std::vector<std::string> >()->multitoken(), "List of files containing additional command line parameters (multiple arguments)")

namespace boost
{
   namespace program_options
   {
      template<class Type>
      inline bool readOption(const boost::program_options::variables_map& vm,
            const std::string& optionName,
            Type& var,
            bool required) {
         bool ret = false;
         try
         {
            if(vm.count(optionName))
            {
               var = vm[optionName].as<Type>();
               ret = true;
            }
            else
            {
               if(required)
               {
                  throw std::runtime_error(optionName + " is missing");
               }
            }
         }
         catch(boost::bad_any_cast& e)
         {
            throw std::runtime_error("readOption() cast failed for option:" + optionName + " with error:" + e.what());
         }
         return ret;
      }

      
      ////////////////////////////////////////////////////////////////////////
      //! \brief Parse arguments from a given file list and add the to a vmap
      //!
      //! Parses a list of response files that contain additional arguments
      //! as though they had been given on the command line, adding them to
      //! the given vmap structure.
      //!
      //! @param vmap                  Parameter map to add to
      //! @param optiopnsDescription   List of available program options used
      //!                              for @c vmap
      //! @param optionName         Name of the response file parameter in
      //!                              @c optionsDescription
      //! @return Returns true if successful, false otherwise
      ////////////////////////////////////////////////////////////////////////
      inline bool parseResponseFiles(boost::program_options::variables_map& vmap,
                             const options_description &optionsDescription,
                             const std::string& optionName)
      {
         if (vmap.count(optionName.c_str())) {
            std::vector<std::string> filesList = vmap[optionName.c_str()].as<std::vector<std::string> >();
            for (unsigned int fileNum = 0; fileNum < filesList.size(); ++fileNum)
            {
               // Load the file and tokenize it
               std::ifstream ifs(filesList[fileNum].c_str());
               if (!ifs) {
                  std::cout << "Could not open the response file\n";
                  return false;
               }
               // Read the whole file into a string
               std::stringstream ss;
               ss << ifs.rdbuf();
               // Split the file content
               boost::char_separator<char> sep(" \n\r");
               std::string sstr = ss.str();
               boost::tokenizer<boost::char_separator<char> > tok(sstr, sep);
               std::vector<std::string> args;
               copy(tok.begin(), tok.end(), std::back_inserter(args));
               // Parse the file and store the options
               boost::program_options::store(boost::program_options::command_line_parser(args).options(optionsDescription).run(), vmap);
            }
         }
         return true;
      }
      
      ////////////////////////////////////////////////////////////////////////
      //! \brief Parse arguments from a given file list and add the to a vmap
      //!
      //! Parses a list of response files that contain additional arguments
      //! as though they had been given on the command line, adding them to
      //! the given vmap structure.
      //! Uses the default parameter name for specifying response files.
      //!
      //! @param vmap                  Parameter map to add to
      //! @param optiopnsDescription   List of available program options used
      //!                              for @c vmap
      //! @return Returns true if successful, false otherwise
      ////////////////////////////////////////////////////////////////////////
      inline bool parseResponseFiles(boost::program_options::variables_map& vmap,
                             const options_description &optionsDescription)
      {
         return parseResponseFiles(vmap, optionsDescription, CLO_GET_ARG_STR3(CLO_RESPONSE_FILES));
      }
      
   }
}

#endif /* PARSECOMMANDLINEOPTIONS_HPP_ */
