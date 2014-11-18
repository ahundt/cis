.. _QuickStartGuides:

===========
Quick Start
===========


.. _FirstSteps:

First Steps
===========

The following steps will show you how to

- download and install CIS on your system.
- use the installation to create an example.
- build and test the example project.

You need to have a Unix-like operating system such as Linux or Mac OS X installed on your
machine in order to follow these steps. At the moment, there is no separate tutorial
available for Windows users, but you can install CygWin as an alternative.
Note, however, that CIS can also be installed and used on Windows.


Install CIS
-----------------

Get a copy of the source code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Clone the `Git <http://git-scm.com/>`__ repository from `GitHub <https://github.com/schuhschuh/cis/>`__ as follows:

.. code-block:: bash
    
    mkdir -p ~/local/src
    cd ~/local/src
    git clone https://github.com/ahundt/cis
    cd cis
    
or :doc:`download` a pre-packaged ``.tar.gz`` of the latest release and unpack it using the following command:

.. code-block:: bash

    mkdir -p ~/local/src
    cd ~/local/src
    tar xzf /path/to/downloaded/cis-$version.tar.gz
    cd cis-$version


Configure the build
~~~~~~~~~~~~~~~~~~~

Configure the build system using CMake 3.0.0 or a more recent version:

.. code-block:: bash
    
    mkdir build && cd build
    ccmake ..

- Press ``c`` to configure the project.
- Change ``CMAKE_INSTALL_PREFIX`` to ``~/local``.
- Set option ``BUILD_EXAMPLE`` to ``ON``.
- Make sure that option ``BUILD_PROJECT_TOOL`` is enabled.
- Press ``g`` to generate the Makefiles.

Note that if you do not have the **BASIS** dependency, an error will appear. 
Simply running the CMake steps a second time will cause BASIS
to automatically download and install.

Build and install CIS
~~~~~~~~~~~~~~~~~~~~~~~~~~~

CMake has generated Makefiles for GNU Make. The build is thus triggered by the make command:

.. code-block:: bash
    
    make

To install CIS after the successful build, run the following command:

.. code-block:: bash
    
    make install

As a result, CMake copies the built files into the installation tree as specified by the
``CMAKE_INSTALL_PREFIX`` variable.

.. _GettingStartedEnvironment:

Set up the environment
~~~~~~~~~~~~~~~~~~~~~~

For the following tutorial steps, set up your environment as follows. In general, however,
only the change of the ``PATH`` environment variable is recommended. The other environment
variables are only needed for the tutorial sessions.

Using the C or TC shell (csh/tcsh):

.. code-block:: bash
    
    setenv PATH "~/local/bin:${PATH}"
    setenv CIS_EXAMPLE_DIR "~/local/share/cis/example"

Using the Bourne Again SHell (bash):

.. code-block:: bash
    
    export PATH="~/local/bin:${PATH} "
    export CIS_EXAMPLE_DIR="~/local/share/basis/example"



Test the Example
~~~~~~~~~~~~~~~~

The following is an example of how to run the main cisHW1-2 executable, cisHW3-4 is similar.


.. code-block:: bash
    
	./cisHW1-2 --dataFilenamePrefix pa1-debug-a --dataFolderPath /path/to/cis/data/PA1-2/


	PivotCalibration result for pa1-debug-a-empivot.txt:

	197.115
	192.677
	192.437
	197.113
	192.677
	192.434



Command Line Format
~~~~~~~~~~~~~~~~~~~

The command line format follows standard conventions, plus the ability to store
a response file, typically named *.rsp, which saves additional command line
parameters for future use and convenience. The available command line parameters
and descriptions for the primary **cisHW1-2** executable file are below.

.. code-block:: bash
    
    ./cisHW1-2
    
    General Options:
      --responseFile arg                    File containing additional command line
                                            parameters
      --help                                produce help message
      --debug                               enable debug output
      --debugParser                         display debug information for data file
                                            parser
    
    Algorithm Options:
      --threads                             run each source data file in a separate
                                            thread
    
    Data Options:
      --pa1                                 set automatic programming assignment 1
                                            source data parameters, overrides
                                            DataFilenamePrefix, exclusive of pa1
      --pa2                                 set automatic programming assignment 2
                                            source data parameters, overrides
                                            DataFilenamePrefix, exclusive of pa2
      --dataFolderPath arg (=/Users/athundt/source/cis/build/bin)
                                            folder containing data files, defaults
                                            to current working directory
      --outputDataFolderPath arg (=/Users/athundt/source/cis/build/bin)
                                            folder for output data files, defaults
                                            to current working directory
      --dataFilenamePrefix arg              constant prefix of data filename path.
                                            Specify this multiple times to run on
                                            many data sources at once
      --dataFileNameSuffix_calbody arg (=-calbody.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_calreadings arg (=-calreadings.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_empivot arg (=-empivot.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_optpivot arg (=-optpivot.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_output1 arg (=-output1.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_ct_fiducials arg (=-ct-fiducials.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_em_fiducials arg (=-em-fiducialss.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_em_nav arg (=-EM-nav.txt)
                                            suffix of data filename path
      --dataFileNameSuffix_output2 arg (=-output2.txt)
                                            suffix of data filename path
      --calbodyPath arg                     full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --calreadingsPath arg                 full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --empivotPath arg                     full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --optpivotPath arg                    full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --output1Path arg                     full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --ct_fiducialsPath arg                full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --em_fiducialsPath arg                full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --em_navPath arg                      full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --output2Path arg                     full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination

The available command line parameters and descriptions for the primary 
**cisHW3-4** executable file are below.

.. code-block:: bash

    ./cisHW3-4
    
    General Options:
      --responseFile arg                    File containing additional command line
                                            parameters
      --help                                produce help message
      --debug                               enable debug output
      --debugParser                         display debug information for data file
                                            parser
    
    Algorithm Options:
      --threads                             run each source data file in a separate
                                            thread
    
    Data Options:
      --pa3                                 set automatic programming assignment 3
                                            source data parameters, overrides
                                            DataFilenamePrefix, exclusive of pa4
      --pa4                                 set automatic programming assignment 4
                                            source data parameters, overrides
                                            DataFilenamePrefix, exclusive of pa3
      --dataFolderPath arg (=/Users/athundt/source/cis/build/bin)
                                            folder containing data files, defaults
                                            to current working directory
      --outputDataFolderPath arg (=/Users/athundt/source/cis/build/bin)
                                            folder for output data files, defaults
                                            to current working directory
      --dataFilenamePrefix arg              constant prefix of data filename path.
                                            Specify this multiple times to run on
                                            many data sources at once
      --dataFilenameProblemPrefix arg (=Problem3)
                                            constant prefix of data typically
                                            starting with "Problem" filename path.
                                            Specify this multiple times to run on
                                            many data sources at once
      --suffixAnswer arg (=-Answer.txt)     suffix of data filename path
      --suffixOutput arg (=-Output.txt)     suffix of data filename path
      --suffixSample arg (=-SampleReadingsTest.txt)
                                            suffix of data filename path
      --suffixMesh arg (=Mesh.sur)          suffix of data filename path
      --suffixBodyA arg (=-BodyA.txt)       suffix of data filename path
      --suffixBodyB arg (=-BodyB.txt)       suffix of data filename path
      --AnswerPath arg                      full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --OutputPath arg                      full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --SamplePath arg                      full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --MeshPath arg                        full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --BodyAPath arg                       full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --BodyBPath arg                       full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --em_fiducialsPath arg                full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --em_navPath arg                      full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
      --output2Path arg                     full path to data txt file, optional
                                            alternative to prefix+suffix name
                                            combination
    

Unit Test
~~~~~~~~~

The easiest way to run the unit test is to build the software, 
then symlink the data folder "PA1-2" from "data/PA1-2" into
the same directory as the unit tests. In other words, the unit 
tests expect the directory "PA1-2" to be in the same directory
as the unit test executable when it is run. The same should be
done for the OUTPUT folder to PA1-2-OUTPUT for comparison of
debug output files for identifying problems in the system.

.. code-block:: bash
    
	ln -s /path/to/cis/data/PA1-2
	ln -s /path/to/cis/OUTPUT PA1-2-OUTPUT
	./cisHW1test

Next Steps
----------

Congratulations! You just finished your first CIS tutorial.

Now check out the :ref:`Tutorials` for more details regarding each of the
above steps and in-depth information about the used commands if you like,
or move on to the various :doc:`How-to Guides <howto>`.


.. _Tutorials:

Advanced Information
====================

For advanced documentation, please see the doxygen API documentation, unit tests, and software manual. If you cannot view
these files and documents, they are visible as inline source code documentation and and restructured text files
found in the /doc folder.  

For a less comprehensive tutorial-like introduction, please refer to the :ref:`FirstSteps` above.

.. ref links are required for the PDF version as the download directive in
   this case does not translate to a hyperlink, but text only.

