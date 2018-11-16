cmake_minimum_required(VERSION 3.10.0)

project(OptIlm_SignalValueGenerator)

if (NOT TARGET adtf::filtersdk)
	 find_package(ADTF COMPONENTS filtersdk)
endif()


set (SOURCEFILES    	
    	stdafx.h 
	OptIlm_SignalValueGenerator.h
	OptIlm_SignalValueGenerator.cpp)
						
#we use the special ADTF CMake command to add an shared object/DLL which has an extension called .adtfplugin
adtf_add_streaming_service(${PROJECT_NAME} ${SOURCEFILES})

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)
	
adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin")

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER OptIlmDriveX)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER OptIlmDriveX)
