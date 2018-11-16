cmake_minimum_required(VERSION 3.10.0)

project(OptIlm_DriverModule)


set (SOURCEFILES    	stdafx.h 
			OptIlm_DriverModule.h
			OptIlm_DriverModule.cpp)


adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER OptIlmDriveX)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER OptIlmDriveX)
