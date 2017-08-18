Project(tetraSpineLearningSine)

link_directories(${LIB_DIR})

add_library(${PROJECT_NAME} SHARED
   
            tgSineStringControl.cpp
            )

link_libraries(tetraSpineHardware
		learningSpines
                sensors
                tgcreator
                util
                controllers
                core
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_executable(AppTetraSpineHardwareSine
	tgSineStringControl.cpp
	LearningSpineSine.cpp
	AppTetraSpineHardwareSine.cpp
)



add_custom_command(TARGET AppTetraSpineHardwareSine POST_BUILD
					COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/logs)
