link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport
		yaml-cpp
		TensegrityModel)

add_executable(App3BarTraditionalX
    threeBarModel.cpp
    App3BarTraditional.cpp
) 

add_executable(App3BarYAML1
    App3BarYAML.cpp
    LengthControllerYAML.cpp
) 
