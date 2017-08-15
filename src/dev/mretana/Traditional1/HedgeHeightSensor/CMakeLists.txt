Project(mretana)

link_libraries(tgcreator core)

add_executable(HedgeHeightSensor
    TensegrityHedgehogModel12.cpp
    AppTensegrityHedgehog12.cpp
    heightSensor.cpp
) 