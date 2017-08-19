/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

#ifndef LOGGER_H
#define LOGGER_H

/**
 * @file Logger.cpp
 * @brief Implementation of data logger for tensegrity
 * @author Manuel Retana
 * $Id$
 */

#include "TensegrityHedgehogModel12.h"

// This library
#include "core/tgObserver.h"
#include "controllers/tgBasicController.h"
#include "core/tgBasicActuator.h"
#include "core/tgBox.h"
#include "core/tgRod.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class TensegrityHedgehogModel12;
//Standard NTRT parameters
class Logger : public tgObserver<TensegrityHedgehogModel12>
{
public:
	
    Logger(const double length = 400);
    
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~Logger();
    
    virtual void onSetup(TensegrityHedgehogModel12& subject);
    
    /**
     * Apply the length controller. Called by notifyStep(dt) of its
     * subject.
     * @param[in] subject - the RPModel that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(TensegrityHedgehogModel12& subject, double dt);
   
//Create the vectors of the objects you will call
    std::vector<tgBox*> Boxes;
    std::vector<tgRod*> Rods;
    std::ofstream fileStream;

//Require these declarations
private:

    const double m_length;
    double globalTime = 0;
    int toggle;

};

#endif //LOGGER_H
