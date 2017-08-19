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

/**
 * @file Logger.cpp
 * @brief Implementation of data logger for tensegrity
 * @author Manuel Retana
 * $Id$
 */

// This module
#include "Logger.h"
#include "core/tgBox.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>



using namespace std;

Logger::Logger(const double length) :
  m_length(length)
{
  if (length < 0.0)
    {
      throw std::invalid_argument("Negative length");
    }
}

Logger::~Logger()
{
	if( fileStream.is_open() )
	{
		fileStream.close();
	}
}	

void Logger::onSetup(TensegrityHedgehogModel12& subject)
{
  //Get Hedgehog
  Rods = subject.getAllRods();
  Boxes = subject.getAllBoxes();
}

//OnStep Function repeats every timestep. The time step can be located in the app (main) cpp file
void Logger::onStep(TensegrityHedgehogModel12& subject, double dt)
{
	//Open file if it is not open
	if( !fileStream.is_open() )
	{
		fileStream.open( "/home/tensegribuntu/logger/HHOnly.txt" ); //Address of where the data logger file writing
	}
	//Display error of the file cannot be opened
	if( !fileStream.good() )
	{
		throw std::invalid_argument("ERROR: Could not open the file for data logging.");
	}
	
//Standard NTRT	
  if (dt <= 0.0) {
    throw std::invalid_argument("dt is not positive");
  }
  else {
    globalTime += dt;
  } 
  
//Calls boxes gets its center of mass and writes it into a file. 
  //Use the btRigidBody to call other parameters and put them in this loop. This only works for the HH box rigid object
    for(int i=0;i<Boxes.size();i++)
	{
		btTransform COM;// center of mass vector object
		// 
		Boxes[i]->getPRigidBody()->getMotionState()->getWorldTransform(COM);
			const btVector3& result = COM.getOrigin();
			fileStream << result << std::endl;
			//std::cout << result<< std::endl; //activate if you want to see output display in screen
	}
}
