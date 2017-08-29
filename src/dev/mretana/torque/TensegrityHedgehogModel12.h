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

#ifndef TENSEGRITY_HEDGEHOGMODEL12_H
#define TENSEGRITY_HEDGEHOGMODEL12_H

/**
 * @file Tensegrity_HedgehogModel.h
 * @brief Contains the the class HedgehogTensegrity.h
  * @author Manuel Retana
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"

//used for data observer
#include "core/tgBasicActuator.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBuildSpec.h"
#include "core/tgObserver.h"
#include "sensors/tgDataObserver.h"
#include "LinearMath/btVector3.h"
#include "core/tgRod.h"
#include "core/tgBox.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"


 
// The C++ Standard Library
#include <vector>

// Forward declarations
//class tgSpringCableActuator;  //Controller
class tgBasicActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * Class that creates the six strut Hedgehog Tensegrity Hybrid model using tgcreator
 */
class TensegrityHedgehogModel12 : public tgSubject<TensegrityHedgehogModel12>, public tgModel
{
	
	
public: 
	
	/**
     * The only constructor. Utilizes default constructor of tgModel
     * Configuration parameters are within the .cpp file in this case,
     * not passed in. 
     */
    TensegrityHedgehogModel12();
	
	/**
     * Function introduved to centrate the Hedgehog
     */
    TensegrityHedgehogModel12(btVector3 origin);
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~TensegrityHedgehogModel12();
    
    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
    
    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    virtual void teardown();
    
    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(double dt);
    
    //Enables the display of location and other quantities
    //virtual void displayCallback();
	
	/**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);

    const std::vector<tgSpringCableActuator*>& getAllMuscles() const; //Controller
    
    std::vector<tgBasicActuator*>& getAllActuators();
    /**
     * Return a vector of all rod bodies for the controllers to work with.
     * @return A vector of all of the rod rigid bodies
     */
    std::vector<tgRod*>& getAllRods();
    
    std::vector<tgBox*>& getAllBoxes();
    
    std::vector<double> getBallCOM(); // Getting COM
    
    std::vector<double> Position(); // Getting COM
    
private:
	
	/**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] tetra: A tgStructure that we're building into
     */
    static void addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height);
	
	/**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addRods(tgStructure& s);
	
	/**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addMuscles(tgStructure& s);

private:
	
	/**
     * A list of all of the muscles. Will be empty until most of the way
     * through setup
     */
    std::vector<tgSpringCableActuator*> allMuscles; //Controller

	std::vector<tgBasicActuator*> allActuators;
/**
         * A function called during setup that determines the positions of
         * the nodes (center points of opposing box faces) 
         * based on construction parameters.
         * @param[in] s: the tgStructure that we're building into
         */
/**
	 * Moves the Hedgehog
	 * First rotates the structure around 3 axises given 3 angles.
	 * Moves the structure to the target point.
	 * et the cube to a given angular velocity
	 * (muscles and markers are moved automatically since they are attached).
	 */
    void moveModel(btVector3 targetPositionVector,btVector3 rotationVector,btVector3 speedVector);

    void addNodes(tgStructure& y);
/**
         * Determines the box nodes (center points of opposing box faces
         * Adds nodes to 'nodes' vector
         */
    void addBoxNodes();

    std::vector <tgNode> nodes;
    btVector3 origin;
    
     /**
     * A list of all of the rods. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgRod*> allRods;
    std::vector<tgBox*> allBoxes;
};

#endif  // TENSEGRITY_HEDGEHOGMODEL12_H
