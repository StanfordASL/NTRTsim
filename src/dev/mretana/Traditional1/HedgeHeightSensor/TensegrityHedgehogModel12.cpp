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
 * @file Tensegrity_HedgehogModel.cpp
 * @brief Contains the definition of the members of the class HedgehogTensegrity
  * @author Manuel Retana
 * $Id$
 */

// This module
#include "TensegrityHedgehogModel12.h"
// This library
#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"
#include "core/tgBox.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
// The C++ Standard Library
#include <stdexcept>
//Display Text Delete not used
#include "GlutStuff.h"
#include "GLDebugFont.h"
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"
#if 0
extern btAlignedObjectArray<btVector3> debugContacts;
extern btAlignedObjectArray<btVector3> debugNormals;
#endif 
#include <iostream>



namespace
{
    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Provide specifics about the hedgehog mass etc
    

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density; //Density of the rods
        double radius;
        double stiffness;
        double stiffness_in;
        double damping;
        double damping_in;
        double rod_length;
        double rod_space; 
        double payload_h; 
        double density_pay;
        double radius_pay;  
        double friction;
        double rollfriction;
        double restitution;
        double pretension;
	double minRestLength;
        bool   history; 
        double maxTens;
        double targetVelocity;
	//Hedgehog Parameters beyond this point
	double width;
	double height;
	double density1; //Density of the box
	double friction1;
	double rollfriction1;
	double restitution1;
	double triangle_length;
        double triangle_height;
        double prism_height;
    } c =
   {
     0.15,    // density (kg / length^3)
     0.3175/2.0,     // radius (length)
     250000.0,   // stiffness of outer muscles (kg / sec^2)
     25000.0,    // stiffness of inner muscles (kg/sec^2) 500
     25000.0,    // damping of outer muscles (kg / sec)
     50.0,     //damping of inner muscles (kg/sec)
     15.0,     // rod_length (length)  15
     3.75,      // rod_space (length) 3.75
     0.5,         // half payload height (length)
     0.1,        //cage density (kg/length^3)
     0.025,        //cage radius (length)
     0.99,      // friction (unitless)
     0.1,     // rollFriction (unitless)
     0.0,      // restitution (?)
     0.0,        // pretension of the payload (force) 0
     0.001,             // Min Actual length in (length)
     false,     // history
     1000000,   // maxTens
     10000,    // targetVelocity
     0.4625, // width (dm)
     0.4625, // length (dm)
     .8,  //Density of box (kg / length^3) 
     0.5,  // friction (unitless)
     0.01, // rollFriction (unitless) Double check what roll friction of a box means
     0,  // restitution (?)
     10.0,     // triangle_length (length) Value not relevant
     10.0,     // triangle_height (length) Value not relevant
     20.0,     // prism_height (length) Value not relevant
  };
} // namespace

TensegrityHedgehogModel12::TensegrityHedgehogModel12() : tgModel() 
{
	origin = btVector3(0,0,0); //Double check if needed
}

TensegrityHedgehogModel12::TensegrityHedgehogModel12(btVector3 center) : tgModel() 
{
	origin = btVector3(center.getX(), center.getY(), center.getZ()); //Double check if needed
}

TensegrityHedgehogModel12::~TensegrityHedgehogModel12()
{
}

void TensegrityHedgehogModel12::addNodes(tgStructure& s,
			    double edge,
                            double width,
                            double height)
{
    //const double half_length = c.rod_length / 2;  //Not needed

    // Nodes for struts
	
	s.addNode(-1.85, 0, -.43);//0
	s.addNode(-0.76, -3.69, 0.65);//1
	s.addNode(-1.85, 0, -1.960);//2
	s.addNode(1.85, -1.08, -3.04);//3
	s.addNode(-0.76, 0, -3.04);//4
	s.addNode(-1.85, -3.69, -1.96);//5
	s.addNode(0.76, 0, -3.04);//6
	s.addNode(1.85, -1.08, 0.65);//7
	s.addNode(1.85, 0, -1.96);//8
	s.addNode(0.76, -3.69, -3.04);//9
	s.addNode(1.85, 0, -.43);//10
	s.addNode(-1.85, -1.08, 0.65);//11
	s.addNode(0.76, 0, 0.65);//12
	s.addNode(1.85, -3.69, -.43);//13
	s.addNode(-.76, 0, 0.65);//14
	s.addNode(-1.85, -1.08, -3.04);//15
	s.addNode(-1.85, -3.69, -0.43);//16
	s.addNode(1.85, -2.61, 0.65);//17
	s.addNode(-0.76, -3.69, -3.04);//18
	s.addNode(-1.85, -2.61, 0.65);//19
	s.addNode(1.85, -3.69, -1.96);//20
	s.addNode(-1.85, -2.61, -3.04);//21
	s.addNode(0.76, -3.69, 0.65);//22
	s.addNode(1.85, -2.61, -3.04);//23
	
	

	
	
    //Nodes for payload
    //s.addNode(0,c.payload_h,0); //12 
    //s.addNode(0,-c.payload_h,0); //13
	
    //Hedgehog Cage
    //Down
    s.addNode(-0.5, -2.35, -1.2); // 24 --
    s.addNode( 0.5, -2.35, -0.2); // 25
    s.addNode(0.5, -2.35, -1.2); // 26  
    s.addNode(-0.5, -2.35, -0.2); // 27
    //Up
    s.addNode(-0.5, -1.35, -1.2); // 28
    s.addNode(0.5, -1.35, -0.2); //  29 ++
    s.addNode(0.5, -1.35, -1.2); //  30 
    s.addNode(-0.5, -1.35, -0.2); //  31
    
    //Central Outer Nodes Cube
    s.addNode(0,-1.85, -1.2); // 32/ 24 to 30 Left
    s.addNode(0, -1.85, -0.2); // 33/ 29 to 27 Right
    s.addNode(-0.5,-1.85, -0.7); // 34/ 24 to 31 Back 
    s.addNode(0.5, -1.85, -0.7); // 35/ 29 to 26 Front
    s.addNode(0, -2.35, -0.7); // 36/  24 to 25 Bottom
    s.addNode(0, -1.35, -0.7); // 37/  28 to  29 Top
	//Hedgehog Cage
	
	 // Midpoint Rod Nodes 
    s.addNode(-1.315,-1.845 ,0.11);// 38 -- 0 to 1
    s.addNode(0,-0.54,-2.5);// 39 -- 2 to 3
    s.addNode(-1.305,-1.845,-2.5);// 40 -- 4 to 5
    s.addNode(1.305,-0.54,-1.195 );// 41 -- 6 to 7
    s.addNode(1.305, -1.845,-2.5);// 42 -- 8 to 9
    s.addNode(0,-0.54 ,0.11);// 43 -- 10 to 11
    s.addNode(1.305, -1.845, 0.11);// 44 -- 12 to 13
    s.addNode(-1.305, -0.54, -1.195);// 45 -- 14 to 15
    s.addNode(0,-3.15 ,0.11 );// 46 -- 16 to 17
    s.addNode(-1.305, -3.15, -1.195);// 47 -- 18 to 19
    s.addNode(0, -3.15, -2.5);// 48 -- 20 to 21
    s.addNode(1.305, -3.15, -1.195);// 49 -- 22 to 23
    
}

void TensegrityHedgehogModel12::addRods(tgStructure& s)
{
    // Struts
   /* s.addPair( 0, 1,"rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair(10, 11, "rod");
    s.addPair( 12,  13, "rod");
    s.addPair( 14,  15, "rod");
    s.addPair( 16,  17, "rod");
    s.addPair( 18,  19, "rod");
    s.addPair( 20,  21, "rod");
    s.addPair(22, 23, "rod");*/
	//Contruct all the bars plus mid bars for the midnodes
    s.addPair( 0, 38,"rod");  
    s.addPair(0, 1, "rod");
    s.addPair( 2,  39, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  40, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  41, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  42, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair(10, 43, "rod");
    s.addPair(10, 11, "rod");
     s.addPair( 12,  44, "rod");
    s.addPair( 12,  13, "rod");
    s.addPair( 14,  45, "rod");
    s.addPair( 14,  15, "rod");
    s.addPair( 16,  46, "rod");
    s.addPair( 16,  17, "rod");
    s.addPair( 18,  47, "rod");
    s.addPair( 18,  19, "rod");
    s.addPair( 20,  48, "rod");
    s.addPair( 20,  21, "rod");
    s.addPair(22, 49, "rod");
    s.addPair(22, 23, "rod");

    // Payload
    //s.addPair(12, 13, "payload_rod");

    //Hedgehogcage
    s.addPair( 24,  28, "payload_rod");
    s.addPair( 25,  29, "payload_rod");
    s.addPair( 26,  30, "payload_rod");
    s.addPair( 27,  31, "payload_rod");
    s.addPair( 24,  26, "payload_rod");
    s.addPair( 24,  27, "payload_rod");
    s.addPair( 25,  26, "payload_rod");
    s.addPair( 25,  27, "payload_rod");
    s.addPair( 28,  30, "payload_rod");
    s.addPair( 28,  31, "payload_rod");
    s.addPair( 29,  30, "payload_rod");
    s.addPair( 29,  31, "payload_rod");
    
    //XPairs 
    //Top
    //s.addPair( 28,  29, "payload_rod");
    s.addPair( 30,  31, "payload_rod");
    //bottom
    //s.addPair( 24,  25, "payload_rod");
    s.addPair( 26,  27, "payload_rod");
    //All other
    s.addPair( 25,  30, "payload_rod");
    
    s.addPair( 25,  31, "payload_rod");
    s.addPair( 26,  28, "payload_rod");
    s.addPair( 27,  28, "payload_rod"); 
    
    // Middle Pairs Change
  // s.addPair( 24,  30, "payload_rod");// Middle Pairs Change
   s.addPair( 24,  32, "payload_rod");
   s.addPair( 24,  30, "payload_rod");
   
   //s.addPair( 27,  29, "payload_rod");
   s.addPair( 27,  33, "payload_rod");
   s.addPair( 27,  29, "payload_rod");
   
   //s.addPair( 24,  31, "payload_rod");
   s.addPair( 24,  34, "payload_rod");
   s.addPair( 24,  31, "payload_rod");
   
   //s.addPair( 26,  29, "payload_rod");
   s.addPair( 26,  35, "payload_rod");
   s.addPair( 26,  29, "payload_rod");
    
     //s.addPair( 24,  25, "payload_rod");
    s.addPair( 24,  36, "payload_rod");
    s.addPair( 24,  25, "payload_rod");
    
    //s.addPair( 28,  29, "payload_rod");
    s.addPair( 28,  37, "payload_rod");
    s.addPair( 28,  29, "payload_rod");
    

}


void TensegrityHedgehogModel12::addMuscles(tgStructure& s)
{
    // Outer Cables
    s.addPair(0, 2,  "muscle");
    s.addPair(0, 14,  "muscle");
    s.addPair(0, 11,  "muscle");
    s.addPair(0, 4, "muscle");
    s.addPair(2, 4,  "muscle");
    s.addPair(2, 15,  "muscle");
    s.addPair(4, 15,  "muscle");
    s.addPair(4, 6, "muscle");
    s.addPair(6, 8,  "muscle");
    s.addPair(6, 3,  "muscle");
    s.addPair(8, 3,  "muscle");
    s.addPair(3, 23, "muscle");
    s.addPair(8, 10,  "muscle");
    s.addPair(10, 12,  "muscle");
    s.addPair(10, 7,  "muscle");
    s.addPair(7, 12, "muscle");
    s.addPair(14, 12,  "muscle");
    s.addPair(14, 11, "muscle");
    s.addPair(23, 20, "muscle");
    s.addPair(9, 23,  "muscle");
    s.addPair(7, 17,  "muscle");
    s.addPair(17, 13, "muscle");
    s.addPair(13, 22, "muscle");
    s.addPair(22, 17,  "muscle");
    s.addPair(22, 1,  "muscle"); 
    s.addPair(1, 16,  "muscle");
    s.addPair(16, 19,  "muscle");
    s.addPair(19, 1,  "muscle");
    s.addPair(19, 11, "muscle");
    s.addPair(5, 18,  "muscle");
    s.addPair(18, 21,  "muscle");
    s.addPair(21, 5,  "muscle");
    s.addPair(21, 15, "muscle");
    s.addPair(18, 9,  "muscle");
    s.addPair(9, 20,  "muscle");
    s.addPair(20, 13,  "muscle");
    s.addPair(16, 5,  "muscle");
/*
    // Hedgehog Cage Internal cables
    s.addPair(1, 24, "muscle_in"); //Bottom 
    s.addPair(5, 26, "muscle_in"); //Bottom 
    s.addPair(9, 25, "muscle_in"); //Bottom
    s.addPair(13, 27, "muscle_in"); //Bottom
    
    s.addPair(0, 28, "muscle_in"); //Top
    s.addPair(4, 30, "muscle_in"); // Top
    s.addPair(8, 29, "muscle_in"); // Top
    s.addPair(12,  31, "muscle_in"); // Top
    */
    //Experimetal Center Nodes CHANGE

    s.addPair(35, 38, "muscle_in"); // Front 35 purple vertical midnode 38 ++
    s.addPair(33, 38, "muscle_in"); // Right 33 purple vertical midnode 38 ++
    
    s.addPair(37, 39, "muscle_in"); // Top 37 yellow top horizontal midnode 39 Back 
    s.addPair(34, 39, "muscle_in"); // Back 34 yellow top horizontal midnode 39 Back
    
    s.addPair(34, 40, "muscle_in"); // Back 34 purple vertical midnode 40 +-
    s.addPair(33, 40, "muscle_in"); // Right 33 purple vertical midnode 40 +-
    
    s.addPair(37, 41, "muscle_in"); // Top 37 yellow top horizontal midnode 41 Left
    s.addPair(32, 41, "muscle_in"); // Back 34 yellow top horizontal midnode 41 Left
    
    s.addPair(34, 42, "muscle_in"); // Back 34 purple vertical midnode 42 --
    s.addPair(32, 42, "muscle_in"); // Left 32 purple vertical midnode 42 --
    
    s.addPair(37, 43, "muscle_in"); // Top 37 yellow top horizontal midnode 43 Front
    s.addPair(35, 43, "muscle_in"); // Front 34 yellow top horizontal midnode 43 Front
    
    s.addPair(35, 44, "muscle_in"); // Front 35 purple vertical midnode 44 -+
    s.addPair(32, 44, "muscle_in"); // Left 32 purple vertical midnode 44 -+
    
    s.addPair(37, 45, "muscle_in"); // Top 37 yellow top horizontal midnode 41 Right
    s.addPair(33, 45, "muscle_in"); // Back 34 yellow top horizontal midnode 41 Right
    
    s.addPair(35, 46, "muscle_in"); // Front 35 purple bottom horizonal bar 46 Front
    s.addPair(36, 46, "muscle_in"); // Bottom 36 purple bottom horizonal bar 46 Front
    
    s.addPair(33, 47, "muscle_in"); // Right 33 yellow bottom horizonal bar 47 Right
    s.addPair(36, 47, "muscle_in"); // Bottom 36 yellow bottom horizonal bar 47 Right
    
    s.addPair(34, 48, "muscle_in"); // Back 34 purple bottom horizonal bar 48 Back
    s.addPair(36, 48, "muscle_in"); // Bottom 36 purple bottom horizonal bar 48 Back
    
    s.addPair(32, 49, "muscle_in"); // Left 32 yellow bottom horizonal bar 49 Left
    s.addPair(36, 49, "muscle_in"); // Bottom 36 yellow bottom horizonal bar 49 Left
     
     
    //s.addPair(0, 35, "muscle_in"); 
    
}

void TensegrityHedgehogModel12::addNodes(tgStructure& y) {
    addBoxNodes();
 
        for(std::size_t i=0;i<nodes.size();i+=2) {
        y.addNode(nodes[i]);
        y.addNode(nodes[i+1]);
        y.addPair(i, i+1, "box");
    }
}

//Hedgehog Box Nodes
void TensegrityHedgehogModel12::addBoxNodes() {
    tgNode node; //Creates the height of Hedgehog based on Nodes

    double x1 = 0; // use the distance between vector 1 and vector 2 to control the height
    double x2 =  0.4625;
    double y1 = 0;
    double y2 = 0.4625;
    double z1 = 0;
    double z2 =  0.4625*sqrt(2); // The length of the diagonal is the input and the output is the height
	//For instance if you want a 1 meter height cube you need to provide sqrt 2 for the height

    node = tgNode(x1, y1, z1, "node");
    nodes.push_back(node);

    node = tgNode(x2, y2, z2, "node");
    nodes.push_back(node);
}

void TensegrityHedgehogModel12::setup(tgWorld& world)
{
	//Define configuration for hedgehog
    const tgBox::Config boxConfig(c.width, c.height, c.density1, c.friction1, c.rollfriction1, c.restitution1);
	
	// Define the configurations of the rods and strings
    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollfriction, c.restitution);

    const tgRod::Config payConfig(c.radius_pay, c.density_pay, c.friction, 
                c.rollfriction, c.restitution);
    
   // Define configuration for actuators
   tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension * c.stiffness / c.stiffness_in, c.history,
					    c.maxTens, c.targetVelocity); 

    tgSpringCableActuator::Config muscleInConfig(c.stiffness_in, c.damping_in, c.pretension, c.history,
                        c.maxTens, c.targetVelocity); 



    // Start creating the structures
    tgStructure s;
    tgStructure y; // Hedgehog
    // Add nodes to the structure
    addNodes(y);  //Nodes of Hedgehog
    addNodes(s, c.triangle_length, c.triangle_height, c.prism_height);
    addRods(s);
    //addNodes(s); //Fun Glitch: if activated rods become boxes

    addMuscles(s);

    
    //Initial location of tensegrity
    s.move(btVector3(0, 6, 0));
    //s.move(btVector3(0, 20, 0)); Change the parameters below as well
    
    
    //Initial Location and orientation of Hedgehog
    y.move(btVector3(0,4.15, -.7)); //y=4.15
    y.addRotation(btVector3(0,4.15,-.7),btVector3(0,0,1), 180); // Z blue Axis y=4.15
    y.addRotation(btVector3(0,4.15,-.7),btVector3(1,0,0), 260); // X red axis y=4.15


    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("payload_rod", new tgRodInfo(payConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("muscle_in", new tgBasicActuatorInfo(muscleInConfig));
    spec.addBuilder("box", new tgBoxInfo(boxConfig));//Hedgehog
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec); //Tensegrity
    tgStructureInfo structureInfos(y, spec); //Hedgehog

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world); //Tensegrity
    structureInfos.buildInto(*this, world); // Hedgehog

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    //allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants()); //controller

    
    //Controllers
    /*   // Get the rod rigid bodies for controller
    std::vector<tgRod*> rods = threeBarModel::find<tgRod>("rod");
    for (int i = 0; i < rods.size(); i++) {
        allRods.push_back(threeBarModel::find<tgRod>(tgString("rod num", i))[0]);    
    }
        
    // Get the actuators for controller
    std::vector<tgBasicActuator*> actuators = threeBarModel::find<tgBasicActuator>("actuator");
    for (int i = 0; i < rods.size(); i++) {
        allActuators.push_back(threeBarModel::find<tgBasicActuator>(tgString("actuator num", i))[0]);    
    }
    */
    //Controllers

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();
    
    btVector3 location(0,0,0);
    btVector3 rotation(0.0,0,0.0);
    btVector3 angular(0,0,60); //Rad/sec y is up.
    //btVector3 angular(30,0,0);
    this->moveModel(location,rotation,angular);

    // Actually setup the children
    tgModel::setup(world);
}

void TensegrityHedgehogModel12::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 angularVector)
{
    std::vector<tgBox *> boxes=find<tgBox>("box");

	btQuaternion initialRotationQuat;
	initialRotationQuat.setEuler(rotationVector[0],rotationVector[1],rotationVector[2]);
	btTransform initialTransform;
	initialTransform.setIdentity();
	initialTransform.setRotation(initialRotationQuat);
	initialTransform.setOrigin(positionVector);
	
	//getPRigidBody()->getLinearVelocity();
	
	for(int i=0;i<boxes.size();i++)
	{
			boxes[i]->getPRigidBody()->setAngularVelocity(angularVector);
			boxes[i]->getPRigidBody()->setWorldTransform(initialTransform * boxes[i]->getPRigidBody()->getWorldTransform());
		        boxes[i]->getPRigidBody()->getLinearVelocity();
	}
}

void TensegrityHedgehogModel12::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void TensegrityHedgehogModel12::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}
const std::vector<tgSpringCableActuator*>& TensegrityHedgehogModel12::getAllMuscles() const //controller
//const std::vector<tgBasicActuator*>& TensegrityHedgehogModel12::getAllMuscles() const
{
    return allMuscles;
}

std::vector<tgRod*>& TensegrityHedgehogModel12::getAllRods()
{
    return allRods;
}

void TensegrityHedgehogModel12::teardown()
{
    notifyTeardown();// obtaining COM
    tgModel::teardown();
}
           
// Return the center of mass of this model
// Pre-condition: This model has 12 rods

std::vector<double> TensegrityHedgehogModel12::getBallCOM() {   
    std::vector <tgRod*> rods = find<tgRod>("rod");
    assert(!rods.empty());
	
   //Calculate COM
    btVector3 ballCenterOfMass(0, 0, 0);
    double ballMass = 0.0; 
    for (std::size_t i = 0; i < rods.size(); i++) {   
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        ballCenterOfMass += rodCenterOfMass * rodMass;
        ballMass += rodMass;
    }

    assert(ballMass > 0.0);
    ballCenterOfMass /= ballMass;

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { result[i] = ballCenterOfMass[i]; }
    std::cout<<"COM: (" << result[0] << ", " << result[1] << ", " << result[2] << ")\n";
    return result;
}
/*
std::vector<double> TensegrityHedgehogModel12::Position() {
//Obtain Position of all objects
	for (int i=0;i<100;i++)
	{
		//print positions of all objects
		for (int j=TensegrityHedgehogModel12->getNumCollisionObjects()-1; j>=0 ;j--)
		{
			btCollisionObject* obj = TensegrityHedgehogModel12->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				btTransform trans;
				body->getMotionState()->getWorldTransform(trans);
				printf("world pos = %f,%f,%f\n",float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
			}
		}
	}
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { result[i] = body[i]; }
    std::cout<<"POS: (" << result[0] << ", " << result[1] << ", " << result[2] << ")\n";
    return result;
}
*/
/*
void TensegrityHedgehogModel12::displayText()
{
	sprintf(result);
}
*/
/*

	int lineWidth=440;
	int xStart = 20; // position
	int yStart = 20; //position

	if((getDebugMode() & btIDebugDraw::DBG_DrawText)!=0) //error getDebugMode()
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		glRasterPos3f(xStart, yStart, 0);
		switch (m_ccdMode)
		{
		case USE_CCD:
			{
				sprintf(buf,"Predictive contacts and motion clamping");
				break;
			}
		case USE_NO_CCD:
			{
				sprintf(buf,"CCD handling disabled");
				break;
			}
		default:
			{
				sprintf(buf,"unknown CCD setting");
			};
		};

		GLDebugDrawString(xStart,20,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"Press 'p' to change CCD mode");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"Press '.' or right mouse to shoot bullets");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"space to restart, h(elp), t(ext), w(ire)");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}	

}
*/
