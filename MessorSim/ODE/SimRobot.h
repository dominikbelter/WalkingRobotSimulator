/** @file SimRobot.h
 *
 * Simulated robot interface
 *
 */

#ifndef _SIMROBOT_H_
#define _SIMROBOT_H_

#include "../SimDefs/simDefs.h"
#include "../math/punctum.h"
#include <iostream>
#include <string>
#include <vector>

namespace robsim {

	///Simulated robot interface
	class SimRobot
	{
	public:
		/// overloaded constructor
		SimRobot(const std::string _name) : name(_name) {};

		/// Name of the robot
		virtual const std::string& getName() const = 0;

		/// Set initial pose of the robot
		virtual void setInitialPosition(robsim::float_type x, robsim::float_type y, robsim::float_type z, robsim::float_type alpha=0, robsim::float_type beta=0, robsim::float_type gamma=0) = 0;

		/// Set reference values for each leg
		virtual void setAngles(const std::vector<robsim::float_type>& _refAngles) = 0;

		/// set reference values for leg
		virtual void setLegAngles(uint_fast8_t legNo, const std::vector<robsim::float_type>& legAngles) = 0;

		/// set reference rotation speed
		virtual void setSpeed(const std::vector<robsim::float_type>& _refSpeed) = 0;

		/// set reference rotation speed
		virtual void setLegSpeed(uint_fast8_t legNo, const std::vector<robsim::float_type>& legSpeed) = 0;

		/// set max load
		virtual void setLoad(const std::vector<robsim::float_type>& maxLoad) = 0;

		/// set max load
		virtual void setLegLoad(uint_fast8_t legNo, const std::vector<robsim::float_type>& maxLoad) = 0;

		/// Get reference values for each leg
		virtual const std::vector<robsim::float_type>& getAngles(void) const = 0;

		/// Get reference values for leg
		virtual void getLegAngles(uint_fast8_t legNo, std::vector<robsim::float_type>& legAngles) const = 0;

		/// Get reference rotation speed
		virtual const std::vector<robsim::float_type>& getSpeed(void) const = 0;

		/// Get reference rotation speed
		virtual void getLegSpeed(uint_fast8_t legNo, std::vector<robsim::float_type>& legSpeed) const = 0;

		/// Get max load
		virtual const std::vector<robsim::float_type>& getLoad(void) const = 0;

		/// Get max load
		virtual void getLegLoad(uint_fast8_t legNo, std::vector<robsim::float_type>& legLoad) const = 0;
	
		/// read current joint positions
		virtual void readAngles(std::vector<robsim::float_type>& currentAngles) const = 0;

		/// read current joint positions
		virtual void readLegAngles(uint_fast8_t legNo, std::vector<robsim::float_type>& currentAngles) const = 0;

		/// read current joints speed
		virtual void readSpeed(std::vector<robsim::float_type>& currentSpeed) const = 0;

		/// read current joints speed
		virtual void readLegSpeed(uint_fast8_t legNo, std::vector<robsim::float_type>& currentSpeed) const = 0;

		/// read current joints speed
		virtual void readLoad(std::vector<robsim::float_type>& currentLoad) const = 0;

		/// read current joints speed
		virtual void readLegLoad(uint_fast8_t legNo, std::vector<robsim::float_type>& currentLoad) = 0;

		/// read reference value for servomotor (angle)
		virtual const robsim::float_type& getRefAngle(uint_fast8_t leg, uint_fast8_t joint) const = 0;

		/// read reference value for servomotor (speed)
		virtual const robsim::float_type& getRefSpeed(uint_fast8_t leg, uint_fast8_t joint) const = 0;

		/// read reference value for servomotor (load)
		virtual const robsim::float_type& getRefLoad(uint_fast8_t leg, uint_fast8_t joint) const = 0;

		/// get robot pose
		virtual const CPunctum getRobotState(void) = 0;
		
		/// get platform position
		virtual void getPosition(robsim::float_type (&position)[3]) = 0;

		/// get platform orientation (roll/pitch/yaw)
		virtual void getRPY(robsim::float_type (&rpy)[3]) = 0;

		/// feet position
		virtual void getFootPosition(uint_fast8_t foot, CPunctum& pose) const = 0;

		/// check if leg touches ground
		virtual bool getContact(uint_fast8_t legNo) const = 0;

		/// Set info about contact with ground
		virtual void setContact(uint_fast8_t legNo, bool state) = 0;

		/// set all servos using selected controller (P/PI/PID)
		virtual void setAllServos() = 0;

		/// create robot in ODE world
		virtual void ODEcreateRobot(dWorldID& world, dSpaceID& space, dJointGroupID& jointgroup, robsim::float_type dt) = 0;

		/// Get ODE geom id
		virtual const dGeomID getGeomId(uint_fast8_t partNo) const = 0;

		/// Virtual descrutor
		virtual ~SimRobot(void) {};

	protected:
		/// Robot name
		const std::string name;
		/// Pose of the robot's body
		CPunctum centre; 
		/// reference values (angles)
		std::vector<robsim::float_type> refAngles;
		/// reference values (angles)
		std::vector<robsim::float_type> refSpeed;
		/// max load
		std::vector<robsim::float_type> refLoad;
		/// contact with ground
		std::vector<robsim::float_type> groundContact;
	};
}

#endif // _SIMROBOT_H_
