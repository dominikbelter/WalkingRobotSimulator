#pragma once
#include "../SimDefs/simDefs.h"
#include "../math/punctum.h"
#include "../math/vector.h"
#include <ode/ode.h>
#include "../functions.h"
#include "IMU.h"
#include "SimRobot.h"

// some constants
#define DENSITY (0.5)        // density of all objects
#define GEOMSPERBODY 1       // maximum number of geometries per body
#define MAX_CONTACTS 2048      // maximum number of contact points per body

typedef struct MyObject //struktura reprezentujaca obiekty
{
	dBodyID Body;                     // the dynamics body
	dGeomID Geom[GEOMSPERBODY];       // geometries representing this body
} MyObject;

#define PI 3.14159265
const double width_max = 0.13;  ///odleglosc od srodka robota do srodkowej nogi
const double width_min = 0.065;  ///odleglosc od osi podluznej robota do przedniej nogi
const double lenght = 0.153289;		  ///odleglosc nog przednich od srodka robota wzdluz osi podluznej 
//parametry nogi zmiany wymagaja rowniez parametry w leg.h
const double segment1=0.055; //lenght pierwszego segmentu
const double segment2=0.16;  //lenght drugiego segmentu
const double segment3=0.230;  //lenght trzeciego segmentu
const double foot_length=0.02;  //dlugosc stopy
const double offset3=0.0;	//przsuniecie d w ostatnim jointie
const double max_servo_speed=12000.1568;	//przsuniecie d w ostatnim jointie

using namespace robsim;

namespace robsim {
	/// create a single simulated robot (Hexapod)
	SimRobot* createSimRobotHexapod(void);
}

class HexRobot : public SimRobot
{
	//funkcje
public:
	/// Pointer
	typedef std::unique_ptr<HexRobot> Ptr;

	HexRobot(void);
	~HexRobot(void);
		
		/// Name of the robot
		const std::string& getName() const;

		/// Set initial pose of the robot
		void setInitialPosition(robsim::float_type x, robsim::float_type y, robsim::float_type z, robsim::float_type alpha=0, robsim::float_type beta=0, robsim::float_type gamma=0);

		/// Set reference values for each leg
		void setAngles(const std::vector<robsim::float_type>& _refAngles);

		/// set reference values for leg
		void setLegAngles(uint_fast8_t legNo, const std::vector<robsim::float_type>& legAngles);

		/// set reference rotation speed
		void setSpeed(const std::vector<robsim::float_type>& _refSpeed);

		/// set reference rotation speed
		void setLegSpeed(uint_fast8_t legNo, const std::vector<robsim::float_type>& legSpeed);

		/// set max load
		void setLoad(const std::vector<robsim::float_type>& maxLoad);

		/// set max load
		void setLegLoad(uint_fast8_t legNo, const std::vector<robsim::float_type>& maxLoad);
	
		/// Get reference values for each leg
		const std::vector<robsim::float_type>& getAngles(void) const;

		/// Get reference values for leg
		void HexRobot::getLegAngles(uint_fast8_t legNo, std::vector<robsim::float_type>& legAngles) const;

		/// Get reference rotation speed
		const std::vector<robsim::float_type>& getSpeed(void) const;

		/// Get reference rotation speed
		void HexRobot::getLegSpeed(uint_fast8_t legNo, std::vector<robsim::float_type>& legSpeed) const;

		/// Get max load
		const std::vector<robsim::float_type>& getLoad(void) const;

		/// Get max load
		void HexRobot::getLegLoad(uint_fast8_t legNo, std::vector<robsim::float_type>& legLoad) const;

		/// read current joint positions
		void readAngles(std::vector<robsim::float_type>& currentAngles) const;

		/// read current joint positions
		void readLegAngles(uint_fast8_t legNo, std::vector<robsim::float_type>& currentAngles) const;

		/// read current joints speed
		void readSpeed(std::vector<robsim::float_type>& currentSpeed) const;

		/// read current joints speed
		void readLegSpeed(uint_fast8_t legNo, std::vector<robsim::float_type>& currentSpeed) const;

		/// read current joints speed
		void readLoad(std::vector<robsim::float_type>& currentLoad) const;

		/// read current joints speed
		void readLegLoad(uint_fast8_t legNo, std::vector<robsim::float_type>& currentLoad);

		/// read reference value for servomotor (angle)
		const robsim::float_type& getRefAngle(uint_fast8_t leg, uint_fast8_t joint) const;

		/// read reference value for servomotor (speed)
		const robsim::float_type& getRefSpeed(uint_fast8_t leg, uint_fast8_t joint) const;

		/// read reference value for servomotor (load)
		const robsim::float_type& getRefLoad(uint_fast8_t leg, uint_fast8_t joint) const;

		/// get robot pose
		const CPunctum getRobotState(void);
		
		/// get platform position
		void getPosition(robsim::float_type (&position)[3]);

		/// get platform orientation (roll/pitch/yaw)
		void getRPY(robsim::float_type (&rpy)[3]);

		/// feet position
		void getFootPosition(uint_fast8_t foot, CPunctum& pose) const;

		/// check if leg touches ground
		bool getContact(uint_fast8_t legNo) const;

		/// Set info about contact with ground
		void setContact(uint_fast8_t legNo, bool state);

		/// set all servos using selected controller (P/PI/PID)
		void setAllServos();

		/// create robot in ODE world
		void ODEcreateRobot(dWorldID& world, dSpaceID& space, dJointGroupID& jointgroup, robsim::float_type dt);

		/// Get ODE geom id
		const dGeomID getGeomId(uint_fast8_t partNo) const;

private:
	/*-----*/
//////////ODE////////////////
	/// ODE - symulacja regulatora w serwomechanizmie
	void setServo(uint_fast8_t servoNo, robsim::float_type value);
	/// ODE ustawia tablice z pozycja i orientacja robota na podstawie stanu robota w ODE
	void setPositionSensors();

public:
	/** nogi robota (leg[0] - pierwsza noga)
                /\		
	           /||\
                ||
	      6 o ----- o 1 
	       /  o   o  \
	      /     |     \
	   5 o     \__/    o 2
          \           /
	       \         / 
	      4 o ----- o 3
	
	*/
	/// ODE - tablica obiektow, z ktorych sklada sie robot
	MyObject Object[25];
	/// ODE - tablica zlacz
	dJointID Joints[24];
	//IMU
	CIMU imu;
	/// styczniki
	char contact[6];
	char filtered_contact[6]; 
};
