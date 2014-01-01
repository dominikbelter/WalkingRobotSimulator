/** @file hexapodRobot.h
 *
 * Messor robot
 *
 */

#include "../SimDefs/simDefs.h"
#include "../math/punctum.h"
#include "../math/vector.h"
#include <ode/ode.h>
#include "../functions.h"
#include "IMU.h"
#include "SimRobot.h"

// some constants
#define DENSITY (0.5)        // density of all objects
#define MAX_CONTACTS 2048      // maximum number of contact points per body

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

	static const uint_fast8_t MESSOR_LEGS = 6;
	static const uint_fast8_t MESSOR_JOINTS_NO = 18 + MESSOR_LEGS;
	static const uint_fast8_t MESSOR_SERVOS_PER_LEG = 3;
	static const uint_fast8_t MESSOR_LINKS_NO = MESSOR_SERVOS_PER_LEG * MESSOR_LEGS + MESSOR_LEGS + 1;//+ body

	HexRobot(void);
	~HexRobot(void);
		
		/// Set initial pose of the robot
		void setInitialPosition(robsim::float_type x, robsim::float_type y, robsim::float_type z, robsim::float_type alpha=0, robsim::float_type beta=0, robsim::float_type gamma=0);

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

		/// set all servos using selected controller (P/PI/PID)
		void setAllServos();

		/// create robot in ODE world
		void ODEcreateRobot(dWorldID& world, dSpaceID& space, dJointGroupID& jointgroup, robsim::float_type dt);

		/// Get ODE geom id
		const dGeomID getGeomId(uint_fast8_t partNo) const;

		/// Check if considered parts should collide
		bool collide(dBodyID& b1, dBodyID& b2) const;

		/// Set info about ODE collisions
		void setODEContacts(dBodyID& b1, dBodyID& b2, dBodyID& groundId);

		/// Get objects number
		std::uint_fast8_t getObjectsNo(void) const {
			return MESSOR_LINKS_NO;
		}

		/// Get legss number
		std::uint_fast8_t getLegsNo(void) const {
			return MESSOR_LEGS;
		}

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
	char filtered_contact[6]; 
};
