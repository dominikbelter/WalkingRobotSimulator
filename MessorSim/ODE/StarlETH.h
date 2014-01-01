/** @file StarlETH.h
 *
 * StarlETH robot
 *
 */

#include "../SimDefs/simDefs.h"
#include "../math/punctum.h"
#include "../math/vector.h"
#include "../functions.h"
#include "IMU.h"
#include "SimRobot.h"

// some constants
#define DENSITY (0.5)        // density of all objects
#define MAX_CONTACTS 2048      // maximum number of contact points per body

#define PI 3.14159265

using namespace robsim;

static const robsim::float_type STARLETH_WIDTH = 0.185;  ///odleglosc od srodka robota do srodkowej nogi
static const robsim::float_type STARLETH_LENGTH = 0.2525;		  ///odleglosc nog przednich od srodka robota wzdluz osi podluznej 
//parametry nogi zmiany wymagaja rowniez parametry w leg.h
static const robsim::float_type STARLETH_SEGMENT1 = 0.0685; //lenght pierwszego segmentu
static const robsim::float_type STARLETH_SEGMENT2 = 0.2;  //lenght drugiego segmentu
static const robsim::float_type STARLETH_SEGMENT3 = 0.2350;  //lenght trzeciego segmentu
static const robsim::float_type STARLETH_FOOT_LENGTH = 0.02;  //dlugosc stopy
static const robsim::float_type STARLETH_OFFSET3 = 0.0;	//przsuniecie d w ostatnim jointie
static const robsim::float_type STARLETH_MAX_SERVO_SPEED = 12000.1568;	//przsuniecie d w ostatnim jointie

namespace robsim {
	/// create a single simulated robot (StarlETH)
	SimRobot* createSimStarlETH(void);
}

class StarlETH : public SimRobot {
public:
	/// Pointer
	typedef std::unique_ptr<StarlETH> Ptr;

	static const uint_fast8_t STARLETH_LEGS = 4;
	static const uint_fast8_t STARLETH_JOINTS_NO = 12;
	static const uint_fast8_t STARLETH_SERVOS_PER_LEG = 3;
	static const uint_fast8_t STARLETH_LINKS_NO = STARLETH_SERVOS_PER_LEG * STARLETH_LEGS + STARLETH_LEGS + 1;//+ body

	StarlETH(void);
	~StarlETH(void);

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
		return STARLETH_LINKS_NO;
	}

	/// Get legss number
	std::uint_fast8_t getLegsNo(void) const {
		return STARLETH_LEGS;
	}

private:
	/*-----*/
//////////ODE////////////////
	/// ODE - symulacja regulatora w serwomechanizmie
	void setServo(uint_fast8_t servoNo, robsim::float_type value);
	/// ODE ustawia tablice z pozycja i orientacja robota na podstawie stanu robota w ODE
	void setPositionSensors();
	/// Create ODE object
	void StarlETH::createODEObject(dSpaceID& space, uint_fast8_t objectId, robsim::float_type* rot, CPunctum& pose, robsim::float_type mass, robsim::float_type* sides);

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
	MyObject Object[STARLETH_LINKS_NO];
	/// ODE - tablica zlacz
	dJointID Joints[STARLETH_JOINTS_NO+STARLETH_LEGS];
	//IMU
	CIMU imu;
	char filtered_contact[6]; 
};

