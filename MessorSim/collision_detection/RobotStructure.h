/** @file RobotStructure.h
 *
 * Mechanical structure and collision detection interface
 *
 */

#ifndef _ROBOTSTRUCTURE_H_
#define _ROBOTSTRUCTURE_H_

#include "../SimDefs/simDefs.h"
#include "../math/punctum.h"
#include "coldet.h"
#include "3dsloader.h"
#include "objects3DS.h"
#include "gl/glut.h"

namespace robsim {
	///Simulated robot interface
	class RobotStructure {
	public:
		/// overloaded constructor
		RobotStructure(const std::string _name) : name(_name) {};

		/// Name of the structure
		virtual const std::string& getName() const {
			return name;
		}

		/// Initialize robot structure
		virtual void init_structures(void) = 0;

		/// Draw robot using openGL
		virtual void GLDrawRobot(robsim::float_type *pos, robsim::float_type * rot, std::vector<robsim::float_type> config) const = 0;

		/// Check collisions
		virtual bool checkCollision(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles, bool * collision_table) const = 0;

		/// Virtual descrutor
		virtual ~RobotStructure(void) {};

	protected:
		/// Robot structure name
		const std::string name;
	};
}

#endif // _ROBOTSTRUCTURE_H_
