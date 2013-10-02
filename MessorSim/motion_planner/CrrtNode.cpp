#include "CrrtNode.h"

CrrtNode::CrrtNode(void)
{
}

CrrtNode::~CrrtNode(void)
{
}

CrrtNode::CrrtNode(float x, float y, float z, float alpha, float beta, float gamma){
	pos[0]=x; pos[1]=y; pos[2]=z;
	rot[0]=alpha; rot[1]=beta; rot[2]=gamma;
}

CrrtNode::CrrtNode(float * position, float * orientation){
	for (int i=0; i<3;i++){
		pos[i] = position[i];
		rot[i] = orientation[i];
	}
}

///set node index
void CrrtNode::SetNodeIndex(int index){
	node_index = index;
}

///get node index
int CrrtNode::GetNodeIndex(){
	return node_index;
}

///set parent index
void CrrtNode::SetParentIndex(int index){
	parent_index = index;
}

///get parent index
int CrrtNode::GetParentIndex(){
	return parent_index;
}

/// rand ronfiguration
void CrrtNode::randomConfig(float min_x, float max_x, float min_y, float max_y){
	this->pos[0] = randFloat(min_x, max_x);
	this->pos[1] = randFloat(min_y, max_y);
}

/// draw node
void CrrtNode::drawNode(double red, double green, double blue, double thickness, const char marker){
	glColor3f(red, green, blue);
	if (marker=='o') {
		glPushMatrix();
		 glTranslatef(body.getElement(1,4)*10, body.getElement(3,4)*10, -body.getElement(2,4)*10);
		 glutSolidSphere(.02*thickness,5,5);
		glPopMatrix();
		/*for (int i=0;i<6;i++){
			glPushMatrix();
			 glTranslatef(foots[i].getElement(1,4)*10, foots[i].getElement(3,4)*10, -foots[i].getElement(2,4)*10);
			 glutSolidSphere(.02*thickness,5,5);
			glPopMatrix();			
		}*/
	}
	/*for (int j=0;j<6;j++){
		glBegin(GL_LINE_STRIP);
		for (int i=0;i<30;i++){
			if ((abs(legs_traj[j][i].getElement(1,4))>0.001)&&(legs_traj[j][i].getElement(1,4)<1)&&(legs_traj[j][i].getElement(1,4)>-1)){
				glVertex3f((legs_traj[j][i]).getElement(1,4)*10, (legs_traj[j][i]).getElement(3,4)*10, -(legs_traj[j][i]).getElement(2,4)*10);
			}
		}
		glEnd();
	}*/
}
