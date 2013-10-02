#include "leg.h"
#include <math.h>
#include <stdio.h>
#include <float.h>

CLeg::CLeg(double x,double y, double z,double alpha,double beta,double gamma)
{
}

CLeg::CLeg(void)
{

	for (int i=0; i<3;i++){
		joint[i].setAngle(0);
	}
}
CLeg::~CLeg(void)
{
}

/// stawia pozycje poczatkowa
void CLeg::reset(double x, double y, double z, double alpha, double beta, double gamma)
{
}

/// ustawia wartosci katow w stawach
void CLeg::setAngle(unsigned char joint_number,double value)
{
		prev_joint[joint_number].setAngle(getAngle(joint_number));
		joint[joint_number].setAngle(value);
}

/// pobiera wartosc kata
double CLeg::getAngle(unsigned char joint_number)
{
	return joint[joint_number].getAngle();
}

/// zwraca poprzedni kat z wstawie nr_joint
double CLeg::getPreviousAngle(unsigned char joint_number)
{
	return prev_joint[joint_number].getAngle();
}

/// ustawia aktualna(poprzednia jezeli ustawiamy nowa) wartosc kata w stawie
double CLeg::setPreviousAngle(unsigned char nr_joint, float value){
	prev_joint[nr_joint].setAngle(value);
	return 0;
}

/// ustawia stan stopy
void CLeg::setFoot(bool contact_value)
{
	foot.setContact(contact_value);
}

/// pobiera stan stycznika
bool CLeg::getContact(void)
{
	return foot.getContact();
}

/// funkcja obliczajaca kinematyke prosta nogi, katy podawane w radianach
CPunctum CLeg::forward_kinematic(double theta1, double theta2, double theta3, int part)
{
	signed char s;
	if(part == 1){
		s=1;
	}
	else{
		s=-1;
	}

	double theta_4 = (0.0*PI)/180;

	CPunctum a1,a2,a3,a4;
//macierz a1
	a1.setElement(cos(theta1),1,1);
	a1.setElement(-sin(theta1),1,2);
	a1.setElement(0.0,1,3);
	a1.setElement(0.0,1,4);
	
	a1.setElement(sin(theta1),2,1);
	a1.setElement(cos(theta1),2,2);
	a1.setElement(0.0,2,3);
	a1.setElement(0.0,2,4);
	
	a1.setElement(0.0,3,1);
	a1.setElement(0.0,3,2);
	a1.setElement(1.0,3,3);
	a1.setElement(0.0,3,4);

	a1.setElement(0.0,4,1);
	a1.setElement(0.0,4,2);
	a1.setElement(0.0,4,3);
	a1.setElement(1.0,4,4);
//macierz a2
	a2.setElement(cos(theta2),1,1);
	a2.setElement(-sin(theta2),1,2);
	a2.setElement(0.0,1,3);
	a2.setElement(segment_1,1,4);
	
	a2.setElement(0.0,2,1);
	a2.setElement(0.0,2,2);
	a2.setElement(-1.0*s,2,3);
	a2.setElement(0.0,2,4);
	
	a2.setElement(sin(theta2)*s,3,1);
	a2.setElement(cos(theta2)*s,3,2);
	a2.setElement(0.0,3,3);
	a2.setElement(0.0,3,4);

	a2.setElement(0.0,4,1);
	a2.setElement(0.0,4,2);
	a2.setElement(0.0,4,3);
	a2.setElement(1.0,4,4);
//macierz a3
	a3.setElement(cos(theta3),1,1);
	a3.setElement(-sin(theta3),1,2);
	a3.setElement(0.0,1,3);
	a3.setElement(segment_2,1,4);
	
	a3.setElement(sin(theta3),2,1);
	a3.setElement(cos(theta3),2,2);
	a3.setElement(0.0,2,3);
	a3.setElement(0.0,2,4);
	
	a3.setElement(0.0,3,1);
	a3.setElement(0.0,3,2);
	a3.setElement(1.0,3,3);
	a3.setElement(0.0,3,4);

	a3.setElement(0.0,4,1);
	a3.setElement(0.0,4,2);
	a3.setElement(0.0,4,3);
	a3.setElement(1.0,4,4);
//macierz a4
	a4.setElement(1.0,1,1);
	a4.setElement(0.0,1,2);
	a4.setElement(0.0,1,3);
	a4.setElement(segment_3,1,4);
	
	a4.setElement(0.0,2,1);
	a4.setElement(1.0,2,2);
	a4.setElement(0.0,2,3);
	a4.setElement(0.0,2,4);
	
	a4.setElement(0.0,3,1);
	a4.setElement(0.0,3,2);
	a4.setElement(1.0,3,3);
	a4.setElement(s*offset_3,3,4);

	a4.setElement(0.0,4,1);
	a4.setElement(0.0,4,2);
	a4.setElement(0.0,4,3);
	a4.setElement(1.0,4,4);

	a1=a1*a2*a3*a4;
	return a1;
}

/// oblicza srodek masy konczyny w ukladzie konczyny 
CPunctum CLeg::computeCenterOfMass(float * angles, int part){
	CPunctum temp;
	temp.setIdentity();

	//masy poszczegolnych elementow
	float mass[3] = {0.134,0.108,0.163};
	signed char s;
	if(part == 1) s=1; else s = 0;

	CPunctum a1,a2,a3,a4;
//macierz a1
	a1.setElement(cos(angles[0]),1,1);	a1.setElement(-sin(angles[0]),1,2); a1.setElement(0.0,1,3); a1.setElement(0.0,1,4);
	a1.setElement(sin(angles[0]),2,1);	a1.setElement(cos(angles[0]),2,2);	a1.setElement(0.0,2,3); a1.setElement(0.0,2,4);
	a1.setElement(0.0,3,1); a1.setElement(0.0,3,2); a1.setElement(1.0,3,3); a1.setElement(0.0,3,4); a1.setElement(0.0,4,1);
	a1.setElement(0.0,4,2); a1.setElement(0.0,4,3); a1.setElement(1.0,4,4);
//macierz a2
	a2.setElement(cos(angles[1]),1,1);	a2.setElement(-sin(angles[1]),1,2); a2.setElement(0.0,1,3); a2.setElement(segment_1,1,4);
	a2.setElement(0.0,2,1); a2.setElement(0.0,2,2); a2.setElement(-1.0*s,2,3); a2.setElement(0.0,2,4);
	a2.setElement(sin(angles[1])*s,3,1); a2.setElement(cos(angles[1])*s,3,2); a2.setElement(0.0,3,3); a2.setElement(0.0,3,4);
	a2.setElement(0.0,4,1); a2.setElement(0.0,4,2); a2.setElement(0.0,4,3); a2.setElement(1.0,4,4);
//macierz a3
	a3.setElement(cos(angles[2]),1,1);	a3.setElement(-sin(angles[2]),1,2); a3.setElement(0.0,1,3); a3.setElement(segment_2,1,4);
	a3.setElement(sin(angles[2]),2,1);	a3.setElement(cos(angles[2]),2,2); a3.setElement(0.0,2,3); a3.setElement(0.0,2,4);	
	a3.setElement(0.0,3,1); a3.setElement(0.0,3,2); a3.setElement(1.0,3,3); a3.setElement(0.0,3,4); 
	a3.setElement(0.0,4,1); a3.setElement(0.0,4,2); a3.setElement(0.0,4,3); a3.setElement(1.0,4,4);

	CPunctum m1;//polozenie srodka masy czlonu 1 w ukladzie zwiazanym z pierwszym ogniwem
	m1.setIdentity();
	m1.setElement(segment_1/2.0,1,4);
	m1=a1*m1;//polozenie masy ogniwa m1 w ukladzie nogi

	CPunctum m2;//polozenie srodka masy czlonu 1 w ukladzie zwiazanym z pierwszym ogniwem
	m2.setIdentity();
	m2.setElement(segment_2/2.0,1,4);
	m2=a1*a2*m2;//polozenie masy ogniwa m1 w ukladzie nogi

	CPunctum m3;//polozenie srodka masy czlonu 1 w ukladzie zwiazanym z pierwszym ogniwem
	m3.setIdentity();
	m3.setElement(segment_2/2.0,1,4);
	m3=a1*a2*a3*m3;//polozenie masy ogniwa m1 w ukladzie nogi

	temp.setElement((mass[0]*m1.getElement(1,4)+mass[1]*m2.getElement(1,4)+mass[2]*m3.getElement(1,4))/(mass[0]+mass[1]+mass[2]),1,4);
	temp.setElement((mass[0]*m1.getElement(2,4)+mass[1]*m2.getElement(2,4)+mass[2]*m3.getElement(1,4))/(mass[0]+mass[1]+mass[2]),2,4);
	temp.setElement((mass[0]*m1.getElement(3,4)+mass[1]*m2.getElement(3,4)+mass[2]*m3.getElement(1,4))/(mass[0]+mass[1]+mass[2]),3,4);
	return temp;
}

/// kinematyka odwrotna dla nogi
bool CLeg::inverse_kinematic(double x, double y, double z,int part, double angles[], float scale)
{
	signed char s;
	if(part == 1){
		s=1;
	}
	else {
		s=-1;
	} 
	
	double r=sqrt(pow(x,2)+pow(y,2));
	angles[0]=atan2(y,x);
	double c=sqrt(pow((r-(segment_1*scale)),2)+pow(z,2));
	double e=(pow(c,2)-pow(segment_2*scale,2)-pow(segment_3*scale,2))/(2*(segment_2*scale)*(segment_3*scale));
	angles[2]=atan2(-sqrt(1-pow(e,2)),e);
	double f=(segment_3*scale)*sin(-angles[2]*s);
	double g=(segment_3*scale)*cos(-angles[2]*s)+(segment_2*scale);
	angles[1]=s*(atan2(z,r-(segment_1*scale))+atan2(f,g));
	//if ((angles[2]<-3.31)||(angles[2]>-0.17)||(angles[1]>1.4)||(angles[1]<-1.57)||abs(angles[0])>1.57||(_isnan(angles[0]))||(_isnan(angles[1]))||(_isnan(angles[2])))
	//if ((angles[1]>1.4)||abs(angles[0])>1.57||(_isnan(angles[0]))||(_isnan(angles[1]))||(_isnan(angles[2])))
	if ((_isnan(angles[0]))||(_isnan(angles[1]))||(_isnan(angles[2])))
		return false;
	else
		return true;
}

/// kinematyka odwrotna dla nogi
bool CLeg::inverse_kinematic(double x, double y, double z,int part, float angles[], float scale)
{
	signed char s;
	if(part == 1){
		s=1;
	}
	else {
		s=-1;
	} 
	
	double r=sqrt(pow(x,2)+pow(y,2));
	angles[0]=atan2(y,x);
	double c=sqrt(pow((r-(segment_1*scale)),2)+pow(z,2));
	double e=(pow(c,2)-pow((segment_2*scale),2)-pow((segment_3*scale),2))/(2*(segment_2*scale)*(segment_3*scale));
	angles[2]=atan2(-sqrt(1-pow(e,2)),e);
	double f=(segment_3*scale)*sin(-angles[2]*s);
	double g=(segment_3*scale)*cos(-angles[2]*s)+(segment_2*scale);
	angles[1]=s*(atan2(z,r-(segment_1*scale))+atan2(f,g));
	//if ((angles[2]<-3.31)||(angles[2]>-0.17)||(angles[1]>1.4)||(angles[1]<-1.57)||abs(angles[0])>1.57||(_isnan(angles[0]))||(_isnan(angles[1]))||(_isnan(angles[2])))
	if ((angles[1]>1.4)||abs(angles[0])>1.57||(_isnan(angles[0]))||(_isnan(angles[1]))||(_isnan(angles[2])))
		return false;
	else
		return true;
}

/// przesuwa stope o zadana odleglosc w ukladzie nogi
bool CLeg::moveFoot(double x, double y, double z, int part) 
{
	float delta_angle[3];
	if (!computeMoveDeltaAngle(x, y, z, part, delta_angle))
		return false;
    // wpisanie wartosci zadanych
	joint[0].setAngle(joint[0].getAngle() + delta_angle[0]);
	joint[1].setAngle(joint[1].getAngle() + delta_angle[1]);
	joint[2].setAngle(joint[2].getAngle() + delta_angle[2]);
	return true;
}

/// obliczenie zmiany kata w stawach przy przesunieciu stopy o zadana odleglosc w ukladzie nogi
bool CLeg::computeMoveDeltaAngle(double x, double y, double z, int part, float * delta_angle) 
{
	double start_angle[3]={joint[0].getAngle(),joint[1].getAngle(),joint[2].getAngle()};
	double end_angle[3];

	// obliczenie aktualnej pozycji stopy w ukladzie stopy
	CPunctum punkt_poczatkowy = forward_kinematic(joint[0].getAngle(),joint[1].getAngle(),joint[2].getAngle(),part);
	CPunctum punkt_koncowy;
	punkt_koncowy.createTRMatrix(0,0,0,x,y,z);

	//przesuniecie punktu o zadan wartosc
	punkt_koncowy = punkt_koncowy + punkt_poczatkowy;

	//obliczenie katow w stawach, ktore przeniosa stope do zadanego punktu
	if (!inverse_kinematic(punkt_koncowy.getElement(1,4),punkt_koncowy.getElement(2,4),punkt_koncowy.getElement(3,4), part, end_angle))
		return false;

    // wpisanie wartosci zadanych
	delta_angle[0] = end_angle[0] - start_angle[0];
	delta_angle[1] = end_angle[1] - start_angle[1];
	delta_angle[2] = end_angle[2] - start_angle[2];
	return true;
}

/// obliczenie zmiany kata w stawach przy zmianie wartosci zadanych
void CLeg::computeMoveDeltaAngle(double alpha, double beta, double gamma, float * delta_angle)
{
	double start_angle[3]={joint[0].getAngle(),joint[1].getAngle(),joint[2].getAngle()};

    // wpisanie wartosci zadanych
	delta_angle[0] = alpha - start_angle[0];
	delta_angle[1] = beta - start_angle[1];
	delta_angle[2] = gamma - start_angle[2];
}

//podaje pozycje danej stopy
CPunctum CLeg::getPosition(int part)
{
	int _side;
	if (part==1)
		_side = 1;
	else
		_side = 0;
	CPunctum position;
	return position=forward_kinematic(joint[0].getAngle(),joint[1].getAngle(),joint[2].getAngle(),_side);

}

//podaje pozycje neutralna danej stopy
CPunctum CLeg::getNeutralPosition(int part)
{
	int _side;
	if (part==1)
		_side = 1;
	else
		_side = 0;
	CPunctum position;
	return position=forward_kinematic(zero_angle[0],zero_angle[1],zero_angle[2],_side);

}

//ustawia pozycje stopy w w ukladzie stopy
bool CLeg::setFootPosition(double x,double y, double z,int part)
{
	CPunctum point;
	int _side;
	if (part==1)
		_side = 1;
	else
		_side = -1;
	point.createTRMatrix(0,0,0,x,y,z);
	double temp_angle[3];
	if (!inverse_kinematic(point.getElement(1,4),point.getElement(2,4),point.getElement(3,4), _side, temp_angle))
		return false;

    // wpisanie wartosci zadanych
	joint[0].setAngle(temp_angle[0]);
	joint[1].setAngle(temp_angle[1]);
	joint[2].setAngle(temp_angle[2]);
	return true;
}

float CLeg::computeKinematicMargin(double alpha,double beta, double gamma,int part,int up){
	CPunctum foot = forward_kinematic(alpha, beta, gamma,part);
	return computeKinematicMarginPos(foot.getElement(1,4),foot.getElement(2,4),foot.getElement(3,4),part,up);
}

float CLeg::computeKinematicMarginApprox(double alpha,double beta, double gamma,int part,int up){
	float in[3];
	in[0]=alpha; in[1]=beta; in[2]=gamma;
	if (part==1)
		return +0.213997*exp(-(0.882840*pow(in[0]-(-0.524747),2.0)+0.755944*pow(in[1]-(-0.186396),2.0)+3.117294*pow(in[2]-(-2.143134),2.0)))+0.300682*exp(-(0.330664*pow(in[0]-(-0.665568),2.0)+0.615100*pow(in[1]-(-0.987683),2.0)+3.884737*pow(in[2]-(-2.773023),2.0)))-0.110149*exp(-(0.677266*pow(in[0]-(-0.377686),2.0)+0.905046*pow(in[1]-(-0.923140),2.0)+0.867287*pow(in[2]-(-1.965722),2.0)))+1.610011*exp(-(0.406056*pow(in[0]-(1.449802),2.0)+0.924338*pow(in[1]-(0.615054),2.0)+2.631372*pow(in[2]-(-1.620659),2.0)))-0.651399*exp(-(0.625780*pow(in[0]-(-0.047113),2.0)+0.795551*pow(in[1]-(0.240753),2.0)+0.485909*pow(in[2]-(-1.430491),2.0)))-0.104554*exp(-(1.015587*pow(in[0]-(0.097852),2.0)+0.924338*pow(in[1]-(-0.096307),2.0)+2.634845*pow(in[2]-(-1.336492),2.0)))+0.397274*exp(-(0.697621*pow(in[0]-(-0.812596),2.0)+0.900124*pow(in[1]-(0.492083),2.0)+3.046304*pow(in[2]-(-1.392228),2.0)))-0.031675*exp(-(0.620738*pow(in[0]-(-0.014557),2.0)+0.000000*pow(in[1]-(-0.350883),2.0)+0.000000*pow(in[2]-(-2.112527),2.0)))-0.182713*exp(-(0.829855*pow(in[0]-(0.309434),2.0)+0.924338*pow(in[1]-(-0.722190),2.0)+1.151106*pow(in[2]-(-1.764135),2.0)))+0.707834*exp(-(0.369130*pow(in[0]-(-0.107970),2.0)+0.924338*pow(in[1]-(0.498493),2.0)+2.960940*pow(in[2]-(-1.950277),2.0)))+0.088938*exp(-(0.630979*pow(in[0]-(0.610305),2.0)+0.359513*pow(in[1]-(0.062955),2.0)+1.509057*pow(in[2]-(-2.424324),2.0)))+0.280805*exp(-(1.111776*pow(in[0]-(0.194326),2.0)+0.697669*pow(in[1]-(-0.504376),2.0)+0.000000*pow(in[2]-(-0.565708),2.0)))+0.062576*exp(-(0.918662*pow(in[0]-(0.908989),2.0)+0.924338*pow(in[1]-(0.388785),2.0)+2.385224*pow(in[2]-(-0.916824),2.0)))+0.600881*exp(-(0.616652*pow(in[0]-(0.223974),2.0)+0.563531*pow(in[1]-(-1.037436),2.0)+0.965809*pow(in[2]-(-1.128416),2.0)))-0.437047*exp(-(0.467316*pow(in[0]-(-0.636741),2.0)+0.924338*pow(in[1]-(0.286006),2.0)+2.651147*pow(in[2]-(-1.294018),2.0)))+0.145895*exp(-(0.794294*pow(in[0]-(0.769212),2.0)+0.420335*pow(in[1]-(-0.847688),2.0)+0.957649*pow(in[2]-(-3.120434),2.0)))+0.347161*exp(-(0.898852*pow(in[0]-(-0.655287),2.0)+0.924338*pow(in[1]-(-1.015638),2.0)+1.834364*pow(in[2]-(-1.953313),2.0)))+1.578501*exp(-(0.619639*pow(in[0]-(0.942838),2.0)+0.427792*pow(in[1]-(0.172809),2.0)+1.532078*pow(in[2]-(-1.305233),2.0)))+0.085384*exp(-(0.909438*pow(in[0]-(-0.880483),2.0)+0.484699*pow(in[1]-(0.057310),2.0)+0.000000*pow(in[2]-(-3.073606),2.0)))-0.257953*exp(-(0.377792*pow(in[0]-(-1.255229),2.0)+0.924338*pow(in[1]-(0.316567),2.0)+0.896972*pow(in[2]-(-2.477396),2.0)))-1.067138*exp(-(0.571148*pow(in[0]-(0.676919),2.0)+0.924338*pow(in[1]-(0.409365),2.0)+1.244223*pow(in[2]-(-1.200015),2.0)))-0.229655*exp(-(0.852933*pow(in[0]-(0.938558),2.0)+0.536825*pow(in[1]-(-0.055186),2.0)+3.109625*pow(in[2]-(-2.214594),2.0)))-1.195072*exp(-(0.561142*pow(in[0]-(-0.086696),2.0)+0.478329*pow(in[1]-(-0.792227),2.0)+2.118772*pow(in[2]-(-2.346383),2.0)))-0.419293*exp(-(0.924598*pow(in[0]-(-0.282718),2.0)+0.610392*pow(in[1]-(0.319501),2.0)+3.229387*pow(in[2]-(-2.379881),2.0)))+0.127864*exp(-(0.584074*pow(in[0]-(0.598841),2.0)+0.417120*pow(in[1]-(-0.748580),2.0)+0.130779*pow(in[2]-(-1.566968),2.0)))-0.341137*exp(-(0.938453*pow(in[0]-(-1.060047),2.0)+0.651607*pow(in[1]-(-0.429546),2.0)+2.663859*pow(in[2]-(-2.038738),2.0)))-0.419604*exp(-(0.278523*pow(in[0]-(0.263160),2.0)+0.558362*pow(in[1]-(1.149084),2.0)+1.358759*pow(in[2]-(-1.384453),2.0)))-0.393830*exp(-(0.776008*pow(in[0]-(0.099520),2.0)+0.924338*pow(in[1]-(-0.680959),2.0)+2.296762*pow(in[2]-(-0.913146),2.0)))-0.594292*exp(-(1.067449*pow(in[0]-(0.439673),2.0)+0.826884*pow(in[1]-(0.169439),2.0)+2.517710*pow(in[2]-(-1.535809),2.0)))-0.700068*exp(-(0.548426*pow(in[0]-(0.392423),2.0)+0.240719*pow(in[1]-(-0.824391),2.0)+0.919693*pow(in[2]-(-1.092054),2.0)))+0.179723*exp(-(1.111776*pow(in[0]-(1.203684),2.0)+0.566460*pow(in[1]-(-0.159323),2.0)+0.693903*pow(in[2]-(-0.535330),2.0)))-0.621266*exp(-(0.591272*pow(in[0]-(0.135962),2.0)+0.924338*pow(in[1]-(-0.207174),2.0)+2.746204*pow(in[2]-(-1.312724),2.0)))-0.722641*exp(-(1.111776*pow(in[0]-(-0.363904),2.0)+0.789996*pow(in[1]-(0.307051),2.0)+1.923428*pow(in[2]-(-1.643238),2.0)))-0.457645*exp(-(0.637645*pow(in[0]-(-0.294714),2.0)+0.607188*pow(in[1]-(0.213330),2.0)+2.890744*pow(in[2]-(-0.524143),2.0)))+0.353690*exp(-(0.752083*pow(in[0]-(-0.535878),2.0)+0.924338*pow(in[1]-(-0.026922),2.0)+0.000000*pow(in[2]-(-2.249166),2.0)))-0.350588*exp(-(0.816411*pow(in[0]-(1.104572),2.0)+0.841650*pow(in[1]-(-0.614293),2.0)+1.520169*pow(in[2]-(-1.250434),2.0)))-0.323264*exp(-(0.651470*pow(in[0]-(0.094063),2.0)+0.579720*pow(in[1]-(0.392419),2.0)+3.144178*pow(in[2]-(-0.894157),2.0)))-0.313593*exp(-(1.020683*pow(in[0]-(0.475411),2.0)+0.845041*pow(in[1]-(-1.562088),2.0)+1.494954*pow(in[2]-(-1.506768),2.0)))-0.229779*exp(-(0.109556*pow(in[0]-(-0.543000),2.0)+0.491799*pow(in[1]-(0.223802),2.0)+1.393585*pow(in[2]-(-1.581835),2.0)))+0.015713*exp(-(0.334882*pow(in[0]-(1.430991),2.0)+0.734480*pow(in[1]-(-0.741722),2.0)+0.000000*pow(in[2]-(-2.049758),2.0)))-0.633893*exp(-(0.664428*pow(in[0]-(0.065897),2.0)+0.924338*pow(in[1]-(-0.714270),2.0)+0.153635*pow(in[2]-(-0.559921),2.0)))+0.203376*exp(-(0.613202*pow(in[0]-(-0.200571),2.0)+0.924338*pow(in[1]-(-0.364899),2.0)+1.329709*pow(in[2]-(-1.560153),2.0)))+0.788977*exp(-(0.681376*pow(in[0]-(-0.177992),2.0)+0.720938*pow(in[1]-(0.079685),2.0)+1.194513*pow(in[2]-(-1.969813),2.0)))+0.596391*exp(-(0.594109*pow(in[0]-(-0.365166),2.0)+0.784595*pow(in[1]-(-0.406772),2.0)+3.049528*pow(in[2]-(-2.250274),2.0)))-0.499167*exp(-(0.791703*pow(in[0]-(-1.142241),2.0)+0.677858*pow(in[1]-(0.382244),2.0)+2.513755*pow(in[2]-(-2.255160),2.0)))-0.034304*exp(-(0.468815*pow(in[0]-(-1.646393),2.0)+0.739089*pow(in[1]-(-0.787709),2.0)+0.978845*pow(in[2]-(-2.198015),2.0)))+0.652804*exp(-(0.186177*pow(in[0]-(-1.175260),2.0)+0.694896*pow(in[1]-(-0.804295),2.0)+2.068506*pow(in[2]-(-2.088019),2.0)))-2.943198*exp(-(0.204744*pow(in[0]-(1.023155),2.0)+0.852052*pow(in[1]-(0.548092),2.0)+2.082321*pow(in[2]-(-1.670878),2.0)))-0.210814*exp(-(0.459430*pow(in[0]-(-0.594482),2.0)+0.924338*pow(in[1]-(-0.074210),2.0)+2.556912*pow(in[2]-(-1.762879),2.0)))-1.921617*exp(-(0.510719*pow(in[0]-(0.128432),2.0)+0.597940*pow(in[1]-(0.412617),2.0)+1.096387*pow(in[2]-(-1.568026),2.0)))+0.137159*exp(-(0.771973*pow(in[0]-(-0.280697),2.0)+0.647106*pow(in[1]-(1.173934),2.0)+1.539408*pow(in[2]-(-1.726445),2.0)))-1.499348*exp(-(0.399576*pow(in[0]-(0.178510),2.0)+0.924338*pow(in[1]-(-0.399863),2.0)+0.894789*pow(in[2]-(-1.197674),2.0)))-0.315833*exp(-(0.080237*pow(in[0]-(0.865463),2.0)+0.760828*pow(in[1]-(-2.068293),2.0)+2.640703*pow(in[2]-(-1.720017),2.0)))-0.079872*exp(-(0.808144*pow(in[0]-(0.388428),2.0)+0.619131*pow(in[1]-(1.205508),2.0)+3.009613*pow(in[2]-(0.663541),2.0)))-0.895270*exp(-(1.111776*pow(in[0]-(0.823773),2.0)+0.589262*pow(in[1]-(0.580938),2.0)+1.134779*pow(in[2]-(-1.402733),2.0)))+0.108163*exp(-(0.481954*pow(in[0]-(-0.576897),2.0)+0.773718*pow(in[1]-(-0.268860),2.0)+1.041712*pow(in[2]-(-2.035397),2.0)))+0.146725*exp(-(0.890678*pow(in[0]-(-0.997899),2.0)+0.683795*pow(in[1]-(-0.027247),2.0)+2.256546*pow(in[2]-(-2.621238),2.0)))+1.603854*exp(-(0.659623*pow(in[0]-(-0.384111),2.0)+0.577395*pow(in[1]-(-0.170453),2.0)+0.804990*pow(in[2]-(-1.108459),2.0)))+0.006392*exp(-(1.111776*pow(in[0]-(0.839507),2.0)+0.924338*pow(in[1]-(0.030369),2.0)+2.555418*pow(in[2]-(-1.606457),2.0)))+0.194491*exp(-(0.972035*pow(in[0]-(0.565723),2.0)+0.924338*pow(in[1]-(0.344773),2.0)+1.686756*pow(in[2]-(-2.061232),2.0)))-0.024792*exp(-(1.095933*pow(in[0]-(-1.382939),2.0)+0.805280*pow(in[1]-(-1.097459),2.0)+2.282258*pow(in[2]-(-1.702632),2.0)))+0.051168*exp(-(0.707415*pow(in[0]-(0.599777),2.0)+0.701572*pow(in[1]-(0.813374),2.0)+1.692726*pow(in[2]-(-1.321312),2.0)))-0.346231*exp(-(0.564373*pow(in[0]-(-0.514190),2.0)+0.924338*pow(in[1]-(0.384999),2.0)+0.534770*pow(in[2]-(-0.772728),2.0)))-0.812220*exp(-(0.573954*pow(in[0]-(-0.013849),2.0)+0.846375*pow(in[1]-(-1.091435),2.0)+1.324875*pow(in[2]-(-1.608236),2.0)))+0.664237*exp(-(0.548031*pow(in[0]-(-0.906040),2.0)+0.924338*pow(in[1]-(0.664981),2.0)+1.778471*pow(in[2]-(-2.334958),2.0)))-0.201657*exp(-(0.373345*pow(in[0]-(-0.089906),2.0)+0.681041*pow(in[1]-(-0.407214),2.0)+2.863150*pow(in[2]-(-0.351350),2.0)))-0.619715*exp(-(0.598805*pow(in[0]-(-0.691264),2.0)+0.868440*pow(in[1]-(-0.813660),2.0)+2.140150*pow(in[2]-(-2.079202),2.0)))+3.859906*exp(-(0.455032*pow(in[0]-(0.193840),2.0)+0.732505*pow(in[1]-(0.499054),2.0)+1.075025*pow(in[2]-(-1.467225),2.0)))+1.937195*exp(-(0.736186*pow(in[0]-(0.339727),2.0)+0.473698*pow(in[1]-(-0.233450),2.0)+1.874246*pow(in[2]-(-1.519643),2.0)))+0.091964*exp(-(1.019209*pow(in[0]-(0.014981),2.0)+0.288005*pow(in[1]-(0.170946),2.0)+0.000000*pow(in[2]-(-1.519346),2.0)))-0.131049*exp(-(0.899460*pow(in[0]-(1.723521),2.0)+0.328670*pow(in[1]-(0.862933),2.0)+0.800032*pow(in[2]-(-0.690125),2.0)))+0.024018*exp(-(0.564956*pow(in[0]-(1.154018),2.0)+0.068925*pow(in[1]-(-0.028393),2.0)+4.518928*pow(in[2]-(-1.267662),2.0)))+0.441648*exp(-(0.920468*pow(in[0]-(-1.043359),2.0)+0.566705*pow(in[1]-(0.364208),2.0)+2.072945*pow(in[2]-(-1.579733),2.0)))-0.173665*exp(-(0.659065*pow(in[0]-(0.597360),2.0)+0.539586*pow(in[1]-(0.442485),2.0)+2.802289*pow(in[2]-(-0.651589),2.0)))-0.449842*exp(-(0.828295*pow(in[0]-(0.066858),2.0)+0.851950*pow(in[1]-(0.324155),2.0)+4.167803*pow(in[2]-(-2.043238),2.0)))-0.130477*exp(-(0.794143*pow(in[0]-(0.641201),2.0)+0.332055*pow(in[1]-(-0.238729),2.0)+3.508789*pow(in[2]-(-1.719821),2.0)))-0.404224*exp(-(0.581358*pow(in[0]-(-0.438536),2.0)+0.684534*pow(in[1]-(1.091409),2.0)+1.023692*pow(in[2]-(-1.144993),2.0)))+0.559694*exp(-(0.550998*pow(in[0]-(1.245443),2.0)+0.576964*pow(in[1]-(0.488224),2.0)+2.945927*pow(in[2]-(-2.062290),2.0)))-0.082398*exp(-(0.261471*pow(in[0]-(-0.430785),2.0)+0.758219*pow(in[1]-(-0.549695),2.0)+0.294178*pow(in[2]-(-1.624400),2.0)))-0.279848*exp(-(0.845270*pow(in[0]-(-0.759508),2.0)+0.782483*pow(in[1]-(0.282547),2.0)+2.744241*pow(in[2]-(-0.972145),2.0)))+0.616016*exp(-(0.444573*pow(in[0]-(0.095785),2.0)+0.575721*pow(in[1]-(0.598082),2.0)+1.568475*pow(in[2]-(-0.630787),2.0)))+0.348835*exp(-(0.347789*pow(in[0]-(-0.994952),2.0)+0.454213*pow(in[1]-(0.681447),2.0)+1.228778*pow(in[2]-(-1.056120),2.0)))+0.287200*exp(-(0.807958*pow(in[0]-(0.875147),2.0)+0.924338*pow(in[1]-(0.225517),2.0)+3.185356*pow(in[2]-(-1.523507),2.0)))-0.337703*exp(-(0.306438*pow(in[0]-(-0.616528),2.0)+0.618888*pow(in[1]-(-0.260658),2.0)+2.781577*pow(in[2]-(-2.911216),2.0)))-0.013472*exp(-(0.785048*pow(in[0]-(-0.766495),2.0)+0.924338*pow(in[1]-(-0.188037),2.0)+1.636770*pow(in[2]-(-0.840697),2.0)))+1.076541*exp(-(0.634334*pow(in[0]-(0.389840),2.0)+0.738920*pow(in[1]-(-0.382891),2.0)+1.464124*pow(in[2]-(-0.768726),2.0)))+0.718439*exp(-(0.442021*pow(in[0]-(-0.040033),2.0)+0.831394*pow(in[1]-(-0.431676),2.0)+0.266592*pow(in[2]-(-1.903896),2.0)))-0.538588*exp(-(0.583418*pow(in[0]-(-0.250912),2.0)+0.836858*pow(in[1]-(0.088902),2.0)+0.000000*pow(in[2]-(-1.637302),2.0)))-0.060097*exp(-(0.959351*pow(in[0]-(-0.892020),2.0)+0.924338*pow(in[1]-(1.139724),2.0)+0.684088*pow(in[2]-(-1.338684),2.0)))+0.176001*exp(-(0.492122*pow(in[0]-(0.413638),2.0)+0.924338*pow(in[1]-(-0.868768),2.0)+1.116571*pow(in[2]-(-1.491360),2.0)));
	else
		return -0.334589*exp(-(0.323600*pow(in[0]-(0.080407),2.0)+0.932012*pow(in[1]-(0.037026),2.0)+2.249306*pow(in[2]-(-2.279827),2.0)))-0.309661*exp(-(0.805163*pow(in[0]-(0.664366),2.0)+1.118853*pow(in[1]-(0.045262),2.0)+2.249306*pow(in[2]-(-1.285367),2.0)))-0.262459*exp(-(0.689891*pow(in[0]-(-0.387386),2.0)+0.862942*pow(in[1]-(-1.458148),2.0)+2.249306*pow(in[2]-(-2.406903),2.0)))-0.366124*exp(-(0.000000*pow(in[0]-(1.086487),2.0)+1.105923*pow(in[1]-(-0.772088),2.0)+2.249306*pow(in[2]-(-1.422548),2.0)))-0.029944*exp(-(0.000000*pow(in[0]-(0.704211),2.0)+0.160969*pow(in[1]-(-0.924781),2.0)+1.552095*pow(in[2]-(-2.823132),2.0)))-0.033916*exp(-(0.557823*pow(in[0]-(0.300853),2.0)+1.420678*pow(in[1]-(0.183894),2.0)+1.831440*pow(in[2]-(-2.748306),2.0)))-0.465445*exp(-(0.718271*pow(in[0]-(-0.617498),2.0)+1.163637*pow(in[1]-(0.060393),2.0)+1.756516*pow(in[2]-(-1.209939),2.0)))+0.057153*exp(-(0.629101*pow(in[0]-(0.501119),2.0)+0.638781*pow(in[1]-(0.027240),2.0)+1.414647*pow(in[2]-(-0.628076),2.0)))-0.294324*exp(-(0.729353*pow(in[0]-(-0.742071),2.0)+0.847293*pow(in[1]-(1.044067),2.0)+1.274821*pow(in[2]-(-1.511970),2.0)))-0.091165*exp(-(0.927253*pow(in[0]-(0.227192),2.0)+1.099188*pow(in[1]-(-0.283951),2.0)+1.041885*pow(in[2]-(-0.770992),2.0)))-1.186466*exp(-(0.000000*pow(in[0]-(0.952321),2.0)+0.557262*pow(in[1]-(-0.038983),2.0)+1.810448*pow(in[2]-(-1.172178),2.0)))+1.003976*exp(-(0.324258*pow(in[0]-(-0.423800),2.0)+1.339845*pow(in[1]-(-1.063817),2.0)+1.884906*pow(in[2]-(-1.411939),2.0)))+1.564677*exp(-(0.097828*pow(in[0]-(0.282342),2.0)+1.014078*pow(in[1]-(0.669376),2.0)+1.489215*pow(in[2]-(-1.457287),2.0)))+0.041902*exp(-(0.643018*pow(in[0]-(-0.179769),2.0)+0.985674*pow(in[1]-(-0.488469),2.0)+0.668608*pow(in[2]-(-1.197239),2.0)))+0.080994*exp(-(1.188356*pow(in[0]-(1.056056),2.0)+1.122882*pow(in[1]-(-1.318381),2.0)+1.547621*pow(in[2]-(-1.509336),2.0)))+0.056162*exp(-(0.449748*pow(in[0]-(-0.897019),2.0)+0.856917*pow(in[1]-(1.255696),2.0)+2.054918*pow(in[2]-(-0.932161),2.0)))-0.122429*exp(-(0.827686*pow(in[0]-(-0.210544),2.0)+1.037697*pow(in[1]-(0.851084),2.0)+2.249306*pow(in[2]-(-0.888145),2.0)))-0.330976*exp(-(0.346615*pow(in[0]-(0.891740),2.0)+0.721804*pow(in[1]-(-0.552886),2.0)+1.789290*pow(in[2]-(-0.747542),2.0)))-1.917650*exp(-(0.000000*pow(in[0]-(-0.494996),2.0)+0.632874*pow(in[1]-(-0.034876),2.0)+1.686979*pow(in[2]-(-1.712383),2.0)))-0.001758*exp(-(1.053006*pow(in[0]-(1.340250),2.0)+1.364370*pow(in[1]-(0.544066),2.0)+1.818005*pow(in[2]-(-1.571430),2.0)))-0.734813*exp(-(0.611456*pow(in[0]-(-1.460802),2.0)+0.253521*pow(in[1]-(-0.100372),2.0)+1.398779*pow(in[2]-(-2.239092),2.0)))+0.078196*exp(-(0.000000*pow(in[0]-(1.034933),2.0)+1.085548*pow(in[1]-(0.700204),2.0)+2.249306*pow(in[2]-(-0.472117),2.0)))+1.011239*exp(-(0.155649*pow(in[0]-(-1.549398),2.0)+0.992882*pow(in[1]-(-0.207761),2.0)+1.392841*pow(in[2]-(-1.588516),2.0)))+0.278612*exp(-(0.377200*pow(in[0]-(0.039408),2.0)+0.831537*pow(in[1]-(1.502389),2.0)+0.665935*pow(in[2]-(-1.012646),2.0)))+0.228248*exp(-(0.601411*pow(in[0]-(-0.027535),2.0)+1.406744*pow(in[1]-(0.111191),2.0)+1.582253*pow(in[2]-(-1.187046),2.0)))-0.014676*exp(-(0.476649*pow(in[0]-(-0.222073),2.0)+1.420678*pow(in[1]-(-0.994908),2.0)+1.103068*pow(in[2]-(-0.063509),2.0)))+0.519291*exp(-(0.138938*pow(in[0]-(0.528165),2.0)+1.007432*pow(in[1]-(0.966207),2.0)+0.547801*pow(in[2]-(-1.470111),2.0)))+0.264206*exp(-(0.000000*pow(in[0]-(-0.697170),2.0)+0.992495*pow(in[1]-(-1.492423),2.0)+1.779272*pow(in[2]-(-1.034308),2.0)))+0.021442*exp(-(1.386524*pow(in[0]-(-0.689878),2.0)+0.919507*pow(in[1]-(0.358730),2.0)+1.225371*pow(in[2]-(-1.590059),2.0)))+0.343581*exp(-(0.776605*pow(in[0]-(0.268518),2.0)+0.227444*pow(in[1]-(1.473689),2.0)+2.249306*pow(in[2]-(-1.846254),2.0)))-0.282832*exp(-(0.077288*pow(in[0]-(0.590412),2.0)+0.644249*pow(in[1]-(0.596238),2.0)+0.775701*pow(in[2]-(-0.383160),2.0)))-0.060809*exp(-(0.419555*pow(in[0]-(-0.175823),2.0)+0.402683*pow(in[1]-(0.782146),2.0)+1.512669*pow(in[2]-(-0.062298),2.0)))+0.027813*exp(-(0.220546*pow(in[0]-(-0.448961),2.0)+0.328207*pow(in[1]-(-0.722730),2.0)+1.565365*pow(in[2]-(-0.215478),2.0)))+0.139322*exp(-(0.121417*pow(in[0]-(0.525126),2.0)+0.382862*pow(in[1]-(0.029967),2.0)+2.249306*pow(in[2]-(-2.081949),2.0)))+0.707699*exp(-(0.492626*pow(in[0]-(-0.893913),2.0)+0.797894*pow(in[1]-(-0.417882),2.0)+1.752389*pow(in[2]-(-2.089988),2.0)))-1.327245*exp(-(0.361081*pow(in[0]-(1.064572),2.0)+0.453913*pow(in[1]-(-0.090510),2.0)+1.912633*pow(in[2]-(-2.211437),2.0)))-0.117399*exp(-(0.905341*pow(in[0]-(-0.127873),2.0)+0.693282*pow(in[1]-(-0.895079),2.0)+2.249306*pow(in[2]-(-1.375859),2.0)))+0.350791*exp(-(0.462737*pow(in[0]-(-0.323235),2.0)+0.638524*pow(in[1]-(-0.616334),2.0)+2.249306*pow(in[2]-(-2.385046),2.0)))-0.268789*exp(-(0.000000*pow(in[0]-(-0.762412),2.0)+1.098026*pow(in[1]-(-0.276130),2.0)+2.103515*pow(in[2]-(-3.512424),2.0)))-1.014859*exp(-(0.000000*pow(in[0]-(-1.097591),2.0)+0.646206*pow(in[1]-(0.324285),2.0)+1.969599*pow(in[2]-(-2.002489),2.0)))-0.106726*exp(-(0.789044*pow(in[0]-(0.901853),2.0)+0.427963*pow(in[1]-(1.125597),2.0)+2.249306*pow(in[2]-(-0.860844),2.0)))-0.133265*exp(-(1.032619*pow(in[0]-(-0.549325),2.0)+0.610491*pow(in[1]-(0.373060),2.0)+2.233990*pow(in[2]-(-2.373824),2.0)))-1.858847*exp(-(0.463774*pow(in[0]-(0.380389),2.0)+0.688559*pow(in[1]-(0.724448),2.0)+1.441616*pow(in[2]-(-1.910845),2.0)))+0.235124*exp(-(0.523715*pow(in[0]-(1.683343),2.0)+0.092320*pow(in[1]-(0.993579),2.0)+0.930244*pow(in[2]-(-1.925888),2.0)))-0.098464*exp(-(0.530581*pow(in[0]-(1.671060),2.0)+0.000000*pow(in[1]-(-0.911411),2.0)+1.200900*pow(in[2]-(-2.495741),2.0)))-0.049575*exp(-(0.695152*pow(in[0]-(0.750087),2.0)+1.298329*pow(in[1]-(-0.240410),2.0)+1.200317*pow(in[2]-(-2.663779),2.0)))+0.219099*exp(-(0.666455*pow(in[0]-(-1.279148),2.0)+1.359137*pow(in[1]-(-0.086280),2.0)+2.249306*pow(in[2]-(-1.115042),2.0)))+0.226864*exp(-(0.624424*pow(in[0]-(1.487241),2.0)+0.984028*pow(in[1]-(0.314477),2.0)+0.592336*pow(in[2]-(-1.168262),2.0)))+0.795741*exp(-(0.127176*pow(in[0]-(0.223314),2.0)+1.233895*pow(in[1]-(0.336459),2.0)+1.411233*pow(in[2]-(-2.115034),2.0)))+0.568679*exp(-(0.000000*pow(in[0]-(0.033105),2.0)+1.420678*pow(in[1]-(0.208355),2.0)+2.016412*pow(in[2]-(-1.861260),2.0)))+0.141584*exp(-(0.743679*pow(in[0]-(-2.413702),2.0)+1.388068*pow(in[1]-(0.207546),2.0)+1.924151*pow(in[2]-(-1.737852),2.0)))-0.555357*exp(-(0.176918*pow(in[0]-(-0.652508),2.0)+0.902944*pow(in[1]-(-0.621540),2.0)+2.249306*pow(in[2]-(-1.196655),2.0)))-0.213400*exp(-(0.583655*pow(in[0]-(0.320016),2.0)+0.852319*pow(in[1]-(0.678768),2.0)+2.249306*pow(in[2]-(-1.493642),2.0)))+0.109555*exp(-(1.143346*pow(in[0]-(-0.107343),2.0)+0.760401*pow(in[1]-(1.142946),2.0)+2.041039*pow(in[2]-(-1.018770),2.0)))+1.461191*exp(-(0.201941*pow(in[0]-(0.145412),2.0)+0.994272*pow(in[1]-(0.406654),2.0)+1.578107*pow(in[2]-(-1.824131),2.0)))+0.376643*exp(-(0.350786*pow(in[0]-(0.387142),2.0)+0.977145*pow(in[1]-(0.306066),2.0)+2.249306*pow(in[2]-(-1.145530),2.0)))-0.396590*exp(-(0.900147*pow(in[0]-(0.503127),2.0)+1.106208*pow(in[1]-(-0.982814),2.0)+1.574390*pow(in[2]-(-2.557511),2.0)))-0.770252*exp(-(0.645259*pow(in[0]-(-0.471668),2.0)+0.912797*pow(in[1]-(-0.783614),2.0)+1.686979*pow(in[2]-(-2.481146),2.0)))+0.001915*exp(-(0.409681*pow(in[0]-(-0.461769),2.0)+0.594168*pow(in[1]-(1.111987),2.0)+0.759782*pow(in[2]-(-2.030767),2.0)))-0.420346*exp(-(0.681596*pow(in[0]-(0.261738),2.0)+0.741879*pow(in[1]-(0.063224),2.0)+1.827390*pow(in[2]-(-1.470007),2.0)))+0.106269*exp(-(0.378438*pow(in[0]-(-0.074455),2.0)+0.630449*pow(in[1]-(-0.327256),2.0)+0.630077*pow(in[2]-(0.073907),2.0)))-0.095446*exp(-(1.535463*pow(in[0]-(0.977288),2.0)+0.484549*pow(in[1]-(0.488153),2.0)+1.908840*pow(in[2]-(-1.457756),2.0)))+0.117606*exp(-(0.498621*pow(in[0]-(-0.753018),2.0)+0.966440*pow(in[1]-(0.198428),2.0)+1.267768*pow(in[2]-(-0.192511),2.0)))-0.145541*exp(-(0.685552*pow(in[0]-(-0.523100),2.0)+1.420678*pow(in[1]-(0.619890),2.0)+1.110376*pow(in[2]-(-1.126694),2.0)))-0.204247*exp(-(0.491999*pow(in[0]-(-0.494245),2.0)+0.777125*pow(in[1]-(2.566200),2.0)+1.269572*pow(in[2]-(-0.766139),2.0)))-0.627153*exp(-(0.302000*pow(in[0]-(-0.057853),2.0)+0.776712*pow(in[1]-(0.752129),2.0)+1.994575*pow(in[2]-(-0.803366),2.0)))-2.108921*exp(-(0.095964*pow(in[0]-(0.481761),2.0)+0.653693*pow(in[1]-(0.491868),2.0)+1.955781*pow(in[2]-(-1.610875),2.0)))-0.175029*exp(-(0.204408*pow(in[0]-(0.667241),2.0)+0.867677*pow(in[1]-(-1.202471),2.0)+1.052065*pow(in[2]-(-2.284265),2.0)))+3.102648*exp(-(0.109751*pow(in[0]-(0.337228),2.0)+0.994589*pow(in[1]-(-0.195601),2.0)+1.283403*pow(in[2]-(-1.495123),2.0)))+0.393267*exp(-(0.512025*pow(in[0]-(-0.500488),2.0)+0.854203*pow(in[1]-(0.459591),2.0)+2.249306*pow(in[2]-(-0.756183),2.0)))-0.265850*exp(-(0.924986*pow(in[0]-(-0.668469),2.0)+1.319744*pow(in[1]-(-0.940243),2.0)+1.394974*pow(in[2]-(-2.147634),2.0)))+0.646253*exp(-(0.000000*pow(in[0]-(0.735491),2.0)+1.208664*pow(in[1]-(1.197454),2.0)+1.614563*pow(in[2]-(-1.784355),2.0)))-0.720378*exp(-(0.358029*pow(in[0]-(-1.125501),2.0)+1.420678*pow(in[1]-(-0.323116),2.0)+2.034897*pow(in[2]-(-1.968878),2.0)))+0.253522*exp(-(0.653285*pow(in[0]-(-1.293493),2.0)+0.988060*pow(in[1]-(1.094813),2.0)+0.950396*pow(in[2]-(-1.823839),2.0)))-0.256931*exp(-(0.519897*pow(in[0]-(-1.034522),2.0)+0.662743*pow(in[1]-(-0.512735),2.0)+1.384345*pow(in[2]-(-0.735723),2.0)))-0.081506*exp(-(1.005764*pow(in[0]-(-0.697870),2.0)+1.399964*pow(in[1]-(-0.143379),2.0)+0.504958*pow(in[2]-(-1.433799),2.0)))+0.603647*exp(-(0.698536*pow(in[0]-(1.015819),2.0)+1.170166*pow(in[1]-(0.365416),2.0)+1.548151*pow(in[2]-(-2.051267),2.0)))-0.074907*exp(-(1.077439*pow(in[0]-(-0.854065),2.0)+0.715224*pow(in[1]-(-0.298648),2.0)+2.249306*pow(in[2]-(-1.196697),2.0)))+0.263918*exp(-(0.216073*pow(in[0]-(-0.306690),2.0)+1.038209*pow(in[1]-(1.334969),2.0)+2.249306*pow(in[2]-(-2.184179),2.0)))-0.895151*exp(-(0.297292*pow(in[0]-(-0.393301),2.0)+0.296442*pow(in[1]-(-0.354602),2.0)+1.637208*pow(in[2]-(-1.676763),2.0)))+0.797862*exp(-(0.311933*pow(in[0]-(-0.320465),2.0)+0.689489*pow(in[1]-(0.498084),2.0)+2.137608*pow(in[2]-(-1.559720),2.0)))+0.062698*exp(-(1.096881*pow(in[0]-(0.083434),2.0)+0.816842*pow(in[1]-(-0.074598),2.0)+1.089534*pow(in[2]-(-1.372417),2.0)))+1.306045*exp(-(0.000000*pow(in[0]-(0.495352),2.0)+1.420678*pow(in[1]-(-0.719131),2.0)+0.831864*pow(in[2]-(-1.985843),2.0)))-0.186171*exp(-(0.383866*pow(in[0]-(0.592593),2.0)+1.185654*pow(in[1]-(0.547697),2.0)+2.249306*pow(in[2]-(-2.909369),2.0)))-0.228240*exp(-(0.252209*pow(in[0]-(2.049914),2.0)+1.138727*pow(in[1]-(0.597820),2.0)+2.033900*pow(in[2]-(-1.000122),2.0)))+0.718674*exp(-(0.411169*pow(in[0]-(0.028220),2.0)+1.240246*pow(in[1]-(-0.299701),2.0)+1.453667*pow(in[2]-(-2.176986),2.0)))+0.651846*exp(-(0.447559*pow(in[0]-(0.449091),2.0)+1.420678*pow(in[1]-(0.990173),2.0)+1.222242*pow(in[2]-(-1.869177),2.0)))+0.216367*exp(-(0.395798*pow(in[0]-(0.025357),2.0)+0.765402*pow(in[1]-(0.045537),2.0)+1.618434*pow(in[2]-(-2.872418),2.0)))-0.058205*exp(-(0.864008*pow(in[0]-(-0.028713),2.0)+0.731287*pow(in[1]-(-0.278627),2.0)+1.893918*pow(in[2]-(-1.937376),2.0)))+0.049019*exp(-(0.000000*pow(in[0]-(-0.211651),2.0)+0.765904*pow(in[1]-(0.752160),2.0)+2.249306*pow(in[2]-(-2.163595),2.0)));
}

float CLeg::computeKinematicMarginPos(double x,double y, double z,int part,int up){
	if (!isFootPositionAvailableLocal(x, y, z, part,0))
		return 0;
	for (int r=0;r<40;r++){
		for (int theta=0;theta<8;theta++){
			for (int phi=0;phi<8;phi++){
				if (!isFootPositionAvailableLocal(x+r*0.005*sin(theta*0.785)*cos(phi*0.785), y+r*0.005*sin(theta*0.785)*sin(phi*0.785), z+r*0.005*cos(theta*0.785), part,0))
					return r*0.005;				
			}
		}
	}
	/*if (isFootPositionAvailableLocal(x, y, z, part,0))
	{
		for (int r=0;r<200;r++){
			for (int theta=0;theta<8;theta++){
				for (int phi=0;phi<8;phi++){
					if (!isFootPositionAvailableLocal(x+r*0.001*sin(theta*0.785)*cos(phi*0.785), y+r*0.001*sin(theta*0.785)*sin(phi*0.785), z+r*0.001*cos(theta*0.785), part,0))
						return r*0.001;				
				}
			}
		}
	}
	else {
		for (int r=0;r<200;r++){
			for (int theta=0;theta<8;theta++){
				for (int phi=0;phi<8;phi++){
					if (isFootPositionAvailableLocal(x+r*0.001*sin(theta*0.785)*cos(phi*0.785), y+r*0.001*sin(theta*0.785)*sin(phi*0.785), z+r*0.001*cos(theta*0.785), part,0))
						return -r*0.001;				
				}
			}
		}
	}*/
}

/// computes delta_z to maximize kinematic margin
float CLeg::maximizeKinematicMarginPos(double x,double y, double z,int part,int up){
	float max=computeKinematicMarginPos(x,y,z, part, up);
	float margin;
	float delta_z_max=0;
	for (int i=0;i<60;i++){
		margin=computeKinematicMarginPos(x,y,z-0.15+0.005*i, part, up);
		if (margin>max){
			delta_z_max=-0.15+0.005*i;
		}
	}
	return delta_z_max;
}

//sprawdza czy pozycja stopy jest osiagalna
bool CLeg::isFootPositionAvailableLocal(double x,double y, double z,int part,int up, float scale)
{
	CPunctum point;
	int _side;
	if (part==1)
		_side = 1;
	else
		_side = -1;
	if (up==0) {
		point.createTRMatrix(0,0,0,x,y,z);
		double temp_angle[3];
		return inverse_kinematic(point.getElement(1,4),point.getElement(2,4),point.getElement(3,4), _side, temp_angle, scale);
	}
	else if (up==1) {
		point.createTRMatrix(0,0,0,0,0,z);
		double temp_angle[3];
		return inverse_kinematic(0.078,-0.0083,point.getElement(3,4), _side, temp_angle, scale);
	}
	return true;
}
