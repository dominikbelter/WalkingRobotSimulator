#include "PositionRecorder.h"
#include <stdlib.h>
#include <gl\glut.h> // glut.h includes gl.h and glu.h

CPositionRecorder::CPositionRecorder(void)
{
	delay=0;
	delay_iterator=0;
}

CPositionRecorder::~CPositionRecorder(void)
{
}

/// dopisanie macierzy do listy
void CPositionRecorder::savePunctum (CPunctum punkt){
	if (delay_iterator==delay){
		container.push_back(punkt);
		rec_it = container.end();
		delay_iterator=0;
	}
	else
		delay_iterator++;
}

/// dopisanie macierzy do listy
void CPositionRecorder::insertPunctum (CPunctum punkt){
		container.insert(rec_it,punkt);
}

/// pobierz ostatnio zapisany punkt
void CPositionRecorder::getLastPoint(float *x, float *y, float *z){
	CPunctum temp;
	if (container.size()>0){
		temp = container.back();
		*x = temp.getElement(1,4);
		*y = temp.getElement(2,4);
		*z = temp.getElement(3,4);
	}
}

/// pobierz ostatnio zapisany punkt
CPunctum CPositionRecorder::getLast(void){
	return container.back();
}

/// dopisanie punktu do listy
void CPositionRecorder::savePosition (double x, double y, double z){
	CPunctum temp;
	temp.createTRMatrix(0,0,0,x,y,z);
	savePunctum(temp);
}

/// dopisanie punktu do listy - tylko orientacja
void CPositionRecorder::saveOrientation (double alpha, double beta, double gamma){
	CPunctum temp;
	temp.createTRMatrix(alpha,beta,gamma,0,0,0);
	savePunctum(temp);
}

/// dopisanie punktu do listy na podstawie wspolrzednych i katow eulera
void CPositionRecorder::savePositionAndOrientation (double x, double y, double z,double alpha, double beta, double gamma){
	CPunctum temp;
	temp.createTRMatrix(alpha,beta,gamma,x,y,z);
	temp.orientation[0]=alpha; temp.orientation[1]=beta; temp.orientation[2]=gamma;
	savePunctum(temp);
}

/// dopisanie punktu do listy na podstawie wspolrzednych i katow eulera
void CPositionRecorder::insertPositionAndOrientation (double x, double y, double z,double alpha, double beta, double gamma){
	CPunctum temp;
	temp.createTRMatrix(alpha,beta,gamma,x,y,z);
	temp.orientation[0]=alpha; temp.orientation[1]=beta; temp.orientation[2]=gamma;
	insertPunctum(temp);
}

void CPositionRecorder::plot(double red, double green, double blue, double thickness, const char marker)
{
	if (container.size()>0){
		std::list<CPunctum>::iterator it,it_prev;
		glPushMatrix();
		glColor3f(red, green, blue);
		glLineWidth(thickness);
		int liczba=0;
		it_prev=container.begin();
		it=container.begin();
		if (it!=container.end()){
			it++;
			for ( it ; it != container.end(); it++ ){
				glBegin(GL_LINES);
					glVertex3f((*it_prev).getElement(1,4)*10, (*it_prev).getElement(3,4)*10, -(*it_prev).getElement(2,4)*10);
					glVertex3f((*it).getElement(1,4)*10, (*it).getElement(3,4)*10, -(*it).getElement(2,4)*10);
				glEnd();
				it_prev=it;
 			}
		}
		glPopMatrix();

		it=container.begin();
		if ((marker=='o')&&(it!=container.end())) {
			for ( it=container.begin() ; it != container.end(); it++){
				glPushMatrix();
				if (liczba%12==0)
					glColor3f(1, 0,0);
				else
					glColor3f(red, green, blue);
				glTranslatef((*it).getElement(1,4)*10, (*it).getElement(3,4)*10, -(*it).getElement(2,4)*10);
			///	if ((*it).isFoothold())
			//		glutSolidSphere(.05*thickness,5,5);
			//	else
					glutSolidSphere(.01*thickness,5,5);
				glPopMatrix();
				liczba++;
			}
		}
	}
}

/// ustawia opoznienie
void CPositionRecorder::setDelay(int value){
	delay = value;
}

/// save to file
void CPositionRecorder::savePosition2file(const char * filename)
{
	plot_stop=true; //wy³¹czenie rysowania zawartoœci position recordera na czas zapisu do pliku - zabezpieczenie przed konfliktem miêdzy w¹tkami

	FILE * plik;
	std::list<CPunctum>::iterator it_save;

	if (container.size()>0){
		fopen_s(&plik, filename,"w+t");
		fprintf(plik,"x=[",filename);
		for ( it_save = container.begin() ; it_save != container.end(); it_save++ ){
			fprintf(plik,"%.4f,",(*it_save).getElement(1,4)*10);
		}
		fprintf(plik,"];\n");

		fprintf(plik,"y=[",filename);
		for ( it_save = container.begin() ; it_save != container.end(); it_save++ ){
			fprintf(plik,"%.4f,",(*it_save).getElement(2,4)*10);
		}
		fprintf(plik,"];\n");

		fprintf(plik,"z=[",filename);
		for ( it_save = container.begin() ; it_save != container.end(); it_save++ ){
			fprintf(plik,"%.4f,",(*it_save).getElement(3,4)*10);
		}
		fprintf(plik,"];\n");
		fprintf(plik,"plot3(-y*10,x*10,z*10);\n");

		fclose(plik);
	}

	plot_stop=false;
}

/// ustawia iterator na poczatek listy
void CPositionRecorder::setBegin(){
	rec_it = container.begin();
}

/// pobierz pierwszy element
bool CPositionRecorder::getFirst(CPunctum * point){
	rec_it = container.begin();
	if (rec_it != container.end()){
		*point = *rec_it;
		return true;
	}
	else
		return false;
}

/// pobierz kolejny element element
bool CPositionRecorder::getNext(CPunctum * point){
	if (rec_it != container.end()){
		rec_it++;
		*point = *rec_it;
		return true;
	}
	else
		return false;
}

/// aktualizuj wartosc aktualnego elementu
void CPositionRecorder::updateElement(CPunctum * point){
	*rec_it = *point;
}

/// aktualizuj wartosc aktualnego elementu
void CPositionRecorder::updatePreviousElement(CPunctum * point){
	if (rec_it!=container.begin()){
		rec_it--;
		*rec_it = *point;
		rec_it++;
	}

}

/// pobierz aktualny element element
bool CPositionRecorder::getElement(CPunctum * point){
	if (rec_it != container.end()){
		*point = *rec_it;
		rec_it++;
		return true;
	}
	else
		return false;
}

/// pobierz kolejny element element - z przesunieciem
bool CPositionRecorder::getNextShift(CPunctum * point, int shift){
	for (int i=0;i<shift;i++){
		if (rec_it != container.end())
			rec_it++;
	}
	if (rec_it != container.end()){
		*point = *rec_it;
		return true;
	}
	else
		return false;
}
/// wstaw element
bool CPositionRecorder::insertAfter(CPunctum * point){
	if (rec_it != container.end()){
		rec_it++;
	}
	container.insert(rec_it,(*point));
	return true;
}

/// cofa iterator o 'shift' pozycji
bool CPositionRecorder::reverse(int shift){
	for (int i=0;i<shift;i++){
		if (rec_it != container.begin())
			rec_it--;
		else 
			return false;
	}
	return true;
}
//usuwa ostatnie 'shift' elementow z listy
void CPositionRecorder::eraseLast(int shift){
	rec_it = container.end();
	if (rec_it != container.begin())
		rec_it--;
	std::list<CPunctum>::iterator del;
	for (int i=0;i<shift;i++){
		del=rec_it;
		if (rec_it != container.begin())
			rec_it--;
		container.erase(del);
	}
}
/// czysci zawartosc
void CPositionRecorder::clear(){
	container.clear();
}

/// odwraca kolejnosc zawartosci
void CPositionRecorder::reverseOrder(){
	container.reverse();
}

/// pobierz liczbe elementow
int CPositionRecorder::getSize(){
	return container.size();
}