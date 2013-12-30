#include "localMap.h"
#include <iostream>


using namespace std;

     const float width_max_ = 0.13;   ///odleglosc od srodka robota do srodkowej nogi
     const float width_min_ = 0.065;  ///odleglosc od osi podluznej robota do przedniej nogi
     const float length_ = 0.153289;  ///odleglosc nog przednich od srodka robota wzdluz osi podluznej


 CLocalMap::CLocalMap(int size_x, int size_y, float raster_x, float raster_y, bool source, CIdealMap* map) : CPureScan(source,map){
	
	////printf("Odczyt danych konfiguracyjnych \n");
	//config.readParameters("Config/parameters.cfg");

	//scanner_x = config.readParameter("SCANNER_X");
	//scanner_y = config.readParameter("SCANNER_Y");
	//scanner_z = config.readParameter("SCANNER_Z");

	//scanner_alpha = config.readParameter("SCANNER_ALPHA");
	//scanner_beta =  config.readParameter("SCANNER_BETA");		
	//scanner_gamma = config.readParameter("SCANNER_GAMMA");

	scanner_x = 0;
	scanner_y = 0.202;
	scanner_z = 0.15;

	scanner_alpha = -45;
	scanner_beta = 0;		
	scanner_gamma = 0;

	

              //Ustawia wymiary mapy 
//		this->hrf=hrf;
	  this->size_x=size_x;
	  this->size_y=size_y;
	  this->raster_x=raster_x;
	  this->raster_y=raster_y;
//	  this->map=map;

	  draw_robot_position[0]=0;
	  draw_robot_position[1]=0;
	  draw_robot_position[2]=0;
//	  draw_robot_orientation[0]=0;
//	  draw_robot_orientation[1]=0;
//	  draw_robot_orientation[2]=0;

	  this->source = source;
//		  this->_imu = config.readParameter("IMU");
//		  this->h_refParam = config.readParameter("H_REF");
//	  this->_imu = 0;
//	  this->h_refParam = 1;
	  this->robot_dzTemp = 0;						

              CreateMap();
		
	  //reset pliku StanRobota.txt
	  robotState = fopen ("pliki/wynik/StanRobota.txt","w");
	  fclose (robotState);

	  if(source == 1){ //Tworzenie mapy terenu offline, na podstawie plików .txt
		  readRobot = fopen("pliki/zrodlo/StanRobota.txt","r"); //otwieramy plik do odczytu
    		  if (readRobot!=NULL) //jezeli udalo się otworzyc plik
		  	printf("Plik 'StanRobota.txt' wczytany poprawnie ..\n");
		  else{
			printf("Nie udalo sie otworzyć pliku 'StanRobota.txt'. \n");
			printf("Aplikacja zostanie zamknięta .. \n");
			exit(0);
		       }
	  }	
// 		  printf("proba polaczenia z kamera \n");
// 		  if(InitCamera())
// 		      printf("nie udalo sie zainicjalizowac kamery\n");
// 		  rpccaller_cam=new RPCCallerCamera("127.0.0.1",2743);
// 		  rpccaller_cam->Run();
// 		  rpccaller_cam->InitializeSHM();
// 		  printf("nawiazano polaczenie z kamera .. \n");
	  image_no=1;
	}
		
// destruktor klasy CLocalMap
CLocalMap::~CLocalMap() {
}

// Tworzy mape.
void CLocalMap::CreateMap() {
    grid_map = new SMapa* [ size_x ];
		for( int i = 0; i < size_x; i++){

			grid_map[ i ] = new SMapa[ size_y ];
			for( int j = 0; j < size_y; j++){

				grid_map[ i ][ j ].elevation = 0;
				grid_map[ i ][ j ].inclination_x = 0;
				grid_map[ i ][ j ].inclination_y = 0;
				grid_map[ i ][ j ].certainty = 0;

			}
		}

		increment_x = 0;
		increment_y = 0;
}

// Zeruje mape
void CLocalMap::ClearMap( ){
	for( int i = 0; i < size_x; i++){
		for( int j = 0; j < size_y; j++){
			grid_map[ i ][ j ].elevation = 0;
			grid_map[ i ][ j ].inclination_x = 0;
			grid_map[ i ][ j ].inclination_y = 0;
			grid_map[ i ][ j ].certainty = 0;
		}
	}
        increment_x = 0;
	increment_y = 0;
}

// akutalizująca mapy na podstawie współrzędnych xyz w pomiaru
void CLocalMap::UpdateMap(float x, float y, float atrybut,float max_err){
        
	int id_mapa_x = (int)((float)x / (raster_x*100) + (float)size_x / 2);
	int id_mapa_y = (int)((float)size_y / 2 - (float)y / (raster_y*100));

        //printf("id_mapa_x=%d, id_mapa_y=%d size_x=%d \n",id_mapa_x, id_mapa_y, size_x );

	if(id_mapa_x >= 0 && id_mapa_x < size_x && id_mapa_y >= 0 && id_mapa_y < size_y){

		float roznica = atrybut - grid_map[ id_mapa_x ][ id_mapa_y ].elevation;
		if( roznica < 0) roznica = -roznica;

     
		if( atrybut > grid_map[ id_mapa_x ][ id_mapa_y ].elevation  || grid_map[ id_mapa_x ][ id_mapa_y ].certainty == 0)
                {
                   // printf("x=%f; y=%f; atrybut = %f \n", x, y, atrybut);
			grid_map[ id_mapa_x ][ id_mapa_y ].elevation = atrybut;
                }

		if( roznica < max_err || grid_map[ id_mapa_x ][ id_mapa_y ].certainty == 0)
		{
   			grid_map[ id_mapa_x ][ id_mapa_y ].certainty += W;	

			if(grid_map[ id_mapa_x ][ id_mapa_y ].certainty > 100)
			{
				grid_map[ id_mapa_x ][ id_mapa_y ].certainty = 100;
			}
		}
              //  else printf("roznica = %f; max_err = %f \n", roznica, max_err);
	}

}

// przesuniecie mapy 
void CLocalMap::MoveMap( float dx, float dy, float dkat_Z ){

		increment_x += dx;
		increment_y += dy;

		int di = (int)(increment_x / raster_x);
		int dj = (int)(increment_y / raster_y);

		if( di != 0 || dj != 0 ){
			SMapa** grid_map2_new = new SMapa* [ size_x ];
			for( int i = 0; i < size_y; i++){
				grid_map2_new[ i ] = new SMapa[ size_y ];
					for( int j = 0; j < size_y; j++){

                                                grid_map2_new[ i ][ j ].elevation = 0;
						grid_map2_new[ i ][ j ].certainty = 0;
					}
			}

			for( int i = 0; i < size_x; i++ )
				for( int j = 0; j < size_y; j++ ){

					if( (i - di) >= 0 && (i - di) < size_x && (j + dj) >= 0 && (j + dj) < size_y ){
						grid_map2_new[ i - di ][ j + dj ].elevation	= grid_map[ i ][ j ].elevation;
						grid_map2_new[ i - di ][ j + dj ].certainty	= grid_map[ i ][ j ].certainty;
					}
				}

			for( int i = 0; i < size_x; i++ )
				for( int j = 0; j < size_y; j++ ){
					grid_map[ i ][ j ].elevation = grid_map2_new[ i ][ j ].elevation;
					grid_map[ i ][ j ].certainty = grid_map2_new[ i ][ j ].certainty;
				}

			for( int i = 0; i < size_x; i++)	delete grid_map2_new[ i ];
			delete grid_map2_new;

			if( di != 0 ) increment_x -= (di * raster_x);
			if( dj != 0 ) increment_y -= (dj * raster_y);
		}

                float increment_x_rot = increment_x * cos(deg2rad(dkat_Z)) - increment_y * sin(deg2rad(dkat_Z));
		float increment_y_rot = increment_x * sin(deg2rad(dkat_Z)) + increment_y * cos(deg2rad(dkat_Z));

		increment_x = increment_x_rot;
		increment_y = increment_y_rot;

	}

// obrot mapy
void CLocalMap::RotateMap(float kat_X, float kat_Y, float kat_Z){

	// w pierwszej kolejnosci przesuniecie mapy

        CPunctum dg_s;
		dg_s.setZero();
        dg_s.pos[1]=scanner_y;
        dg_s.pos[2]=scanner_z; 
	
        CPunctum Tg_s;
		Tg_s = makeTransformMatrix("alpha",kat_X) * makeTransformMatrix("beta",kat_Y);
        CPunctum RotZ;
		RotZ = makeTransformMatrix("gamma", kat_Z);
      
        CPunctum dg_s2;
		dg_s2 = Tg_s * RotZ * dg_s;

      
	float dx = dg_s2.pos[0];
	float dy = (dg_s2.pos[1] - dg_s.pos[1]);
	MoveMap( dx, dy, 0 );

	// obrot
	int i2, j2;

	SMapa** grid_map2_new = new SMapa* [ size_x ];
	for( int i = 0; i < size_x; i++){
		grid_map2_new[ i ] = new SMapa[ size_y ];
			for( int j = 0; j < size_y; j++){
				grid_map2_new[ i ][ j ].elevation = 0;
				grid_map2_new[ i ][ j ].certainty = 0;
			}
	}

	for( int i = 0; i < size_x; i++ )
		for( int j = 0; j < size_y; j++ ){

			i2 = (int)( cos(kat_Z) * i + sin(kat_Z) * j + (1 - cos(kat_Z)) * (int)(((float)size_x) / 2) - sin(kat_Z) * (int)(((float)size_y) / 2));
			j2 = (int)(-sin(kat_Z) * i + cos(kat_Z) * j + sin(kat_Z) * (int)(((float)size_x) / 2) + (1 - cos(kat_Z)) * (int)(((float)size_y) / 2));

			if(i2 >= 0 && i2 < size_x && j2 >= 0 && j2 < size_y && grid_map[ i2 ][ j2 ].certainty != 0){
				grid_map2_new[ i ][ j ].elevation	= grid_map[ i2 ][ j2 ].elevation;
				grid_map2_new[ i ][ j ].certainty	= grid_map[ i2 ][ j2 ].certainty;
			}
		}

	for( int i = 0; i < size_x; i++ )
		for( int j = 0; j < size_y; j++ ){
			grid_map[ i ][ j ].elevation     = grid_map2_new[ i ][ j ].elevation;
			grid_map[ i ][ j ].certainty = grid_map2_new[ i ][ j ].certainty;
		}

	for( int i = 0; i < size_x; i++)	delete grid_map2_new[ i ];
	delete grid_map2_new;

}

//wyznacza nachylenia w kierunkach "x" i "y"
void CLocalMap::CalculateInclination(){
	float data1, data2;
	for( int i = 1; i < size_x - 1; i++ )
		for( int j = 1; j < size_y - 1; j++ ){

			data1 =	grid_map[ i ][ j ].elevation;
			data2 = grid_map[ i ][ j - 1 ].elevation;
			grid_map[ i ][ j ].inclination_y = (float)57.2957795 * atan((data2 - data1) / raster_y);

			data2 = grid_map[ i + 1 ][ j ].elevation;
			grid_map[ i ][ j ].inclination_x = (float)57.2957795 * atan((data2 - data1) / raster_x);

		}	

	}

// zwraca wskaźnik do mapy
SMapa** CLocalMap::GetMapPointer( ){
    return grid_map;
}

// zwraca wysokość w (x,y)
float CLocalMap::GetElevation(int x, int y){
		y=y+int(scanner_y/raster_y);//modyfikacje, tak aby zwracal wartosci w ukladzie robota
		if(x >= 0 && x < size_x && y >= 0 && y < size_y && GetCertainty(x, y-int(scanner_y/raster_y))>0)
			return grid_map[ x ][ y ].elevation+scanner_z;
		else 
			return 0;
}

// zwraca wiarygodność w (x,y)
float CLocalMap::GetCertainty(int x, int y){
    y=y+int(scanner_y/raster_y);//modyfikacje, tak aby zwracal wartosci w ukladzie robota
    if(x >= 0 && x < size_x && y >= 0 && y < size_y)
        return (float)grid_map[ x ][ y ].certainty;
    else
	return 0;
}

// zwraca nachylenie_X w (x,y)
float CLocalMap::GetInclination_X(int x, int y){
	if(x >= 0 && x < size_x && y >= 0 && y < size_y)
		return grid_map[ x ][ y ].inclination_x;
	else
		return 0;
}

// zwraca nachylenie_Y w (x,y)
float CLocalMap::GetInclination_Y(int x, int y){
	if(x >= 0 && x < size_x && y >= 0 && y < size_y)
		return grid_map[ x ][ y ].inclination_y;
	else
		return 0;
}

//
void CLocalMap::Calculate3DMap(int no, float kat_X, float kat_Y, float kat_Z, float deltaY){

    float Alfa = deg2rad(scanner_alpha);

    float err = 0;		
    float err2 = 0;

    float gamma;


    float dz = robot_dz*100;

    for(int i=0; i<no; i++)
    {
	float l = (Get_l(i))*0.001; //zamiana na metry

        float* wsp_punkt_pomiar;
        wsp_punkt_pomiar = new float[3];
        wsp_punkt_pomiar[0] = Get_X(i);
        wsp_punkt_pomiar[1] = Get_Y(i);
        wsp_punkt_pomiar[2] = 0;

	//  OBRÓT PUNKTU WZGLĘDEM OSI X i Y
	gamma = CalculateGlobalCoordinates(wsp_punkt_pomiar, kat_Y, kat_X, Alfa);

	
	if((rad2deg(kat_Z)>0 && rad2deg(kat_Z)<90) || (rad2deg(kat_Z)<0 && rad2deg(kat_Z)>-90))
	{
		float deltaY2 = CalculateDeltaY_afterRotate(wsp_punkt_pomiar[ 0 ], wsp_punkt_pomiar[ 1 ], kat_Z);
		err2 = CalculateMaxErr(kat_X, kat_Y, Get_kat_rad(i), Alfa, gamma, deltaY2, l);
	}
	else err = CalculateMaxErr(kat_X, kat_Y, Get_kat_rad(i), Alfa, gamma, deltaY, l);

	//printf("Err = %f \n", err);

	//Aktualizacja mapy
	UpdateMap(wsp_punkt_pomiar[ 0 ], wsp_punkt_pomiar[ 1 ], (wsp_punkt_pomiar[ 2 ] + dz)/100, (err+err2)); 
	err = 0; err2 = 0;
        delete wsp_punkt_pomiar;
    }
}

// Wyznacza współrzędne globalne, tórjwymiarowe; zwaraca wartosc globalna pochylanie skanera "gamma"
float CLocalMap::CalculateGlobalCoordinates(float* punkty, float kat_Y, float kat_X, float Alfa){

         CPunctum Tg_c;
		 Tg_c = makeTransformMatrix("alpha", kat_X)*makeTransformMatrix("beta",kat_Y)*makeTransformMatrix("alpha",Alfa)*makeTransformMatrix("beta",deg2rad(scanner_beta)); //na radiany
         CPunctum X1;
		 X1.setZero();
         X1.rot[0][0] = punkty[0];
         X1.rot[1][0] = punkty[1];
         X1.rot[2][0] = punkty[2];

         /*printf("x-> %f %f %f %f \n", X1(0,0), X1(0,1),X1(0,2), X1(0,3));
         printf("y-> %f %f %f %f \n", X1(1,0), X1(1,1),X1(1,2), X1(1,3));
         printf("z-> %f %f %f %f \n", X1(2,0), X1(2,1),X1(2,2), X1(2,3));
         printf("    %f %f %f %f \n", X1(3,0), X1(3,1),X1(3,2), X1(3,3));
*/

         CPunctum X;
		 X = Tg_c * X1;

         /*printf("x-> %f %f %f %f \n", X(0,0), X(0,1),X(0,2), X(0,3));
         printf("y-> %f %f %f %f \n", X(1,0), X(1,1),X(1,2), X(1,3));
         printf("z-> %f %f %f %f \n", X(2,0), X(2,1),X(2,2), X(2,3));
         printf("    %f %f %f %f \n", X(3,0), X(3,1),X(3,2), X(3,3));
*/
         punkty[ 0 ] = (float) X.rot[0][0];
         punkty[ 1 ] = (float) X.rot[1][0];
         punkty[ 2 ] = (float) X.rot[2][0];

       //  printf("x=%f; y=%f; z=%f; \n", punkty[0], punkty[1], punkty[2]);

         float gamma = atan2(-Tg_c.rot[1][2],Tg_c.rot[2][2]);
	 //gamma = gamma * 180 / 3.14;

	 return gamma;

}

// funkcja główna: wykonuje skan i tworzy mape
void CLocalMap::makeScan(float angle_min, float angle_max, int scan_no, float deltaX,float deltaY,float z,float kat_X,float kat_Y,float kat_Z){
		
		
	
        int points_no = scan(angle_min, angle_max, scan_no, source);

	//printf("init deltaY %f \n", deltaY);
	//printf("init katZ %f \n", rad2deg(kat_Z));

	//if(deltaY != 0) deltaY = 0.025;
    ////printf("points_no %d",points_no);
	//printf("delta Y %f \n", deltaY);
	
	//if(abs(rad2deg(kat_Z)) < 4) kat_Z = 0;
	//else kat_Z = deg2rad(-30);

	//if (kat_Z > 0 ) kat_Z = kat_Z * (-1);

	//printf("kat_Z = %f \n", rad2deg(kat_Z));

	//if(deltaY != 0) deltaY = 0.02;

	//if(abs(rad2deg(kat_Z)) < 5) kat_Z = 0;
	//else kat_Z = deg2rad(-30);
	

robot_dz=z;
		
	//Jesli dzialamy w trybie offline odczytuje dane o stanie Robota w momencie skanu z pliku .txt
	if(source == 1) readRobotState(&deltaX, &deltaY, &robot_dz, &kat_X, &kat_Y, &kat_Z);		

	// zmienna pomocnicza "robot_dzTemp" zapamietuje pierwsza wartosc "robot_dz" z pliku i we wszystkich pozostałych skanach pozostaje ona stała
	// pozwala to na nie uwzglednianie wartosci "h_ref" podczas odczytywania danych z pliku (UWAGA: funkcja dziala poprawnie tylko wtedy gdy chod robota nie ulega zmianie)  
	if(source == 1 && h_refParam == 0 && robot_dzTemp == 0) robot_dzTemp = robot_dz;
	if(source == 1 && h_refParam == 0 && robot_dzTemp != 0) robot_dz = robot_dzTemp;

//	if(_imu == 0){
//		kat_X = 0;//imu_rot_x;
//		kat_Y = 0; //imu_rot_y;
//	}
	//else{
	//	kat_X = imu_rot_x;
	//	kat_Y = imu_rot_y;
	//}

        //Zapisuje stan robota w momencie skanu: deltaX - przemieszczenie w X, deltaY - przemieszczenie w Y, robot_dz -  wyznaczona wysokosc robota, kat_X - kat Pitch platformy, kat_Y - kat Roll platformy, kat_Z - obrot robota
	//saveRobotState2file(deltaX, deltaY,robot_dz, kat_X, kat_Y, kat_Z);    	
//	saveRobotState2file(deltaX, deltaY,z, kat_X, kat_Y, kat_Z);//DB

	printf("[deltaX = %.3f, deltaY = %.3f, robot_dz = %.3f, kat_X = %.3f, kat_Y = %.3f, kat_Z = %.3f]\n",deltaX, deltaY,z, kat_X, kat_Y, kat_Z);

	MoveMap(deltaX,deltaY,rad2deg(kat_Z));
    	RotateMap(kat_X, kat_Y, kat_Z);
    	Calculate3DMap(points_no, kat_X, kat_Y, kat_Z, deltaY);
    	CalculateInclination();

   	FilterMap(grid_map, size_x, size_y, "modifiedCAS");
	
//	IplImage* m_img;//obraz
// 	rpccaller_cam->snap();
// 	rpccaller_cam->getImage(&m_img);
	// Save the frame into a file
//	char str [20];

// 	sprintf (str,"img%04d.jpg",image_no);

// 	cvSaveImage(str,m_img);
	image_no++;
    //FilterMap(grid_map2, size_x, size_y, "CAS");
}



// makes 4x4 transform matrix
CPunctum CLocalMap::makeTransformMatrix(const char * type, float value){
  CPunctum tmp;
  tmp.setEye();
  float x,y,z;
  if (!strcmp(type,"x")){
     tmp.pos[0]=value;
  }
  else if (!strcmp(type,"y")){
     tmp.pos[1]=value;
  }
  else if (!strcmp(type,"z")){
     tmp.pos[2]=value;
  }
  else if (!strcmp(type,"alpha")){
	  x=1;
	  y=0;
	  z=0;
	  tmp.rot[0][0]=x*x*(1-cos(value))+cos(value);
	  tmp.rot[0][1]=x*y*(1-cos(value))-z*sin(value);
	  tmp.rot[0][2]=x*z*(1-cos(value))+y*sin(value);

	  tmp.rot[1][0]=y*x*(1-cos(value))+z*sin(value);
	  tmp.rot[1][1]=y*y*(1-cos(value))+cos(value);
	  tmp.rot[1][2]=y*z*(1-cos(value))-x*sin(value);

	  tmp.rot[2][0]=x*z*(1-cos(value))-y*sin(value);
	  tmp.rot[2][1]=y*z*(1-cos(value))+x*sin(value);
	  tmp.rot[2][2]=z*z*(1-cos(value))+cos(value);
  }
  else if (!strcmp(type,"beta")){
	  y=1; z=0; x=0;
	  tmp.rot[0][0] = x*x*(1-cos(value))+cos(value);
	  tmp.rot[0][1] = x*y*(1-cos(value))-z*sin(value);
	  tmp.rot[0][2] = x*z*(1-cos(value))+y*sin(value);

	  tmp.rot[1][0] = y*x*(1-cos(value))+z*sin(value);
	  tmp.rot[1][1] = y*y*(1-cos(value))+cos(value);
	  tmp.rot[1][2] = y*z*(1-cos(value))-x*sin(value);

	  tmp.rot[2][0] = x*z*(1-cos(value))-y*sin(value);
	  tmp.rot[2][1] = y*z*(1-cos(value))+x*sin(value);
	  tmp.rot[2][2] = z*z*(1-cos(value))+cos(value);
  }
  else if (!strcmp(type,"gamma")){
	  z=1; x=0; y=0;
	  tmp.rot[0][0]=x*x*(1-cos(value))+cos(value);
	  tmp.rot[0][1]=x*y*(1-cos(value))-z*sin(value);
	  tmp.rot[0][2]=x*z*(1-cos(value))+y*sin(value);

	  tmp.rot[1][0]=y*x*(1-cos(value))+z*sin(value);
	  tmp.rot[1][1]=y*y*(1-cos(value))+cos(value);
	  tmp.rot[1][2]=y*z*(1-cos(value))-x*sin(value);

	  tmp.rot[2][0]=x*z*(1-cos(value))-y*sin(value);
	  tmp.rot[2][1]=y*z*(1-cos(value))+x*sin(value);
	  tmp.rot[2][2]=z*z*(1-cos(value))+cos(value);
  }
  return tmp;
}

///get mesh_x_size
int CLocalMap::getXnumPoints(){//get mesh_x_size
  return size_x;
}

///get mesh_y_size
int CLocalMap::getYnumPoints(){//get mesh_y_size
  return size_x;
}

///get x size in meters
float CLocalMap::getXsize()
{
  return size_x*raster_x;
}

///get y size in meters
float CLocalMap::getYsize()
{
  return size_y*raster_y;
}

///get x raster size in meters
float CLocalMap::getXRastersize(){
  return raster_x;
}

///get y raster size in meters
float CLocalMap::getYRastersize(){
  return raster_y;
}

///wyznacza wspolczynnik skali dla X
float  CLocalMap::getScaleFactorX()
{
 float factorX;
  return factorX=getXsize()/getXnumPoints();
}

///wyznacza wspolczynnik skali dla Y
float CLocalMap::getScaleFactorY()
{
  float factorY;
  return factorY=getYsize()/getYnumPoints();
}

///wyznacza wspolrzedne X srodka terenu w jego wspolrzednych
float CLocalMap::getCenterX()
{
  float centerX;
  return centerX=getXsize()/2;
}

///wyznacza wspolrzedne Y srodka terenu w jego wspolrzednych
float CLocalMap::getCenterY()
{
  float centerY;
  return centerY=getYsize()/2;
}

/// save to file
void CLocalMap::saveMap2file(char * nazwa_pliku){
	
	char buffer [40];
	sprintf(buffer, "pliki/mapy/%s.m",nazwa_pliku);
	
	FILE * plik;
	//plik = fopen(filename,"w+t");
	plik = fopen(buffer,"w+t");

	fprintf(plik,"close all; clear all\n");
	fprintf(plik,"[X,Y] = meshgrid(%f:%f:(%f-%f),%f:%f:(%f-%f));\n",-getXsize()/2,getXRastersize(),getXsize()/2,getXRastersize(), -getYsize()/2,getYRastersize(),getYsize()/2,getYRastersize());
	fprintf(plik,"Z=[];\n");
	float elev=0;
	for (int i=0; i<getXnumPoints();i++){
	  for (int j=0; j<getYnumPoints();j++){
	    elev = 0.75*GetElevation(i, j);
	    if (elev==-1)
	      fprintf(plik,"Z(%d,%d)=%f; ",i+1,j+1,0.0);
	    else if (elev>0.5)
	      fprintf(plik,"Z(%d,%d)=%f; ",i+1,j+1,0.5);
	    else if (elev<-0.3)
	      fprintf(plik,"Z(%d,%d)=%f; ",i+1,j+1,-0.3);
	    else
	      fprintf(plik,"Z(%d,%d)=%f; ",i+1,j+1,elev);
	  }
	}
	fprintf(plik,"\nsurf(X,Y,Z);\n");
	fprintf(plik,"axis([%f %f %f %f -0.2 0.5])\n",-getXsize()/2,getXsize()/2,-getYsize()/2,getYsize()/2);
	fprintf(plik,"xlabel('x');\n");
	fprintf(plik,"ylabel('y');\n");
	fprintf(plik,"zlabel('z');\n");
	fprintf(plik,"view([-167.5, 70]);\n");
	
	fclose(plik);

	//saveMap2dat(filename);
}

// save map to .dat
void CLocalMap::saveMap2dat(char * nazwa_pliku){

	char buffer_wys [40];
	char buffer_nach_x [40];
	char buffer_nach_y [40];
	char buffer_wiar [40];
  	//char nazwa_pliku[10];

  	sprintf(buffer_wys, "pliki/mapy/%s-wysokosc.dat",nazwa_pliku);
	sprintf(buffer_nach_x, "pliki/mapy/%s-nachylenie_x.dat",nazwa_pliku);
	sprintf(buffer_nach_y, "pliki/mapy/%s-nachylenie_y.dat",nazwa_pliku);
	sprintf(buffer_wiar, "pliki/mapy/%s-wiarygodnosc.dat",nazwa_pliku);

	FILE* str_wys  = fopen(buffer_wys,"w"); 
	FILE* str_nach_x = fopen(buffer_nach_x,"w");
	FILE* str_nach_y = fopen(buffer_nach_y,"w");
	FILE* str_wiar = fopen(buffer_wiar,"w");

	float wsp;

	if( str_wys != NULL && str_nach_x != NULL && str_nach_y != NULL && str_wiar != NULL ){

		char komorka[ 30 ];
		char linia[ 3000 ];

		sprintf(linia, "0 ");
		for( int x = 0; x < size_x; x++ ){
			wsp = x * raster_x - ((float)size_x / 2 * raster_x);
			sprintf( komorka,"%f ", wsp);
			strcat( linia, komorka );
		}
		strcat( linia, "\n" );
		fprintf(str_wys,linia);
		fprintf(str_nach_x,linia);
		fprintf(str_nach_y,linia);
		fprintf(str_wiar,linia);

		for( int x = 0; x < size_y; x++ ){
			wsp = x * raster_y - ((float)size_y / 2 * raster_y);
			wsp *= -1;
			sprintf(linia, "%f ", wsp);
			for( int y = 0; y < size_x; y++ ){
				sprintf( komorka,"%f ", GetElevation(y, x));
				strcat( linia, komorka );
			}
			strcat( linia, "\n" );
			fprintf(str_wys,linia);
			sprintf(linia, "");
			sprintf(linia, "%f ", wsp);
			for( int y = 0; y < size_x; y++ ){
				sprintf( komorka,"%f ", GetInclination_X(y, x));
				strcat( linia, komorka );
			}
			strcat( linia, "\n" );
			fprintf(str_nach_x,linia);
			sprintf(linia, "");
			sprintf(linia, "%f ", wsp);
			for( int y = 0; y < size_x; y++ ){
				sprintf( komorka,"%f ", GetInclination_Y(y, x));
				strcat( linia, komorka );
			}
			strcat( linia, "\n" );
			fprintf(str_nach_y,linia);
			sprintf(linia, "");
			sprintf(linia, "%f ", wsp);
			for( int y = 0; y < size_x; y++ ){
				sprintf( komorka,"%f ", GetCertainty(y, x));
				strcat( linia, komorka );
			}
			strcat( linia, "\n" );
			fprintf(str_wiar,linia);
			sprintf(linia, "");
		}

			fclose(str_wys);
			fclose(str_nach_x);
			fclose(str_nach_y);
			fclose(str_wiar);
		}

	


}

//Wyznacz wysokość robota (funkcja zakłada, że wszystkie nogi mają kontakt z podłożem)
float CLocalMap::CalculateRobotHight(float *sf, float kat_X, float kat_Y ){

    //Wyznaczenie macierzy przejście z układu globalnego do układu platformy
     CPunctum Tg_pl;
	 Tg_pl = makeTransformMatrix("alpha", kat_X)*makeTransformMatrix("beta",kat_Y);

    //Macierz O_0
     CPunctum O_0;
	 O_0.setEye();
     O_0.pos[0] = width_min_; //0.065;
     O_0.pos[1] = length_ - scanner_y;	//-0.046711;
     O_0.pos[2] = -scanner_z;	//-0.156;//0.149; //0.181;

     //Macierz O_1
     CPunctum O_1;
	 O_1.setEye();
     O_1.pos[0] = width_max_;//0.13;
     O_1.pos[1] = -scanner_y;
     O_1.pos[2] = -scanner_z;	//-0.156;

    //Macierz O_2
     CPunctum O_2;
	 O_2.setEye();
     O_2.pos[0] = width_min_; //0.065;
     O_2.pos[1] = -length_-scanner_y;	//-0.355289;
     O_2.pos[2] = -scanner_z;	//-0.156;

    //Macierz O_3
     CPunctum O_3;
	 O_3. setEye();
     O_3.rot[0][0] = -1;
     O_3.pos[0] = -width_min_;//-0.065;
     O_3.pos[1] = -length_-scanner_y;	//-0.355289;
     O_3.rot[2][2] = -1;
     O_3.pos[2] = -scanner_z;

     //Macierz O_4
     CPunctum O_4 ;
	 O_4.setEye();
     O_4.rot[0][0] = -1;
     O_4.pos[0] = -width_max_; //-0.13;
     O_4.pos[1] = -scanner_y;
     O_4.rot[2][2] = -1;
     O_4.pos[2] = -scanner_z;	//-0.156;

     //Macierz O_5
     CPunctum O_5;
	 O_5.setEye();
     O_5.rot[0][0] = -1;
     O_5.pos[0] = -width_min_;//-0.065;
     O_5.pos[1] = length_ - scanner_y;	//-0.046711;
     O_5.rot[2][2] = -1;
     O_5.pos[2] = -scanner_z;	//-0.156;

     if( foot == 0 ) foot = new SFoot[ 6 ];

     float dz = 0;
     int leg_nr = 0;

     float dz_firstLeg = 0;

     do{
         CPunctum s_f;
		 s_f.setZero();
         s_f.pos[0] = sf[leg_nr*3 + 0];
         s_f.pos[1] = sf[leg_nr*3 + 1];
         s_f.pos[2] = sf[leg_nr*3 + 2];

         CPunctum s_pl;

         switch(leg_nr){
             case 0:
                 s_pl = O_0*s_f;
                 break;
             case 1:
                 s_pl = O_1*s_f;
                 break;
             case 2:
                 s_pl = O_2*s_f;
                 break;
             case 3:
                 s_pl = O_3*s_f;
                 break;
             case 4:
                 s_pl = O_4*s_f;
                 break;
             case 5:
                 s_pl = O_5*s_f;
                 break;
         }

         CPunctum s_g;
		 s_g = Tg_pl * s_pl;

         //Wyznczamy współrzędne komórki w której znajduje się noga robota
		printf("x_f = %f \n", s_g.pos[0]);
		printf("y_f = %f \n", s_g.pos[1]);


            foot[ leg_nr ].x = (int)(s_g.pos[0] / raster_x + (float)size_x / 2);
            foot[ leg_nr ].y = (int)((float)size_y / 2 - s_g.pos[1] / raster_y);

         //Składowa wysokości dz_1
            float dz_1 = -s_g.pos[2];
	    if(leg_nr == 0) dz_firstLeg = dz_1;

         //Wyznaczamy wysokość na jakiej stoi stopa robota uwzględniając komórki sąsiadujące (okienko 3x1)
            int n = 0;
            float h_ref = 0;


            for(int dx = -1; dx < 2; dx++ )
                            for(int dy = 0; dy < 1; dy++ )	//for( dy = -1; dy < 2; dy++ )
                                    if(true /*GetCertainty(foot[ leg_nr ].x+dx, foot[ leg_nr ].y+dy) > 0 */){ // badanie wysokosci teren wokol stopy w oknie 3x3
                                            h_ref += GetElevation(foot[ leg_nr ].x+dx, foot[ leg_nr ].y+dy); // uwzgledniane sa tylko te komorki, ktorych wartosc byla juz kiedys zmierzona
                                            n++;
                                    }
					
             if( n!=0 && h_refParam==1){ //Jeśli robot jest na znanej wysokosci i parametr h_ref ma być uwzględniany wyznacza h_ref
	     		h_ref /= n;	// wyznaczenie sredniej wartosci wysokosci terenu w oknie 3x1 wokol stopy!
                	dz = dz_1 + (h_ref/100);
             }

     	     leg_nr++;

     }while(dz==0 && leg_nr<5);

     if( dz == 0 ) dz = dz_firstLeg;    // jezeli zadna stopa nie znajduje sie w komorce o znanej zmierzonej wysokosci
					// wowczas h_ref = 0 a dz wynika z wysokosci dz_firsLeg 
     robot_dz = dz;

return dz;



}

/*void CLocalMap::setAngles(float kat_X, float kat_Y){
		imu_rot_x = kat_X;
		imu_rot_y = kat_Y;
		printf("kat X = %f \n", rad2deg(imu_rot_x));
		printf("kat Y = %f \n \n", rad2deg(imu_rot_y));
}*/

// save robot state
void CLocalMap::saveRobotState2file(float x,float y,float z,float alpha,float beta,float gamma){

			robotState = fopen ("pliki/wynik/StanRobota.txt","a");

		        fprintf(robotState,"deltaX = %f deltaY = %f  robot_dz = %f aplha = %f beta = %f gamma = %f \n", x, y, z, alpha, beta, gamma);

			fclose (robotState);
}

// read robot state
void CLocalMap::readRobotState(float * x,float * y,float * z,float * alpha,float * beta,float * gamma){

	char str[200];// param_name[50];//wiersz z pliku

	if (fgets (str, 200, readRobot)) {
		sscanf (str,"deltaX = %f deltaY = %f  robot_dz = %f aplha = %f beta = %f gamma = %f ", x, y, z, alpha, beta, gamma);
	}
	else {
	fclose(readRobot);
	printf("Brak informacji o przesunięciu i orientacji robota.\n");
	printf("Sprawdź czy pliki 'skany.txt' oraz 'StanRobota.txt' dotycza tego samego pomiaru. \n");
	printf("Aplikacja zostanie zamknięta .. \n");	
	exit(0);
	}

}

// wyznacza delta_z_max
float CLocalMap::CalculateMaxErr(float kat_X, float kat_Y, float kat_rad, float Alfa, float gamma, float deltaY, float l){


	float dAlfa = 0.01745;
    	float  dl = 0.01;

	float err = 0;			// calkowity błąd pomiaru wysokosci
  	float delta_zg_phi;		// błędy cząstkowe - pochodne cząstkowe * błąd
  	float delta_zg_psi; 		
  	float delta_zg_alpha; 			
  	float delta_zg_l; 			

	delta_zg_l    = -cos(kat_X)*sin(kat_Y)*sin(kat_rad) + cos(kat_rad)*sin(kat_X)*cos(-Alfa) + cos(kat_rad)*cos(kat_X)*sin(-Alfa)*cos(kat_Y);
	delta_zg_alpha = -l*cos(kat_rad)*sin(kat_X)*sin(-Alfa) + l*cos(kat_rad)*cos(kat_X)*cos(-Alfa)*cos(kat_Y);
	delta_zg_psi  = -cos(kat_X)*cos(kat_Y)*l*sin(kat_rad) - l*cos(kat_rad)*cos(kat_X)*sin(-Alfa)*sin(kat_Y);
	delta_zg_phi   =  sin(kat_X)*sin(kat_Y)*l*sin(kat_rad) + l*cos(kat_rad)*cos(kat_X)*cos(-Alfa) - l*cos(kat_rad)*sin(kat_X)*sin(-Alfa)*cos(kat_Y);


	delta_zg_l = (float)(dl*delta_zg_l);
	delta_zg_alpha = (float)(dAlfa*delta_zg_alpha);
	delta_zg_psi = (float)(dAlfa*delta_zg_psi);
	delta_zg_phi = (float)(dAlfa*delta_zg_phi);

	err = std::abs(delta_zg_alpha) + std::abs(delta_zg_l) + std::abs(delta_zg_phi) + std::abs(delta_zg_psi);
	err *= 100; // błąd w cm
	err += (deltaY*100) * tan(-gamma);	

	return err;
}

// wyznacza delta_y na podstawie obrotu
float CLocalMap::CalculateDeltaY_afterRotate(float x_s, float y_s, float kat_Z){
	float deltaY_r = 0;
	float d_r = scanner_y*100;
	float d = y_s;	
	float kat = std::abs(kat_Z);
	float x_p;

	//float x_p = 2*(d_r + d)*cos(kat) - d_r - d*(cos(kat)/sin(kat));

	if(kat_Z>0) x_p = (d + d_r)*(sin(kat/2)/cos(kat/2));	
	else x_p = (-1)*(d + d_r)*(sin(kat/2)/cos(kat/2));

	float deltaX = x_s - x_p;
	deltaY_r = deltaX*(sin(kat_Z)/cos(kat_Z));

	deltaY_r = deltaY_r/100;
	//printf("xs = %f ; xp = %f ; d = %f deltaY_r = %f \n", x_s,x_p,d,deltaY_r);

	return deltaY_r;
}

///  przelicza wspolrzedne terenu na wspolrzedne lokalne
void CLocalMap::CalculateLocalCoordinates(int x, int y, float res[2]){
	double centerX=getCenterX();
	double centerY=getCenterY();
	double factorX=getScaleFactorX();
	double factorY=getScaleFactorY();

	res[0]=(x)*factorX-centerX+(factorX/2);

	res[1]=-(y*factorY-centerY+(factorY/2));  
}

///przelicza wspolrzedne robota na wspolrzedne terenu
void CLocalMap::CalculateGroundCoordinates(double x, double y, int * result){
	double centerX=getCenterX();
	double centerY=getCenterY();
	double factorX=getScaleFactorX();
	double factorY=getScaleFactorY();

	result[0]=my_round((x+centerX-(factorX/2))/factorX);

	result[1]=my_round((-y+centerY-getScaleFactorY()/2)/getScaleFactorY());  
}

//funkcja do zaokraglania
int CLocalMap::my_round (double x) 
{
	int i = (int) x;
	if (x >= 0.0) 
	{
		return ((x-i) >= 0.5) ? (i + 1) : (i);
	} 
	else 
	{
		return (-x+i >= 0.5) ? (i - 1) : (i);
	}
} 

//get max height from the square
double CLocalMap::getMaxSquareHeight(int x, int y, int size)
{
	float max=-1e13;
	if ((x<0)||(y<0)||(x>=size_x)||(y>=size_y)) return -1;
	else
		max = GetElevation(x, y);
	for (int i=-size;i<=size;i++){
	    for (int j=-size;j<=size;j++){
		if ((x+i<0)||(y+j<0)||(x+i>=size_x)||(y+j>=size_y)) return -1;
		else if (GetElevation(x+i, y+j)>max)
			max = GetElevation(x+j, y+j);
	    }
	}
	return max;
}

/// oblicz wspolczynniki jakosci wariancja sferyczna
float CLocalMap::ComputeSphericalVariance(int size, int x, int y){
	size = 1;
	float ** normal;
	int max = (size*2)*(size*2)*2;
	normal = new float *[max];
	for (int i=0;i<max;i++)
		normal[i]=new float[3];
	float** vert;
	for (int i=0; i<3; i++)
	{
		vert=new float*[3];
		vert[i]=new float[3];
	}
	int iter=0;
	for (int i=-size;i<size;i++){
		for (int j=-size;j<size;j++) {
			CalculateLocalCoordinates(x+i*4, y+j*4, &(vert[0][0]));
			CalculateLocalCoordinates(x+i*4+4, y+j*4+4, &(vert[1][0]));
			CalculateLocalCoordinates(x+i*4+4, y+j*4, &(vert[2][0]));
			vert[0][2]=getMaxSquareHeight(x+i*4, y+j*4, 3);//points[x+i*4][y+j*4][l];
			vert[1][2]=getMaxSquareHeight(x+i*4+4, y+j*4+4, 3);//points[x+i*4+1][y+j*4+1][l];
			vert[2][2]=getMaxSquareHeight(x+i*4+4, y+j*4, 3);//points[x+i*4+1][y+j*4][l];
			if (iter==max)
				int rr=4;
			calcNormal(vert, normal[iter]);
			iter++;
			CalculateLocalCoordinates(x+i*4, y+j*4, &(vert[0][0]));
			CalculateLocalCoordinates(x+i*4, y+j*4+4, &(vert[1][0]));
			CalculateLocalCoordinates(x+i*4+4, y+j*4+4, &(vert[2][0]));
			vert[0][2]=getMaxSquareHeight(x+i*4, y+j*4, 3);//points[x+i*4][y+j*4][l];
			vert[1][2]=getMaxSquareHeight(x+i*4, y+j*4+4, 3);//points[x+i*4+1][y+j*4+1][l];
			vert[2][2]=getMaxSquareHeight(x+i*4+4, y+j*4+4, 3);//points[x+i*4+1][y+j*4][l];
			calcNormal(vert, normal[iter]);
			iter++;
		}
	}
	float sum[3]={0,0,0};
	for (int i=0;i<max;i++){
		sum[0]+=normal[i][0];
		sum[1]+=normal[i][1];
		sum[2]+=normal[i][2];
	}
	float R = sqrt(pow(sum[0],2)+pow(sum[1],2)+pow(sum[2],2));
	float omega=1-(R/max);

	for (int i=0; i<3; i++)
	{
		delete [] vert[i];
	}
	delete [] vert;

	for (int i=0;i<max;i++)
		delete [] normal[i];
	delete [] normal;
	return omega;
}
	

//rysowanie mapy mierzonej przez robota
void CLocalMap::DrawLocalMap(void)
{
	robsim::float_type pos[3];
	//pobieranie pozycji robota celem przesuwania mapy wraz ze zmianą położenia robota
	map->dynamicWorld->robotODE->getRPY(pos);
	draw_robot_position[0] = pos[0]; draw_robot_position[1] = pos[1]; draw_robot_position[2] = pos[2];

	// mapa zbudowana z prostopadłościanów bez dolnych ścianek - celem przyspieszenia rysowania
	for (int i=0; i<getXnumPoints(); i++)
	{
		for (int j=0; j<getYnumPoints(); j++)
		{
				glColor3f(0.0, 0.99, 0.0);
			
				glPushMatrix();
				glTranslatef((draw_robot_position[0]+((i-getXnumPoints()/2)*raster_x))*10,0.0,(-draw_robot_position[1]+((j-getYnumPoints()/2)*raster_y))*10);
				glBegin(GL_QUADS);
					glNormal3f(0.0,1.0,0.0);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10, -raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10, raster_y*5);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10, raster_y*5);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10, -raster_y*5); 

					glNormal3f(0.0,0.0,-1.0);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10-0.2, -raster_y*5);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10, -raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10, -raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10-0.2, -raster_y*5);
					 
					glNormal3f(0.0,0.0,1.0);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10-0.2, raster_y*5);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10, raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10, raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10-0.2, raster_y*5);
					 
					glNormal3f(1.0,0.0,0.0);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10-0.2, raster_y*5);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10, raster_y*5);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10, -raster_y*5);
					glVertex3f(raster_x*5, (GetElevation(i,j))*10-0.2, -raster_y*5);

					glNormal3f(-1.0,0.0,0.0);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10-0.2, raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10, raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10, -raster_y*5);
					glVertex3f(-raster_x*5, (GetElevation(i,j))*10-0.2, -raster_y*5);

				glEnd();
				glPopMatrix();
		}
	}
}