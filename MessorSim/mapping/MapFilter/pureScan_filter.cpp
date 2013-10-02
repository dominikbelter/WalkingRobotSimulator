#include "pureScan_filter.h"

#define PI 3.14159265



void CPureScan_filter::FilterScan(vector<long> &r, const char* type)
{
    if (!strcmp(type,"max_angle")){
        MaxAngle_segmentation(r);
    }
    else if (!strcmp(type,"odleg")){
        Odleg_segmentation(r);
    }
    else if (!strcmp(type,"stosun")){
        SegmentacjaStosun(r);
    }
    else if (!strcmp(type,"dist_correct")){
	DistanceCorrection(r);
    }
}


void CPureScan_filter::WczytajKaty(){

	ifstream file("kat_rad.txt");
	string line;

        while ( getline(file, line) )
        {
            float value;
            istringstream iss(line);
            while (iss >> value)
            {
                kat_radiany.push_back(value);
            }
        }
}

float CPureScan_filter::MeasureErr(int measure)
{
    float err;

    if(measure<1000) err=10;
    else err=measure/100;
return err;
}


void CPureScan_filter::MaxAngle_segmentation(vector<long> &r){

    int group[726];
    float sstd[726];
    float Dphi=0.36*PI/180;
    float theta_imax=105*PI/180;

    int scan_size = r.size();

    for(int i=0;i<scan_size;i++)
    {
       sstd[i]=MeasureErr(r[i]);
    }

    float kmin=cos(theta_imax+Dphi)/cos(theta_imax);
    float kmax=cos(theta_imax-Dphi)/cos(theta_imax);

    int grup_nr=1; //okresla ilosc grup
    group[0]=0;
    
    for(int i=0;i<scan_size-1;i++){ //petla grupowania
        if( ((r[i+1]>(r[i]*kmin-3*sstd[i])) && (r[i+1]<(r[i]*kmax+3*sstd[i]))) )
	{}
        else{
            group[grup_nr]=i+1;
            grup_nr++;
	}
    }

    int grup_index = 0;
    int grup_size;

    for(int i=0; i<scan_size; i++){
        if(group[grup_index]==i){
            if(grup_index!=grup_nr-1)
                grup_size = group[grup_index+1]-group[grup_index];
            else
                grup_size = scan_size-group[grup_nr-1];

            grup_index++;
        }
        
        if(grup_size>4){}
        else r[i]=0;
    }
}

 void CPureScan_filter::Odleg_segmentation(vector<long>& r){
    float dmax=50;
    float x[726],y[726],d[726];
    int group[726];

    dmax=8;//graniczna odleglosc euklidesow

    int scan_size = r.size();

    // algorytm grupowania

    for(int i=0;i<scan_size;i++){
	x[i] = (r[i] * sin(- kat_radiany[i] ) ); 
	y[i] = (r[i] * cos( kat_radiany[i] ) ); 
    }
    for(int i=0;i<scan_size;i++){
	   d[i]=sqrt((x[i+1]-x[i])*(x[i+1]-x[i])+(y[i+1]-y[i])*(y[i+1]-y[i]));  //odleglosc euklidesowa 2D
    }

    int grup_nr=1;   //okresla ilosc grup
    group[0]=0;      

    for(int i=0;i<scan_size-1;i++){   //petla grupowania
        if(d[i]>dmax){
            group[grup_nr]=i+1;
            grup_nr++;
	}
    }

    int grup_index = 0;
    int grup_size;

    for(int i=0; i<scan_size; i++){
        if(group[grup_index]==i){
            if(grup_index!=grup_nr-1)
                grup_size = group[grup_index+1]-group[grup_index];
            else
                grup_size = scan_size-group[grup_nr-1];

            grup_index++;
        }

        if(grup_size>4){}
        else r[i]=0;
    }
 }

  void CPureScan_filter::SegmentacjaStosun(vector<long>& r){

      float x[726],y[726],d[726];
      float kmax, dmax;
      int group[726];

      dmax=20;//graniczna odleglosc euklidesow
      kmax=1.18;//graniczny stosunek odleglosci

      int scan_size = r.size();

      for(int i=0;i<scan_size;i++){
          x[i] = (r[i] * sin(- kat_radiany[i] ) ); 
          y[i] = (r[i] * cos( kat_radiany[i] ) ); 
      }

      for(int i=0;i<scan_size;i++){
	  d[i]=sqrt((x[i+1]-x[i])*(x[i+1]-x[i])+(y[i+1]-y[i])*(y[i+1]-y[i]));  //obliczenie odleglosci euklidesowej
      }

      int grup_nr=1;	//okresla ilosc grup
      group[0]=0;      

      for(int i=1;i<scan_size-1;i++){   //petla grupowania
          if(d[i]<dmax || ((d[i-1]/d[i]>kmax)&&(r[i-1]!=0))){
              if(d[i]>dmax){}
          }
          else{
              group[grup_nr]=i+1;
              grup_nr++;
          }
      }

      int grup_index = 0;
      int grup_size;

      for(int i=0; i<scan_size; i++){
          if(group[grup_index]==i){
              if(grup_index!=grup_nr-1)
                  grup_size = group[grup_index+1]-group[grup_index];
              else
                  grup_size = scan_size-group[grup_nr-1];

              grup_index++;
           }

          if(grup_size>4){}
          else r[i]=0;
       }
  }

void CPureScan_filter::DistanceCorrection(vector<long>& r){

	int scan_size = r.size();
	
	
	for(int i=0;i<scan_size;i++){
		//long temp  = (long)(r[i]*0.036635165 - 23.4);
		//long temp  = (long)(-r[i]*0.156635165); //+ 140);		

		if(r[i]>40) {
		  long temp = (long)(-r[i] * 0.24 + (560*0.24));		 //0.23635165
		  r[i] = r[i] - temp;	
		}
     	}
}
