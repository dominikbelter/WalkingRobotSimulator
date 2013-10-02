#ifndef _pureScan_filter_H
#define _pureScan_filter_H

#include <string.h>
#include <vector>
#include <math.h>
#include <cstdio>

#include <fstream>
#include <sstream>

using namespace std;

class CPureScan_filter
{
	private:
            void MaxAngle_segmentation(vector<long>& r);
            void Odleg_segmentation(vector<long>& r);
            void SegmentacjaStosun(vector<long>& r);
            float MeasureErr(int measure);
            void WczytajKaty();
		
	    void DistanceCorrection(vector<long>& r);
	
            vector<float> kat_radiany;
        public:
            CPureScan_filter(){ WczytajKaty();}
            void FilterScan(vector<long> &r, const char * type);
            void FilterScan();

};

#endif
