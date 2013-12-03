#ifndef RunningMedian_h
#define RunningMedian_h

//    FILE: RunningMedian.h
//  AUTHOR: Rob dot Tillaart at gmail dot com  
// PURPOSE: RunningMedian library for Arduino
// VERSION: 0.1.02
//     URL: http://arduino.cc/playground/Main/RunningMedian
// HISTORY: See RunningMedian.cpp
//
// Released to the public domain
//

#define RUNNINGMEDIANVERSION "0.1.02"
// should at least be 5 to be practical
#define MEDIAN_MIN 1
#define MEDIAN_MAX 25


class RunningMedian 
{
	public:
    RunningMedian(int);
	RunningMedian();
	void clear();
    void add(double);
    double getMedian();

    double getAverage();
    double getHighest();
    double getLowest();

	protected:
    int _size;
    int _cnt;
    int _idx;
    double _ar[MEDIAN_MAX];
    double _as[MEDIAN_MAX];
	void sort();
};

#endif
// END OF FILE
