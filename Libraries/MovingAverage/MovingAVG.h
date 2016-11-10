#ifndef _MOVING_AVG_H
#define _MOVING_AVG_H

#define FilterWindowSize 10

class MovingAvgInt{
private:
	int currentPos=0,currentSize=0,sum=0;
	int data[FilterWindowSize];
public:
	MovingAvgInt(){
	};
	void add(int d){
		if(currentSize<FilterWindowSize) currentSize+=1;
		else sum-=data[currentPos];
		data[currentPos]=d;
		sum+=d;
		currentPos=(currentPos+1)%FilterWindowSize;
	}
	int read(){
		return sum/currentSize;
	}
};

class MovingAvgFloat{
private:
	int currentPos=0,currentSize=0;
	float data[FilterWindowSize];
	float sum=0;
public:
	MovingAvgFloat(){
	};
	void add(float d){
		if(currentSize<FilterWindowSize) currentSize+=1;
		else sum-=data[currentPos];
		data[currentPos]=d;
		sum+=d;
		currentPos=(currentPos+1)%FilterWindowSize;
	}
	int read(){
		return sum/currentSize;
	}
};

#endif