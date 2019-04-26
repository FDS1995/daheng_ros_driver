#include<sys/time.h>

class CTimeCounter  
{
public: 
    CTimeCounter() 
    { 
        Begin(); 
    } 
    void Begin() 
    { 
        gettimeofday(&tvBegin, &tzBegin);
    } 
    long End() 
    { 
        gettimeofday(&tvEnd, &tzEnd);
        return ((tvEnd.tv_sec * 1000000 + tvEnd.tv_usec) - (tvBegin.tv_sec * 1000000 + tvBegin.tv_usec)); 
    }
 
protected: 
    struct timeval  tvBegin;
    struct timezone tzBegin;
    struct timeval  tvEnd;	
    struct timezone tzEnd;	
};
