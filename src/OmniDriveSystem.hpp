#ifndef _OMNIDRIVESYSTEM_H_
#define _OMNIDRIVESYSTEM_H_

#include "DriveLayout.hpp"

class OmniDriveSystem
{
public:
	OmniDriveSystem(const DriveLayout& driveLayout);
	
	void projectVelocity( float* m, const float* maxrpm, float vx, float vy, float omega ) const;
	void unprojectVelocity( float* vx, float* vy, float* omega, float m1, float m2, float m3, float m4 ) const;
private:
	DriveLayout _layout;
};

#endif //_OMNIDRIVESYSTEM_H_
