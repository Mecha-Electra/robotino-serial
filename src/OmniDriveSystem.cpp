#include "OmniDriveSystem.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

OmniDriveSystem::OmniDriveSystem(const DriveLayout& drivelayout)
: _layout(drivelayout)
{
	
}

void OmniDriveSystem::projectVelocity( float* m, const float* maxrpm, float vx, float vy, float omega ) const
{
  //Projection matrix
  static const float v0[2] = { -0.5f * sqrt( 3.0f ),  0.5f };
  static const float v1[2] = {  0.0f              , -1.0f };
  static const float v2[2] = {  0.5f * sqrt( 3.0f ),  0.5f };

  //Scale omega with the radius of the robot
  float vOmegaScaled = _layout.rb * omega ;

  //Convert from m/s to RPM
  const float k = 60.0f * _layout.gear * _layout.encoder / ( 2.0f * M_PI * _layout.rw );
  //Compute the desired velocity
  m[0] = ( v0[0] * vx + v0[1] * vy + vOmegaScaled ) * k;
  m[1] = ( v1[0] * vx + v1[1] * vy + vOmegaScaled ) * k;
  m[2] = ( v2[0] * vx + v2[1] * vy + vOmegaScaled ) * k;
  m[3] = 0;
  
	float scale = 1;
	for( int i=0; i<3; ++i)
	{
		float ma = fabs(m[i]);
		
		if(ma > maxrpm[i]) 
		{
			float s = maxrpm[i]/ma;
			if(s<scale)
			{
				scale = s;
			}
		}
	}
	
	for( int i=0; i<3; ++i)
	{
		m[i] *= scale;
	}
}


void OmniDriveSystem::unprojectVelocity( float* vx, float* vy, float* omega, float m1, float m2, float m3, float /*m4*/ ) const
{
	//std::cout << m1 << " " << m2 << " " << m3 << std::endl;

	//Convert from RPM to mm/s
	const float k = 2.0 * M_PI * _layout.rw / ( 60.0f * _layout.gear );

	*vx = k * ( m3 - m1 ) / sqrt( 3.0f );
	*vy = k * 2.0f / 3.0f * ( m1 + 0.5f * ( m3 - m1 ) - m2 );

	float vw = *vy + k * m2;

	*omega = vw / _layout.rb;
}
