#ifndef _DRIVELAYOUT_H_
#define _DRIVELAYOUT_H_

class DriveLayout
{
public:
	DriveLayout()
		: rb( 0.17f )
		, rw( 0.06f )
		, gear( 32.0f )
		, left_to_right_wheel( 0.52f )
		, front_to_rear_wheel( 0.55f )
		, encoder( 2000.0f )
	{
	}

	/**
	* distance from center to wheel center
	* Set by stated from robotino.xml.
	*/
	float rb;

	/**
	* radius wheel in m
	* Set by stated from robotino.xml.
	*/
	float rw;

	/**
	* gear
	* Set by stated from robotino.xml.
	*/
	float gear;
	
	float left_to_right_wheel;
	
	float front_to_rear_wheel;
	
	float encoder;
};

#endif //_DRIVELAYOUT_H_
