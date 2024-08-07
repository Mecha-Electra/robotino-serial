//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#ifndef _REC_IOCONTROL_REMOTESTATE_SETSTATE_H_
#define _REC_IOCONTROL_REMOTESTATE_SETSTATE_H_

#include <cmath>

#define REC_IOCONTROL_REMOTESTATE_SETSTATE_KEY "REC_IOCONTROL_REMOTESTATE_SETSTATE_KEY"
	/**
	@brief The SetState is the collection of all set point values being send to Robotino.
	*/
class SetState{
	public:
		/*
		SetState();

		SetState(const SetState& other);

		SetState& operator=(const SetState& other);
		void reset();

		bool fromQDSAProtocol(const unsigned char* data);

		void toQDSAProtocol(unsigned char* buffer) const;
		*/

		bool brake[4];

		/// Unit: RPM
		float speedSetPoint[4];

		bool resetPosition[4];

		/** proportional constant */
		unsigned char kp[4];
		/** integral constant */
		unsigned char ki[4];
		/** differential constant */
		unsigned char kd[4];
		/** digital outputs */
		bool dOut[8];
		/** relays */
		bool relays[2];

		bool shutdown;

	inline void reset()
	{

		//Set default values
		for (int mi = 0; mi < 4; mi++)
		{
			kp[mi] = 255;
			ki[mi] = 255;
			kd[mi] = 255;
			brake[mi] = false;
			speedSetPoint[mi] = 0.0f;
			resetPosition[mi] = false;
		}

		for (int i = 0; i < 8; i++)
		{
			dOut[i] = false;
		}

		for (int i = 0; i < 2; i++)
		{
			relays[i] = false;
		}

		shutdown = false;
	}
	inline bool fromQDSAProtocol(const unsigned char* data)
	{
		float speed[3];

		if (*(data) != 'R' ||
			*(data + 1) != 'E' ||
			*(data + 2) != 'C' ||
			*(data + 44) != 'r' ||
			*(data + 45) != 'e' ||
			*(data + 46) != 'c')
		{
			return false;
		}

		//from master
		shutdown = ((*(data + 3) & 1 << 1) > 0);

		/*
		isRobotinoView_1_7 = ( ( *(data+3) & 1<<3 ) > 0 );
		*/

		//from slave 0
		dOut[0] = ((*(data + 4) & 1 << 1) > 0);
		dOut[1] = ((*(data + 4) & 1 << 2) > 0);
		dOut[2] = ((*(data + 4) & 1 << 3) > 0);
		dOut[3] = ((*(data + 4) & 1 << 4) > 0);
		relays[0] = ((*(data + 4) & 1 << 5) > 0);
		speed[0] = *(data + 6);
		if (0 == (*(data + 5) & (1 << 1)))
		{
			speed[0] = -speed[0];
		}

		resetPosition[0] = ((*(data + 5) & (1 << 3)) > 0 ? true : false);

		kp[0] = *(data + 11);
		ki[0] = *(data + 12);
		kd[0] = *(data + 13);

		//from slave 1
		dOut[4] = ((*(data + 14) & 1 << 1) > 0);
		dOut[5] = ((*(data + 14) & 1 << 2) > 0);
		dOut[6] = ((*(data + 14) & 1 << 3) > 0);
		dOut[7] = ((*(data + 14) & 1 << 4) > 0);
		relays[1] = ((*(data + 14) & 1 << 5) > 0);
		speed[1] = *(data + 16);
		if (0 == (*(data + 15) & (1 << 1)))
		{
			speed[1] = -speed[1];
		}

		resetPosition[1] = ((*(data + 15) & (1 << 3)) > 0 ? true : false);

		kp[1] = *(data + 21);
		ki[1] = *(data + 22);
		kd[1] = *(data + 23);

		//from slave 2
		//*(data+24);
		speed[2] = *(data + 26);
		if (0 == (*(data + 25) & (1 << 1)))
		{
			speed[2] = -speed[2];
		}

		resetPosition[2] = ((*(data + 25) & (1 << 3)) > 0 ? true : false);

		kp[2] = *(data + 31);
		ki[2] = *(data + 32);
		kd[2] = *(data + 33);

		//from slave 3
		//*(data+34);
		speed[3] = *(data + 36);
		if (0 == (*(data + 35) & (1 << 1)))
		{
			speed[3] = -speed[3];
		}

		resetPosition[3] = ((*(data + 35) & (1 << 3)) > 0 ? true : false);

		for (unsigned int i = 0; i < 4; ++i)
		{
			speedSetPoint[i] = speed[i] * 900.0f * 60.0f / 2000.0f;
		}

		return true;
	}

	inline void toQDSAProtocol(unsigned char* buffer) const
	{
		//speedSetPoint is in rpm, speed in inc/900ms
		float speed[4];
		for (unsigned int i = 0; i < 4; ++i)
		{
			// 2000 = Encoder Resolution
			// 900 = Update Frequency in Hz
			// 60 = 1 min in seconds
			speed[i] = speedSetPoint[i] / 900.0f / 60.0f;
			if (speed[i] > 255.0f)
			{
				speed[i] = 255.0f;
			}
			else if (speed[i] < -255.0f)
			{
				speed[i] = -255.0f;
			}
		}

		*buffer = 'R';
		*(buffer + 1) = 'E';
		*(buffer + 2) = 'C';

		//to master
		*(buffer + 3) = (shutdown ? 1 << 1 : 0);

		/*This bit is ignored by Robotino's IO board with Atmel microcontroller
		The IO board with LPC microcontroller uses this bit to check, if old software (Robotino View <1.7) is connected.
		Old software does not set this bit. New software (Robotino View >1.7) sets this bit.
		If this bit is clear settings for kp, ki and kd are ignored.
		*/
		*(buffer + 3) |= (1 << 3);

		//to slave 0
		*(buffer + 4) = 1; //brake off
		*(buffer + 4) |= (dOut[0] ? 1 << 1 : 0);
		*(buffer + 4) |= (dOut[1] ? 1 << 2 : 0);
		*(buffer + 4) |= (dOut[2] ? 1 << 3 : 0);
		*(buffer + 4) |= (dOut[3] ? 1 << 4 : 0);
		*(buffer + 4) |= (relays[0] ? 1 << 5 : 0);
		*(buffer + 5) = 0;
		if (speed[0] >= 0)
		{
			*(buffer + 5) = (1 << 1);
		}
		if (resetPosition[0])
		{
			*(buffer + 5) |= (1 << 3);
		}
		*(buffer + 6) = static_cast<unsigned char>(fabs(speed[0]));
		*(buffer + 7) = 0;
		*(buffer + 8) = 0;
		*(buffer + 9) = 0;
		*(buffer + 10) = 0;
		*(buffer + 11) = kp[0];
		*(buffer + 12) = ki[0];
		*(buffer + 13) = kd[0];

		//to slave 1
		*(buffer + 14) = 1; //brake off
		*(buffer + 14) |= (dOut[4] ? 1 << 1 : 0);
		*(buffer + 14) |= (dOut[5] ? 1 << 2 : 0);
		*(buffer + 14) |= (dOut[6] ? 1 << 3 : 0);
		*(buffer + 14) |= (dOut[7] ? 1 << 4 : 0);
		*(buffer + 14) |= (relays[1] ? 1 << 5 : 0);
		*(buffer + 15) = 0;
		if (speed[1] >= 0)
		{
			*(buffer + 15) = (1 << 1);
		}
		if (resetPosition[1])
		{
			*(buffer + 15) |= (1 << 3);
		}
		*(buffer + 16) = static_cast<unsigned char>(fabs(speed[1]));
		*(buffer + 17) = 0;
		*(buffer + 18) = 0;
		*(buffer + 19) = 0;
		*(buffer + 20) = 0;
		*(buffer + 21) = kp[1];
		*(buffer + 22) = ki[1];
		*(buffer + 23) = kd[1];

		//to slave 2
		*(buffer + 24) = 1; //brake off
		*(buffer + 25) = 0;
		if (speed[2] >= 0)
		{
			*(buffer + 25) = (1 << 1);
		}
		if (resetPosition[2])
		{
			*(buffer + 25) |= (1 << 3);
		}
		*(buffer + 26) = static_cast<unsigned char>(fabs(speed[2]));
		*(buffer + 27) = 0;
		*(buffer + 28) = 0;
		*(buffer + 29) = 0;
		*(buffer + 30) = 0;
		*(buffer + 31) = kp[2];
		*(buffer + 32) = ki[2];
		*(buffer + 33) = kd[2];

		//to slave 3
		*(buffer + 34) = 1; //brake off
		*(buffer + 35) = 0;
		if (speed[3] >= 0)
		{
			*(buffer + 35) = (1 << 1);
		}
		if (resetPosition[3])
		{
			*(buffer + 35) |= (1 << 3);
		}

		*(buffer + 36) = static_cast<unsigned char>(fabs(speed[3]));
		*(buffer + 37) = 0;
		*(buffer + 38) = 0;
		*(buffer + 39) = 0;
		*(buffer + 40) = 0;
		*(buffer + 41) = kp[3];
		*(buffer + 42) = ki[3];
		*(buffer + 43) = kd[3];

		*(buffer + 44) = 'r';
		*(buffer + 45) = 'e';
		*(buffer + 46) = 'c';
	}
};
#endif
