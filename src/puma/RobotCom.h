#ifndef _ROBOT_COM_H_
#define _ROBOT_COM_H_

#include "unitMsg.h"
#include "cs225.h"

namespace Puma {

class RobotCom {
public:
	RobotCom();
	~RobotCom();

	void _float();
	void _break();

	// mode = FLOAT,
	//        JTRACK, TRACK
	void control( ControlMode mode, float *q, int numArgs );

	// mode = JTRACK
	void jointControl( ControlMode mode, float q0, float q1, float q2, float q3, float q4, float q5);

	// get_type = GET_CURTIME (gv.curTime),
	//            GET_JPOS (gv.q), GET_JVEL (gv.dq), GET_TORQ (gv.tau), 
	//            GET_IPOS (gv.x)
	void getStatus( UiToServoMessageType get_type, float *arg );

	void setStatus( ConstantType set_type, ControlMode control_mode, float *arg, int numArgs );

	void setConstant( ConstantType set_type, float arg );

	// voltage = -10.0 ~ + 10.0
    void controlGripper( float voltage );

	void setSocketBlock( bool fBlock );

private:
#ifdef WIN32
	SOCKET robotSocket;
#else
	int robotSocket;
#endif

    void sendMessage(AMsg &mOut);
    void processBrokenSocket();


	CMappedMsg GetMsg();
	bool IsDataAvailable() { return ready() > 0; }
	int ready();

	int Receive();
	int Peek( char *buff, int nbytes );
	int Peek();
	uint16_t  Unpack2B( char *text );
	int InitByteCounter();
	
	char *buffer_;
	int bufferSize_;
	int byteCounter_;

	uint16_t size_;
	int readSize_;

};

inline int RobotCom::InitByteCounter()
{
  byteCounter_ = 4; // 2*sizeof(short);
  return byteCounter_;
}

}

#endif // _ROBOT_COM_H_
