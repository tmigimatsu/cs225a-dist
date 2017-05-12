
#include <iostream>
#include <string>
#include "RobotCom.h"
#include <cstdarg>

#ifndef WIN32
	
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h> 	
#include <unistd.h>
#include <sys/ioctl.h>

#endif //not Win32
using namespace std;
using namespace Puma;

static const string DEFAULT_SERVER = "192.168.2.2"; //youbot server
static const int PR_NETWORK_PORT_CONTROL = 8189;

/*********************************************************************
 * RobotCom constructor:  Create a socket connection to the system that
 * communicates with the robot.
 */
RobotCom::RobotCom()
{
	InitByteCounter();
	bufferSize_ = MAX_MSG_SIZE; //nbytes + InitByteCounter();
	buffer_ = new char[bufferSize_];

	memset(buffer_,'\0',bufferSize_);
	readSize_ = 1;



	// Initialize the WINSOCK module.  Since this is the only object
    // in the program that uses WINSOCK, we hide this initialization
    // here.
    //
#ifdef WIN32
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2,2), &wsaData);
#endif //WIN32

	// Create the socket
    //
	robotSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	
	// Ask the user for the name of the robot server,
    // and connect the socket.
    //
	bool socketIsOpen = false;
	while (!socketIsOpen) {
		int rc;
		
		string robotServer;
		cout << "Please enter the name of the robot server ["
			<< DEFAULT_SERVER << "]: " << flush;
		
		getline(cin, robotServer);

		if (robotServer.empty()) {
			robotServer = DEFAULT_SERVER;
		}

		cout << "You entered "<<robotServer<<endl;

	  // connect()
#ifdef WIN32
	  SOCKADDR_IN serveraddr;
#else
		struct sockaddr_in serveraddr;
		bzero((char *) &serveraddr, sizeof(serveraddr));

		struct hostent *server;
		server = gethostbyname(robotServer.c_str());
#endif

		serveraddr.sin_family = AF_INET;
		serveraddr.sin_port = htons(PR_NETWORK_PORT_CONTROL);

#ifdef WIN32 	 
		serveraddr.sin_addr.s_addr = inet_addr( robotServer.c_str() );
#else

		if (server != nullptr)
			bcopy((char *)server->h_addr, (char *)&serveraddr.sin_addr.s_addr, server->h_length);
#endif  

#ifdef WIN32 	 
		rc = connect( robotSocket, (SOCKADDR *)&serveraddr, sizeof(serveraddr) );
#else
		rc = connect( robotSocket, (struct sockaddr *)&serveraddr, sizeof(serveraddr) );
#endif
		if (rc != 0) {
			cerr << "Error: cannot open socket to '" << robotServer
				<< "' port " << PR_NETWORK_PORT_CONTROL << ". Try again."
				<< endl;
			continue;
		}
			socketIsOpen = true;
	}
	
	// Make the socket non-blocking
	//
	setSocketBlock( false );
}

void RobotCom::setSocketBlock( bool fBlock )
{
	// Make the socket non-blocking
	//
	unsigned long nonBlocking;
	if( !fBlock )
		nonBlocking = 1;
	else
		nonBlocking = 0;

#ifdef WIN32
	ioctlsocket(robotSocket, FIONBIO, &nonBlocking);
#else
	ioctl(robotSocket, FIONBIO, &nonBlocking);
#endif	

}

/*********************************************************************
 * RobotCom destructor
 */
RobotCom::~RobotCom()
{
	delete[] buffer_;

	_break();
#ifdef WIN32
  closesocket(robotSocket);
	WSACleanup();	
#else
	close(robotSocket);	
#endif
    
}



/*********************************************************************
 * Assorted outgoing messages
 */

void  RobotCom::_float()
{
    CMsg mOut;
    mOut.WriteMessageType(CONTROL_MODE);
    mOut.WriteInt(FLOAT);
    sendMessage(mOut);
}

void RobotCom::control( ControlMode mode, float *arg, int numArgs )
{
    CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType(CONTROL_MODE);
    mOut.WriteInt(mode);
	if( numArgs > 0 )
		mOut.WriteFloat( arg, numArgs );
    sendMessage(mOut);
}

void RobotCom::jointControl( ControlMode mode, float q0, float q1, float q2, float q3, float q4, float q5)
{
	if( mode != NJMOVE && mode != JMOVE
	  && mode != NJGOTO && mode != JGOTO
	  && mode != NJTRACK && mode != JTRACK )
	{
		std::cout<<"Warning: RobotCom::jointControl: Unsupported messagetype provided!"<<std::endl;
		return;
	}
    CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType(CONTROL_MODE);
    mOut.WriteInt(mode);
    mOut.WriteFloat(q0);
    mOut.WriteFloat(q1);
    mOut.WriteFloat(q2);
    mOut.WriteFloat(q3);
    mOut.WriteFloat(q4);
    mOut.WriteFloat(q5);
    sendMessage(mOut);
}

void RobotCom::setStatus( ConstantType set_type, ControlMode control_mode, float *arg, int numArgs ) {
	CMsg mOut;
	mOut.Init();
	mOut.WriteMessageType(SET_CONSTANT);
	mOut.WriteInt(set_type);
    mOut.WriteInt(control_mode);
	if (numArgs > 0)
		mOut.WriteFloat(arg, numArgs);
	sendMessage(mOut);
}

void RobotCom::controlGripper( float voltage )
{
    if( voltage < -10.0 || 10.0 < voltage ) return;

    CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType(GRIPPER);
	mOut.WriteInt( (int)(voltage * 4096.0 / 10.0) );
	sendMessage(mOut);
}


void RobotCom::_break() {
    CMsg mOut;
    mOut.WriteMessageType(BREAK);
    sendMessage(mOut);
}

// get_type = GET_CURTIME (gv.curTime),
//            GET_JPOS (gv.q), GET_JVEL (gv.dq), GET_TORQ (gv.tau), 
//            GET_IPOS (gv.x)
void RobotCom::getStatus( UiToServoMessageType get_type, float *arg )
{
  CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType( get_type );
	mOut.WriteInt( GUI );
	sendMessage(mOut);

	uint16_t expectedMesgType = NO_MSSG;
	int numData; // TODO: remove

	switch( get_type )
	{
	case GET_CURTIME:
		expectedMesgType = CURTIME_DATA;
		numData = 1;
		break;
	case GET_JPOS:
		expectedMesgType = JPOS_DATA;
		numData = 6;
		break;
	case GET_JVEL:
		expectedMesgType = JVEL_DATA;
		numData = 6;
		break;
	case GET_TORQ:
		expectedMesgType = TORQ_DATA;
		numData = 6;
		break;
	case GET_IPOS:
		expectedMesgType = IPOS_DATA;
		numData = 7;
		break;
	default:
		std::cout<<"Warning: RobotCom::getStatus: Unsupported messagetype provided!"<<std::endl;
		return;
		break;
	}

	for(;;)
	{
		if( IsDataAvailable() )
		{
			CMappedMsg mIn = GetMsg();
			uint16_t mesgType = mIn.ReadMessageType();

			if( mesgType == expectedMesgType )
			{
				mIn.ReadFloat( arg, numData );
				return;
			}
		}
	}
}



/*********************************************************************
 * sendMessage(): Send a message to the robot.
 */
void RobotCom::sendMessage(AMsg &mOut)
{
    char *ptr;
    int bytesLeft = mOut.GetRawMsg(ptr);
    while (bytesLeft > 0) {
        int sendSize = send(robotSocket, ptr, bytesLeft, 0);
        if (sendSize <= 0) {
            processBrokenSocket();
            return;
        }

        ptr += sendSize;
        bytesLeft -= sendSize;
    }
}

/*********************************************************************
 * processIncomingMessages(): Process any messages received from the
 * robot.  If the robot hasn't sent any, then return immediately.
 */

CMappedMsg RobotCom::GetMsg()
{
  CMappedMsg msg;
  msg.SetMsg( buffer_, size_ );
  return msg;
}

int RobotCom::ready()
{
  size_ = 0;
  if (readSize_ == 1)
  {
    char s[2];
    if (Peek(s,2) < 2)
      return -1;
    size_ = Unpack2B(s);
    // printf("got message of size %d\n", size_);
    memset(buffer_,'\0',bufferSize_);
    InitByteCounter();
    readSize_ = 0;
  }

  if (Peek() < size_ )
    return -1;

  if (Receive() != size_)
  {
    perror("PrDataInput: ready() error");
    return -2;
  }

  //type_ = Unpack2B(buffer_+2);

  readSize_ = 1;
  return size_;// - InitByteCounter());
}

// p279 in UNIX Network Programming by Stevens
int RobotCom::Receive()
{
  char *ptr = buffer_;

  int nleft,nread;
  nleft = size_;
  while(nleft>0)
  {
    nread = recv(robotSocket,ptr,nleft,0);
    if(nread<0) return (nread); /* error */
    else if(nread==0) break; /* EOF */
    nleft -= nread;
    ptr += nread;
  }
  return(size_ - nleft);  // return >= 0
}

int RobotCom::Peek()
{
  int arg=0;
#ifndef WIN32
#ifdef PR_QNX
  if (ioctl((uint16_t)robotSocket,FIONREAD,&arg)<0)
#else // PR_QNX
  if (ioctl(robotSocket,FIONREAD,&arg)<0)
#endif // PR_QNX
#else //#ifndef WIN32
  if ( ioctlsocket( robotSocket, FIONREAD, (u_long FAR*) &arg ) < 0 )
#endif
  {
    perror("RobotCom::Peek():ioctl() error");
    arg = -1;
  }

  //        fprintf( stderr, "peek %d\n", arg );
  return arg;
}

int RobotCom::Peek( char *buff, int nbytes )
{
  int arg = Peek();
  if (arg >= nbytes)
  {
    if (recv(robotSocket,buff,nbytes,MSG_PEEK)!=nbytes)
    {
      perror("RobotCom::Peek():recv() error");
      arg = -1;
    }
  }
  return arg;
}

uint16_t RobotCom::Unpack2B( char *text )
{
  return ntohs(*(uint16_t *)text);
  //return *(short *)text;
}

/*********************************************************************
 * processBrokenSocket(): Called when the socket connection to the
 * robot is broken.
 */
void RobotCom::processBrokenSocket()
{
	printf("error: processBrokenSocket()\n");
    return;
}


int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
