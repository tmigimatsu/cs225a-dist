#ifndef UNITMSG_H
#define UNITMSG_H

#include <string.h>
#include <assert.h>
#ifdef WIN32
#include <WinSock.h>
#else
#include <netinet/in.h>
#endif
#include <stdio.h>
#include <stdint.h>

namespace Puma {

static const int MAX_MSG_SIZE = 1024;

//#define PR_DOUBLE_PRECISION // double precision
#ifdef PR_DOUBLE_PRECISION
typedef double Float;
#else // PR_DOUBLE_PRECISION
typedef float Float;
#endif // PR_DOUBLE_PRECISION

//-----------------------------------------------------------------------------
class AMsg
  //-----------------------------------------------------------------------------
  // Abstract message class
{
protected:
  int m_bufferSize;
  int m_byteCounter;
  int m_fEndian;
  uint16_t m_mesgType;
  char* m_szBuf;
  bool m_fBuffer;

  int InitByteCounter();

  int Pack( const char *data, int num, int size );
  static inline void Pack1B( char *text, char data );
  static inline void Pack2B( char *text, uint16_t data );
  static inline void Pack4B( char *text, uint32_t data );
  void Pack8B( char *text, uint64_t data ) const;

  int Unpack( char *data, int num, int size );
  static inline char  Unpack1B( const char *text );
  static inline uint16_t Unpack2B( const char *text );
  static inline uint32_t Unpack4B( const char *text );
  uint64_t Unpack8B( const char *text ) const;

public:
  AMsg();

  void Init();

  int GetRawMsg( char*& szMsg )
  { 
    Pack2B(m_szBuf,(uint16_t)m_byteCounter); // fill in the size of the message
    szMsg = m_szBuf; 
    return m_byteCounter; 
  }

  int GetContent( char*& szMsg ) const
  {
    if( m_byteCounter >= 4 )
      szMsg = m_szBuf+4;
    else
      szMsg = NULL;
    return m_bufferSize-4;
  }

  int GetBufSize() const { return m_bufferSize; }

  virtual void SetMsg( const char* szMsg, int size ) = 0;

  // Write routines:
  int WriteChar ( const char    *data, int num ) { return Pack(data,num,1); }
  int WriteShort( const int16_t *data, int num ) { return Pack((const char *)data,num,2); }
  int WriteInt  ( const int32_t *data, int num ) { return Pack((const char *)data,num,4); }
  int WriteLong ( const int32_t *data, int num ) { return Pack((const char *)data,num,4); }
  int WriteFloat( const Float   *data, int num ) { return Pack((const char *)data,num,sizeof(Float)); }

  int  WriteBool ( bool value  ) { char val = (value ? '\01' : '\0'); return Pack((const char *)(&val),1,1); }
  int32_t  WriteInt  ( int32_t value   ) { return Pack((const char *)(&value),1,4); }
  int  WriteFloat( Float value ) { return Pack((const char *)(&value),1,sizeof(Float)); }
  int  WriteString( const char * str ) { return Pack(str, (int)strlen(str)+1,sizeof(char)); }
  void WriteMessageType( int mesgType ) 
  { 
    //printf("mt=%d\n", mesgType);
    assert( m_fBuffer );
    m_mesgType = (uint16_t) mesgType;
    Pack2B(m_szBuf+2,(uint16_t)m_mesgType);
  }


  int ReadChar ( char    *data, int num ) { return Unpack(data,num,1); }
  int ReadShort( int16_t *data, int num ) { return Unpack((char *)data,num,2); }
  int ReadInt  ( int32_t *data, int num ) { return Unpack((char *)data,num,4); }
  int ReadLong ( int32_t *data, int num ) { return Unpack((char *)data,num,4); }
  int ReadFloat( Float   *data, int num ) { return Unpack((char *)data,num,sizeof(Float)); }


  // Read routines:
  bool  ReadBool () { char val='\0'; Unpack(&val,1,1); return val != '\0'; }
  int32_t ReadInt () { int32_t val=0; Unpack((char *)(&val),1,4); return val; }
  Float ReadFloat() { Float val=0; Unpack((char *)(&val),1,sizeof(Float)); return val; }
  void  ReadString( char * str, int bufLen ) { Unpack(str,bufLen,sizeof(char)); }
  int   ReadMessageType() const 
  {
    return m_mesgType;
  }
};





//-----------------------------------------------------------------------------
class CMsg : public AMsg
  //-----------------------------------------------------------------------------
  // message with a local buffer.  message is to be used for unit of information
  // trasfered between server and ui and vice versa.
{
public:
  CMsg() { m_szBuf = &m_locBuf[0]; }

  void SetMsg( const char* szMsg, int size )
  {
    strncpy( m_szBuf, szMsg, size );
    m_mesgType = Unpack2B(m_szBuf+2);
    m_bufferSize = size;
    // printf("mm of type %d, s=%d; ", m_mesgType, m_bufferSize );
    InitByteCounter();
  }

private:
  char m_locBuf[MAX_MSG_SIZE];
};


//-----------------------------------------------------------------------------
class CMappedMsg : public AMsg
  //-----------------------------------------------------------------------------
  // message whose buffer is mapped to some other memory
{
public:
  CMappedMsg() { m_fBuffer = false; }

  void SetMsg( const char* szMsg, int size )
  {
    m_szBuf = const_cast<char*> (szMsg);
    m_mesgType = Unpack2B(m_szBuf+2);
    m_bufferSize = size;
    // printf("mm of type %d, s=%d; ", m_mesgType, m_bufferSize );
    InitByteCounter();
  }

private:
};

//-----------------------------------------------------------------------------
// Inline funcitons
//-----------------------------------------------------------------------------

// Note: since we are commucating between x86 machines, we do not need 
// to conver to network format.  If you are computing between machines of
// different architectures, comment out the conversion functions.
inline void AMsg::Pack1B( char *text, char data )
{
  text[0] = data;
}

inline void AMsg::Pack2B( char *text, uint16_t data )
{
  *(uint16_t *)text = htons(data);
  //*(short *)text = data;
}

inline void AMsg::Pack4B( char *text, uint32_t data )
{
  *(uint32_t *)text = htonl(data);
  //*(long *)text = data;
}


inline char AMsg::Unpack1B( const char *text )
{
  return text[0];
}

inline uint16_t AMsg::Unpack2B( const char *text )
{
  return ntohs(*(const uint16_t *)text);
  //return *(const short *)text;
}

inline uint32_t AMsg::Unpack4B( const char *text )
{
  return ntohl(*(const uint32_t *)text);
  //return *(const long *)text;
}

}

#endif  // UNITMSG_H
