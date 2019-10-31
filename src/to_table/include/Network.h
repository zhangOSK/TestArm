#ifndef NETWORK_H
#define NETWORK_H

#ifdef _WIN32
	#define WIN32_LEAN_AND_MEAN
	#include <winsock2.h>
#else
	#define INVALID_SOCKET	-1
	#define SOCKET int
#endif

class CNetwork
{
public:
    CNetwork();
    ~CNetwork();
    bool  Connect(const char* pServerAddr, unsigned short nPort);
    void  Disconnect();
    bool  Connected() const;
    bool  CreateUDPSocket(unsigned short &nUDPPort, bool bBroadcast = false);
    int   Receive(char* rtDataBuff, int nDataBufSize, bool bHeader, int nTimeout, unsigned int *ipAddr = nullptr);
    bool  Send(const char* pSendBuf, int nSize);
    bool  SendUDPBroadcast(const char* pSendBuf, int nSize, short nPort, unsigned int nFilterAddr = 0);
    char* GetErrorString();
    int   GetError() const;
    bool  IsLocalAddress(unsigned int nAddr) const;
    unsigned short GetUdpServerPort();
    unsigned short GetUdpBroadcastServerPort();

private:
    bool InitWinsock();
    void SetErrorString();
    unsigned short GetUdpServerPort(SOCKET nSocket);

private:
    SOCKET     mSocket;
    SOCKET     mUDPSocket;
    SOCKET     mUDPBroadcastSocket;
    char       mErrorStr[256];
    unsigned long mLastError;
};


#endif

// #ifndef NETWORK_H
// #define NETWORK_H

// typedef unsigned int DWORD;

// class COutput;

// class CNetwork
// {
// public:
//     CNetwork();
//     bool  Connect(const char* pServerAddr, unsigned short nPort);//char* pServerAddr, int nPort);
//     void  Disconnect();
//     bool  Connected();
//     bool  CreateUDPSocket(int nUDPPort, bool bBroadcast = false);
//     int   Receive(char* rtDataBuff, int nDataBufSize, bool bHeader, int nTimeout, unsigned int *ipAddr = NULL);
//     bool  Send(const char* pSendBuf, int nSize);
//     bool  SendUDPBroadcast(const char* pSendBuf, int nSize, short nPort, unsigned int nFilterAddr = 0);
//     char* GetErrorString();
//     int   GetError();
//     bool  IsLocalAddress(unsigned int nAddr);

// private:
//     bool InitWinsock();
//     void SetErrorString();

// private:
//     COutput*   mpoOutput;
//     //modified Socket to int (socket is windows)
//     int     mhSocket;
//     int     mhUDPSocket;
//     int      mhUDPBroadcastSocket;
//     //---------------------------------------
//     char       maErrorStr[256];
//     DWORD      mnLastError;
// };


// #endif
