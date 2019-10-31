#include "RTProtocol.h"
#pragma warning( pop )
#include "RTPacket.h"
#include <string>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>


using namespace std;

// Start the main program
int main(int argc, char* argv[]) {

    ros::init(argc, argv, "Qualisys2Ros");
    tf::TransformBroadcaster publisher;
    ros::NodeHandle n;

    string serverAddress; // The address of the computer connected to the Qualisys motion tracking system 
    n.getParam("server_address", serverAddress);

    // Defining global variables
    float x, y, z, roll, pitch, yaw;
    uint bodyCount;
    uint frameNumber;
    const unsigned short basePort = 22222; // The base port (as entered in QTM, TCP/IP port number, in the RT output tab of the workspace options
    const int            majorVersion = 1;
    const int            minorVersion = 20;
    const bool           bigEndian = false;
    bool dataAvailable = false;
    bool streamFrames = false;
    unsigned short udpPort = 6734;

    // Defining a protocol that connects to the Qualisys
    CRTProtocol rtProtocol;

    // Connecting to the server
    ROS_INFO_STREAM("Connecting to the Qualisys Motion Tracking system specified at: " << serverAddress << ":" << basePort);

    // Infinite Measurement Loop
    while (ros::ok()) {        

        if (!rtProtocol.Connected())
        {
            if (!rtProtocol.Connect(serverAddress.c_str(), basePort, &udpPort, majorVersion, minorVersion, bigEndian))
            {
                ROS_INFO_STREAM("rtProtocol.Connect: %s\n\n" << rtProtocol.GetErrorString());
                sleep(1);
                continue;
            }
        }

        if (!dataAvailable)
        {
            if (!rtProtocol.Read6DOFSettings(dataAvailable))
            {
                ROS_INFO_STREAM("rtProtocol.Read6DOFSettings: " << rtProtocol.GetErrorString());
                sleep(1);
                continue;
            }
        }

        if (!streamFrames)
        {
            if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0,0, NULL, CRTProtocol::cComponent6d))
            {
                ROS_INFO_STREAM("rtProtocol.StreamFrames: " << rtProtocol.GetErrorString());
                sleep(1);
                continue;
            }
        }

        CRTPacket::EPacketType packetType;
        CRTPacket* rtPacket = rtProtocol.GetRTPacket();
        bodyCount  = rtPacket->Get6DOFBodyCount();
  
        ROS_INFO_STREAM("Number of bodies found :" << bodyCount);

        if (rtProtocol.ReceiveRTPacket(packetType, true)) {
            switch (packetType) {
                // Case 1 - sHeader.nType 0 indicates an error
                case CRTPacket::PacketError :
                    ROS_ERROR_STREAM_THROTTLE(1,"Error when streaming frames: " << rtProtocol.GetRTPacket()->GetErrorString());
                    break;
                case CRTPacket::PacketNoMoreData :  // No more data
                    ROS_WARN_STREAM_THROTTLE(1,"No more data");
                    break;

                // Case 2 - Data received
                case CRTPacket::PacketData :

                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    //bodyCount  = rtPacket->Get6DOFBodyCount();

                    if (bodyCount <= 0) {
                        ROS_WARN_THROTTLE(1,"No Bodies Found");
                    } else {
                        for (unsigned int i = 0; i < bodyCount; i++) 
                        {
                            if(rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                            {
                                const char* pTmpStr = rtProtocol.Get6DOFBodyName(i);
                                ROS_INFO_STREAM(pTmpStr);

                                if(pTmpStr != "Table" && pTmpStr != "Talos_Torso")
                                {
                                    tf::Matrix3x3 rotMat = tf::Matrix3x3(rotationMatrix[0],rotationMatrix[1],rotationMatrix[2]
                                        ,rotationMatrix[3],rotationMatrix[4],rotationMatrix[5],
                                        rotationMatrix[6],rotationMatrix[7],rotationMatrix[8]);
                                    tf::Quaternion q;
                                    rotMat.getRotation(q);
                                    q[3] = -q[3];
                                    tf::Transform transform;
                                    transform.setOrigin(tf::Vector3(fX, fY, fZ)/1000.);
                                    transform.setRotation(q);
                                    publisher.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Qualisys", pTmpStr));
                                }
                            }
                        }
                    }        

                    break;

                default:
                    ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case");

            }
        }
    }

    ROS_INFO("Shutting down");
    rtProtocol.StreamFramesStop();
    rtProtocol.Disconnect();
    return 1;
}