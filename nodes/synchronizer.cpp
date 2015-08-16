// Headers
#include <string>
#include <unistd.h>
#include <iostream>

// ZeroMQ Header
#include <zmq.hpp>

using namespace std;
using namespace zmq;

int main(int argc, char** argv)
{

	// ZeroMQ Initialization
	char c;
	string Message;

	//  Prepare our context and socket
	context_t context(1);
	socket_t mocapSocket(context, ZMQ_PAIR);
	socket_t kinectSocket(context, ZMQ_PAIR);

	// Connecting to the sockets
	mocapSocket.connect("tcp://192.168.0.3:5555");
	cout << "[ZMQ Mocap] Connected to server" << endl;

	kinectSocket.connect("tcp://localhost:5555");
	cout << "[ZMQ Kinect] Connected to server" << endl;

	while (1)
	{
		cout << "Start new trial? Y/N" << endl;
		cin >> c;

		if (c == 'N')
		{
			// Stop Server
			message_t stopServerMocap(10);
			memcpy((void *)stopServerMocap.data(), "StopServer", 10);
			mocapSocket.send(stopServerMocap);
			cout << "[ZMQ Mocap] Sent StopServer"  << endl;
			message_t stopServerKinect(10);
			memcpy((void *)stopServerKinect.data(), "StopServer", 10);
			kinectSocket.send(stopServerKinect);
			cout << "[ZMQ Kinect] Sent StopServer"  << endl;
			break;
		}

		else
		{
			// Start New Trial
			message_t newTrialKinect(8);
			memcpy((void *)newTrialKinect.data(), "NewTrial", 8);
			kinectSocket.send(newTrialKinect);
			cout << "[ZMQ Kinect] Sent NewTrial" << endl;
			message_t newTrialMocap(8);
			memcpy((void *)newTrialMocap.data(), "NewTrial", 8);
			mocapSocket.send(newTrialMocap);
			cout << "[ZMQ Mocap] Sent NewTrial" << endl;

			// wait for reply
			message_t mocapReply, kinectReply;
			mocapSocket.recv(&mocapReply);
			kinectSocket.recv(&kinectReply);

			Message = string(static_cast<char *>(mocapReply.data()), mocapReply.size());
			if (Message == "Ready")
				cout << "[ZMQ Mocap] Received Ready" << endl;
			else
			{
				cout << "[ZMQ Mocap] Invalid reply" << endl;
				continue;
			}

			Message = string(static_cast<char *>(kinectReply.data()), kinectReply.size());
			if (Message == "Ready")
				cout << "[ZMQ Kinect] Received Ready" << endl;
			else
			{
				cout << "[ZMQ Kinect] Invalid reply" << endl;
				continue;
			}

			// Send record fileName
			cout << "Record Filename?" << endl;
			cin >> Message;

			message_t fileNameMocap(Message.length());
			memcpy((void *)fileNameMocap.data(), Message.c_str(), Message.length());
			mocapSocket.send(fileNameMocap);
			cout << "[ZMQ Mocap] Sent filename" << endl;
			message_t fileNameKinect(Message.length());
			memcpy((void *)fileNameKinect.data(), Message.c_str(), Message.length());
			kinectSocket.send(fileNameKinect);
			cout << "[ZMQ Kinect] Sent filename" << endl;

			cout << "Start Recording with Delay? Y/N" << endl;
			cin >> Message;

			if (Message == "Y")
				sleep(5);

			// Start Recording Request
			message_t startRequestMocap(14);
			memcpy((void *)startRequestMocap.data(), "StartRecording", 14);
			mocapSocket.send(startRequestMocap);
			cout << "[ZMQ Mocap] Sent StartRecording" << endl;
			message_t startRequestKinect(14);
			memcpy((void *)startRequestKinect.data(), "StartRecording", 14);
			kinectSocket.send(startRequestKinect);
			cout << "[ZMQ Kinect] Sent StartRecording" << endl;

			// Stop Recording Request
      cout << "Stop Recording? Y" << endl;
      cin >> c;

		  message_t stopRequestMocap(13);
			memcpy((void *)stopRequestMocap.data(), "StopRecording", 13);
			mocapSocket.send(stopRequestMocap);
			cout << "[ZMQ Mocap] Sent StopRecording" << endl;
			message_t stopRequestKinect(13);
			memcpy((void *)stopRequestKinect.data(), "StopRecording", 13);
			kinectSocket.send(stopRequestKinect);
			cout << "[ZMQ Kinect] Sent StopRecording" << endl;
		}
	}

	// Safe exit
	mocapSocket.close();
	kinectSocket.close();
	return 0;
}
