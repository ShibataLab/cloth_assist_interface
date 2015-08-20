// kinect_synchronizer.cpp: Program to synchronize kinect and motion capture system using zeromq
// Requirements: marker_recorder and MocapInterface.exe servers running with marker tracking running
// Author: Nishanth Koganti
// Date: 2015/8/20

// TODO:
// 0) Add two modes in the synchronizer depending on stopping method (DONE)
// 1) Improve method to initialize IP addresses
// 2) Maybe QT Interface for all the questions asked?

// headers
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <iostream>

// zeromq header
#include <zmq.hpp>

// preprocessor directives
#define MOCAPIP "tcp://192.168.0.3:5555"
#define KINECTIP "tcp://localhost:5555"

// namespace declarations
using namespace std;
using namespace zmq;

int main(int argc, char** argv)
{
	// syncMode variable initialization
	int syncMode = 1;
	if (argc > 1)
		syncMode = atoi(argv[1]);

	// zeromq variable initialization
	char c;
	string Message;

	//  prepare context and socket
	context_t context(1);
	socket_t mocapSocket(context, ZMQ_PAIR);
	socket_t kinectSocket(context, ZMQ_PAIR);

	// connect to the servers
	mocapSocket.connect(MOCAPIP);
	cout << "[ZMQ Mocap] Connected to server" << endl;

	kinectSocket.connect(KINECTIP);
	cout << "[ZMQ Kinect] Connected to server" << endl;

	// main loop of program
	while (1)
	{
		// start new trial
		cout << "Start new trial? Y/N" << endl;
		cin >> c;

		if (c == 'N')
		{
			// stop both kinect and mocap servers
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
			// start new trial
			message_t newTrialKinect(8);
			memcpy((void *)newTrialKinect.data(), "NewTrial", 8);
			kinectSocket.send(newTrialKinect);
			cout << "[ZMQ Kinect] Sent NewTrial" << endl;
			message_t newTrialMocap(8);
			memcpy((void *)newTrialMocap.data(), "NewTrial", 8);
			mocapSocket.send(newTrialMocap);
			cout << "[ZMQ Mocap] Sent NewTrial" << endl;

			// wait for replies
			message_t mocapReply, kinectReply;
			mocapSocket.recv(&mocapReply);
			kinectSocket.recv(&kinectReply);

			// check for ready message
			Message = string(static_cast<char *>(mocapReply.data()), mocapReply.size());
			if (Message == "Ready")
				cout << "[ZMQ Mocap] Received Ready" << endl;
			else
			{
				// try again if we get invalid reply
				cout << "[ZMQ Mocap] Invalid reply" << endl;
				continue;
			}

			// check for ready message
			Message = string(static_cast<char *>(kinectReply.data()), kinectReply.size());
			if (Message == "Ready")
				cout << "[ZMQ Kinect] Received Ready" << endl;
			else
			{
				// try again if we get invalid reply
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

			// flag added here to either wait for single person or proceed when two people are recording
			cout << "Start Recording with Delay? Y/N" << endl;
			cin >> Message;

			if (Message == "Y")
				sleep(6);

			// start recording request
			message_t startRequestMocap(14);
			memcpy((void *)startRequestMocap.data(), "StartRecording", 14);
			mocapSocket.send(startRequestMocap);
			cout << "[ZMQ Mocap] Sent StartRecording" << endl;

			//have a gap of atleast 5 milliseconds
			usleep(5000);

			// send kinect message
			message_t startRequestKinect(14);
			memcpy((void *)startRequestKinect.data(), "StartRecording", 14);
			kinectSocket.send(startRequestKinect);
			cout << "[ZMQ Kinect] Sent StartRecording" << endl;

			if (syncMode == 0)
			{
				// stop recording request obtained from kinect
				message_t kinectSignal;
				kinectSocket.recv(&kinectSignal);

				Message = string(static_cast<char *>(kinectSignal.data()), kinectSignal.size());
				if (Message == "StoppedRecording")
				{
					cout << "[ZMQ Kinect] Received StoppedRecording" << endl;
				}

				else
				{
					cout << "[ZMQ Kinect] Invalid reply" << endl;
					continue;
				}
			}
			else
			{
				// wait for message from user
				cout << "Stop recording? Y" << endl;
				cin >> Message;

				// send stop recording request to kinect as well
				message_t stopRequestKinect(13);
				memcpy((void *)stopRequestKinect.data(), "StopRecording", 13);
				kinectSocket.send(stopRequestKinect);
				cout << "[ZMQ Kinect] Sent StopRecording" << endl;
			}

		  message_t stopRequestMocap(13);
			memcpy((void *)stopRequestMocap.data(), "StopRecording", 13);
			mocapSocket.send(stopRequestMocap);
			cout << "[ZMQ Mocap] Sent StopRecording" << endl;
		}
	}

	// Safe exit
	mocapSocket.close();
	kinectSocket.close();
	return 0;
}
