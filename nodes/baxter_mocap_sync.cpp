// baxter_mocap_sync.cpp: Program to synchronize baxter and motion capture system using zeromq
// Requirements: baxter server scripts and MocapInterface.exe servers running
// Author: Nishanth Koganti
// Date: 2015/8/24

// TODO:
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
#define MOCAPIP "tcp://192.168.0.6:5555"
#define BAXTERIP "tcp://localhost:5555"

// namespace declarations
using namespace std;
using namespace zmq;

int main(int argc, char** argv)
{
	// zeromq variable initialization
	char c;
	string Message;

	//  prepare context and socket
	context_t context(1);
	socket_t mocapSocket(context, ZMQ_PAIR);
	socket_t baxterSocket(context, ZMQ_PAIR);

	// connect to the servers
	mocapSocket.connect(MOCAPIP);
	cout << "[ZMQ Mocap] Connected to server" << endl;

	baxterSocket.connect(BAXTERIP);
	cout << "[ZMQ Baxter] Connected to server" << endl;

	// main loop of program
	while (1)
	{
		// start new trial
		cout << "Start new trial? Y/N" << endl;
		cin >> c;

		if (c == 'N')
		{
			// stop both baxter and mocap servers
			message_t stopServerMocap(10);
			memcpy((void *)stopServerMocap.data(), "StopServer", 10);
			mocapSocket.send(stopServerMocap);
			cout << "[ZMQ Mocap] Sent StopServer"  << endl;
			message_t stopServerBaxter(10);
			memcpy((void *)stopServerBaxter.data(), "StopServer", 10);
			baxterSocket.send(stopServerBaxter);
			cout << "[ZMQ Baxter] Sent StopServer"  << endl;
			break;
		}

		else
		{
			// start new trial
			message_t newTrialBaxter(8);
			memcpy((void *)newTrialBaxter.data(), "NewTrial", 8);
			baxterSocket.send(newTrialBaxter);
			cout << "[ZMQ Baxter] Sent NewTrial" << endl;
			message_t newTrialMocap(8);
			memcpy((void *)newTrialMocap.data(), "NewTrial", 8);
			mocapSocket.send(newTrialMocap);
			cout << "[ZMQ Mocap] Sent NewTrial" << endl;

			// wait for replies
			message_t mocapReply, baxterReply;
			mocapSocket.recv(&mocapReply);
			baxterSocket.recv(&baxterReply);

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
			Message = string(static_cast<char *>(baxterReply.data()), baxterReply.size());
			if (Message == "Ready")
				cout << "[ZMQ Baxter] Received Ready" << endl;
			else
			{
				// try again if we get invalid reply
				cout << "[ZMQ Baxter] Invalid reply" << endl;
				continue;
			}

      // send baxter playback fileName
      cout << "Baxter Playback Filename?" << endl;
			cin >> Message;

			message_t fileNamePlayback(Message.length());
			memcpy((void *)fileNamePlayback.data(), Message.c_str(), Message.length());
			baxterSocket.send(fileNamePlayback);
			cout << "[ZMQ Baxter] Sent playback filename" << endl;

			// Send record fileName
			cout << "Record Filename?" << endl;
			cin >> Message;

			message_t fileNameMocap(Message.length());
			memcpy((void *)fileNameMocap.data(), Message.c_str(), Message.length());
			mocapSocket.send(fileNameMocap);
			cout << "[ZMQ Mocap] Sent filename" << endl;
			message_t fileNameBaxter(Message.length());
			memcpy((void *)fileNameBaxter.data(), Message.c_str(), Message.length());
			baxterSocket.send(fileNameBaxter);
			cout << "[ZMQ Baxter] Sent filename" << endl;

			// flag added here to either wait for single person or proceed when two people are recording
			cout << "Start Recording with Delay? Y/N" << endl;
			cin >> Message;

			if (Message == "Y" || Message == "y")
				sleep(6);

			// start recording request
			message_t startRequestMocap(14);
			memcpy((void *)startRequestMocap.data(), "StartRecording", 14);
			mocapSocket.send(startRequestMocap);
			cout << "[ZMQ Mocap] Sent StartRecording" << endl;

			// have a gap of atleast 5 milliseconds
			// usleep(5000);

			// send baxter message
			message_t startRequestBaxter(14);
			memcpy((void *)startRequestBaxter.data(), "StartRecording", 14);
			baxterSocket.send(startRequestBaxter);
			cout << "[ZMQ Baxter] Sent StartRecording" << endl;

			// stop recording request obtained from baxter record server
			message_t baxterSignal;
			baxterSocket.recv(&baxterSignal);
			Message = string(static_cast<char *>(baxterSignal.data()), baxterSignal.size());

			if (Message == "StoppedRecording")
			{
				cout << "[ZMQ Baxter] Received StoppedRecording" << endl;
			}

			else
			{
				cout << "[ZMQ Baxter] Invalid reply" << endl;
				continue;
			}

		  message_t stopRequestMocap(13);
			memcpy((void *)stopRequestMocap.data(), "StopRecording", 13);
			mocapSocket.send(stopRequestMocap);
			cout << "[ZMQ Mocap] Sent StopRecording" << endl;
		}
	}

	// Safe exit
	mocapSocket.close();
	baxterSocket.close();
	return 0;
}
