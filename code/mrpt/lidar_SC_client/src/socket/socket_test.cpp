/************************************************************************
    > File Name: socket_test.cpp
    > Author: tangbf
    > Mail: 
    > Created Time: 2018年06月21日 星期四 15时35分12秒
 ************************************************************************/

#include <iostream>
#include <mrpt/utils.h>
#include <mrpt/system.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

#include <mrpt/examples_config.h>
string myDataDir(MRPT_EXAMPLES_BASE_DIRECTORY + string("imageBasics/"));

void client()
{
	try
	{
		printf("[Client] Started\n");
		CClientTCPSocket	sock;

		printf("[Client] Connecting\n");

		sock.connect( "127.0.0.1", 15000 );

		printf("[Client] Connected. Waiting for a message...\n");

//		cout << "pending: " << sock.getReadPendingBytes() << endl;
//		mrpt::system::sleep(4000);
//		cout << "pending: " << sock.getReadPendingBytes() << endl;

		while(true){
			CMessage	msg;
			bool ok = sock.receiveMessage( msg, 2000,2000);

			if (!ok)
			{
				printf("[Client] Error receiving message!!\n");
			}
			else
			{
				printf("[Client] Message received OK!:\n");
				printf("  MSG Type: %i\n", msg.type );
				printf("  MSG Length: %u bytes\n", (unsigned int)msg.content.size() );

				printf("[Client] Parsing image...\n");
				CImage		img;
				msg.deserializeIntoExistingObject( &img );

				printf("[Client] Saving image...\n");
				img.saveToFile("received_frame.jpg");
				printf("[Client] Done!!\n");
			}
		}

		

		printf("[Client] Finish\n");
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}
	catch(...)
	{
		cerr << "[thread_client] Runtime error!" << endl;;
	}
}



int main(int argc, char** argv)
{
	cout << myDataDir << endl;
    client();
}
