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


#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <ctype.h>

#define SERV_IP "127.0.0.1"
#define SERV_PORT 9855

int main(void)
{
	int lfd, cfd;
	struct sockaddr_in serv_addr, clie_addr;
	socklen_t clie_addr_len;
	char clie_IP[BUFSIZ];
	unsigned char buf[BUFSIZ];
	int n, i, ret;

	lfd = socket(AF_INET, SOCK_STREAM, 0);
	if(lfd == -1){
		perror("socket error");
		exit(1);
	}

	bzero(&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(SERV_PORT);
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	ret = bind(lfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
	if(ret == -1){
		perror("bind error");
		exit(1);
	}

	ret = listen(lfd, 128);
	if(ret == -1){
		perror("listen error");
		exit(1);
	}

	clie_addr_len = sizeof(clie_addr);
	cfd = accept(lfd, (struct sockaddr *)&clie_addr, &clie_addr_len);
	if(cfd == -1){
		perror("accept error");
		exit(1);
	}

	printf("cliet IP:%s, client port:%d\n",
		    inet_ntop(AF_INET, &clie_addr.sin_addr.s_addr, clie_IP, sizeof(clie_IP)),
		  	ntohs(clie_addr.sin_port));
	while(1)
	{
		n = read(cfd, buf, 1);
		// printf("%u", buf[0]);
		// for(i = 0; i < n; i++)
		// 	buf[i] = toupper(buf[i]);
		printf("%x ", buf[0]);
		write(cfd, buf, 1);
		
	}

	close(lfd);
	close(cfd);

//	printf("1223");
	return 0;
}

// void server()
// {
// 	try
// 	{
// 		printf("[Server] Started\n");

// 		CServerTCPSocket		server( 15000, "127.0.0.1" , 10, mrpt::utils::LVL_DEBUG);
// 		CClientTCPSocket		*client;

//         while(true){
//             client = server.accept( 2000 );

//             cout << client << endl << endl;

//             if (client)
//             {
//                 printf("[Server] Connection accepted\n");

//                 // Load test image:
//                 CImage	img;
//                 img.loadFromFile(myDataDir+string("frame_color.jpg"));

//                 // Send a message with the image:
//                 CMessage	msg;
//                 msg.type	= 0x10;
//                 msg.serializeObject( &img );

//                 printf("[Server] Sending message...\n");
//                 client->sendMessage( msg );
//                 printf("[Server] Message sent!!\n");

//                 mrpt::system::sleep(50);

//                 delete client;
//             }
//         }

		

// 		printf("[Server] Finish\n");
// 	}
// 	catch(std::exception &e)
// 	{
// 		cerr << e.what() << endl;
// 	}
// 	catch(...)
// 	{
// 		printf("[thread_server] Runtime error!\n");
// 	}
// }



// int main(int argc, char** argv)
// {
// 	cout << myDataDir << endl;
//     server();
// }
