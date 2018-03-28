//
// Created by roboy on 28.03.18.
//

#include "roboy_plexus/udpServer.hpp"

 void UdpServer::listen() {
        printf("TADA \n");
        int sd, rc, n;
        socklen_t cliLen;
        struct sockaddr_in cliAddr, servAddr;
        char msg[MAX_MSG];
        int broadcast = 1;

        /* socket creation */
        sd=socket(AF_INET, SOCK_DGRAM, 0);
        if(sd<0) {
            printf(" cannot open socket \n");
            return;
        }

        if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcast,sizeof broadcast) == -1) {
            perror("setsockopt (SO_BROADCAST)");
            return;
        }

        /* bind local server port */
        servAddr.sin_family = AF_INET;
        servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        servAddr.sin_port = htons(LOCAL_SERVER_PORT);
        rc = bind (sd, (struct sockaddr *) &servAddr,sizeof(servAddr));
        if(rc<0) {
            printf(" cannot bind port number %d \n",
                   LOCAL_SERVER_PORT);
            return;
        }

        printf("waiting for data on port UDP %u\n",LOCAL_SERVER_PORT);

        /* server infinite loop */
//    while(1) {

        /* init buffer */
        memset(msg,0x0,MAX_MSG);


        /* receive message */
        cliLen = sizeof(cliAddr);
        n = recvfrom(sd, msg, MAX_MSG, 0,
                     (struct sockaddr *) &cliAddr, &cliLen);

        if(n<0) {
            printf("cannot receive data \n");
        }

        /* print received message */
        printf("from %s:UDP%u : %s \n",
               inet_ntoa(cliAddr.sin_addr),
               ntohs(cliAddr.sin_port),msg);

//    }/* end of server infinite loop */

    }


