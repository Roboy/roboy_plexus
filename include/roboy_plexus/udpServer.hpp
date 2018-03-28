//
// Created by roboy on 28.03.18.
//

#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h> /* close() */
#include <string.h> /* memset() */
#include <ros/ros.h>

#define LOCAL_SERVER_PORT 1500
#define MAX_MSG 100


class UdpServer {
    public:
        static void listen();
        int x = 5;
};

