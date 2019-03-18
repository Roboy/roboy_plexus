#include <ifaddrs.h>
#include "common_utilities/UDPSocket.hpp"

uint64_t pack754(long double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) return 0; // get this special case out of the way

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

long double unpack754(uint64_t i, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1<<(expbits-1)) - 1;
    shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i>>(bits-1))&1? -1.0: 1.0;

    return result;
}

UDPSocket::UDPSocket(const char *server_IP, int server_port, const char *client_IP, int client_port, bool exclusive):
        exclusive(exclusive){
    cout << "creating socket on " << server_IP << ":" << server_port << " with client " << client_IP << ":"
                                          << client_port << endl;
    int rv;

    struct addrinfo hints, *p;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    char pstr[10];
    snprintf(pstr, 10, "%d", server_port);
    if ((rv = getaddrinfo(server_IP, pstr, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return;
    }

    int yes = 1;
    // loop through all the results, make a socket and bind
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            fprintf(stderr, "talker: socket\n");
            continue;
        }
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &yes, sizeof(int)) == -1) {
            fprintf(stderr, "setsockopt\n");
            continue;
        };
        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            fprintf(stderr, "listener: bind\n");
            continue;
        }
        break;
    }

    if (p == NULL) {
        fprintf(stderr, "talker: failed to bind socket\n");
        return;
    }

    // set 100ms timeout
    setTimeOut(100000);

    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr(client_IP);
    client_addr.sin_port = htons(client_port);
    client_addr_len = sizeof(client_addr);

    initialized = true;
}

UDPSocket::UDPSocket(const char *client_IP, int client_port, bool exclusive):exclusive(exclusive) {
    string myIP, myBroadcastIP;
    if (whatsMyIP(myIP,myBroadcastIP)) {
        cout <<"creating socket on " << myIP << ":" << BROADCAST_PORT << " with client " << client_IP << ":" << client_port << endl;
        int rv;
        struct addrinfo hints, *p;

        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        char pstr[10];
        snprintf(pstr, 10, "%d", BROADCAST_PORT);
        if ((rv = getaddrinfo(myIP.c_str(), pstr, &hints, &servinfo)) != 0) {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
            return;
        }

        int yes = 1;
        // loop through all the results, make a socket and bind
        for (p = servinfo; p != NULL; p = p->ai_next) {
            if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
                fprintf(stderr, "talker: socket\n");
                continue;
            }
            if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &yes, sizeof(int)) == -1) {
                fprintf(stderr, "setsockopt\n");
                continue;
            };
            if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
                close(sockfd);
                fprintf(stderr, "listener: bind\n");
                continue;
            }
            break;
        }

        if (p == NULL) {
            fprintf(stderr, "talker: failed to bind socket\n");
            return;
        }

        // set 100ms timeout
        setTimeOut(100000);

        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = inet_addr(client_IP);
        client_addr.sin_port = htons(client_port);
        client_addr_len = sizeof(client_addr);

        initialized = true;
    }else{
        fprintf(stderr, "could not create UDP socket\n");
    }
}

UDPSocket::UDPSocket(const char *client_IP, int client_port, int server_port, bool exclusive):exclusive(exclusive) {
    string myIP, myBroadcastIP;
    if (whatsMyIP(myIP, myBroadcastIP)) {
        cout << "creating socket on " << myIP << ":" << server_port << " with client " << client_IP << ":" << client_port << endl;
        int rv;
        struct addrinfo hints, *p;

        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        char pstr[10];
        snprintf(pstr, 10, "%d", server_port);
        if ((rv = getaddrinfo(myIP.c_str(), pstr, &hints, &servinfo)) != 0) {
            fprintf(stderr, "getaddrinfo: %s", gai_strerror(rv));
            return;
        }

        int yes = 1;
        // loop through all the results, make a socket and bind
        for (p = servinfo; p != NULL; p = p->ai_next) {
            if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
                fprintf(stderr, "talker: socket");
                continue;
            }
            if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &yes, sizeof(int)) == -1) {
                fprintf(stderr, "setsockopt");
                continue;
            };
            if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
                close(sockfd);
                fprintf(stderr, "listener: bind");
                continue;
            }
            break;
        }

        if (p == NULL) {
            fprintf(stderr, "talker: failed to bind socket");
            return;
        }

        // set 100ms timeout
        setTimeOut(100000);

        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = inet_addr(client_IP);
        client_addr.sin_port = htons(client_port);
        client_addr_len = sizeof(client_addr);

        initialized = true;
    }else{
        fprintf(stderr, "could not create UDP socket");
    }
}

UDPSocket::UDPSocket(int port, int broadcastIP, bool broadcaster) {
    string myIP, myBroadcastIP;
    whatsMyIP(myIP, myBroadcastIP); // myBroadcastIP will not be used here

    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(port);
    char br_IP[16];
    convertByte2Text(broadcastIP,br_IP);
    broadcast_addr.sin_addr.s_addr = inet_addr(br_IP);
    broadcast_addr_len = sizeof(broadcast_addr);

    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(port);
    client_addr.sin_addr.s_addr = INADDR_ANY;
    client_addr_len = sizeof(client_addr);

    // creat UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        fprintf(stderr, "talker: socket");
        return;
    }

    // Allow broadcasts
    int yes = true;
    if  (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (const void *)&yes, sizeof(int)) == -1) {
        fprintf(stderr, "broadcasting not allowed");
        return;
    }

    // set 100ms timeout
    setTimeOut(100000);

    if(!broadcaster) {
        // Bind an address to our socket, so that client programs can listen to this server
        if (bind(sockfd, (struct sockaddr *) &client_addr, client_addr_len) == -1) {
            close(sockfd);
            fprintf(stderr, "broadcaster bind error");
            return;
        }
    }

    initialized = true;
}

UDPSocket::UDPSocket(int port, bool broadcaster) {
    string myIP, myBroadcastIP;
    whatsMyIP(myIP, myBroadcastIP);

    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(port);
    broadcast_addr.sin_addr.s_addr = inet_addr(myBroadcastIP.c_str());
    broadcast_addr_len = sizeof(broadcast_addr);

    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(port);
    client_addr.sin_addr.s_addr = INADDR_ANY;
    client_addr_len = sizeof(client_addr);

    // creat UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        fprintf(stderr, "talker: socket");
        return;
    }

    // Allow broadcasts
    int yes = true;
    if  (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (const void *)&yes, sizeof(int)) == -1) {
        fprintf(stderr, "broadcasting not allowed");
        return;
    }

    // set 100ms timeout
    setTimeOut(100000);

    if(!broadcaster) {
        // Bind an address to our socket, so that client programs can listen to this serve
        if (bind(sockfd, (struct sockaddr *) &client_addr, client_addr_len) == -1) {
            close(sockfd);
            fprintf(stderr, "broadcaster bind error");
            return;
        }
    }

    initialized = true;
}

UDPSocket::~UDPSocket() {
    close(sockfd);
}

uint32_t UDPSocket::receiveHostIP(const char *key, uint32_t &IP){
    if(!receiveUDPFromClient())
        return 0;
    char output[strlen(key)];
    bool access_granted = true;
    uint8_t user[4] = {0xBF, 0x42, 0x76, 0xE9};
    int j = 0;
    for (int i=0; i<strlen(key); i++)
    {
        output[i] = buf[i+4] ^ user[j];
        if(output[i]!=key[i])
            access_granted = false;
        j++;
        if(j == sizeof(user))
            j = 0;
    }
//    printf("%s\n", output);
    if(access_granted){
        IP = (uint32_t)((uint8_t)buf[3] << 24 | (uint8_t)buf[2] << 16 | (uint8_t)buf[1] << 8 | (uint8_t)buf[0]);
        return 3;
    }else if(numbytes < strlen(key)+1){
        printf( "received more bytes than expected %ld\n", numbytes);
    }else if(numbytes > strlen(key)+1) {
        printf("received less bytes than expected  %ld\n", numbytes);
    }
    return 0;
}

bool UDPSocket::broadcastHostIP(uint32_t IP){
    numbytes = 4;
    memcpy(buf,&IP,sizeof(IP));
    return broadcastUDP();
}

bool UDPSocket::broadcastHostIP(char *key, int length){
    numbytes = length+4;
    memcpy(buf,&myIP.first,sizeof(myIP.first));
    memcpy(&buf[4],key,length);
    return broadcastUDP();
}

bool UDPSocket::receiveSensorData(vector<uint32_t> &sensorID, vector<bool> &lighthouse, vector<bool> &axis, vector<uint32_t> &sweepDuration){
    if(receiveUDP()){
        if(numbytes == 32){ // without timestamp
            union {
                uint32_t sensor[8];
                uint8_t data[32];
            }spi_frame;
            memcpy(spi_frame.data, buf, 32);
//            for(uint i = 0; i<32;i++){
//                printf("%d ",buf[i]);
//            }
//            printf("\n");
//            printf(BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " "
//                           BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN "\n",
//                   BYTE_TO_BINARY(spi_frame.data[3]), BYTE_TO_BINARY(spi_frame.data[2]),
//                   BYTE_TO_BINARY(spi_frame.data[1]), BYTE_TO_BINARY(spi_frame.data[0]));
            int j = 0;
            for(uint i=0; i<8; i++){
                uint32_t val = (uint32_t)((uint8_t)buf[j+3]<<24|(uint8_t)buf[j+2]<<16|(uint8_t)buf[j+1]<<8|(uint8_t)buf[j]);
                int valid = (val >> 29) & 0x1;
                if(!valid)
                    continue;
                lighthouse.push_back((val >> 31) & 0x1);
                axis.push_back((val >> 30) & 0x1);
                sensorID.push_back((val >>19) & 0x3FF);
                sweepDuration.push_back((val & 0x7FFFF));
                j+=4;
            }
            return !sensorID.empty();
        }else if(numbytes == 34){ // with timestamp
            union {
                uint32_t sensor[8];
                uint8_t data[32];
            }spi_frame;
            uint16_t timestamp = (uint16_t)(buf[1]<<8|buf[0]);
            memcpy(spi_frame.data, &buf[2], 32);
//            for(uint i = 0; i<32;i++){
//                printf("%d ",buf[i]);
//            }
//            printf("\n");
//            printf(BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " "
//                           BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN "\n",
//                   BYTE_TO_BINARY(spi_frame.data[3]), BYTE_TO_BINARY(spi_frame.data[2]),
//                   BYTE_TO_BINARY(spi_frame.data[1]), BYTE_TO_BINARY(spi_frame.data[0]));
            int j = 2;
            for(uint i=0; i<8; i++){
                uint32_t val = (uint32_t)((uint8_t)buf[j+3]<<24|(uint8_t)buf[j+2]<<16|(uint8_t)buf[j+1]<<8|(uint8_t)buf[j]);
                int valid = (val >> 29) & 0x1;
                if(!valid)
                    continue;
                lighthouse.push_back((val >> 31) & 0x1);
                axis.push_back((val >> 30) & 0x1);
                sensorID.push_back((val >>19) & 0x3FF);
                sweepDuration.push_back((val & 0x7FFFF));
                j+=4;
            }
            return !sensorID.empty();
        }else if(numbytes == 8){
            uint16_t magic_number = (uint16_t)((uint8_t)buf[1]<<8|(uint8_t)buf[0]);
            if(magic_number==0xBEEF){
                sensorID.push_back(buf[4]);
                lighthouse.push_back(buf[2]);
                axis.push_back(buf[3]);
                sweepDuration.push_back((uint16_t)((uint8_t)buf[7]<<8|(uint8_t)buf[6]));
//            ROS_INFO(BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(buf[3]), BYTE_TO_BINARY(buf[4]), BYTE_TO_BINARY(buf[5]));
                return true;
            }
        }
    }
    return false;
}

bool UDPSocket::convertByte2Text(uint32_t inet, char *inet_str){
    if(inet_ntop(AF_INET, &inet, inet_str, INET_ADDRSTRLEN) == NULL)
        return false;
    else
        return true;
}

bool UDPSocket::convertText2Byte(char *inet_str, uint32_t *inet){
    if(inet_pton(AF_INET, inet_str, inet)<= 0)
        return false;
    else
        return true;
}

bool UDPSocket::setTimeOut(int usecs) {
// set 10ms timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = usecs;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        fprintf(stderr, "Error setting timeout\n");
        return false;
    }
    return true;
}

bool UDPSocket::whatsMyIP(string &ip, string &broadcast_ip, bool preferEthernet) {
    struct ifaddrs *ifAddrStruct = NULL;
    void *tmpAddrPtr = NULL, *tmpAddrPtr2 = NULL;

    getifaddrs(&ifAddrStruct);
    char IP[INET_ADDRSTRLEN], Broadcast_IP[INET_ADDRSTRLEN];

    bool eth_ip = false, wifi_ip = false;
    string eth_ip_str, eth_broadcast_ip_str, wifi_ip_str, wifi_broadcast_ip_str;
    for (struct ifaddrs *ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;
            tmpAddrPtr2 = &((struct sockaddr_in *) ifa->ifa_ifu.ifu_broadaddr)->sin_addr;

            inet_ntop(AF_INET, tmpAddrPtr, IP, INET_ADDRSTRLEN);
            inet_ntop(AF_INET, tmpAddrPtr2, Broadcast_IP, INET_ADDRSTRLEN);
            string str(ifa->ifa_name);
            if (str.find("eth") != std::string::npos ||
                str.find("enp") != std::string::npos) { // if wifi or ethernet adapter
                printf("%s IP Address %s Broadcast IP %s\n", ifa->ifa_name, IP, Broadcast_IP);
                eth_ip_str = string(IP);
                eth_broadcast_ip_str = string(Broadcast_IP);
                eth_ip = true;
            }
            if (str.find("wlp") != std::string::npos ||
                str.find("wlx") != std::string::npos) { // if wifi or ethernet adapter
                printf("%s IP Address %s Broadcast IP %s\n", ifa->ifa_name, IP, Broadcast_IP);
                wifi_ip_str = string(IP);
                wifi_broadcast_ip_str = string(Broadcast_IP);
                wifi_ip = true;
            }
        }
    }

    if(preferEthernet && eth_ip){
        myIP.second = eth_ip_str;
        myBroadcastIP.second = eth_broadcast_ip_str;
        convertText2Byte((char*)myIP.second.c_str(),&myIP.first);
        convertText2Byte((char*)myIP.second.c_str(),&myBroadcastIP.first);
        printf("using eth IP Address %s\n", eth_ip_str.c_str());
        ip = myIP.second;
        broadcast_ip = myBroadcastIP.second;
        return true;
    }

    if(wifi_ip){
        myIP.second = wifi_ip_str;
        myBroadcastIP.second = wifi_broadcast_ip_str;
        convertText2Byte((char*)myIP.second.c_str(),&myIP.first);
        convertText2Byte((char*)myIP.second.c_str(),&myBroadcastIP.first);
        printf("using wifi IP Address %s\n", wifi_ip_str.c_str());
        ip = myIP.second;
        broadcast_ip = myBroadcastIP.second;
        return true;
    }else if(eth_ip){
        myIP.second = eth_ip_str;
        myBroadcastIP.second = eth_broadcast_ip_str;
        convertText2Byte((char*)myIP.second.c_str(),&myIP.first);
        convertText2Byte((char*)myIP.second.c_str(),&myBroadcastIP.first);
        printf("using eth IP Address %s\n", eth_ip_str.c_str());
        ip = myIP.second;
        broadcast_ip = myBroadcastIP.second;
        return true;
    }


    return false;
}

int UDPSocket::receiveUDP() {
    if ((numbytes = recv(sockfd, buf, MAXBUFLENGTH - 1, 0)) ==
        -1) {
//        ROS_DEBUG_THROTTLE(5, "received nothing");
        return 0;
    }else {
//        ROS_DEBUG_THROTTLE(5, "got message of length %ld: " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " "
//        BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " ", numbytes, BYTE_TO_BINARY(buf[4]),
//                BYTE_TO_BINARY(buf[3]), BYTE_TO_BINARY(buf[2]), BYTE_TO_BINARY(buf[1]), BYTE_TO_BINARY(buf[0]));
    }
    return numbytes;
}

int UDPSocket::receiveUDPFromClient() {
    if ((numbytes = recvfrom(sockfd, buf, MAXBUFLENGTH - 1, 0, (struct sockaddr *) &client_addr, &client_addr_len)) ==
        -1) {
//        ROS_DEBUG_THROTTLE(5, "received nothing");
        return 0;
    }else {
//        ROS_DEBUG_THROTTLE(1, "got message: " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " "
//        BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN " ",
//                BYTE_TO_BINARY(buf[3]), BYTE_TO_BINARY(buf[2]), BYTE_TO_BINARY(buf[1]), BYTE_TO_BINARY(buf[0]));
    }
    return numbytes;
}

bool UDPSocket::sendUDPToClient() {
    if ((numbytes = sendto(sockfd, buf, numbytes, 0, (struct sockaddr *) &client_addr, client_addr_len) == -1)) {
//        ROS_ERROR_THROTTLE(1, "could not send");
        return false;
    }
    return true;
}

bool UDPSocket::broadcastUDP() {
    if ((numbytes = sendto(sockfd, buf, numbytes, 0, (struct sockaddr *) &broadcast_addr, broadcast_addr_len)) ==
        -1) {
        printf("could not broadcast");
        return false;
    }
    return true;
}
