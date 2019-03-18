#include <ros/ros.h>
#include <common_utilities/UDPSocket.hpp>
#include <boost/regex_fwd.hpp>
#include <boost/regex.hpp>

const char* key = "The path of the righteous man is beset on all sides by the inequities of the "
                  "selfish and the tyranny of evil men. Blessed is he who, in the name of "
                  "charity and good will, shepherds the weak through the valley of the darkness. "
                  "For he is truly his brother's keeper and the finder of lost children. And I "
                  "will strike down upon thee with great vengeance and furious anger those who "
                  "attempt to poison and destroy my brothers. And you will know I am the Lord "
                  "when I lay my vengeance upon you\0";

int main(int argc, char *argv[]){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ROS_MASTER_URI_receiver");
    }
    // create the IP broadcast Socket
    UDPSocketPtr receiver_socket = UDPSocketPtr(new UDPSocket(BROADCAST_PORT, false));
    uint32_t status = 0;


    char username[20];
    getlogin_r(username, 20);

    string user = string(username);
    user.erase(std::remove(user.begin(), user.end(), '\n'), user.end());

    ostringstream bashrc;
    ifstream in_file("/home/"+user+"/.bashrc");


    bashrc << in_file.rdbuf();
    string str = bashrc.str();

    const char* pattern_ip = "\nexport ROS_MASTER_URI=http://\\b(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b:11311";//.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\"\n"
    const char* pattern_env = "\nexport ROS_MASTER_URI=http://\\$ROS_IP:11311";
    boost::regex re_ip(pattern_ip);
    boost::regex re_env(pattern_env);

    ROS_INFO_STREAM("listening for HOST IP\n");
    while(status!=3) {// && !ros::master::check()) {
        usleep(10000);

        uint32_t host_IP;
        status = receiver_socket->receiveHostIP(key, host_IP);
        if (status == 3) {
            char IP[16];
            receiver_socket->convertByte2Text(host_IP, IP);
            ROS_INFO("Received HOST IP: %s", IP);
            char ros_master_uri[100];
            sprintf(ros_master_uri, "\nexport ROS_MASTER_URI=http://%s:11311", IP);
//            printf("%s\n", ros_master_uri);
            setenv("ROS_MASTER_URI", ros_master_uri, 1);

            boost::cmatch what;
            string newstr;

            if (boost::regex_search(str.c_str(), what, re_ip)) {
                newstr = boost::regex_replace(str, re_ip, ros_master_uri);
            }
            else {

                if (boost::regex_search(str.c_str(), what, re_env)) {
                    newstr = boost::regex_replace(str, re_env, ros_master_uri);
                } else {
                    newstr = str + ros_master_uri;
                }
            }

            in_file.close();
            ofstream out_file("/home/"+user+"/.bashrc");
            out_file << newstr;

            ROS_INFO_STREAM("Updated /home/"+user+"/.bashrc with: " << ros_master_uri);

//            char hostname[20];
//            /* Open the command for reading. */
//            FILE *fp = popen("hostname", "r");
//            fgets(hostname, sizeof(hostname)-1, fp);
//
//            string name = string(hostname);
//            name.erase(std::remove(name.begin(), name.end(), '\n'), name.end());
//
//            printf("Hostname: %s\n", hostname );
//            auto nh = ros::NodeHandlePtr(new ros::NodeHandle(name + "_config"));
//            int test;
//            nh->getParam("test", test);
//            ROS_INFO_STREAM(test);

        }
    }
    system (argv[1]);
}