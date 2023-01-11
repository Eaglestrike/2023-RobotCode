#pragma once

#include <string>
#include <vector>

#ifdef _WIN32
// no windows support necessary
#else
extern "C" {
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
}
#endif


namespace network_library {

class NetworkClient
{
public:
    constexpr static int REC_BUF_SIZE = 1024;

    NetworkClient(const std::string& remote_addr, const int port);
    ~NetworkClient();

    int send(const std::string& message);
    std::string receive();


private:
    int m_sockfd;
    #ifdef _WIN32
    // TODO add windows socket here
    #else
    struct sockaddr_in m_remote_addr;
    #endif
    std::vector<char> m_recv_buffer;
};

}
