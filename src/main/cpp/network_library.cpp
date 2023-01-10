#include "network_library.h"

#include <cstdio>
#include <cstring>
#include <iostream>

extern "C" {
#include <fcntl.h>
}


namespace network_library
{

NetworkClient::NetworkClient(const std::string& remote_addr, const int port)
    : m_sockfd(socket(PF_INET, SOCK_DGRAM, 0))
    , m_remote_addr()
    , m_recv_buffer(REC_BUF_SIZE)
{
    if (m_sockfd < 0) {
        perror("[NetworkClient::NetworkClient] failed to create socket");
        return;
	}

    // mark socket nonblocking for real-time operations
    int status = fcntl(m_sockfd, F_SETFL, fcntl(m_sockfd, F_GETFL, 0) | O_NONBLOCK);
    if (status == -1) {
        perror("[NetworkClient::NetworkClient] failed to configure nonblocking socket");
        m_sockfd = -1;
        return;
    }

	memset(&m_remote_addr, 0, sizeof(m_remote_addr));
	m_remote_addr.sin_family = AF_INET;

    inet_pton(AF_INET, remote_addr.c_str(), &m_remote_addr.sin_addr);
    m_remote_addr.sin_port = htons(port);
}

int NetworkClient::send(const std::string& message)
{
    if (m_sockfd < 0) {
        return -1;
    }

    if (message.empty()) {
        return 0;
    }

	// send data
	const auto send_status = sendto(m_sockfd, message.c_str(), message.length(), 0,
                                    (struct sockaddr*)&m_remote_addr, sizeof(m_remote_addr));
    if (send_status < 0)
    {
        perror("[NetworkClient::send] failed to send message");
    }
    return send_status;
}

std::string NetworkClient::receive()
{
    // received echoed data back
	const auto received_bytes = recv(m_sockfd, m_recv_buffer.data(), REC_BUF_SIZE, 0);
    if (received_bytes == 0)
    {
        // no messages to receive, or no connection
        return "";
    }
    if (received_bytes < 0)
    {
        const bool no_data = (errno == EAGAIN) || (errno == EWOULDBLOCK);
        if (no_data) {
            // No data to receive, so just return empty
            return "";
        }

        // actual error, so print a message
        perror("[NetworkClient::receive] failed to receive message");
        return "";
    }

    std::string result;
    result.resize(received_bytes);
    for (int i = 0; i < received_bytes; i++)
    {
        result[i] = m_recv_buffer.at(i);
    }
    return result;
}

NetworkClient::~NetworkClient()
{
    if (m_sockfd > 0) {
        close(m_sockfd);
    }
}

}