#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

class SocketClient
{
public:
  SocketClient(std::string host, int port, unsigned long long staleTime, unsigned long long deadTime);

  void Init();

  bool HasConn();
  bool IsStale();

  std::vector<double> GetData();

private:
  void m_SocketLoop(std::string host, int port);

  std::thread m_th;

  std::string m_host;
  int m_port;
  unsigned long long m_staleTime;
  unsigned long long m_deadTime;

  std::atomic<unsigned long long> m_lastTimeMs;

  std::atomic<bool> m_hasInit;
  std::atomic<bool> m_hasConn;

  std::atomic<int> m_camId;
  std::atomic<int> m_tagId;
  std::atomic<double> m_x;
  std::atomic<double> m_y;
  std::atomic<double> m_angZ;
  std::atomic<long long> m_age;
  std::atomic<unsigned long long> m_count;
};
