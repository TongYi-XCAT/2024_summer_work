#include <iostream>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <arpa/inet.h>

#define SERVER_PORT 8080
#define SERVER_IP "127.0.0.1"
#define LISTEN_NUM 10
#define BUFMAX 100

using namespace std;

int main()
{
    //缓冲区定义
    
    uint8_t buffer[BUFMAX];

    //创建Socket()，监听的套接字
    
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0)
    {
        cerr << "socket error" << endl;
        return 1;
    }
    
    //绑定Socket()返回值和网络端口
    
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) < 0)
    {
        close(serverSocket);
        cerr << "inet_pton error" << endl;
        return -1;
    }
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        close(serverSocket);
        cerr << "bind error" << endl;
        return -1;
    }

    //设置监听

    if (listen(serverSocket, LISTEN_NUM) < 0)
    {
        close(serverSocket);
        std::cout << "listen error" << std::endl;
        return -1;
    }

    //创建客户端套接字
    cout << "listening~~~" << endl;
    struct sockaddr_in clientAddr;
    socklen_t clientAddrSize = sizeof(clientAddr);
    memset(&clientAddr, 0, sizeof(clientAddr));
    int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientAddrSize);
    if (clientSocket < 0)
    {
        close(serverSocket);
        cerr << "accept error" << endl;
        return -1;
    }

    //初始化缓冲区，接收！

    memset(buffer, 0, sizeof(buffer));
    if (recv(clientSocket, buffer, sizeof(buffer), 0) < 8)
    {
        close(serverSocket);
        close(clientSocket);
        cerr << "recv error" << endl;
    }

    //从客户端收信息输出

    for (uint8_t value : buffer)
    {
        cout << hex << static_cast<int>(value) << " ";
    }
    cout << endl;

    close(serverSocket);
    close(clientSocket);
    return 0;
}