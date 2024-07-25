#include <iostream>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <iomanip>
#include <vector>
#include <arpa/inet.h>

#define CLIENT_PORT 8000
#define SERVER_IP "10.2.20.66"


typedef struct {
    //数据头部
    u_int16_t Start = 0x0D00;
    u_int16_t MessageType;
    u_int32_t DataID;
    u_int32_t DataTotalLength;
    u_int32_t Offset;
    u_int32_t DataLength;
    //数据
    u_char Data[10218];
    //数据尾部
    u_int16_t End = 0x0721;
} MessageBuffer;

enum MessageType {
    STRING_MSG = 0X0000,
    IMAGE_MSG = 0X1145,
    CAMERA_INFO = 0X1419,
    TRANSFORM = 0X1981,
    TRANSFORM_REQUEST = 0X1982
};

typedef struct{
    double CameraMatrix[9];
    double DistortionCoefficients[5];
} CameraInfoData;

typedef struct{
    double Translation[3];
    double Rotation[4];
} TransformData;

typedef struct{
    char From[10218 / 2];
    char To[10218 / 2];
} TransformRequestData;

int main()
{

    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket < 0)
    {
        std::cerr << "socket error" << std::endl;
        return -1;
    }

    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(CLIENT_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) < 0)
    {
        std::cerr << "inet_pton error" << std::endl;
        close(clientSocket);
        return -1;
    }
    std::cout << "con~~" << std::endl;
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::cerr << "connect error" << std::endl;
        close(clientSocket);
        return -1;
    }

    
    std::vector<MessageBuffer> Data;
    std::vector<u_int8_t*> Vid;
    MessageBuffer buffer;
    //视频读完的标记
    bool flag = false;
    //只读个视频
    while(true)
    {
        //初始化缓冲区
        memset(&buffer, 0, sizeof(buffer));
        //没消息就是读完了
        if (recv(clientSocket, &buffer, sizeof(buffer), 0) < 8)
        {
            std::cerr << "recv error" << std::endl;
            break;
        }
        else
        {
            //这是视频
            if(buffer.MessageType == IMAGE_MSG)
            {
                //这是某张的第一包数据
                u_int8_t* img;
                if(Vid.size() < buffer.DataID && buffer.DataTotalLength > buffer.DataLength)
                {
                    img = new u_int8_t[buffer.DataTotalLength];
                    memcpy(img, buffer.Data, buffer.DataLength);
                    Vid.push_back(img);
                }
                if(Vid.size() == buffer.DataID && buffer.Offset+buffer.DataLength < buffer.DataTotalLength)
                {
                    memcpy(img+buffer.Offset, buffer.Data, buffer.DataLength);
                }
                if(Vid.size() == buffer.DataID && buffer.Offset+buffer.DataLength == buffer.DataTotalLength)
                {
                    memcpy(img+buffer.Offset, buffer.Data, buffer.DataLength);
                    std::cout << buffer.DataID << std::endl;
                }
            }
            if(buffer.MessageType == STRING_MSG)
            {
                if(buffer.DataTotalLength == buffer.DataLength)
                {
                    char str[buffer.DataTotalLength];
                    std::string show = str;
                    memcpy(str, buffer.Data, buffer.DataLength);
                    std::cout << show << std::endl;
                }
            }
        }
    }
    

    close(clientSocket);

    return 0;
}
