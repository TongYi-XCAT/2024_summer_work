CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(socket)

ADD_EXECUTABLE(server ./src/server.cpp)
ADD_EXECUTABLE(client ./src/client.cpp)