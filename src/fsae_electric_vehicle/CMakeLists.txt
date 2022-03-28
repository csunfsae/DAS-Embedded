cmake_minimum_required(VERSION 2.8.3)

project(fsae_electric_vehicle)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  std_msgs
  genmsg
  message_generation
)

roslaunch_add_file_check(src/launch)

add_message_files(FILES
  gps.msg
  serial.msg
  speedometer.msg
  brake_pressure.msg
  coolant.msg
  drive_train.msg
  suspension.msg
  data_struct.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	INCLUDE_DIRS socket.io-client-cpp
	INCLUDE_DIRS json-develop
	INCLUDE_DIRS CANController
 	INCLUDE_DIRS SignalR-Client-Cpp-main
	CATKIN_DEPENDS message_runtime
)

include_directories(

  socket.io-client-cpp
  json-develop
  json-develop/include/nlohmann
  json-develop/include
  CANController
  SignalR-Client-Cpp-main
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SignalR-Client-Cpp-main/src/signalrclient/hub_connection.cpp SignalR-Client-Cpp-main/src/signalrclient/signalr_value.cpp SignalR-Client-Cpp-main/src/signalrclient/hub_connection_builder.cpp SignalR-Client-Cpp-main/src/signalrclient/hub_connection_impl.cpp SignalR-Client-Cpp-main/src/signalrclient/callback_manager.cpp SignalR-Client-Cpp-main/src/signalrclient/connection_impl.cpp SignalR-Client-Cpp-main/src/signalrclient/signalr_client_config.cpp SignalR-Client-Cpp-main/src/signalrclient/logger.cpp SignalR-Client-Cpp-main/src/signalrclient/cancellation_token_source.cpp SignalR-Client-Cpp-main/src/signalrclient/transport_factory.cpp SignalR-Client-Cpp-main/src/signalrclient/signalr_default_scheduler.cpp SignalR-Client-Cpp-main/src/signalrclient/json_hub_protocol.cpp SignalR-Client-Cpp-main/src/signalrclient/url_builder.cpp SignalR-Client-Cpp-main/src/signalrclient/trace_log_writer.cpp SignalR-Client-Cpp-main/src/signalrclient/negotiate.cpp SignalR-Client-Cpp-main/src/signalrclient/handshake_protocol.cpp SignalR-Client-Cpp-main/src/signalrclient/jsoncpp.cpp SignalR-Client-Cpp-main/src/signalrclient/uri_builder.cpp SignalR-Client-Cpp-main/src/signalrclient/json_helpers.cpp SignalR-Client-Cpp-main/src/signalrclient/uri.cpp SignalR-Client-Cpp-main/src/signalrclient/cancellation_token.cpp SignalR-Client-Cpp-main/src/signalrclient/websocket_transport.cpp SignalR-Client-Cpp-main/src/signalrclient/transport.cpp


socket.io-client-cpp/src/sio_client.cpp socket.io-client-cpp/src/sio_socket.cpp socket.io-client-cpp/src/internal/sio_client_impl.cpp socket.io-client-cpp/src/internal/sio_packet.cpp json-develop/include/nlohmann CANController/CANController.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC 
    SignalR-Client-Cpp-main/src/signalrclient
    #SignalR-Client-Cpp-main/include/signalrclient/
    socket.io-client-cpp/src 
    socket.io-client-cpp/lib/websocketpp 
    socket.io-client-cpp/lib/rapidjson/include
    json-develop/include/nlohmann
    CANController
)


add_executable(DriveTrain src/cpp/drivetrain/drive_train.cpp CANController/CANController.cpp)
target_link_libraries(DriveTrain ${catkin_LIBRARIES})
target_compile_features(DriveTrain PUBLIC cxx_std_17)
add_dependencies(DriveTrain fsae_electric_vehicle_generate_messages_cpp)

add_executable(CoolantTemp src/cpp/Coolant_temp/main.cpp  CANController/CANController.cpp)
target_link_libraries(CoolantTemp ${catkin_LIBRARIES})
target_compile_features(CoolantTemp PUBLIC cxx_std_17)
add_dependencies(CoolantTemp fsae_electric_vehicle_generate_messages_cpp)

add_executable(Brakes src/cpp/brakes/brake_pressure.cpp CANController/CANController.cpp)
target_link_libraries(Brakes ${catkin_LIBRARIES})
target_compile_features(Brakes PUBLIC cxx_std_17)
add_dependencies(Brakes fsae_electric_vehicle_generate_messages_cpp)

add_executable(SocketIOSender src/cpp/SocketIOSender/main.cpp)
target_link_libraries(SocketIOSender ${catkin_LIBRARIES} ${PROJECT_NAME})
target_compile_features(SocketIOSender PUBLIC cxx_std_17)
add_dependencies(SocketIOSender fsae_electric_vehicle_generate_messages_cpp)

add_executable(SPEEDOMETER src/cpp/speedometer/main.cpp CANController/CANController.cpp)
target_link_libraries(SPEEDOMETER ${catkin_LIBRARIES})
target_compile_features(SPEEDOMETER PUBLIC cxx_std_17)
add_dependencies(SPEEDOMETER fsae_electric_vehicle_generate_messages_cpp)

add_executable(Suspension src/cpp/suspension/main.cpp CANController/CANController.cpp)
target_link_libraries(Suspension ${catkin_LIBRARIES})
target_compile_features(Suspension PUBLIC cxx_std_17)
add_dependencies(Suspension fsae_electric_vehicle_generate_messages_cpp)

add_executable(gps_lap_timer src/cpp/gps_lap_timer/gps_lap_timer.cpp src/cpp/gps_lap_timer/GPSUtilities.cpp CANController/CANController.cpp)
target_link_libraries(gps_lap_timer ${catkin_LIBRARIES})
target_compile_features(gps_lap_timer PUBLIC cxx_std_17)
add_dependencies(gps_lap_timer fsae_electric_vehicle_generate_messages_cpp)


########################## These are old python files ############################
#
# This reader.py is for reading all data being published (except gps data) and outputting it
#
#catkin_install_python(
# PROGRAMS
#  src/python/reader/reader.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
# This gps.py is for reading GPS data being published and outputting it
#
#catkin_install_python(
#  PROGRAMS
#  src/python/gps/gps.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
##################################################################################



install(TARGETS SocketIOSender
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS SPEEDOMETER
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS Brakes
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS CoolantTemp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS DriveTrain
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS Suspension
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS gps_lap_timer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

message("Bin Dir is ${CATKIN_PACKAGE_BIN_DESTINATION}")

install(PROGRAMS
   src/js/reader.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install (FILES
  src/launch/main.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

