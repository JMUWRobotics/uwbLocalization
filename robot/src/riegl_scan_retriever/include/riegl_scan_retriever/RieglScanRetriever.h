//
// Created by lant on 23.11.21.
//

#ifndef ESA_CHALLENGE_RIEGLSCANRETRIEVER_H
#define ESA_CHALLENGE_RIEGLSCANRETRIEVER_H

#include <riegl_scan_retriever/get_scan.h>
#include <ros/node_handle.h>


#define C_SERVICE_NAME "scan_retriever"
std::set<std::string> get_directories(const std::string& directory);
class RieglScanRetriever {
public:
    RieglScanRetriever(std::string ipAddress, const ros::NodeHandle &nodeHandle, std::string scan_dir);
    bool retrieveScan(riegl_scan_retriever::get_scan::Request &req, riegl_scan_retriever::get_scan::Response &response);

    void setService(ros::ServiceServer& server);

private :

    std::string _ipAddress;
    ros::NodeHandle _nodeHandle;
    ros::ServiceServer _server;
    std::string _scan_dir;
};


#endif //ESA_CHALLENGE_RIEGLSCANRETRIEVER_H
