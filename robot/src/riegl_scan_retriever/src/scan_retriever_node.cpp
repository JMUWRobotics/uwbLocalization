//
// Created by lant on 23.11.21.
//

#include <ros/ros.h>
#include <riegl_scan_retriever/RieglScanRetriever.h>

int main(int argcv, char **argv) {
    ros::init(argcv, argv, "riegl_scan_retriever");
    ros::NodeHandle nodeHandle("~");
    std::string ip;
    nodeHandle.getParam("ip", ip);
    std::string scan_dir;
    nodeHandle.getParam("scan_dir", scan_dir);
    RieglScanRetriever retriever(ip, nodeHandle, scan_dir);
    auto service = nodeHandle.advertiseService(C_SERVICE_NAME, &RieglScanRetriever::retrieveScan, &retriever);
    retriever.setService(service);
    ros::spin();
}