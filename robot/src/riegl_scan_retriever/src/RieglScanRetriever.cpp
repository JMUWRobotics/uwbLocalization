//
// Created by lant on 23.11.21.
//

#include "riegl_scan_retriever/RieglScanRetriever.h"
#include <pwd.h>
#include <string>
#include <iostream>
#include <dirent.h>
#include <utility>
#include <sys/stat.h>

bool existsDir(const char *path) {
    struct stat info{};
    return stat(path, &info) == 0 && (info.st_mode & S_IFDIR);
}

/**
 * Returns the file names of the given directory
 */
std::set<std::string> files(const std::string &directory) {
    DIR *dir;
    std::set<std::string> files;

    struct dirent *ent;
    if ((dir = opendir(directory.c_str())) != nullptr) {
        /* print all the files and directories within directory */
        while ((ent = readdir(dir)) != nullptr) {
            files.insert(ent->d_name);
        }
        closedir(dir);
    } else {
        /* could not open directory */
        perror("");
        ROS_ERROR("Could not open directory");
        return files;
    }
    return files;
}

/**
 * returns all directories within the given directory, except . and ..
 */
std::set<std::string> get_directories(const std::string &directory) {
    std::set<std::string> sorted_by_name;

    for (auto &entry: files(directory)) {
        if (entry == "." || entry == "..")
            continue;

        auto file = directory + "/" + entry;
        if (existsDir(file.c_str()))
            sorted_by_name.insert(file);
    }
    return sorted_by_name;
}

bool RieglScanRetriever::retrieveScan(riegl_scan_retriever::get_scan::Request &req,
                                      riegl_scan_retriever::get_scan::Response &response) {
    int rate;
    _nodeHandle.getParam("rate", rate);
    std::string all_scans_directory = _scan_dir;
    std::set<std::string> directories = get_directories(all_scans_directory);
    if (directories.empty()) {
        ROS_ERROR("no directories found in %s", all_scans_directory.c_str());
        return false;
    }

    auto target_directory = *directories.begin() + "/";
    auto command = "uftp -H " + _ipAddress + " -M " + _ipAddress
                   + " -R " + std::to_string(rate)
                   + "-z " + target_directory;
    std::cout << command << std::endl;
    int status = system(command.c_str());
    if (status < 0)
        ROS_ERROR("Error: %s", strerror(errno));
    else {
        if (WIFEXITED(status))
            ROS_INFO("Program returned normally, exit code %i", WEXITSTATUS(status));
        else
            ROS_ERROR("Program exited abnormally, exit code %i", WEXITSTATUS(status));
    }

    ROS_INFO("Transfer finished with error code %i", WEXITSTATUS(status));
    response.error_code = WEXITSTATUS(status);
    return true;
}

RieglScanRetriever::RieglScanRetriever(std::string ipAddress, const ros::NodeHandle &nodeHandle, std::string scan_dir)
        : _ipAddress(std::move(ipAddress)), _nodeHandle(nodeHandle),
          _scan_dir(std::move(scan_dir)) {

}

void RieglScanRetriever::setService(ros::ServiceServer &server) {
    this->_server = server;
}
