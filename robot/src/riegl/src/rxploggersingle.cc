#include <ros/ros.h>

#include <riegl/scanlib.hpp>
#include <riegl/RieglStatus.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include <string.h>
#include <iostream>
#include <exception>
#include <cmath>
#include <limits>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// You might need to adjust the below include path, depending on your
// compiler setup and which tr1 implementation you are using.
#if defined(_MSC_VER)
#   include <memory>
#else
#   include <tr1/memory>
#endif

using namespace scanlib;
using namespace std;
using namespace std::tr1;


#include <rclock/logDir.h>

/**
 * Converts a class T to a string of width with padding 0
 *
 * @param t output
 * @param width length
 *
 * @return string of t
 *
 */
template <class T>
inline std::string to_string(const T& t, int width)
{
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(width) << t;
  return ss.str();
}

bool finished = false;

void statusCallback(const riegl::RieglStatus::ConstPtr& rs) {
  if (rs->status == 2) {
    finished = true;
    ROS_INFO("FINISHED");
  } else {
    finished = false;
  }
}

/**
 * Converts a right-hand-side matrix into a 3DTK matrix
 * @param *inMatrix pointer to matrix (double[16])
 * @param *outMatrix pointer to matrix (double[16])
 * @param scale used for unit conversion, default 100.0 for Riegl
 */
inline void to3DTKMat(const double *inMatrix,
          double *outMatrix, float scale = 100.0)
{
    outMatrix[0] = inMatrix[5];
    outMatrix[1] = -inMatrix[9];
    outMatrix[2] = -inMatrix[1];
    outMatrix[3] = -inMatrix[13];
    outMatrix[4] = -inMatrix[6];
    outMatrix[5] = inMatrix[10];
    outMatrix[6] = inMatrix[2];
    outMatrix[7] = inMatrix[14];
    outMatrix[8] = -inMatrix[4];
    outMatrix[9] = inMatrix[8];
    outMatrix[10] = inMatrix[0];
    outMatrix[11] = inMatrix[12];
    outMatrix[12] = -scale*inMatrix[7];
    outMatrix[13] = scale*inMatrix[11];
    outMatrix[14] = scale*inMatrix[3];
    outMatrix[15] = inMatrix[15];
}

static inline void Matrix4ToEuler(const double *alignxf,
                                  double *rPosTheta,
                                  double *rPos = 0)
{

  double _trX, _trY;

  // Calculate Y-axis angle
  if(alignxf[0] > 0.0) {
    rPosTheta[1] = asin(alignxf[8]);
  } else {
    rPosTheta[1] = M_PI - asin(alignxf[8]);
  }

  double  C    =  cos( rPosTheta[1] );
  if ( fabs( C ) > 0.005 )  {                 // Gimbal lock?
    _trX      =  alignxf[10] / C;             // No, so get X-axis angle
    _trY      =  -alignxf[9] / C;
    rPosTheta[0]  = atan2( _trY, _trX );
    _trX      =  alignxf[0] / C;              // Get Z-axis angle
    _trY      = -alignxf[4] / C;
    rPosTheta[2]  = atan2( _trY, _trX );
  } else {                                    // Gimbal lock has occurred
    rPosTheta[0] = 0.0;                       // Set X-axis angle to zero
    _trX      =  alignxf[5];  //1                // And calculate Z-axis angle
    _trY      =  alignxf[1];  //2
    rPosTheta[2]  = atan2( _trY, _trX );
  }

  rPosTheta[0] = rPosTheta[0];
  rPosTheta[1] = rPosTheta[1];
  rPosTheta[2] = rPosTheta[2];

  if (rPos != 0) {
    rPos[0] = alignxf[12];
    rPos[1] = alignxf[13];
    rPos[2] = alignxf[14];
  }
}

int main(int argc, char* argv[])
{
  int BUFLENGTH = 1024;
  char buf[BUFLENGTH];
  ros::init(argc, argv, "RXPLogger");
  ros::NodeHandle n;
  string ip, fileprefix, filename, posename;
  n.param<std::string>("/riegl/ip", ip, "192.168.0.125");
  n.param<std::string>("/log/rxp", fileprefix, "scan");

  // wait for global logging directory to become available
  ros::service::waitForService("logDirectory");
  // request path to logging dir
  rclock::logDir::Request empty;
  rclock::logDir::Response dir;
  ros::service::call("logDirectory", empty, dir);
  ros::Subscriber status_sub = n.subscribe("rieglstatus", 100, statusCallback);

  // add rxp to dir

  try {
    // The basic_rconnection class contains the communication
    // protocol between the scanner or file and the program.
    std::shared_ptr<basic_rconnection> rc;

    for(unsigned int scan_index = 0; ros::ok(); scan_index++) {
      //repeatedly open new files
      rc = basic_rconnection::create("rdtp://" + ip +"/current");
      rc->open();
      ROS_INFO("Opening file %d", scan_index);
      finished = false;
      filename = dir.directory + fileprefix + to_string(scan_index, 3) + ".rxp"; // /tmp/dat/YY_MM_DD_HH_MM_SS/raw.rxp
      posename = dir.directory + fileprefix + to_string(scan_index, 3) + ".pose"; 
      FILE *p;
      p = fopen(posename.c_str(),"w");
      while(ros::ok() && !finished) {
        int f = open(filename.c_str(), O_WRONLY|O_APPEND|O_CREAT, S_IRWXU|S_IRWXG|S_IRWXO);
        int read = rc->readsome(buf, BUFLENGTH);
        if (read < 0 ) { 
          ROS_FATAL("RXPLOGGER: Error while reading from VZ-400. Code: %d", -read);
          close(f);
          exit(0);
        }
        int written = write(f, buf, read);

        if (written < 0 ) { 
          ROS_FATAL("RXPLOGGER: Error while writing to file %s. Code: %d", filename.c_str(), -written);
          close(f);
          exit(0);
        }

        close(f);
        ros::spinOnce();
      }
      rc->close();
      if(p != NULL) {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        listener.waitForTransform("/odom_combined", "/riegl", ros::Time(0), ros::Duration(3.0) );
        listener.lookupTransform("/odom_combined", "/riegl", ros::Time(0), transform);

        tf::Matrix3x3 m(transform.getRotation());
        const double in_matrix[16] ={m[0][0],m[0][1],m[0][2], transform.getOrigin().getX(),
                                    m[1][0],m[1][1],m[1][2], transform.getOrigin().getY(),
                                    m[2][0],m[2][1],m[2][2], transform.getOrigin().getZ(),
                                    0      ,0      ,0      , 1};

        // Converting to left handed matrix
        double out_matrix[16], rPos[3], rPosTheta[16];
        to3DTKMat(in_matrix, out_matrix,1);
        Matrix4ToEuler(out_matrix,rPosTheta,rPos);

        // Extracting Position and Orientaion
        double x = 100.0*rPos[0];
        double y = 100.0*rPos[1];
        double z = 100.0*rPos[2];
        double roll  = 1.0*rPosTheta[0]/M_PI*180;
        double pitch = 1.0*rPosTheta[1]/M_PI*180;
        double yaw   = 1.0*rPosTheta[2]/M_PI*180;

        fprintf(p,"%f %f %f\n%f %f %f", x, y, z, roll, pitch, yaw); //x y z \n roll pitch yaw
        fclose(p);
      }      
    }
    return 0;
  }
  catch(exception& e) {
    cerr << e.what() << endl;
    return 1;
  }
  catch(...) {
    cerr << "unknown exception" << endl;
    return 1;
  }

  return 0;
}
