#include <ros/ros.h>

#include <riegl/scanlib.hpp>

#include <iostream>
#include <exception>
#include <cmath>
#include <limits>
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

#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "riegl/RieglStatus.h"
#include "sensor_msgs/LaserScan.h"

using geometry_msgs::Point32;
// The import class is derived from pointcloud class, which assembles the
// scanner data into distinct targets and computes x,y,z pointcloud data.
// The pointcloud class and its base class have a huge number of overridables
// that give access to all of the scanners data, e.g. "gps" or "housekeeping"
// data. The pointcloud class also has the necessary logic to align the
// data to gps information (if embedded in the rxp stream) and will return
// timestamps in the domain of the gps.

//#include "newmat/newmatap.h"
//#include "newmat/newmat.h"
//#include "globals.icc"


double TransMat[16];

template <class T> inline void Transform(T *V)
{
  T X = TransMat[0] * V[0] + TransMat[4] * V[1] + TransMat[8] * V[2] + TransMat[12];
  T Y = TransMat[1] * V[0] + TransMat[5] * V[1] + TransMat[9] * V[2] + TransMat[13];
  T Z = TransMat[2] * V[0] + TransMat[6] * V[1] + TransMat[10] * V[2] + TransMat[14];
  V[0] = X;
  V[1] = Y;
  V[2] = Z;
}

#define MAPNUM 2
#define MAPBUCKETS 720 
#define MAPEXTENT 30.0

#define MAP_UNKOWN -1
#define MAP_FREE 0
#define MAP_FULL 100
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class importer
: public pointcloud
{

  public:
    template <class T> inline T rad(const T deg) {
      return ( (2 * M_PI * deg) / 360 );
    }

    template <class T>
    inline T deg(const T rad)
    {
        return ( (rad * 360) / (2 * M_PI) );
    }



    importer() : pointcloud(false) // set this to true if you need gps aligned timing
  {
     
    points.reserve(5000000);                // TODO think about the size
    stop_service = n.advertiseService("stopSending", &importer::stopSending, this);
    start_service = n.advertiseService("startSending", &importer::startSending, this);
    map_service = n.advertiseService("sendMapOnly", &importer::sendMapOnly, this);
    full_service = n.advertiseService("sendFull", &importer::sendFull, this);
     
    double freq = 50;
    resp = 0.5;
    
    laserMsg.header.frame_id="riegl";
    laserMsg.angle_min = 0;
    laserMsg.angle_max = rad(360.0);
    laserMsg.angle_increment = rad(resp);
    laserMsg.time_increment  = 1/(freq*MAPBUCKETS);
    laserMsg.scan_time       = 1.0/freq;
    laserMsg.range_min = 0.05;
    laserMsg.range_max = 50.0;
    laserMsg.ranges.resize(MAPBUCKETS);
    laserMsg.intensities.resize(0);
    
    pointMsg.header.frame_id="riegl";
    pointMsg.channels.resize(1);
    pointMsg.channels[0].name="intensity";

    STATE = 1;
    map_requested = false;

    resetMap();

    status = n.advertise<riegl::RieglStatus>("rieglstatus",100);
    map_publisher = n.advertise<sensor_msgs::LaserScan>("riegl2d",100);
    point_publisher = n.advertise<sensor_msgs::PointCloud>("riegl3d",100);
    //sendMap();
  }

    bool done() { 
      return map_requested;
    }

  protected:
    bool map_requested;
    ros::NodeHandle n;
    ros::Publisher status;
    ros::Publisher map_publisher; 
    ros::Publisher point_publisher; 
    ros::ServiceServer start_service; 
    ros::ServiceServer stop_service; 
    ros::ServiceServer map_service;
    ros::ServiceServer full_service;

    int pointcount;
    vector<double *> points;
    sensor_msgs::LaserScan laserMsg; 
    sensor_msgs::PointCloud pointMsg; 
    double resp;
    int STATE;      // 0: no data is published
                    // 1: 2D map is published scans are ignored
                    // 2: 2D map and subsampled 3D scan is published 
                    // 3: 2D map and full 3D scan is published 
    int SAMPLE = 10; // only every 10th point
    
    //publish 2D slice and reduced point cloud
    bool startSending(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response) {
      STATE = 2;
      pointcount = 0;
      return true;
    }
    
    //publish only 2D point data
    bool sendMapOnly(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response) {
      STATE = 1;
      pointcount = 0;
      return true;
    }
    
    //Publish full point cloud
    bool sendFull(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response) {
      STATE = 3;
      pointcount = 0;
      return true;
    }
    
    //Do not publish any point data
    bool stopSending(std_srvs::Empty::Request& request, std_srvs::Empty::Request& response) {
      STATE = 0;
      pointcount = 0;
      return true;
    }

    inline void toPolar(double x, double y, double * polar) {
      double r = sqrt(x*x + y*y);
      double theta = atan2(y,x);

      polar[0] = r;
      polar[1] = theta;
    }

    // add point to 2D laserscan message
    inline bool mapset(double x, double y) {
      double * polar = new double[2];
      toPolar(y, x, polar); 
      int tindex = (int)((deg(polar[1]) + 270.0) * MAPNUM);
      tindex = (MAPBUCKETS + MAPBUCKETS - 1 - tindex)%MAPBUCKETS; 
      //ROS_ERROR("%d %lf", tindex, deg(polar[1]));
      if(tindex >= 0 && tindex < MAPBUCKETS) {
        if(polar[0] < MAPEXTENT && polar[0] > 0.4) {
	        laserMsg.ranges[tindex] = polar[0];
          if(laserMsg.ranges[tindex] < 0 || (laserMsg.ranges[tindex] > -1 && polar[1] < laserMsg.ranges[tindex])) {
              laserMsg.ranges[tindex] = polar[0];
              //ROS_ERROR("%lf", polar[0]);
          } 
          delete[] polar;
          return true;
        }
      }
      delete[] polar;
      return false;
    }

    bool mapCallback( )
    {
      map_requested = true;
      return true;
    }


    void resetMap() {
      for(int j = 0; j < MAPBUCKETS; j++)
      {
        laserMsg.ranges[j] = -2;
      }
    }

    void sendMap() {    
      laserMsg.header.stamp = ros::Time::now();
      pointMsg.header.stamp = ros::Time::now();

      if(STATE > 0) map_publisher.publish(laserMsg);
      if(STATE > 1) point_publisher.publish(pointMsg);
    }

    // overridden from pointcloud class
    void on_echo_transformed(echo_type echo)
    {
      //publish laserscan 
      if (STATE > 0) {
        target& t(targets[target_count - 1]);
        double *point = new double[4];
        point[0] = t.vertex[0];
        point[1] = t.vertex[1];
        point[2] = t.vertex[2];
        point[3] = t.reflectance;

        // only points with certain angle
        // not sure why, though? maybe to avoid seeing robot
        pointcount++;
        double range = std::sqrt( t.vertex[0]*t.vertex[0] + t.vertex[1]*t.vertex[1] + t.vertex[2]*t.vertex[2]);
        // 2D scan
        if ( point[2] > -0.30  && point[2] < 0.20) { // TODO FIXME
          if(range > 0.5) mapset(point[0], point[1]); 
        } 

        bool del = true;
        double theta = 0.0;

        //publish 3D pointcloud
        if(STATE > 1) {

          if(STATE == 3  || pointcount == SAMPLE) {
            //if scanner sees robot
            if(range > 0.50) {
              // phi = atan2(t.vertex[1],t.vertex[0]);
              // phi = ((phi<0.0)?(phi+2.0*pi):phi);
              theta = std::acos(t.vertex[2]/range);
            }

            if(theta > 1.0 && theta < 1.95) {
              points.push_back(point);
              del = false;
            }
            pointcount = 0;
          } 
        }
        if(del) {
          delete[] point;
        }
      }
    }

   /**
    * Prepare point cloud message
    */ 
    void prepareCloud() {

      if(STATE > 1) {
        pointMsg.points.resize(points.size()); 
        pointMsg.channels[0].values.resize(points.size());

        int i = 0;
        for(i = 0; i < points.size(); ) {
          Point32 tmp;
          double *point = points[i];
          tmp.x = point[0];
          tmp.y = point[1];
          tmp.z = point[2];

          pointMsg.points[i] = tmp; 
          pointMsg.channels[0].values[i] = point[3];

          delete[] point;
          i++; 
        }
        points.clear();
      }
    }

    void on_frame_start_up(const frame_start_up<iterator_type>& arg) {
      basic_packets::on_frame_start_up(arg);
      riegl::RieglStatus rs;
      rs.header.stamp = ros::Time::now();
      rs.header.frame_id = "riegl";
      rs.status = 1; 
      status.publish(rs);
      resetMap();
    }
    void on_frame_start_dn(const frame_start_dn<iterator_type>& arg) {
      basic_packets::on_frame_start_dn(arg);
      riegl::RieglStatus rs;
      rs.header.stamp = ros::Time::now();
      rs.header.frame_id = "riegl";
      rs.status = 1; 
      status.publish(rs);
      resetMap();
    }

    void on_frame_stop(const frame_stop<iterator_type>& arg) {
      basic_packets::on_frame_stop(arg);
       
      riegl::RieglStatus rs;
      rs.header.stamp = ros::Time::now();
      rs.header.frame_id = "riegl";
      rs.status = 2; 
      status.publish(rs);
      
      // fully interpreting the points ...
      if (STATE != 0) {
        prepareCloud();
        sendMap();
        points.clear();
        //STATE = 1;
      }
    }

    // overridden from basic_packets
    // this function gets called when a the scanner emits a notification
    // about an exceptional state.
    void on_unsolicited_message(const unsolicited_message<iterator_type>& arg) {
      pointcloud::on_unsolicited_message(arg);
      // in this example we just print a warning to stderr
      cerr << "WARNING: " << arg.message << endl;
      // the following line would put out the entire content of the packet
      // converted to ASCII format:
      // cerr << arg << endl;
    }
};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "RIEGL_sensor");
  ros::NodeHandle n;
  string ip;
  n.param<std::string>("/riegl/ip", ip, "192.168.0.125");

  try {


    // The basic_rconnection class contains the communication
    // protocol between the scanner or file and the program.
    std::shared_ptr<basic_rconnection> rc;

    // The static create function analyses the argument and
    // gives back a suitable class that is derived from
    // basic_rconnection and can handle the requested protocol.
    // The argument string is modelled using the common 'uri'
    // syntax, i.e. a protocol specifier such as 'file:' or
    // 'rdtp:" is followed by the 'resource location'.
    // E.g. 'file:C:/scans/test.rxp' would specify a file that
    // is stored on drive C in the scans subdirectory.
    rc = basic_rconnection::create("rdtp://" + ip +"/current");
    rc->open();

    // The decoder class scans off distinct packets from the
    // continuous data stream i.e. the rxp format and manages
    // the packets in a buffer.
    decoder_rxpmarker dec(rc);

    // The importer ( based on pointcloud and basic_packets class)
    // recognizes the packet types and calls into a distinct
    // function for each type. The functions are overidable
    // virtual functions, so a derived class can get a callback
    // per packet type.
    importer     imp;
    /*
       ros::Rate loop_rate(10);
       while (!imp.done() && ros::ok()) {
       ros::spinOnce();
       loop_rate.sleep();
       }
     */

    // The buffer, despite its name is a structure that holds
    // pointers into the decoder buffer thereby avoiding
    // unnecessary copies of the data.
    buffer       buf;

    // This is the main loop, alternately fetching data from
    // the buffer and handing it over to the packets recognizer.
    // Please note, that there is no copy overhead for packets
    // which you do not need, since they will never be touched
    // if you do not access them.
    for ( dec.get(buf); !dec.eoi() && ros::ok(); dec.get(buf) ) {
      imp.dispatch(buf.begin(), buf.end());
      ros::spinOnce();

    }

    cout <<  "done with dispatching!!!" << endl;

    rc->close();
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
