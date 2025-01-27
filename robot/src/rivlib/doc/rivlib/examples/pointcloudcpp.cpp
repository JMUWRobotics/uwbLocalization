// $Id: pointcloudcpp.cpp 835 2013-06-03 07:21:40Z RS $

// (c) Riegl 2008-2013
// pointcloudcpp.cpp - Example for reading pointcloud data
// from Riegl's *.rxp format.
//
// NOTE: rivilib expects a working C++ 11 setup!
// This example uses the RiVLib as a statically linked C++ library.
//
// Compile instructions:
//   Please read the instructions in CmakeLists.txt
//
// Usage instructions:
// Invoke the program as:
//   pointcloudcpp <uri>
//   where uri is e.g. 'file:../scan.rxp' when reading from a file or
//   'rdtp://ip-addr/current' when reading real-time data from the scanner
//   (replace ip-addr with the ip-addr of your scanner).
// The program will read the pointcloud data, filter out single and first targets
// and print them out to the console in ASCII.
//

#include <riegl/scanlib.hpp>

#include <iostream>
#include <exception>
#include <cmath>
#include <limits>
#include <memory>

using namespace scanlib;
using namespace std;

// The import class is derived from pointcloud class, which assembles the
// scanner data into distinct targets and computes x,y,z pointcloud data.
// The pointcloud class and its base class have a huge number of overridables
// that give access to all of the scanners data, e.g. "gps" or "housekeeping"
// data. The pointcloud class also has the necessary logic to align the
// data to gps information (if embedded in the rxp stream) and will return
// timestamps in the domain of the gps.

class importer
    : public pointcloud
{
    ostream& o;
    unsigned long line;
public:
    importer(ostream& o_)
        : pointcloud(false) // set this to true if you need gps aligned timing
        , o(o_)
        , line(0)
    {
        o.precision(10);
    }

protected:

    // overridden from pointcloud class
    void on_echo_transformed(echo_type echo)
    {
        // here we select which target types we are interested in
        if ( pointcloud::first == echo || pointcloud::single == echo) {
            // targets is a member std::vector that contains all
            // echoes seen so far, i.e. the current echo is always
            // indexed by target_count-1.
            target& t(targets[target_count-1]);

            // transform to polar coordinates
            double range = std::sqrt(
                t.vertex[0]*t.vertex[0]
                + t.vertex[1]*t.vertex[1]
                + t.vertex[2]*t.vertex[2]
            );
            if (range > numeric_limits<double>::epsilon()) {
                double phi = atan2(t.vertex[1],t.vertex[0]);
                phi = ((phi<0.0)?(phi+2.0*pi):phi);
                double theta = std::acos(t.vertex[2]/range);
                t.vertex[0] = static_cast<float>(range);
                t.vertex[1] = static_cast<float>((360.0/(2.0*pi))*theta);
                t.vertex[2] = static_cast<float>((360.0/(2.0*pi))*phi);
            }
            else
                t.vertex[0] = t.vertex[1] = t.vertex[2] = 0.0;

            // print out the result
            o << t.vertex[0] << ", " << t.vertex[1] << ", " << t.vertex[2]
              << ", " << t.time << endl;

        }
    }

    // overridden from basic_packets
    // this function gets called when a the scanner emits a notification
    // about an exceptional state.
    void on_unsolicited_message(const unsolicited_message<iterator_type>& arg) {
        pointcloud::on_unsolicited_message(arg);
        // in this example we just print a warning to stderr
        cerr << "MESSAGE: " << arg.message << endl;
        // the following line would put out the entire content of the packet
        // converted to ASCII format:
        // cerr << arg << endl;
    }

    void on_line_start_up(const line_start_up<iterator_type>& arg) {
        pointcloud::on_line_start_up(arg);
        o << "line: " << ++line << endl;
    }

    void on_line_start_dn(const line_start_dn<iterator_type>& arg) {
        pointcloud::on_line_start_dn(arg);
        o << "line: " << ++line << endl;
    }

    // The following line shows the override for end of line
    //void on_line_sop(const line_stop<iterator_type>& arg) {
    //    pointcloud::on_line_stop(arg);
    //}

};

int main(int argc, char* argv[])
{
    try {

        if (argc == 2) {

            // The basic_rconnection class contains the communication
            // protocol between the scanner or file and the program.
            shared_ptr<basic_rconnection> rc;

            // The static create function analyses the argument and
            // gives back a suitable class that is derived from
            // basic_rconnection and can handle the requested protocol.
            // The argument string is modelled using the common 'uri'
            // syntax, i.e. a protocol specifier such as 'file:' or
            // 'rdtp:" is followed by the 'resource location'.
            // E.g. 'file:C:/scans/test.rxp' would specify a file that
            // is stored on drive C in the scans subdirectory.
            rc = basic_rconnection::create(argv[1]);
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
            importer     imp(cout);

            // The buffer, despite its name is a structure that holds
            // pointers into the decoder buffer thereby avoiding
            // unnecessary copies of the data.
            buffer       buf;

            // This is the main loop, alternately fetching data from
            // the buffer and handing it over to the packets recognizer.
            // Please note, that there is no copy overhead for packets
            // which you do not need, since they will never be touched
            // if you do not access them.
            for ( dec.get(buf); !dec.eoi(); dec.get(buf) ) {
                imp.dispatch(buf.begin(), buf.end());
            }

            rc->close();
            return 0;
        }

        cerr << "Usage: " << argv[0] << "<input-uri>" << endl;
        return 1;
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
