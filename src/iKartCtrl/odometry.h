/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/math/Math.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

#ifndef M_PI
#define M_PI 3.14159265
#endif

class Odometry
{
private:
    Property iKartCtrl_options;
    yarp::os::Semaphore   mutex;
 
    //encoder variables
    double              encA_offset;
    double              encB_offset;
    double              encC_offset;

    double              encA;
    double              encB;
    double              encC;

    //measured motor velocity
    double              velA;
    double              velB;
    double              velC;

    //estimated motor velocity
    double              velA_est;
    double              velB_est;
    double              velC_est;
    iCub::ctrl::AWLinEstimator      *encvel_estimator;

    //robot geometry
    double              geom_r;
    double              geom_L;

    //thread period
    double              period;

    yarp::sig::Vector enc;
    yarp::sig::Vector encv;

public:
    //estimated cartesian velocity
    double              vel_x;
    double              vel_y;
    double              vel_lin;
    double              vel_heading;
    double              vel_theta;

    //estimated odometer 
    double              traveled_distance;
    double              traveled_angle;

    //estimated cartesian pos
    double              odom_x;
    double              odom_y;
    double              odom_theta;

private:
    ResourceFinder                  &rf;
    PolyDriver                      *control_board_driver;
    BufferedPort<Bottle>            port_odometry;
    BufferedPort<Bottle>            port_odometer;

    string                          localName;
    IEncoders                       *ienc;

public:
    Odometry(unsigned int _period, ResourceFinder &_rf, Property options, PolyDriver* _driver); 
    ~Odometry();
    bool reset_odometry();
    bool open();
    void compute();
    void printStats();
    void close();

};

#endif