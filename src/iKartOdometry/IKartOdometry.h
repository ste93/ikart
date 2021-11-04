//
// Created by ste on 21/10/21.
//

#ifndef NAVIGATION_IKARTODOMETRY_H
#define NAVIGATION_IKARTODOMETRY_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/OdometryData.h>
#include <yarp/dev/IOdometry2D.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/WrapperSingle.h>

constexpr double default_period = 0.02;

#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

/**
 * @ingroup dev dev_impl_navigation
 *
 * \section iKartOdometry_parameters Device description
 * \brief `iKartOdometry`: A device for generating a cer odometry.
 * This device will generate the odometry and then the user can retrieve it by calling `getOdometry()`.
 *
 *   Parameters required by this device are:
 * | Parameter name | SubParameter            | Type    | Units          | Default Value | Required                       | Description                                                                      | Notes|
 * |:--------------:|:-----------------------:|:-------:|:--------------:|:-------------:|:-----------------------------: |:--------------------------------------------------------------------------------:|:-----|
 * | period         |      -                  | double  | s              |   0.01        | No                             | refresh period of the broadcasted values in s           | default 0.01s |
 * | geom_r         |      -                  | double  | s              |   -           | Yes                            | right geometry of the robot                                                      |      |
 * | geom_l         |      -                  | double  | s              |   -           | Yes                            | left geometry of the robot                                                       |      |
 *
 * Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device iKartOdometry
 * period 0.01
 * geom_r 10
 * geom_l 10
 * g_angle 10
 * \endcode
 *
 * example of xml file
 *
 * \code{.unparsed}
 * <?xml version="1.0" encoding="UTF-8" ?>
 * <!DOCTYPE robot PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">
 * <robot name="iKartOdometry" build="2" portprefix="test" xmlns:xi="http://www.w3.org/2001/XInclude">
 *   <devices>
 *     <device xmlns:xi="http://www.w3.org/2001/XInclude" name="iKartOdometry_device" type="iKartOdometry">
 *       <param name="period">0.01</param>
 *       <param name="geom_r">10.2</param>
 *       <param name="geom_l">10.2</param>
 *       <param name="g_angle">10.2</param>
 *     </device>
 *   </devices>
 * </robot>
 * \endcode
 *
 * example of command via terminal.
 *
 * \code{.unparsed}
 * yarpdev --device iKartOdometry --geom_l 10 --geom_r 10 --g_angle 10 --period 0.01
 * \endcode
 */

class IKartOdometry :
        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public yarp::dev::Nav2D::IOdometry2D,
        public yarp::dev::WrapperSingle
{
public:
    IKartOdometry();
    ~IKartOdometry() override;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IOdometry2D
    bool   getOdometry(yarp::dev::OdometryData& odom) override;
    bool   resetOdometry() override;

    // WrapperSingle
    bool attach(yarp::dev::PolyDriver *driver) override;
    bool detach() override;
    // auxiliar
    void compute();

protected:
    bool threadInit() override;

    void threadRelease() override;

    void run() override;

private:
    yarp::dev::OdometryData m_odometryData;
    double base_vel_lin{0.0};

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

    iCub::ctrl::AWLinEstimator      *encvel_estimator;

    //robot geometry
    double              geom_r;
    double              geom_L;
    double              g_angle;

    yarp::sig::Vector enc;
    yarp::sig::Vector encv;

    std::mutex m_odometry_mutex;
    double last_time;
    double m_period{0.01};


    // estimated odom
    double              traveled_distance{0.0};
    double              traveled_angle{0.0};

    //motor control interfaces
    yarp::dev::IEncoders                       *ienc{nullptr};

};
#endif //NAVIGATION_IKARTODOMETRY_H
