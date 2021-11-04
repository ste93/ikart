
#include "IKartOdometry.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>


namespace {
    YARP_LOG_COMPONENT(IKARTODOM, "yarp.device.IKartOdometry")
}


IKartOdometry::IKartOdometry():
        yarp::os::PeriodicThread(m_period)
{
    encvel_estimator =new iCub::ctrl::AWLinEstimator(3,1.0);
    enc.resize(3);
    encv.resize(3);
    yCTrace(IKARTODOM);
}

IKartOdometry::~IKartOdometry()
{
    yCTrace(IKARTODOM);
    close();
}


bool IKartOdometry::close()
{
    yCTrace(IKARTODOM);
    detach();
    return true;
}

bool IKartOdometry::getOdometry(yarp::dev::OdometryData& odom)
{
    std::lock_guard lock(m_odometry_mutex);
    odom.odom_x = m_odometryData.odom_x;
    odom.odom_y = m_odometryData.odom_y;
    odom.odom_theta  = m_odometryData.odom_theta;
    odom.base_vel_x = m_odometryData.base_vel_x;
    odom.base_vel_y = m_odometryData.base_vel_y;
    odom.base_vel_theta = m_odometryData.base_vel_theta;
    odom.odom_vel_x  = m_odometryData.odom_vel_x;
    odom.odom_vel_y = m_odometryData.odom_vel_y;
    odom.odom_vel_theta = m_odometryData.odom_vel_theta;
    return true;
}

bool IKartOdometry::resetOdometry()
{
    if (ienc) {
        ienc->getEncoder(0, &encA_offset);
        ienc->getEncoder(1, &encB_offset);
        ienc->getEncoder(2, &encC_offset);
    }
    m_odometryData.odom_x=0;
    m_odometryData.odom_y=0;
    if (encvel_estimator) {
        encvel_estimator->reset();
    }
    yCInfo(IKARTODOM,"Odometry reset done");
    return true;
}


bool IKartOdometry::open(yarp::os::Searchable& config)
{

    // open the control board driver
    yCInfo(IKARTODOM,"Opening the motors interface...");
    if (!config.check("period"))
    {
        yCWarning(IKARTODOM, "using default period of 0.01");
    } else {
        m_period = config.find("period").asFloat64();
    }


    //reset odometry
    resetOdometry();

    //get robot geometry
    yarp::os::Bottle geometry_group = config.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yCError(IKARTODOM,"iKart_Odometry::open Unable to find ROBOT_GEOMETRY group!");
        return false;
    }
    if (!geometry_group.check("geom_r"))
    {
        yCError(IKARTODOM,"Missing param geom_r in [ROBOT_GEOMETRY] group");
        return false;
    }
    if (!geometry_group.check("geom_l"))
    {
        yCError(IKARTODOM,"Missing param geom_l in [ROBOT_GEOMETRY] group");
        return false;
    }
    if (!geometry_group.check("g_angle"))
    {
        yCError(IKARTODOM,"Missing param g_angle in [ROBOT_GEOMETRY] group");
        return false;
    }
    geom_r = geometry_group.find("geom_r").asFloat64();
    geom_L = geometry_group.find("geom_l").asFloat64();
    g_angle = geometry_group.find("g_angle").asFloat64();
    geometry_group.toString();

    yarp::os::PeriodicThread::setPeriod(m_period);
    return yarp::os::PeriodicThread::start();
}

bool IKartOdometry::attach(yarp::dev::PolyDriver *driver) {
    if (!driver->isValid())
    {
        yCError(IKARTODOM) << "not valid poly driver";
        return false;
    }
    if(!driver->view(ienc)){
        yCError(IKARTODOM) << "iencoder device has not been viewed";
        return false;
    }
    int axesNumber;
    ienc->getAxes(&axesNumber);
    if(axesNumber != 2){
        yCError(IKARTODOM) << "failed to get correct number of axes";
        return false;
    }
    return true;
}

bool IKartOdometry::detach() {
    ienc = nullptr;
    return true;
}

void IKartOdometry::compute()
{
    std::lock_guard lock(m_odometry_mutex);
    if (ienc) {

        //read the encoders (deg)
        ienc->getEncoder(0,&encA);
        ienc->getEncoder(1,&encB);
        ienc->getEncoder(2,&encC);

        //read the speeds (deg/s)
        ienc->getEncoderSpeed(0,&velA);
        ienc->getEncoderSpeed(1,&velB);
        ienc->getEncoderSpeed(2,&velC);

        //remove the offset and convert in radians
        enc[0]= -(encA - encA_offset) * 0.0174532925;
        enc[1]= -(encB - encB_offset) * 0.0174532925;
        enc[2]= -(encC - encC_offset) * 0.0174532925;

        //estimate the speeds
        iCub::ctrl::AWPolyElement el;
        el.data=enc;
        el.time=yarp::os::Time::now();
        encv= encvel_estimator->estimate(el);

        // -------------------------------------------------------------------------------------
        // The following formulas are adapted from:
        // "A New Odometry System to reduce asymmetric Errors for Omnidirectional Mobile Robots"
        // -------------------------------------------------------------------------------------

        //compute the orientation. odom_theta is expressed in radians
        m_odometryData.odom_theta = geom_r*(enc[0]+enc[1]+enc[2])/(3*geom_L);

        //build the kinematics matrix
        yarp::sig::Matrix kin;
        kin.resize(3,3);
        kin.zero();
        kin(0,0) = -sqrt(3.0)/2.0;
        kin(0,1) = 0.5;
        kin(0,2) = geom_L;
        kin(1,0) = sqrt(3.0)/2.0;
        kin(1,1) = 0.5;
        kin(1,2) = geom_L;
        kin(2,0) = 0;
        kin(2,1) = -1.0;
        kin(2,2) = geom_L;
        kin      = kin/geom_r;

        yarp::sig::Matrix m_gangle;
        m_gangle.resize(3,3);
        m_gangle.zero();
        m_gangle(0,0) = cos (g_angle);
        m_gangle(0,1) = -sin (g_angle);
        m_gangle(1,0) = sin (g_angle);
        m_gangle(1,1) = cos (g_angle);
        m_gangle(2,2) = 1;

        yarp::sig::Matrix ikin = yarp::math::luinv(kin);
        ikin=m_gangle*ikin;

        //build the rotation matrix
        yarp::sig::Matrix m1;
        m1.resize(3,3);
        m1.zero();
        m1(0,0) = cos (m_odometryData.odom_theta);
        m1(0,1) = -sin (m_odometryData.odom_theta);
        m1(1,0) = sin (m_odometryData.odom_theta);
        m1(1,1) = cos (m_odometryData.odom_theta);
        m1(2,2) = 1;

        yarp::sig::Matrix m2;
        m2.resize(3,3);
        m2.zero();
        m2(0,0) = cos (0.0);
        m2(0,1) = -sin (0.0);
        m2(1,0) = sin (0.0);
        m2(1,1) = cos (0.0);
        m2(2,2) = 1;

        yarp::sig::Vector odom_cart_vels;  //velocities expressed in the world reference frame
        yarp::sig::Vector ikart_cart_vels; //velocities expressed in the ikart reference frame
        odom_cart_vels  = m1*ikin*encv;
        ikart_cart_vels = m2*ikin*encv;

        m_odometryData.base_vel_x     = ikart_cart_vels[0];
        m_odometryData.base_vel_y     = ikart_cart_vels[1];
        m_odometryData.base_vel_theta = ikart_cart_vels[2];
        base_vel_lin   = sqrt(m_odometryData.odom_vel_x*m_odometryData.odom_vel_x + m_odometryData.odom_vel_y*m_odometryData.odom_vel_y);

        m_odometryData.odom_vel_x      = odom_cart_vels[0];
        m_odometryData.odom_vel_y      = odom_cart_vels[1];
        m_odometryData.odom_vel_theta  = odom_cart_vels[2];

        //these are not currently used
        if (base_vel_lin<0.001)
        {
            m_odometryData.odom_vel_theta = 0;
            m_odometryData.base_vel_theta = 0;
        }
        else
        {
            m_odometryData.odom_vel_theta  = atan2(m_odometryData.odom_vel_x,m_odometryData.odom_vel_y)*RAD2DEG;
            m_odometryData.base_vel_theta = atan2(m_odometryData.base_vel_x,m_odometryData.base_vel_y)*RAD2DEG;
        }
        //convert from radians back to degrees
        m_odometryData.odom_theta       *= RAD2DEG;
        traveled_angle   *= RAD2DEG;

        //the integration step
        double period=el.time-last_time;
        m_odometryData.odom_x= m_odometryData.odom_x + (m_odometryData.odom_vel_x * period);
        m_odometryData.odom_y= m_odometryData.odom_y + (m_odometryData.odom_vel_y * period);

        //compute traveled distance (odometer)
        traveled_distance = traveled_distance + fabs(base_vel_lin   * period);
        traveled_angle    = traveled_angle    + fabs(m_odometryData.base_vel_theta * period);
        
        last_time = yarp::os::Time::now();
    }
}

bool IKartOdometry::threadInit() {
    return true;
}

void IKartOdometry::threadRelease() {
    detach();
}

void IKartOdometry::run() {
    compute();
}

