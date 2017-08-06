#include "yesense.h"
#include "baseyesense.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <QCoreApplication>


class YesenseCallback:public QCoreApplication, public yesense::BaseYesense{
public:
    YesenseCallback(int argc,char** argv);
    ~YesenseCallback(){}
    virtual int onYesnseCallback(const QVector<double>& data);
    virtual int onYesenseErrorCallback(const QSerialPort::SerialPortError error);
    int exec();
private:
    ros::NodeHandlePtr nh_;
    ros::Publisher pub_imu_;
    sensor_msgs::Imu imu_data_;
    int seq_;
};

YesenseCallback::YesenseCallback(int argc,char** argv)
    :QCoreApplication(argc, argv)
    ,seq_(0)
{
    ros::init(argc,argv,"yesense_node",ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle());
    pub_imu_ = nh_->advertise<sensor_msgs::Imu>("yesense",1);
}

int YesenseCallback::onYesnseCallback(const QVector<double>& data)
{
    imu_data_.header.seq = seq_;
    imu_data_.header.stamp = ros::Time::now();
    imu_data_.header.frame_id = "yesense";

    imu_data_.orientation.w = data.at(18);
    imu_data_.orientation.x = data.at(19);
    imu_data_.orientation.y = data.at(20);
    imu_data_.orientation.z = data.at(21);
    for(int i=0;i<9;i++)
        imu_data_.orientation_covariance[i] = 0.0;

    imu_data_.angular_velocity.x = data.at(9);
    imu_data_.angular_velocity.y = data.at(10);
    imu_data_.angular_velocity.z = data.at(11);
    for(int i=0;i<9;i++)
        imu_data_.angular_velocity_covariance[i] = 0.0;


    imu_data_.linear_acceleration.x = data.at(0);
    imu_data_.linear_acceleration.y = data.at(1);
    imu_data_.linear_acceleration.z = data.at(2);
    for(int i=0;i<9;i++)
        imu_data_.linear_acceleration_covariance[i] = 0.0;

    pub_imu_.publish(imu_data_);

    seq_ ++;
    if(seq_ > 65536)
           seq_ = 0;

}
int YesenseCallback::onYesenseErrorCallback(const QSerialPort::SerialPortError error)
{
    BaseYesense::onYesenseErrorCallback(error);
    return 0;
}

int YesenseCallback::exec()
{
	return QCoreApplication::exec();
}

int main(int argc,char**argv){

    YesenseCallback yesenseCallback(argc,argv);

    yesense::Yesense yesense_("/dev/ttyUSB0",460800,boost::bind(&YesenseCallback::onYesnseCallback,&yesenseCallback,_1));
    yesense_.setYesenseErrorCallback(boost::bind(&YesenseCallback::onYesenseErrorCallback,&yesenseCallback,_1));
    yesense_.open();
    
    yesenseCallback.exec();
    return 0;
}
