#include "yesense.h"
#include "baseyesense.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <QCoreApplication>
#include <signal.h>

class YesenseApplication:public QCoreApplication, public yesense::BaseYesense{
public:
    YesenseApplication(int argc,char** argv);
    ~YesenseApplication(){}
    virtual int onYesnseCallback(const QVector<double>& data);
    virtual int onYesenseErrorCallback(const QSerialPort::SerialPortError error);
    virtual void onYesenseOpenSuccessCallback(const std::string& port, const int baudrate);
    int exec();
    void attachObject(yesense::Yesense& yesenseObject);
protected:
    static void onRosNodeShutdown(int);

private:
    ros::NodeHandlePtr nh_;
    ros::Publisher pub_imu_;
    unsigned int seq_;
    static yesense::Yesense *yesense_;
};

yesense::Yesense* YesenseApplication::yesense_ = NULL;

YesenseApplication::YesenseApplication(int argc,char** argv)
    :QCoreApplication(argc, argv)
    ,seq_(0)
{
    ros::init(argc,argv,"yesense_node",ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle());
    pub_imu_ = nh_->advertise<sensor_msgs::Imu>("yesense",1);


    //bool result = connect(this,SIGNAL(aboutToQuit()),this,SLOT(onRosNodeShutdown()));
    signal(SIGINT,&YesenseApplication::onRosNodeShutdown);
}

int YesenseApplication::onYesnseCallback(const QVector<double>& data)
{
    sensor_msgs::Imu imu_data_;
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
    if(seq_ >= 4294967295)
           seq_ = 0;

}
int YesenseApplication::onYesenseErrorCallback(const QSerialPort::SerialPortError error)
{
    BaseYesense::onYesenseErrorCallback(error);
    return 0;
}

void YesenseApplication::onYesenseOpenSuccessCallback(const std::string& port, const int baudrate)
{
	ROS_INFO("open device: %s:%d success",port.c_str(),baudrate);
}


int YesenseApplication::exec()
{
	return QCoreApplication::exec();
}


void YesenseApplication::attachObject(yesense::Yesense& yesenseObject)
{
	yesense_ = &yesenseObject;
}

void YesenseApplication::onRosNodeShutdown(int sig)
{
	ROS_INFO("close serial");
	//close serial
	yesense_->close();

	//close app
	QCoreApplication::exit();
}



int main(int argc,char**argv){

    YesenseApplication yesenseApp(argc,argv);

    yesense::Yesense yesense_("/dev/ttyUSB0",460800,boost::bind(&YesenseApplication::onYesnseCallback,&yesenseApp,_1));
    yesense_.setYesenseErrorCallback(boost::bind(&YesenseApplication::onYesenseErrorCallback,&yesenseApp,_1));
    yesense_.setYesenseOpenSuccessCallback(boost::bind(&YesenseApplication::onYesenseOpenSuccessCallback,&yesenseApp,_1,_2));
    yesenseApp.attachObject(yesense_);
    yesense_.open();
    
    yesenseApp.exec();
    return 0;
}
