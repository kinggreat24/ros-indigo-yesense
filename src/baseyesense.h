#ifndef BASEYESENSE_H
#define BASEYESENSE_H
namespace yesense {
class BaseYesense{
public:
    virtual int onYesnseCallback(const QVector<double>& data) = 0;
    virtual int onYesenseErrorCallback(const QSerialPort::SerialPortError error){
        switch(error)
        {
           case QSerialPort::NoError:
                std::cout<<"open port success"<<std::endl;
                break;
           case QSerialPort::DeviceNotFoundError:
		std::cout<<"can not find the device, please check"<<std::endl;
                break;
           case QSerialPort::PermissionError:
		std::cout<<"another process or a user not having enough permission and credentials to open."<<std::endl;
                break;
           case QSerialPort::ResourceError:
                std::cout<<"An I/O error occurred when a resource becomes unavailable"<<std::endl;
                break;
           default:
                break;
        }
    }
   virtual void onYesenseOpenSuccessCallback(const std::string& port, const int baudrate) = 0;
};
}
#endif // BASEYESENSE_H
