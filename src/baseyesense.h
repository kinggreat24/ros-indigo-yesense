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
                break;
           case QSerialPort::PermissionError:
                break;
           case QSerialPort::ResourceError:
                std::cout<<"An I/O error occurred when a resource becomes unavailable"<<std::endl;
                break;
           default:
                break;
        }
    }
};
}
#endif // BASEYESENSE_H
