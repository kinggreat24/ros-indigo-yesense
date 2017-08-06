#ifndef YESENSE_H
#define YESENSE_H

#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <string>
#include <QSerialPort>
#include <QObject>
#include <QVector>
class QByteArray;


namespace yesense {

class Yesense: public QObject,public boost::noncopyable
{
    Q_OBJECT
public:
    typedef boost::function<int(const QVector<double>&)> OnYesenseCallback;
    typedef boost::function<void(const QSerialPort::SerialPortError error_code)> OnYesenseErrorCallback;

    Yesense();
    Yesense(const std::string port,const int baudrate);
    Yesense(const std::string port,const int baudrate,OnYesenseCallback yesenseCallback);
    ~Yesense();

    int setYesenseCallback(OnYesenseCallback yesenseCallback);
    int setYesenseErrorCallback(OnYesenseErrorCallback yesenseErrorCallback);

    int setBaudrate(const int baudrate);
    int getBaudrate()const;

    int setYesensePort(const std::string port);
    std::string getYesensePort()const;

    int open();
    int close();

    int setYesensePortConfig();

protected Q_SLOTS:
    void readable();
    void getErrorFromSerial(QSerialPort::SerialPortError);
Q_SIGNALS:
    void messageReady(QVector<double> &);

private:
    void serialize(const QByteArray& array);
private:
    QSerialPort *m_serialPort;
    int m_badurate;
    std::string m_yesensePort;
    QVector<double> m_serialData;
    QByteArray m_Array;
    int m_packLength;


    OnYesenseCallback m_yesenseCallbak;
    OnYesenseErrorCallback m_yesenseErrorCallback;

};

} //end of  yesense namespace





#endif // YESENSE_H
