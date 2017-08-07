#ifndef YESENSE_H
#define YESENSE_H

#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <string>
#include <QSerialPort>
#include <QObject>
#include <QVector>
#include <QString>

class QByteArray;
class QTextStream;
class QFile;


namespace yesense {

class Yesense: public QObject,public boost::noncopyable
{
    Q_OBJECT
public:
    typedef boost::function<int(const QVector<double>&)> OnYesenseCallback;
    typedef boost::function<void(const QSerialPort::SerialPortError error_code)> OnYesenseErrorCallback;
    typedef boost::function<void(const std::string&,const int baudrate)> OnOpenYesenseSuccessCallback;

    Yesense();
    Yesense(const std::string port,const int baudrate);
    Yesense(const std::string port,const int baudrate,OnYesenseCallback yesenseCallback);
    ~Yesense();

    int setYesenseCallback(OnYesenseCallback yesenseCallback);
    int setYesenseErrorCallback(OnYesenseErrorCallback yesenseErrorCallback);
    int setYesenseOpenSuccessCallback(OnOpenYesenseSuccessCallback yesenseOpenSuccessCallback);


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
    void initParam();
    void serialize(const QByteArray& array);
    void exportData(const QVector<double>& WData);
	
private:
    QSerialPort *m_serialPort;
    int m_badurate;
    std::string m_yesensePort;
    QVector<double> m_serialData;
    QVector<double> MagCalibData;
    QVector<double> m_GyroData;
    QByteArray m_Array;
    int m_packLength;
    int m_second;

    bool m_isCalib;
    bool m_isGyro;
    bool m_isSaveData;
    
    bool m_isTimeStamp;
    bool m_isFileExist;

    bool m_isAcc;
    bool m_isFreeAcc;
    bool m_isSpdDif;
    bool m_isSpd;
    bool m_isMag;
    bool m_isEula;
    bool m_isQuar;
    bool m_isDirectDif;
  

    QString m_strFilePathName;
    QFile *m_fileHandler;
    QTextStream *m_write;

    OnYesenseCallback m_yesenseCallbak;
    OnYesenseErrorCallback m_yesenseErrorCallback;
    OnOpenYesenseSuccessCallback m_yesenseOpenSuccessCallback;
};

} //end of  yesense namespace





#endif // YESENSE_H
