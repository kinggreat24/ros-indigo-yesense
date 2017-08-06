#include "yesense.h"
#include <QByteArray>
#include <iostream>

namespace yesense {

Yesense::Yesense()
    :m_serialPort(NULL)
    ,m_badurate(0)
    ,m_yesensePort(std::string(""))
    ,m_packLength(127)
    ,m_yesenseCallbak(NULL){

}

 Yesense::Yesense(const std::string port,const int baudrate)
    :m_serialPort(NULL)
    ,m_badurate(baudrate)
    ,m_yesensePort(port)
    ,m_packLength(127)
    ,m_yesenseCallbak(NULL){

 }

 Yesense::Yesense(const std::string port,const int baudrate,OnYesenseCallback yesenseCallback)
     :m_serialPort(NULL)
     ,m_badurate(baudrate)
     ,m_yesensePort(port)
     ,m_packLength(127)
     ,m_yesenseCallbak(yesenseCallback){

 }

 Yesense::~Yesense(){}

 int Yesense::setYesenseCallback(OnYesenseCallback yesenseCallback){
     m_yesenseCallbak = yesenseCallback;
     return 0;
 }

int Yesense::setYesenseErrorCallback(OnYesenseErrorCallback yesenseErrorCallback)
{
    m_yesenseErrorCallback = yesenseErrorCallback;
    return 0;
}

 int Yesense::setBaudrate(const int baudrate){
     m_badurate = baudrate;
     return 0;
 }
 int Yesense::getBaudrate()const{
     return m_badurate;
 }

 int Yesense::setYesensePort(const std::string port){
     m_yesensePort = port;
     return 0;
 }
 std::string Yesense::getYesensePort()const{
     return m_yesensePort;
 }

 int Yesense::open(){
    m_serialPort = new QSerialPort(QString::fromStdString((m_yesensePort)));

    m_serialPort->setBaudRate(m_badurate);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);
    m_serialPort->setReadBufferSize(460800);

    //if (m_serialPort->open(QIODevice::ReadWrite))
    if (m_serialPort->open(QIODevice::ReadWrite))
    {
        connect(m_serialPort, SIGNAL(readyRead()), this, SLOT(readable()));
        connect(m_serialPort, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(getErrorFromSerial(QSerialPort::SerialPortError)));
        return true;
    }
    else
    {
        QSerialPort::SerialPortError errorInformation = m_serialPort->error();
        return false;
    }
 }

 int Yesense::close(){
     if(m_serialPort){
            delete m_serialPort;
            m_serialPort = NULL;

     }
     return 0;
 }

 void Yesense::readable(){

     QByteArray array = m_serialPort->readAll();
     m_serialData.clear();

     serialize(array);
     emit messageReady(m_serialData);

    if(m_yesenseCallbak)
           m_yesenseCallbak(m_serialData);

 }
 void Yesense::getErrorFromSerial(QSerialPort::SerialPortError error)
 {
    if(m_yesenseErrorCallback)
        m_yesenseErrorCallback(error);

 }
 void Yesense::serialize(const QByteArray& arr)
 {
     m_Array.append(arr);
     int iRemoveIndex = 0;
     QVector<double> WData; //写数据
     unsigned char iHeader0, iHeader1, iHeader2, iLen, iCheckLen;
     unsigned char iHigh, iLow;
     unsigned char iFour, iThree, iTwo, iOne;
     int iOct;
     for (int i = 0; i < m_Array.size() - m_packLength + 1; ++i)
     {
         iHeader0 = m_Array.at(i);
         iHeader1 = m_Array.at(i + 1);
         iHeader2 = m_Array.at(i + 2);
         iLen = m_Array.at(i + 5);
         iCheckLen = m_Array.at(i + 126);
         if (iHeader0 == 0x59 && iHeader1 == 0x49 && iHeader2 == 0x53 && iLen == iCheckLen)
         {
             //时间
             //int timeStamp = int(m_Array.at(i + 4)<<8 | m_Array.at(i + 3));
             iHigh = m_Array.at(i + 4);
             iLow = m_Array.at(i + 3);
             iOct = (iHigh << 8 | iLow);
             //m_Tid.append(iOct);
             WData.append(iOct);

             //0x10加速度
             if (m_Array.at(i + 6) == 0x10)
             {
                 QByteArray accVector = m_Array.mid(i + 8, 12);
                 int DATA;
                 iFour = accVector.at(3);
                 iThree = accVector.at(2);
                 iTwo = accVector.at(1);
                 iOne = accVector.at(0);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 //DATA = int((accVector.at(3) << 24)|(accVector.at(2) << 16) | (accVector.at(1) << 8)|accVector.at(0));
                 double ax = DATA*0.000001;
                 m_serialData.append(ax);
                 WData.append(ax);

                 iFour  = accVector.at(7);
                 iThree = accVector.at(6);
                 iTwo   = accVector.at(5);
                 iOne   = accVector.at(4);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 //DATA = int((accVector.at(7) << 24) | (accVector.at(6) << 16) | (accVector.at(5) << 8) | accVector.at(4));
                 double ay = DATA*0.000001;
                 m_serialData.append(ay);
                 WData.append(ay);

                 iFour  = accVector.at(11);
                 iThree = accVector.at(10);
                 iTwo   = accVector.at(9);
                 iOne   = accVector.at(8);
                 DATA   = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 //DATA = int((accVector.at(11) << 24) | (accVector.at(10) << 16) | (accVector.at(9) << 8) | accVector.at(8));
                 double az = DATA*0.000001;
                 m_serialData.append(az);
                 WData.append(az);
             }
             //0x11自由加速度
             if (m_Array.at(i + 20) == 0x11)
             {
                 QByteArray freeAccVector = m_Array.mid(i + 22, 12);
                 int DATA;
                 iFour  = freeAccVector.at(3);
                 iThree = freeAccVector.at(2);
                 iTwo   = freeAccVector.at(1);
                 iOne   = freeAccVector.at(0);
                 DATA   = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 //DATA = int((freeAccVector.at(3) << 24) | (freeAccVector.at(2) << 16) | (freeAccVector.at(1) << 8) | freeAccVector.at(0));

                 double ax = DATA*0.000001;
                 m_serialData.append(ax);
                 WData.append(ax);

                 //DATA = int((freeAccVector.at(7) << 24) | (freeAccVector.at(6) << 16) | (freeAccVector.at(5) << 8) | freeAccVector.at(4));
                 iFour  = freeAccVector.at(7);
                 iThree = freeAccVector.at(6);
                 iTwo   = freeAccVector.at(5);
                 iOne   = freeAccVector.at(4);
                 DATA   = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double ay = DATA*0.000001;
                 m_serialData.append(ay);
                 WData.append(ay);

                 //DATA = int((freeAccVector.at(11) << 24) | (freeAccVector.at(10) << 16) | (freeAccVector.at(9) << 8) | freeAccVector.at(8));
                 iFour = freeAccVector.at(11);
                 iThree = freeAccVector.at(10);
                 iTwo = freeAccVector.at(9);
                 iOne = freeAccVector.at(8);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double az = DATA*0.000001;
                 m_serialData.append(az);
                 WData.append(az);
             }
             //0x11速度增量
             if (m_Array.at(i + 34) == 0x12)
             {
                 QByteArray speedBaisVector = m_Array.mid(i + 36, 12);
                 int DATA;
                 //DATA = int((speedBaisVector.at(3) << 24) | (speedBaisVector.at(2) << 16) | (speedBaisVector.at(1) << 8) | speedBaisVector.at(0));
                 iFour = speedBaisVector.at(3);
                 iThree = speedBaisVector.at(2);
                 iTwo = speedBaisVector.at(1);
                 iOne = speedBaisVector.at(0);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double ax = DATA*0.000001;
                 m_serialData.append(ax);
                 WData.append(ax);

                 //DATA = int((speedBaisVector.at(7) << 24) | (speedBaisVector.at(6) << 16) | (speedBaisVector.at(5) << 8) | speedBaisVector.at(4));
                 iFour = speedBaisVector.at(7);
                 iThree = speedBaisVector.at(6);
                 iTwo = speedBaisVector.at(5);
                 iOne = speedBaisVector.at(4);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double ay = DATA*0.000001;
                 m_serialData.append(ay);
                 WData.append(ay);

                 //DATA = int((speedBaisVector.at(11) << 24) | (speedBaisVector.at(10) << 16) | (speedBaisVector.at(9) << 8) | speedBaisVector.at(8));
                 iFour = speedBaisVector.at(11);
                 iThree = speedBaisVector.at(10);
                 iTwo = speedBaisVector.at(9);
                 iOne = speedBaisVector.at(8);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double az = DATA*0.000001;
                 m_serialData.append(az);
                 WData.append(az);
             }

             //0x20角速度
             if (m_Array.at(i + 48) == 0x20)
             {
                 QByteArray speedVector = m_Array.mid(i + 50, 12);
                 int DATA;
                 //DATA = int((speedVector.at(3) << 24) | (speedVector.at(2) << 16) | (speedVector.at(1) << 8) | speedVector.at(0));
                 iFour = speedVector.at(3);
                 iThree = speedVector.at(2);
                 iTwo = speedVector.at(1);
                 iOne = speedVector.at(0);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double wx = DATA*0.000001;
                 m_serialData.append(wx);
                 WData.append(wx);

                 //DATA = int((speedVector.at(7) << 24) | (speedVector.at(6) << 16) | (speedVector.at(5) << 8) | speedVector.at(4));
                 iFour = speedVector.at(7);
                 iThree = speedVector.at(6);
                 iTwo = speedVector.at(5);
                 iOne = speedVector.at(4);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double wy = DATA*0.000001;
                 m_serialData.append(wy);
                 WData.append(wy);

                 //DATA = int((speedVector.at(11) << 24) | (speedVector.at(10) << 16) | (speedVector.at(9) << 8) | speedVector.at(8));
                 iFour = speedVector.at(11);
                 iThree = speedVector.at(10);
                 iTwo = speedVector.at(9);
                 iOne = speedVector.at(8);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double wz = DATA*0.000001;
                 m_serialData.append(wz);
                 WData.append(wz);
             }

             //0x30磁场强度
             if (m_Array.at(i + 62) == 0x30)
             {
                 QByteArray magVector = m_Array.mid(i + 64, 12);
                 int DATA;
                 //DATA = int((magVector.at(3) << 24) | (magVector.at(2) << 16) | (magVector.at(1) << 8) | magVector.at(0));
                 iFour = magVector.at(3);
                 iThree = magVector.at(2);
                 iTwo = magVector.at(1);
                 iOne = magVector.at(0);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double mx = DATA*0.001;
                 m_serialData.append(mx);
                 WData.append(mx);

                 //DATA = int((magVector.at(7) << 24) | (magVector.at(6) << 16) | (magVector.at(5) << 8) | magVector.at(4));
                 iFour = magVector.at(7);
                 iThree = magVector.at(6);
                 iTwo = magVector.at(5);
                 iOne = magVector.at(4);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double my = DATA*0.001;
                 m_serialData.append(my);
                 WData.append(my);

                 //DATA = int((magVector.at(11) << 24) | (magVector.at(10) << 16) | (magVector.at(9) << 8) | magVector.at(8));
                 iFour = magVector.at(11);
                 iThree = magVector.at(10);
                 iTwo = magVector.at(9);
                 iOne = magVector.at(8);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double mz = DATA*0.001;
                 m_serialData.append(mz);
                 WData.append(mz);
             }

             //0x40欧拉角
             if (m_Array.at(i + 76) == 0x40)
             {
                 QByteArray eulaVector = m_Array.mid(i + 78, 12);
                 int DATA;
                 //DATA = int((eulaVector.at(3) << 24) | (eulaVector.at(2) << 16) | (eulaVector.at(1) << 8) | eulaVector.at(0));
                 iFour = eulaVector.at(3);
                 iThree = eulaVector.at(2);
                 iTwo = eulaVector.at(1);
                 iOne = eulaVector.at(0);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double dPitch = DATA*0.000001;
                 m_serialData.append(dPitch);
                 WData.append(dPitch);


                 //DATA = int((eulaVector.at(7) << 24) | (eulaVector.at(6) << 16) | (eulaVector.at(5) << 8) | eulaVector.at(4));
                 iFour = eulaVector.at(7);
                 iThree = eulaVector.at(6);
                 iTwo = eulaVector.at(5);
                 iOne = eulaVector.at(4);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double dRoll = DATA*0.000001;
                 m_serialData.append(dRoll);
                 WData.append(dRoll);

                 //DATA = int((eulaVector.at(11) << 24) | (eulaVector.at(10) << 16) | (eulaVector.at(9) << 8) | eulaVector.at(8));
                 iFour = eulaVector.at(11);
                 iThree = eulaVector.at(10);
                 iTwo = eulaVector.at(9);
                 iOne = eulaVector.at(8);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double dYaw = DATA*0.000001;
                 m_serialData.append(dYaw);
                 WData.append(dYaw);
             }

             //0x41四元数
             if (m_Array.at(i + 90) == 0x41)
             {
                 QByteArray qutaVector = m_Array.mid(i + 92, 16);
                 int DATA;
                 //DATA = int((qutaVector.at(3) << 24) | (qutaVector.at(2) << 16) | (qutaVector.at(1) << 8) | qutaVector.at(0));
                 iFour = qutaVector.at(3);
                 iThree = qutaVector.at(2);
                 iTwo = qutaVector.at(1);
                 iOne = qutaVector.at(0);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double q0 = DATA*0.000001;
                 m_serialData.append(q0);
                 WData.append(q0);

                 //DATA = int((qutaVector.at(7) << 24) | (qutaVector.at(6) << 16) | (qutaVector.at(5) << 8) | qutaVector.at(4));
                 iFour = qutaVector.at(7);
                 iThree = qutaVector.at(6);
                 iTwo = qutaVector.at(5);
                 iOne = qutaVector.at(4);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double q1 = DATA*0.000001;
                 m_serialData.append(q1);
                 WData.append(q1);

                 //DATA = int((qutaVector.at(11) << 24) | (qutaVector.at(10) << 16) | (qutaVector.at(9) << 8) | qutaVector.at(8));
                 iFour = qutaVector.at(11);
                 iThree = qutaVector.at(10);
                 iTwo = qutaVector.at(9);
                 iOne = qutaVector.at(8);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double q2 = DATA*0.000001;
                 m_serialData.append(q2);
                 WData.append(q2);

                 //DATA = int((qutaVector.at(15) << 24) | (qutaVector.at(14) << 16) | (qutaVector.at(13) << 8) | qutaVector.at(12));
                 iFour = qutaVector.at(15);
                 iThree = qutaVector.at(14);
                 iTwo = qutaVector.at(13);
                 iOne = qutaVector.at(12);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double q3 = DATA*0.000001;
                 m_serialData.append(q3);
                 WData.append(q3);

             }

             //0x42方位增量
             if (m_Array.at(i + 108) == 0x42)
             {
                 QByteArray qutaBias = m_Array.mid(i + 110, 16);
                 int DATA;
                 iFour = qutaBias.at(3);
                 iThree = qutaBias.at(2);
                 iTwo = qutaBias.at(1);
                 iOne = qutaBias.at(0);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 //DATA = int((qutaBias.at(3) << 24) | (qutaBias.at(2) << 16) | (qutaBias.at(1) << 8) | qutaBias.at(0));
                 double b0 = DATA*0.000001;
                 m_serialData.append(b0);
                 WData.append(b0);

                 //DATA = int((qutaBias.at(7) << 24) | (qutaBias.at(6) << 16) | (qutaBias.at(5) << 8) | qutaBias.at(4));
                 iFour = qutaBias.at(7);
                 iThree = qutaBias.at(6);
                 iTwo = qutaBias.at(5);
                 iOne = qutaBias.at(4);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double b1 = DATA*0.000001;
                 m_serialData.append(b1);
                 WData.append(b1);

                 //DATA = int((qutaBias.at(11) << 24) | (qutaBias.at(10) << 16) | (qutaBias.at(9) << 8) | qutaBias.at(8));
                 iFour = qutaBias.at(11);
                 iThree = qutaBias.at(10);
                 iTwo = qutaBias.at(9);
                 iOne = qutaBias.at(8);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double b2 = DATA*0.000001;
                 m_serialData.append(b2);
                 WData.append(b2);

                 //DATA = int((qutaBias.at(15) << 24) | (qutaBias.at(14) << 16) | (qutaBias.at(13) << 8) | qutaBias.at(12));
                 iFour = qutaBias.at(15);
                 iThree = qutaBias.at(14);
                 iTwo = qutaBias.at(13);
                 iOne = qutaBias.at(12);
                 DATA = int((iFour << 24) | (iThree << 16) | (iTwo << 8) | iOne);
                 double b3 = DATA*0.000001;
                 m_serialData.append(b3);
                 WData.append(b3);

                 i = i + 126;
                 iRemoveIndex = i + 1;

             }

         }
     }
     m_Array.remove(0, iRemoveIndex);
 }


}
