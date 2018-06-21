
//#include "hwdrivers-precomp.h" // Precompiled headers

#include <mrpt/utils/CClientTCPSocket.h>
#include "LGlidar.h"
#include <mrpt/hwdrivers/CSerialPort.h>
//#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // in library mrpt-maps
#include <mrpt/system/os.h>
#include <thread>
#include <chrono>

IMPLEMENTS_GENERIC_SENSOR(LGlidar, mrpt::hwdrivers)//implement of virtual funciton of basici class

using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace std;

const int MINIMUM_PACKETS_TO_SET_TIMESTAMP_REFERENCE = 10;

LGlidar::LGlidar()
    : m_motorSpeed_rpm(300),
      m_sensorPose(0, 0, 0), m_rx_buffer(40000),
      m_reduced_fov(0), m_com_port(""), m_port_dir(10940),
      m_I_am_owner_serial_port(false), m_timeStartUI(0),
      m_timeStartSynchDelay(0), m_disable_firmware_timestamp(false),
      m_intensity(false), m_scan_interval(0) {
  m_sensorLabel = "LGlidar";
  data.clear();
}

LGlidar::~LGlidar() {
  if (m_stream) {
    turnOff();

    if (m_I_am_owner_serial_port)
      delete m_stream;
    m_stream = nullptr;
  }
  //m_win.reset();
}

//调用该函数，返回一组激光数据，满360个前线程阻塞
void LGlidar::doProcessSimple(
    bool &outThereIsObservation,
    mrpt::obs::CObservation2DRangeScan &outObservation, bool &hardwareError) {
  	// cout<<"do process simple"<<endl;
    outThereIsObservation = false;
    hardwareError = false;

  // cout<< "ensureStreamIsOpen:"<<ensureStreamIsOpen()<<endl;
  if (!ensureStreamIsOpen()) {
    m_timeStartUI = 0;
    m_timeStartSynchDelay = 0;
    hardwareError = true;
    return;
  }

  // Wait for 360distance input data and observation:
  int nRanges = 360;


  // cout<<"receiveResponse:"<<receiveResponse(outObservation)<<endl;
  if (!receiveResponse(outObservation)) {//if not get 360 data 
    // No new data
    return;
  }

  // printf("===========\n");


  // And the scan ranges:
  outObservation.rightToLeft = false;

  outObservation.aperture = 2 * M_PI;

  outObservation.maxRange = 3.5;
  outObservation.stdError = 0.010f;
  outObservation.sensorPose = m_sensorPose;
  outObservation.sensorLabel = m_sensorLabel;

  outObservation.resizeScan(nRanges);

  if (m_intensity)
    outObservation.setScanHasIntensity(true);

  if (0 == data.size()){
    printf("error, don/t get the laser data\n");
  } 

  //std::reverse(data.begin(), data.end());//reverse

  for (int i = 0; i < nRanges; i++) {
  
    int range_mm = data[i];
    // printf("range_mm is %d\n", range_mm);
    outObservation.setScanRange(i, range_mm * 0.001f);
    // printf("outObservation.scan[%d] is %f\n", i, outObservation.scan[i]);
    outObservation.setScanRangeValidity(
        i,
        range_mm >= 100 && (outObservation.scan[i] <= outObservation.maxRange));

  }

  // Do filter:
  C2DRangeFinderAbstract::filterByExclusionAreas(outObservation);
  C2DRangeFinderAbstract::filterByExclusionAngles(outObservation);
  // Do show preview:
  C2DRangeFinderAbstract::processPreview(outObservation);

  outThereIsObservation = true;
}

bool LGlidar::receiveResponse(mrpt::obs::CObservation2DRangeScan &outObservation) {
  m_rcv_data.clear();
  data.clear();
  if (!ensureStreamIsOpen())
    return false;

  try {
    // Process response:

    int while_count = 0;
    char buffer[100];
    int line_size = 0;
	// cout<<"line_size:readline():"<<readline(buffer, 42)<<endl;  //-1
    while ((line_size = readline(buffer, 81)) > 0) {

      // cout<<"[line_size is] "<<line_size<<endl;
      
      // printf("..................................\n");
      // for (int i = 0; i < 81; ++i)
      // {
      //   printf(" %04x ", buffer[i]);
      // }
      // printf("\n");
      // printf("..................................\n");

      //check sum
      // if (while_count != 0) {
       
      //   if (! checkSum(buffer, line_size - 2, buffer[line_size - 1])) {
      //     cout<<"Checksum() is error!..."<<endl;
      //     continue;
      //   }
      // }

      //record the time of 0 while_count
      if(while_count == 0) outObservation.timestamp = mrpt::system::now();

      //put distance input data
      if (!addLengthData(data, buffer, outObservation)) return false;

      int data_size = data.size();
      // cout << "data_size:" << data_size << endl;

      if (data_size >= 360)
      {
        // cout<<"data_size = 360,could send to buffer"<<endl; 
        break;
      }
       ++while_count;
       //printf("\nwhile_count is %d\n", while_count);
    }
    if (line_size == -1) return false;
    return true;
  } catch (std::exception &e) {
    MRPT_LOG_ERROR_FMT("[Hokuyo] receiveResponse() Exception: %s", e.what());
    return false;
  } 
}

int LGlidar::readline(char* buf, const size_t count)
{
    // cout<<"[LGlidar::readline]:"<<"..."<<endl;
    size_t filled = 0;
    bool is_find_first = false;
    bool is_find_second = false;
    bool is_find_third = false;
    bool is_fill_FirstToThird = false;

    while (filled < count) {;
        char recv_ch;
		// cout<<"assureBufferHasBytes:"<<(!assureBufferHasBytes(1))<<endl;

        if (!assureBufferHasBytes(1)){
          cout<<"[LGlidar::assureBufferHasBytes]: is empty"<<"..."<<endl;
          //mrpt::system::sleep(10);
          return -1;
        }
        recv_ch = m_rx_buffer.pop();
        if (!is_find_first)
        {
            if (!isFirstSync(recv_ch))
            {
                continue; 
            }            
            buf[filled++] = recv_ch;     
            is_find_first = true; //ok,find 0xAA, set the tag true,and don't into here again
            continue;
        }

        if(is_find_first && !is_find_second){
          if(!isSecondSync(recv_ch)){
            is_find_first = false;
            filled --;
            continue;
          }
          buf[filled++] = recv_ch;
          is_find_second = true;
          continue;
        }
        
        buf[filled++] = recv_ch;
    }
    //cout<<"[qrk::readline]:filled"<<" is "<<filled<<endl;
    buf[filled + 1] = '\0';

    return static_cast<int>(filled);
    
}


bool LGlidar::addLengthData(std::vector<long>& data, char *line, mrpt::obs::CObservation2DRangeScan &outObservation)
  {
    //printf("[addLengthData]:..........\n");
    int data_size = data.size();
    if (data_size >= 359){
      // printf("the scanlaser data is full(360)!!!\n");
    }

    float zeroOffset;
    float startingAngle;
    long distance[22];
    long intensity[22];

    zeroOffset = (float)((line[9] & 0x00ff) * 256 +(line[10] & 0x00ff)) / 100;
    startingAngle = (float)(((line[11] & 0x00ff) * 256 + (line[12] & 0x00ff)) / 100 + zeroOffset);
    int n_diff = (int)startingAngle - data_size;
    if(n_diff < 0) n_diff += 360;
    // printf("zeroOffset:%f ;startingAngle: %f ;n_diff:%d \n", zeroOffset, startingAngle, n_diff);

    //caculate the six distance
    for (int i = 0; i < 22; i++)
    {
      intensity[i] = line[i * 3 +13] & 0x00ff;
      distance[i] = (line[i*3 + 14] & 0x00ff) * 256 + (line[i * 3 + 15] & 0x00ff);
    }

    // push n_diff zero of distance
    for (int i = 0; i < n_diff; ++i)
    {
      if (n_diff > 90) return false; //if the invalid points is too much ,this 360 data mabe ignore
      data.push_back(0.0);
    }

    //push distance to data
    for (int i = 0; i < 22; ++i)
    {
      data.push_back(distance[i]);    
    }

    return true;
  }








/*-------------------------------------------------------------
                                                loadConfig_sensorSpecific
-------------------------------------------------------------*/

void LGlidar::loadConfig_sensorSpecific(
    const mrpt::utils::CConfigFileBase &configSource,
    const std::string &iniSection) {
  printf("this is  LGlidar::loadConfig_sensorSpecific!\n");

  m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port);

  m_sensorPose.setFromValues(
      configSource.read_float(iniSection, "pose_x", 0),
      configSource.read_float(iniSection, "pose_y", 0),
      configSource.read_float(iniSection, "pose_z", 0),
      DEG2RAD(configSource.read_float(iniSection, "pose_yaw", 0)),
      DEG2RAD(configSource.read_float(iniSection, "pose_pitch", 0)),
      DEG2RAD(configSource.read_float(iniSection, "pose_roll", 0)));

  C2DRangeFinderAbstract::loadCommonParams(configSource, iniSection);
  return;
}

/*-------------------------------------------------------------
                                                turnOn
-------------------------------------------------------------*/
bool LGlidar::turnOn() {
  MRPT_START
	
  // Bound?
  if (!ensureStreamIsOpen())
    return false;


  CSerialPort *COM = dynamic_cast<CSerialPort *>(m_stream);

	// cout<<"COM != nullptr:"<<(COM != nullptr)<<endl;
  //connect to LG by usb 
  if (COM != nullptr) {
      // It is a COM:
    COM->setConfig(115200);
    COM->setTimeouts(100, 0, 200, 0, 50);
	
      // Assure the laser is off and quiet:
    switchLaserOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));//this thread sleep 10ms
    COM->purgeBuffers();
	
  }
	
  if (COM != nullptr) { 
      // Set 230400 baud rate: no use for LGlidar
      //setHighBaudrate();
    COM->setConfig(115200);
  }

  // Turn on the laser:
  if (!switchLaserOn())
    return false;

  // Set the motor speed:
  /*
  if (m_motorSpeed_rpm)
    if (!setMotorSpeed(m_motorSpeed_rpm))
      return false;
  */

  // Set HS mode:
  //setHighSensitivityMode(m_highSensMode);

  // Display sensor information:
  //if (!displaySensorInfo(&m_sensor_info))
  //  return false;

  // Set for scanning angles:
  //m_firstRange = m_sensor_info.scan_first;
  //m_lastRange = m_sensor_info.scan_last;
  //m_firstRange = 0;
  //m_lastRange = 359;

  // Artificially reduced FOV?
  /*
  if (m_reduced_fov > 0 && m_reduced_fov < 2 * M_PI) {
    int center = (m_lastRange + m_firstRange) >> 1;
    const int half_range =
        static_cast<int>((m_sensor_info.scans_per_360deg / 360) *
                         RAD2DEG(m_reduced_fov)) >>
        1;
    m_firstRange = center - half_range;
    m_lastRange = center + half_range;
    MRPT_LOG_INFO_STREAM("[HOKUYO::turnOn] Using reduced FOV: ranges ["
                         << m_firstRange << "-" << m_lastRange << "] for "
                         << RAD2DEG(m_reduced_fov) << " deg. FOV");
  }
  */
/*
  if (!displayVersionInfo()) {
    // return false; // It's not SO important
  }
  */

  // Start!
  /*
  if (!startScanningMode())
    return false;
  */
  return true;

  MRPT_END
}

/*-------------------------------------------------------------
                                                turnOff
-------------------------------------------------------------*/
bool LGlidar::turnOff() {
  // Turn off the laser:
  if (!switchLaserOff())
    return false;

  return true;
}
//thie is set 230400 to the laser service of Ho.. ,but this is not for LG lidar
/*
bool CHokuyoURG::setHighBaudrate() {
  //char rcv_status0, rcv_status1;
  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG(
      "[CHokuyoURG::setHighBaudrate] Changing baudrate to 230400...");

  // Send command:
  sendCmd("SS230400\x0A");


  MRPT_LOG_DEBUG("OK\n");
  return true;
}
*/
/*-------------------------------------------------------------
                                                assureBufferHasBytes
-------------------------------------------------------------*/
bool LGlidar::assureBufferHasBytes(const size_t nDesiredBytes) {
  ASSERT_(nDesiredBytes < m_rx_buffer.capacity());
  // cout<<"nDesiredBytes:"<<nDesiredBytes<<"\t"<<"m_rx_buffer.capacity()"<<m_rx_buffer.capacity()<<endl;
	
  // cout<<"[CHokuyoURG::assureBufferHasBytes]: m_rx_buffer.size() is "<<m_rx_buffer.size()<<endl;
  // cout<<"[CHokuyoURG::assureBufferHasBytes]: nDesiredBytes is "<<nDesiredBytes<<endl;

  if (m_rx_buffer.size() >= nDesiredBytes) {
    return true;
	
  } else {
    // Try to read more bytes:
    uint8_t buf[128];
    const size_t to_read = std::min(m_rx_buffer.available(), sizeof(buf));
    // cout<<"[CHokuyoURG::assureBufferHasBytes]: m_rx_buffer.available() is "<<m_rx_buffer.available()<<" to_read is "<<to_read<<endl;
    size_t nRead = 0;
    try {
      
      // cout<<"[CHokuyoURG::assureBufferHasBytes]: buf is "<<buf<<" to_read is "<<to_read<<endl;
      nRead = m_stream->ReadBuffer(buf, to_read);
      // cout<<" ...................................... "<<endl;
      //printf("Read buf\n");
      // printf("3nRead is %d\n",int(nRead));

      m_rx_buffer.push_many(buf, nRead);
    } catch (std::exception &) {
      printf("0 bytes read, nRead is \n");
    }
	  //  printf("laterly, assureBufferHasBytes is %d, nRead is %d\n",int(m_rx_buffer.size()),nRead);
    return (m_rx_buffer.size() >= nDesiredBytes);
  }
}



bool LGlidar::switchLaserOn() {
  //char rcv_status0, rcv_status1;

  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG("[CHokuyoURG::switchLaserOn] Switching laser ON...");

  // Send command:
  sendCmd("b");
  /*
  // Receive response:
  if (!receiveResponse(rcv_status0, rcv_status1)) {
    MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__ << ": Error in response");
    return false;
  }
  */

  MRPT_LOG_DEBUG("OK\n");
  return true;
}

bool LGlidar::switchLaserOff() {
  //char rcv_status0, rcv_status1;

  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG("[CHokuyoURG::switchLaserOff] Switching laser OFF...");
  // Send command:
  sendCmd("e");

  // Receive response:
  /*
  if (!receiveResponse(rcv_status0, rcv_status1)) {
    MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__ << ": Error in response");
    return false;
  }
  */
  MRPT_LOG_DEBUG("OK\n");
  return true;
}

void LGlidar::setScanInterval(unsigned int skipScanCount) {
  m_scan_interval = skipScanCount;
}
unsigned int LGlidar::getScanInterval() const { return m_scan_interval; }

//not need this function,LGlidar can not set the speed
/*
bool LGlidar::setMotorSpeed(int motoSpeed_rpm) {
  char rcv_status0, rcv_status1;
  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG_FMT("[CHokuyoURG::setMotorSpeed] Setting to %i rpm...",
                     motoSpeed_rpm);

  // Send command:
  int motorSpeedCode = (600 - motoSpeed_rpm) / 6;
  if (motorSpeedCode < 0 || motorSpeedCode > 10) {
    MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__
                          << " Motorspeed must be in the range 540-600 rpm");
    return false;
  }

  char cmd[20];
  os::sprintf(cmd, 20, "CR%02i\x0A", motorSpeedCode);
  sendCmd(cmd);



  MRPT_LOG_DEBUG("OK\n");
  return true;
}
*/
/*-------------------------------------------------------------
                                                setHighSensitivityMode
-------------------------------------------------------------*/
/*
bool LGlidar::setHighSensitivityMode(bool enabled) { //not need

  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG_FMT(
      "[CHokuyoURG::setHighSensitivityMode] Setting HS mode to: %s...",
      enabled ? "true" : "false");

  // Send command:
  char cmd[20];
  os::sprintf(cmd, 20, "HS%i\x0A", enabled ? 1 : 0);
  sendCmd(cmd);



  MRPT_LOG_DEBUG("OK\n");
  return true;
}
*/
/*-------------------------------------------------------------
                                                                                                setIntensityMode
-------------------------------------------------------------*/
bool LGlidar::setIntensityMode(bool enabled) {
  m_intensity = enabled;
  return true;
}
/*
bool CHokuyoURG::displayVersionInfo() {
  char rcv_status0, rcv_status1;
  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG("[CHokuyoURG::displayVersionInfo] Asking info...");

  // Send command:
  sendCmd("VV\x0A");

  // Receive response:
  if (!receiveResponse(rcv_status0, rcv_status1)) {
    MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__ << ": Error in response");
    return false;
  }

  MRPT_LOG_DEBUG("OK\n");

  // PRINT:
  for (auto &c : m_rcv_data)
    if (c == ';')
      c = '\n';
  m_rcv_data[m_rcv_data.size()] = '\0';

  if (!m_rcv_data.empty()) {
    MRPT_LOG_INFO_STREAM(
        "\n------------- HOKUYO Scanner: Version Information ------\n"
        << &m_rcv_data[0]
        << "\n"
           "-------------------------------------------------------\n\n");
  }
  return true;
}
*/
/*-------------------------------------------------------------
                                                displaySensorInfo
-------------------------------------------------------------*/
/*
bool LGlidar::displaySensorInfo(TSensorInfo *out_data) { //not need
  //char rcv_status0, rcv_status1;
  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG("[CHokuyoURG::displaySensorInfo] Asking for info...");

  // Send command:
  sendCmd("PP\x0A");

  // Receive response:
  if (!receiveResponse(rcv_status0, rcv_status1)) {
    MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__ << ": Error in response");
    return false;
  }
  MRPT_LOG_DEBUG("OK\n");

  // PRINT:
  for (auto &c : m_rcv_data)
    if (c == ';')
      c = '\n';
  m_rcv_data[m_rcv_data.size()] = '\0';

  if (!m_rcv_data.empty()) {
    MRPT_LOG_INFO_STREAM(
        "\n------------- HOKUYO Scanner: Product Information ------\n"
        << &m_rcv_data[0]
        << "\n"
           "-------------------------------------------------------\n\n");
  }

  // Parse the data:
  if (out_data) {
    const char *ptr;

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "DMAX:")))
      out_data->d_max = 0.001 * atoi(ptr + 5);
    else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "DMIN:")))
      out_data->d_min = 0.001 * atoi(ptr + 5);
    else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "ARES:")))
      out_data->scans_per_360deg = atoi(ptr + 5);
    else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "SCAN:")))
      out_data->motor_speed_rpm = atoi(ptr + 5);
    else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "AMIN:")))
      out_data->scan_first = atoi(ptr + 5);
    else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "AMAX:")))
      out_data->scan_last = atoi(ptr + 5);
    else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "AFRT:")))
      out_data->scan_front = atoi(ptr + 5);
    else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

    if (nullptr != (ptr = strstr(&m_rcv_data[0], "MODL:"))) {
      char aux[30];
      memcpy(aux, ptr + 5, 8);
      aux[8] = '\0';
      out_data->model = aux;
    } else
      MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");
  }

  return true;
}
*/

//for LGlidar,no mode,only send "b",then the laser-sense send data
/*
bool LGlidar::startScanningMode() { //not need
  //char rcv_status0, rcv_status1;
  if (!ensureStreamIsOpen())
    return false;

  MRPT_LOG_DEBUG("[CHokuyoURG::startScanningMode] Starting scanning mode...");

  // Send command:
  // 'M' 'D'
  // 'XXXX' (starting step)
  // 'XXXX' (end step)
  // 'XX' (cluster count)
  // 'X' (scan interval)
  // 'XX' (number of scans)
  char cmd[50];
  unsigned int scan_interval = m_scan_interval;
  if (scan_interval > 9)
    scan_interval = 9;
  os::sprintf(cmd, 50, "M%c%04u%04u01%u00\x0A", m_intensity ? 'E' : 'D',
              m_firstRange, m_lastRange, scan_interval);

  sendCmd(cmd);

  // Receive response:
  if (!receiveResponse(rcv_status0, rcv_status1)) {
    MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__ << ": Error in response");
    return false;
  }

  MRPT_LOG_DEBUG("OK\n");
  return true;
}
*/
bool LGlidar::ensureStreamIsOpen() {
  MRPT_START

	  // cout<<"m_stream:"<<m_stream<<endl;
  if (m_stream) {
    // Socket or USB connection?
	// cout<<"m_i_dir.empty :"<<(!m_ip_dir.empty() && m_port_dir)<<endl;
    if (!m_ip_dir.empty() && m_port_dir) {
      // Has the port been disconected (USB serial ports)??
      CClientTCPSocket *COM = dynamic_cast<CClientTCPSocket *>(m_stream);

      if (COM != nullptr) {
        if (COM->isConnected())
          return true;

        // It has been disconnected... try to reconnect:
        // MRPT_LOG_ERROR("[CHokuyoURG] Socket connection lost! trying to "
        //                "reconnect...");

        try {
          COM->connect(m_ip_dir, m_port_dir);
          // OK, reconfigure the laser:
          turnOn();
          return true;
        } catch (...) {
          // Not yet..
          return false;
        }
      } else {
        return true; // Assume OK
      }
    } else {
      // Has the port been disconected (USB serial ports)??
      CSerialPort *COM = dynamic_cast<CSerialPort *>(m_stream);
	  // cout<<"com :"<<(COM != nullptr)<<endl;
      if (COM != nullptr) {
		  // cout<<"com is open :"<<(COM->isOpen())<<endl;
        if (COM->isOpen())
          return true;

        // It has been disconnected... try to reconnect:
        MRPT_LOG_ERROR_STREAM(
            __CURRENT_FUNCTION_NAME__
            << ": Serial port connection lost! Trying to reconnect...");

        try {
          COM->open();
          // OK, reconfigure the laser:
          turnOn();
          return true;
        } catch (...) {
          // Not yet..
          return false;
        }
      } else {
        return true; // Assume OK
      }
    }
  } else {
    if (m_com_port.empty() && m_ip_dir.empty() && !m_port_dir) {
      THROW_EXCEPTION(
          "No stream bound to the laser nor COM serial port or ip and "
          "port provided in 'm_com_port','m_ip_dir' and 'm_port_dir'");
    }

    if (!m_ip_dir.empty()) {
      // Try to open the serial port:
      CClientTCPSocket *theCOM = new CClientTCPSocket();

      MRPT_LOG_INFO_STREAM(__CURRENT_FUNCTION_NAME__ << " Connecting to "
                                                     << m_ip_dir << ":"
                                                     << m_port_dir);
      theCOM->connect(m_ip_dir, m_port_dir);

      if (!theCOM->isConnected()) {
        MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__
                              << " Cannot connect with the server '"
                              << m_com_port << "'");
        delete theCOM;
        return false;
      }

      // Bind:
      bindIO(theCOM);

      m_I_am_owner_serial_port = true;
    } else {
      // Try to open the serial port:
      CSerialPort *theCOM = new CSerialPort(m_com_port, true);

      if (!theCOM->isOpen()) {
        MRPT_LOG_ERROR_STREAM(__CURRENT_FUNCTION_NAME__
                              << " Cannot open serial port '" << m_com_port
                              << "'");
        delete theCOM;
        return false;
      }

      // Bind:
      bindIO(theCOM);

      m_I_am_owner_serial_port = true;
    }

    return true;
  }
  MRPT_END
}

void LGlidar::initialize() {
  if (m_verbose)
    this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);

  if (!ensureStreamIsOpen())
    return;

  if (!turnOn()) {
    MRPT_LOG_ERROR("[Hokuyo] Error initializing HOKUYO scanner");
    return;
  }
}

void LGlidar::purgeBuffers() {
  if (!ensureStreamIsOpen())
    return;

  if (m_ip_dir.empty()) {
    CSerialPort *COM = dynamic_cast<CSerialPort *>(m_stream);
    if (COM != nullptr) {
      COM->purgeBuffers();
    }
  } else // Socket connection
  {
    CClientTCPSocket *COM = dynamic_cast<CClientTCPSocket *>(m_stream);

    size_t to_read = COM->getReadPendingBytes();

    if (to_read) {
      void *buf = malloc(sizeof(uint8_t) * to_read);

      size_t nRead = m_stream->ReadBuffer(buf, to_read);

      if (nRead != to_read)
        THROW_EXCEPTION(
            "Error in purge buffers: read and expected number of bytes "
            "are different.");

      free(buf);
    }
  }
}




bool LGlidar::isFirstSync(const char ch)
{
    int tag = ch;
    tag &= 0x00FF;

    //printf("first tag is %04x, and results is %d\n",tag, (tag == 0x00FA));
    return (tag == 0x00AA) ;
    
}

bool LGlidar::isSecondSync(const char ch)
{
    int tag = ch;
    tag &= 0x00FF;

    //printf("first tag is %04x, and results is %d\n",tag, (tag == 0x00FA));
    return (tag == 0x0000) ;
    
}

bool LGlidar::isThirdSync(const char ch)
{
    int tag = ch;
    tag &= 0x00FF;

    //printf("first tag is %04x, and results is %d\n",tag, (tag == 0x00FA));
    return (tag == 0x0049) ;
    
}

bool LGlidar::isSecondAngle(const char ch)
{
    int tag = ch;
    tag &= 0x00FF;

    //printf("second tag is %04x, and results is %d\n",tag, ((tag >= 0x00FA) && (tag <= 0x00DB)));
    return ((tag >= 0x00A0) && (tag <= 0x00DB));

}

//change by cwy,20180529,is ok!
bool LGlidar::checkSum(const char* buffer, int size, char actual_sum)
{
  const char* p = buffer;
  const char* last_p = p + size;

  char expected_sum = 0x00;
  while (p < last_p) {
    expected_sum += *p++;
  }
  
  expected_sum = 0xFF - expected_sum;
  return (expected_sum == actual_sum) ? true : false;
}

void LGlidar::sendCmd(const char *str) {
  MRPT_START
  ASSERT_(str != nullptr);//if point str is nullptr, error 
  ASSERT_(m_stream != nullptr);
  const size_t N = strlen(str);
  m_stream->WriteBuffer(str, N);

  cout<<"[sendcmd]:str is "<<str<<" "<<N<<endl;

  MRPT_LOG_DEBUG_STREAM("[Hokuyo] sendCmd(): `" << str << "`");

  //m_lastSentMeasCmd = std::string(str); // for echo verification
  //cout<<"[sendcmd]:m_lastSentMeasCmd is "<<m_lastSentMeasCmd<<endl;
  MRPT_END
}
