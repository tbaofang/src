#ifndef LGlidar_H
#define LGlidar_H

#include <mrpt/utils/circular_buffer.h>
//#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt {
namespace hwdrivers {
/** 
 */
class LGlidar : public C2DRangeFinderAbstract {
  DEFINE_GENERIC_SENSOR(LGlidar) //I don't what is this? include spme iplementtion of virtual function
public:
  /** Used in CHokuyoURG::displayVersionInfo */
  struct TSensorInfo {
    /** The sensor model */
    std::string model;
    /** Min/Max ranges, in meters. */
    double d_min{0}, d_max{0};
    /** Number of measuremens per 360 degrees. */
    int scans_per_360deg{0};
    /** First, last, and front step of the scanner angular span. */
    int scan_first{0}, scan_last{0}, scan_front{0};
    /** Standard motor speed, rpm. */
    int motor_speed_rpm{0};
  };

private:
  /** The first and last ranges to consider from the scan. */
  //int m_firstRange, m_lastRange;
  /** The motor speed (default=600rpm) */
  int m_motorSpeed_rpm;
  /** The sensor 6D pose: */
  poses::CPose3D m_sensorPose;
  /** Auxiliary buffer for readings */
  mrpt::utils::circular_buffer<uint8_t> m_rx_buffer;

  std::vector<long> data;


  //check the first char of 42data ?
  bool isFirstSync(const char ch);
  //check the second char of 42data ?
  bool isSecondAngle(const char ch);
  //get the 42 data of a frame data add by cwy  20180604
  int readline(char* buf, const size_t count);
  //change by cwy,20180529,is ok!
  bool checkSum(const char* buffer, int size, char actual_sum);
  //add the bufdata to the m_rcv_data
  bool addLengthData(std::vector<long>& data, char *line, mrpt::obs::CObservation2DRangeScan &outObservation);

  /** Switchs the laser on.
   * \return false on any error
   */
  bool switchLaserOn();

  /** Switchs the laser off
   * \return false on any error
   */
  bool switchLaserOff();


  /** Turns the laser on */
  void initialize();




  /** ,
   * put distance data input data(vector)(360)
   * \
   */
  bool receiveResponse(mrpt::obs::CObservation2DRangeScan &outObservation);

  /** if a minimum number of bytes in the input buffer, return true
   * else reading from the serial port only if required.
   * 
   * \return false if the number of bytes are not available, even after
   * trying to fetch more data from the serial port.
   */
  bool assureBufferHasBytes(const size_t nDesiredBytes);

public:
  /** Constructor
   */
  LGlidar();

  /** Destructor: turns the laser off */
  //virtual ~LGlidar();
  ~LGlidar();

  //implement the virtual function
  void  GetRuntimeClass() { return ;}


  /** Specific laser scanner "software drivers" must process here new data
   * from the I/O stream, and, if a whole scan has arrived, return it.
   *  This method will be typically called in a different thread than other
   * methods, and will be called in a timely fashion.
   */
  void doProcessSimple(bool &outThereIsObservation,
                       mrpt::obs::CObservation2DRangeScan &outObservation,
                       bool &hardwareError);

  /** Enables the scanning mode (which may depend on the specific laser
   * device); this must be called before asking for observations to assure
   * that the protocol has been initializated.
   * \return If everything works "true", or "false" if there is any error.
   */
  bool turnOn();

  /** Disables the scanning mode (this can be used to turn the device in low
   * energy mode, if available)
   * \return If everything works "true", or "false" if there is any error.
   */
  bool turnOff();

  /** Empties the RX buffers of the serial port */
  void purgeBuffers();

  /** If set to non-empty, the serial port will be attempted to be opened
   * automatically when this class is first used to request data from the
   * laser.  */
  void setSerialPort(const std::string &port_name) { m_com_port = port_name; }
  /** Set the ip direction and port to connect using Ethernet communication */
  void setIPandPort(const std::string &ip, const unsigned int &port) {
    m_ip_dir = ip;
    m_port_dir = port;
  }

  /** Returns the currently set serial port \sa setSerialPort */
  const std::string getSerialPort() { return m_com_port; }
  /** If called (before calling "turnOn"), the field of view of the laser is
   * reduced to the given range (in radians), discarding the rest of measures.
   *  Call with "0" to disable this reduction again (the default).
   */
  void setReducedFOV(const double fov) { m_reduced_fov = fov; }
  /** Changes the high sensitivity mode (HS) (default: false)
   * \return false on any error
   */
  //bool setHighSensitivityMode(bool enabled);

  /** If true scans will capture intensity. (default: false)
   * Should not be called while scanning.
   * \return false on any error
   */
  bool setIntensityMode(bool enabled);

  /** Set the skip scan count (0 means send all scans).
   * Must be set before initialize()
   */
  void setScanInterval(unsigned int skipScanCount);
  unsigned int getScanInterval() const;

  void sendCmd(const char *str);

protected:
  /** temp buffer for incoming data packets */
  std::string m_rcv_data;

  /** Returns true if there is a valid stream bound to the laser scanner,
   * otherwise it first try to open the serial port "m_com_port"
   */
  bool ensureStreamIsOpen();

  /** Used to reduce artificially the interval of scan ranges. */
  double m_reduced_fov;

  /** If set to non-empty, the serial port will be attempted to be opened
   * automatically when this class is first used to request data from the
   * laser. */
  std::string m_com_port;

  /** If set to non-empty and m_port_dir too, the program will try to connect
   * to a Hokuyo using Ethernet communication */
  std::string m_ip_dir;
  /** If set to non-empty and m_ip_dir too, the program will try to connect to
   * a Hokuyo using Ethernet communication */
  unsigned int m_port_dir;

  /** The information gathered when the laser is first open */
  TSensorInfo m_sensor_info;

  bool m_I_am_owner_serial_port;

  /** Time of the first data packet, for synchronization purposes. */
  uint32_t m_timeStartUI;
  /** Counter to discard to first few packets before setting the
   * correspondence between device and computer timestamps. */
  int m_timeStartSynchDelay;
  mrpt::system::TTimeStamp m_timeStartTT;
  bool m_disable_firmware_timestamp;
  /** Get intensity from lidar scan (default: false) */
  bool m_intensity;
  unsigned int m_scan_interval;

  /** See the class documentation at the top for expected parameters */
  void
  loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource,
                            const std::string &iniSection);

}; // End of class

} // namespace hwdrivers

} // namespace mrpt

#endif
