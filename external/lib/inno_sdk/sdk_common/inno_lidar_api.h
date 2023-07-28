/*
 *  Copyright (C) 2021 Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Innovusion LIDAR SDK Header File
 *
 *  The file provides the data structures definition and exported functions of
 *  Innovusion LIDAR SDK.
 */

/**
 *  Simple 7-step introduction describing how to use the SDK to write
 *  an Innovusion Lidar driver.
 *
 *  step 1a: (OPTIONAL)
 *    Use inno_lidar_setup_sig_handler() to setup handling of SIGSEGV, SIGBUS,
 *        and SIGFPE signals.
 * 
 *  step 1b: (OPTIONAL)
 *    Use inno_lidar_set_logs() to specify the log file fds (file descriptors)
 *        and callback.
 *    Use inno_lidar_set_log_level() to specify the log level.
 *
 *  step 2: (REQUIRED)
 *    Use inno_lidar_open_live() to connect to a sensor.
 *    Or use inno_lidar_open_file() to open a raw data file.
 *    Both methods return a lidar handle.
 *

 *  step 3: (REQUIRED)
 *    Use inno_lidar_set_callbacks() to specify the lidar_message and lidar_packet
 *        callbacks.
 *    Use the lidar_message callback to handle warning, error, and critical-error
 *        notifications that might occur.
 *    The lidar_packet callback provides access to the lidar pointcloud data.
 *
 *  step 4A: (REQUIRED)
 *    Use inno_lidar_start() to start reading from live lidar or file.
 *    Corresponding callbacks will be triggered.  The callbacks are called in
 *    the context of a thread dedicated to each lidar handle, i.e. one callback
 *    queue per lidar handle.  Please note that multiple callbacks may be
 *    called AT THE SAME TIME for different lidar handles.
 *
 *  step 5: (REQUIRED)
 *    Use inno_lidar_stop() to stop reading.  No further callbacks will be
 *    called once this call returns.
 *
 *  step 6: (REQUIRED)
 *    Use inno_lidar_close() to close a lidar handle and release any
 *    associated resources.
 * 
 *  Threading:
 *    In inno_lidar_start(), the SDK library will use pthread library to spawn
 *    threads to read and process data and make callbacks.
 */


#ifndef SDK_COMMON_INNO_LIDAR_API_H_
#define SDK_COMMON_INNO_LIDAR_API_H_

#include <math.h>
#include <pthread.h>
#include <stdint.h>

#include "./inno_lidar_packet.h"
#include "./inno_lidar_packet_utils.h"
#include "../utils/inno_lidar_log.h"
#include "./ring_id_converter_interface.h"

/*****************
 * SDK VERSION
 *****************/
#define INNO_SDK_V_MAJOR "2"
#define INNO_SDK_V_MINOR "3"
#define INNO_SDK_V_DOT "0"
#define INNO_SDK_VERSION_IN_HEADER \
  INNO_SDK_V_MAJOR "." INNO_SDK_V_MINOR "." INNO_SDK_V_DOT "."

/*****************
 * data structure
 *****************/
#if defined(_QNX_) || defined (__MINGW64__)
struct cpu_set_t;
#endif

enum InnoLidarProtocol {
  INNO_LIDAR_PROTOCOL_NONE = 0,
  INNO_LIDAR_PROTOCOL_RAW_TCP = 1,
  INNO_LIDAR_PROTOCOL_RAW_MEM = 2,
  INNO_LIDAR_PROTOCOL_PCS_TCP = 3,
  INNO_LIDAR_PROTOCOL_PCS_UDP = 4,
  INNO_LIDAR_PROTOCOL_RAW_FILE = 5,
  INNO_LIDAR_PROTOCOL_PCS_FILE = 6,
  INNO_LIDAR_PROTOCOL_MAX = 7,
};

enum InnoLidarState {
  INNO_LIDAR_STATE_ERROR = 0,
  INNO_LIDAR_STATE_STARTING = 1,
  INNO_LIDAR_STATE_READY = 2,
  INNO_LIDAR_STATE_STREAMING = 3,
  INNO_LIDAR_STATE_UNKNOWN = 4,
  INNO_LIDAR_STATE_INVALID = 5,
};

enum InnoRecorderCallbackType {
  INNO_RECORDER_CALLBACK_TYPE_NONE = 0,
  INNO_RECORDER_CALLBACK_TYPE_RAW = 1,
  INNO_RECORDER_CALLBACK_TYPE_RAW2 = 2,
  INNO_RECORDER_CALLBACK_TYPE_RAW3 = 3,
  INNO_RECORDER_CALLBACK_TYPE_RAW4 = 4,
  INNO_RECORDER_CALLBACK_TYPE_CALI = 5,
  INNO_RECORDER_CALLBACK_TYPE_SCATTER = 6,
  INNO_RECORDER_CALLBACK_TYPE_MAX = 7,
};

extern "C" {
  /*******************
   * callbacks typedef
   *******************/

  /* @param lidar_handle The handle of the lidar that triggered the callback.
   * @param context Callback context passed in inno_lidar_set_callbacks().
   * @param from_remote > 0 means the message is from remote source
   * @param level Severity of message.
   * @param code Error code.
   * @param error_message Error message.
   * @return Void.
   */
  typedef void (*InnoMessageCallback)(int lidar_handle,
                                      void *context,
                                      uint32_t from_remote,
                                      enum InnoMessageLevel level,
                                      enum InnoMessageCode code,
                                      const char *error_message);


  /*
   * @param lidar_handle The handle of the lidar that triggered the callback.
   * @param context Callback context passed in inno_lidar_set_callbacks().
   * @param data Pointer to InnoDataPacket
   * @return 0
   */
  typedef int (*InnoDataPacketCallback)(int lidar_handle,
                                        void *context,
                                        const InnoDataPacket *data);
  /*
   * @param lidar_handle The handle of the lidar that triggered the callback.
   * @param context Callback context passed in inno_lidar_set_callbacks().
   * @param status Pointer to InnoDataPacket
   * @return 0
   */
  typedef int (*InnoStatusPacketCallback)(int lidar_handle,
                                          void *context,
                                          const InnoStatusPacket *status);

  /*
   * @param context Callback context passed in inno_lidar_set_callbacks().
   * @return Return the unix time in second defined in
   *         https://en.wikipedia.org/wiki/Unix_time
   *         e.g. ros::Time::now().toSec();
   */
  typedef double (*InnoHosttimeCallback)(void *context);

  /* reserved for internal use */
  typedef int (*InnoRecorderCallback)(int lidar_handle, void *context,
                                      enum InnoRecorderCallbackType type,
                                      const char *buffer, int len);

  /********************
   * exported functions
   ********************/

  /*
   * @brief Get Innovusion lidar API version
   * @return Version string.
   */
  const char *inno_api_version(void);

  /*
   * @brief Get Innovusion lidar API build tag.
   * @return Build tag string.
   */
  const char *inno_api_build_tag(void);

  /*
   * @brief Get Innovusion lidar API build time.
   * @return Build time string.
   */
  const char *inno_api_build_time(void);


  /*
   * @brief Setup sigaction for SIGSEGV SIGBUS SIGFPE (optional)
   *        The signal handler will print out the stack backtrace
   *        and then call old signal handler.
   * @return Void.
   */
  void inno_lidar_setup_sig_handler();

  /**
   * @brief : inno_lidar_log_callback
   */
  void inno_lidar_log_callback(InnoLogCallback log_callback,
                                void *ctx);
  /*
   * @brief Set log files fds (file descriptors).
   *        This function can be only called once.
   * @param out_fd All log whose level is less or equals
   *        to INNO_LOG_LEVEL_INFO will be written to this file.
   *        -1 means do not write.
   * @param error_fd All log whose level is bigger or equals
   *        to INNO_LOG_LEVEL_WARNING will be written to this file.
   *        -1 means do not write.
   * @param rotate_file_base_file
   * @param rotate_file_number
   * @param rotate_file_size_limit
   * @param log_callback
   * @param callback_ctx
   * @param rotate_file_base_file_err
   * @param rotate_file_number_err
   * @param rotate_file_size_limit_err
   * @param flags (1 means use use_async)
   * @return Void.
   */
  void inno_lidar_set_logs(int out_fd,
                           int error_fd,
                           const char *rotate_file_base_file,
                           uint32_t rotate_file_number,
                           uint64_t rotate_file_size_limit,
                           InnoLogCallback log_callback,
                           void *callback_ctx,
                           const char *rotate_file_base_file_err = NULL,
                           uint32_t rotate_file_number_err = 0,
                           uint64_t rotate_file_size_limit_err = 0,
                           uint64_t flags = 0);


  /*
   * @brief Set debug level for all lidar handles.
   *        This function can be called any time.
   *        It may be called multiple times, e.g. to decrease the log
   *        level before executing some code and then increasing the
   *        log level again after the section of code completes.
   * @param log_level Set new debug level.
   * @return Void.
   */
  void inno_lidar_set_log_level(enum InnoLogLevel log_level);

  /*
   * @brief Open a lidar handle for a live Innovusion lidar unit.
   * @param name Contains the name of lidar, e.g. "forward-1"
   *        The name should be less than 32 characters.
   * @param lidar_ip Contains the lidar IP string, e.g. "172.168.1.10".
   * @param port Lidar PORT, e.g. 8001.
   * @param protocol Specify protocol,
   * @param udp_port UDP port to receive data, when 
   *                 protocol is INNO_LIDAR_PROTOCOL_UDP
   * @return Return lidar handle
   */
  int inno_lidar_open_live(const char *name,
                           const char *lidar_ip,
                           uint16_t port,
                           enum InnoLidarProtocol protocol,
                           uint16_t udp_port);


  /*
   * @brief Open a lidar handle from an Innovusion lidar proprietary
   *        data file
   * @param name Name of lidar.
   *        The name should be less than 32 characters.
   * @param filename The filename of an Innovusion lidar
   *        proprietary data file.
   * @param raw_format
   * @param play_rate The playback rate,
   *        == 0 means play as fast as possible
   *        <= 100 means MB/s, e.g. 15 means play at 15 MB/s
   *         > 100 means rate_multiplier = play_rate / 10000.0,
   *           e.g, 15000 means play at 1.5x speed
   * @param rewind How many times rewind before stop, 0 means no rewind,
   *        < 0 means rewind infinity times.
   * @return Return a lidar handle.
   */
  int inno_lidar_open_file(const char *name,
                           const char *filename,
                           bool raw_format,
                           int play_rate,
                           int rewind,
                           int64_t skip);

  /*
   * @brief Set lidar working mode
   * @param mode Target mode
   * @param mode_before_change Mode bofore this change
   * @param status_before_change Lidar Status before this change
   * @return 0 means success, otherwise failure (e.g. invalid lidar handle)
   */
  int inno_lidar_set_mode(int handle,
                          enum InnoLidarMode mode,
                          enum InnoLidarMode *mode_before_change,
                          enum InnoLidarStatus *status);

  /*
   * @brief Get lidar current working mode and status
   * @param handle Lidar handle
   * @param mode Current work mode
   * @param pre_mode Previous work mode
   * @param status Work status
   * @param in_transition_mode_ms Time (in ms) stay in transition mode
   * @return 0 means success, otherwise failure (e.g. invalid lidar handle)
   */
  int inno_lidar_get_mode_status(int handle,
                                 enum InnoLidarMode *mode,
                                 enum InnoLidarMode *pre_mode,
                                 enum InnoLidarStatus *status,
                                 uint64_t *in_transition_mode_ms);

  /*
   * @brief Get lidar attribute like frame_rate
   * @param handle Lidar handle
   * @param attribute Name of the attribute
   * @param buffer Buffer to store the string value
   * @param Buffer_len Length of buffer, the recommended buffer_len is
   *                 1024, the size of buffer needs to be >= buffer_len.
   * @return 0 means success, otherwise failure (e.g. invalid lidar handle)
   */
  int inno_lidar_get_attribute_string(int handle,
                                      const char *attribute,
                                      char *buffer, size_t buffer_len);

  /*
   * @brief Set lidar attribute
   * @param handle Lidar handle
   * @param attribute Name of the attribute
   * @param buffer Buffer to store the string value
   * @return 0 means success, otherwise failure (e.g. invalid lidar handle)
   */
  int inno_lidar_set_attribute_string(int handle,
                                      const char *attribute,
                                      const char *buffer);

  /*
   * @brief Set callbacks for a lidar handle. Developers need to
   *        set callbacks before calling inno_lidar_start().
   *        For every lidar handle, inno_lidar_set_callbacks can
   *        only be called once.
   * @param handle Lidar handle (from either inno_lidar_open_live or
   *        inno_lidar_open_file).
   * @param message_callback This callback is called when some warning/error
   *                            happens.
   *                            For a given lidar handle, no message_callback or
   *                            frame_callback will be called until the
   *                            previous callback (for the same lidar handle)
   *                            has returned.
   *                            Multiple callbacks may happen at the same time
   *                            for different lidar handles.
   *                            Pass NULL pointer means no callback.
   * @param data_callback This callback is called when one frame or
   *                            sub-frame is available.  The callback happens
   *                            in a thread dedicated to that lidar_handle.
   *                            For a given lidar handle, no message_callback or
   *                            frame_callback will be called until the
   *                            previous callback (for the same lidar handle)
   *                            has returned.
   *                            Multiple callbacks may happen at the same time
   *                            for different lidar handles.
   *                            Pass NULL pointer means no callback.
   * @param status_callback
   * @param get_host_time This callback will get the current host time.
   *                           Pass NULL pointer means clock_gettime(CLOCK_REALTIME)
   *                           will be used.
   * @param callback_context Context passed in when callback is invoked.
   * @return 0 means success, otherwise failure (e.g. invalid lidar handle)
   */
  int inno_lidar_set_callbacks(int handle,
                               InnoMessageCallback message_callback,
                               InnoDataPacketCallback data_callback,
                               InnoStatusPacketCallback status_callback,
                               InnoHosttimeCallback get_host_time,
                               void *callback_context);

  /* reserved for internal use */
  int inno_lidar_set_recorder_callback(int handle,
                                       enum InnoRecorderCallbackType type,
                                       InnoRecorderCallback callback,
                                       void *callback_context);

  /*
   * @brief Specify the CPU affinity for all threads created by the
   *        library for this lidar instance. This function can only
   *        be called before inno_lidar_start() is called. Please see
   *        pthread_setaffinity_np() for reference.
   * @param handle Lidar handle.
   * @param cpusetsize size of *cpuset.
   * @param cpuset cupset that will be passed to pthread_setaffinity_np().
   * @param exclude_callback_thread 1 means do not set affinity for callback
   *        thread, 0 otherwise.
   * @return 0 means successful stored the affinity settings that will be
   *         passed in when calling pthread_setaffinity_np(). It doesn't
   *         not guarantee that pthread_setaffinity_np() will be successful.
   *         1 invalid handle
   */
  int inno_lidar_thread_setaffinity_np(int handle,
                                       size_t cpusetsize,
                                       const cpu_set_t *cpuset,
                                       int exclude_callback_thread);

  /*
   * @brief Set config name-value pair for a lidar handle.
   *        This function should be called before inno_lidar_start is called,
   *        and can be called multiple times.
   * @param handle Lidar handle
   * @param cfg_name Name of the config item.
   *        It must start with "Lidar_" and have two parts: section and key.
   *        separated by '/'. e.g. Lidar_section1/key1
   * @param cfg_value Value of the config item.
   * @return 0 means success
   *         1 invalid handle
   *         2 invalid name
   *         3 invalid value
   */
  int inno_lidar_set_config_name_value(int handle,
                                       const char *cfg_name,
                                       const char *cfg_value);

  /*
   * @brief Change the meaning of the ref data in InnoChannelPoint
   *        structures to indicate either reflectance or intensity.
   * @param handle Lidar handle
   * @param mode Either INNO_REFLECTANCE_MODE_INTENSITY
   *        or INNO_REFLECTANCE_MODE_REFLECTIVITY
   *        default is INNO_REFLECTANCE_MODE_REFLECTIVITY
   *        since 1.5.0
   * @return 0 means success, otherwise failure (e.g. invalid handle).
   */
  int inno_lidar_set_reflectance_mode(int handle,
                                      enum InnoReflectanceMode mode);

  /*
   * @brief Change the number & type of points returned each laser pulse.
   * @param handle Lidar handle.
   * @param ret_mode
   * @return 0 means success, otherwise failure (e.g. invalid handle).
   */
  int inno_lidar_set_return_mode(int handle,
                                 enum InnoMultipleReturnMode ret_mode);

  /*
   * @brief Set ROI (center point)
   * @param handle Lidar handle.
   * @param horz_angle ROI horizontal center (in [-60, 60] degrees)
   * @param vert_angle ROI vertical center (in [-25, 25] degrees)
   *        Setting either horz_angle or vert_angle to
   *        kInnoNopROI, i.e. 10000.0
   *        will maintain the previous value for that angle.  Any other value
   *        outside the allowed range will result in a failure code return and
   *        no change in ROI center.
   * @return 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_set_roi(int handle, double horz_angle, double vert_angle);

  /*
   * @brief Get current ROI (center point)
   * @param handle Lidar handle.
   * @param horz_angle Store return ROI horizontal center (in [-60, 60] degrees)
   * @param vert_angle StoreROI vertical center (in [-25, 25] degree)
   * @return 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_get_roi(int handle, double *horz_angle, double *vert_angle);

  /*
   * @brief Set the velocity and angular velocity for motion correction
   * @param handle Lidar handle.
   * @param velocity_m_s[3]: velocities in x, y, z axis, unit is m/s
   * @param angular_velocity_rad_s[3]: angular velocities in x, y, z axis, RAD/s
   * @return 0 means success, -1 if handle is invalid
   */
  int inno_lidar_set_motion_compensation(int handle,
                                         double velocity_m_s[3],
                                         double angular_velocity_rad_s[3]);

  /*
   * @brief Description: Query lidar unit's state
   * @param handle Lidar handle.
   * @param state
   * @param error_code Address to store error code. NULL means
   *        do not store error code
   * @return 0 means success, -1 if handle is invalid
   */
  int  inno_lidar_get_fw_state(int handle,
                               InnoLidarState *state,
                               int *error_code);


  /*
   * @brief Query lidar unit's firmware version.
   * @param handle Lidar handle.
   * @param buffer Buffer to store the firmware version.
   * @param Buffer_len Length of buffer, the recommended buffer_len is
   *                 512, the size of buffer needs to be >= buffer_len.
   * @return 0 cannot get firmware version
   *        -1 invlid lidar handle
   *        -2 buffer_len is too small
   *        -3 source is file
   *        otherwise return the size of firmware version string
   *         stored in buffer, not include the trailing zero
   *        Sample content in the buffer:
   *           App Version: app-2.3.0-rc8.134
   *             build-time: 2019-08-14-18-19-25
   *           FPGA Datecode: 0x190814e2
   *             fpga-ver: 0x13
   *             fpga-rev: 0x0e
   *             board-rev: 0x2
   *           Firmware Version: 2.3.1-rc3-418.2019-08-15-17-41-40
   *             build-tag: 2.3.1-rc3-418
   *             build-time: 2019-08-15-17-41-40
   *             build-git-tag: 1.0.19
   */
  int inno_lidar_get_fw_version(int handle, char *buffer, int buffer_len);

  /*
   * @brief Query lidar unit's S/N
   * @param handle Lidar handle.
   * @param buffer Buffer to store the S/N.
   * @param buffer_len Length of buffer, the recommended buffer_len is
   *                 128, the size of buffer needs to be >= buffer_len.
   * @return 0 cannot get S/N
   *         -1 invlid lidar handle
   *         -2 buffer_len is too small
   *         -3 source is file
   *         otherwise return the size of S/N string
   *         stored in buffer, not include the trailing zero
   */
  int inno_lidar_get_sn(int handle, char *buffer, int buffer_len);

  /*
   * @brief Query lidar unit's model
   * @param handle Lidar handle.
   * @param buffer Buffer to store the model.
   * @param buffer_len Length of buffer, the recommended buffer_len is
   *        32, the size of buffer needs to be >= buffer_len
   * @return 0 cannot get model
   *        -1 invlid lidar handle
   *        -2 buffer_len is too small
   *        -3 source is file
   *        otherwise return the size of model string
   *        stored in buffer, not include the trailing zero
   */
  int inno_lidar_get_model(int handle, char *buffer, int buffer_len);

  /*
   * @brief Start to read data from live lidar unit or from a data file.
   *        The message_callback, frame_callback and cframe_callback will
   *        be called only after this function is called.  They may
   *        be called before this function returns since they are in
   *        different threads.
   * @param handle Lidar handle.
   * @return 0 means success, otherwise failure (e.g. invalid handle).
   */
  int inno_lidar_start(int handle);

  /*
   * @brief Stop reading data from live lidar unit or from a data file.
   *        The message_callback, frame_callback and cframe_callback will
   *        not be called once this function is returned.
   * @param handle Lidar handle.
   * @return 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_stop(int handle);


  /*
   * @brief Stop all lidar handles.
   * @return number of handles stopped.
   */
  int inno_lidar_stop_all();


  /*
   * @brief Close the lidar handle and release all resources.
   *        This function can be called on a valid lidar handle only
   *        after inno_lidar_stop() call has returned,
   *        or inno_lidar_start() has never been called before.
   * @param handle Lidar handle.
   * @return 0 means success, otherwise failure (e.g. invalid handle).
   */
  int inno_lidar_close(int handle);


  /*
   * @brief Close all lidar handles and release all resources.
   * @return number of handles closed.
   */
  int inno_lidar_close_all();

  RingIdConverterInterface *inno_lidar_get_ring_id_converter(int handle);
};

#endif  // SDK_COMMON_INNO_LIDAR_API_H_
