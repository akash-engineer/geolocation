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


#ifndef SDK_COMMON_INNO_LIDAR_OTHER_API_H_
#define SDK_COMMON_INNO_LIDAR_OTHER_API_H_

extern "C" {
  /********************
   * exported functions
   ********************/

  /* reserved for internal use */
  int inno_lidar_read_ps_reg(int handle, uint16_t off, uint32_t *value);
  int inno_lidar_read_pl_reg(int handle, uint16_t off, uint32_t *value);
  int inno_lidar_write_ps_reg(int handle, uint16_t off, uint32_t value);
  int inno_lidar_write_pl_reg(int handle, uint16_t off, uint32_t value);

  /*
   * @brief Use lidar configuration parameter file for a lidar handle.
   *        This function should be called before inno_lidar_start is called.
   * @param handle Lidar handle.
   * @param lidar_model Lidar model name, e.g. "E" or "REV_E"
   * @param yaml_filename Full file path of the lidar configuration
   *        parameter file. The file is in yaml format.
   * @return 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_set_parameters(int handle, const char *lidar_model,
                                const char *yaml_filename);
};

#endif  // SDK_COMMON_INNO_LIDAR_OTHER_API_H_
