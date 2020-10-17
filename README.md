#  Livox

2020.10.18



## Software

[Livox-SDK](https://github.com/Livox-SDK/Livox-SDK/blob/master/README_CN.md)

---



### 1.环境要求

* 已安装有cmake的Ubuntu 18.04

  ```shell 
  sudo apt install cmake pkg-config
  ```

* 下载LivoxSDK，在需要存放SDK的文件夹下运行如下命令：

  ```shell
  git clone https://github.com/Livox-SDK/Livox-SDK.git
  cd Livox-SDK
  ```

* 安装apr库

  ```shell
  sudo ./third_party/apr/apr_build.sh
  ```

  或者

  ```shell
  sudo apt install libapr1-dev
  ```

* 编译

  ```shell
  cd build && cmake ..
  make
  sudo make install
  ```



### 2.运行样例

* 在直连Livox LiDAR的情况下，进入编译时的build文件夹运行 *lidar_sample*

  ```shell
  cd sample/lidar && ./lidar_sample
  ```

  如果通过 Livox Hub 连接雷达，则运行 *hub_sample*

  ```shell
  cd sample/hub && ./hub_sample
  ```

  

### 3.数据的获取

* **概述**

  * 软件首先初始化SDK，随后配置来自雷达信息的回调函数`SetBroadcastCallback(OnDeviceBroadcast);`。

    * 在该回调函数`OnDeviceBroadcast`中，将当前在线的设备加入在线列表，并为每个设备设置一个用于接收数据的回调函数`SetDataCallback(handle, GetLidarData, NULL);`。

    * 在该函数`GetLidarData`中，来自雷达的数据会被自动的存放至`LivoxEthPacket *data`指向的位置，通过这个指针，可以获取如下的信息：

      ```c++
      typedef struct {
        uint8_t version;              /**< Packet protocol version. */
        uint8_t slot;                 /**< Slot number used for connecting LiDAR. */
        uint8_t id;                   /**< LiDAR id. */
        uint8_t rsvd;                 /**< Reserved. */
        uint32_t err_code;      /**< Device error status indicator information. */
        uint8_t timestamp_type;       /**< Timestamp type. */
        /** Point cloud coordinate format, refer to \ref PointDataType . */
        uint8_t data_type;
        uint8_t timestamp[8];         /**< Nanosecond or UTC format timestamp. */
        uint8_t data[1];              /**< Point cloud data. */
      } LivoxEthPacket;
      ```

      其中最后一项`uint8_t data[1]`是指向点云数据存放位置的指针，可以通过判断点云数据的不同类型，像附录中` GetLidarData`一样利用一个新的指针`LivoxImuPoint *p_point_data`来接收这个数据。

      *点云数据有很多种，可以在附表或*livox_def.h*中查看。需要注意的是点云数据和imu数据同时只能获取其中之一*

    * 接收到`LivoxImuPoint *p_point_data`后，通过数据的数量和单个结构体的长度，可以计算出本次所有点云在内存中的长度，通过如下直接复制内存区域的方式读取：

      ```c++
      memcpy(raw_point,p_point_data->data,RAW_POINT_NUM*sizeof(LivoxRawPoint));
      ```

      其中`raw_point`是需要接收所有数据的位置，`RAW_POINT_NUM`和`LivoxRawPoint`均取决与需要接收何种点云数据/imu数据。默认情况下数据为Type 2`LivoxExtendRawPoint`。

      以此时为例，接收到的是笛卡尔坐标系下的原始数据v，包含坐标和反射率的信息，`RAW_POINT_NUM = 100`

      至此，我们成功将一次扫描得到的点云信息存在了`raw_point`所对应的内存中，要进一步使用该数据可通过数据长度来逐个读取。

  * 然后可以配置雷达的状态变化触发的回调函数`SetDeviceStateUpdateCallback(OnDeviceInfoChange);`

  * 配置后可以启动设备`Start()`，设备将保持在线状态。

  * 使用结束后，停止扫描，并配置停止扫描触发的回调函数`LidarStopSampling(devices[i].handle, OnStopSampleCallback, NULL);`

  * 最后结束使用时，去初始化SDK`Uninit();`

* **部分文件结构**

  * **Livox-SDK/sdk_core/include/livox_sdk.h**     函数的定义

  * **Livox-SDK/sdk_core/include/livox_def.h**    结构体的定义

  * **Livox-SDK/sample**    sample的存放位置

    * **/lidar**    电脑直连设备并接收数据的sample

    * **/lidar_lvx_file**    电脑直连设备并储存为lvx文件的sample

    * **/hub\***  使用hub连接多个设备的sample

  * **Livox-SDK/build/sample**    编译后的文件存放的位置

* **需要的库**

  ```c++
  #include <stdio.h>
  #include <stdlib.h>
  #include <unistd.h>
  #include <string.h>
  #include <apr_general.h>
  #include <apr_getopt.h>
  #include "livox_sdk.h"
  ```

  

* **启动配置**

  * Initialize Livox-SDK

    ```c++
    Init();
    ```

  * 设置来自Livox LiDAR的广播信息的回调函数    **回调函数的设置必须在启动雷达 之前**

    ```c++
    SetBroadcastCallback(OnDeviceBroadcast);
    ```

    * 先声明并实现`OnDeviceBroadcast`函数

    * 声明并实现其中的`OnSampleCallback`函数

  * 设置Livox LiDAR的状态变化的回调函数    	*雷达上线/下线会触发该函数（非必须）*

    ```c++
    SetDeviceStateUpdateCallback(OnDeviceInfoChange); 
    ```

  * 搜索在线的设备，搜索到会自动变为连接状态，未搜索到返回 `false`

    ```c++
    Start();
    ```

* **关闭**

  * 停止扫描

    ```c++
      int i = 0;
      for (i = 0; i < kMaxLidarCount; ++i) {
        if (devices[i].device_state == kDeviceStateSampling) {
    /** Stop the sampling of Livox LiDAR. */
          LidarStopSampling(devices[i].handle, OnStopSampleCallback, NULL);
        }
      }
    ```

  * Uninitialize Livox-SDK

    ```c++
    Uninit();
    ```




* **需要实现的函数及样例**

  * `void OnDeviceBroadcast(const BroadcastDeviceInfo *info)`

    ```c++
    void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
      if (info == NULL || info->dev_type == kDeviceTypeHub) {
        return;
      }
    
      printf("Receive Broadcast Code %s\n", info->broadcast_code);
    
      if (lidar_count > 0) {
        bool found = false;
        int i = 0;
        for (i = 0; i < lidar_count; ++i) {
          if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0) {
            found = true;
            break;
          }
        }
        if (!found) {
          return;
        }
      }
    
      bool result = false;
      uint8_t handle = 0;
      result = AddLidarToConnect(info->broadcast_code, &handle);
      if (result == kStatusSuccess) {
        /** Set the point cloud data for a specific Livox LiDAR. */
        SetDataCallback(handle, GetLidarData, NULL);
        devices[handle].handle = handle;
        devices[handle].device_state = kDeviceStateDisconnect;
      }
    }
    ```

  * `void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)`

    ```c++
    void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
      if (data) {
        data_recveive_count[handle] ++ ;
        if (data_recveive_count[handle] % 100 == 0) {
          /** Parsing the timestamp and the point cloud data. */
          uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
          if(data ->data_type == kCartesian) {
            LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
          }else if ( data ->data_type == kSpherical) {
            LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
          }else if ( data ->data_type == kExtendCartesian) {
            LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
          }else if ( data ->data_type == kExtendSpherical) {
            LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
          }else if ( data ->data_type == kDualExtendCartesian) {
            LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
          }else if ( data ->data_type == kDualExtendSpherical) {
            LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
          }else if ( data ->data_type == kImu) {
            LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
          }
          //printf("data_type %d data[0] %d\n", data->data_type, data->data[0]);
          //printf("data_type %d packet num %d\n", data->data_type, data_recveive_count[handle]);
        }
      }
    }
    ```

    


* 附录

  ```c++
  /** Point data type. */
  typedef enum {
    kCartesian,               /**< Cartesian coordinate point cloud. */
    kSpherical,               /**< Spherical coordinate point cloud. */
    kExtendCartesian,         /**< Extend cartesian coordinate point cloud. */
    kExtendSpherical,         /**< Extend spherical coordinate point cloud. */
    kDualExtendCartesian,     /**< Dual extend cartesian coordinate  point cloud. */
    kDualExtendSpherical,     /**< Dual extend spherical coordinate point cloud. */
    kImu,                     /**< IMU data. */
    kMaxPointDataType         /**< Max Point Data Type. */
  } PointDataType;
  
  /** Cartesian coordinate format. */
  typedef struct {
    int32_t x;            /**< X axis, Unit:mm */
    int32_t y;            /**< Y axis, Unit:mm */
    int32_t z;            /**< Z axis, Unit:mm */
    uint8_t reflectivity; /**< Reflectivity */
  } LivoxRawPoint;
  
  /** Spherical coordinate format. */
  typedef struct {
    uint32_t depth;       /**< Depth, Unit: mm */
    uint16_t theta;       /**< Zenith angle[0, 18000], Unit: 0.01 degree */
    uint16_t phi;         /**< Azimuth[0, 36000], Unit: 0.01 degree */
    uint8_t reflectivity; /**< Reflectivity */
  } LivoxSpherPoint;
  
  /** Standard point cloud format */
  typedef struct {
    float x;              /**< X axis, Unit:m */
    float y;              /**< Y axis, Unit:m */
    float z;              /**< Z axis, Unit:m */
    uint8_t reflectivity; /**< Reflectivity */
  } LivoxPoint;
  
  /** Extend cartesian coordinate format. */
  typedef struct {
    int32_t x;            /**< X axis, Unit:mm */
    int32_t y;            /**< Y axis, Unit:mm */
    int32_t z;            /**< Z axis, Unit:mm */
    uint8_t reflectivity; /**< Reflectivity */
    uint8_t tag;          /**< Tag */
  } LivoxExtendRawPoint;
  
  /** Extend spherical coordinate format. */
  typedef struct {
    uint32_t depth;       /**< Depth, Unit: mm */
    uint16_t theta;       /**< Zenith angle[0, 18000], Unit: 0.01 degree */
    uint16_t phi;         /**< Azimuth[0, 36000], Unit: 0.01 degree */
    uint8_t reflectivity; /**< Reflectivity */
    uint8_t tag;          /**< Tag */
  } LivoxExtendSpherPoint;
  
  /** Dual extend cartesian coordinate format. */
  typedef struct {
    int32_t x1;            /**< X axis, Unit:mm */
    int32_t y1;            /**< Y axis, Unit:mm */
    int32_t z1;            /**< Z axis, Unit:mm */
    uint8_t reflectivity1; /**< Reflectivity */
    uint8_t tag1;          /**< Tag */
    int32_t x2;            /**< X axis, Unit:mm */
    int32_t y2;            /**< Y axis, Unit:mm */
    int32_t z2;            /**< Z axis, Unit:mm */
    uint8_t reflectivity2; /**< Reflectivity */
    uint8_t tag2;          /**< Tag */
  } LivoxDualExtendRawPoint;
  
  /** Dual extend spherical coordinate format. */
  typedef struct {
    uint16_t theta;        /**< Zenith angle[0, 18000], Unit: 0.01 degree */
    uint16_t phi;          /**< Azimuth[0, 36000], Unit: 0.01 degree */
    uint32_t depth1;       /**< Depth, Unit: mm */
    uint8_t reflectivity1; /**< Reflectivity */
    uint8_t tag1;          /**< Tag */
    uint32_t depth2;       /**< Depth, Unit: mm */
    uint8_t reflectivity2; /**< Reflectivity */
    uint8_t tag2;          /**< Tag */
  } LivoxDualExtendSpherPoint;
  
  /** IMU data format. */
  typedef struct {
    float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
    float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
    float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
    float acc_x;         /**< Accelerometer X axis, Unit:g */
    float acc_y;         /**< Accelerometer Y axis, Unit:g */
    float acc_z;         /**< Accelerometer Z axis, Unit:g */
  } LivoxImuPoint;
  ```

  
