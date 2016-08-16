#ifndef AMCL_POINT_CLOUD_H
#define AMCL_POINT_CLOUD_H

#include "amcl_sensor.h"
#include "../map/map.h"

namespace amcl
{

class AMCLPointCloudData : public AMCLSensorData
{
  public:
    AMCLPointCloudData () {points=NULL;};
    virtual ~AMCLPointCloudData() {delete [] points;};
  public: int points_size;
  public: double (*points)[3];
};

class AMCLPointCloud : public AMCLSensor
{
  public: AMCLPointCloud(map_t* map);

  public: virtual ~AMCLPointCloud(); 

  public: void SetElevationDifferenceMapModel(double z_hit,
                                              double z_rand,
                                              double z_sigma);

  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  public: void SetPointCloudPose(pf_vector_t& point_cloud_pose) 
          {this->point_cloud_pose = point_cloud_pose;}

  private: static double ElevationDifferenceMapModel(AMCLPointCloudData *data, 
                                                     pf_sample_set_t* set);

  private: double time;

  private: map_t *map;

  private: pf_vector_t point_cloud_pose;
  
  private: double z_hit;
  private: double z_rand;
  private: double z_sigma;
};


}

#endif
