#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <string.h>

#include "amcl_point_cloud.h"

using namespace amcl;

AMCLPointCloud::AMCLPointCloud(map_t* map) : AMCLSensor()
{
  this->time = 0.0;

  this->map = map;

  // ros::NodeHandle nh;
  // local_map_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("local_map_cloud", 2, true);

  return;
}

AMCLPointCloud::~AMCLPointCloud(){}

void 
AMCLPointCloud::SetElevationDifferenceMapModel(double z_hit,
                                               double z_rand,
                                               double z_sigma)
{
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->z_sigma = z_sigma;
}

bool AMCLPointCloud::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{

  pf_update_sensor(pf, (pf_sensor_model_fn_t) ElevationDifferenceMapModel, data);  

  return true;
}



double AMCLPointCloud::ElevationDifferenceMapModel(AMCLPointCloudData *data, pf_sample_set_t* set)
{
  AMCLPointCloud *self = (AMCLPointCloud*) data->sensor;

  double total_weight = 0.0;
  // double z_hit_denom = 2 * self->z_sigma * self->z_sigma;

  for (int j = 0; j < set->sample_count; j++)
  {
    pf_sample_t *sample = set->samples + j;
    pf_vector_t pose = sample->pose;

    double p = 0.0;

    for (int i = 0; i < data->points_size; i++)
    {
      double point_x = data->points[i][0];
      double point_y = data->points[i][1];

      pf_vector_t hit;
      hit.v[0] = pose.v[0] + point_x * cos(pose.v[2]) - point_y *sin(pose.v[2]);
      hit.v[1] = pose.v[1] + point_x * sin(pose.v[2]) + point_y *cos(pose.v[2]);
      int mi = MAP_GXWX(self->map, hit.v[0]), mj = MAP_GYWY(self->map, hit.v[1]);

      double z = self->map->cells[MAP_INDEX(self->map,mi,mj)].diff - data->points[i][2];
      double z_hit_denom = 2 * (self->z_sigma + self->map->cells[MAP_INDEX(self->map,mi,mj)].cov) * (self->z_sigma + self->map->cells[MAP_INDEX(self->map,mi,mj)].cov);
      double pz = data->points[i][2] * exp(-(z * z) / (z_hit_denom));
      p += pz * pz * pz;

    }
    sample->weight *= p;
    total_weight += sample->weight;
  }
    
  return(total_weight);
}
