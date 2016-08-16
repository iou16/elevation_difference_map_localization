#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <iostream>

#include "amcl_point_cloud.h"

using namespace amcl;

AMCLPointCloud::AMCLPointCloud(map_t* map) : AMCLSensor()
{
  this->time = 0.0;

  this->map = map;

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

  for (int j = 0; j < set->sample_count; j++)
  {
    // clock_t start = clock();
    /// map_t *obs_map = map_alloc();
    /// obs_map->size_x = 300;
    /// obs_map->size_y = 300;
    /// obs_map->scale = 0.1;
    /// obs_map->origin_x = -15.0;
    /// obs_map->origin_y = -15.0;
    /// obs_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*obs_map->size_x*obs_map->size_y);
    map_t *obs_map = map_alloc();
    obs_map->size_x = self->map->size_x;
    obs_map->size_y = self->map->size_y;
    obs_map->scale = self->map->scale;
    obs_map->origin_x = self->map->origin_x;
    obs_map->origin_y = self->map->origin_y;
    obs_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*obs_map->size_x*obs_map->size_y);
    // clock_t end1 = clock();
    // std::cout << "end1 :" << (double)(end1 - start) /CLOCKS_PER_SEC << std::endl;

    pf_sample_t *sample = set->samples + j;
    pf_vector_t pose = sample->pose;

    pose = pf_vector_coord_add(self->point_cloud_pose, pose);

    double p = 1.0;

    double z_hit_denom = 2 * self->z_sigma * self->z_sigma;
    // double z_rand_mult = 1.0/30.0;

    for (int i = 0; i < data->points_size; i++)
    {
      double point_x = data->points[i][0];
      double point_y = data->points[i][1];

      pf_vector_t hit;
      hit.v[0] = pose.v[0] + point_x * cos(pose.v[2]) - point_y *sin(pose.v[2]);
      hit.v[1] = pose.v[1] + point_x * sin(pose.v[2]) + point_y *cos(pose.v[2]);

      map_updata_cell(obs_map, hit.v[0], hit.v[1], data->points[i][2]);
      /// map_updata_cell(obs_map, data->points[i][0], data->points[i][1], data->points[i][2]);
    }
    
    // clock_t end2 = clock();
    // std::cout << "end2 :" << (double)(end2 - end1) /CLOCKS_PER_SEC << std::endl;

    for (int mi = MAP_GXWX(self->map, pose.v[0]-15); mi < MAP_GXWX(self->map, pose.v[0]+15); mi++)
    {
      for (int mj = MAP_GYWY(self->map, pose.v[1]-15); mj < MAP_GYWY(self->map, pose.v[1]+15); mj++)
      {
        double pz = 0.0;
        
        if((!(self->map->cells[MAP_INDEX(self->map,mi,mj)].flag))||(!(obs_map->cells[MAP_INDEX(obs_map,mi,mj)].flag))) continue;

        if((self->map->cells[MAP_INDEX(self->map,mi,mj)].diff < 0.05)||(obs_map->cells[MAP_INDEX(obs_map,mi,mj)].flag < 0.05)) continue;

        if((!MAP_VALID(self->map, mi, mj))||(!MAP_VALID(obs_map, mi, mj))) continue;

        double z = self->map->cells[MAP_INDEX(self->map,mi,mj)].diff - obs_map->cells[MAP_INDEX(self->map,mi,mj)].diff;
        pz += self->z_hit * exp(-(z * z) / z_hit_denom);
        // pz += self->z_rand * z_rand_mult;

        p += pz*pz*pz;
      }
    }
    /// int mk = 0, ml = 0;
    /// for (int mi = MAP_GXWX(self->map, pose.v[0]-15); mi < MAP_GXWX(self->map, pose.v[0]+15); mi++)
    /// {
    ///   for (int mj = MAP_GYWY(self->map, pose.v[1]-15); mj < MAP_GYWY(self->map, pose.v[1]+15); mj++)
    ///   {
    ///     double pz = 0.0;
    ///     
    ///     if((!(self->map->cells[MAP_INDEX(self->map,mi,mj)].flag))||(!(obs_map->cells[MAP_INDEX(obs_map,mk,ml)].flag))) continue;

    ///     if((self->map->cells[MAP_INDEX(self->map,mi,mj)].diff < 0.05)||(obs_map->cells[MAP_INDEX(obs_map,mk,ml)].flag < 0.05)) continue;

    ///     if((!MAP_VALID(self->map, mi, mj))||(!MAP_VALID(obs_map, mk, ml))) continue;

    ///     double z = self->map->cells[MAP_INDEX(self->map,mi,mj)].diff - obs_map->cells[MAP_INDEX(self->map,mk,ml)].diff;
    ///     pz += self->z_hit * exp(-(z * z) / z_hit_denom);
    ///     // pz += self->z_rand * z_rand_mult;

    ///     p += pz*pz*pz;
    ///     ml++;
    ///   }
    ///   mk++;
    ///   ml=0;
    /// }
    // clock_t end3 = clock();
    // std::cout << "end3 :" << (double)(end3 - end2) /CLOCKS_PER_SEC << std::endl;

    sample->weight *= p;
    total_weight += sample->weight;
    map_free(obs_map);
    obs_map = NULL;
  }

  return(total_weight);
}
