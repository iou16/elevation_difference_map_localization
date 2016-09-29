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

  // map_t *obs_map = map_alloc();
  // obs_map->size_x = self->map->size_x;
  // obs_map->size_y = self->map->size_y;
  // obs_map->scale = self->map->scale;
  // obs_map->origin_x = self->map->origin_x;
  // obs_map->origin_y = self->map->origin_y;
  // obs_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*obs_map->size_x*obs_map->size_y);

  for (int j = 0; j < set->sample_count; j++)
  {
    // clock_t start = clock();
    map_t *obs_map = map_alloc();
    obs_map->size_x = self->map->size_x;
    obs_map->size_y = self->map->size_y;
    obs_map->scale = self->map->scale;
    obs_map->origin_x = self->map->origin_x;
    obs_map->origin_y = self->map->origin_y;
    obs_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*obs_map->size_x*obs_map->size_y);
    // clock_t end1 = clock();
    // std::cout << "end1 :" << (double)(end1 - start) /CLOCKS_PER_SEC << std::endl;
    
    // for(int i=0; i<obs_map->size_x*obs_map->size_y; i++) {
    //     if (obs_map->cells[i].diff != 0.0) printf("%f\n",obs_map->cells[i].diff);
    //     if (obs_map->cells[i].min != 0.0) printf("%f\n",obs_map->cells[i].diff);
    //     if (obs_map->cells[i].max != 0.0) printf("%f\n",obs_map->cells[i].diff);
    //     // obs_map->cells[i].diff = 0.0;
    //     // obs_map->cells[i].min = 0.0;
    //     // obs_map->cells[i].max = 0.0;
    // }

    pf_sample_t *sample = set->samples + j;
    pf_vector_t pose = sample->pose;

    pose = pf_vector_coord_add(self->point_cloud_pose, pose);

    // double p = 1.0;
    double p = 0.0;

    // map_t *obs_map = map_alloc();
    // obs_map->size_x = 300;
    // obs_map->size_y = 300;
    // obs_map->scale = 0.1;
    // obs_map->origin_x = pose.v[0];
    // obs_map->origin_y = pose.v[1];
    // obs_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*obs_map->size_x*obs_map->size_y);

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

    // std::cout << "test" << std::endl;
    // sensor_msgs::PointCloud map_cloud;
    // map_cloud.points.clear();
    for (int mi = MAP_GXWX(self->map, pose.v[0]-15); mi < MAP_GXWX(self->map, pose.v[0]+15); mi++)
    {
      for (int mj = MAP_GYWY(self->map, pose.v[1]-15); mj < MAP_GYWY(self->map, pose.v[1]+15); mj++)
      {
        // double pz = 0.0;
        
        // if(self->map->cells[MAP_INDEX(self->map,mi,mj)].diff == 0.0 || obs_map->cells[MAP_INDEX(obs_map,mi,mj)].diff == 0.0) continue;
        // std::cout << "test" << std::endl;
        // if(self->map->cells[MAP_INDEX(self->map,mi,mj)].diff < 0.05 || obs_map->cells[MAP_INDEX(obs_map,mi,mj)].diff < 0.05) continue;

        if((!MAP_VALID(self->map, mi, mj))||(!MAP_VALID(obs_map, mi, mj))) continue;

        double z = self->map->cells[MAP_INDEX(self->map,mi,mj)].diff - obs_map->cells[MAP_INDEX(obs_map,mi,mj)].diff;
        // pz += self->z_hit * exp(-(z * z) / z_hit_denom);
        double pz = obs_map->cells[MAP_INDEX(obs_map,mi,mj)].diff * exp(-(z * z) / (z_hit_denom + self->map->cells[MAP_INDEX(self->map,mi,mj)].cov)); 
        // pz += self->z_rand * z_rand_mult;

        // p += pz*pz*pz;
        p += pz;
        // p += pz;
        // std::cout << "test" << std::endl;
        // geometry_msgs::Point32 map_point;
        // map_point.x = MAP_WXGX(obs_map, mi);
        // map_point.y = MAP_WXGX(obs_map, mj);
        // map_point.z = obs_map->cells[MAP_INDEX(obs_map,mi,mj)].diff;
        // map_cloud.points.push_back(map_point);
      }
    }
    // map_cloud.header.frame_id = "map";
    // map_cloud.header.stamp = ros::Time::now();
    // AMCLPointCloud::local_map_cloud_pub.publish(map_cloud);

    // int mk = MAP_GXWX(obs_map, pose.v[0]-15), ml = MAP_GYWY(obs_map, pose.v[1]-15);
    // // std::cout << "test" << std::endl;
    // for (int mi = MAP_GXWX(self->map, pose.v[0]-15); mi < MAP_GXWX(self->map, pose.v[0]+15); mi++)
    // {
    //   for (int mj = MAP_GYWY(self->map, pose.v[1]-15); mj < MAP_GYWY(self->map, pose.v[1]+15); mj++)
    //   {
    //     double pz = 0.0;
    //     
    //     if((!(self->map->cells[MAP_INDEX(self->map,mi,mj)].flag))||(!(obs_map->cells[MAP_INDEX(obs_map,mk,ml)].flag))) continue;
    //     // std::cout << "test" << std::endl;
    //     if((self->map->cells[MAP_INDEX(self->map,mi,mj)].diff < 0.05)||(obs_map->cells[MAP_INDEX(obs_map,mk,ml)].flag < 0.05)) continue;

    //     // if((!MAP_VALID(self->map, mi, mj))||(!MAP_VALID(obs_map, mk, ml))) continue;

    //     double z = self->map->cells[MAP_INDEX(self->map,mi,mj)].diff - obs_map->cells[MAP_INDEX(obs_map,mk,ml)].diff;
    //     pz += self->z_hit * exp(-(z * z) / z_hit_denom);
    //     // pz += self->z_rand * z_rand_mult;

    //     p += pz*pz*pz;
    //     ml++;
    //     // std::cout << "test" << std::endl;
    //   }
    // mk++;
    // ml=MAP_GYWY(obs_map, pose.v[1]-15);
    // }
    
    sample->weight *= p;
    total_weight += sample->weight;
    map_free(obs_map);
    obs_map = NULL;
    // memset(obs_map->cells, 0, sizeof(map_cell_t)*obs_map->size_x*obs_map->size_y);
  }
  // map_free(obs_map);
  // obs_map = NULL;
    
  return(total_weight);
}
