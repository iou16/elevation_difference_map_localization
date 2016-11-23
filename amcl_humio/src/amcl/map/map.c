#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "map.h"


map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  map->origin_x = 0;
  map->origin_y = 0;
  
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  map->cells = (map_cell_t*) NULL;
  
  return map;
}


void map_free(map_t *map)
{
  // map->cells=0;
  free(map->cells);
  map->cells=0;
  free(map);
  return;
}

