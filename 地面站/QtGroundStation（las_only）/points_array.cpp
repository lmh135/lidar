/*
===============================================================================

  FILE:  points_array.c
  
  CONTENTS:
  
    a class like module to use manage points data
    
===============================================================================
*/
#include <stdlib.h>
#include "points_array.h"

static int s_point_num_used = 5;
static PointNode **s_points_header = NULL;


void PointStorage_create_point(PointNode *p, float x, float y, float z)
{
	p->xyz[0] = x;
	p->xyz[1] = y;
	p->xyz[2] = z;
    p->delete_sn = 0;
 }



/*inline int PointStorage_get_available_num(void)
{
    //return s_point_num_available;
    return s_point_num_used;
}
*/
/** 
 * @brief   get the address of the certain point
 * 
 * @param   position
 * 
 * @return  the point address
 */
// Need
/*inline PointNode* PointStorage_get_point(int position)
{
    if (position >= s_point_num_used)
        return NULL;
    return s_points_header[position];
}
*/
/*inline PointNode* PointStorage_get_point(PointNode *p, int position)
{
	if (position >= s_point_num_used)
		return NULL;
	return (p + position);
}
*/
