/*
===============================================================================

  FILE:  points_array.h
  
  CONTENTS:
  
    see corresponding .c file
  
===============================================================================
*/
#ifndef POINTS_ARRAY_H
#define POINTS_ARRAY_H
			
typedef struct _PointNode
{
	float xyz[3];               //do we need "double"?TODO
	char point_properties;
	char rgb[3];
	int delete_sn;
	int selected;
	int highlight;
	//struct _PointNode *next;
}PointNode;
void PointStorage_create_point(PointNode *p, float x, float y, float z);
//inline int PointStorage_get_available_num(void);
//inline PointNode* PointStorage_get_point(PointNode *p, int position);
#endif
