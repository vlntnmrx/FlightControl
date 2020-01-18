#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <math.h>

typedef struct __quaternion {
	float q0;
	float q1;
	float q2;
	float q3;
} Quaternion;

typedef struct __vector {
	float x;
	float y;
	float z;
} Vector;

Quaternion q_pos;

Quaternion* quat_multiply(Quaternion *q1, Quaternion *q2, Quaternion *qout);
Quaternion* quat_add(Quaternion *qa, Quaternion *qb, Quaternion *qout);
Quaternion* vec2quat(Vector *v, Quaternion *qout);
Quaternion* quat_qpq(Quaternion *q, Quaternion *p, Quaternion *qout);
Quaternion* quat_rotate(Quaternion* q, float angle);
Vector* vec_rotate(Vector *v, Quaternion *qr, Vector *vout);
Vector* quat2vec(Quaternion *q, Vector *vout);
float quat_length(Quaternion *q);
float vec_length(Vector *v);
void quat_norm(Quaternion *qt);
void quat_copy(Quaternion *qin, Quaternion *qout);
void quat_conj(Quaternion *qt);
void quat_scale(Quaternion *qt, float scale);

#endif // __QUATERNION_H__
//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
