/*
 * quaternion.c
 *
 *  Created on: 16.12.2019
 *      Author: Valentin Marx
 */
#include "main.h"
#include "quaternion.h"

Quaternion* quat_multiply(Quaternion *q1, Quaternion *q2, Quaternion *qout) {
	Quaternion qtemp; //Zwischenspeicher anlegen, falls q1/q2 = qout
	qtemp.q0 = (q1->q0 * q2->q0) - (q1->q1 * q2->q1) - (q1->q2 * q2->q2)
			- (q1->q3 * q2->q3);
	qtemp.q1 = (q1->q0 * q2->q1) + (q1->q1 * q2->q0) + (q1->q2 * q2->q3)
			- (q1->q3 * q2->q2);
	qtemp.q2 = (q1->q0 * q2->q2) + (q1->q2 * q2->q0) + (q1->q3 * q2->q1)
			- (q1->q1 * q2->q3);
	qtemp.q3 = (q1->q0 * q2->q3) + (q1->q3 * q2->q0) + (q1->q1 * q2->q2)
			- (q1->q2 * q2->q1);
	quat_copy(&qtemp, qout);
	return qout;
}

Quaternion* quat_add(Quaternion *qa, Quaternion *qb, Quaternion *qout) {
	qout->q0 = qa->q0 + qb->q0;
	qout->q1 = qa->q1 + qb->q1;
	qout->q2 = qa->q2 + qb->q2;
	qout->q3 = qa->q3 + qb->q3;
	return qout;
}

Quaternion* vec2quat(Vector *v, Quaternion *qout) {
	qout->q0 = 0;
	qout->q1 = v->x;
	qout->q2 = v->y;
	qout->q3 = v->z;
	return qout;
}

Quaternion* quat_qpq(Quaternion *q, Quaternion *p, Quaternion *qout) {
	Quaternion qi;
	quat_copy(q, &qi);
	quat_conj(&qi);
	quat_multiply(q, p, qout);
	quat_multiply(qout, &qi, qout);
	return qout;
}

Quaternion* quat_rotate(Quaternion *q, float angle) {
	q->q0 = 0;
	quat_norm(q);
	q->q0 = cos(angle / 2);
	float sa = sin(angle / 2);
	q->q1 *= sa;
	q->q2 *= sa;
	q->q3 *= sa;
	return q;
}

Vector* vec_rotate(Vector *v, Quaternion *qr, Vector *vout) {
	//Lege Variablen für Vektor Quat und Konjugation an
	Quaternion qv, qc, qout;
	vec2quat(v, &qv); //Vektor in Quat umwandeln
	quat_copy(qr, &qc); // qr duplizieren
	quat_conj(&qc); //qc konjugieren
	quat_multiply(qr, &qv, &qout); //Von Links hinmultiplizieren
	quat_multiply(&qout, &qc, &qout); // Von rechts hinmultiplizieren
	quat2vec(&qout, vout);
	return vout;
}
Vector* quat2vec(Quaternion *q, Vector *vout) {
	vout->x = q->q1;
	vout->x = q->q2;
	vout->z = q->q3;
	return vout;
}
float quat_length(Quaternion *qt) {
	float l;
	l = (qt->q0 * qt->q0) + (qt->q1 * qt->q1) + (qt->q2 * qt->q2)
			+ (qt->q3 * qt->q3);
	l = sqrtf(l);
	return l;
}
float vec_length(Vector *v) {
	float l;
	l = (v->x * v->x) + (v->y * v->y) + (v->z * v->z);
	l = sqrtf(l);
	return l;
}
void quat_norm(Quaternion *qt) {
	float l;
	l = 1.0f / quat_length(qt);
	quat_scale(qt, l);
}

void quat_copy(Quaternion *qin, Quaternion *qout) {
	qout->q0 = qin->q0;
	qout->q1 = qin->q1;
	qout->q2 = qin->q2;
	qout->q3 = qin->q3;
}
void quat_conj(Quaternion *qt) {
	qt->q1 *= -1.0f;
	qt->q2 *= -1.0f;
	qt->q3 *= -1.0f;
}

void quat_scale(Quaternion *qt, float scale) {
	qt->q0 *= scale;
	qt->q1 *= scale;
	qt->q2 *= scale;
	qt->q3 *= scale;
}
