/*
 * control.c
 *
 *  Created on: 20.12.2019
 *      Author: Valentin Marx
 */

#include "control.h"

//I-Regler init
float i_pitch = 0, i_roll = 0, i_yaw = 0;
//D-Regler init
float d_pitch = 0, d_roll = 0, d_yaw = 0;

float looptime = 0.0;

/**
 * @brief Steuert Pitch
 * @return tatsächlich angewendete steuerung
 * @param amount Wert von -500 bis 500, der Auslenkung entspricht
 */
int16_t pitch(int16_t amount) {
	servo[4] = 1500 + amount;
	if (servo[4] <= 1000) {
		servo[4] = 1000;
		return 0;
	} else if (servo[0] >= 2000) {
		servo[4] = 2000;
		return 0;
	} else {
		return amount;
	}
}

/**
 * @brief Steuert Roll
 * @return tatsächlich angewendete steuerung
 * @param amount Wert von -500 bis 500, der Auslenkung entspricht
 */
int16_t roll(int16_t amount) {
	servo[1] = 1500 + amount;
	//servo[4] = 2000 - (servo[3] - 1000); //servo4 = invers servo3
	if (servo[1] <= 1000) {
		servo[1] = 1000;
		//servo[4] = 2000;
		return 0;
	} else if (servo[1] >= 2000) {
		servo[1] = 2000;
		//servo[4] = 1000;
		return 0;
	} else {
		return amount;
	}
}

/**
 * @brief Steuert yaw
 * @return tatsächlich angewendete steuerung
 * @param amount Wert von -500 bis 500, der Auslenkung entspricht
 */
int16_t yaw(int16_t amount) {
	servo[0] = 1500 + amount;
	if (servo[0] <= 1000) {
		servo[0] = 1000;
		return 0;
	} else if (servo[0] >= 2000) {
		servo[0] = 2000;
		return 0;
	} else {
		return amount;
	}
}

/**
 * @brief Steuert Motor
 * @return tatsächlich angewendete steuerung
 * @param amount Wert von 0 bis 1000, der Motorstärke entspricht
 */

int16_t throttle(int16_t amount) {
	servo[2] = 1000 + amount;
	if (servo[2] <= 1000) {
		servo[2] = 1000;
		return 0;
	} else if (servo[2] >= 2000) {
		servo[2] = 2000;
		return 0;
	} else {
		return amount;
	}
}

/**
 * @brief Setze alle Servos in die Mitte und den Motor aus
 */
void center_off() {
	servo[0] = 1500;
	servo[1] = 1500;
	servo[2] = 1000;
	servo[3] = 1500;
	servo[4] = 1500;
}

void pid_init(){
	//Roll
	pid_r.p = 300.0f;
	pid_r.i = 10.0f;
	pid_r.d = 0.0f;
	//Yaw
	pid_y.p = 300.0f;
	pid_y.i = 10.0f;
	pid_y.d = 0.0f;
	//Pitch
	pid_p.p = 300.0f;
	pid_p.i = 10.0f;
	pid_p.d = 0.0f;

}

/**
 * @brief PID für pitch
 */
void pid_pitch() {
	//Aktuelle Verdehrung rausfinden
		Quaternion qx = { 0, 0, 1, 0 }, qxi, qxs;
		quat_qpq(&q_ist, &qx, &qxi); //Projeziere z Achse auf aktuelle Position
		quat_qpq(&q_soll, &qx, &qxs); //Projeziere z-Achse auf soll Position
		float regel = qxs.q3 - qxi.q3; //Regelgröße berechnen

		//P
		float pr = regel * pid_p.p;

		//I
		i_pitch += regel * looptime;
		float ir = i_pitch * pid_p.i;

		//D
		float dr = (regel - d_pitch) * pid_p.d;
		d_pitch = regel;

		regel = pr + ir + dr;
		pitch(regel);
}
void pid_roll() {
	//Aktuelle Verdehrung rausfinden
		Quaternion qx = { 0, 1, 0, 0 }, qxi, qxs;
		quat_qpq(&q_ist, &qx, &qxi); //Projeziere z Achse auf aktuelle Position
		quat_qpq(&q_soll, &qx, &qxs); //Projeziere z-Achse auf soll Position
		float regel = qxs.q3 - qxi.q3; //Regelgröße berechnen

		//P
		float pr = regel * pid_r.p;

		//I
		i_roll += regel * looptime;
		float ir = i_roll * pid_r.i;

		//D
		float dr = (regel - d_roll) * pid_r.d;
		d_roll = regel;

		regel = pr + ir + dr;
		roll(regel);
}
void pid_yaw() {
	//Aktuelle Verdehrung rausfinden
	Quaternion qx = { 0, 0, 1, 0 }, qxi, qxs;
	quat_qpq(&q_ist, &qx, &qxi); //Projeziere y Achse auf aktuelle Position
	quat_qpq(&q_soll, &qx, &qxs); //Projeziere y-Achse auf soll Position
	float regel = qxs.q1 - qxi.q1; //Regelgröße berechnen

	//P
	float pr = regel * pid_y.p;

	//I
	i_yaw += regel * looptime;
	float ir = i_yaw * pid_y.i;

	//D
	float dr = (regel - d_yaw) * pid_y.d;
	d_yaw = regel;

	regel = pr + ir + dr;
	yaw(regel);
}
