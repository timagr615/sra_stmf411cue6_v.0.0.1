/*
 * fusion.h
 *
 *  Created on: 12 янв. 2022 г.
 *      Author: timagr615
 */

#ifndef INC_FUSION_H_
#define INC_FUSION_H_

#include "stdbool.h"
#include "matrix.h"
#include "math.h"
#include "coordinates.h"
#include "ekf.h"

//extern Model model;


//#define n_st 5
//#define n_o 5
#define sG 2.5
#define sC 2.5
#define sV 0.3
//#define sC 0.1
#define sA 2.5
#define varG 2.5
#define varC 1
#define varA 0.1
#define varVe 0.3

enum DataPointType {
	IMU = 0,
	GPS
};

typedef struct {
	/* Q Matrix parameters - model Variances */
	double sGPS;  // Variance of x, y coordinates
	double sCourse; // Variance of course
	double sVelocity; // Variance of velocity
	double sYaw;  // Variance of yaw rate (z component of earth gyro data
	double sAcceleration; // Variance of longitudinal acceleration

	/* R Matrix parameters - measurement errors */
	double varGps; // GPS error: typically sensor accuracy; In R matrix varGps*hdop
	double varVelocity; // Velocity error from gps datasheet
	double varCourse;  // Course error
	double varYaw;  // Yaw Rate error
	double varAcceleration;  // Acceleration error

} EKFParams;


typedef struct {
	bool first_data_point;
	//bool initialized;
	uint32_t timestamp;
	uint8_t data_type;

	GeoPoint first_point;

	GeoPoint prev_point;
	GeoPoint curr_point;
	double dt;
	double dx_first, dy_first;
	double dx, dy;
	double mx, my;
	double ds;

	Vector raw_data;
	Vector z;

	double hdop;
} DataPoint;


//DataPoint dataPointInit();
void dataPointInit(DataPoint *point, Model *model);
void setData(DataPoint *point, Model *model, double lat, double lon, double alt, double vel, double course, double heading,
		double yaw_rate, double acc, double hdop, uint32_t dtime);
void getRawData(DataPoint *point, double lat, double lon, double alt, double vel, double course, double heading,
		double yaw_rate, double acc, double hdop, uint32_t dtime);



typedef struct {
	bool initialized;
	//uint8_t n_states, n_obs;
	EKF KF;
	DataPoint Point;

	double sGPS;
	double sCourse;
	double sVelocity;
	double sYaw;
	double sAccel;
	double varGps;
	double varVelocity;
	double varCourse;
	double varYaw;
	double varAcc;


} Fusion;

//Fusion InitFusion(uint8_t n_states, uint8_t n_obs, double sGPS, double sCourse, double sVelocity, double sYaw, double sAccel,
	//	double varGps, double varVelocity, double varYaw, double varAcc);
//void InitFusion(Fusion *fusion, uint8_t n_states, uint8_t n_obs, double sGPS, double sCourse, double sVelocity, double sYaw, double sAccel,
	//	double varGps, double varVelocity, double varYaw, double varAcc);

void InitFusion(Fusion *fusion, Model *model, EKFParams *params);

void startFusion(Fusion *fusion, Model *model);
void computeFusion(Fusion *fusion, Model *model, double lat, double lon, double alt, double vel,
		double course, double heading, double yaw_rate, double acc, double hdop, uint32_t dtime);
void processFusion(Fusion *fusion, Model *model, double lat, double lon, double alt, double vel,
		double course, double heading, double yaw_rate, double acc, double hdop, uint32_t dtime);

#endif /* INC_FUSION_H_ */
