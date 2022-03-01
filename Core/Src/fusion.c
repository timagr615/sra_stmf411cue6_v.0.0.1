/*
 * fusion.c
 *
 *  Created on: 12 янв. 2022 г.
 *      Author: timagr615
 */

#include "fusion.h"
#include "matrix.h"
#include "utils.h"

extern UART_HandleTypeDef huart1;


void dataPointInit(DataPoint *point, Model *model){
	//DataPoint point;
	point->first_data_point = true;
	//point.initialized = true;
	point->timestamp = 0;
	point->data_type = IMU;
	point->hdop = 0.0;
	point->dt = 0.0;

	point->first_point.lat = 0.0;
	point->first_point.lon = 0.0;

	point->prev_point.lat = 0.0;
	point->prev_point.lon = 0.0;
	//point.curr_point.lat = 0.0;
	//point.curr_point.lon = 0.0;
	point->dx = 0.0;
	point->dy = 0.0;
	point->dx_first = 0.0;
	point->dy_first = 0.0;
	point->mx = 0.0;
	point->my = 0.0;
	point->ds = 0.0;

	alloc_vector(&point->raw_data, 8);
	alloc_vector(&point->z, model->n_obs);
	//return point;
}

void getRawData(DataPoint *point, double lat, double lon, double alt, double vel, double course, double heading, double yaw_rate,
		double acc, double hdop, uint32_t dtime){
	//uint8_t data[100];

	point->raw_data.data[0] = lat;
	point->raw_data.data[1] = lon;
	point->raw_data.data[2] = alt;
	point->raw_data.data[3] = vel;
	point->raw_data.data[4] = course;
	point->raw_data.data[5] = heading;
	point->raw_data.data[6] = yaw_rate;
	point->raw_data.data[7] = acc;
	point->timestamp = dtime;
	point->hdop = hdop;
	//print_float(data, lat);
	point->dt = (double)point->timestamp/1000;
}

void setData(DataPoint *point, Model *model, double lat, double lon, double alt,  double vel, double course, double heading, double yaw_rate,
		double acc, double hdop, uint32_t dtime){
	getRawData(point, lat, lon, alt, vel, course, heading, yaw_rate, acc, hdop, dtime);
	if (point->first_data_point == true && point->raw_data.data[0] != 0.0 && point->raw_data.data[1] != 0.0){
		point->dx = 0.0;
		point->dy = 0.0;
		point->mx = 0.0;
		point->my = 0.0;
		point->ds = 0.0;
		point->dx_first = CoordLongitudeToMeters(point->raw_data.data[1], 0.0, point->raw_data.data[2]);
		point->dy_first = CoordLatitudeToMeters(point->raw_data.data[0], 0.0, point->raw_data.data[2]);
		point->first_point.lat = point->raw_data.data[0];
		point->first_point.lon = point->raw_data.data[1];
		point->prev_point.lat = point->raw_data.data[0];
		point->prev_point.lon = point->raw_data.data[1];
		point->first_data_point = false;

	}
	else if (point->first_data_point != true){
		point->dx = CoordLongitudeToMeters(point->raw_data.data[1], 0.0, point->raw_data.data[2])-point->dx_first;
		point->dy = CoordLatitudeToMeters(point->raw_data.data[0], 0.0, point->raw_data.data[2])-point->dy_first;

		//point->dx = CoordLongitudeToMeters(point->raw_data.data[1], point->prev_point.lon, point->raw_data.data[5]);
		//point->dy = CoordLatitudeToMeters(point->raw_data.data[0], point->prev_point.lat, point->raw_data.data[5]);
		point->ds = sqrt(point->dx*point->dx + point->dy*point->dy);
		if (point->ds == 0.0){
			point->data_type = IMU;
		}
		else {
			point->data_type = GPS;
		}
		//point->mx += point->dx;
		//point->my += point->dy;
		point->mx = point->dx;
		point->my = point->dy;
		point->prev_point.lat = point->raw_data.data[0];
		point->prev_point.lon = point->raw_data.data[1];
	}

	if (model->model == CTRA || model->model == CTRV){
		point->z.data[0] = point->mx;  // x
		point->z.data[1] = point->my;  // y
		point->z.data[2] = point->raw_data.data[3]; // velocity
		point->z.data[3] = point->raw_data.data[6]; // yaw_rate
		point->z.data[4] = point->raw_data.data[4]; // acceleration
	} else {
		// CHCV model
		point->z.data[0] = point->mx;  // x
		point->z.data[1] = point->my;  // y
		point->z.data[2] = point->raw_data.data[3]; // velocity
		point->z.data[3] = point->raw_data.data[4]; // course
		point->z.data[4] = point->raw_data.data[7]; // acceleration
	}

}


void InitFusion(Fusion *fusion, Model *model, EKFParams *params){

	dataPointInit(&fusion->Point, model);
	alloc_filter(&fusion->KF, model->n_states, model->n_obs);

	fusion->initialized = false;
	fusion->sGPS = params->sGPS;
	fusion->sCourse = params->sCourse;
	fusion->sVelocity = params->sVelocity;
	fusion->sYaw = params->sYaw;
	fusion->sAccel = params->sAcceleration;
	fusion->varGps = params->varGps;
	fusion->varVelocity = params->varVelocity;
	fusion->varCourse = params->varCourse;
	fusion->varYaw = params->varYaw;
	fusion->varAcc = params->varAcceleration;

}

void startFusion(Fusion *fusion, Model *model){
	setPInit(&fusion->KF, model);
	fusion->initialized = true;

	//print_float(ok, (float)fusion.initialized);
}

void computeFusion(Fusion *fusion, Model *model, double lat, double lon, double alt, double vel,
		double course, double heading, double yaw_rate, double acc, double hdop, uint32_t dtime){
	setData(&fusion->Point, model, lat, lon, alt, vel, course, heading, yaw_rate, acc, hdop, dtime);
	if (fusion->Point.hdop == 0.0){
		fusion->Point.hdop = 1;
	}
	double varGp = fusion->Point.hdop*fusion->varGps;
	double varV = fusion->Point.hdop*fusion->varVelocity;
	//uint8_t data[100];
	//print_float(data, fusion->Point.hdop);
	updateR(&fusion->KF, model, varGp, varV, fusion->varCourse, fusion->varYaw, fusion->varAcc);
	updateQ(&fusion->KF, model, fusion->sGPS, fusion->sCourse, fusion->sVelocity, fusion->sYaw, fusion->sAccel, fusion->Point.dt);
	step(&fusion->KF, model, &fusion->Point.z, fusion->Point.data_type, fusion->Point.dt);
}

void processFusion(Fusion *fusion, Model *model, double lat, double lon, double alt, double vel,
		double course, double heading, double yaw_rate, double acc, double hdop, uint32_t dtime){

	if (fusion->initialized == true){
		computeFusion(fusion, model, lat, lon, alt, vel, course, heading, yaw_rate, acc, hdop, dtime);
	}
	else {
		startFusion(fusion, model);
	}
}
