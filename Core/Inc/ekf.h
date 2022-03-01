/*
 * ekf.h
 *
 *  Created on: 9 янв. 2022 г.
 *      Author: timagr615
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include "matrix.h"
#include "stdint.h"

enum ModelType {
	CTRA = 0,
	CTRV,
	CHCV
};

typedef struct {
	uint8_t model;
	uint8_t n_states;
	uint8_t n_obs;
} Model;

typedef struct {
	//int timestamp;
	uint8_t state_dim; // x, y, heading, velocity, yaw rate, acceleration
	uint8_t obs_dim;  // x, y, velocity, yaw rate, acceleration

	//////////// PREDICTION ///////////
	// state_k = f(state_k-1)
	// calculate JA
	// P = JA*P*JA_T + Q
	//////////// CORRECTION ///////////
	// Calculate Hx
	//Calculate JH
	// S = JH*P*JH_T + R
	// K = P*JH_T*S^-1
	// y = Z - Hx
	// state = state + K*y
	// P = (I - K*JH)*P




	Vector state; // State - x y heading velocity yaw_rat long_acceleration 6-dim
	Vector Hx; // Mesurement function x, y, velocity, yaw_rate, acceleration 5-dim
	Vector y; // Innovation or Residual 5-dim
	Vector Ky; // 6-dim


	/* This group of matrices must be specified by the user. */
	Matrix JA; // Jacobian state matrix 6x6
	Matrix Q; // process noise covariance 6x6
	Matrix JH; // Jacobian model matrix 5x6
	Matrix R; // observation noice covariance 5x5
	Matrix P; // initial covariance/uncertainity in states 6x6

	Matrix JAP; //6x6
	Matrix Pp; // 6x6
	Matrix PJHt; //6x5
	Matrix JHPJHt; //5x5

	Matrix S; //5x5
	Matrix SInv; //5x5
	Matrix K; // 6x5

	Matrix KJH; // 6x6




} EKF;

void alloc_filter(EKF *f, uint8_t state_dim, uint8_t obs_dim);
void free_filter(EKF *f);
void update(EKF *f, Model *model, Vector *z, uint8_t dataType);
void predict(EKF *f);
void updateJA_CTRA(EKF *f, const double dt);
void calculate_J_CTRA(EKF *f, const double dt);
void updateJA_CTRV(EKF *f, const double dt);
void calculate_J_CTRV(EKF *f, const double dt);
void updateJA_CHCV(EKF *f, const double dt);
void calculate_J_CHCV(EKF *f, const double dt);
void updateJH(EKF *f);
void computeHx(EKF *f, Model *model);
void computeJH(EKF *f, Model *model, uint8_t dataType);
void step(EKF *f, Model *model, Vector *z, uint8_t dataType, const double dt);
void updateQ(EKF *f, Model *model, double sGPS, double sCourse, double sVelocity, double sYaw, double sAcceleration, const double dt);
void updateR(EKF *f, Model *model, double varGPS, double varSpeed, double varCourse, double varYaw, double varAcc);
void setPInit(EKF *f, Model *model);




#endif /* INC_EKF_H_ */
