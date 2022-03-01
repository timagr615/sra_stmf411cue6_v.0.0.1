/*
 * ekf.c
 *
 *  Created on: 9 янв. 2022 г.
 *      Author: timagr615
 */


#include "ekf.h"
#include "math.h"
#include "fusion.h"
#include "utils.h"
//extern UART_HandleTypeDef huart1;

void alloc_filter(EKF *f, uint8_t state_dim, uint8_t obs_dim){
	//EKF f;
	f->state_dim = state_dim;
	f->obs_dim = obs_dim;


	alloc_vector(&f->state, state_dim);
	alloc_vector(&f->Hx, obs_dim);
	alloc_vector(&f->y, obs_dim);
	alloc_vector(&f->Ky, state_dim);

	alloc_matrix(&f->JA, state_dim, state_dim);
	alloc_matrix(&f->Q, state_dim, state_dim);
	alloc_matrix(&f->JH, obs_dim, state_dim);
	alloc_matrix(&f->R, obs_dim, obs_dim);
	alloc_matrix(&f->P, state_dim, state_dim);

	alloc_matrix(&f->JAP, state_dim, state_dim);
	alloc_matrix(&f->Pp, state_dim, state_dim);
	alloc_matrix(&f->PJHt, state_dim, obs_dim);
	alloc_matrix(&f->JHPJHt, obs_dim, obs_dim);

	alloc_matrix(&f->S, obs_dim, obs_dim);
	alloc_matrix(&f->SInv, obs_dim, obs_dim);
	alloc_matrix(&f->K , state_dim, obs_dim);
	alloc_matrix(&f->KJH, state_dim, state_dim);
	//return f;


}

void free_filter(EKF *f){
	free_vector(&f->state);
	free_vector(&f->Hx);
	free_vector(&f->y);
	free_vector(&f->Ky);

	free_matrix(&f->JA);
	free_matrix(&f->Q);
	free_matrix(&f->JH);
	free_matrix(&f->R);
	free_matrix(&f->P);
	free_matrix(&f->JAP);
	free_matrix(&f->Pp);
	free_matrix(&f->PJHt);
	free_matrix(&f->JHPJHt);
	free_matrix(&f->S);
	free_matrix(&f->SInv);
	free_matrix(&f->K);
	free_matrix(&f->KJH);
}

/**************** CRTA MODEL **************************/

void updateJA_CTRA(EKF *f, const double dt){
	/*******************************************



	        x = x + (-ψ˙v sin(ψ)-a cos(ψ)+a cos(dtψ˙+ψ)+(dtψ˙a+ψ˙v)(sin(dtψ˙+ψ))/ψ˙**2
	        y = y + (ψ˙v cos(ψ)-a sin(ψ)+a sin(dtψ˙+ψ)+(-dtψ˙a-ψ˙v)cos(dtψ˙+ψ))/ψ˙**2
	        ψ = ψ +dtψ˙
	        v = v + adt
	        ψ˙ = ψ˙
	        a = a
	     *******************************************/
	if (fabs(f->state.data[4]) < 0.01){
		f->state.data[0] = f->state.data[0] + (f->state.data[2]*dt)*cos(f->state.data[3])+(f->state.data[5]*dt*dt/2)*cos(f->state.data[3]);
		f->state.data[1] = f->state.data[1] + (f->state.data[2]*dt)*sin(f->state.data[3])+(f->state.data[5]*dt*dt/2)*sin(f->state.data[3]);
		f->state.data[2] = f->state.data[2] + f->state.data[5]*dt;
		f->state.data[3] = f->state.data[3];

		f->state.data[4] = 0.0000001;
		f->state.data[5] = f->state.data[5];
	}
	else {
		f->state.data[0] = f->state.data[0] +
				(1/f->state.data[4]/f->state.data[4])*
				((f->state.data[2]*f->state.data[4]+f->state.data[4]*f->state.data[5]*dt)*sin(f->state.data[4]*dt+f->state.data[3])+
				+f->state.data[5]*cos(f->state.data[3]+f->state.data[4]*dt)-f->state.data[2]*f->state.data[4]*sin(f->state.data[3])-
				-f->state.data[5]*cos(f->state.data[3]));
		f->state.data[1] = f->state.data[1] +
				(1/f->state.data[4]/f->state.data[4])*
				((-f->state.data[2]*f->state.data[4]-f->state.data[4]*f->state.data[5]*dt)*cos(f->state.data[4]*dt+f->state.data[3])+
				+f->state.data[5]*sin(f->state.data[2]+f->state.data[4]*dt)+f->state.data[2]*f->state.data[4]*cos(f->state.data[3])-
				-f->state.data[5]*sin(f->state.data[3]));


		f->state.data[2] = f->state.data[2] + f->state.data[5]*dt;
		f->state.data[3] = fmod((f->state.data[3]+f->state.data[4]*dt + M_PI), (2.0*M_PI)) - M_PI;

		f->state.data[4] = f->state.data[4];
		f->state.data[5] = f->state.data[5];
	}
	calculate_J_CTRA(f, dt);
	/*char ok[] = "\n\r STATE PARAMS IN UPDATE JA; \n\r";
	HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);
	uint8_t data[100];
	print_float(data, f->state.data[0]);
	print_float(data, f->state.data[1]);
	print_float(data, f->state.data[2]);
	print_float(data, f->state.data[3]);
	print_float(data, f->state.data[4]);
	print_float(data, f->state.data[5]);*/
}

void calculate_J_CTRA(EKF *f, const double dt){
	const double psi = f->state.data[3];
	const double v = f->state.data[2];
	const double dpsi = f->state.data[4];
	const double a = f->state.data[5];

	const double THRESHOLD = 0.01;
	if(dpsi > THRESHOLD) // Avoid deviding by zero
	{
		//const double tr = v/dpsi;
		const double dpsi_inv = 1/dpsi/dpsi;
		const double dpp = dt*dpsi + psi;

		const double r14 = dpsi_inv*(-dpsi*v*cos(psi)+a*sin(psi)-a*sin(dpp)+(dt*dpsi*a+dpsi*v)*cos(dpp));
		const double r13 = dpsi_inv*(-dpsi*sin(psi)+dpsi*sin(dpp));
		const double r15 = dpsi_inv*(-dt*a*sin(dpp)+dt*(dt*dpsi*a+dpsi*v)*cos(dpp)-v*sin(psi)+(dt*a+v)*sin(dpp))-
				2*dpsi_inv/dpsi*(-dpsi*v*sin(psi)-a*cos(psi)+a*cos(dpp)+(dt*dpsi*a+dpsi*v)*sin(dpp));
		const double r16 = dpsi_inv*(dt*dpsi*sin(dpp)-cos(psi)+cos(dpp));

		const double r24 = dpsi_inv*(-dpsi*v*sin(psi)-a*cos(psi)+a*cos(dpp)+(dt*dpsi*a+dpsi*v)*sin(dpp));
		const double r23 = dpsi_inv*(dpsi*cos(psi)-dpsi*cos(dpp));
		const double r25 = dpsi_inv*(dt*a*cos(dpp)+dt*(dt*dpsi*a+dpsi*v)*sin(dpp)+v*cos(psi)-(dt*a+v)*cos(dpp))-
				2*dpsi_inv/dpsi*(dpsi*v*cos(psi)-a*sin(psi)+a*sin(dpp)-(dt*dpsi*a+dpsi*v)*cos(dpp));
		const double r26 = dpsi_inv*(-dt*dpsi*cos(dpp)-sin(psi)+sin(dpp));

		set_matrix(&f->JA,
				1.0, 0.0, r13, r14, r15, r16,
				0.0, 1.0, r23, r24, r25, r26,
				0.0, 0.0, 1.0, 0.0, 0.0,  dt,
				0.0, 0.0, 0.0, 1.0, dt,  0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	}
	else {
		const double r13 = -v*sin(psi)*dt-a*sin(psi)*dt*dt/2;
		const double r14 = cos(psi)*dt;
		const double r16 = cos(psi)*dt*dt/2;

		const double r23 = v*cos(psi)*dt+a*cos(psi)*dt*dt/2;
		const double r24 = sin(psi)*dt;
		const double r26 = sin(psi)*dt*dt/2;
		set_matrix(&f->JA,
				1.0, 0.0, r13, r14, 0.0, r16,
				0.0, 1.0, r23, r24, 0.0, r26,
				0.0, 0.0, 1.0, 0.0, 0.0, dt,
				0.0, 0.0, 0.0, 1.0, dt, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	}
}


/*************** CRTV MODEL ************************/

void updateJA_CTRV(EKF *f, const double dt)
{
	/* State Equation Update Rule
		        x + v/ψ˙(−sin(ψ) + sin(dtψ˙+ψ))
		        y + v/ψ˙(cos(ψ) − cos(dtψ˙+ψ))
		        dtψ˙+ ψ
		        dta + v
		        ψ˙
		        a
		        */
	if (fabs(f->state.data[4]) < 0.01){
		f->state.data[0] = f->state.data[0] + (f->state.data[2] * dt) * cos(f->state.data[3]);
		f->state.data[1] = f->state.data[1] + (f->state.data[2] * dt) * sin(f->state.data[3]);
		f->state.data[2] = f->state.data[2] + f->state.data[5] * dt;
		f->state.data[3] = f->state.data[3];
		f->state.data[4] = 0.0000001;
		f->state.data[5] = f->state.data[5];

	} else {
		f->state.data[0] = f->state.data[0] + (f->state.data[2]/f->state.data[4]) * (sin(f->state.data[4] * dt + f->state.data[3])
				- sin(f->state.data[3]));
		f->state.data[1] = f->state.data[1] + (f->state.data[2]/f->state.data[4]) * (-cos(f->state.data[4] * dt + f->state.data[3])
				+ cos(f->state.data[3]));
		f->state.data[2] = f->state.data[2] + f->state.data[5] * dt;
		f->state.data[3] = fmod((f->state.data[3] + f->state.data[4] * dt + M_PI), (2.0 * M_PI)) - M_PI;

		f->state.data[4] = f->state.data[4];
		f->state.data[5] = f->state.data[5];
	}
	calculate_J_CTRV(f, dt);
}
void calculate_J_CTRV(EKF *f, const double dt)
{

	const double velocity = f->state.data[2];
	const double psi_dot = f->state.data[4];
	const double psi = f->state.data[3];
	//const double v = f->state.data[3];
	//const double dpsi = f->state.data[4];
	//const double a = f->state.data[5];

	const double THRESHOLD = 0.01;
	if(psi_dot > THRESHOLD) // Avoid deviding by zero
	{
		const double turn_radius = (velocity/psi_dot);
		const double psi_dot_inverse = 1/psi_dot;
		const double pdotp = dt * psi_dot + psi;

		//const double r13 = turn_radius * (-cos(dt * psi_dot) + psi);
		const double r14 =  turn_radius * (cos(pdotp) - cos(psi));
		const double r13 = psi_dot_inverse * (-sin(psi) + sin(pdotp));
		const double r15 = dt * turn_radius * cos(pdotp) - (turn_radius/psi_dot) * (-sin(psi) + sin(pdotp));

		const double r24 = turn_radius * (-sin(psi) + sin(pdotp));
		const double r23 = psi_dot_inverse * (cos(psi) - cos(pdotp));
		const double r25 = dt * turn_radius * sin(pdotp) - (turn_radius/psi_dot) * (cos(psi) - cos(pdotp));

		set_matrix(&f->JA,
				1.0, 0.0, r13, r14, r15, 0.0,
				0.0, 1.0, r23, r24, r25, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0,  dt,
				0.0, 0.0, 0.0, 1.0, dt,  0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	}
	else {
		const double r13 = -velocity*sin(psi)*dt;
		const double r14 = cos(psi)*dt;

		const double r23 = velocity*cos(psi)*dt;
		const double r24 = sin(psi)*dt;

		set_matrix(&f->JA,
				1.0, 0.0, r13, r14, 0.0, 0.0,
				0.0, 1.0, r23, r24, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0, dt,
				0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	}
}

/********************** CHCV MODEL  *************************/

void updateJA_CHCV(EKF *f, const double dt){

	f->state.data[0] = f->state.data[0] + (f->state.data[2] * dt) * cos(f->state.data[3]) + (f->state.data[4]*0.5*dt*dt)*cos(f->state.data[3]);
	f->state.data[1] = f->state.data[1] + (f->state.data[2] * dt) * sin(f->state.data[3]) + (f->state.data[4]*0.5*dt*dt)*sin(f->state.data[3]);
	f->state.data[2] = f->state.data[2] + f->state.data[4] * dt;
	f->state.data[3] = f->state.data[3];
	f->state.data[4] = f->state.data[4];

	calculate_J_CHCV(f, dt);

	/*char ok[] = "\n\r STATE PARAMS IN UPDATE JA; \n\r";
	HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);
	uint8_t data[100];
	print_float(data, f->state.data[0]);
	print_float(data, f->state.data[1]);
	print_float(data, f->state.data[2]);
	print_float(data, f->state.data[3]);
	print_float(data, f->state.data[4]);*/
	//	print_float(data, f->state.data[5]);
}
void calculate_J_CHCV(EKF *f, const double dt){
	const double v = f->state.data[2];
	const double psi = f->state.data[3];
	const double a = f->state.data[4];

	const double r14 = -v*dt*sin(psi) - 0.5*a*dt*dt*sin(psi);
	const double r13 = dt*cos(psi);
	const double r15 = 0.5*dt*dt*cos(psi);

	const double r24 = v*dt*cos(psi) + 0.5*a*dt*dt*cos(psi);
	const double r23 = dt*sin(psi);
	const double r25 = 0.5*dt*dt*sin(psi);

	set_matrix(&f->JA,
			1.0, 0.0, r13, r14, r15,
			0.0, 1.0, r23, r24, r25,
			0.0, 0.0, 1.0, 0.0, dt,
			0.0, 0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 1.0);
}

void predict(EKF *f){
	// P = JA*P*JA_T + Q
	/*char okk0[] = "\n\r MATRIX JA \n\r";
			//uint8_t data[100];
				HAL_UART_Transmit(&huart1, okk0, strlen((char *)okk0), 0xFFFF);
			print_matrix(&f->JA);*/
	multiply_matrix(&f->JA, &f->P, &f->JAP);
	multiply_by_transpose_matrix(&f->JAP, &f->JA, &f->Pp);
	add_matrix(&f->Pp, &f->Q, &f->Pp);
}

void computeHx(EKF *f, Model *model){
	if (model->model == CTRA || model->model == CTRV){
		f->Hx.data[0] = f->state.data[0];
		f->Hx.data[1] = f->state.data[1];
		f->Hx.data[2] = f->state.data[2];
		f->Hx.data[3] = f->state.data[4];
		f->Hx.data[4] = f->state.data[5];
	} else {
		f->Hx.data[0] = f->state.data[0];
		f->Hx.data[1] = f->state.data[1];
		f->Hx.data[2] = f->state.data[2];
		f->Hx.data[3] = f->state.data[3];
		f->Hx.data[4] = f->state.data[4];
	}


}

void computeJH(EKF *f, Model *model, uint8_t dataType){
	if (dataType == GPS){
		if (model->model == CTRA || model->model == CTRV){
			set_matrix(&f->JH,
					1.0,0.0,0.0,0.0,0.0,0.0,
					0.0,1.0,0.0,0.0,0.0,0.0,
					0.0,0.0,1.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,1.0,0.0,
					0.0,0.0,0.0,0.0,0.0,1.0);
		} else {
			set_matrix(&f->JH,
					1.0,0.0,0.0,0.0,0.0,
					0.0,1.0,0.0,0.0,0.0,
					0.0,0.0,1.0,0.0,0.0,
					0.0,0.0,0.0,1.0,0.0,
					0.0,0.0,0.0,0.0,1.0);
		}

	} else {
		if (model->model == CTRA || model->model == CTRV){
			set_matrix(&f->JH,
					0.0,0.0,0.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,1.0,0.0,
					0.0,0.0,0.0,0.0,0.0,1.0);
		} else {
			set_matrix(&f->JH,
					0.0,0.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,0.0,
					0.0,0.0,0.0,0.0,1.0);
		}
	}

}

void update(EKF *f, Model *model, Vector *z, uint8_t dataType){

	/************************************************
	  	 z - observation vector x, y, velocity, yaw_rate, acceleration
	 	 S = JH*P*JH_T + R
	 	 K = P*JH_T*S^-1
	 	 y = Z - Hx
	 	 state = state + K*y
	 	 P = (I - K*JH)*P
	 	 *************************/

	/*char ok[] = "\n\r";
					  HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);
			uint8_t data[100];
			for (int i = 0; i < f->S.rows; ++i){
				for (int j = 0; j < f->S.cols; ++j){
					print_float(data, f->S.data[i][j]);
				}
				char ok[] = "\n\r";
								  HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);
			}*/

	computeHx(f, model);
	computeJH(f, model, dataType);

	/*char okk01[] = "\n\r MATRIX Pp \n\r";
			uint8_t data[100];
				HAL_UART_Transmit(&huart1, okk01, strlen((char *)okk01), 0xFFFF);
			print_matrix(&f->PJHt);*/
	multiply_by_transpose_matrix(&f->Pp, &f->JH, &f->PJHt);
	multiply_matrix(&f->JH, &f->PJHt, &f->JHPJHt);
	add_matrix(&f->JHPJHt, &f->R, &f->S);

	/*char okk0[] = "\n\r MATRIX PJHt \n\r";
		//uint8_t data[100];
			HAL_UART_Transmit(&huart1, okk0, strlen((char *)okk0), 0xFFFF);
		print_matrix(&f->PJHt);

	char okk1[] = "\n\r MATRIX S \n\r";
	//uint8_t data[100];
		HAL_UART_Transmit(&huart1, okk1, strlen((char *)okk1), 0xFFFF);
	print_matrix(&f->S);*/

	destructive_invert_matrix(&f->S, &f->SInv);

	multiply_matrix(&f->PJHt, &f->SInv, &f->K);

	/*char okk[] = "\n\r MATRIX K \n\r";
	HAL_UART_Transmit(&huart1, okk, strlen((char *)okk), 0xFFFF);

	print_matrix(&f->K);*/


	/////// y = Z - Hx //////////
	for (int i = 0; i < f->y.dim; ++i){
		f->y.data[i] = z->data[i] - f->Hx.data[i];
	}

	char ok0[] = "\n\r  Z  \n\r";
	printf(ok0);
	//HAL_UART_Transmit(&huart1, ok0, strlen((char *)ok0), 0xFFFF);
	uint8_t data[100];
	print_float(data, z->data[0]);
	print_float(data, z->data[1]);
	//print_float(data, z->data[2]);
	//print_float(data, z->data[3]);
	//print_float(data, z->data[4]);

	/*char ok00[] = "\n\r Y = Z - h(x) \n\r";
	HAL_UART_Transmit(&huart1, ok00, strlen((char *)ok00), 0xFFFF);
	//uint8_t data[100];
	print_float(data, f->y.data[0]);
	print_float(data, f->y.data[1]);
	print_float(data, f->y.data[2]);
	print_float(data, f->y.data[3]);
	print_float(data, f->y.data[4]);*/
	//print_float(data, f->y.data[5]);
	multiply_matrix_vector(&f->K, &f->y, &f->Ky);


	/*char ok1[] = "\n\r MATRIX K \n\r";
	HAL_UART_Transmit(&huart1, ok1, strlen((char *)ok1), 0xFFFF);

	print_matrix(&f->K);*/

	for (int i = 0; i < f->state.dim; ++i){
		f->state.data[i] = f->state.data[i] + f->Ky.data[i];
	}
	/*char ok2[] = "\n\r Ky = K*y \n\r";
	HAL_UART_Transmit(&huart1, ok2, strlen((char *)ok2), 0xFFFF);
	//uint8_t data[100];
	print_float(data, f->Ky.data[0]);
	print_float(data, f->Ky.data[1]);
	print_float(data, f->Ky.data[2]);
	print_float(data, f->Ky.data[3]);
	print_float(data, f->Ky.data[4]);
	print_float(data, f->Ky.data[5]);*/

	//char ok3[] = "\n\r STATE VECTOR in UPDATE \n\r";
	//HAL_UART_Transmit(&huart1, ok3, strlen((char *)ok3), 0xFFFF);
	//uint8_t data[100];
	print_float(data, f->state.data[0]);
	print_float(data, f->state.data[1]);
	//print_float(data, f->state.data[2]);
	//print_float(data, f->state.data[3]);
	//print_float(data, f->state.data[4]);
	//print_float(data, f->state.data[5]);

	multiply_matrix(&f->K, &f->JH, &f->KJH);
	subtract_from_identity_matrix(&f->KJH);
	multiply_matrix(&f->KJH, &f->Pp, &f->P);
}

void step(EKF *f, Model *model, Vector *z, uint8_t dataType, const double dt){
	//updateJA(f, dt);
	if (model->model == CTRA){
		updateJA_CTRA(f, dt);
	} else if (model->model == CTRV){
		updateJA_CTRV(f, dt);
	} else {
		updateJA_CHCV(f, dt);
	}


	predict(f);
	update(f, model, z, dataType);
}

void updateQ(EKF *f, Model *model, double sGPS, double sCourse, double sVelocity, double sYaw, double sAcceleration, const double dt){
	sGPS *= 0.5*dt*dt;
	sCourse *= dt;
	sVelocity *= dt;
	sYaw *= dt;
	if (model->model == CTRA || model->model == CTRV){
		set_matrix(&f->Q,
				pow(sGPS,2), 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, pow(sGPS,2), 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, pow(sVelocity,2), 0.0, 0.0,
				0.0, 0.0, 0.0, pow(sCourse,2), 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, pow(sYaw,2), 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, pow(sAcceleration,2));
	} else {
		set_matrix(&f->Q,
				pow(sGPS,2), 0.0, 0.0, 0.0, 0.0,
				0.0, pow(sGPS,2), 0.0, 0.0, 0.0,
				0.0, 0.0, pow(sVelocity,2), 0.0, 0.0,
				0.0, 0.0, 0.0, pow(sCourse,2), 0.0,
				0.0, 0.0, 0.0, 0.0, pow(sAcceleration,2));
	}


}
void updateR(EKF *f, Model *model, double varGPS, double varSpeed, double varCourse, double varYaw, double varAcc){
	if (model->model == CTRA || model->model == CTRV){
		set_matrix(&f->R,
				pow(varGPS,2), 0.0, 0.0, 0.0, 0.0,
				0.0, pow(varGPS, 2), 0.0, 0.0, 0.0,
				0.0, 0.0, pow(varSpeed, 2), 0.0, 0.0,
				0.0, 0.0, 0.0, pow(varYaw, 2), 0.0,
				0.0, 0.0, 0.0, 0.0, pow(varAcc, 2));
	} else {
		set_matrix(&f->R,
				pow(varGPS,2), 0.0, 0.0, 0.0, 0.0,
				0.0, pow(varGPS, 2), 0.0, 0.0, 0.0,
				0.0, 0.0, pow(varSpeed, 2), 0.0, 0.0,
				0.0, 0.0, 0.0, pow(varCourse, 2), 0.0,
				0.0, 0.0, 0.0, 0.0, pow(varAcc, 2));
	}


}
void setPInit(EKF *f, Model *model){
	if (model->model == CTRA || model->model == CTRV){
		set_matrix(&f->P,
				1000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 1000.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1000.0);
	} else {
		set_matrix(&f->P,
				1000.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 1000.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 1000.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 1000.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1000.0);
	}

}







