/*
 * matrix.c
 *
 *  Created on: Jan 8, 2022
 *      Author: timagr615
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "matrix.h"
#include "utils.h"

extern UART_HandleTypeDef huart1;

void alloc_vector(Vector *v, uint8_t dim){
	//Vector v;
	v->dim = dim;
	v->data = (double*) malloc(sizeof(double)*v->dim);
	for (int i = 0; i < v->dim; ++i){
		v->data[i] = 0.0;
	}
	//return v;
}

void free_vector(Vector *v){
	free(v->data);
}


void multiply_matrix_vector(Matrix *m, Vector *a, Vector *b){
	assert_p(m->cols == a->dim);
	assert_p(m->rows == b->dim);
	for (int i = 0; i < m->rows; ++i){
		b->data[i] = 0.0;
		for(int j = 0; j < m->cols; ++j){
			b->data[i] += m->data[i][j]*a->data[j];
		}
	}
}

void alloc_matrix(Matrix *m, int rows, int cols)
{
	//Matrix m;
	m->rows = rows;
	m->cols = cols;
	m->data = (double**) malloc(sizeof(double*)*m->rows);
	for (int i = 0; i < m->rows; ++i){
		m->data[i] = (double*) malloc(sizeof(double)*m->cols);
		assert_p(m->data[i]);
		for (int j =0; j < m->cols; ++j){
			m->data[i][j] = 0.0;
		}
	}
	//return m;
}

void free_matrix(Matrix *m){
	assert_p(m->data != NULL);
	for (int i = 0; i < m->rows; ++i){
		free(m->data[i]);
	}
	free(m->data);
}

void set_matrix(Matrix *m, ...){
	va_list ap;
	va_start(ap, m);

	for (int i = 0; i < m->rows; ++i){
		for (int j = 0; j < m->cols; ++j){
			m->data[i][j] = va_arg(ap, double);
		}
	}
	va_end(ap);
}

void set_identity_matrix(Matrix *m){
	assert_p(m->rows == m->cols);
	for (int i = 0; i < m->rows; ++i){
		for (int j = 0; j < m->cols; ++j){
			if (i == j){
				m->data[i][j] = 1.0;
			} else {
				m->data[i][j] = 0.0;
			}
		}
	}
}

void copy_matrix(Matrix *source, Matrix *destination){
	assert_p(source->rows == destination->rows);
	assert_p(source->cols == destination->cols);
	for (int i = 0; i < source->rows; ++i){
		for (int j = 0; j < source->cols; ++j){
			destination->data[i][j] = source->data[i][j];
		}
	}
}

void print_matrix(Matrix *m){
	uint8_t str[100];
	for (int i = 0; i < m->rows; ++i){
		for (int j = 0; j < m->cols; ++j){
			if (j > 0){
				HAL_UART_Transmit(&huart1, " ", strlen((char *)(" ")), 0xFFFF);
			}
			print_mat(str, m->data[i][j]);
		}
		HAL_UART_Transmit(&huart1, "\n", strlen((char *)("\n")), 0xFFFF);
	}
	HAL_UART_Transmit(&huart1, "\n\r", strlen((char *)("\n\r")), 0xFFFF);
}

void add_matrix(Matrix *a, Matrix *b, Matrix *c){
	assert_p(a->rows == b->rows);
	assert_p(a->rows == c->rows);
	assert_p(a->cols == b->cols);
	assert_p(a->cols == c->cols);
	//uint8_t data[100];
	//char ok[] = "\n\r";
		//	HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);
	for (int i = 0; i < a->rows; ++i) {
		for (int j = 0; j < a->cols; ++j) {
			c->data[i][j] = a->data[i][j] + b->data[i][j];
			//print_float(data, a->data[i][j]);
			//print_float(data, b->data[i][j]);
			//print_float(data, c->data[i][j]);
		}
		//char ok[] = "\n\r";
		//HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);

	}
}

void subtract_matrix(Matrix *a, Matrix *b, Matrix *c) {
	assert_p(a->rows == b->rows);
	assert_p(a->rows == c->rows);
	assert_p(a->cols == b->cols);
	assert_p(a->cols == c->cols);
	for (int i = 0; i < a->rows; ++i) {
		for (int j = 0; j < a->cols; ++j) {
			c->data[i][j] = a->data[i][j] - b->data[i][j];
		}
	}
}

void subtract_from_identity_matrix(Matrix *a) {
	assert_p(a->rows == a->cols);
	for (int i = 0; i < a->rows; ++i) {
		for (int j = 0; j < a->cols; ++j) {
			if (i == j) {
				a->data[i][j] = 1.0 - a->data[i][j];
			} else {
				a->data[i][j] = 0.0 - a->data[i][j];
			}
		}
	}
}

void multiply_matrix(Matrix *a, Matrix *b, Matrix *c){
	assert_p(a->cols == b->rows);
	assert_p(a->rows == c->rows);
	assert_p(b->cols == c->cols);
	for (int i = 0; i < c->rows; ++i) {
	    for (int j = 0; j < c->cols; ++j) {
	    	c->data[i][j] = 0.0;
	    	for (int k = 0; k < a->cols; ++k){
	    		c->data[i][j] += a->data[i][k]*b->data[k][j];
	    	}
	    }
	}
}

/* This is multiplying a by b-tranpose so it is like multiply_matrix
   but references to b reverse rows and cols. */
void multiply_by_transpose_matrix(Matrix *a, Matrix *b, Matrix *c) {
	assert_p(a->cols == b->cols);
	assert_p(a->rows == c->rows);
	assert_p(b->rows == c->cols);
	for (int i = 0; i < c->rows; ++i) {
		for (int j = 0; j < c->cols; ++j) {
			/* Calculate element c.data[i][j] via a dot product of one row of a
	 	 	 with one row of b */
			c->data[i][j] = 0.0;
			for (int k = 0; k < a->cols; ++k) {
				c->data[i][j] += a->data[i][k] * b->data[j][k];
			}
		}
	}
}

void transpose_matrix(Matrix *input, Matrix *output) {
	assert_p(input->rows == output->cols);
	assert_p(input->cols == output->rows);
	for (int i = 0; i < input->rows; ++i) {
		for (int j = 0; j < input->cols; ++j) {
			output->data[j][i] = input->data[i][j];
		}
	}
}

int equal_matrix(Matrix *a, Matrix *b, double tolerance) {
	assert_p(a->rows == b->rows);
	assert_p(a->cols == b->cols);
	for (int i = 0; i < a->rows; ++i) {
		for (int j = 0; j < a->cols; ++j) {
			if (abs(a->data[i][j] - b->data[i][j]) > tolerance) {
				return 0;
			}
		}
	}
	return 1;
}

void scale_matrix(Matrix *m, double scalar) {
	assert_p(scalar != 0.0);
	for (int i = 0; i < m->rows; ++i) {
		for (int j = 0; j < m->cols; ++j) {
			m->data[i][j] *= scalar;
		}
	}
}

void swap_rows(Matrix *m, int r1, int r2) {
	assert_p(r1 != r2);
	double* tmp = m->data[r1];
	m->data[r1] = m->data[r2];
	m->data[r2] = tmp;
}

void scale_row(Matrix *m, int r, double scalar) {
	assert_p(scalar != 0.0);
	for (int i = 0; i < m->cols; ++i) {
		m->data[r][i] *= scalar;
	}
}

/* Add scalar * row r2 to row r1. */
void shear_row(Matrix *m, int r1, int r2, double scalar) {
	assert_p(r1 != r2);
	for (int i = 0; i < m->cols; ++i) {
		m->data[r1][i] += scalar * m->data[r2][i];
	}
}

/* Uses Gauss-Jordan elimination.
   The elimination procedure works by applying elementary row
   operations to our input matrix until the input matrix is reduced to
   the identity matrix.
   Simultaneously, we apply the same elementary row operations to a
   separate identity matrix to produce the inverse matrix.
   If this makes no sense, read wikipedia on Gauss-Jordan elimination.

   This is not the fastest way to invert matrices, so this is quite
   possibly the bottleneck. */

int destructive_invert_matrix(Matrix *input, Matrix *output) {
	assert_p(input->rows == input->cols);
	assert_p(input->rows == output->rows);
	assert_p(input->rows == output->cols);
	//print_matrix(input);
	set_identity_matrix(output);
	//print_matrix(output);
	/* Convert input to the identity matrix via elementary row operations.
     	 The ith pass through this loop turns the element at i,i to a 1
     	 and turns all other elements in column i to a 0. */
	for (int i = 0; i < input->rows; ++i) {
		//HAL_UART_Transmit(&huart1, "Inverse \n\r", strlen((char *)"Inverse \n\r"), 0xFFFF);
		if (input->data[i][i] == 0.0) {
			/* We must swap rows to get a nonzero diagonal element. */
			int r;
			for (r = i + 1; r < input->rows; ++r) {
				if (input->data[r][i] != 0.0) {
					break;
				}
			}
			if (r == input->rows) {
				/* Every remaining element in this column is zero, so this
	   	   matrix cannot be inverted. */
				return 0;
			}
			swap_rows(input, i, r);
			swap_rows(output, i, r);
		}

		/* Scale this row to ensure a 1 along the diagonal.
       	   We might need to worry about overflow from a huge scalar here. */
		double scalar = 1.0 / input->data[i][i];
		scale_row(input, i, scalar);
		scale_row(output, i, scalar);
		//print_matrix(output);

		/* Zero out the other elements in this column. */
		for (int j = 0; j < input->rows; ++j) {
			if (i == j) {
				continue;
			}
			double shear_needed = -input->data[j][i];
			shear_row(input, j, i, shear_needed);
			shear_row(output, j, i, shear_needed);
		}
	}
	//print_matrix(output);
	return 1;
}



