/*
 * matrix.h
 *
 *  Created on: Jan 8, 2022
 *      Author: timagr615
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_
#include "stdint.h"

typedef struct {
	int dim;
	double* data;
} Vector;

typedef struct {
	int rows;
	int cols;

	double** data;
	//double data[rows][cols];
} Matrix;


/* Allocate memory for a new vector
 	 Zeros out the vector.
   Assert-fails if we are out of memory.
 */

void alloc_vector(Vector *v, uint8_t dim);

void free_vector(Vector *v);

/* Multiply matrix by vector*/
void multiply_matrix_vector(Matrix *m, Vector *a, Vector *b);


/* Allocate memory for a new matrix.
   Zeros out the matrix.
   Assert-fails if we are out of memory.
*/

void alloc_matrix(Matrix *m, int rows, int cols);

/*Free memory for a matrix */
void free_matrix(Matrix *m);

/* Set values of a matrix, row by row. */
void set_matrix(Matrix *m, ...);

/* Turn m into an identity matrix. */
void set_identity_matrix(Matrix *m);

/* Copy a matrix. */
void copy_matrix(Matrix *source, Matrix *destination);

/* Pretty-print a matrix. */
void print_matrix(Matrix *m);

/* Add matrices a and b and put the result in c. */
void add_matrix(Matrix *a, Matrix *b, Matrix *c);

/* Subtract matrices a and b and put the result in c. */
void subtract_matrix(Matrix *a, Matrix *b, Matrix *c);

/* Subtract from the identity matrix in place. */
void subtract_from_identity_matrix(Matrix *a);

/* Multiply matrices a and b and put the result in c. */
void multiply_matrix(Matrix *a, Matrix *b, Matrix *c);

/* Multiply matrix a by b-transpose and put the result in c. */
void multiply_by_transpose_matrix(Matrix *a, Matrix *b, Matrix *c);

/* Transpose input and put the result in output. */
void transpose_matrix(Matrix *input, Matrix *output);

/* Whether two matrices are approximately equal. */
int equal_matrix(Matrix *a, Matrix *b, double tolerance);

/* Multiply a matrix by a scalar. */
void scale_matrix(Matrix *m, double scalar);

/* Swap rows r1 and r2 of a matrix.
   This is one of the three "elementary row operations". */
void swap_rows(Matrix *m, int r1, int r2);

/* Multiply row r of a matrix by a scalar.
   This is one of the three "elementary row operations". */
void scale_row(Matrix *m, int r, double scalar);

/* Add a multiple of row r2 to row r1.
   Also known as a "shear" operation.
   This is one of the three "elementary row operations". */
void shear_row(Matrix *m, int r1, int r2, double scalar);

/* Invert a square matrix.
   Returns whether the matrix is invertible.
   input is mutated as well by this routine. */
int destructive_invert_matrix(Matrix *input, Matrix *output);

#endif /* INC_MATRIX_H_ */
