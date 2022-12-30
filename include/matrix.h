#ifndef MATRIX_H_
#define MATRIX_H_

/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Matrix libraries for physics
 * @addtogroup MATRIX
 **/

#define NULL ((void *)0)

#define ELEM(mtx, row, col) \
  mtx->data[(col-1) * mtx->rows + (row-1)]

/*
///////////////////////////////////////

MATRIX AND VECTOR DEFINITIONS AND OPERATIONS

A matrix has (rows, cols) dimensions.
A vector has (rows, 1) dimensions.

Source: http://theory.stanford.edu/~arbrad/pfe/06/matrix.c

///////////////////////////////////////
*/
typedef struct 
{
  int rows;
  int cols;
  float * data;
} matrix;

/*
********************************************
** FUNCTION DECLERATIONS
********************************************
*/

matrix * newMatrix(int rows, int cols);
int deleteMatrix(matrix * mtx);
matrix * copyMatrix(matrix * mtx);
int setMatrixElement(matrix * mtx, int row, int col, float val);
int getMatrixElement(matrix * mtx, int row, int col, float * val);
int nRows(matrix * mtx, int * n);
int nCols(matrix * mtx, int * n);
int printMatrix(matrix * mtx);
int transposeMatrix(matrix * in, matrix * out);
int sumMatrix(matrix * mtx1, matrix * mtx2, matrix * sum);
int productMatrix(matrix * mtx1, matrix * mtx2, matrix * prod);
int dotProductMatrix(matrix * v1, matrix * v2, float * prod);
int identityMatrix(matrix * m);
int isMatrixSquare(matrix * mtx);
int isMatrixDiagonal(matrix * mtx);
int isMatrixUpperTriangular(matrix * mtx);
int diagonalMatrix(matrix * v, matrix * mtx);



#endif // MATRIX_H_