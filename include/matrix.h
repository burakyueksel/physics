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
int matrixConcatenation(matrix* mtx1, matrix* mtx2, matrix* m_conv);
int nRows(matrix * mtx, int * n);
int nCols(matrix * mtx, int * n);
int printMatrix(matrix * mtx);
int transposeMatrix(matrix * in, matrix * out);
matrix * returnTransposedMatrix(matrix * mtx);
matrix * returnNegMatrix (matrix* mtx);
matrix * returnProductMatrix(matrix * mtx1, matrix * mtx2);
matrix * returnProductScalarMatrix(float scalar, matrix* mtx);
matrix * returnCrossProduct3DVec(matrix * mtx1, matrix * mtx2);
matrix * returnVee(matrix * mtx);
matrix * returnHat(matrix * vec3D);
int sumMatrix(matrix * mtx1, matrix * mtx2, matrix * sum);
int sum3Matrix(matrix * mtx1, matrix * mtx2, matrix * mtx3, matrix * sum);
int sum4Matrix(matrix * mtx1, matrix * mtx2, matrix * mtx3, matrix * mtx4, matrix * sum);
int sum5Matrix(matrix * mtx1, matrix * mtx2, matrix * mtx3, matrix * mtx4, matrix * mtx5, matrix * sum);
int subtractMatrix(matrix * mtx1, matrix * mtx2, matrix * subtract);
int productMatrix(matrix * mtx1, matrix * mtx2, matrix * prod);
int crossProduct3DVec(matrix * mtx1, matrix * mtx2, matrix * crossProd);
int hat(matrix * vec3D, matrix * skewSymMtx);
int vee(matrix * mtx, matrix * veeVec);
float normL2Vec(matrix * vec);
int dotProductVector(matrix * v1, matrix * v2, float * prod);
int identityMatrix(matrix * m);
int isMatrixSquare(matrix * mtx);
int isMatrixDiagonal(matrix * mtx);
int isMatrixUpperTriangular(matrix * mtx);
int isMatrixSymmetric (matrix* mtx);
int isMatrixHermitian (matrix* mtx);
int isMatrixSymmetricPositiveDefinite (matrix* mtx);
int diagonalMatrix(matrix * v, matrix * mtx);
int invertDiagonalMatrix(matrix* m, matrix* m_inv);
int isSameDimension(matrix* mtx1, matrix* mtx2);
int productScalarMatrix(float scalar, matrix* mtx, matrix* product);
int coleskyDecomp(matrix* A, matrix* L);
int inverseMatrixChol(matrix* A, matrix* Ainv);


#endif // MATRIX_H_