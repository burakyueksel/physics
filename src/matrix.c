/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Matrix libraries for physics
 * @addtogroup MATRIX
 **/

#include <assert.h>
#include "matrix.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h> // for sqrt

/* Creates a ``rows by cols'' matrix with all values 0.  
 * Returns NULL if rows <= 0 or cols <= 0 and otherwise a
 * pointer to the new matrix.
 */
matrix * newMatrix(int rows, int cols)
{
  if (rows <= 0 || cols <= 0) return NULL;

  // allocate a matrix structure
  matrix * m = (matrix *) malloc(sizeof(matrix));

  // set dimensions
  m->rows = rows;
  m->cols = cols;

  // allocate a float array of length rows * cols
  m->data = (float *) malloc(rows*cols*sizeof(float));
  // set all data to 0
  int i;
  for (i = 0; i < rows*cols; i++)
    m->data[i] = 0.0;

  return m;
}

/* Deletes a matrix.  Returns 0 if successful and -1 if mtx 
 * is NULL.
 */
int deleteMatrix(matrix * mtx)
{
  if (!mtx) return -1;
  // free mtx's data
  assert (mtx->data);
  free(mtx->data);
  // free mtx itself
  free(mtx);
  return 0;
}

/* Copies a matrix.  Returns NULL if mtx is NULL.
 */
matrix * copyMatrix(matrix * mtx)
{
  if (!mtx) return NULL;

  // create a new matrix to hold the copy
  matrix * cp = newMatrix(mtx->rows, mtx->cols);

  // copy mtx's data to cp's data
  memcpy(cp->data, mtx->data, 
         mtx->rows * mtx->cols * sizeof(float));

  return cp;
}

/* Sets the (row, col) element of mtx to val.  Returns 0 if
 * successful, -1 if mtx is NULL, and -2 if row or col are
 * outside of the dimensions of mtx.
 */
int setMatrixElement(matrix * mtx, int row, int col, float val) 
{
  if (!mtx) return -1;
  assert (mtx->data);
  if (row <= 0 || row > mtx->rows ||
      col <= 0 || col > mtx->cols)
    return -2;

  ELEM(mtx, row, col) = val;
  return 0;
}

/* Sets the reference val to the value of the (row, col) 
 * element of mtx.  Returns 0 if successful, -1 if either 
 * mtx or val is NULL, and -2 if row or col are outside of 
 * the dimensions of mtx.
 */
int getMatrixElement(matrix * mtx, int row, int col, float * val)
{
  if (!mtx || !val) return -1;
  assert (mtx->data);
  if (row <= 0 || row > mtx->rows ||
      col <= 0 || col > mtx->cols)
    return -2;

  *val = ELEM(mtx, row, col);
  return 0;
}

/* matrix concatenation into a matrix
 * m_conv = [mtx1 mtx2]
 * Returns 0 if successful, -1 if any of the matrices/vectors
 * are NULL, and -2 if the dimensions of the matrices are
 * incompatible.
 */

int matrixConcatenation (matrix* mtx1, matrix* mtx2, matrix* m_conv)
{
  if (!mtx1 || !mtx2 || !m_conv) return -1;
  if (mtx1->rows != mtx2->rows ||
     (mtx1->cols + mtx1->cols) != m_conv->cols ||
      mtx1->rows != m_conv->rows)
    return -2;
  int row, col;
  for (col = 1; col <= m_conv->cols; col++)
    for (row = 1; row <= m_conv->rows; row++)
      if (col <= mtx1->cols)
        ELEM(m_conv, row, col) = ELEM(mtx1, row, col);
      else
        ELEM(m_conv, row, col) = ELEM(mtx2, row, (col-mtx1->cols));

  return 0;
}

/* Sets the reference n to the number of rows of mtx.
 * Returns 0 if successful and -1 if mtx or n is NULL.
 */
int nRows(matrix * mtx, int * n)
{
  if (!mtx || !n) return -1;
  *n = mtx->rows;
  return 0;
}

/* Sets the reference n to the number of columns of mtx.
 * Returns 0 if successful and -1 if mtx is NULL.
 */
int nCols(matrix * mtx, int * n) 
{
  if (!mtx || !n) return -1;
  *n = mtx->rows;
  return 0;
}

/* Prints the matrix to stdout.  Returns 0 if successful 
 * and -1 if mtx is NULL.
 */
int printMatrix(matrix * mtx)
{
  if (!mtx) return -1;
  
  int row, col;
  for (row = 1; row <= mtx->rows; row++) {
    for (col = 1; col <= mtx->cols; col++) {
      // Print the floating-point element with
      //  - either a - if negative or a space if positive
      //  - at least 3 spaces before the .
      //  - precision to the hundredths place
      printf("% 6.6f ", ELEM(mtx, row, col));
    }
    // separate rows by newlines
    printf("\n");
  }
  return 0;
}

/* Writes the transpose of matrix in into matrix out.  
 * Returns 0 if successful, -1 if either in or out is NULL,
 * and -2 if the dimensions of in and out are incompatible.
 */
int transposeMatrix(matrix * in, matrix * out)
{
  if (!in || !out) return -1;
  if (in->rows != out->cols || in->cols != out->rows)
    return -2;

  int row, col;
  for (row = 1; row <= in->rows; row++)
    for (col = 1; col <= in->cols; col++)
      ELEM(out, col, row) = ELEM(in, row, col);
  return 0;
}

/* Reads matrix mtx and returns its transpose.
 * Note that the created matrix transposeMatrix inside this function
 * is NOT deleted later.
*/
matrix * returnTransposedMatrix(matrix * mtx)
{
  int row, col;
  matrix* transposeMatrix = newMatrix(mtx->rows, mtx->cols);
  for (row = 1; row <= mtx->rows; row++)
    for (col = 1; col <= mtx->cols; col++)
      ELEM(transposeMatrix, col, row) = ELEM(mtx, row, col);
  return transposeMatrix;
}


/* Reads matrix mtx and returns its negative, i.e. -mtx
 * by multiplying all its elements with -1.
 * Note that the created negMatrix inside this function is NOT
 * deleted later.
*/
matrix * returnNegMatrix (matrix* mtx)
{
  int row, col;
  matrix* negativeMatrix = newMatrix(mtx->rows, mtx->cols);
  for (row = 1; row <= mtx->rows; row++)
    for (col = 1; col <= mtx->cols; col++)
      ELEM(negativeMatrix, col, row) = -ELEM(mtx, row, col);
  return negativeMatrix;
}

/* Writes the sum of matrices mtx1 and mtx2 into matrix 
 * sum. Returns 0 if successful, -1 if any of the matrices 
 * are NULL, and -2 if the dimensions of the matrices are
 * incompatible.
 */
int sumMatrix(matrix * mtx1, matrix * mtx2, matrix * sum)
{
  if (!mtx1 || !mtx2 || !sum) return -1;
  if (mtx1->rows != mtx2->rows ||
      mtx1->rows != sum->rows ||
      mtx1->cols != mtx2->cols ||
      mtx1->cols != sum->cols)
    return -2;

  int row, col;
  for (col = 1; col <= mtx1->cols; col++)
    for (row = 1; row <= mtx1->rows; row++)
      ELEM(sum, row, col) = 
        ELEM(mtx1, row, col) + ELEM(mtx2, row, col);
  return 0;
}

/* Writes the sum of matrices mtx1, mtx2 and mtx3 into matrix
 * sum. Returns 0 if successful, -1 if any of the matrices
 * are NULL, and -2 if the dimensions of the matrices are
 * incompatible.
 */
int sum3Matrix(matrix * mtx1, matrix * mtx2, matrix * mtx3, matrix * sum)
{
  if (mtx1->rows != mtx2->rows ||
      mtx1->rows != mtx3->rows ||
      mtx1->rows != sum->rows  ||
      mtx1->cols != mtx2->cols ||
      mtx1->cols != mtx3->cols ||
      mtx1->cols != sum->cols)
    return -2;

  int row, col;
  for (col = 1; col <= mtx1->cols; col++)
    for (row = 1; row <= mtx1->rows; row++)
      ELEM(sum, row, col) =
        ELEM(mtx1, row, col) + ELEM(mtx2, row, col) + ELEM(mtx3, row, col);
  return 0;
}

/* Writes the sum of matrices mtx1, mtx2, mtx3 and mtx4 into matrix
 * sum. Returns 0 if successful, -1 if any of the matrices
 * are NULL, and -2 if the dimensions of the matrices are
 * incompatible.
 */
int sum4Matrix(matrix * mtx1, matrix * mtx2, matrix * mtx3, matrix * mtx4, matrix * sum)
{
  if (mtx1->rows != mtx2->rows ||
      mtx1->rows != mtx3->rows ||
      mtx1->rows != mtx4->rows ||
      mtx1->rows != sum->rows  ||
      mtx1->cols != mtx2->cols ||
      mtx1->cols != mtx3->cols ||
      mtx1->cols != mtx4->cols ||
      mtx1->cols != sum->cols)
    return -2;

  int row, col;
  for (col = 1; col <= mtx1->cols; col++)
    for (row = 1; row <= mtx1->rows; row++)
      ELEM(sum, row, col) =
        ELEM(mtx1, row, col) + ELEM(mtx2, row, col) + ELEM(mtx3, row, col) + ELEM(mtx4, row, col);
  return 0;
}


/* Writes the sum of matrices mtx1, mtx2, mtx3, mtx4 and mtx5 into matrix
 * sum. Returns 0 if successful, -1 if any of the matrices
 * are NULL, and -2 if the dimensions of the matrices are
 * incompatible.
 */
int sum5Matrix(matrix * mtx1, matrix * mtx2, matrix * mtx3, matrix * mtx4, matrix * mtx5, matrix * sum)
{
  if (mtx1->rows != mtx2->rows ||
      mtx1->rows != mtx3->rows ||
      mtx1->rows != mtx4->rows ||
      mtx1->rows != mtx5->rows ||
      mtx1->rows != sum->rows  ||
      mtx1->cols != mtx2->cols ||
      mtx1->cols != mtx3->cols ||
      mtx1->cols != mtx4->cols ||
      mtx1->cols != mtx5->cols ||
      mtx1->cols != sum->cols)
    return -2;

  int row, col;
  for (col = 1; col <= mtx1->cols; col++)
    for (row = 1; row <= mtx1->rows; row++)
      ELEM(sum, row, col) =
        ELEM(mtx1, row, col) + ELEM(mtx2, row, col) + ELEM(mtx3, row, col) + ELEM(mtx4, row, col) + ELEM(mtx5, row, col);
  return 0;
}

/* Writes the subtraction of matrice mtx2 from mtx1 into another matrix
 * Returns 0 if successful, -1 if any of the matrices
 * are NULL, and -2 if the dimensions of the matrices are
 * incompatible.
 */
int subtractMatrix(matrix * mtx1, matrix * mtx2, matrix * subtract)
{
  if (!mtx1 || !mtx2 || !subtract) return -1;
  if (mtx1->rows != mtx2->rows ||
      mtx1->rows != subtract->rows ||
      mtx1->cols != mtx2->cols ||
      mtx1->cols != subtract->cols)
    return -2;

  int row, col;
  for (col = 1; col <= mtx1->cols; col++)
    for (row = 1; row <= mtx1->rows; row++)
      ELEM(subtract, row, col) =
        ELEM(mtx1, row, col) - ELEM(mtx2, row, col);
  return 0;
}

/* Writes the product of matrices mtx1 and mtx2 into matrix
 * prod.  Returns 0 if successful, -1 if any of the 
 * matrices are NULL, and -2 if the dimensions of the 
 * matrices are incompatible.
 */
int productMatrix(matrix * mtx1, matrix * mtx2, matrix * prod)
{
  if (!mtx1 || !mtx2 || !prod) return -1;
  if (mtx1->cols != mtx2->rows ||
      mtx1->rows != prod->rows ||
      mtx2->cols != prod->cols)
    return -2;

  int row, col, k;
  for (col = 1; col <= mtx2->cols; col++)
    for (row = 1; row <= mtx1->rows; row++) {
      float val = 0.0;
      for (k = 1; k <= mtx1->cols; k++)
        val += ELEM(mtx1, row, k) * ELEM(mtx2, k, col);
      ELEM(prod, row, col) = val;
    }
  return 0;
}

/* Writes the product of matrices mtx1 and mtx2 into matrix
 * prod and returns matix prod.
 * Note that the created prod inside this function is NOT
 * deleted later.
 */
matrix * returnProductMatrix(matrix * mtx1, matrix * mtx2)
{
  matrix* prod = newMatrix(mtx1->rows, mtx2->cols);
  // if any of the input matrices do not exist or dimensions do not fit
  // return prod as 0 matrix
  if (!mtx1 || !mtx2 || !prod ||
      mtx1->cols != mtx2->rows ||
      mtx1->rows != prod->rows ||
      mtx2->cols != prod->cols)
    return prod;

  int row, col, k;
  for (col = 1; col <= mtx2->cols; col++)
    for (row = 1; row <= mtx1->rows; row++) {
      float val = 0.0;
      for (k = 1; k <= mtx1->cols; k++)
        val += ELEM(mtx1, row, k) * ELEM(mtx2, k, col);
      ELEM(prod, row, col) = val;
    }
  return prod;
}

/* Writes the cross product of two 3D vectors mtx1 and mtx2
 * Returns 0 if successful, -1 if any of the
 * vectors are NULL, and -2 if the dimensions of the
 * vectors are incompatible and not 3x1
 */
int crossProduct3DVec(matrix * mtx1, matrix * mtx2, matrix * crossProd)
{
  if (!mtx1 || !mtx2 || !crossProd) return -1;
  if (mtx1->cols != mtx2->cols ||
      mtx1->rows != mtx2->rows ||
      mtx2->cols != crossProd->cols ||
      mtx2->rows != crossProd->rows ||
      mtx1->cols != 1 ||
      mtx1->rows != 3)
    return -2;

  ELEM(crossProd,1,1) = ELEM(mtx1,2,1) * ELEM(mtx2,3,1) - ELEM(mtx1,3,1) * ELEM(mtx2,2,1);
  ELEM(crossProd,2,1) = ELEM(mtx1,3,1) * ELEM(mtx2,1,1) - ELEM(mtx1,1,1) * ELEM(mtx2,3,1);
  ELEM(crossProd,3,1) = ELEM(mtx1,1,1) * ELEM(mtx2,2,1) - ELEM(mtx1,2,1) * ELEM(mtx2,1,1);

  return 0;
}

/* Writes the cross product of two 3D vectors mtx1 and mtx2
 * Returns crossProd  vector.
 * Note that the created crossProd inside this function is NOT
 * deleted later.
 */
matrix * returnCrossProduct3DVec(matrix * mtx1, matrix * mtx2)
{
  matrix* crossProd = newMatrix(3,1);
  // if any of the input matrices (vectors) do not exist or dimensions do not fit
  // return crossProd as a 3x1 vector with all elements 0.
  if (!mtx1 || !mtx2 || !crossProd ||
      mtx1->cols != mtx2->cols ||
      mtx1->rows != mtx2->rows ||
      mtx2->cols != crossProd->cols ||
      mtx2->rows != crossProd->rows ||
      mtx1->cols != 1 ||
      mtx1->rows != 3)
    return crossProd;

  ELEM(crossProd,1,1) = ELEM(mtx1,2,1) * ELEM(mtx2,3,1) - ELEM(mtx1,3,1) * ELEM(mtx2,2,1);
  ELEM(crossProd,2,1) = ELEM(mtx1,3,1) * ELEM(mtx2,1,1) - ELEM(mtx1,1,1) * ELEM(mtx2,3,1);
  ELEM(crossProd,3,1) = ELEM(mtx1,1,1) * ELEM(mtx2,2,1) - ELEM(mtx1,2,1) * ELEM(mtx2,1,1);

  return crossProd;
}

/* Hat operation: Writes the skew symmetric matrix from a 3D vector mtx
 * hat operator allows moving from the real vector space to Lie algebra
 * Returns 0 if successful, -1 if any of the
 * vector or matrix are NULL, and -2 if the dimensions of the
 * vector and matrix are incompatible and not 3x1 and 3x3
 */
int hat(matrix * vec3D, matrix * skewSymMtx)
{
  if (!vec3D || !skewSymMtx) return -1;
  if (vec3D->cols != skewSymMtx->cols ||
      skewSymMtx->cols != skewSymMtx->rows ||
      vec3D->cols != 3 ||
      vec3D->rows != 1)
    return -2;

  ELEM(skewSymMtx,1,1) = 0.0f;
  ELEM(skewSymMtx,1,2) = -ELEM(vec3D,3,1);
  ELEM(skewSymMtx,1,3) = ELEM(vec3D,2,1);
  ELEM(skewSymMtx,2,1) = ELEM(vec3D,3,1);
  ELEM(skewSymMtx,2,2) = 0.0f;
  ELEM(skewSymMtx,2,3) = -ELEM(vec3D,1,1);
  ELEM(skewSymMtx,3,1) = -ELEM(vec3D,2,1);
  ELEM(skewSymMtx,3,2) = ELEM(vec3D,1,1);
  ELEM(skewSymMtx,3,3) = 0.0f;

  return 0;
}

/* Hat operation: Writes the skew symmetric matrix from a 3D vector mtx
 * hat operator allows moving from the real vector space to Lie algebra
 * Returns skewSymMtx after the algorithm runs.
 * If any of the
 * vector or matrix are NULL, or if the dimensions of the
 * vector and matrix are incompatible and not 3x1 and 3x3
 * it returns 3x3 skewSymMtx with all elements zero.
 */
matrix * returnHat(matrix * vec3D)
{
  matrix* skewSymMtx = newMatrix(3,3);
  if (!vec3D || !skewSymMtx ||
      vec3D->cols != skewSymMtx->cols ||
      skewSymMtx->cols != skewSymMtx->rows ||
      vec3D->cols != 3 ||
      vec3D->rows != 1)
    return skewSymMtx;

  ELEM(skewSymMtx,1,1) = 0.0f;
  ELEM(skewSymMtx,1,2) = -ELEM(vec3D,3,1);
  ELEM(skewSymMtx,1,3) = ELEM(vec3D,2,1);
  ELEM(skewSymMtx,2,1) = ELEM(vec3D,3,1);
  ELEM(skewSymMtx,2,2) = 0.0f;
  ELEM(skewSymMtx,2,3) = -ELEM(vec3D,1,1);
  ELEM(skewSymMtx,3,1) = -ELEM(vec3D,2,1);
  ELEM(skewSymMtx,3,2) = ELEM(vec3D,1,1);
  ELEM(skewSymMtx,3,3) = 0.0f;

  return skewSymMtx;
}

/* Vee operation: Writes the vee vector from a 3D matrix mtx
 * The Vee operator allows to move from the Lie algebra g to the real vector
 * space of dimension equal to the one of the Lie algebra.
 * Returns 0 if successful, -1 if any of the
 * vector or matrix are NULL, and -2 if the dimensions of the
 * vector and matrix are incompatible and not 3x1 and 3x3
 */
int vee(matrix * mtx, matrix * veeVec)
{
  if (!mtx || !veeVec) return -1;
  if (mtx->cols != veeVec->rows ||
      mtx->cols != mtx->rows ||
      mtx->cols != 3 ||
      veeVec->cols !=1)
    return -2;

  ELEM(veeVec,3,1) = ELEM(mtx,3,2);
  ELEM(veeVec,3,2) = ELEM(mtx,1,3);
  ELEM(veeVec,3,3) = ELEM(mtx,2,1);

  return 0;
}

/* Vee operation: Writes the vee vector from a 3D matrix mtx
 * The Vee operator allows to move from the Lie algebra g to the real vector
 * space of dimension equal to the one of the Lie algebra.
 * Returns veeVec after the algorithm runs.
 * if any of the
 * vector or matrix are NULL, or if the dimensions of the
 * vector and matrix are incompatible and not 3x1 and 3x3
 * it retuns a 3x1 vector of zeros
 */
matrix * returnVee(matrix * mtx)
{
  matrix* veeVec = newMatrix(3,1);
  if (!mtx || !veeVec ||
      mtx->cols != veeVec->rows ||
      mtx->cols != mtx->rows ||
      mtx->cols != 3 ||
      veeVec->cols !=1)
    return veeVec;

  ELEM(veeVec,3,1) = ELEM(mtx,3,2);
  ELEM(veeVec,3,2) = ELEM(mtx,1,3);
  ELEM(veeVec,3,3) = ELEM(mtx,2,1);

  return veeVec;
}

/* Compute the norm of a vector
 */
float normL2Vec(matrix * vec)
{
  if (!vec) return -1;
  if (vec->cols !=1) return -2;
  float sqrtSum = 0.0f;
  int row;
  for (row = 1; row <= vec->rows; row++)
  {
    sqrtSum += ELEM(vec, row, 1) * ELEM(vec, row, 1);
  }
  // return the L2 norm
  return sqrt(sqrtSum);
}

/* Writes the dot product of vectors v1 and v2 into 
 * reference prod.  Returns 0 if successful, -1 if any of
 * v1, v2, or prod are NULL, -2 if either matrix is not a 
 * vector, and -3 if the vectors are of incompatible 
 * dimensions.
 */
int dotProductVector(matrix * v1, matrix * v2, float * prod)
{
  if (!v1 || !v2 || !prod) return -1;
  if (v1->cols != 1 || v2->cols != 1) return -2;
  if (v1->rows != v2->rows) return -3;

  *prod = 0;
  int i;
  for (i = 1; i <= v1->rows; i++)
    *prod += ELEM(v1, i, 1) * ELEM(v2, i, 1);
  return 0;
}

/* Create an identity matrix
 */
int identityMatrix(matrix * m)
{
  if (!m || m->rows != m->cols) return -1;
  int row, col;
  for (col = 1; col <= m->cols; col++)
    for (row = 1; row <= m->rows; row++)
      if (row == col) 
        ELEM(m, row, col) = 1.0;
      else 
        ELEM(m, row, col) = 0.0;
  return 0;
}

/* check if a matrix is square
 */
int isMatrixSquare(matrix * mtx)
{
  return mtx && mtx->rows == mtx->cols;
}

/* Check if a matrix is diagonal
 */

int isMatrixDiagonal(matrix * mtx)
{
  // does it exist?
  if (!mtx) return -2;
  // it should be square a matrix
  if (!isMatrixSquare(mtx)) return 0;
  int row, col;
  for (col = 1; col <= mtx->cols; col++)
    for (row = 1; row <= mtx->rows; row++)
      // if the element is not on the diagonal and not 0
      if (row != col && ELEM(mtx, row, col) != 0.0)
        // then the matrix is not diagonal
        return 0;
  return 1;
}

/* Check if a matrix is upper triangular
 */
int isMatrixUpperTriangular(matrix * mtx)
{
  // does it exist?
  if (!mtx) return -2;
  // it should be square a matrix
  if (!isMatrixSquare(mtx)) return 0;
  int row, col;
  // looks at positions below the diagonal
  for (col = 1; col <= mtx->cols; col++)
    for (row = col+1; row <= mtx->rows; row++) 
      if (ELEM(mtx, row, col) != 0.0)
        return 0;
  return 1;
}

/** @brief Check if a matrix is symmetric
 */
int isMatrixSymmetric (matrix* mtx)
{
  // does it exist?
  if (!mtx) return -2;
  // it should be a square matrix
  if (!isMatrixSquare(mtx)) return -1;
  int row, col;
  for (row = 1; row<=mtx->rows; row++)
  {
    for (col = 1; col<=row; col++)
    {
      /* Check for symmetry
       by verifying if the matrix is symmetric by comparing the elements
       of the matrix with their transpose
      */
      if (ELEM(mtx, row, col) != ELEM(mtx, col, row))
      {
        return 0; // not symmetric
      }
    }
  }
  return 1; // success. It is symmetric
}

/** @brief Check if a matrix is Hermitian
 * by comparing the elements of the matrix with their conjugate transpose
 * using the fabs function to handle floating-point numbers with small errors.
 */
int isMatrixHermitian (matrix* mtx)
{
  // does it exist?
  if (!mtx) return -2;
  // it should be a square matrix
  if (!isMatrixSquare(mtx)) return -1;
  int row, col;
  for (row = 1; row <= mtx->rows; row++)
    for (col = 1; col <= mtx->cols; col++)
      if (fabs(ELEM(mtx, row, col)-ELEM(mtx, col, row)> 1e-15))
        return 0;
  return 1;
}

/** @brief Check if a matrix is symmetric and positive definite
 * It does checking for both symmetry and positive definiteness
 */
int isMatrixSymmetricPositiveDefinite (matrix* mtx)
{
  // FIXME: this is not working correctly
  // does it exist?
  if (!mtx) return -3;
  // it should be a square matrix
  if (!isMatrixSquare(mtx)) return -2;
  int row, col;
  float sum;
  for (row = 1; row<=mtx->rows; row++)
  {
    sum = 0;
    for (col = 1; col<=row; col++)
    {
      /* Check for symmetry
       by verifying if the matrix is symmetric by comparing the elements
       of the matrix with their transpose
      */
      if (ELEM(mtx, row, col) != ELEM(mtx, col, row))
      {
        return -1; // not symmetric
      }
      sum += ELEM(mtx, row, col)*ELEM(mtx, row, col);
    }
    /* Check positive definiteness
    by verifying that all of its leading principal minors are positive.
    This is done by checking that the diagonal element of matrix is greater
    than the sum of squares of the elements above it. If this condition holds
    for all elements of the matrix, the matrix is positive definite.
    */
    if (ELEM(mtx, row, row)<=sum)
    {
      printf("matrix element is: %f .\n", ELEM(mtx, row, row));
      printf("sum is: %f .\n", sum);
      return 0; // not positive definite.
    }
  }
  return 1; // success. It is symmetric and positive definite
}

/* Create a diagonal matrix from a vector
 */
int diagonalMatrix(matrix * v, matrix * mtx)
{
  if (!v || !mtx ||
      v->cols > 1 || v->rows != mtx->rows ||
      mtx->cols != mtx->rows)
    return -1;
  int row, col;
  for (col = 1; col <= mtx->cols; col++)
    for (row = 1; row <= mtx->rows; row++)
      if (row == col) 
        ELEM(mtx, row, col) = ELEM(v, col, 1);
      else
        ELEM(mtx, row, col) = 0.0;
  return 0;
}

/* invert a diagonal matrix
 */
int invertDiagonalMatrix(matrix* m, matrix* m_inv)
{
  // do m and m_inv exist?
  if (!m || !m_inv) return -3;
  // m and m_inv must have the same dimensions
  if (!isSameDimension(m, m_inv))  return -2;
  // matrix needs to be diagonal for this function
  if (!isMatrixDiagonal(m)) return -1;

  int row, col;
  for (col = 1; col <= m_inv->cols; col++)
    for (row = 1; row <= m_inv->rows; row++)
      if (row == col)
        ELEM(m_inv, row, col) = 1/ELEM(m, row, col);
      else
        ELEM(m_inv, row, col) = 0.0;
  return 0;
}

int isSameDimension(matrix* mtx1, matrix* mtx2)
{
  // do m and m_inv exist?
  if (!mtx1 || !mtx1) return -1;
  // m and m_inv must have the same dimensions
  if (mtx1->rows != mtx2->rows || mtx1->cols != mtx2->cols) return 0;
  // looks good
  return 1;
}

/* Compute the product of a matrix (or vector) with a scalar
 */
int productScalarMatrix(float scalar, matrix* mtx, matrix* product)
{
  // do the inputs even exist?
  if (!scalar || !mtx) return -2;
  // mtx and product must have the same dimensions
  if (mtx->rows != product->rows || mtx->cols != product->cols) return -1;

  int row, col;
  for (col = 1; col <= product->cols; col++)
    for (row = 1; row <= product->rows; row++)
      if (row == col)
      ELEM(product, row, col) = scalar * ELEM(mtx, row, col);
  return 0;
}

/* Compute the product of a matrix (or vector) with a scalar
 */
matrix * returnProductScalarMatrix(float scalar, matrix* mtx)
{
  matrix* product = copyMatrix(mtx);
  // if any of the inputs do not exist or dimensions do not fit
  // return product as same as mtx (copied) before the algorithm.
  if (!scalar || !mtx || mtx->rows != product->rows || mtx->cols != product->cols) return product;

  int row, col;
  for (col = 1; col <= product->cols; col++)
    for (row = 1; row <= product->rows; row++)
      if (row == col)
      ELEM(product, row, col) = scalar * ELEM(mtx, row, col);
  return product;
}

/** @brief Compute Cholesky Decomposition of matrix A.
 *  A is a Hermitian, positive-definite. Meaning when its elements are all real numbers,
 *  it is positive definite symmetric matrix.
 *  Then we can write A = L*L^T (cholesky decpmposition), where L is a lower triangular matrix
 *  and L^T is its transpose, which is an upper triangular matrix.
 */
int coleskyDecomp(matrix* A, matrix* L)
{
  /* source1: https://rosettacode.org/wiki/Cholesky_decomposition#C
   * source2 : https://en.wikipedia.org/wiki/Cholesky_decomposition*/
  // do the inputs even exist?
  if (!A || !L) return -3;
  // are the matrices square?
  if (!isMatrixSquare(A) || !isMatrixSquare(L)) return -2;
  // are the matrices of the same dimension?
  if (!isSameDimension(A, L)) return -1;
  // is matrix symmetric and positive definite?
  /*
  A Hermitian matrix is always positive definite because all the eigenvalues of a Hermitian matrix are real and positive.
  This is because the eigenvalues of Hermitian matrices are the same as the eigenvalues of its real symmetric matrices and
  those are always real and positive.
  In summary, a Hermitian matrix is guaranteed to be positive definite,
  because it's a symmetric matrix with all real eigenvalues, which are guaranteed to be positive.
  */
  if (!isMatrixSymmetric(A) || !isMatrixHermitian(A)) return 0;
  int row, col, k;
  float sum;
  for (row = 1; row<=L->rows; row++)
  {
    for (col = 1; col<=row; col++)
    {
      sum = 0;
      if (col==row)
      {
        for (k=1; k<row; k++)
        {
          sum += ELEM(L, row, k) * ELEM(L, row, k);
        }
        ELEM(L, row, col) = sqrt(ELEM(A, row, col) - sum);
      }
      else
      {
        for (k=1; k<row; k++)
        {
          sum += ELEM(L, row, k) * ELEM(L, col, k);
        }
        ELEM(L, row, col) = (ELEM(A, row, col) - sum) / ELEM(L, col, col);
      }
    }
  }
  return 1; // success
}

int choleskyFwdSolve(matrix* L, matrix* y)
{
  int row, col, k;
  float sum;
  for (row = 1; row <=L->rows; row++)
  {
    for (col = 1; col <= L->cols; col++)
    {
      if (row == col)
      {
        ELEM(y, row, 1) = 1 / ELEM(L, row, row);
      }
      else
      {
        sum = 0;
        for (k = 1; k <= row; k++)
        {
          sum += ELEM(L, row, k) * ELEM(y, k, 1);
        }
        ELEM(y, row, 1) = (1 - sum) / ELEM(L, row, row);
      }
    }
  }
  return 1; // success
}

/*
  // Solve LT*x = y for x using backward substitution
*/
int choleskyBckSolve(matrix* LT, matrix* y, matrix* x)
{
  int row, col, k;
  float sum;
  for (row = LT->rows; row >= 1; row--)
  {
    for (col = LT->cols; col >= 1; col--)
    {
      if (row == col)
      {
          ELEM(x, row, 1)= ELEM(y, row, 1) / ELEM(LT, row, row);
      }
      else
      {
        sum = 0;
        for (k = LT->cols; k >= row; k--)
        {
          sum += ELEM(LT, row, k) * ELEM(x, k, 1);
        }
        ELEM(x, row, 1) = (ELEM(y, row, 1) - sum) / ELEM(LT, row, row);
      }
    }
  }
  return 1; // success
}

/*
  // Store the result in the inverse matrix
*/
int choleskyStore(matrix* Ainv, matrix* x, matrix* y)
{
  int row, col;
  for (row = 1; row <=Ainv->rows; row++)
  {
    for (col = 1; col <=Ainv->cols; col++)
    {
        ELEM(Ainv, row, col) = ELEM(x, row, 1) * ELEM(y, col, 1);
    }
  }
  return 1; // success
}

/** @brief Compute inverse of a matrix A using cholesky decomposition
 *  This only wroks when A is a Hermitian, positive-definite. Meaning when its elements are all real numbers,
 *  it is positive definite symmetric matrix.
 *  Then we can write A = L*L^T (cholesky decpmposition), where L is a lower triangular matrix
 *  and L^T is its transpose, which is an upper triangular matrix.
 *
 *  It inverse is then A^-1 = (LL^T)^-1 = L^-T L^-1
 *  Now, using the property of triangular matrix inversion, we know that L^-1 and L^-T can be obtained
 *  by solving the following systems of equations:
 *  1. Lx = I, where x is L^-1 and I is the identity matrix
 *  2. L^Tx = x, where x is L^-1
 *  We can solve these system of equation using forward and backward substitution respectively.
 *
 *  Notice that this will only work when A is positive definite and symmetric.
 *  Daily life (of an engineer working in robotics, aviation, etc) examples for A:
 *  -> Moment of Inertia
 *  -> Inertia Matrix
 *  -> Covariance matrix (Kalman Filter)
 */
int inverseMatrixChol(matrix* A, matrix* Ainv)
{
  // do the inputs even exist?
  if (!A || !Ainv) return -2;
  // is square, is same dimension, is pos def and is symmetric checks are done in cholesky decomp
  // create an empty matrix for cholesky decomposition, in the same dimension of A
  matrix* L = newMatrix(A->rows, A->cols);
  // compute the cholesky decomposition of A, which is L
  int chol_status = coleskyDecomp(A, L);
  if (!chol_status) return chol_status;
  // create x and y vectors for backward and forward substitution
  matrix* x = newMatrix(A->rows, 1);
  matrix* y = copyMatrix(x);
  // create an empty matrix, which will be the transpose of L
  matrix* LT= copyMatrix(L);
  // compute the transpose of L
  transposeMatrix(L,LT);
  // Solve L*y = I for y using forward substitution
  choleskyFwdSolve(L,y);
  // Solve LT*x = y for x using backward substitution
  choleskyBckSolve(L,y,x);
  // Store the result in the inverse matrix
  choleskyStore(Ainv,x,y);
  return 1; //success
}


/*
bool isPositiveDefinite(int n, double a[n][n]) {
    double det = 1;
    for (int i = 0; i < n; i++) {
        det *= a[i][i];
    }
    if (det <= 0) {
        return false;
    }
    for (int i = 1; i <= n; i++) {
        double sub_det = 1;
        for (int j = 0; j < i; j++) {
            for (int k = 0; k < i; k++) {
                sub_matrix[j][k] = a[j][k];
            }
        }
        for (int j = 0; j < i; j++) {
            sub_det *= sub_matrix[j][j];
        }
        if (sub_det <= 0) {
            return false;
        }
    }
    return true;
}

*/
/*
// An alternative using cholesky

// Function to perform Cholesky decomposition
void choleskyDecomposition(int n, double a[n][n], double L[n][n]) {
    int i, j, k;
    // i: row, j: column
    double sum;
    for (i = 0; i < n; i++) {
        for (j = 0; j <= i; j++) {
            sum = 0;
            if (i == j) {
                for (k = 0; k < j; k++) {
                    sum += L[j][k] * L[j][k];
                }
                L[j][j] = sqrt(a[j][j] - sum);
            } else {
                for (k = 0; k < j; k++) {
                    sum += L[i][k] * L[j][k];
                }
                L[i][j] = (a[i][j] - sum) / L[j][j];
            }
        }
    }
}

// Function to compute the inverse of a matrix using Cholesky decomposition
void inverse(int n, double a[n][n], double inverse[n][n]) {
    double L[n][n], LT[n][n], y[n], x[n];
    int i, j, k;
    double sum;
    choleskyDecomposition(n, a, L);
    // Compute the transpose of L
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            LT[i][j] = L[j][i];
        }
    }
    // Solve L*y = I for y using forward substitution
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (i == j) {
                y[i] = 1 / L[i][i];
            } else {
                sum = 0;
                for (k = 0; k < i; k++) {
                    sum += L[i][k] * y[k];
                }
                y[i] = (1 - sum) / L[i][i];
            }
        }
    }
    // Solve LT*x = y for x using backward substitution
    for (i = n - 1; i >= 0; i--) {
        for (j = n - 1; j >= 0; j--) {
            if (i == j) {
                x[i] = y[i] / LT[i][i];
            } else {
                sum = 0;
                for (k = n - 1; k > i; k--) {
                    sum += LT[i][k] * x[k];
                }
                x[i] = (y[i] - sum) / LT[i][i];
            }
        }
    }
    // Store the result in the inverse matrix
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            inverse[i][j] = x[i] * y[j];
        }
    }
}
*/


/*
   Cholesky-decomposition matrix-inversion code, adapated from
   http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt
  source: https://github.com/simondlevy/TinyEKF/blob/master/src/tiny_ekf.c

  Notice that cholesky-decomposition does matrix inversion only to positive definite Hermitian matrices.
  This means symmetric positive definite matrices when all components of the matrix are real numbers.
  Example to such matrices:
  - Moment of inertia
  - Covariance matrices

  Usage:
  if (cholsl(posDefSymmMatrix, inverseOfMatrix, tempValue, dimensionOfMatrix)) return 1;

static int choldc1(double * a, double * p, int n) {
    int i,j,k;
    double sum;

    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            sum = a[i*n+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= a[i*n+k] * a[j*n+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    return 1; // error
                }
                p[i] = sqrt(sum);
            }
            else {
                a[j*n+i] = sum / p[i];
            }
        }
    }

    return 0; // success
}

static int choldcsl(double * A, double * a, double * p, int n) 
{
    int i,j,k; double sum;
    for (i = 0; i < n; i++) 
        for (j = 0; j < n; j++) 
            a[i*n+j] = A[i*n+j];
    if (choldc1(a, p, n)) return 1;
    for (i = 0; i < n; i++) {
        a[i*n+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*n+k] * a[k*n+i];
            }
            a[j*n+i] = sum / p[j];
        }
    }

    return 0; // success
}


static int cholsl(double * A, double * a, double * p, int n) 
{
    int i,j,k;
    if (choldcsl(A,a,p,n)) return 1;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            a[i*n+j] = 0.0;
        }
    }
    for (i = 0; i < n; i++) {
        a[i*n+i] *= a[i*n+i];
        for (k = i + 1; k < n; k++) {
            a[i*n+i] += a[k*n+i] * a[k*n+i];
        }
        for (j = i + 1; j < n; j++) {
            for (k = j; k < n; k++) {
                a[i*n+j] += a[k*n+i] * a[k*n+j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            a[i*n+j] = a[j*n+i];
        }
    }

    return 0; // success
}

*/

/*

// function to find the inverse of a matrix using LU decomposition
void inverse(double A[N][N], double inverse[N][N])
{
    double L[N][N], U[N][N];
    double y[N], x[N];
    double b[N],z[N];
    int i, j, k;
    double det = 1;

    // LU decomposition
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            U[i][j] = A[i][j];
            if (i > j)
                L[i][j] = 0;
            else
                L[i][j] = 1;
        }
    }

    for (k = 0; k < N; k++)
    {
        for (i = k + 1; i < N; i++)
        {
            L[i][k] = U[i][k] / U[k][k];
            for (j = k; j < N; j++)
                U[i][j] = U[i][j] - L[i][k] * U[k][j];
        }
    }

    // find the determinant of A
    for (i = 0; i < N; i++)
        det = det * L[i][i] * U[i][i];

    // find the inverse of A
    for (j = 0; j < N; j++)
    {
        for (i = 0; i < N; i++)
        {
            if (i == j)
                b[i] = 1;
            else
                b[i] = 0;
        }

        // forward substitution
        for (i = 0; i < N; i++)
        {
            y[i] = b[i];
            for (k = 0; k < i; k++)
                y[i] = y[i] - L[i][k] * y[k];
        }

        // backward substitution
        for (i = N - 1; i >= 0; i--)
        {
            z[i] = y[i];
            for (k = i + 1; k < N; k++)
                z[i] = z[i] - U[i][k] * z[k];
            x[i] = z[i] / U[i][i];
        }

        // store the result in inverse matrix
        for (i = 0; i < N; i++)
            inverse[i][j] = x[i];
    }
}
*/

/*

int inverseSquareMatrix(matrix* mtx, matrix* mtx_inv)
{
  // if the matrix is not square, do not do inversion.
  if (!isMatrixSquare(mtx)) return 0;
  int row, col;
  for (col = 1; col <= mtx->cols; col++)
  {
    for (row = 1; row <= mtx->rows; row++)
    {
      ELEM(mtx_inv, row, col) = 0.0;
    }
    ELEM(mtx_inv, row, col) = 1.0;
  }

}


int invert_matrix(int n, double A[n][n], double A_inv[n][n]) {
    int i, j, k;
    double max, temp, sum;

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            A_inv[i][j] = 0.0;
        }
        A_inv[i][i] = 1.0;
    }

    for (i = 0; i < n; i++) {
        max = -1.0;
        for (k = i; k < n; k++) {
            if (fabs(A[k][i]) > max) {
                max = fabs(A[k][i]);
                j = k;
            }
        }
        if (fabs(max) < EPSILON) {
            return 0;
        }
        if (j != i) {
            for (k = 0; k < n; k++) {
                temp = A[i][k];
                A[i][k] = A[j][k];
                A[j][k] = temp;

                temp = A_inv[i][k];
                A_inv[i][k] = A_inv[j][k];
                A_inv[j][k] = temp;
            }
        }
        temp = A[i][i];
        for (k = 0; k < n; k++) {
            A[i][k] /= temp;
            A_inv[i][k] /= temp;
        }
        for (j = 0; j < n; j++) {
            if (j != i) {
                temp = A[j][i];
                for (k = 0; k < n; k++) {
                    A[j][k] -= A[i][k] * temp;
                    A_inv[j][k] -= A_inv[i][k] * temp;
                }
            }
        }
    }
    return 1;
}
*/