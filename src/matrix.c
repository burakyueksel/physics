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
      printf("% 6.2f ", ELEM(mtx, row, col));
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

/* Writes the dot product of vectors v1 and v2 into 
 * reference prod.  Returns 0 if successful, -1 if any of
 * v1, v2, or prod are NULL, -2 if either matrix is not a 
 * vector, and -3 if the vectors are of incompatible 
 * dimensions.
 */
int dotProductMatrix(matrix * v1, matrix * v2, float * prod)
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

int isMatrixSquare(matrix * mtx)
{
  return mtx && mtx->rows == mtx->cols;
}

int isMatrixDiagonal(matrix * mtx)
{
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

int isMatrixUpperTriangular(matrix * mtx)
{
  if (!isMatrixSquare(mtx)) return 0;
  int row, col;
  // looks at positions below the diagonal
  for (col = 1; col <= mtx->cols; col++)
    for (row = col+1; row <= mtx->rows; row++) 
      if (ELEM(mtx, row, col) != 0.0)
        return 0;
  return 1;
}

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

int invertDiagonalMatrix(matrix* m, matrix* m_inv)
{
  // do m and m_inv exist?
  if (!m || !m_inv) return -2;
  // m and m_inv must have the same dimensions
  if (m->rows != m_inv->rows || m->cols != m_inv->cols) return -1;
  // matrix needs to be diagonal for this function
  if (!isMatrixDiagonal(m)) return 0;

  int row, col;
  for (col = 1; col <= m_inv->cols; col++)
    for (row = 1; row <= m_inv->rows; row++)
      if (row == col)
        ELEM(m_inv, row, col) = 1/ELEM(m, row, col);
      else
        ELEM(m_inv, row, col) = 0.0;
  return 0;
}

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