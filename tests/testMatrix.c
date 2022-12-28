/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Test math library functions
 * @addtogroup TEST
 **/

#include "matrix.h"

int main() {
  matrix * A, * Ac, * B, * c, * d, * M, * ct, * mdp;
  float dp;

  A = newMatrix(3, 3);
  setMatrixElement(A, 1, 1, 1.0);
  setMatrixElement(A, 1, 2, .25);
  setMatrixElement(A, 1, 3, -.1);
  setMatrixElement(A, 2, 2, .4);
  setMatrixElement(A, 2, 3, .3);
  setMatrixElement(A, 3, 2, .1);
  setMatrixElement(A, 3, 3, -.3);
  printf("Matrix A:\n");
  printMatrix(A);

  Ac = copyMatrix(A);
  printf("\nCopy of A:\n");
  printMatrix(Ac);

  B = newMatrix(3, 3);
  setMatrixElement(B, 1, 1, .5);
  setMatrixElement(B, 2, 2, 2.0);
  setMatrixElement(B, 3, 3, 1.0);
  printf("\nMatrix B:\n");
  printMatrix(B);

  c = newMatrix(3, 1);
  setMatrixElement(c, 1, 1, 1.0);
  setMatrixElement(c, 3, 1, 1.0);
  printf("\nVector c:\n");
  printMatrix(c);

  d = newMatrix(3, 1);
  setMatrixElement(d, 2, 1, 1.0);
  setMatrixElement(d, 3, 1, 1.0);
  printf("\nVector d:\n");
  printMatrix(d);

  M = newMatrix(3, 3);
  transposeMatrix(A, M);
  printf("\nA':\n");
  printMatrix(M);

  ct = newMatrix(1, 3);
  transposeMatrix(c, ct);
  printf("\nc':\n");
  printMatrix(ct);

  sumMatrix(A, B, M);
  printf("\nA + B:\n");
  printMatrix(M);

  productMatrix(A, B, M);
  printf("\nA * B:\n");
  printMatrix(M);

  mdp = newMatrix(1, 1);
  productMatrix(ct, d, mdp);
  getMatrixElement(mdp, 1, 1, &dp);
  printf("\nDot product (1): %.2f\n", dp);

  dotProductMatrix(c, d, &dp);
  printf("\nDot product (2): %.2f\n", dp);

  productMatrix(A, c, d);
  printf("\nA * c:\n");
  printMatrix(d);

  printf("\nisMatrixUpperTriangular(A): %d"
         "\nisMatrixUpperTriangular(B): %d"
         "\nisMatrixDiagonal(A): %d"
         "\nisMatrixDiagonal(B): %d\n",
         isMatrixUpperTriangular(A),
         isMatrixUpperTriangular(B),
         isMatrixDiagonal(A),
         isMatrixDiagonal(B));

  identityMatrix(A);
  printf("\nIdentity:\n");
  printMatrix(A);

  diagonalMatrix(c, A);
  printf("\nDiagonal from c:\n");
  printMatrix(A);

  deleteMatrix(A);
  deleteMatrix(Ac);
  deleteMatrix(B);
  deleteMatrix(c);
  deleteMatrix(d);
  deleteMatrix(M);
  deleteMatrix(ct);
  deleteMatrix(mdp);

  return 0;
}