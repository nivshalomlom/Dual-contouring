using System;

namespace LinerAlgebra
{

    /// <summary>
    /// A MxN matrix implementation in C#
    /// </summary>
    public class Matrix : ICloneable
    {

        #region Internal properties

        private float[,] data;

        #endregion

        #region Properties

        public int width { get => data.GetLength(0); }

        public int height { get => data.GetLength(1); }

        public float this[int i, int j]
        {
            get => this.data[i, j];
            set => this.data[i, j] = value;
        }

        #endregion

        #region Functions

        /// <summary>
        /// A method to decompose this matrix to 2 matrices Q and R such that A = Q * R <br/>
        /// Q - A orthogonal matrix (meaning that Q * Q_T = I, and that Q_T = Q_inverse)
        /// R - A upper triangular matrix
        /// </summary>
        /// <returns> A array containing {Q, R} </returns>
        public Matrix[] QR_Decomposition()
        {
            /* 
            Compute Uk = Ak - sigma(
                range: j|1 -> k - 1)
                action: proj(Uj, Ak)) 

            proj(u, a) = (<u, a> / <u, u>) * u
            */
            Matrix U = this.Clone() as Matrix;
            for (int i = 0; i < this.width; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    float ua = 0;
                    for (int k = 0; k < this.height; k++)
                        ua += U[j, k] * this[i, k];

                    float uu = 0;
                    for (int k = 0; k < this.height; k++)
                        uu += U[j, k] * U[j, k];
                    
                    float ratio = ua / uu;
                    for (int k = 0; k < this.height; k++)
                    U[i, k] -= ratio * U[j, k];
                }
            }

            // Compute Q by normallzing all U columns
            Matrix Q = new Matrix(this.width, this.height);
            for (int i = 0; i < this.width; i++)
            {
                float amplitude = 0f;
                for (int j = 0; j < this.height; j++)
                    amplitude += U[i, j] * U[i, j];
                amplitude = (float) Math.Sqrt(amplitude);

                for (int j = 0; j < this.height; j++)
                    Q[i, j] = U[i, j] / amplitude;
            }

            // Compute R = Q_T * A
            Matrix R = Q.Transpose() * this;
            return new Matrix[] {Q, R};
        }

        /// <summary>
        /// A method to decompose this matreix to L a lower triangular matrix and U a upper triangular matrix, such that A = L * U
        /// <br/> Note: this method only works for square matrices!
        /// </summary>
        /// <returns> A array containing {L, U} </returns>
        public Matrix[] LU_Decomposition()
        {
            // Check matrix is square
            if (this.width != this.height)
                throw new System.Exception("Error: Cannot decompose non square matrix!");

            Matrix L = new Matrix(this.width, this.height);
            Matrix U = new Matrix(this.width, this.height);

            for (int i = 0; i < this.width; i++)
            {
                // Comptue upper
                for (int j = i; j < this.height; j++)
                {
                    float sum = 0f;
                    for (int k = 0; k < i; k++)
                        sum += L[i, k] * U[k, j];

                    U[i, j] = this[i, j] - sum;
                }

                // Compute lower
                for (int j = i; j < this.height; j++)
                {
                    if (i == j)
                        L[i, j] = 1;
                    else
                    {
                        float sum = 0f;
                        for (int k = 0; k < i; k++)
                            sum += L[j, k] * U[k, i];

                        L[j, i] = (this[j, i] - sum) / U[i, i];
                    }
                }
            }

            return new Matrix[] {L, U};
        }

        /// <returns> The determinant of the matrix (if computable) </returns>
        public float Determinant()
        {
            // Check matrix is square
            if (this.width != this.height)
                throw new System.Exception("Error: Cannot compute determinant of a non square matrix!");
            
            // Decompose to LU
            Matrix[] LU = this.LU_Decomposition();

            // Compute Det(A) = Det(LU) = Det(U) * Det(L)
            // Which are the multiplication of the diagonal
            float det = 1f;
            for (int i = 0; i < this.width; i++)
                det *= LU[0][i, i] * LU[1][i, i];

            // Return result
            return det;
        }

        /// <summary>
        /// A method to transpose this matrix (flip it on the main diagonal)
        /// </summary>
        /// <returns> A transposed copy of this matrix </returns>
        public Matrix Transpose()
        {
            Matrix result = new Matrix(this.height, this.width);
            for (int i = 0; i < this.width; i++)
                for (int j = 0; j < this.height; j++)
                    result[j, i] = this[i, j];

            return result;
        }

        /// <summary>
        /// A method to find the eigen values and vectors of this matrix
        /// Note: this uses the QR iterative algorithm by computing a better and better guess and stopping when if limit reached or toleranceValue achieved
        /// </summary>
        /// <param name="toleranceValue"> The maximum diffrence between iterations </param>
        /// <param name="maxIterations"> The maximum number of iterations before force stopping </param>
        /// <returns> A array containing {A, Q}:
        /// <br/>A - A diagnoal matrix containg the eigenValues on the diagonal
        /// <br/>Q - A matrix which has the eigenVectors as it's columns </returns>
        public Matrix[] Eigen(float toleranceValue = 0.01f, int maxIterations = 100)
        {
            // Create A, and Q
            Matrix A = this.Clone() as Matrix;
            Matrix Q = Matrix.Identity(A.height);

            // Iterate untill toleranceValue is met or iteration limit
            for (int i = 0; i < maxIterations; i++)
            {
                Matrix[] QR = A.QR_Decomposition();

                Matrix next = QR[1] * QR[0];
                Q *= QR[0];

                // Check for toleranceValue
                for (int j = 0; j < A.width; j++)
                    if (Math.Abs(next[j, j] - A[j, j]) > toleranceValue)
                        goto loop;
                
                break;
                loop: A = next;
            }

            // Return result
            return new Matrix[] {A, Q};
        }

        /// <summary>
        /// A method to perform a action on each element of the matrix
        /// </summary>
        /// <param name="action"> The action to preform </param>
        public void forEach(Func<float, float> action)
        {
            for (int i = 0; i < this.width; i++)
                for (int j = 0; j < this.height; j++)
                    this[i, j] = action(this[i, j]);
        }

        /// <returns> A clone of this matrix </returns>
        public object Clone()
        {
            return new Matrix(this);
        }

        /// <returns> A string representation of this matrix </returns>
        public override string ToString()
        {
            string output = "";
            for (int j = 0; j < this.height; j++)
            {
                string row = "[";
                for (int i = 0; i < this.width; i++)
                    row += this[i, j] + ",";
                output += row.Substring(0, row.Length - 1) + "]\n";
            }

            return output;
        }

        /// <summary>
        /// A method to create a identity matrix for a given size
        /// </summary>
        /// <param name="size"> The size of the matrix </param>
        /// <returns> A identity matrix with the given size </returns>
        public static Matrix Identity(int size)
        {
            Matrix identity = new Matrix(size, size);

            for (int i = 0; i < size; i++)
                identity[i, i] = 1f;

            return identity;
        }

        #endregion

        #region Static Functions

        /// <summary>
        /// A method to create a vertical vector from a number vector
        /// </summary>
        /// <param name="numbers"> The number vector </param>
        /// <returns> A vertical vector matrix containing the given numbers </returns>
        public static Matrix BuildVerticalVector(params float[] numbers)
        {
            Matrix vector = new Matrix(1, numbers.Length);
            for (int i = 0; i < numbers.Length; i++)
                vector[0, i] = numbers[i];
            return vector;
        }

        /// <summary>
        /// A method to create a horizontal vector from a number vector
        /// </summary>
        /// <param name="numbers"> The number vector </param>
        /// <returns> A horizontal vector matrix containing the given numbers </returns>
        public static Matrix BuildHorizontalVector(params float[] numbers)
        {
            Matrix vector = new Matrix(numbers.Length, 1);
            for (int i = 0; i < numbers.Length; i++)
                vector[i, 0] = numbers[i];
            return vector;
        }

        #endregion

        #region Operators

        // Multiplication
        public static Matrix operator *(Matrix m1, Matrix m2)
        {
            if (m1.width != m2.height)
                throw new System.Exception("Error: Cannot multiply these matrices!");

            Matrix result = new Matrix(m2.width, m1.height);
            for (int i = 0; i < result.width; i++)
                for (int j = 0; j < result.height; j++)
                    for (int k = 0; k < m1.width; k++)
                        result[i, j] += m2[i, k] * m1[k, j];

            return result;
        }

        public static Matrix operator *(Matrix m1, float constant)
        {
            Matrix result = new Matrix(m1.width, m1.height);
            for (int i = 0; i < result.width; i++)
                for (int j = 0; j < result.height; j++)
                    result[i, j] = m1[i, j] * constant;

            return result;
        }

        public static Matrix operator *(float constant, Matrix m1) => m1 * constant;

        // Addition
        public static Matrix operator +(Matrix m1, Matrix m2)
        {
            if (m1.width != m2.width || m1.height != m2.height)
                throw new System.Exception("Error: Cannot add matrices of different dimensions!");

            Matrix result = new Matrix(m1.width, m1.height);
            for (int i = 0; i < result.width; i++)
                for (int j = 0; j < result.height; j++)
                    result[i, j] = m1[i, j] + m2[i, j];

            return result;
        }

        public static Matrix operator +(Matrix m1, float constant)
        {
            Matrix result = new Matrix(m1.width, m1.height);
            for (int i = 0; i < result.width; i++)
                for (int j = 0; j < result.height; j++)
                    result[i, j] = m1[i, j] + constant;

            return result;
        }

        // Subtraction
        public static Matrix operator -(Matrix m1, Matrix m2)
        {
            if (m1.width != m2.width || m1.height != m2.height)
                throw new System.Exception("Error: Cannot subtract matrices of different dimensions!");

            Matrix result = new Matrix(m1.width, m1.height);
            for (int i = 0; i < result.width; i++)
                for (int j = 0; j < result.height; j++)
                    result[i, j] = m1[i, j] - m2[i, j];

            return result;
        }

        public static Matrix operator -(Matrix m1, float constant)
        {
            Matrix result = new Matrix(m1.width, m1.height);
            for (int i = 0; i < result.width; i++)
                for (int j = 0; j < result.height; j++)
                    result[i, j] = m1[i, j] - constant;

            return result;
        }

        // Division
        public static Matrix operator /(Matrix m1, float constant)
        {
            Matrix result = new Matrix(m1.width, m1.height);
            for (int i = 0; i < result.width; i++)
                for (int j = 0; j < result.height; j++)
                    result[i, j] = m1[i, j] / constant;

            return result;
        }

        #endregion

        #region Constructors

        /// <summary>
        /// A constructor to create a new matrix
        /// </summary>
        /// <param name="width"> The matrix width </param>
        /// <param name="height"> The matrix height </param>
        public Matrix(int width, int height) => this.data = new float[width, height];

        /// <summary>
        /// A constructor to create a new matrix
        /// </summary>
        /// <param name="matrix"> The matrix data in a 2d float array </param>
        public Matrix(float[,] matrix) => this.data = matrix;

        // Private constructor for cloning
        private Matrix(Matrix matrix) => this.data = matrix.data.Clone() as float[,];

        #endregion

    }
    
}