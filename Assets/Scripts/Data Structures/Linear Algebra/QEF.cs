using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A method to represent a QEF of the form sigma(normal[i] * (point - intersection[i]))
/// </summary>
public class QEF
{

    #region Properties

    // Computation variables
    private Matrix A_TA, A_TB, B_TB;
    private List<float> fixedValues;

    #endregion

    #region Constructors

    /// <summary>
    /// A constructor to build a new 3D QEF of the form: sigma(normal[i] * (point - intersection[i]))
    /// </summary>
    /// <param name="intersections"> The intersections to add to the QEF </param>
    /// <param name="normals"> The normal of each intersection </param>
    /// <param name="fixedValues"> Values to fix for missing axies </param>
    public QEF(List<Vector3> intersections, List<Vector3> normals, List<float> fixedValues = null)
    {
        // A is a matrix containing normals in each line
        // B is a vector of the dot product of each point and it's normal
        Matrix A = new Matrix(3, intersections.Count);
        Matrix B = new Matrix(1, intersections.Count);

        for (int i = 0; i < intersections.Count; i++)
        {
            A[0, i] = normals[i].x;
            A[1, i] = normals[i].y;
            A[2, i] = normals[i].z;

            B[0, i] = Vector3.Dot(intersections[i], normals[i]);
        }

        this.InitializeInternalMatrices(A, B);
        this.InitializeFixedValues(fixedValues);
    }

    /// <summary>
    /// A constructor to build a new 2D QEF of the form: sigma(normal[i] * (point - intersection[i]))
    /// </summary>
    /// <param name="intersections"> The intersections to add to the QEF </param>
    /// <param name="normals"> The normal of each intersection </param>
    /// <param name="fixedValues"> Values to fix for missing axies </param>
    public QEF(List<Vector2> intersections, List<Vector2> normals, List<float> fixedValues = null)
    {
        // A is a matrix containing normals in each line
        // B is a vector of the dot product of each point and it's normal
        Matrix A = new Matrix(2, intersections.Count);
        Matrix B = new Matrix(1, intersections.Count);

        for (int i = 0; i < intersections.Count; i++)
        {
            A[0, i] = normals[i].x;
            A[1, i] = normals[i].y;

            B[0, i] = Vector2.Dot(intersections[i], normals[i]);
        }

        this.InitializeInternalMatrices(A, B);
        this.InitializeFixedValues(fixedValues);
    }

    // Private constructor for quick copying
    private QEF(Matrix A_TA, Matrix A_TB, Matrix B_TB, List<float> fixedValues = null)
    {
        this.A_TA = A_TA;
        this.A_TB = A_TB;
        this.B_TB = B_TB;

        this.InitializeFixedValues(fixedValues);
    }

    // A method to initialize fixed values
    void InitializeFixedValues(List<float> fixedValues)
    {
        if (fixedValues != null)
            this.fixedValues = new List<float>(fixedValues);
        else 
        {
            this.fixedValues = new List<float>();
            this.fixedValues.Add(float.NegativeInfinity);
            this.fixedValues.Add(float.NegativeInfinity);
            this.fixedValues.Add(float.NegativeInfinity);
        }
    }

    // A method to compute the internal matrices needed for the computation
    void InitializeInternalMatrices(Matrix A, Matrix B)
    {
        Matrix A_T = A.Transpose();
        this.A_TA = A_T * A;
        this.A_TB = A_T * B;
        this.B_TB = B.Transpose() * B;
    }

    #endregion

    #region Functions

    /// <summary>
    /// A method to minimize the QEF
    /// </summary>
    /// <returns> A value X the minimizes the QEF </returns>
    public float[] Solve()
    {
        // U is a matrix containing eigen-vectors in each line
        // eigen[1] is U_T
        Matrix[] eigen = this.A_TA.Eigen(Mathf.Epsilon, 500);
        Matrix U = eigen[1].Transpose();

        // D is a matrix containing 1 / eigen-values on the diagnal
        Matrix D = new Matrix(this.A_TA.width, this.A_TA.height);
        for (int i = 0; i < this.A_TA.width; i++)
        {
            float value = 1 / eigen[0][i, i];
            D[i, i] = value < Mathf.Epsilon ? 0f : value;
        }

        // (A_T * A) ^ -1 = U_T * D * U
        Matrix A_AT_inv = eigen[1] * D * U;
        Matrix X = A_AT_inv * this.A_TB;

        // Constrain axis
        List<float> output = new List<float>();
        for (int i = 0, j = 0; i < this.fixedValues.Count; i++)
        {
            if (fixedValues[i] != float.NegativeInfinity)
                output.Add(fixedValues[i]);
            else output.Add(X[0, j++]);
        }

        // Return result
        return new float[] {output[0], output[1], output[2], this.Evaluate(output.ToArray())};
    }

    /// <summary>
    /// A method to evaluate the QEF for a given point
    /// </summary>
    /// <param name="point"> The point to evaluate </param>
    /// <returns> The error of the point </returns>
    public float Evaluate(float[] point)
    {
        // Turn the input into a vector matrix
        Matrix X = new Matrix(1, this.A_TA.height);
        for (int i = 0; i < X.height; i++)
            X[0, i] = point[i];

        // Compute error
        Matrix X_T = X.Transpose();
        Matrix result = X_T * this.A_TA * X - 2f * X_T * this.A_TB + this.B_TB;
        return result[0, 0];
    }

    /// <summary>
    /// Creates a new QEF with a contrained axis
    /// </summary>
    /// <param name="axis"> The axis to contain </param>
    /// <param name="value"> The value to contain to </param>
    /// <returns> A new QEF with a contrained axis </returns>
    public QEF fixAxis(int axis, float value)
    {
        // Create smaller constrained matrices
        Matrix A_TA = new Matrix(this.A_TA.width - 1, this.A_TA.height - 1);
        Matrix A_TB = new Matrix(this.A_TB.width, this.A_TB.height - 1);

        for (int i = 0; i < this.A_TA.width; i++)
            for (int j = 0; j < this.A_TA.height; j++)
            {
                // Remove the axis row
                if (j == axis)
                    continue;
                
                // Remove the axis column and subtract it from A_TB
                int y = j > axis ? j - 1 : j;
                if (i == axis)
                {
                    A_TB[0, y] = this.A_TB[0, j] - this.A_TA[axis, j] * value;
                    continue;
                }
                
                // Update A_TA
                int x = i > axis ? i - 1 : i;
                A_TA[x, y] = this.A_TA[i, j];
            }

        // Add constraint and create new QEF
        List<float> fixedValues = new List<float>(this.fixedValues);
        fixedValues[axis] = value;
        return new QEF(A_TA, A_TB, this.B_TB, fixedValues);
    }

    /// <summary>
    /// A operator to accumulate 2 QEF's
    /// </summary>
    /// <param name="a"> The first QEF </param>
    /// <param name="b"> The second QEF </param>
    /// <returns> A new QEF from the sum, with no constained axis </returns>
    public static QEF operator +(QEF a, QEF b) => new QEF(a.A_TA + b.A_TA, a.A_TB + b.A_TB, a.B_TB + b.B_TB, null); 

    #endregion

}