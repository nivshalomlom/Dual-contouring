using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DualContour
{

    #region Constants

    // Delegates
    public delegate float Density(float x, float y, float z);
    public delegate bool Condition<T>(T item);

    // Cube edge data
    private static readonly int[] edgeA = {0, 0, 1, 2, 4, 4, 5, 6, 0, 1, 2, 3};
    private static readonly int[] edgeB = {1, 2, 3, 3, 5, 6, 7, 7, 4, 5, 6, 7};

    // Quad order for triangulation
    private static readonly int[][] quads = new int[][]
    {
        new int[] {0, 1, 2, 3},
        new int[] {4, 5, 6, 7},
        new int[] {0, 4, 2, 6},
        new int[] {1, 5, 3, 7},
        new int[] {0, 1, 4, 5},
        new int[] {2, 3, 6, 7}
    };

    // Hyperparameters for QEF fixes
    private const float BIAS_STRENGTH = 0.01f;
    
    private const bool BIAS_FIX = true;
    private const bool CONSTRAINTS_FIX = true;

    #endregion

    #region Variables

    // Field parameters
    private Cube bounds;
    private Density function;

    // The rendered shape
    private Octree<QEF> root;

    #endregion

    #region Utility functions

    /// <summary>
    /// A method to filter a list according to a given filter
    /// </summary>
    /// <param name="list"> The list to filter </param>
    /// <param name="filter"> The filter condition </param>
    /// <typeparam name="T"> The type of the items in the list </typeparam>
    /// <returns> A new list containing only the items that satisfy the filter condition </returns>
    private List<T> FilterList<T>(List<T> list, Condition<T> filter)
    {
        List<T> result = new List<T>();
        foreach (T item in list)
            if (filter(item))
                result.Add(item);

        return result;
    }

    /// <summary>
    /// A method to approximate the intersections between a given cube and a density function
    /// </summary>
    /// <param name="bounds"> The cube </param>
    /// <param name="function"> The density function </param>
    /// <param name="isoLevel"> The value of the density function edge </param>
    /// <returns> A list of all points of intersection </returns>
    private List<Vector3> FindIntersections(Cube bounds, Density function, float isoLevel) 
    {
        List<Vector3> corners = bounds.GetCorners();
        List<Vector3> intersections = new List<Vector3>();

        // Foreach edge
        for (int i = 0; i < 12; i++)
        {
            // Get its 2 vertices
            Vector3 v1 = corners[edgeA[i]];
            Vector3 v2 = corners[edgeB[i]];

            // Compute their density
            float v1Density = function(v1.x, v1.y, v1.z);
            float v2Density = function(v2.x, v2.y, v2.z);

            // Check if above or below isoLevel
            bool v1Sign = v1Density < isoLevel;
            bool v2Sign = v2Density < isoLevel;

            // Check if edge intersects isoLevel
            if (v1Sign != v2Sign)
            {
                // Interpolate point on edge
                float t = (isoLevel - v1Density) / (v2Density - v1Density);
                intersections.Add(v1 + (v2 - v1) * t);
            }
        }

        return intersections;
    }

    /// <summary>
    /// A method to approximate the gradient of a density funtion at a given point
    /// </summary>
    /// <param name="point"> The point to evalutae </param>
    /// <param name="function"> The density function </param>
    /// <returns> A vector3 representing the gradient </returns>
    private Vector3 ApproximateGradientAt(Vector3 point, Density function)
    {
        float eps = Mathf.Epsilon;
        float tEps = 2f * eps;

        return new Vector3
        (
            (function(point.x + eps, point.y, point.z) - function(point.x - eps, point.y, point.z)) / tEps,
            (function(point.x, point.y + eps, point.z) - function(point.x, point.y - eps, point.z)) / tEps,
            (function(point.x, point.y, point.z + eps) - function(point.x, point.y, point.z - eps)) / tEps
        );
    }

    #endregion

    #region QEF solving

    /// <summary>
    /// A method to add intersections and normals that add a bias towards the center of mass
    /// </summary>
    /// <param name="intersections"> The intersections list </param>
    /// <param name="normals"> The normals list </param>
    private void AddBias(List<Vector3> intersections, List<Vector3> normals)
    {
        // Find the average point of intersection
        Vector3 massPoint = Vector3.zero;
        intersections.ForEach(v => massPoint += v);
        massPoint /= intersections.Count;

        intersections.Add(massPoint);
        intersections.Add(massPoint);
        intersections.Add(massPoint);

        // Create axis normals
        normals.Add(new Vector3(BIAS_STRENGTH, 0f, 0f));
        normals.Add(new Vector3(0f, BIAS_STRENGTH, 0f));
        normals.Add(new Vector3(0f, 0f, BIAS_STRENGTH));
    }

    /// <summary>
    /// A method to solve a given QEF and keep the result inside the given bounds
    /// </summary>
    /// <param name="bounds"> The bounds of work </param>
    /// <param name="err"> The QEF to solve </param>
    /// <returns> A point the minimizes the QEF inside the given bounds </returns>
    private float[] ApplyConstraint(Cube bounds, QEF err)
    {
        // A function to filter invalid points
        bool Bounded(float[] result)
        {
            Vector3 point = new Vector3(result[0], result[1], result[2]);
            return bounds.Contains(point);
        }

        // Get bounds size and create result list
        Vector3 size = bounds.size;
        List<float[]> results = new List<float[]>();

        // Constrain QEF to 6 planes bordering the cell
        results.Add(err.fixAxis(0, bounds.min.x).Solve());
        results.Add(err.fixAxis(0, bounds.min.x + size.x).Solve());
        results.Add(err.fixAxis(1, bounds.min.y).Solve());
        results.Add(err.fixAxis(1, bounds.min.y + size.y).Solve());
        results.Add(err.fixAxis(2, bounds.min.z).Solve());
        results.Add(err.fixAxis(2, bounds.min.z + size.z).Solve());

        // Filter invalid point
        results = this.FilterList(results, Bounded);

        // If all points filtered, try the 12 lines bordering the cell
        if (results.Count == 0)
        {
            results.Add(err.fixAxis(1, bounds.min.y).fixAxis(0, bounds.min.x).Solve());
            results.Add(err.fixAxis(1, bounds.min.y + size.y).fixAxis(0, bounds.min.x).Solve());
            results.Add(err.fixAxis(1, bounds.min.y).fixAxis(0, bounds.min.x + size.x).Solve());
            results.Add(err.fixAxis(1, bounds.min.y + size.y).fixAxis(0, bounds.min.x + size.x).Solve());
            results.Add(err.fixAxis(2, bounds.min.z).fixAxis(0, bounds.min.x).Solve());
            results.Add(err.fixAxis(2, bounds.min.z + size.z).fixAxis(0, bounds.min.x).Solve());
            results.Add(err.fixAxis(2, bounds.min.z).fixAxis(0, bounds.min.x + size.x).Solve());
            results.Add(err.fixAxis(2, bounds.min.z + size.z).fixAxis(0, bounds.min.x + size.x).Solve());
            results.Add(err.fixAxis(2, bounds.min.z).fixAxis(1, bounds.min.y).Solve());
            results.Add(err.fixAxis(2, bounds.min.z + size.z).fixAxis(1, bounds.min.y).Solve());
            results.Add(err.fixAxis(2, bounds.min.z).fixAxis(1, bounds.min.y + size.y).Solve());
            results.Add(err.fixAxis(2, bounds.min.z + size.z).fixAxis(1, bounds.min.y + size.y).Solve());

            // Filter invalid point
            results = this.FilterList(results, Bounded);
        }

        // If no valid point found try corners of the cube
        if (results.Count == 0)
            foreach (Vector3 corner in bounds.GetCorners())
                results.Add(new float[] {corner.x, corner.y, corner.z});

        // Select result with lowest error
        float[] best = null;
        float bestErr = float.PositiveInfinity;
        foreach (float[] result in results)
        {
            float pointErr = err.Evaluate(result);
            if (bestErr > pointErr)
            {
                best = result;
                bestErr = pointErr;
            }
        }

        return best;
    }

    /// <summary>
    /// A method to build and solve a QEF from the given intersections inside the given bounds
    /// </summary>
    /// <param name="intersections"> The intersections to add to the QEF </param>
    /// <param name="normals"> A list containing the normal of each intersection point </param>
    /// <param name="bounds"> The bounds of work </param>
    /// <returns> The point the minimizes the QEF and it's error in a array in this format {point.x, point.y, point.z, Error(point)} </returns>
    private QEF CreateQEF(List<Vector3> intersections, List<Vector3> normals, Cube bounds)
    {
        // Add terms to the QEF to add a bias towards the average point of intersection
        if (BIAS_FIX)
            this.AddBias(intersections, normals);

        // Create and solve the QEF
        QEF err = new QEF(intersections, normals);

        // Create the QEF
        err.Solve();
        return err;
    }

    #endregion

    #region Octree construction

    /// <summary>
    /// A method to try a simplfy a given octree root into a single vertex
    /// </summary>
    /// <param name="root"> The node to simplify </param>
    /// <param name="function"> The density function of the scalar field the octree is tiling </param>
    /// <returns> A QEF if a simplification is possible, null otherwise </returns>
    private QEF Simplify(Octree<QEF> root, Density function)
    {
        // Sum all child QEF's
        QEF sum = null;
        for (int i = 0; i < 8; i++)
        {
            if (root[i] == null)
                continue;
            
            // Child not lead, cant simplify
            QEF data = root[i].GetData();
            if (data == null)
                return null;

            if (sum == null)
                sum = data;
            else sum += data;
        }

        return sum;
    }

    /// <summary>
    /// A method to process a octree node and solve it's QEF
    /// </summary>
    /// <param name="node"> The node </param>
    /// <param name="function"> The density function tiling this field </param>
    /// <param name="isoLevel"> The value of the density function edge </param>
    /// <returns> True if the node contains geometry, faslse otherwise </returns>
    private bool ProcessNode(Octree<QEF> node, Density function, float isoLevel)
    {
        // Compute intersections
        Cube bounds = node.GetBounds();
        List<Vector3> intersections = this.FindIntersections(bounds, function, isoLevel);

        // If cube is fully inside or oustide the shape trim this branch
        if (intersections.Count == 0 || intersections.Count == 8)
            return false;
        
        // Compute normals of intersections
        List<Vector3> normals = new List<Vector3>();
        intersections.ForEach(point => normals.Add(this.ApproximateGradientAt(point, function)));

        // Solve QEF and store result in this node
        QEF data = this.CreateQEF(intersections, normals, bounds);
        node.SetData(data);
        return true;
    }

    /// <summary>
    /// A method to build a octree for a density function
    /// </summary>
    /// <param name="root"> The root of the tree </param>
    /// <param name="function"> The density function </param>
    /// <param name="isoLevel"> The value of the density function edge </param>
    /// <param name="simplificationTolerenceValue"> The maximum residual to allow simplification on </param>
    /// <param name="depth"> The maximum depth for the octree </param>
    /// <returns> True if the density intersects the octree, false otherwise </returns>
    private bool BuildOctree(Octree<QEF> root, Density function, float isoLevel, float simplificationTolerenceValue, int depth)
    {
        // If maximum depth reached
        if (depth == 0)
            return this.ProcessNode(root, function, isoLevel);

        // Subdivide the tree
        root.SubDivide();
        int liveChildren = 0;

        // Check each child node
        for (int i = 0; i < 8; i++)
        {
            // If branch is empty trim it
            if (!this.BuildOctree(root[i], function, isoLevel, simplificationTolerenceValue, depth - 1))
                root[i] = null;
            else liveChildren++;
        }

        // If all child nodes are trimmed trim the root as well
        if (liveChildren == 0)
            return false;

        // Attempt to simplify the root node
        QEF err = this.Simplify(root, function);

        // If simplification succeeded and is under the given tolernce value
        if (err != null)
        {
            // Compute simlified value
            float[] soultion = err.Solve();
            Cube bounds = root.GetBounds();

            // If not in bounds apply constraint
            if (!bounds.Contains(new Vector3(soultion[0], soultion[1], soultion[2])))
            {
                float[] bounded = this.ApplyConstraint(bounds, err);
                soultion[0] = bounded[0];
                soultion[1] = bounded[1];
                soultion[2] = bounded[2];
                soultion[3] = err.Evaluate(bounded);
            }

            // If above tolernce value reject
            if (soultion[3] >= simplificationTolerenceValue)
                return true;

            // Set root as pseudo leaf
            root.DeleteChildren();
            root.SetData(err);
        }
        return true;
    }

    #endregion

    #region Mesh generation

    private Mesh BuildMesh(Octree<QEF> root)
    {
        return null;
    }

    /// <summary>
    /// A method to generate mesh for the contuor
    /// </summary>
    /// <param name="isoLevel"> The value of the density function edge </param>
    /// <param name="simplificationTolerenceValue"> The error threshold for simplifying octree nodes </param>
    /// <param name="maxOctreeDepth"> The level of detail for the octree, max number of vertexes = 8 ^ maxOctreeDepth </param>
    /// <returns> A mesh of the contuor </returns>
    public Mesh Generate(float isoLevel, float simplificationTolerenceValue, int maxOctreeDepth)
    {
        this.root = new Octree<QEF>(this.bounds);
        this.BuildOctree(this.root, this.function, isoLevel, simplificationTolerenceValue, maxOctreeDepth);
        return this.BuildMesh(this.root);
    }

    #endregion

    /// <summary>
    /// A constructor to build a new dual contuor
    /// </summary>
    /// <param name="bounds"> The bounds to sample </param>
    /// <param name="function"> A implicit density function representing the shape to contuor </param>
    public DualContour(Cube bounds, Density function)
    {
        this.bounds = bounds;
        this.function = function;
    }

    // FIXME: Debug zone

    private void Get(Octree<QEF> root, List<Vector3> result)
    {
        if (root.GetData() != null)
        {
            float[] data = root.GetData().GetLastResult();
            result.Add(new Vector3(data[0], data[1], data[2]));
            return;
        }

        for (int i = 0; i < 8; i++)
            if (root[i] != null)
                Get(root[i], result);
    }

    public List<Vector3> GetVertexes()
    {
        List<Vector3> result = new List<Vector3>();
        this.Get(this.root, result);
        return result;
    }

    /*
    
    TODO:
    1. Test the new octree building - done
    2. Triangulation
    
    */

}
