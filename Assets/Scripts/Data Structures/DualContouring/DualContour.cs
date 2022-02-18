using System.Collections.Generic;
using UnityEngine;

using DualContouring.Utility;
using DualContouring;

using LinerAlgebra;

/// <summary>
/// A class to take a density function and build a mesh from it
/// </summary>
public class DualContour
{

    #region Constants

    // Hyperparameters for QEF fixes
    private const float BIAS_STRENGTH = 0.01f;
    
    private const bool BIAS_FIX = true;
    private const bool CONSTRAINTS_FIX = true;

    #endregion

    #region Variables

    // Field parameters
    private Cuboid bounds;
    private Density function;
    private float isoLevel;

    // The rendered shape
    private Octree<NodeData> root;

    #endregion

    #region QEF solving

    /// <summary>
    /// A method to add intersections and normals that add a bias towards the center of mass
    /// </summary>
    /// <param name="intersections"> The intersections list </param>
    /// <param name="normals"> The normals list </param>
    private static void AddBias(List<Vector3> intersections, List<Vector3> normals)
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
    private static float[] ApplyConstraint(Cuboid bounds, QEF err)
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
        results = ContourUtility.FilterList(results, Bounded);

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
            results = ContourUtility.FilterList(results, Bounded);
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
    private static QEF CreateQEF(List<Vector3> intersections, List<Vector3> normals)
    {
        // Add terms to the QEF to add a bias towards the average point of intersection
        if (BIAS_FIX)
            AddBias(intersections, normals);

        // Create and solve the QEF
        QEF err = new QEF(intersections, normals);

        // Create the QEF
        return err;
    }

    #endregion

    #region Octree construction

    /// <summary>
    /// A method to try and simplify a given octree node
    /// </summary>
    /// <param name="root"> The node to simplify </param>
    /// <param name="function"> The function we are contuoring </param>
    /// <param name="simplificationTolerenceValue"> The maximum error value the a simplification can have to be accepted </param>
    private static void Simplify(Octree<NodeData> root, float simplificationTolerenceValue)
    {
        // Signs to approximate corner encoding
        int[] signs = {-1, -1, -1, -1, -1, -1, -1, -1};
	    int midSign = -1;

        // Sum all child QEF's
        QEF sum = null;
        for (int i = 0; i < 8; i++)
        {
            // Skip trimmed branches
            if (root[i] == null)
                continue;
            
            // Child not lead, cant simplify
            QEF data = root[i].GetData().qef;
            if (data == null)
                return;

            // Accumulate qef
            if (sum == null)
                sum = data;
            else sum += data;

            // Update signs
            int encoding = root[i].GetData().cornerEncoding;
            signs[i] = (encoding>> i) & 1;
            midSign = (encoding >> (7 - i)) & 1;
        }

        // Solve sum QEF and bound if needed
        float[] solution = sum.Solve();
        Cuboid bounds = root.GetBounds();
        if (!bounds.Contains(new Vector3(solution[0], solution[1], solution[2])))
        {
            float[] bounded = ApplyConstraint(bounds, sum);
            solution[0] = bounded[0];
            solution[1] = bounded[1];
            solution[2] = bounded[2];
            solution[3] = sum.Evaluate(bounded);
        }

        // Reject simplification if error is to high
        if (solution[3] > simplificationTolerenceValue)
            return;

        // Build the new corner encoding
        int rootCorners = 0;
        for (int i = 0; i < 8; i++)
            rootCorners |= (signs[i] == -1 ? midSign : signs[i]) << i;

        // Turn root in a pseado leaf
        root.DeleteChildrenArray();
        root.SetData(new NodeData(sum, new Vector3(solution[0], solution[1], solution[2]), rootCorners, Vector3.Distance(bounds.min, bounds.max)));
    }

    /// <summary>
    /// A method to process a octree node and solve it's QEF
    /// </summary>
    /// <param name="node"> The node </param>
    /// <param name="function"> The density function tiling this field </param>
    /// <param name="isoLevel"> The value of the density function edge </param>
    /// <returns> True if the node contains geometry, faslse otherwise </returns>
    private static bool ProcessNode(Octree<NodeData> node, Density function, float isoLevel)
    {
        // Compute intersections
        int encoding;
        Cuboid bounds = node.GetBounds();
        List<Vector3> intersections = ContourUtility.FindIntersections(bounds, function, isoLevel, out encoding);

        // If cube is fully inside or oustide the shape trim this branch
        if (intersections.Count == 0)
            return false;
        
        // Compute normals of intersections
        List<Vector3> normals = new List<Vector3>();
        intersections.ForEach(point => normals.Add(ContourUtility.ApproximateGradientAt(point, function)));

        // Solve QEF
        QEF qef = CreateQEF(intersections, normals);
        float[] result = qef.Solve();

        // Solve bounded if needed
        if (!bounds.Contains(new Vector3(result[0], result[1], result[2])))
            result = ApplyConstraint(bounds, qef);

        // Store data 
        node.SetData(new NodeData(qef, new Vector3(result[0], result[1], result[2]), encoding, Vector3.Distance(bounds.min, bounds.max)));
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
    private static bool BuildOctree(Octree<NodeData> root, Density function, float isoLevel, float simplificationTolerenceValue, int depth)
    {
        // If maximum depth reached
        if (depth == 0)
            return ProcessNode(root, function, isoLevel);

        // Subdivide the tree
        root.SubDivide();
        int liveChildren = 0;

        // Check each child node
        for (int i = 0; i < 8; i++)
        {
            // If branch is empty trim it
            if (!BuildOctree(root[i], function, isoLevel, simplificationTolerenceValue, depth - 1))
                root[i] = null;
            else liveChildren++;
        }

        // If all child nodes are trimmed trim the root as well
        if (liveChildren == 0)
            return false;

        // Attempt to simplify the root node
        Simplify(root, simplificationTolerenceValue);
        return true;
    }

    #endregion

    #region Mesh generation

    /// <summary>
    /// A method to build the contuor's mesh
    /// </summary>
    /// <param name="root"> The root of the octree to start building from </param>
    /// <returns> The mesh of the contuor </returns>
    private static List<Mesh> BuildMesh(Octree<NodeData> root)
    {
        // Create lists for mesh data
        List<Vector3> vertices = new List<Vector3>();
        List<int> indices = new List<int>();
        List<Mesh> meshes = new List<Mesh>();

        // Create meshes
        ContourBuilder.ContourCellProc(root, vertices, indices, meshes);
        if (vertices.Count > 0)
            ContourBuilder.CreateMesh(vertices, indices, meshes);

        // Return meshes
        return meshes;
    }

    /// <summary>
    /// A method to generate mesh for the contuor
    /// </summary>
    /// <param name="isoLevel"> The value of the density function edge </param>
    /// <param name="simplificationTolerenceValue"> The error threshold for simplifying octree nodes </param>
    /// <param name="maxOctreeDepth"> The level of detail for the octree, max number of vertexes = 8 ^ maxOctreeDepth </param>
    /// <returns> A mesh of the contuor </returns>
    public List<Mesh> Generate(float isoLevel, float simplificationTolerenceValue, int maxOctreeDepth)
    {
        // Build octree
        this.isoLevel = isoLevel;
        this.root = new Octree<NodeData>(this.bounds);

        // Check if shape was found
        if (!BuildOctree(this.root, this.function, isoLevel, simplificationTolerenceValue, maxOctreeDepth))
            return null;

        // Generate mesh
        return BuildMesh(this.root);
    }

    #endregion

    /// <summary>
    /// A constructor to build a new dual contuor
    /// </summary>
    /// <param name="bounds"> The bounds to sample </param>
    /// <param name="function"> A implicit density function representing the shape to contuor </param>
    public DualContour(Cuboid bounds, Density function)
    {
        this.bounds = bounds;
        this.function = function;
        this.isoLevel = 0f;
    }

}
