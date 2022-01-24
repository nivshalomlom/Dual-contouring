using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DualContour
{

    #region Constants

    // Delegates
    public delegate float Density(float x, float y, float z);
    public delegate bool Condition<T>(T item);

    // Mesh reconstruction
    private static readonly int[,] cellProcFaceMask = 
    {
        {0, 4, 0},
        {1, 5, 0},
        {2, 6, 0},
        {3, 7, 0},
        {0, 2, 1},
        {4, 6, 1},
        {1, 3, 1},
        {5, 7, 1},
        {0, 1, 2},
        {2, 3, 2},
        {4, 5, 2},
        {6, 7, 2}
    };

    private static readonly int[,,] faceProcFaceMask = 
    {
	    {
            {4, 0, 0},
            {5, 1, 0},
            {6, 2, 0},
            {7, 3, 0}
        },
	    {
            {2, 0, 1},
            {6, 4, 1},
            {3, 1, 1},
            {7, 5, 1}
        },
	    {
            {1, 0, 2},
            {3, 2, 2},
            {5, 4, 2},
            {7, 6, 2}
        }
    };

    private static readonly int[,] orders =
    {
        {0, 0, 1, 1},
        {0, 1, 0, 1},
    };

    private static readonly int[,,] faceProcEdgeMask = 
    {
        {
            {1, 4, 0, 5, 1, 1},
            {1, 6, 2, 7, 3, 1},
            {0, 4, 6, 0, 2, 2},
            {0, 5, 7, 1, 3, 2}
        },
        {
            {0, 2, 3, 0, 1, 0},
            {0, 6, 7, 4, 5, 0},
            {1, 2, 0, 6, 4, 2},
            {1, 3, 1, 7, 5, 2}
        },
        {
            {1, 1, 0, 3, 2, 0},
            {1, 5, 4, 7, 6, 0},
            {0, 1, 5, 0, 4, 1},
            {0, 3, 7, 2, 6, 1}
        }
    };

    private static readonly int[,,] edgeProcEdgeMask = 
    {
	    {
            {3,2,1,0,0},
            {7,6,5,4,0}
        },
	    {
            {5,1,4,0,1},
            {7,3,6,2,1}
        },
	    {
            {6,4,2,0,2},
            {7,5,3,1,2}
        },
    };

    private static readonly int[,] edgevmap = 
    {
        {0, 4}, {1, 5}, {2, 6}, {3, 7}, // X - axis
        {0, 2}, {1, 3}, {4, 6}, {5, 7}, // Y - axis
        {0, 1}, {4, 5}, {2, 3}, {6, 7}  // Z - axis
    };

    private static readonly int[,] processEdgeMask = 
    {
        {3, 2, 1, 0},
        {7, 5, 6, 4},
        {11, 10, 9, 8}
    };

    private static readonly int[,] cellProcEdgeMask = 
    {
        {0, 1, 2, 3, 0},
        {4, 5, 6, 7, 0},
        {0, 4, 1, 5, 1},
        {2, 6, 3, 7, 1},
        {0, 2, 4, 6, 2},
        {1, 3, 5, 7, 2}
    };

    // Hyperparameters for QEF fixes
    private const float BIAS_STRENGTH = 0.01f;
    
    private const bool BIAS_FIX = true;
    private const bool CONSTRAINTS_FIX = true;

    #endregion

    #region Variables

    // Field parameters
    private Cubiod bounds;
    private Density function;
    private float isoLevel;

    // The rendered shape
    private Octree<NodeData> root;

    // Density cache
    private Dictionary<Vector3, float> DensityMap;

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
    private List<Vector3> FindIntersections(Cubiod bounds, Density function, float isoLevel) 
    {
        // A macro to quicly access the density cache
        float CacheLookUp(Vector3 key)
        {
            if (!this.DensityMap.ContainsKey(key))
                DensityMap.Add(key, function(key.x, key.y, key.z));
            return this.DensityMap[key];
        }

        // Lists for corners and intersections
        List<Vector3> corners = bounds.GetCorners();
        List<Vector3> intersections = new List<Vector3>();

        // Foreach edge
        for (int i = 0; i < 12; i++)
        {
            // Get its 2 vertices
            Vector3 v1 = corners[edgevmap[i, 0]];
            Vector3 v2 = corners[edgevmap[i, 1]];

            // Compute their density
            float v1Density = CacheLookUp(v1);
            float v2Density = CacheLookUp(v2);

            // Check if above or below isoLevel
            bool v1Sign = v1Density <= isoLevel;
            bool v2Sign = v2Density <= isoLevel;

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
    private float[] ApplyConstraint(Cubiod bounds, QEF err)
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
    private QEF CreateQEF(List<Vector3> intersections, List<Vector3> normals, Cubiod bounds)
    {
        // Add terms to the QEF to add a bias towards the average point of intersection
        if (BIAS_FIX)
            this.AddBias(intersections, normals);

        // Create and solve the QEF
        QEF err = new QEF(intersections, normals);

        // Create the QEF
        return err;
    }

    #endregion

    #region Octree construction

    /// <summary>
    /// A class to hold the data of a qef node
    /// </summary>
    private struct NodeData
    {
        public QEF qef;
        public Vector3 vertex;

        /// <summary>
        /// A constructor to build a new NodeData struct
        /// </summary>
        /// <param name="qef"> The qef of the node </param>
        /// <param name="vertex"> The result of said qef </param>
        public NodeData(QEF qef, Vector3 vertex)
        {
            this.qef = qef;
            this.vertex = vertex;
        }

    }

    /// <summary>
    /// A method to try a simplfy a given octree root into a single vertex
    /// </summary>
    /// <param name="root"> The node to simplify </param>
    /// <param name="function"> The density function of the scalar field the octree is tiling </param>
    /// <returns> A QEF if a simplification is possible, null otherwise </returns>
    private QEF Simplify(Octree<NodeData> root, Density function)
    {
        // Sum all child QEF's
        QEF sum = null;
        for (int i = 0; i < 8; i++)
        {
            if (root[i] == null)
                continue;
            
            // Child not lead, cant simplify
            QEF data = root[i].GetData().qef;
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
    private bool ProcessNode(Octree<NodeData> node, Density function, float isoLevel)
    {
        // Compute intersections
        Cubiod bounds = node.GetBounds();
        List<Vector3> intersections = this.FindIntersections(bounds, function, isoLevel);

        // If cube is fully inside or oustide the shape trim this branch
        if (intersections.Count == 0 || intersections.Count == 8)
            return false;
        
        // Compute normals of intersections
        List<Vector3> normals = new List<Vector3>();
        intersections.ForEach(point => normals.Add(this.ApproximateGradientAt(point, function)));

        // Solve QEF and store result in this node
        QEF qef = this.CreateQEF(intersections, normals, bounds);
        float[] result = qef.Solve();
        node.SetData(new NodeData(qef, new Vector3(result[0], result[1], result[2])));
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
    private bool BuildOctree(Octree<NodeData> root, Density function, float isoLevel, float simplificationTolerenceValue, int depth)
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

        // If simplification is possible
        if (err != null)
        {
            // Compute simlified value
            float[] soultion = err.Solve();
            Cubiod bounds = root.GetBounds();

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

            // Set root as leaf
            root.DeleteChildren();
            root.SetData(new NodeData(err, new Vector3(soultion[0], soultion[1], soultion[2])));
        }
        return true;
    }

    #endregion

    #region Mesh generation

    /// <summary>
    /// A method to convert a given quad to triangles
    /// </summary>
    /// <param name="nodes"> The quad nodes </param>
    /// <param name="direction"> The direction of the edge </param>
    /// <param name="vertices"> The vertex list </param>
    /// <param name="indices"> The trinagle ordering list </param>
    private void ContourProcessEdge(Octree<NodeData>[] nodes, int direction, List<Vector3> vertices, List<int> indices)
    {
        // Control parameters
        int[] ptrs = {-1, -1, -1, -1};
        bool[] signChange = {false, false, false, false};

        float minSize = float.PositiveInfinity;
        int minIndex = 0;
        bool flip = false;

        // For each vertex in quad
        for (int i = 0; i < 4; i++)
        {
            Cubiod bounds = nodes[i].GetBounds();
            List<Vector3> corners = bounds.GetCorners();

            float size = bounds.size.x;
            int edge = processEdgeMask[direction, i];

            // Check for sign change in edge crossing
            bool c0Sign = this.DensityMap[corners[edgevmap[edge, 0]]] <= this.isoLevel;
            bool c1Sign = this.DensityMap[corners[edgevmap[edge, 1]]] <= this.isoLevel;
            signChange[i] = c0Sign != c1Sign;

            // Find smallest cell
            if (size < minSize)
            {
                minSize = size;
                minIndex = i;
                flip = c0Sign;
            }

            // Create vertex and add to list
            ptrs[i] = vertices.Count;
            vertices.Add(nodes[i].GetData().vertex);
        }

        // Render quad id needed
        if (signChange[minIndex])
        {
            // Flip surfce normals if needed
            if (flip)
            {
                indices.Add(ptrs[0]);
                indices.Add(ptrs[1]);
                indices.Add(ptrs[3]);

                indices.Add(ptrs[0]);
                indices.Add(ptrs[3]);
                indices.Add(ptrs[2]);
            }
            else
            {
                indices.Add(ptrs[0]);
                indices.Add(ptrs[3]);
                indices.Add(ptrs[1]);

                indices.Add(ptrs[0]);
                indices.Add(ptrs[2]);
                indices.Add(ptrs[3]);
            }
        }
    }

    /// <summary>
    /// A method to connect edges along a face
    /// </summary>
    /// <param name="nodes"> The edges of the face </param>
    /// <param name="direction"> The connection direction </param>
    /// <param name="vertices"> The vertex list </param>
    /// <param name="indices"> The trinagle ordering list </param>
    private void ContourEdgeProc(Octree<NodeData>[] nodes, int direction, List<Vector3> vertices, List<int> indices)
    {
        // Check input validity
        bool allLeaves = true;
        for (int i = 0; i < 4; i++)
        {
            if (nodes[i] == null)
                return;
            else if (!nodes[i].IsLeaf())
                allLeaves = false;
        }

        // If all are leaves build all triangles
        if (allLeaves)
            this.ContourProcessEdge(nodes, direction, vertices, indices);
        // Resolve all adjecnt edges
        else
            for (int i = 0; i < 2; i++)
            {
                Octree<NodeData>[] edgeNodes = new Octree<NodeData>[4];
                for (int j = 0; j < 4; j++)
                    edgeNodes[j] = nodes[j].IsLeaf() ? nodes[j] : nodes[j][edgeProcEdgeMask[direction, i, j]];

                this.ContourEdgeProc(edgeNodes, edgeProcEdgeMask[direction, i, 4], vertices, indices);
            }
    }

    /// <summary>
    /// A method to connect faces along a given edge
    /// </summary>
    /// <param name="nodes"> The nodes of quad edge </param>
    /// <param name="direction"> The connection direction </param>
    /// <param name="vertices"> The vertex list </param>
    /// <param name="indices"> The trinagle ordering list </param>
    private void ContourFaceProc(Octree<NodeData>[] nodes, int direction, List<Vector3> vertices, List<int> indices) 
    {
        // If one of the nodes is null face dosnt exist
        if (nodes[0] == null || nodes[1] == null)
            return;
        
        // If all are leaves nothing can be done
        if (nodes[0].IsLeaf() && nodes[1].IsLeaf())
            return;

        // Resolve all adjecnt faces
        for (int i = 0; i < 4; i++)
        {
            Octree<NodeData>[] faceNodes = new Octree<NodeData>[2];
            for (int j = 0; j < 2; j++)
                faceNodes[j] = nodes[j].IsLeaf() ? nodes[j] : nodes[j][faceProcFaceMask[direction, i, j]];

            this.ContourFaceProc(faceNodes, faceProcFaceMask[direction, i, 2], vertices, indices);
        }

        // Resolve all adjecnt edges
        for (int i = 0; i < 4; i++)
        {
            Octree<NodeData>[] edgeNodes = new Octree<NodeData>[4];
            int order = faceProcEdgeMask[direction, i, 0];

            for (int j = 0; j < 4; j++)
            {
                Octree<NodeData> node = nodes[orders[order, j]];
                edgeNodes[j] = node.IsLeaf() ? node : node[faceProcEdgeMask[direction, i, j + 1]];
            }

            this.ContourEdgeProc(edgeNodes, faceProcEdgeMask[direction, i, 5], vertices, indices);
        }
    }

    /// <summary>
    /// A method to triangulate a octree cell
    /// </summary>
    /// <param name="node"> The cell to process </param>
    /// <param name="vertices"> The vertex list </param>
    /// <param name="indices"> The trinagle ordering list </param>
    private void ContourCellProc(Octree<NodeData> node, List<Vector3> vertices, List<int> indices)
    {
        // If node is leaf nothing can be done
        if (node.IsLeaf())
            return;

        // Resolve each child
        for (int i = 0; i < 8; i++)
            if (node[i] != null)
                this.ContourCellProc(node[i], vertices, indices);

        // Check for crossings on each edge
        for (int i = 0; i < 12; i++)
        {
            Octree<NodeData>[] faceNodes = 
            {
                node[cellProcFaceMask[i, 0]],
                node[cellProcFaceMask[i, 1]]
            };

            this.ContourFaceProc(faceNodes, cellProcFaceMask[i, 2], vertices, indices);
        }

        // Check for crossings on each face
        for (int i = 0; i < 6; i++)
        {
            Octree<NodeData>[] edgeNodes = new Octree<NodeData>[4];
            for (int j = 0; j < 4; j++)
                edgeNodes[j] = node[cellProcEdgeMask[i, j]];

            this.ContourEdgeProc(edgeNodes, cellProcEdgeMask[i, 4], vertices, indices);
        }
    }

    private Mesh BuildMesh(Octree<NodeData> root)
    {
        // Create triangles
        List<Vector3> vertices = new List<Vector3>();
        List<int> indices = new List<int>();
        this.ContourCellProc(root, vertices, indices);
        
        // Create mesh
        Mesh mesh = new Mesh();
        mesh.SetVertices(vertices);
        mesh.SetTriangles(indices, 0);
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
        return mesh;
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
        // Build octree
        this.isoLevel = isoLevel;
        this.root = new Octree<NodeData>(this.bounds);
        this.BuildOctree(this.root, this.function, isoLevel, simplificationTolerenceValue, maxOctreeDepth);

        // Generate mesh and clear cache
        Mesh mesh = this.BuildMesh(this.root);
        this.DensityMap.Clear();
        return mesh;
    }

    #endregion

    /// <summary>
    /// A constructor to build a new dual contuor
    /// </summary>
    /// <param name="bounds"> The bounds to sample </param>
    /// <param name="function"> A implicit density function representing the shape to contuor </param>
    public DualContour(Cubiod bounds, Density function)
    {
        this.bounds = bounds;
        this.function = function;
        this.isoLevel = 0f;

        this.DensityMap = new Dictionary<Vector3, float>();
    }

}
