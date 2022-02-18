using System.Collections.Generic;
using UnityEngine;

using LinerAlgebra;

namespace DualContouring
{

    /// <summary>
    /// A class to hold the data of a qef node
    /// </summary>
    public struct NodeData
    {
        public QEF qef;
        public Vector3 vertex;
        public int cornerEncoding, index;
        public float size;

        /// <summary>
        /// A constructor to build a new NodeData struct
        /// </summary>
        /// <param name="qef"> The qef of the node </param>
        /// <param name="vertex"> The result of said qef </param>
        /// <param name="cornerEncoding"> A number whose bits represent what corners crossed the boundry </param>
        /// <param name="size"> The length of the diagnel of the node bounds </param>
        public NodeData(QEF qef, Vector3 vertex, int cornerEncoding, float size)
        {
            this.qef = qef;
            this.vertex = vertex;
            this.cornerEncoding = cornerEncoding;
            this.index = -1;
            this.size = size;
        }

    }

    /// <summary>
    /// A static class to handle mesh building from a given octree
    /// </summary>
    public static class ContourBuilder
    {
        private static int MAX_VERTS_PER_MESH = 30000;

        /// <summary>
        /// A method to create a new mesh and add it to the provided mesh list
        /// </summary>
        /// <param name="vertices"> The mesh vertices </param>
        /// <param name="indices"> The mesh triangles </param>
        /// <param name="meshes"> The mesh list </param>
        public static void CreateMesh(List<Vector3> vertices, List<int> indices, List<Mesh> meshes)
        {
            // Create new mesh
            Mesh mesh = new Mesh();
            mesh.SetVertices(vertices);
            mesh.SetTriangles(indices, 0);
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();

            // Add new mesh to list add clear buffers
            meshes.Add(mesh);
            vertices.Clear();
            indices.Clear();
        }

        /// <summary>
        /// A method to convert a given quad to triangles
        /// </summary>
        /// <param name="nodes"> The quad nodes </param>
        /// <param name="direction"> The direction of the edge </param>
        /// <param name="vertices"> The vertex list </param>
        /// <param name="indices"> The trinagle ordering list </param>
        private static void ContourProcessEdge(Octree<NodeData>[] nodes, int direction, List<Vector3> vertices, List<int> indices, List<Mesh> meshes)
        {
            // Get needed tables
            int[,] processEdgeMask = DualContouringData.processEdgeMask;
            int[,] edgevmap = DualContouringData.edgevmap;

            // Collapse buffers to mesh if needed
            if (vertices.Count + 4 > MAX_VERTS_PER_MESH)
                CreateMesh(vertices, indices, meshes);

            // Control parameters
            int[] ptrs = {-1, -1, -1, -1};
            bool[] signChange = {false, false, false, false};

            // Quad creation parameters
            float minSize = float.PositiveInfinity;
            int minIndex = 0;
            bool flip = false;

            // For each vertex in quad
            for (int i = 0; i < 4; i++)
            {
                // Node bounds and data
                Cuboid bounds = nodes[i].GetBounds();
                NodeData data = nodes[i].GetData();

                // Node size and edge data
                float size = data.size;
                int edge = processEdgeMask[direction, i];
                int encoding = data.cornerEncoding;

                // Check for sign change in edge crossing
                bool c0Sign = ((encoding >> edgevmap[edge, 0]) & 1) > 0;
                bool c1Sign = ((encoding >> edgevmap[edge, 1]) & 1) > 0;
                signChange[i] = c0Sign != c1Sign;

                // Find smallest cell
                if (size < minSize)
                {
                    minSize = size;
                    minIndex = i;
                    flip = c0Sign;
                }

                // Prevent duplicates vertices
                if (data.index == -1)
                {
                    data.index = vertices.Count;
                    vertices.Add(data.vertex);
                }
                ptrs[i] = data.index;
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
        private static void ContourEdgeProc(Octree<NodeData>[] nodes, int direction, List<Vector3> vertices, List<int> indices, List<Mesh> meshes)
        {
            // Get needed tables
            int[,,] edgeProcEdgeMask = DualContouringData.edgeProcEdgeMask;

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
                ContourProcessEdge(nodes, direction, vertices, indices, meshes);
            // Resolve all adjecnt edges
            else
                for (int i = 0; i < 2; i++)
                {
                    Octree<NodeData>[] edgeNodes = new Octree<NodeData>[4];
                    for (int j = 0; j < 4; j++)
                        edgeNodes[j] = nodes[j].IsLeaf() ? nodes[j] : nodes[j][edgeProcEdgeMask[direction, i, j]];

                    ContourEdgeProc(edgeNodes, edgeProcEdgeMask[direction, i, 4], vertices, indices, meshes);
                }
        }

        /// <summary>
        /// A method to connect faces along a given edge
        /// </summary>
        /// <param name="nodes"> The nodes of quad edge </param>
        /// <param name="direction"> The connection direction </param>
        /// <param name="vertices"> The vertex list </param>
        /// <param name="indices"> The trinagle ordering list </param>
        private static void ContourFaceProc(Octree<NodeData>[] nodes, int direction, List<Vector3> vertices, List<int> indices, List<Mesh> meshes) 
        {
            // Get needed tables
            int[,,] faceProcFaceMask = DualContouringData.faceProcFaceMask;
            int[,,] faceProcEdgeMask = DualContouringData.faceProcEdgeMask;
            int[,] orders = DualContouringData.orders;

            // If one of the nodes is null face dosnt exist
            if (nodes[0] == null || nodes[1] == null)
                return;
            
            // If all are leaves nothing can be done
            if (nodes[0].IsLeaf() && nodes[1].IsLeaf())
                return;

            // Resolve all adjacent faces
            for (int i = 0; i < 4; i++)
            {
                Octree<NodeData>[] faceNodes = new Octree<NodeData>[2];
                for (int j = 0; j < 2; j++)
                    faceNodes[j] = nodes[j].IsLeaf() ? nodes[j] : nodes[j][faceProcFaceMask[direction, i, j]];

                ContourFaceProc(faceNodes, faceProcFaceMask[direction, i, 2], vertices, indices, meshes);
            }

            // Resolve all adjacent edges
            for (int i = 0; i < 4; i++)
            {
                Octree<NodeData>[] edgeNodes = new Octree<NodeData>[4];
                int order = faceProcEdgeMask[direction, i, 0];

                for (int j = 0; j < 4; j++)
                {
                    Octree<NodeData> node = nodes[orders[order, j]];
                    edgeNodes[j] = node.IsLeaf() ? node : node[faceProcEdgeMask[direction, i, j + 1]];
                }

                ContourEdgeProc(edgeNodes, faceProcEdgeMask[direction, i, 5], vertices, indices, meshes);
            }
        }

        /// <summary>
        /// A method to triangulate a octree cell
        /// </summary>
        /// <param name="node"> The cell to process </param>
        /// <param name="vertices"> The vertex list </param>
        /// <param name="indices"> The trinagle ordering list </param>
        public static void ContourCellProc(Octree<NodeData> node, List<Vector3> vertices, List<int> indices, List<Mesh> meshes)
        {
            // Get needed tables
            int[,] cellProcFaceMask = DualContouringData.cellProcFaceMask;
            int[,] cellProcEdgeMask = DualContouringData.cellProcEdgeMask;

            // If node is leaf nothing can be done
            if (node.IsLeaf())
                return;

            // Resolve each child
            for (int i = 0; i < 8; i++)
                if (node[i] != null)
                    ContourCellProc(node[i], vertices, indices, meshes);

            // Check for crossings on each edge
            for (int i = 0; i < 12; i++)
            {
                Octree<NodeData>[] faceNodes = 
                {
                    node[cellProcFaceMask[i, 0]],
                    node[cellProcFaceMask[i, 1]]
                };

                ContourFaceProc(faceNodes, cellProcFaceMask[i, 2], vertices, indices, meshes);
            }

            // Check for crossings on each face
            for (int i = 0; i < 6; i++)
            {
                Octree<NodeData>[] edgeNodes = new Octree<NodeData>[4];
                for (int j = 0; j < 4; j++)
                    edgeNodes[j] = node[cellProcEdgeMask[i, j]];

                ContourEdgeProc(edgeNodes, cellProcEdgeMask[i, 4], vertices, indices, meshes);
            }
        }

    }

}