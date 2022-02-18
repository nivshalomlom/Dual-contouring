using System.Collections.Generic;
using UnityEngine;

using LinerAlgebra;

namespace DualContouring.Utility
{

    /// <summary>
    /// A class for dual contouring utility methods
    /// </summary>
    class ContourUtility
    {
        /// <summary>
        /// A method to filter a list according to a given filter
        /// </summary>
        /// <param name="list"> The list to filter </param>
        /// <param name="filter"> The filter condition </param>
        /// <typeparam name="T"> The type of the items in the list </typeparam>
        /// <returns> A new list containing only the items that satisfy the filter condition </returns>
        public static List<T> FilterList<T>(List<T> list, Condition<T> filter)
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
        public static List<Vector3> FindIntersections(Cuboid bounds, Density function, float isoLevel, out int cornerEncoding) 
        {
            // Lists for corners and intersections
            List<Vector3> corners = bounds.GetCorners();
            List<Vector3> intersections = new List<Vector3>();

            // Foreach edge
            cornerEncoding = 0;
            for (int i = 0; i < 12; i++)
            {
                int v1Index = DualContouringData.edgevmap[i, 0];
                int v2Index = DualContouringData.edgevmap[i, 1];
                // Get its 2 vertices
                Vector3 v1 = corners[v1Index];
                Vector3 v2 = corners[v2Index];

                // Compute their density
                float v1Density = function(v1.x, v1.y, v1.z);
                float v2Density = function(v2.x, v2.y, v2.z);

                // Check if above or below isoLevel
                bool v1Sign = v1Density <= isoLevel;
                bool v2Sign = v2Density <= isoLevel;

                // Update cornerEncoding
                if (v1Sign)
                    cornerEncoding |= 1 << v1Index;
                if (v2Sign)
                    cornerEncoding |= 1 << v2Index;

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
        public static Vector3 ApproximateGradientAt(Vector3 point, Density function)
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

        /// <summary>
        /// A method to filter a octree for a given condition
        /// </summary>
        /// <param name="filter"> The condition </param>
        /// <param name="root"> The root of the octree </param>
        /// <param name="output"> The output copy tree </param>
        /// <returns> True if the current root has leaves that satisfy the condition </returns>
        private static bool FilterOctree(Condition<Octree<NodeData>> filter, Octree<NodeData> root, Octree<NodeData> output)
        {
            if (root.IsLeaf())
                return filter(root);

            output.SubDivide();
            bool hasChildren = false;

            for (int i = 0; i < 8; i++)
            {
                // If relevent copy data
                if (root[i] != null && FilterOctree(filter, root[i], output[i]))
                {
                    output[i].SetData(root[i].GetData());
                    hasChildren = true;
                }
                // Else delete it
                else output[i] = null;
            }

            return hasChildren;
        }

        /// <summary>
        /// A method to filter a octree for a given condition
        /// </summary>
        /// <param name="filter"> The condition </param>
        /// <param name="root"> The root of the octree </param>
        /// <returns> A new octree that contains only nodes that have leaves that satisfy the condition, null if none satisfy </returns>
        public static Octree<NodeData> FilterOctree(Condition<Octree<NodeData>> filter, Octree<NodeData> root)
        {
            Octree<NodeData> output = new Octree<NodeData>(root.GetBounds());
            if (FilterOctree(filter, root, output))
            {
                output.SetData(root.GetData());
                return output;
            }
            return null;
        }

    }

}