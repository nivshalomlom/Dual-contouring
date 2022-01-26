using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A class to represent a 3D cuboid
/// </summary>
public struct Cuboid
{

    public Vector3 min, max;

    public Vector3 size 
    {
        get { return this.max - this.min; }
        set { this.max = this.min + value; }
    }

    public Vector3 center
    {
        get { return this.min + this.size / 2f; }
        set 
        {
            Vector3 halfExtents = this.size / 2f;
            this.min = value - halfExtents;
            this.max = value + halfExtents;
        }
    }

    /// <summary>
    /// A constructor to create a new cube
    /// </summary>
    /// <param name="min"> The bottom anchor of the cube </param>
    /// <param name="max"> The upper anchor of the cube </param>
    public Cuboid(Vector3 min, Vector3 max)
    {
        this.min = min;
        this.max = max;
    }

    /// <summary>
    /// A method to check if a given point is contained in this cube
    /// </summary>
    /// <param name="point"> The point to be checked </param>
    /// <returns> True if contained, false otherwise </returns>
    public bool Contains(Vector3 point)
    {
        if (this.min.x >= point.x || this.max.x <= point.x)
            return false;
        
        if (this.min.y >= point.y || this.max.y <= point.y)
            return false;
        
        if (this.min.z >= point.z || this.max.z <= point.z)
            return false;

        return true;
    }

    /// <returns> A lise containing the 8 corners of the cube </returns>
    public List<Vector3> GetCorners()
    {
        List<Vector3> corners = new List<Vector3>();
        for (int i = 0; i < 8; i++)
        {
            Vector3 offset = new Vector3
            (
                (i & 1) > 0 ? this.size.x : 0,
                (i & 2) > 0 ? this.size.y : 0,
                (i & 4) > 0 ? this.size.z : 0
            );

            corners.Add(this.min + offset);
        }

        return corners;
    }

}
