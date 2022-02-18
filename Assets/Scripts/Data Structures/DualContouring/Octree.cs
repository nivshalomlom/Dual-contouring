using UnityEngine;
using LinerAlgebra;

/// <summary>
/// A Octree implementation in C#
/// </summary>
/// <typeparam name="T"> The type of data to be stored in this tree </typeparam>
public class Octree<T>
{

    private Cuboid bounds;
    private T data;

    private Octree<T>[] children;

    /// <summary>
    /// A constructor to create a new Octree
    /// </summary>
    /// <param name="bounds"> The 3D cube representing the bounds of the space to be stored in this octree </param>
    public Octree(Cuboid bounds, Octree<T>[] children = null)
    {
        if (children != null && children.Length != 8)
            throw new System.ArgumentException("Octree: children array must be of size 8!");

        this.bounds = bounds;
        this.children = null;
    }

    #region Functions

    /// <returns> The bounds of the space in this Octree </returns>
    public Cuboid GetBounds() => this.bounds;

    /// <summary>
    /// A method to divide the bounds of this Octree to 8 equal sized children
    /// </summary>
    public void SubDivide()
    {
        if (this.children != null)
            return;
        this.children = new Octree<T>[8];

        Vector3 childSize = this.bounds.size / 2f;
        for (int i = 0; i < 8; i++)
        {
            Vector3 offset = new Vector3
            (
                (i & 1) > 0 ? childSize.x : 0,
                (i & 2) > 0 ? childSize.y : 0,
                (i & 4) > 0 ? childSize.z : 0
            );

            Vector3 min = this.bounds.min + offset;
            Vector3 max = min + childSize;

            Cuboid childBounds = new Cuboid(min, max);
            this.children[i] = new Octree<T>(childBounds);
        }
    }

    /// <returns> The data stored in this Octree node </returns>
    public T GetData() => this.data;

    /// <summary>
    /// Stores the given data in this Octree node
    /// </summary>
    /// <param name="data"> The data to be stored </param>
    public void SetData(T data) => this.data = data;

    /// <summary>
    /// A method to delete the array referencing the child nodes
    /// </summary>
    public void DeleteChildrenArray() => this.children = null;

    /// <returns> True if this node is a leaf, false otherwise </returns>
    public bool IsLeaf() => this.children == null;

    // Quick access to octree child nodes
    public Octree<T> this[int i]
    {    
        get => this.children[i];
        set => this.children[i] = value;
    }

    #endregion

}
