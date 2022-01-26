using System.Collections.Generic;
using System;

/// <summary>
/// A implementation of a max binary heap <br/> 
/// source: <see cref="https://en.wikipedia.org/wiki/Binary_heap"/>
/// </summary>
/// <typeparam name="T"> the type of items to be stored in the heap, needs to implement the System.IComparable interface </typeparam>
public class MaxHeap<T> where T : IComparable<T>
{
    
    private List<T> heap;

    /// <summary>
    /// The amount of elements in the heap
    /// </summary>
    public int Count 
    { 
        get { return heap.Count; }
    }

    /// <summary>
    /// A constructor for a new max heap
    /// </summary>
    /// <param name="items"> optional, items to be inserted on creation </param>
    public MaxHeap(params T[] items)
    {
        this.heap = new List<T>();

        foreach (T item in items)
            this.Insert(item);
    }

    #region Utility methods

    private int Parent(int index) => (index - 1) / 2;
    private int LeftChild(int index) => 2 * index + 1;
    private int RightChild(int index) => 2 * index + 2;
    private bool IsInBounds(int index) => index > -1 && index < this.heap.Count;

    /// <summary>
    /// A method to swap to elements in the heap
    /// </summary>
    /// <param name="index1"> The index of the first element </param>
    /// <param name="index2"> The index of the secon element</param>
    private void Swap(int index1, int index2)
    {
        T temp = this.heap[index1];
        this.heap[index1] = this.heap[index2];
        this.heap[index2] = temp;
    }

    /// <summary>
    /// A recursive method to check if a given item is in the heap
    /// </summary>
    /// <param name="currentIndex"> The index the search is at </param>
    /// <param name="item"> The item to find </param>
    /// <returns> True if contained, false otherwise </returns>
    private bool Contains(int currentIndex, T item)
    {
        // If item is found
        if (this.heap[currentIndex].Equals(item))
            return true;

        // Else check left if exists and smaller then the target item
        int leftIndex = this.LeftChild(currentIndex);
        if (this.IsInBounds(leftIndex) && this.heap[currentIndex].CompareTo(this.heap[leftIndex]) > 0 && this.Contains(this.LeftChild(currentIndex), item))
            return true;

        // Else check right if exists and smaller then the target item
        int rightIndex = this.RightChild(currentIndex);
        if (this.IsInBounds(rightIndex) && this.heap[currentIndex].CompareTo(this.heap[rightIndex]) > 0 && this.Contains(rightIndex, item))
            return true;
    
        // Else item not found
        return false;
    }

    #endregion

    #region Input / Output

    /// <summary>
    /// A method to insert a item into the heap
    /// </summary>
    /// <param name="item"> the item to be inserted </param>
    public void Insert(T item)
    {
        // Add item to heap and get it's parent
        int itemIndex = this.heap.Count;
        this.heap.Add(item);
        int parentIndex = this.Parent(itemIndex);

        // While a parent exists and is bigger then the new item, swap them
        while (this.IsInBounds(parentIndex) && this.heap[itemIndex].CompareTo(this.heap[parentIndex]) > 0)
        {
            this.Swap(itemIndex, parentIndex);
            itemIndex = parentIndex;
            parentIndex = this.Parent(parentIndex);
        }
    }

    /// <summary>
    /// A method to remove and retrieve the root of the heap
    /// </summary>
    /// <returns> the biggest element in the heap </returns>
    public T ExtractMax()
    {
        // Get root
        T max = this.heap[0];

        // If heap contains no other elements clear it
        if (this.heap.Count == 1)
            this.heap.Clear();
        else
        {
            // Put last item in the root and delete the last element
            this.heap[0] = this.heap[this.heap.Count - 1];
            this.heap.RemoveAt(this.heap.Count - 1);

            // While we can traverse the heap
            int itemIndex = 0;
            while (this.IsInBounds(itemIndex))
            {
                // Get both children
                int left = this.LeftChild(itemIndex);
                int right = this.RightChild(itemIndex);

                // Compare them to the parent
                int leftCmp = this.IsInBounds(left) ? this.heap[itemIndex].CompareTo(this.heap[left]) : 1;
                int rightCmp = this.IsInBounds(right) ? this.heap[itemIndex].CompareTo(this.heap[right]) : 1;

                // If both are smaller heap is valid
                if (leftCmp > 0 && rightCmp > 0)
                    break;

                // If one of them is relevent
                if (leftCmp == 1 || rightCmp == 1)
                {
                    if (leftCmp < 0)
                    {
                        this.Swap(itemIndex, left);
                        itemIndex = left;
                    }
                    else if (rightCmp < 0)
                    {
                        this.Swap(itemIndex, right);
                        itemIndex = right;
                    }
                }
                // If both are relevent
                else
                {
                    int cmp = this.heap[left].CompareTo(this.heap[right]);

                    if (cmp > 0)
                    {
                        this.Swap(itemIndex, left);
                        itemIndex = left;
                    }
                    else
                    {
                        this.Swap(itemIndex, right);
                        itemIndex = right;
                    }
                }
            }
        }
        return max;
    }

    /// <summary>
    /// A method to get the value of the root of the heap
    /// </summary>
    /// <returns> the value of the biggest element in the heap </returns>
    public T PeekMax()
    {
        return this.heap[0];
    }

    /// <summary>
    /// A method to check if a item is contained in the heap
    /// </summary>
    /// <param name="item"> the item to check </param>
    /// <returns> true if item is preset, false otherwise </returns>
    public bool Contains(T item)
    {
        if (this.heap.Count == 0)
            return false;
        
        return this.Contains(0, item);
    }

    /// <summary>
    /// A method to delete all elements from the heap
    /// </summary>
    public void Clear() => this.heap.Clear();

    public bool IsEmpty() => this.heap.Count == 0;

    #endregion

}
