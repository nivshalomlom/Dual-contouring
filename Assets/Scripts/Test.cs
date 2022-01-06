using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{

    // TODO: 
    // 1. Octree simplification - done
    // 2. Triangulation

    DualContour shape;
    float size = 2;

    // Start is called before the first frame update
    void Start()
    {
        Cube bounds = new Cube(Vector3.one * -size * 2, Vector3.one * size * 2);
        
        DualContour.Density heart = (x, y, z) => {
            float result = 2 * x * x + y * y + z * z - 1;
            return Mathf.Pow(result, 3) - z * z * y * y * y;
        };

        DualContour.Density cube = (x, y, z) => {
            float X = Mathf.Abs(x);
            float Y = Mathf.Abs(y);
            float Z = Mathf.Abs(z);
            return Mathf.Max(X, Y, Z) - size;
        };

        this.shape = new DualContour(bounds, cube);
        this.shape.Generate(0f, 0f, 5);
    }

    void OnDrawGizmos()
    {
        foreach (Vector3 leaf in this.shape.GetVertexes())
        {
            Gizmos.DrawSphere(leaf, 0.01f);
        }
    }

}
