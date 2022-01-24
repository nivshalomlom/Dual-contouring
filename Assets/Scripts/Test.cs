using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{

    // TODO: 
    // 1. Add mesh splitting for massive shapes
    // 2. Mutlithreading / Multiprocessing
    // FIXME: 
    // 1. When simplifying unnecessary quads are rendered - fixed!
    // 2. Some quad are missing from complicated shapes in high LOD's

    public Material mat;

    DualContour shape;
    float size = 2;

    // Start is called before the first frame update
    void Start()
    {
        Cubiod bounds = new Cubiod(Vector3.one * -size * 2, Vector3.one * size * 2);

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

        DualContour.Density noise = (x, y, z) => {
            return y - Mathf.PerlinNoise(x, z);
        };

        this.shape = new DualContour(bounds, noise);
        Mesh m = this.shape.Generate(0f, 1e-5f, 6);

        this.gameObject.AddComponent<MeshFilter>().mesh = m;
        this.gameObject.AddComponent<MeshRenderer>().material = this.mat;
    }

}
