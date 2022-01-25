using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{

    // TODO: 
    // 1. Add mesh splitting for massive shapes
    // 2. Mutlithreading / Multiprocessing

    public Material mat;

    DualContour shape;
    float size = 2;

    // Start is called before the first frame update
    void Start()
    {
        Cubiod bounds = new Cubiod(Vector3.one * -size, Vector3.one * size);

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

        DualContour.Density sphere = (x, y, z) => {
            return Vector3.Distance(new Vector3(x, y, z), Vector3.zero) - size / 2f;
        };

        this.shape = new DualContour(bounds, sphere);
        Mesh m = this.shape.Generate(0f, 0f, 6);

        this.gameObject.AddComponent<MeshFilter>().mesh = m;
        this.gameObject.AddComponent<MeshRenderer>().material = this.mat;
    }

}
