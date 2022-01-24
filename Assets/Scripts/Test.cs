using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{

    // TODO: 
    // Figure out why some quads are skipped

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

        this.shape = new DualContour(bounds, heart);
        Mesh m = this.shape.Generate(0f, 6e-6f, 6);

        this.gameObject.AddComponent<MeshFilter>().mesh = m;
        this.gameObject.AddComponent<MeshRenderer>().material = this.mat;
    }

}
