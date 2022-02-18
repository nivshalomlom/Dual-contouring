using UnityEngine;

using DualContouring;
using LinerAlgebra;

public class Test : MonoBehaviour
{

    // TODO: 
    // 1. Add mesh splitting for massive shapes - done
    // 2. Mutlithreading / Multiprocessing

    public Material color;

    // Start is called before the first frame update
    void Start()
    {
        Density heart = (x, y, z) => {
            float result = 2 * x * x + y * y + z * z - 1;
            return Mathf.Pow(result, 3) - z * z * y * y * y;
        };

        float cubeSize = 2f;
        Density cube = (x, y, z) => {
            float X = Mathf.Abs(x);
            float Y = Mathf.Abs(y);
            float Z = Mathf.Abs(z);
            return Mathf.Max(X, Y, Z) - cubeSize;
        };

        float sphereRadius = 8f;
        Density sphere = (x, y, z) => {
            return Vector3.Distance(new Vector3(x, y, z), Vector3.zero) - sphereRadius / 2f;
        };

        Cuboid bounds = new Cuboid(Vector3.one * -2f, Vector3.one * 2f);
        DualContour shape = new DualContour(bounds, heart);
        //ChunkedDualContour shape = new ChunkedDualContour(bounds, sphere, 2, 2, 2);

        foreach (Mesh mesh in shape.Generate(0f, 0f, 6))
        {
            GameObject child = new GameObject("Child mesh");
            child.AddComponent<MeshRenderer>().material = color;
            child.AddComponent<MeshFilter>().mesh = mesh;
            child.transform.parent = this.transform;
        }
    }

}
