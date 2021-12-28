using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class MathOperations 
{
    public static bool PointInTriangles(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p)
    {
        // Lets define some local variables, we can change these
        // without affecting the references passed in

        Vector3 a = p1;
        Vector3 b = p2;
        Vector3 c = p3;

        // Move the triangle so that the point becomes the 
        // triangles origin
        a -= p;
        b -= p;
        c -= p;

        // The point should be moved too, so they are both
        // relative, but because we don't use p in the
        // equation anymore, we don't need it!
        // p -= p;

        // Compute the normal vectors for triangles:
        // u = normal of PBC
        // v = normal of PCA
        // w = normal of PAB

        Vector3 u = Vector3.Cross(b, c);
        Vector3 v = Vector3.Cross(c, a);
        Vector3 w = Vector3.Cross(a, b);

        // Test to see if the normals are facing 
        // the same direction, return false if not
        if (Vector3.Dot(u, v) < 0f)
        {
            return false;
        }
        if (Vector3.Dot(u, w) < 0.0f)
        {
            return false;
        }

        // All normals facing the same way, return true
        return true;
    }
}
