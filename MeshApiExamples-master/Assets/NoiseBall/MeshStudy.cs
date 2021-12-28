using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using UnityEngine;
using UnityEngine.Rendering;
public class MeshStudy : MonoBehaviour
{
    Mesh originalMesh;
    Mesh clonedMesh;
    MeshFilter meshFilter;
    public int[] triangles;
    public Transform scalper;
    public Transform empty;
    public Vector3[] vertices;

    [HideInInspector]
    public bool isCloned = false;


    // For Editor
    public float radius = 0.2f;
    public float pull = 0.3f;
    public float handleSize = 0.03f;
    public List<int>[] connectedVertices;
    public List<Vector3[]> allTriangleList;
    public bool moveVertexPoint = true;
    public Transform pos;
    public Transform plane;
    List<Vector3> Verts = new List<Vector3>();
    List<int> Tris = new List<int>();
    public List<Vector2> UVs = new List<Vector2>();

    Plane plane1;
    void Start()
    {


        InitMesh();
        foreach (Vector3 V3 in vertices)
            Verts.Add(V3);

        foreach (int tri in triangles)
            Tris.Add(tri);

        foreach (Vector2 uv in clonedMesh.uv)
            UVs.Add(uv);


    }

    public void InitMesh()
    {
        UVs.Clear();
        meshFilter = GetComponent<MeshFilter>();
        originalMesh = meshFilter.sharedMesh; //1
        clonedMesh = new Mesh(); //2

        clonedMesh.name = "clone";
        clonedMesh.vertices = originalMesh.vertices;
        clonedMesh.triangles = originalMesh.triangles;
        clonedMesh.normals = originalMesh.normals;
        clonedMesh.uv = originalMesh.uv;
        meshFilter.mesh = clonedMesh;  //3

        vertices = clonedMesh.vertices; //4
        triangles = clonedMesh.triangles;
        isCloned = true; //5
        Debug.Log("Init & Cloned");
    }

    public void Reset()
    {
        if (clonedMesh != null && originalMesh != null) //1
        {

            clonedMesh.vertices = originalMesh.vertices; //2
            clonedMesh.triangles = originalMesh.triangles;
            clonedMesh.normals = originalMesh.normals;
            clonedMesh.uv = originalMesh.uv;
            meshFilter.mesh = clonedMesh; //3

            vertices = clonedMesh.vertices; //4
            triangles = clonedMesh.triangles;
            Verts.Clear();
            Tris.Clear();
            UVs.Clear();
            lastVertex = Vector3.zero;
            newVertex = Vector3.zero;
            lastTris.Clear();

            foreach (Vector3 V3 in vertices)
                Verts.Add(V3);

            foreach (int tri in triangles)
                Tris.Add(tri);

            foreach (Vector2 uv in clonedMesh.uv)
                UVs.Add(uv);
        }
    }

    public void GetConnectedVertices()
    {
        connectedVertices = new List<int>[vertices.Length];
    }

    public void DoAction(int index, Vector3 localPos)
    {
        PullOneVertex(index, localPos);
    }

    // returns List of int that is related to the targetPt.
    private List<int> FindRelatedVertices(Vector3 targetPt, bool findConnected)
    {
        // list of int
        List<int> relatedVertices = new List<int>();

        int idx = 0;
        Vector3 pos;

        // loop through triangle array of indices
        for (int t = 0; t < triangles.Length; t++)
        {
            // current idx return from tris
            idx = triangles[t];
            // current pos of the vertex
            pos = vertices[idx];
            // if current pos is same as targetPt
            if (pos == targetPt)
            {
                // add to list
                relatedVertices.Add(idx);
                // if find connected vertices
                if (findConnected)
                {
                    // min
                    // - prevent running out of count
                    if (t == 0)
                    {
                        relatedVertices.Add(triangles[t + 1]);
                    }
                    // max 
                    // - prevent runnign out of count
                    if (t == triangles.Length - 1)
                    {
                        relatedVertices.Add(triangles[t - 1]);
                    }
                    // between 1 ~ max-1 
                    // - add idx from triangles before t and after t 
                    if (t > 0 && t < triangles.Length - 1)
                    {
                        relatedVertices.Add(triangles[t - 1]);
                        relatedVertices.Add(triangles[t + 1]);
                    }
                }
            }
        }
        // return compiled list of int
        return relatedVertices;
    }
 
    public void ShowTriangle(int idx)
    {

    }
    private bool PointInTriangle(Vector3 TriangleVectors1, Vector3 TriangleVectors2, Vector3 TriangleVectors3, Vector3 P)
    {
        Vector3 A = TriangleVectors1, B = TriangleVectors2, C = TriangleVectors3;
        if (SameSide(P, A, B, C) && SameSide(P, B, A, C) && SameSide(P, C, A, B))
        {
            Vector3 vc1 = Vector3.Cross(A - B, A - C);
            if (Math.Abs(Vector3.Dot(A - P, vc1)) <= .01f)
                return true;
        }

        return false;
    }

    private bool SameSide(Vector3 p1, Vector3 p2, Vector3 A, Vector3 B)
    {
        Vector3 cp1 = Vector3.Cross(B - A, p1 - A);
        Vector3 cp2 = Vector3.Cross(B - A, p2 - A);
        if (Vector3.Dot(cp1, cp2) >= 0) return true;
        return false;

    }
    bool PointInTriangles(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p)
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
    Vector3 lastVertex, newVertex;
    public List<Vector3> lastTris = new List<Vector3>();
    public List<int> previousTris = new List<int>();
    bool isDone;
    bool firstVert = true;
    int generatedTrisNumber;
    public GameObject go;
    float Xp, Yp, Xq, Yq;
    int lastVertIndex;
    [BurstCompile]
    struct NoiseMeshJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction] public NativeArray<int> triangles;
        [NativeDisableParallelForRestriction] public NativeArray<Vector3> Verts;
        public Vector3 newVertex;
        public void Execute(int id)
        {
            int idx1 = id * 3;
            int idx2 = id * 3 + 1;
            int idx3 = id * 3 + 2;

            if (MathOperations.PointInTriangles(Verts[triangles[idx1]], Verts[triangles[idx2]], Verts[triangles[idx3]], newVertex))
            {
                Debug.Log($"{idx1}");
            }
        }

    }
    void OnCollisionExit(Collision collision)
    {
        firstVert = true;
        previousTris.Clear();
    }
    void OnCollisionStay(Collision collision)
    {


        if (collision.gameObject.tag == "cutter")
        {

            ContactPoint contact = collision.contacts[0];
            if (contact.point != null)
            {
                isDone = false;
                empty.transform.position = contact.point;
                newVertex = plane.InverseTransformPoint(empty.position);

                //Creating Native array of triangles for job 
                /* NativeArray<int> lookFortriangles = new NativeArray<int>(Tris.Count, Allocator.Persistent);
                 for (int i = 0; i < Tris.Count; i++)
                     lookFortriangles[i] = Tris[i];

                 //Creating Native array of Verts for job 
                 NativeArray<Vector3> lookForVertices = new NativeArray<Vector3>(Verts.Count, Allocator.Persistent);
                 for (int i = 0; i < Verts.Count; i++)
                     lookForVertices[i] = Verts[i];

                 var job = new NoiseMeshJob
                 {
                     triangles = lookFortriangles,
                     Verts = lookForVertices,
                     newVertex = newVertex
                 };

                 job.Schedule(lookFortriangles.Length / 3, 4).Complete();
                 lookFortriangles.Dispose();
                 lookForVertices.Dispose();*/

                    for (int i = 0; i < lastTris.Count; i += 3)
                    {

                        if (PointInTriangles(lastTris[i], lastTris[i+1], lastTris[i+2], newVertex))
                        {
                            isDone = true;
                            break;
                        }
                    }

                    if (isDone == false)
                    {

                        Verts.Add(newVertex);
                        Verts.Add(newVertex);
                        vertices = Verts.ToArray();
                        clonedMesh.vertices = vertices;

                        for (int i = 0; i < triangles.Length; i += 3)
                        {

                            if (PointInTriangles(vertices[triangles[i]], vertices[triangles[i + 1]], vertices[triangles[i + 2]], newVertex))
                            {
                                Debug.Log(i);
                                List<int> list = new List<int>();
                                lastTris.Add(vertices[triangles[i]]);
                                lastTris.Add(vertices[triangles[i + 1]]);
                                lastTris.Add(vertices[triangles[i + 2]]);
                                previousTris.Add(triangles[i]);
                                previousTris.Add(triangles[i + 1]);
                                previousTris.Add(triangles[i + 2]);
                                // Coeff Calculation 
                                float distXYZ = Vector3.Distance(vertices[triangles[i]], vertices[triangles[i + 1]]);
                                float distUV = Vector3.Distance(UVs[triangles[i]], UVs[triangles[i + 1]]);
                                float coef = distUV / distXYZ;

                                // Distances Calculation 
                                float distancep_i1 = coef * Vector3.Distance(newVertex, vertices[triangles[i + 1]]);
                                float distancei2_i1 = coef * Vector3.Distance(vertices[triangles[i + 2]], vertices[triangles[i + 1]]);
                                float distancep_i2 = coef * Vector3.Distance(newVertex, vertices[triangles[i + 2]]);
                                float distancei_i2 = coef * Vector3.Distance(vertices[triangles[i]], vertices[triangles[i + 2]]);

                                // Projection Calculation 
                                float angle = Mathf.Acos((Mathf.Pow(distancep_i1, 2) - Mathf.Pow(distancei2_i1, 2) - Mathf.Pow(distancep_i2, 2)) / (-2 * distancei2_i1 * distancep_i2));
                                float distancep1_i2 = distancep_i2 * Mathf.Sin(angle);
                                float distancep2_i2 = distancep_i2 * Mathf.Cos(angle);

                                // Coordinates Calculation
                                Vector2 P2 = new Vector2((distancep2_i2 * UVs[triangles[i]].x + (distancei_i2 - distancep2_i2) * UVs[triangles[i + 2]].x) / distancei_i2, (distancep2_i2 * UVs[triangles[i]].y + (distancei_i2 - distancep2_i2) * UVs[triangles[i + 2]].y) / distancei_i2);
                                Vector2 P1 = new Vector2((distancep1_i2 * UVs[triangles[i + 1]].x + (distancei2_i1 - distancep1_i2) * UVs[triangles[i + 2]].x) / distancei2_i1, (distancep1_i2 * UVs[triangles[i + 1]].y + (distancei2_i1 - distancep1_i2) * UVs[triangles[i + 2]].y) / distancei2_i1);

                                Vector2 P = new Vector2(P2.x + P1.x - UVs[triangles[i + 2]].x, P2.y + P1.y - UVs[triangles[i + 2]].y);
                                UVs.Add(P);
                                UVs.Add(P);

                                if (firstVert)
                                {
                                Debug.Log("k");
                                    Tris.Add(vertices.Length - 1);
                                    Tris.Add(triangles[i]);
                                    Tris.Add(triangles[i + 1]);
                                    Tris.Add(vertices.Length - 1);
                                    Tris.Add(triangles[i + 1]);
                                    Tris.Add(triangles[i + 2]);
                                    Tris.Add(vertices.Length - 1);
                                    Tris.Add(triangles[i + 2]);
                                    Tris.Add(triangles[i]);
                                    firstVert = !firstVert;
                                    generatedTrisNumber = 3;
                                }
                                else
                                {
                                    IEnumerable<int> duplicates = previousTris.GroupBy(x => x)
                                                   .Where(g => g.Count() > 1)
                                                   .Select(x => x.Key);


                                    for (int j = (triangles.Length - 1) - (generatedTrisNumber * 3) + 1; j < triangles.Length; j += 3)
                                    {
                                        int k = 0;
                                        foreach (int v in duplicates)
                                        {

                                            if (v == triangles[j] || v == triangles[j + 1] || v == triangles[j + 2])
                                                k++;
                                            if (k >= 2)
                                            {

                                                Tris.Add(vertices.Length - 2);
                                                Tris.Add(vertices.Length - 4);
                                                Tris.Add(triangles[j + 1]);


                                                Tris.Add(vertices.Length - 1);
                                                Tris.Add(triangles[j + 2]);
                                                Tris.Add(vertices.Length - 3);
                                                list.Add(j + 1);
                                                list.Add(j + 2);
                                                Tris.RemoveAt(j);
                                                Tris.RemoveAt(j);
                                                Tris.RemoveAt(j);

                                                break;


                                            }
                                        }
                                    }


                                    for (int l = i; l < i + 3; l++)
                                    {
                                        if (!duplicates.Contains(triangles[l]))
                                        {
                                            list.Add(l);
                                        }
                                    }
                                    Tris.Add(vertices.Length - 2);
                                    Tris.Add(triangles[list[0]]);
                                    Tris.Add(triangles[list[2]]);

                                    Tris.Add(vertices.Length - 1);
                                    Tris.Add(triangles[list[2]]);
                                    Tris.Add(triangles[list[1]]);

                                    previousTris.RemoveAt(0);
                                    previousTris.RemoveAt(0);
                                    previousTris.RemoveAt(0);

                                    generatedTrisNumber = 4;
                                }
                                Tris.RemoveAt(i);
                                Tris.RemoveAt(i);
                                Tris.RemoveAt(i);
                                break;
                            }


                        }

                        triangles = Tris.ToArray();
                        //clonedMesh.uv = UVs.ToArray();
                        clonedMesh.uv = UVs.ToArray();
                        clonedMesh.triangles = triangles;
                        clonedMesh.RecalculateNormals();
                        lastVertIndex = Verts.Count - 1;
                    }
                }
        }
    }

    // Pulling only one vertex pt, results in broken mesh.
    private void PullOneVertex(int index, Vector3 newPos)
    {
        vertices[index] = newPos; //1
        clonedMesh.vertices = vertices;

        clonedMesh.RecalculateNormals(); //3
    }

    private void PullSimilarVertices(int index, Vector3 newPos)
    {
        Vector3 targetVertexPos = vertices[index]; //1
        List<int> relatedVertices = FindRelatedVertices(targetVertexPos, false); //2
        foreach (int i in relatedVertices) //3
        {
            vertices[i] = newPos;
        }
        clonedMesh.vertices = vertices; //4
        clonedMesh.triangles = triangles;
        clonedMesh.RecalculateNormals();

    }

    // To test Reset function
    public void EditMesh()
    {
        vertices[2] = new Vector3(2, 3, 4);
        vertices[3] = new Vector3(1, 2, 4);
        clonedMesh.vertices = vertices;
        clonedMesh.RecalculateNormals();
    }
}
