using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.LinearAlgebra.Double;

public class CalculateHomographyScript : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
        double[,] sc = new double[4, 2] { { 1, 2 }, { 5, 3 }, { 4, 10 }, { 1, 6 } };
        double[,] im = new double[4, 2] { { 1, 2 }, { 5, 3 }, { 4, 10 }, { 1, 6 } };

        double[,] homography = FindHomography(sc, im);

        Run1_1(homography);


        double[,] xy = new double[3, 1] { { sc[2,0] }, { sc[2,1] } , { 1 } };
        Debug.Log("1.3 - Projection From Scene Point =>");
        Run1_3(homography, xy);


        double[,] uv = new double[3, 1] { { im[1, 0] }, { im[1, 1] }, { 1 } };
        Debug.Log("1.4 - Projection From Image Point =>");
        Run1_4(homography, uv);

        Run1_567();

    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public static double[,] FindHomography(double[,] m, double[,] n){

        int lenX = 9;
        int lenY = m.GetLength(0);
        
        double[,] matrx = new double[2 * lenY, lenX];

        int p = 0;
        for (int i = 0; i < 2 * lenY; i++){
            if (i % 2 == 0){
                matrx[i, 0] = -1 * m[p, 0];
                matrx[i, 1] = -1 * m[p, 1];
                matrx[i, 2] = -1;
                matrx[i, 3] = 0;
                matrx[i, 4] = 0;
                matrx[i, 5] = 0;
                matrx[i, 6] = n[p, 0] * n[p, 0];
                matrx[i, 7] = n[p, 0] * n[p, 1];
                matrx[i, 8] = n[p, 0];
                }
            else{
                matrx[i, 0] = 0;
                matrx[i, 1] = 0;
                matrx[i, 2] = 0;
                matrx[i, 3] = -1 * m[p, 0];
                matrx[i, 4] = -1 * m[p, 1];
                matrx[i, 5] = -1;
                matrx[i, 6] = m[p, 0] * n[p, 1];
                matrx[i, 7] = n[p, 1] * m[p, 1];
                matrx[i, 8] = n[p, 1];
                ++p;
                }
        }

        var densMtr = DenseMatrix.OfArray(matrx);
        var svdMtr = densMtr.Svd(true);
        
        var svdArray = svdMtr.VT.Row(svdMtr.VT.RowCount - 1).ToArray();

        double[,] result = new double[3, 3];

        int row = 0;
        for (int i = 0; i < svdArray.Length; i++){
            if (i % 3 == 0 && i != 0)
                ++row;
            result[row, i % 3] = svdArray[i];
        }


        return result;
    }

    private void Run1_1(double[,] homography ) {


        string printedMx = "";

        for (int i = 0; i < homography.GetLength(0); i++)
        {
            for (int j = 0; j < homography.GetLength(1); j++)
                printedMx += homography[i, j] + " ";
            printedMx += "\n";
        }

        Debug.Log("1.1 - Homography Matrix : \n\n" + printedMx);
        
        }


    public static double[,] Run1_3(double[,] homMatx, double[,] xy)
    {

        double[,] multipliedMx = MatrixMultiply(homMatx, xy);
        double[,] result = new double[,] { { multipliedMx[0, 0] / multipliedMx[2, 0] }, { multipliedMx[1, 0] / multipliedMx[2, 0] }, { 1 } };

        Debug.Log("(x,y) : " + xy[0, 0] + " , " + xy[1, 0] + " , " + xy[2, 0]);
        Debug.Log("(u,v) : " + result[0, 0] + " , " + result[1, 0] + " , " + result[2, 0]);

        return result;
    }
    
    public static double[,] Run1_4(double[,] homMatx, double[,] uv)
    {

        var inverseHomMatx= DenseMatrix.OfArray(homMatx).Inverse().ToArray();

        double[,] multipliedMx = MatrixMultiply(inverseHomMatx, uv);
        double[,] result = new double[,] { { multipliedMx[0, 0] / multipliedMx[2, 0] }, { multipliedMx[1, 0] / multipliedMx[2, 0] }, { 1 } };
        Debug.Log("(u,v) : " + uv[0, 0] + " , " + uv[1, 0] + " , " + uv[2, 0]);
        Debug.Log("(x,y) : " + result[0, 0] + " , " + result[1, 0] + " , " + result[2, 0]);

        return result;
    }

    public void Run1_567(){

        double[,] scn = new double[5, 2] { { 200, 300 }, { 400, 500 }, { 300, 500 }, { 400, 700 }, { 100, 300 } };

        List<double[,]> imgs = new List<double[,]>();
        List<double[,]> homMatxes = new List<double[,]>();
        
        List<double[,]> points = new List<double[,]>();
        List<double[,]> actualPoints = new List<double[,]>();

        imgs.Add(new double[5,2] {{ 422, 532 }, { 876, 674 }, { 988, 728 }, { 678, 889 } , { 832, 945 } });
        imgs.Add(new double[5,2] {{ 648, 742 }, { 889, 454 }, { 892, 826 }, { 668, 922 } , { 794, 829 } });
        imgs.Add(new double[5,2] {{ 448, 582 }, { 666, 892 }, { 566, 884 }, { 288, 424 } , { 998, 664 } });

        for(int i=0;i<3;i++){
            homMatxes.Add(FindHomography(scn, imgs[i]));
        }
        
        points.Add(new double[3, 1] { { 500 }, { 500 }, { 1 } });
        points.Add(new double[3, 1] { { 300 }, { 700 }, { 1 } });
        points.Add(new double[3, 1] { { 400 }, { 800 }, { 1 } });

        actualPoints.Add(new double[3, 1] { { 700 }, { 300 }, { 1 } });
        actualPoints.Add(new double[3, 1] { { 100 }, { 1200 }, { 1 } });
        actualPoints.Add(new double[3, 1] { { 200 }, { 1100 }, { 1 } });

        Debug.Log("1.5 - Calculate Images =>");
        Debug.Log("First Img =>");
        PrintError(homMatxes[0], points, actualPoints);

        actualPoints.Clear();

        actualPoints.Add(new double[3, 1] { { 600 }, { 200 }, { 1 } });
        actualPoints.Add(new double[3, 1] { { 250 }, { 1100 }, { 1 } });
        actualPoints.Add(new double[3, 1] { { 300 }, { 1100 }, { 1 } });

        Debug.Log("Second Img =>");
        PrintError(homMatxes[1], points, actualPoints);

        actualPoints.Clear();

        actualPoints.Add(new double[3, 1] { { 50 }, { 1000 }, { 1 } });
        actualPoints.Add(new double[3, 1] { { 550 }, { 350 }, { 1 } });
        actualPoints.Add(new double[3, 1] { { 600 }, { 200 }, { 1 } });

        Debug.Log("Third Img =>");
        PrintError(homMatxes[2], points, actualPoints);


        Debug.Log("1.6 - Projecting Images From Scene Points =>");        
        List<double[,]> scenePoints = new List<double[,]>();
        
        scenePoints.Add(new double[3,1] {{ 7.5f }, { 5.5f }, { 1 }});
        scenePoints.Add(new double[3,1] {{ 6.3f }, { 3.3f }, { 1 }});
        scenePoints.Add(new double[3,1] {{ 0.1f }, { 0.1f }, { 1 }});

        for(int i= 0;i<3;i++){
            for(int j=0; j<3; j++){
                Run1_3(homMatxes[i], scenePoints[j]);
            }
        }


        Debug.Log("1.7 - Projecting Images From Image Points =>");
        List<double[,]> imagePoints = new List<double[,]>();
        
        imagePoints.Add(new double[3,1] {{ 500 }, { 400 }, { 1 }}); 
        imagePoints.Add(new double[3,1] {{ 86 }, { 167 }, { 1 }});
        imagePoints.Add(new double[3,1] {{ 10 }, { 10 }, { 1 }});

        for(int i= 0;i<3;i++){
            for(int j=0; j<3; j++){
                Run1_4(homMatxes[i], imagePoints[j]);
            }
        }

    }

    public static double[,] MatrixMultiply(double[,] a, double[,] b)
    {
        int m = a.GetLength(0);
        int n = b.GetLength(1);

        double[,] res = new double[m, n];

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                res[i, j] = 0;
                for (int k = 0; k < a.GetLength(1); k++)
                {
                    res[i, j] += a[i, k] * b[k, j];
                }
            }
        }
        return res;
    }

    private void PrintError(double[,] homMatx, List<double[,]> points, List<double[,]> actualPoints)
    {
        string printResult = "";
        int ind = 0;

        for (int i = 0; i < homMatx.GetLength(0); i++)
        {
            for (int j = 0; j < homMatx.GetLength(1); j++)
                printResult += homMatx[i, j] + " ";
            printResult += "\n";
        }

        Debug.Log("HomMatrix was calculated => \n\n" + printResult);

        foreach (double[,] xy in points)
        {
            double[,] uv = actualPoints[ind++];
            var res = Run1_3(homMatx, xy);
            var error = CalculateErr(res, uv);
            Debug.Log("Error => %" + error);
        }
    }

    public float CalculateErr(double[,] result, double[,] actual)
    {
        var x1 = result[0, 0];
        var y1 = result[1, 0];
        var x2 = actual[0, 0];      
        var y2 = actual[1, 0];
 

        float resDif = Mathf.Sqrt(Mathf.Pow((float)(x1), 2) + Mathf.Pow((float)(y1), 2));
        float actualDif = Mathf.Sqrt(Mathf.Pow((float)(x2), 2) + Mathf.Pow((float)(y2), 2));

        return Mathf.Abs((actualDif - resDif) / actualDif * 100);
    }

}
