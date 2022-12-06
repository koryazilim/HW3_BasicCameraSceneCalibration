using UnityEngine;

public class HManager : MonoBehaviour {

    public static HManager instance;

    public HImage refImg;
    public HImage currentImg;
    public HImage[] imgs = null;
    public int pointCountForHmg = 5;

    public Transform augmentedObject;

    private void Awake()
    {
        if (instance == null)
            instance = this;
        else if (instance != this)
            Destroy(gameObject);

        imgs = FindObjectsOfType<HImage>();
    }
    
    public void ApplyProjection(HImage img)
    {
        double[,] r = ImgToDoubleArr(refImg);
        double[,] d = ImgToDoubleArr(img);
        double[,] hm = CalculateHomographyScript.FindHomography(r, d);


        // Debug.Log("r length 0 :" + r.GetLength(0));
        // Debug.Log("r length 1 :" + r.GetLength(1));
        // Debug.Log("d length 0 :" + d.GetLength(0));
        // Debug.Log("d length 1 :" + d.GetLength(1));

        /*
        for (int i=0;i<r.GetLength(0);i++){
            for(int j=0; j<r.GetLength(1);j++){
                if(j>10)
                    break;
                //  Debug.Log("r["+i+","+j+"]=" + r[i,j]);
                
            }
            if(i>10)
                break;
        }        
        for (int i=0;i<d.GetLength(0);i++){
            for(int j=0; j<d.GetLength(1);j++){
                if(j>10)
                    break;
                // Debug.Log("d["+i+","+j+"]=" + d[i,j]);
                
            }
            if(i>10)
                break;
        }        
        */

        Vector3 objPoint = augmentedObject.transform.localPosition;
        double[,] p = new double[,] { { objPoint.x }, { objPoint.y }, { 1 } };
        double[,] uv = CalculateHomographyScript.Run1_3(hm, p);

        Transform newOrigin = new GameObject("NewOrigin").transform;
        newOrigin.SetParent(img.origin);

        double[,] o = new double[3, 1] { { 0 }, { 0 }, { 1 } };
        double[,] or = CalculateHomographyScript.Run1_3(hm, o);
        newOrigin.localPosition = new Vector3((float)or[0, 0], (float)or[1, 0], 0);

        Vector3 refDir = refImg.origin.transform.position - objPoint;
        Vector3 projDir = newOrigin.localPosition - (new Vector3((float)uv[0, 0], (float)uv[1, 0], 0));
        // print("refdir : " + refDir + " , projdir : " + projDir);

        float zAngle = Vector2.SignedAngle(refDir, projDir);
        float dist1 = Mathf.Sqrt(Mathf.Pow((float)(p[0, 0]) , 2) + Mathf.Pow((float)(p[1, 0]), 2));
        float dist2 = Mathf.Sqrt(Mathf.Pow((float)(uv[0, 0] - or[0, 0]) , 2) + Mathf.Pow((float)(uv[1, 0] - or[1, 0]), 2));
        float scaleFactor = dist2 / dist1;

        Transform obj = Instantiate(augmentedObject, img.origin);
        obj.localScale *= scaleFactor;
        obj.RotateAround(newOrigin.transform.position, Vector3.forward, zAngle);
        obj.localPosition = new Vector3((float)uv[0,0], (float)uv[1,0], 0);

    }

    public void ApplyProjectionToAll()
    {
        foreach (HImage img in imgs){
            if (img.anchors.Count == pointCountForHmg && img != refImg)
                ApplyProjection(img);
        }

        ResetPlaceModes();
    }
 
    public double[,] ImgToDoubleArr(HImage img)
    {
        double[,] arr = new double[5, 2];
        int i = 0;
        foreach (HAnchor a in img.anchors) {
            arr[i, 0] = a.pos.x;
            arr[i, 1] = a.pos.y;

            i++;
        }
        return arr;
    }

    public void ResetPlaceModes()
    {
        foreach (HImage img in imgs)
            img.placeMode = false;
    }

    public void QuitApplication()
    {
        Application.Quit();
    }
}
