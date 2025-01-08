using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using Dummiesman;
using UnityEngine.UI;


public class ModelFileLoader : MonoBehaviour
{
    public Text myText;
    [SerializeField] private GameObject modelContainerObj;
    private string modelFileFolderPath = "/";
    [SerializeField] private string modelFileName = "specimen.obj";
    [SerializeField] private string materialFileName = "specimen.mtl";
    private bool isFileLoaded = false; // Prevent concurrent loading calls

    // Start is called before the first frame update
    void Start()
    {
        modelFileFolderPath = Application.persistentDataPath; // Path.Combine(System.Environment.GetFolderPath(System.Environment.SpecialFolder.MyDocuments)); // Path.GetDirectoryName(modelFileFolderPath);
    }

    // Update is called once per frame
    void Update()
    {
        if (!isFileLoaded)
        {
            CheckAndLoadFile();
            myText.text += "\n Finding File from " + modelFileFolderPath;
        }
        else
        {
            myText.text += "\n File Loaded! from " + modelFileFolderPath;
        }
    }
    private void CheckAndLoadFile()
    {
        string filePath = Path.Combine(modelFileFolderPath, modelFileName);
        string mtlPath = Path.Combine(modelFileFolderPath, materialFileName);

        Debug.Log(filePath);
        if (File.Exists(filePath) && File.Exists(mtlPath))
        {
            Debug.Log($"File found: {filePath}");
            // Call your function to load the GLTF file
            loadObjFile(filePath, mtlPath, modelContainerObj);
            isFileLoaded = true;
        }
        else
        {
            Debug.Log("File not found.");
        }
    }

    private void loadObjFile(string filePath, string mtlPath, GameObject parentObj)
    {
        // Your OBJ loading logic goes here
        var loadedObj = new OBJLoader().Load(filePath, mtlPath);
        loadedObj.transform.SetParent(parentObj.transform);
        loadedObj.transform.localEulerAngles = Vector3.zero;
        loadedObj.transform.localPosition = Vector3.zero;
        Debug.Log($"OBJ file loaded from: {filePath}");
    }
}
