using UnityEngine;
using Microsoft.MixedReality.Toolkit.Utilities;

public class meshController : MonoBehaviour
{
    // Reference to your specimen object
    public GameObject specimen;

    // Material with transparency that will be modified
    private Material materialWithTransparency;

    void Start()
    {
        specimen = GameObject.Find("specimen");
        if (specimen != null)
        {
            Debug.Log(specimen.name);
            Debug.Log(specimen.transform.GetChild(0).name);
            // Get the renderer component from the specimen
            Renderer renderer = specimen.GetComponent<MeshRenderer>();

            // Get the material (note: using sharedMaterial would affect all objects using this material)
            // Using material creates an instance that only affects this object
            materialWithTransparency = renderer.material;

            SetTransparency(0.5f);
        }
    }

    void Update()
    {
        if (specimen == null)
        {
            specimen = GameObject.Find("specimen");
            if (specimen != null)
            {
                Debug.Log(specimen.name);
                Debug.Log(specimen.transform.GetChild(0).name);
                specimen = specimen.transform.GetChild(0).gameObject;
                // Get the renderer component from the specimen
                Renderer renderer = specimen.GetComponent<Renderer>();

                // Get the material (note: using sharedMaterial would affect all objects using this material)
                // Using material creates an instance that only affects this object
                materialWithTransparency = renderer.material;
                SetupTransparentMaterial();
            }
        }
        else
        {
            SetTransparency(0.1f);
        }
    }

    // Call this method to set transparency (0 = fully transparent, 1 = fully opaque)
    public void SetTransparency(float alphaValue)
    {
        // Ensure the alpha value is within valid range
        alphaValue = Mathf.Clamp01(alphaValue);

        // Get current color and modify its alpha
        Color color = materialWithTransparency.color;
        color.a = alphaValue;
        materialWithTransparency.color = color;
    }

    private void SetupTransparentMaterial()
    {
        // Standard shader transparency setup
        materialWithTransparency.SetFloat("_Mode", 2); // 3 = Transparent mode; 2 = Fade mode
        materialWithTransparency.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        materialWithTransparency.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        materialWithTransparency.SetInt("_ZWrite", 0);
        materialWithTransparency.DisableKeyword("_ALPHATEST_ON");
        materialWithTransparency.EnableKeyword("_ALPHABLEND_ON");
        materialWithTransparency.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        materialWithTransparency.renderQueue = 3000;
    }
}