using UnityEngine;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.UI;

public class meshController : MonoBehaviour
{
    // Reference to your specimen object
    public GameObject specimen;
    public PinchSlider transparencySlider;
    public GameObject meshMenu;
    float alphaValue;

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
        }
    }
    GameObject GetChildThatIsNotTarget(GameObject parentObject)
    {
        // Make sure parent exists and has children
        if (parentObject == null || parentObject.transform.childCount == 0)
        {
            return null;
        }

        // Go through children (max 2 as per your description)
        for (int i = 0; i < parentObject.transform.childCount; i++)
        {
            Transform child = parentObject.transform.GetChild(i);

            // If this child is not named "target", return it
            if (child.name.ToLower() != "target" && child.name.ToLower() != "sphere")
            {
                return child.gameObject;
            }
        }

        // If all children are named "target" or there are no children
        return null;
    }

    void Update()
    {
        if (specimen == null)
        {
            specimen = GameObject.Find("specimen");
            if (specimen != null)
            {
                Debug.Log(specimen.name);
                specimen = GetChildThatIsNotTarget(specimen);
                Debug.Log(specimen.name);

                // Get the renderer component from the specimen
                Renderer renderer = specimen.GetComponent<Renderer>();

                // Get the material (note: using sharedMaterial would affect all objects using this material)
                // Using material creates an instance that only affects this object
                materialWithTransparency = renderer.material;
                SetupTransparentMaterial();
                //SetTransparency(0.5f);
                if (transparencySlider != null)
                {
                    transparencySlider.SliderValue = 0.5f;
                }
            }
        }
        else
        {
            // SetTransparency(0.1f);
        }
    }

    public void SetTransparencyBySlider()
    {
        if (specimen != null && transparencySlider != null)
        {
            SetTransparency(transparencySlider.SliderValue);
        }
    }

    public void IncTransparency(float dAlpha=-0.2f)
    {
        // Lower Alpha = Higher Transparency
        UpdateTransparency(dAlpha);
    }

    public void DecTransparency(float dAlpha=0.2f)
    {
        // Higher Alpha = Lower Transparency
        UpdateTransparency(dAlpha);
    }

    public void UpdateTransparency(float dAlpha)
    {
        if (specimen == null)
        {
            return;
        }

        // Get current color and modify its alpha
        Color color = materialWithTransparency.color;
        alphaValue = Mathf.Clamp01(color.a + dAlpha);
        color.a = alphaValue;
        materialWithTransparency.color = color;
        if (transparencySlider != null)
        {
            transparencySlider.SliderValue = alphaValue; // TODO: this would involk SetTransparency, which is not ideal.
        }
    }

    // Call this method to set transparency (0 = fully transparent, 1 = fully opaque)
    public void SetTransparency(float newAlphaValue)
    {
        if (specimen == null)
        {
            return;
        }
        // Ensure the alpha value is within valid range
        alphaValue = Mathf.Clamp01(newAlphaValue);

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
    //public void hideMenu()
    //{
    //    if (meshMenu  != null)
    //    {
    //        meshMenu.enabled = false;
    //    }
    //}

    //public void showMenu()
    //{
    //    myText.enabled = true;
    //}
}