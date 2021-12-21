using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RelaxBar : MonoBehaviour
{
    public Slider slider;
    public Gradient gradient;
    public Image fill;

    public void SetRelax(float relax)
    {
        slider.value = relax;
        fill.color = gradient.Evaluate(slider.normalizedValue);
    }
    public void SetMaxRelax(float maxRelax)
    {
        slider.maxValue = maxRelax;

        fill.color = gradient.Evaluate(slider.normalizedValue);

    }
}
