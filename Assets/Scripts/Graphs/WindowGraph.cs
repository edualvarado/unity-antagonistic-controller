/****************************************************
 * File: WindowGraph.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 27/01/2021
   * Project: ** WORKING TITLE **
   * Last update: 17/03/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class WindowGraph : MonoBehaviour
{
    [SerializeField] private Sprite circleSprite;
    //private RectTransform _graphContainer;

    //public RectTransform Rect
    //{
    //    get
    //    {
    //        return _graphContainer;
    //    }
    //}

    private void Awake()
    {
        //_graphContainer = transform.Find("GraphContainer").GetComponent<RectTransform>();
        List<int> valueList = new List<int> {0, 25};
        //ShowGraph(valueList);
    }

    public GameObject CreateCircle(RectTransform graphContainer, Vector2 anchoredPosition, Color color, string name)
    {
        GameObject gameObject = new GameObject(name, typeof(Image));
        gameObject.transform.SetParent(graphContainer, false);
        gameObject.GetComponent<Image>().sprite = circleSprite;
        gameObject.GetComponent<Image>().color = color;
        return gameObject;
    }

    public GameObject CreateLine(RectTransform graphContainer, Vector2 dotPositionA, Vector2 dotPositionB, Color color, string name)
    {
        GameObject gameObject = new GameObject(name, typeof(Image));
        gameObject.transform.SetParent(graphContainer, false);
        gameObject.GetComponent<Image>().color = new Color(color.r, color.g, color.b, 0.5f);
        return gameObject;
    }

    public void MoveCircle(GameObject point, Vector2 anchoredPosition)
    {
        RectTransform rectTransform = point.GetComponent<RectTransform>();
        rectTransform.anchoredPosition = anchoredPosition;
        rectTransform.sizeDelta = new Vector2(11, 11);
        rectTransform.anchorMin = new Vector3(0, 0);
        rectTransform.anchorMax = new Vector3(0, 0);
    }

    public void MoveLine(GameObject line, Vector2 dotPositionA, Vector2 dotPositionB)
    {
        RectTransform rectTransform = line.GetComponent<RectTransform>();
        Vector2 dir = (dotPositionB - dotPositionA).normalized;
        float distance = Vector2.Distance(dotPositionA, dotPositionB);
        rectTransform.anchorMin = new Vector2(0, 0);
        rectTransform.anchorMax = new Vector2(0, 0);
        rectTransform.sizeDelta = new Vector2(distance, 3f);
        rectTransform.anchoredPosition = dotPositionA + dir * distance * 0.5f;
        rectTransform.localEulerAngles = new Vector3(0, 0, (Mathf.Atan2(dir.y, dir.x) * 180 / Mathf.PI));
    }

    public void SetTransparencyLine(GameObject line, Color color, float t)
    {
        line.GetComponent<Image>().color = new Color(color.r, color.g, color.b, t);
    }

    public void SetTransparencyPoint(GameObject point, Color color, float t)
    {
        point.GetComponent<Image>().color = new Color(color.r, color.g, color.b, t);
    }

    public void ShowGraph(RectTransform graphContainer, List<int> valueList)
    {
        float graphHeight = graphContainer.sizeDelta.y;
        float yMaximum = 100f;
        float xSize = 10f;
        //float yMaximum = 1f;
        //float xSize = 1f;

        GameObject lastCircleGameObject = null;
        for (int i = 0; i < valueList.Count; i++)
        {
            float xPosition = i * xSize;
            float yPosition = (valueList[i] / yMaximum) * graphHeight;
            GameObject circleGameObject = CreateCircle(graphContainer, new Vector2(xPosition, yPosition), Color.white, "point");
            if(lastCircleGameObject != null)
            {
                CreateDotConnection(graphContainer, lastCircleGameObject.GetComponent<RectTransform>().anchoredPosition, circleGameObject.GetComponent<RectTransform>().anchoredPosition);
            }
            lastCircleGameObject = circleGameObject;
        }
    }

    public void CreateDotConnection(RectTransform graphContainer, Vector2 dotPositionA, Vector2 dotPositionB)
    {
        GameObject gameObject = new GameObject("dotConnection", typeof(Image));
        gameObject.transform.SetParent(graphContainer, false);
        gameObject.GetComponent<Image>().color = new Color(1, 1, 1, 0.5f);
        RectTransform rectTransform = gameObject.GetComponent<RectTransform>();
        Vector2 dir = (dotPositionB - dotPositionA).normalized;
        float distance = Vector2.Distance(dotPositionA, dotPositionB);
        rectTransform.anchorMin = new Vector2(0, 0);
        rectTransform.anchorMax = new Vector2(0, 0);
        rectTransform.sizeDelta = new Vector2(distance, 3f);
        rectTransform.anchoredPosition = dotPositionA + dir * distance * 0.5f;
        rectTransform.localEulerAngles = new Vector3(0, 0, (Mathf.Atan2(dir.y, dir.x) * 180 / Mathf.PI));
    }
}
