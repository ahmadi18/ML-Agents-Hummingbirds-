using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Manages a collection of flower plants and attached flowers
/// </summary>
public class FlowerArea : MonoBehaviour
{
    //The diameter of the area where the agent and flower can be used for observing relative distance from agent to flower
    public const float AreaDiameter = 20f;

    //The list of all flower plants in this flower area (flower plants have multiple flowers)
    private List<GameObject> flowerplants;

    // A lookup dictionary for looking up a flower from a nectar collider
    private Dictionary<Collider, Flower> nectarFlowerDictionary;

    /// <summary>
    /// The list of all flowers in the flower area
    /// </summary>
    public List<Flower> Flowers { get; private set; }
   
}
