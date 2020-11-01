using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using UnityEngine;

/// <summary>
/// Manages a single flower with nectar
/// </summary>


public class Flower : MonoBehaviour
{
    [Tooltip("The color when the flower is full")]
    public Color fullFlowerColor = new Color(1f, 0f, .3f);

    [Tooltip("The color when the flower is empty")]
    public Color emptyFlowerColor = new Color(.5f, 0f, 1f);

    /// <summary>
    /// The trigger collider representing the nectar
    /// </summary>
    
    [HideInInspector]
    public Collider nectarCollider;


    // The solid collider representing the flower petals 
    private Collider flowerCollider;

    //The flower's material
    private Material flowerMaterial;

    /// <summary>
    /// A Vector pointing out of the flower
    /// </summary>
  
    public Vector3 FlowerUpVector
    {
        get
        {
            return nectarCollider.transform.up;
        }

    }

    /// <summary>
    /// The center position of the nectar collider
    /// </summary>
    public Vector3 FlowerCenterPosition
    {
        get
        {
            return nectarCollider.transform.position;
        }
    }

    /// <summary>
    /// The amount of nectar remaining in the flower 
    /// </summary>
    public float NectarAmount { get; private set; }

    /// <summary>
    /// Whether the flower has any remaining nectar
    /// </summary>
    public bool HasNectar
    {
        get
        {
            return NectarAmount > 0f;
        }
    }


}
