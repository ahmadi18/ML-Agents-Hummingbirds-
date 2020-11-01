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
    /// <summary>
    /// Attempts to remove nectar from the flower
    /// </summary>
    /// <param name="amount">The amount of nectar to remove</param>
    /// <returns>The actual amount successfully removed</returns>
    public float Feed(float amount)
    {
        //Track how much nectar was successfully taken (can not be more than availible)
        float nectarTaken = Mathf.Clamp(amount, 0f, NectarAmount);

        //Subtract the nectar
        NectarAmount -= amount;
        if (NectarAmount <= 0)
        {
            //No nectar remaining
            NectarAmount = 0;

            //Disable the flower and nectar colliders

            flowerCollider.gameObject.SetActive(false);
            nectarCollider.gameObject.SetActive(false);

            //Change the flower color to indicate that it is empty
            flowerMaterial.SetColor("_BaseColor", emptyFlowerColor);
        }

        //Return the amount of nectar that was taken
        return nectarTaken;
    }
}

