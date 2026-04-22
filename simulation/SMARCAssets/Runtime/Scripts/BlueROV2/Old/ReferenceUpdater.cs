using UnityEngine;

public class ReferenceUpdater : MonoBehaviour
{
    public bool Controller_mode = true;
    

    [Header("Movement Settings")]
    public float movementSpeed = 5.0f; // Speed of movement
    public float verticalSpeed = 3.0f; // Speed of moving up and down (space and shift)

    private Vector3 velocity = Vector3.zero;
    
    public void OnTickChange(bool tick)
    {
        Controller_mode = tick;
    }

    void FixedUpdate()
    {
        if (Controller_mode)
        {
            // Handle movement input
            HandleMovementInput();

            // Move the object based on velocity
            transform.position += velocity * Time.fixedDeltaTime;
        }
    }

    void HandleMovementInput()
    {
        // Reset velocity
        velocity = Vector3.zero;

        // The Input class is deprecated in the new input system
        // If you want to use keyboard controls, use the new Input System package from Unity and set it up accordingly.
        // WASD controls for movement in the X and Z plane
    }
}
