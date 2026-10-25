// OrbitCamera.cs -- right-drag to orbit, middle-drag to pan, scroll to zoom.
// Added automatically to the Main Camera by TwinPlayback.
using UnityEngine;

public class OrbitCamera : MonoBehaviour
{
    public Vector3 target = new Vector3(0f, 0.25f, 0.1f);
    public float orbitSpeed = 4f, zoomSpeed = 0.3f, panSpeed = 0.002f;
    float dist, yaw, pitch;

    void Start()
    {
        Vector3 d = transform.position - target;
        dist = d.magnitude;
        yaw = Mathf.Atan2(d.x, d.z) * Mathf.Rad2Deg;
        pitch = Mathf.Asin(d.y / dist) * Mathf.Rad2Deg;
    }

    void LateUpdate()
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        if (Input.GetMouseButton(1))
        {
            yaw += Input.GetAxis("Mouse X") * orbitSpeed;
            pitch = Mathf.Clamp(pitch - Input.GetAxis("Mouse Y") * orbitSpeed, -10f, 85f);
        }
        if (Input.GetMouseButton(2))
        {
            target -= transform.right * Input.GetAxis("Mouse X") * panSpeed * dist * 20f;
            target -= transform.up * Input.GetAxis("Mouse Y") * panSpeed * dist * 20f;
        }
        dist = Mathf.Clamp(dist * (1f - Input.mouseScrollDelta.y * zoomSpeed * 0.1f), 0.2f, 6f);
#endif
        // (with only the new Input System enabled the camera stays put -- see README)
        Quaternion q = Quaternion.Euler(pitch, yaw + 180f, 0f);
        transform.position = target - q * Vector3.forward * dist;
        transform.LookAt(target);
    }
}
