using UnityEngine;

/// <summary>
/// Walks a Humanoid character (e.g. a Mixamo character) from direct commands, without physics.
///
/// The script moves and turns the transform itself (no forces, can't fall over) and keeps the
/// feet on the ground with a downward ray. An In Place walk clip animates the body; its playback
/// speed is scaled to the ground speed so the feet don't slide. Gestures from
/// AstronautGesturePlayer work at the same time (the arms are overridden after the Animator).
///
/// Keyboard (legacy Input Manager): hold Up arrow = walk forward, Down arrow = walk backward
/// (the normal walk clip plays, the body moves back), Left / Right arrows = turn.
///
/// Animator Controller requirements:
///   - a float parameter "Speed"
///   - Idle state (default) -> Walk when Speed > 0.1, Walk -> Idle when Speed < 0.1
///   - Apply Root Motion OFF (this script turns it off at start)
/// </summary>
[RequireComponent(typeof(Animator))]
public class AstronautWalker : MonoBehaviour
{
    [Header("Motion")]
    [Tooltip("Walking speed (m/s).")]
    public float walkSpeed = 1.0f;
    [Tooltip("Backward walking speed (m/s).")]
    public float backwardSpeed = 0.5f;
    [Tooltip("Turn rate (deg/s).")]
    public float turnRate = 90f;
    [Tooltip("How fast the speed ramps up and down (m/s^2).")]
    public float acceleration = 2f;
    [Tooltip("Ground speed (m/s) at which the walk clip played at 1x has no foot sliding. Tune by eye.")]
    public float clipNativeSpeed = 1.0f;
    [Tooltip("Play the walk clip slowly while turning on the spot, so the feet step instead of gliding.")]
    public bool stepWhenTurning = true;

    [Header("Ground")]
    public bool snapToGround = true;
    public LayerMask groundMask = ~0;
    [Tooltip("The ground probe starts this far above the feet.")]
    public float probeHeight = 2.5f;

    [Header("Input")]
    public bool keyboardControl = true;

    [Header("Debug")]
    [Tooltip("m/s, negative = walking backward.")]
    [SerializeField] private float currentSpeed;
    [SerializeField] private float currentTurnRate;

    /// <summary>Ground speed (m/s), negative = walking backward.</summary>
    public float CurrentSpeed => currentSpeed;
    /// <summary>Turn rate (deg/s), positive = turning left (counter-clockwise seen from above).</summary>
    public float CurrentTurnRate => currentTurnRate;

    Animator animator;
    int speedHash;
    bool hasSpeedParam;
    readonly RaycastHit[] hits = new RaycastHit[16];

    void Awake()
    {
        animator = GetComponent<Animator>();
        animator.applyRootMotion = false;
        speedHash = Animator.StringToHash("Speed");
        foreach (var p in animator.parameters)
            if (p.nameHash == speedHash && p.type == AnimatorControllerParameterType.Float) hasSpeedParam = true;
        if (!hasSpeedParam)
            Debug.LogWarning("AstronautWalker: the Animator Controller has no float parameter 'Speed', so the body won't animate.", this);
    }

    void Update()
    {
        float targetSpeed = 0f, turn = 0f;

#if ENABLE_LEGACY_INPUT_MANAGER
        if (keyboardControl)
        {
            targetSpeed = (Input.GetKey(KeyCode.UpArrow) ? walkSpeed : 0f)
                        - (Input.GetKey(KeyCode.DownArrow) ? backwardSpeed : 0f);
            turn = ((Input.GetKey(KeyCode.LeftArrow) ? 1f : 0f) - (Input.GetKey(KeyCode.RightArrow) ? 1f : 0f)) * turnRate;
        }
#else
        if (keyboardControl && Time.time < 0.1f)
            Debug.LogWarning("AstronautWalker: keyboard needs 'Active Input Handling' = Input Manager (Old) or Both (Project Settings > Player).", this);
#endif

        // Move and turn (Unity yaw is clockwise-positive seen from above, so negate).
        currentSpeed = Mathf.MoveTowards(currentSpeed, targetSpeed, acceleration * Time.deltaTime);
        currentTurnRate = turn;
        transform.Rotate(Vector3.up, -turn * Time.deltaTime, Space.World);
        Vector3 fwd = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
        transform.position += fwd * currentSpeed * Time.deltaTime;

        if (snapToGround) SnapToGround();

        // Animation: the walk clip's speed follows the ground speed (forward or backward).
        float animSpeed = Mathf.Abs(currentSpeed);
        if (stepWhenTurning && Mathf.Abs(turn) > 1f && animSpeed < 0.3f * walkSpeed)
            animSpeed = 0.3f * walkSpeed;
        if (hasSpeedParam) animator.SetFloat(speedHash, animSpeed);
        animator.speed = animSpeed > 0.1f && clipNativeSpeed > 0f ? animSpeed / clipNativeSpeed : 1f;
    }

    void SnapToGround()
    {
        Vector3 origin = transform.position + Vector3.up * probeHeight;
        int n = Physics.RaycastNonAlloc(origin, Vector3.down, hits, probeHeight * 2f, groundMask, QueryTriggerInteraction.Ignore);
        float best = float.MaxValue;
        float groundY = 0f;
        for (int i = 0; i < n; i++)
        {
            if (hits[i].collider.transform.IsChildOf(transform)) continue;   // skip our own collider
            if (hits[i].distance < best) { best = hits[i].distance; groundY = hits[i].point.y; }
        }
        if (best < float.MaxValue)
            transform.position = new Vector3(transform.position.x, groundY, transform.position.z);
    }
}