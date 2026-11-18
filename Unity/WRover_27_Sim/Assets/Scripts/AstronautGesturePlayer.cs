using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

/// <summary>
/// Plays arm poses on a Unity Humanoid character (e.g. Mixamo Y Bot) from plain-text motion files.
///
/// Angle convention, per arm, in the body's side plane:
///     0   = arm straight out to the side, parallel to the floor
///   +90   = straight up        -90 = straight down along the body
///   anim  = let the Animator (idle / walk clip) drive this arm
/// "right" / "left" are the character's OWN right and left (seen from the front, its right arm
/// is on your left).
///
/// Motion file (.txt, assigned as a TextAsset; the file name is the motion name):
///     # comment
///     loop                      (optional: repeat the steps forever)
///     # right  left  blend(s)  hold(s)
///       0      -45   0.5       0
/// One line per step. blend = seconds to move into the step, hold = seconds to stay before the
/// next step. blend and hold are optional (defaults 0.5 and 0). When a motion without "loop"
/// ends, the last pose is held until another motion or Release() is called.
///
/// The pose is applied in LateUpdate, after the Animator, so it overrides the walk clip's arm
/// swing while the legs keep walking.
///
/// Keyboard (legacy Input Manager): 1..9 play the motion files in the order of the list,
/// 0 releases both arms back to the animation.
/// </summary>
[RequireComponent(typeof(Animator))]
public class AstronautGesturePlayer : MonoBehaviour
{
    public class MotionStep
    {
        public float right = float.NaN;   // NaN = "anim"
        public float left = float.NaN;
        public float blend = 0.5f;
        public float hold = 0f;
    }

    public class MotionFile
    {
        public string name;
        public bool loop;
        public List<MotionStep> steps = new List<MotionStep>();
    }

    [Tooltip("Motion files (.txt). Keys 1..9 play them in this order.")]
    public TextAsset[] motionFiles = new TextAsset[0];
    [Tooltip("Keep the elbow straight while an arm is posed.")]
    public bool straightenElbows = true;
    public bool keyboardShortcuts = true;
    [Tooltip("Seconds to blend back to the animation on Release (key 0).")]
    public float releaseBlend = 0.5f;

    [SerializeField] private string currentMotion = "";

    /// <summary>Name of the motion playing or held ("" when the arms follow the animation).</summary>
    public string CurrentMotion => currentMotion;
    /// <summary>Raised when a motion starts ("" on release). Handy for publishing a ground-truth label.</summary>
    public event Action<string> MotionStarted;
    public IReadOnlyList<string> MotionNames => order;

    class Arm
    {
        public Transform upper, lower, hand;
        public int sideSign;        // +1 right arm, -1 left arm
        public float angle;         // deg, current
        public float weight;        // 0 = animation, 1 = pose
    }

    Animator animator;
    Arm rightArm, leftArm;
    readonly Dictionary<string, MotionFile> motions = new Dictionary<string, MotionFile>(StringComparer.OrdinalIgnoreCase);
    readonly List<string> order = new List<string>();
    Coroutine routine;

    // ---------------------------------------------------------------- setup
    void Awake()
    {
        animator = GetComponent<Animator>();
        if (!animator.isHuman)
        {
            Debug.LogError("AstronautGesturePlayer: set the model's Rig to Humanoid (FBX import settings > Rig).", this);
            enabled = false;
            return;
        }
        rightArm = MakeArm(HumanBodyBones.RightUpperArm, HumanBodyBones.RightLowerArm, HumanBodyBones.RightHand, +1);
        leftArm = MakeArm(HumanBodyBones.LeftUpperArm, HumanBodyBones.LeftLowerArm, HumanBodyBones.LeftHand, -1);

        foreach (var f in motionFiles)
        {
            if (f == null) continue;
            try
            {
                var m = Parse(f.name, f.text);
                motions[m.name] = m;
                order.Add(m.name);
            }
            catch (FormatException e)
            {
                Debug.LogError($"AstronautGesturePlayer: {f.name}: {e.Message}", this);
            }
        }
    }

    Arm MakeArm(HumanBodyBones upper, HumanBodyBones lower, HumanBodyBones hand, int sign)
    {
        return new Arm
        {
            upper = animator.GetBoneTransform(upper),
            lower = animator.GetBoneTransform(lower),
            hand = animator.GetBoneTransform(hand),
            sideSign = sign
        };
    }

    static MotionFile Parse(string name, string text)
    {
        var m = new MotionFile { name = name };
        string[] lines = text.Split('\n');
        for (int i = 0; i < lines.Length; i++)
        {
            string line = lines[i];
            int hash = line.IndexOf('#');
            if (hash >= 0) line = line.Substring(0, hash);
            line = line.Trim();
            if (line.Length == 0) continue;

            if (line.Equals("loop", StringComparison.OrdinalIgnoreCase)) { m.loop = true; continue; }

            string[] tok = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (tok.Length < 2)
                throw new FormatException($"line {i + 1}: expected 'right left [blend] [hold]'");

            var step = new MotionStep
            {
                right = ParseArm(tok[0], i + 1),
                left = ParseArm(tok[1], i + 1)
            };
            if (tok.Length > 2) step.blend = Mathf.Max(0f, ParseNumber(tok[2], i + 1));
            if (tok.Length > 3) step.hold = Mathf.Max(0f, ParseNumber(tok[3], i + 1));
            m.steps.Add(step);
        }
        if (m.steps.Count == 0) throw new FormatException("no steps");
        return m;
    }

    static float ParseArm(string tok, int line)
    {
        if (tok.Equals("anim", StringComparison.OrdinalIgnoreCase)) return float.NaN;
        return Mathf.Clamp(ParseNumber(tok, line), -180f, 180f);
    }

    static float ParseNumber(string tok, int line)
    {
        if (float.TryParse(tok, NumberStyles.Float, CultureInfo.InvariantCulture, out float v)) return v;
        throw new FormatException($"line {line}: '{tok}' is not a number (or 'anim')");
    }

    // ---------------------------------------------------------------- public API
    public bool Play(string motionName)
    {
        if (!motions.TryGetValue(motionName, out var m))
        {
            Debug.LogWarning($"AstronautGesturePlayer: no motion called '{motionName}'.", this);
            return false;
        }
        if (routine != null) StopCoroutine(routine);
        currentMotion = m.name;
        routine = StartCoroutine(Run(m));
        MotionStarted?.Invoke(m.name);
        return true;
    }

    /// <summary>Blend both arms back to the animation.</summary>
    public void Release()
    {
        if (routine != null) StopCoroutine(routine);
        currentMotion = "";
        routine = StartCoroutine(BlendTo(new MotionStep { blend = releaseBlend }));
        MotionStarted?.Invoke("");
    }

    // ---------------------------------------------------------------- playback
    IEnumerator Run(MotionFile m)
    {
        do
        {
            foreach (var step in m.steps)
            {
                yield return BlendTo(step);
                if (step.hold > 0f) yield return new WaitForSeconds(step.hold);
            }
        } while (m.loop);
        routine = null;   // last pose stays held
    }

    IEnumerator BlendTo(MotionStep step)
    {
        Target(step.right, rightArm, out float rAngle, out float rWeight);
        Target(step.left, leftArm, out float lAngle, out float lWeight);

        // An arm coming from the animation only blends its weight, not its angle.
        float rA0 = rightArm.weight > 0f ? rightArm.angle : rAngle, rW0 = rightArm.weight;
        float lA0 = leftArm.weight > 0f ? leftArm.angle : lAngle, lW0 = leftArm.weight;

        for (float t = 0f; t < step.blend; t += Time.deltaTime)
        {
            float k = Mathf.SmoothStep(0f, 1f, t / step.blend);
            rightArm.angle = Mathf.Lerp(rA0, rAngle, k);
            rightArm.weight = Mathf.Lerp(rW0, rWeight, k);
            leftArm.angle = Mathf.Lerp(lA0, lAngle, k);
            leftArm.weight = Mathf.Lerp(lW0, lWeight, k);
            yield return null;
        }
        rightArm.angle = rAngle; rightArm.weight = rWeight;
        leftArm.angle = lAngle; leftArm.weight = lWeight;
        yield return null;
    }

    static void Target(float value, Arm arm, out float angle, out float weight)
    {
        if (float.IsNaN(value)) { angle = arm.angle; weight = 0f; }
        else { angle = value; weight = 1f; }
    }

    // ---------------------------------------------------------------- apply after the Animator
    void LateUpdate()
    {
        ApplyArm(rightArm);
        ApplyArm(leftArm);
    }

    void ApplyArm(Arm a)
    {
        if (a == null || a.weight <= 0.001f || a.upper == null || a.lower == null) return;

        float rad = a.angle * Mathf.Deg2Rad;
        Vector3 side = transform.right * a.sideSign;
        Vector3 dir = Mathf.Cos(rad) * side + Mathf.Sin(rad) * transform.up;

        Aim(a.upper, a.lower.position - a.upper.position, dir, a.weight);
        if (straightenElbows && a.hand != null)
            Aim(a.lower, a.hand.position - a.lower.position, dir, a.weight);
    }

    static void Aim(Transform bone, Vector3 current, Vector3 desired, float weight)
    {
        Quaternion full = Quaternion.FromToRotation(current, desired) * bone.rotation;
        bone.rotation = Quaternion.Slerp(bone.rotation, full, weight);
    }

    // ---------------------------------------------------------------- keyboard
#if ENABLE_LEGACY_INPUT_MANAGER
    void Update()
    {
        if (!keyboardShortcuts) return;
        if (Input.GetKeyDown(KeyCode.Alpha0)) { Release(); return; }
        for (int i = 0; i < order.Count && i < 9; i++)
            if (Input.GetKeyDown(KeyCode.Alpha1 + i)) Play(order[i]);
    }
#endif
}
