using System;
using System.Collections.Generic;
using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CarDriver : Agent
{
    public enum Axel
    {
        Front,
        Rear
    }

    [Serializable]
    public struct Wheel
    {
        public GameObject wheelModel;
        public WheelCollider wheelCollider;
        public Axel axel;
    }

    [Header("Raycast setup")]
    public float heightOfCheck = 10f;
    public float rangeOfCheck = 300f;
    public float spawnRange = 5f;
    public LayerMask layerMask;

    [Header("Car settings")]
    public float maxAcceleration = 30.0f;
    public float turnSensitivity = 1.0f;
    public float maxSteerAngle = 30.0f;

    [Header("Run stats")]
    public int goalsReached = 0;
    public int goalsThisepisode = 0;
    public int numEpisodes = 0;
    public float closestDistanceToTarget = 9999999999999.9f;
    public Vector3 AgentHeading;
    public Vector3 GoalDirection;
    public float angle = 0f;
    public bool colliding = false;

    public float timeThreshold = 50.0f;
    public float timer = 0f;

    float forwardAmount;
    float turnAmount;

    public List<Wheel> wheels;

    private Rigidbody carRb;

    [Header("Object transforms")]
    public Transform TargetTransform;
    public Transform PlayerTransform;

    void Start()
    {
        carRb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        AnimateWheels();

        float distToTarget = Vector3.Distance(TargetTransform.localPosition, PlayerTransform.localPosition);

        if (distToTarget < closestDistanceToTarget)
        {
            AddReward(0.1f);
            closestDistanceToTarget = distToTarget;
        }

        timer += Time.deltaTime;
        if (timer >= timeThreshold)
        {
            if (carRb.velocity.magnitude <= 0.2f)
            {
                AddReward(-3f);
            }

            timer = 0f;
        }

        if (Input.GetKeyDown(KeyCode.N))
        {
            NewGoal();
        }

        Vector3 GoalDirectionb = (TargetTransform.position - PlayerTransform.position);
        GoalDirection = GoalDirectionb.normalized;
        AgentHeading = PlayerTransform.transform.forward;

        angle = Vector3.Angle(AgentHeading, GoalDirection);

        if (angle > 90)
        {
            AddReward(-0.001f);
        }

        /* used for manaul testing */
        /*
        forwardAmount = Input.GetAxis("Vertical");
        turnAmount = Input.GetAxis("Horizontal");

        foreach (var wheel in wheels)
        {
            wheel.wheelCollider.motorTorque = forwardAmount * 600 * maxAcceleration * Time.deltaTime;
        }

        foreach (var wheel in wheels)
        {
            if (wheel.axel == Axel.Front)
            {
                var _steerAngle = turnAmount * turnSensitivity * maxSteerAngle;
                wheel.wheelCollider.steerAngle = Mathf.Lerp(wheel.wheelCollider.steerAngle, _steerAngle, 0.6f);
            }
        }
        */
        
    }
    void AnimateWheels()
    {
        foreach (var wheel in wheels)
        {
            Quaternion rot;
            Vector3 pos;
            wheel.wheelCollider.GetWorldPose(out pos, out rot);
            wheel.wheelModel.transform.position = pos;
            wheel.wheelModel.transform.rotation = rot;
        }
    }

    public void NewGoal()
    {
        // Generate a random position for the goal
        float xPosition = UnityEngine.Random.Range(-120, 120);
        float zPosition = UnityEngine.Random.Range(-147, 47);

        // Set Vector2 variables for the finnish and player ignoring the height
        Vector2 agentVec2 = new Vector2(PlayerTransform.localPosition.x, PlayerTransform.localPosition.z);
        Vector2 goalVec2 = new Vector2(xPosition, zPosition);

        // Do same raycast magic and reposition the target to somewhere on the ground
        RaycastHit hit;
        // If the ray hits something
        if (Physics.Raycast(new Vector3(xPosition, heightOfCheck, zPosition),
            Vector3.down, out hit, rangeOfCheck, layerMask))
        {
            // If the ray hits ground and is not further than spawnRange we move the goal
            if (hit.collider.CompareTag("ground") && (Vector2.Distance(agentVec2,goalVec2) < spawnRange))
            {
            TargetTransform.localPosition = new Vector3(xPosition, hit.point.y, zPosition);
            }
            else
            {
                NewGoal();
            }
        }
        else
        {
            NewGoal();
        }

        // reset the target distance since the goal has moved
        closestDistanceToTarget = 9999999999999.9f;
    }

    public override void OnEpisodeBegin()
    {
        print("epStart");
        colliding = false;
        numEpisodes += 1;
        closestDistanceToTarget = 9999999999999.9f;
        goalsThisepisode = 0;
        spawnRange = 20 + goalsReached / numEpisodes;

        // Generate a random position for the goal
        float xPosition = UnityEngine.Random.Range(-90, 91);
        float zPosition = UnityEngine.Random.Range(-450, -320);

        // Assign the randomly generated position to the goal
        TargetTransform.localPosition = new Vector3(xPosition, 60, zPosition);

        // make sure that the agent is not moving on respawn and is facing the correct way
        carRb.velocity = Vector3.zero;
        carRb.rotation = Quaternion.Euler(0, 178.077f, 0);

        // spawn in the agent
        PlayerTransform.localPosition = new Vector3(57.9f, 50.41f, 0.2000122f);

        // give the agent a free one
        TargetTransform.localPosition = new Vector3(66f, 50f, -8f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // The position of the agent
        sensor.AddObservation(PlayerTransform.localPosition.x);
        sensor.AddObservation(PlayerTransform.localPosition.y);
        sensor.AddObservation(PlayerTransform.localPosition.z);

        // The position of the goal
        sensor.AddObservation(TargetTransform.localPosition.x);
        sensor.AddObservation(TargetTransform.localPosition.y);
        sensor.AddObservation(TargetTransform.localPosition.z);

        // The distance between the agent and the goal
        sensor.AddObservation(Vector3.Distance(TargetTransform.localPosition, PlayerTransform.localPosition));

        // The closeset the agent has been to the goal
        sensor.AddObservation(closestDistanceToTarget);

        // The direction of the goal
        sensor.AddObservation(GoalDirection);

        // The current heading of the agent
        sensor.AddObservation(AgentHeading);

        // The deviation of the agent from the goals direction
        sensor.AddObservation(angle);

        // The speed of the agent
        sensor.AddObservation(carRb.velocity.magnitude);

        // The timer
        sensor.AddObservation(timer);

        // Whether or not the agent is colliding with something
        sensor.AddObservation(colliding);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var actionTaken = actions.ContinuousActions;

        float forwardAmount = actionTaken[0];
        float turnAmount = actionTaken[1];

        foreach (var wheel in wheels)
        {
            wheel.wheelCollider.motorTorque = forwardAmount * 600 * maxAcceleration * Time.deltaTime;
        }

        foreach (var wheel in wheels)
        {
            if (wheel.axel == Axel.Front)
            {
                var _steerAngle = turnAmount * turnSensitivity * maxSteerAngle;
                wheel.wheelCollider.steerAngle = Mathf.Lerp(wheel.wheelCollider.steerAngle, _steerAngle, 0.6f);
            }
        }

        // AddReward(-0.01f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;

        actions[0] = Input.GetAxis("Vertical");
        actions[1] = Input.GetAxis("Horizontal");
    }

    private void OnTriggerEnter(Collider Finish)
    {
        print("goalReached");
        AddReward(100f);
        goalsReached += 1;
        goalsThisepisode += 1;
        spawnRange += 1;
        NewGoal();
    }

    private void OnCollisionEnter(Collision collision)
    {
        colliding = true;
        if (collision.collider.tag == "boundaryRock")
        {
            print("rockHit");
            AddReward(-100f*(math.abs(goalsThisepisode-1)));
            EndEpisode();
        }
        if (collision.collider.tag == "prop")
        {
            print("propHit");
            AddReward(-1f);
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        colliding = true;
        if (collision.collider.tag == "prop")
        {
            AddReward(-0.2f);
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        colliding = false;
        if (collision.collider.tag == "prop")
        {
            AddReward(0.8f);
        }
    }
}
