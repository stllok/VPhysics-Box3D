#include "vbox_controller_jeep.h"

#include "cbase.h"
#include "vbox_collide.h"
#include "vbox_environment.h"
#include "vbox_object.h"
#include "vbox_surfaceprops.h"

#include <algorithm>
#include <cmath>
#include <cstring>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static constexpr int kFVehicleHandbrakeOn{ 0x00000002 };

static constexpr float kHpToWatts{ 745.7f };
static constexpr float kThrottleOppositionSpeed{ SourceToBox::Distance(5.0f) };
static constexpr float kCoastingBrakeFraction{ 0.1f };

static constexpr float kSteeringHertz{ 30.0f };
static constexpr float kSteeringDampingRatio{ 1.0f };
static constexpr float kSteeringTorqueCap{ 1e9f };

static constexpr float kSuspensionHertzMax{ 60.0f };
static constexpr float kSuspensionDampingRatioMax{ 10.0f };
static constexpr float kWheelRadiusMin{ 1.0f };
static constexpr float kWheelCoincidentDist{ 2.0f };

static b3Quat WheelJointFrameRotation()
{
    return b3MakeQuatFromAxisAngle(b3Vec3{ 0.0f, 1.0f, 0.0f }, -0.5f * B3_PI);
}

static float WheelMaterialFriction(int materialIndex)
{
    surfacedata_t* pSurface = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(materialIndex);
    return pSurface ? pSurface->physics.friction : 1.0f;
}

Box3DVehicleJeep::Box3DVehicleJeep(
    const vehicleparams_t& params, Box3DPhysicsEnvironment* pEnv, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace)
    : Box3DVehicleController(params, pEnv, nVehicleType, pGameTrace)
{
    std::fill_n(m_wheelJoints, VEHICLE_MAX_WHEEL_COUNT, b3_nullJointId);
    m_flBoostDelay = 0.0f;
    m_flBoosterRemaining = 0.0f;
    m_gear = 0;
}

Box3DVehicleJeep::~Box3DVehicleJeep()
{
    for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++)
    {
        // The chassis body may already be gone (the game owns it); a destroyed
        // body takes its joints with it, so only destroy joints still alive.
        if (B3_IS_NON_NULL(m_wheelJoints[i]) && b3Joint_IsValid(m_wheelJoints[i]))
        {
            m_pEnv->DestroyJointSafely(m_wheelJoints[i]);
        }
        m_wheelJoints[i] = b3_nullJointId;
    }
}

void Box3DVehicleJeep::OnObjectDestroyed(Box3DPhysicsObject* pObject)
{
    // A wheel the controller owns is normally destroyed in the dtor, but if the
    // game destroyed one out from under us, forget its joint (which dies with
    // the wheel body). The base clears the chassis/wheel pointers.
    if (pObject)
    {
        for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++)
        {
            if (m_pWheels[i] == pObject)
            {
                m_wheelJoints[i] = b3_nullJointId;
            }
        }
    }

    Box3DVehicleController::OnObjectDestroyed(pObject);
}

void Box3DVehicleJeep::SetSpringLength(int wheelIndex, float length)
{
    Box3DVehicleController::SetSpringLength(wheelIndex, length);
    if (wheelIndex < 0 || wheelIndex >= m_wheelCount || m_vehicleData.wheelsPerAxle <= 0)
        return;

    const int axleIndex = wheelIndex / m_vehicleData.wheelsPerAxle;
    if (axleIndex < 0 || axleIndex >= m_vehicleData.axleCount)
        return;

    const float travel = SourceToBox::Distance(Max(m_vehicleData.axles[axleIndex].wheels.springAdditionalLength, 0.0f));
    const int nFirstWheel = axleIndex * m_vehicleData.wheelsPerAxle;
    const int nLastWheel = Min(nFirstWheel + m_vehicleData.wheelsPerAxle, m_wheelCount);
    for (int i = nFirstWheel; i < nLastWheel; i++)
    {
        if (B3_IS_NON_NULL(m_wheelJoints[i]) && b3Joint_IsValid(m_wheelJoints[i]))
            b3WheelJoint_SetSuspensionLimits(m_wheelJoints[i], 0.0f, travel);
    }
}

void Box3DVehicleJeep::AttachWheels()
{
    if (!m_pEnv)
        return;

    for (int i = 0; i < m_wheelCount; i++)
    {
        if (!IsRaycastVehicle())
        {
            CreateWheelJoint(i);
        }
    }

    // Same-vehicle wheels never collide with each other: overlapping wheel
    // spheres would generate degenerate contact normals. Raycast proxies do
    // not collide at all, so they need no filters.
    if (!IsRaycastVehicle())
    {
        for (int i = 0; i < m_wheelCount; i++)
        {
            if (!m_pWheels[i])
                continue;
            for (int j = i + 1; j < m_wheelCount; j++)
            {
                if (!m_pWheels[j])
                    continue;
                b3FilterJointDef filterDef = b3DefaultFilterJointDef();
                filterDef.base.bodyIdA = m_pWheels[i]->GetBodyID();
                filterDef.base.bodyIdB = m_pWheels[j]->GetBodyID();
                b3CreateFilterJoint(m_pEnv->GetWorldId(), &filterDef);
            }
        }
    }
}

void Box3DVehicleJeep::CreateWheelJoint(int wheelIndex)
{
    Box3DPhysicsObject* pWheel = m_pWheels[wheelIndex];
    if (!pWheel || !m_pEnv || B3_IS_NULL(m_carBodyId))
        return;

    const int axleIndex = wheelIndex / m_vehicleData.wheelsPerAxle;
    const vehicle_axleparams_t& axle = m_vehicleData.axles[axleIndex];
    const float additionalLength = Max(axle.wheels.springAdditionalLength, 0.0f);

    const float stiffness = axle.suspension.springConstant * m_bodyMass;
    const float damping = axle.suspension.springDamping * m_bodyMass;
    const float wheelMass = Max(pWheel->GetMass(), 1.0f);
    const float chassisShare = Max(m_bodyMass / Max(m_wheelCount, 1), 1.0f);
    const float effectiveMass = 1.0f / (1.0f / wheelMass + 1.0f / chassisShare);

    // springConstant <= 0 leaves hertz at 0: the spring stays disabled and the
    // suspension is rigid through its limits instead of a degenerate spring.
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    if (stiffness > 0.0f && effectiveMass > 0.0f)
    {
        hertz = Min(sqrtf(stiffness / effectiveMass) / (2.0f * M_PI_F), kSuspensionHertzMax);
        dampingRatio = clamp(damping / (2.0f * sqrtf(stiffness * effectiveMass)), 0.0f, kSuspensionDampingRatioMax);
    }

    const b3Quat frameRotation = WheelJointFrameRotation();

    b3WheelJointDef def = b3DefaultWheelJointDef();
    def.base.bodyIdA = m_carBodyId;
    def.base.bodyIdB = pWheel->GetBodyID();
    // Frame A sits at the fully extended wheel position: the joint spring's
    // rest (zero translation) is the suspension's natural length, matching the
    // Jolt setup of min length 0 / max length springAdditionalLength below the
    // authored wheel position. Wheel offset is a Source distance -> metres.
    def.base.localFrameA.p = SourceToBox::Distance(m_wheelPosition_Bs[wheelIndex] - Vector(0.0f, 0.0f, additionalLength));
    def.base.localFrameA.q = frameRotation;
    def.base.localFrameB.p = b3Vec3{ 0.0f, 0.0f, 0.0f };
    def.base.localFrameB.q = frameRotation;
    def.base.collideConnected = false;
    def.enableSuspensionSpring = hertz > 0.0f;
    def.suspensionHertz = hertz;
    def.suspensionDampingRatio = dampingRatio;
    def.enableSuspensionLimit = true;
    def.lowerSuspensionLimit = 0.0f;
    // Suspension travel is a Source distance -> metres.
    def.upperSuspensionLimit = SourceToBox::Distance(additionalLength);
    def.enableSpinMotor = true;
    def.maxSpinTorque = 0.0f;
    def.spinSpeed = 0.0f;
    if (axleIndex == 0)
    {
        const float maxSteering = MaxSteeringAngleRad();
        if (maxSteering > 0.0f)
        {
            def.enableSteering = true;
            def.steeringHertz = kSteeringHertz;
            def.steeringDampingRatio = kSteeringDampingRatio;
            def.targetSteeringAngle = 0.0f;
            def.maxSteeringTorque = kSteeringTorqueCap;
            def.enableSteeringLimit = true;
            def.lowerSteeringLimit = -maxSteering;
            def.upperSteeringLimit = maxSteering;
        }
    }

    m_wheelJoints[wheelIndex] = b3CreateWheelJoint(m_pEnv->GetWorldId(), &def);
}

// Suspension articulation along the chassis up axis, in box metres.
float Box3DVehicleJeep::SuspensionTranslation(int wheelIndex) const
{
    const Box3DPhysicsObject* pWheel = m_pWheels[wheelIndex];
    if (!pWheel)
        return 0.0f;
    b3BodyId wheelBody = pWheel->GetBodyID();
    if (B3_IS_NULL(wheelBody) || !b3Body_IsValid(wheelBody))
        return 0.0f;

    const int axleIndex = wheelIndex / m_vehicleData.wheelsPerAxle;
    const float additionalLength = Max(m_vehicleData.axles[axleIndex].wheels.springAdditionalLength, 0.0f);

    b3WorldTransform xf = b3Body_GetTransform(m_carBodyId);
    b3Pos attach = b3TransformWorldPoint(
        xf, SourceToBox::Distance(m_wheelPosition_Bs[wheelIndex] - Vector(0.0f, 0.0f, additionalLength)));
    b3Vec3 up = b3RotateVector(xf.q, b3Vec3{ 0.0f, 0.0f, 1.0f });
    return b3Dot(up, b3SubPos(b3Body_GetPosition(wheelBody), attach));
}

float Box3DVehicleJeep::UpdateBooster(float dt)
{
    m_flBoostDelay = Max(m_flBoostDelay - dt, 0.0f);
    m_flBoosterRemaining = Max(m_flBoosterRemaining - dt, 0.0f);
    m_currentState.boostDelay = m_flBoostDelay;
    return m_flBoostDelay;
}

void Box3DVehicleJeep::HandleBoostKey()
{
    if (m_controls.boost && m_flBoostDelay == 0.0f && m_flBoosterRemaining == 0.0f)
    {
        m_flBoosterRemaining = m_vehicleData.engine.boostDuration;
        m_flBoostDelay = m_vehicleData.engine.boostDuration + m_vehicleData.engine.boostDelay;
    }
}

void Box3DVehicleJeep::HandleBoostDecay()
{
    const float total = m_vehicleData.engine.boostDuration + m_vehicleData.engine.boostDelay;
    if (total > 0.0f)
    {
        m_currentState.boostTimeLeft = m_flBoostDelay != 0.0f ? static_cast<int>(100.0f - (100.0f * (m_flBoostDelay / total)))
                                                              : 100;
    }
}

void Box3DVehicleJeep::Update(float dt, vehicle_controlparams_t& controls)
{
    m_controls = controls;

    if (controls.handbrake)
    {
        m_vehicleFlags |= kFVehicleHandbrakeOn;
    }
    else
    {
        m_vehicleFlags &= ~kFVehicleHandbrakeOn;
    }

    UpdateBooster(dt);
    HandleBoostKey();
}

void Box3DVehicleJeep::OnPreSimulate(float dt)
{
    if (m_wheelCount <= 0 || m_vehicleData.wheelsPerAxle <= 0)
        return;
    if (B3_IS_NULL(m_carBodyId) || !b3Body_IsValid(m_carBodyId))
        return;

    const b3Vec3 forward = b3Body_GetWorldVector(m_carBodyId, b3Vec3{ 0.0f, 1.0f, 0.0f });
    const float speed = b3Dot(forward, b3Body_GetLinearVelocity(m_carBodyId));

    // With any user input, assure that the car is active.
    if (m_controls.steering != 0.0f || m_controls.throttle != 0.0f || m_controls.brake != 0.0f || m_controls.handbrake)
    {
        b3Body_SetAwake(m_carBodyId, true);
    }

    const DriverInputs inputs = ProcessDriverInputs(speed);

    if (IsRaycastVehicle())
    {
        SimulateRaycast(dt, speed, inputs);
    }
    else
    {
        SimulateWheeled(dt, speed, inputs);
    }

    // The game interface reports speed in Source in/s.
    m_currentState.speed = BoxToSource::Distance(speed);
    m_currentState.gear = m_gear;
    m_currentState.boostDelay = m_flBoostDelay;
    HandleBoostDecay();
}

Box3DVehicleJeep::DriverInputs Box3DVehicleJeep::ProcessDriverInputs(float speed)
{
    const vehicle_engineparams_t& engine = m_vehicleData.engine;

    DriverInputs inputs{};
    inputs.torqueMultiplier = 1.0f;
    inputs.handbrake = m_controls.handbrake;

    // Don't throttle when holding handbrake (like Source).
    inputs.throttle = inputs.handbrake ? 0.0f : m_controls.throttle;
    if (m_bEngineDisable)
    {
        inputs.throttle = 0.0f;
    }

    const bool coasting = inputs.throttle == 0.0f && m_controls.brake == 0.0f && !inputs.handbrake;
    inputs.brake = coasting ? kCoastingBrakeFraction : m_controls.brake;

    // Enable the handbrake when driving against the current motion at low
    // speed to avoid slipping when going up hill.
    if ((inputs.throttle < 0.0f && speed > kThrottleOppositionSpeed)
        || (inputs.throttle > 0.0f && speed < -kThrottleOppositionSpeed))
    {
        inputs.handbrake = true;
    }

    inputs.boosting = m_flBoosterRemaining > 0.0f && !m_bEngineDisable;
    if (inputs.boosting)
    {
        m_controls.throttle = 1.0f;
        inputs.throttle = 1.0f;
        const float speedFraction = m_engineMaxSpeed > 0.0f
            ? clamp(fabsf(speed) / SourceToBox::Distance(m_engineMaxSpeed), 0.0f, 1.0f)
            : 1.0f;
        const float speedFactor = 0.1f + 0.9f * speedFraction;
        const float turnFactor = 1.0f - fabsf(m_controls.steering) * 0.95f;
        inputs.torqueMultiplier = Max(1.0f, 2.0f * engine.boostForce * speedFactor * turnFactor);
    }

    return inputs;
}

float Box3DVehicleJeep::GearRatioSafe(int gear) const
{
    const int gearCount = clamp(m_vehicleData.engine.gearCount, 0, VEHICLE_MAX_GEAR_COUNT);
    if (gearCount <= 0)
        return 1.0f;
    return m_vehicleData.engine.gearRatio[clamp(gear, 0, gearCount - 1)];
}

// Estimate engine RPM from the driven wheels' spin and auto-shift.
float Box3DVehicleJeep::UpdateTransmission(float avgSpin, bool boosting)
{
    const vehicle_engineparams_t& engine = m_vehicleData.engine;
    const int gearCount = clamp(engine.gearCount, 0, VEHICLE_MAX_GEAR_COUNT);

    constexpr float kRadPerSecToRpm = 60.0f / (2.0f * M_PI_F);
    m_gear = gearCount > 0 ? clamp(m_gear, 0, gearCount - 1) : 0;
    float rpm = avgSpin * kRadPerSecToRpm * engine.axleRatio * GearRatioSafe(m_gear);
    if (engine.isAutoTransmission && gearCount > 0)
    {
        if (rpm > engine.shiftUpRPM && m_gear < gearCount - 1)
        {
            m_gear++;
        }
        else if (rpm < engine.shiftDownRPM && m_gear > 0)
        {
            m_gear--;
        }
        rpm = avgSpin * kRadPerSecToRpm * engine.axleRatio * GearRatioSafe(m_gear);
    }
    rpm = clamp(rpm, 0.0f, engine.maxRPM);
    if (boosting)
    {
        rpm = engine.maxRPM;
    }
    return rpm;
}

float Box3DVehicleJeep::EngineTorqueAtRedline() const
{
    const vehicle_engineparams_t& engine = m_vehicleData.engine;
    if (engine.maxRPM <= 0.0f)
        return 0.0f;
    return kHpToWatts * engine.horsepower / (engine.maxRPM * (2.0f * M_PI_F / 60.0f));
}

void Box3DVehicleJeep::SimulateWheeled(float, float, const DriverInputs& inputs)
{
    const vehicle_engineparams_t& engine = m_vehicleData.engine;

    float avgSpin = 0.0f;
    int spinSamples = 0;
    for (int i = 0; i < m_wheelCount; i++)
    {
        const int axleIndex = i / m_vehicleData.wheelsPerAxle;
        if (m_vehicleData.axles[axleIndex].torqueFactor <= 0.0f)
            continue;
        if (B3_IS_NULL(m_wheelJoints[i]) || !b3Joint_IsValid(m_wheelJoints[i]))
            continue;
        avgSpin += fabsf(b3WheelJoint_GetSpinSpeed(m_wheelJoints[i]));
        spinSamples++;
    }
    if (spinSamples)
    {
        avgSpin /= spinSamples;
    }

    const float rpm = UpdateTransmission(avgSpin, inputs.boosting);

    // Engine torque budget in N*m.
    const float wheelTorqueBudget = fabsf(inputs.throttle) * EngineTorqueAtRedline() * GearRatioSafe(m_gear) * engine.axleRatio
        * inputs.torqueMultiplier;

    const float topSpeed = SourceToBox::Distance(
        inputs.boosting && m_engineBoostMaxSpeed > 0.0f ? m_engineBoostMaxSpeed : m_engineMaxSpeed);
    const float targetLinearSpeed = inputs.throttle >= 0.0f ? topSpeed : -SourceToBox::Distance(m_engineMaxRevSpeed);

    // Reference brake force in N (gravity is m/s^2, mass is kg).
    const float brakeTorqueBase = 0.5f * m_gravityLength * (m_bodyMass + m_totalWheelMass);
    const float targetSteeringAngle = -m_controls.steering * MaxSteeringAngleRad();
    const float brake = inputs.brake;
    const bool handbrake = inputs.handbrake;

    for (int i = 0; i < m_wheelCount; i++)
    {
        b3JointId joint = m_wheelJoints[i];
        if (B3_IS_NULL(joint) || !b3Joint_IsValid(joint))
            continue;

        const int axleIndex = i / m_vehicleData.wheelsPerAxle;
        const vehicle_axleparams_t& axle = m_vehicleData.axles[axleIndex];
        const float radius = Max(axle.wheels.radius, kWheelRadiusMin);
        // Wheel radius in metres for the box-space torque/spin math.
        const float radiusM = SourceToBox::Distance(radius);

        const float driveTorque = wheelTorqueBudget * axle.torqueFactor * m_torqueScale / m_vehicleData.wheelsPerAxle;
        float brakeTorque = brakeTorqueBase * axle.brakeFactor * radiusM * brake;
        if (handbrake)
        {
            brakeTorque += brakeTorqueBase * radiusM;
        }

        if (brakeTorque > driveTorque)
        {
            b3WheelJoint_SetSpinMotorSpeed(joint, 0.0f);
            b3WheelJoint_SetMaxSpinTorque(joint, brakeTorque);
        }
        else
        {
            b3WheelJoint_SetSpinMotorSpeed(joint, targetLinearSpeed / radiusM);
            b3WheelJoint_SetMaxSpinTorque(joint, driveTorque);
        }

        if (axleIndex == 0 && b3WheelJoint_IsSteeringEnabled(joint))
        {
            b3WheelJoint_SetTargetSteeringAngle(joint, targetSteeringAngle);
        }
    }

    // Stabilizer bars: an anti-roll couple on the chassis from each axle
    // pair's suspension articulation difference. The constant lives in the
    // same per-body-mass regime as springConstant. Applied to the chassis
    // (not the light wheel bodies) and bounded by the suspension travel, so a
    // stiff constant on near-massless wheels cannot blow the solver up; zero
    // travel means zero articulation and the bar stays inert.
    if (m_vehicleData.wheelsPerAxle == 2)
    {
        b3WorldTransform chassisTransform = b3Body_GetTransform(m_carBodyId);
        const b3Vec3 up = b3RotateVector(chassisTransform.q, b3Vec3{ 0.0f, 0.0f, 1.0f });
        for (int axleIndex = 0; axleIndex < m_vehicleData.axleCount; axleIndex++)
        {
            const vehicle_axleparams_t& axle = m_vehicleData.axles[axleIndex];
            const float stabilizer = axle.suspension.stabilizerConstant * m_bodyMass;
            // Suspension travel in metres, matching SuspensionTranslation.
            const float travel = SourceToBox::Distance(Max(axle.wheels.springAdditionalLength, 0.0f));
            if (stabilizer <= 0.0f || travel <= 0.0f)
                continue;

            const int i0 = axleIndex * 2;
            const int i1 = i0 + 1;
            if (i1 >= m_wheelCount || !m_pWheels[i0] || !m_pWheels[i1])
                continue;

            const float articulation = clamp(SuspensionTranslation(i0) - SuspensionTranslation(i1), -travel, travel);
            const float force = stabilizer * articulation;
            b3Pos attach0 = b3TransformWorldPoint(chassisTransform, SourceToBox::Distance(m_wheelPosition_Bs[i0]));
            b3Pos attach1 = b3TransformWorldPoint(chassisTransform, SourceToBox::Distance(m_wheelPosition_Bs[i1]));
            b3Body_ApplyForce(m_carBodyId, b3MulSV(force, up), attach0, false);
            b3Body_ApplyForce(m_carBodyId, b3MulSV(-force, up), attach1, false);
        }
    }

    // Operating params reflect the last completed step.
    m_currentState.engineRPM = rpm;

    float steeringAngle = 0.0f;
    for (int i = 0; i < Min(m_vehicleData.wheelsPerAxle, m_wheelCount); i++)
    {
        b3JointId joint = m_wheelJoints[i];
        if (B3_IS_NULL(joint) || !b3Joint_IsValid(joint) || !b3WheelJoint_IsSteeringEnabled(joint))
            continue;
        const float angle = b3WheelJoint_GetSteeringAngle(joint);
        if (fabsf(angle) > fabsf(steeringAngle))
        {
            steeringAngle = angle;
        }
    }

    m_currentState.steeringAngle = -RAD2DEG(steeringAngle);

    m_currentState.wheelsInContact = 0;
    m_currentState.wheelsNotInContact = 0;
    for (int i = 0; i < m_wheelCount; i++)
    {
        if (WheelContact(i, nullptr, nullptr))
        {
            m_currentState.wheelsInContact++;
        }
        else
        {
            m_currentState.wheelsNotInContact++;
        }
    }
}

void Box3DVehicleJeep::SimulateRaycast(float dt, float speed, const DriverInputs& inputs)
{
    const vehicle_engineparams_t& engine = m_vehicleData.engine;
    const b3WorldTransform xf = b3Body_GetTransform(m_carBodyId);
    const b3Vec3 upB3 = b3RotateVector(xf.q, b3Vec3{ 0.0f, 0.0f, 1.0f });
    const b3Vec3 forwardB3 = b3RotateVector(xf.q, b3Vec3{ 0.0f, 1.0f, 0.0f });
    const Vector up = BoxToSource::Unitless(upB3);
    const QAngle chassisAngles = BoxToSource::Angle(xf.q);

    float radiusSum = 0.0f;
    int radiusSamples = 0;
    for (int a = 0; a < m_vehicleData.axleCount; a++)
    {
        if (m_vehicleData.axles[a].torqueFactor <= 0.0f)
            continue;
        radiusSum += Max(m_vehicleData.axles[a].wheels.radius, kWheelRadiusMin);
        radiusSamples++;
    }
    const float avgRadius = radiusSamples ? radiusSum / radiusSamples : kWheelRadiusMin;
    // Average driven-wheel radius in metres for the box-space spin/thrust math.
    const float avgRadiusM = SourceToBox::Distance(avgRadius);
    const float rpm = UpdateTransmission(fabsf(speed) / avgRadiusM, inputs.boosting);
    m_currentState.engineRPM = rpm;

    // Engine torque budget in N*m.
    const float wheelTorqueBudget = fabsf(inputs.throttle) * EngineTorqueAtRedline() * GearRatioSafe(m_gear) * engine.axleRatio
        * inputs.torqueMultiplier;

    const float topSpeed = SourceToBox::Distance(
        inputs.boosting && m_engineBoostMaxSpeed > 0.0f ? m_engineBoostMaxSpeed : m_engineMaxSpeed);
    const bool speedCapped = inputs.throttle >= 0.0f ? speed > topSpeed : speed < -SourceToBox::Distance(m_engineMaxRevSpeed);
    const float driveDirection = inputs.throttle >= 0.0f ? 1.0f : -1.0f;

    // Reference brake force in N.
    const float brakeForceBase = 0.5f * m_gravityLength * (m_bodyMass + m_totalWheelMass);
    const float massShare = m_bodyMass / Max(m_wheelCount, 1);
    const float invDt = dt > 0.0f ? 1.0f / dt : 0.0f;
    const float steerAngle = -m_controls.steering * MaxSteeringAngleRad();

    int wheelsInContact = 0;
    for (int i = 0; i < m_wheelCount; i++)
    {
        RaycastWheelContact& state = m_raycastContacts[i];
        state = RaycastWheelContact{};

        const int axleIndex = i / m_vehicleData.wheelsPerAxle;
        const vehicle_axleparams_t& axle = m_vehicleData.axles[axleIndex];
        const float radius = Max(axle.wheels.radius, kWheelRadiusMin);
        const float travel = Max(axle.wheels.springAdditionalLength, 0.0f);
        const float castDist = travel + radius;

        // Attach point transformed in box space, then read back to Source for
        // the (Source-space) trace.
        const b3Pos attach = b3TransformWorldPoint(xf, SourceToBox::Distance(m_wheelPosition_Bs[i]));
        const Vector start = BoxToSource::Distance(attach);
        const Vector dirDown = -up;

        float hitDist = -1.0f;
        Vector normal;
        int surfaceProps = 0;
        bool inWater = false;
        CastWheel(start, dirDown, castDist, &hitDist, &normal, &surfaceProps, &inWater);

        Vector wheelCenter = start + dirDown * travel;
        if (hitDist >= 0.0f)
        {
            wheelsInContact++;

            // Suspension spring is box-space: compression -> metres and its rate
            // is the up-axis chassis velocity at the attach point (m/s), so the
            // resulting force is in N. springConstant/springDamping carry 1/s^2
            // and 1/s, so the products land in N directly.
            const float compression = SourceToBox::Distance(castDist - hitDist);
            const float compressionRate = -b3Dot(b3Body_GetWorldPointVelocity(m_carBodyId, attach), upB3);
            const float stiffness = axle.suspension.springConstant * m_bodyMass;
            const float damping = axle.suspension.springDamping * m_bodyMass;
            float suspension = stiffness * compression + damping * compressionRate;
            if (suspension < 0.0f)
            {
                suspension = 0.0f;
            }
            if (axle.suspension.maxBodyForce > 0.0f)
            {
                suspension = Min(suspension, axle.suspension.maxBodyForce * m_bodyMass);
            }
            b3Body_ApplyForce(m_carBodyId, b3MulSV(suspension, upB3), attach, false);

            const Vector contact = start + dirDown * hitDist;
            wheelCenter = start + dirDown * Max(hitDist - radius, 0.0f);

            state.inContact = true;
            state.point = contact;
            state.normal = normal;
            state.surfaceProps = surfaceProps;
            state.inWater = inWater;
            state.suspensionForce = suspension;

            b3Vec3 wheelDir = forwardB3;
            if (axleIndex == 0 && steerAngle != 0.0f)
            {
                wheelDir = b3RotateVector(b3MakeQuatFromAxisAngle(upB3, steerAngle), forwardB3);
            }
            Vector longDir = BoxToSource::Unitless(wheelDir) - normal * DotProduct(BoxToSource::Unitless(wheelDir), normal);
            if (longDir.LengthSqr() > 1e-6f)
            {
                VectorNormalize(longDir);
                const Vector latDir = CrossProduct(normal, longDir);

                // Contact velocity kept in box m/s (Unitless: no unit scale) so
                // the force math below stays in N.
                const Vector contactVel = BoxToSource::Unitless(
                    b3Body_GetWorldPointVelocity(m_carBodyId, SourceToBox::Distance(contact)));
                const float vLong = DotProduct(contactVel, longDir);
                const float vLat = DotProduct(contactVel, latDir);

                float mu = m_wheelFrictionOverride[i];
                if (mu < 0.0f)
                {
                    mu = axle.wheels.frictionScale > 0.0f ? axle.wheels.frictionScale
                                                          : WheelMaterialFriction(axle.wheels.materialIndex);
                }
                const float budget = mu * suspension;

                float driveForce = 0.0f;
                if (!speedCapped)
                {
                    driveForce = driveDirection * wheelTorqueBudget * axle.torqueFactor * m_torqueScale
                        / (m_vehicleData.wheelsPerAxle * SourceToBox::Distance(radius));
                }
                float brakeForce = brakeForceBase * axle.brakeFactor * inputs.brake;
                if (inputs.handbrake)
                {
                    brakeForce += brakeForceBase;
                }
                // Brakes oppose the contact's longitudinal motion and never
                // reverse it within a step.
                brakeForce = Min(brakeForce, massShare * fabsf(vLong) * invDt);
                const float longSign = vLong > 0.0f ? 1.0f : (vLong < 0.0f ? -1.0f : 0.0f);
                float longForce = clamp(driveForce - longSign * brakeForce, -budget, budget);

                const float lateralBudget = sqrtf(Max(budget * budget - longForce * longForce, 0.0f));
                const float latForce = clamp(-vLat * massShare * invDt, -lateralBudget, lateralBudget);

                // longDir/latDir are unit directions and longForce/latForce are
                // N, so the applied vector is already box-space force (Unitless).
                b3Body_ApplyForce(
                    m_carBodyId, SourceToBox::Unitless(longDir * longForce + latDir * latForce), SourceToBox::Distance(contact),
                    false);
            }
        }

        if (m_pWheels[i])
        {
            m_pWheels[i]->SetPosition(wheelCenter, chassisAngles, true);
        }
    }

    m_currentState.steeringAngle = -RAD2DEG(steerAngle);
    m_currentState.wheelsInContact = wheelsInContact;
    m_currentState.wheelsNotInContact = m_wheelCount - wheelsInContact;
}
