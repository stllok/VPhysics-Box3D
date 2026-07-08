#include "vbox_controller_airboat.h"

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

// Yaw steering model: a bounded yaw impulse about the chassis up axis, ramped in
// over kAirboatSteeringInterval, opposed by rotational drag (yaw-rate^2) and
// damping (yaw-rate).
static constexpr float kAirboatSteeringRateMin{ 0.00045f };
static constexpr float kAirboatSteeringRateMax{ 5.0f * kAirboatSteeringRateMin };
static constexpr float kAirboatSteeringInterval{ 0.5f };

static constexpr float kAirboatRotDrag{ 0.00004f };
static constexpr float kAirboatRotDamping{ 0.001f };

// Fan thrust as a target acceleration (m/s^2); force = accel * mass.
static constexpr float kAirboatThrustMax{ 11.0f };
static constexpr float kAirboatThrustMaxReverse{ 7.5f };

// Directional water drag (quadratic in speed) mapped to chassis axes
// x = left/right, y = forward/back, z = up/down.
static constexpr float kAirboatWaterDragLeftRight{ 0.6f };
static constexpr float kAirboatWaterDragForwardBack{ 0.005f };
static constexpr float kAirboatWaterDragUpDown{ 0.0025f };

static constexpr float kAirboatGroundDragLeftRight{ 2.0f };
static constexpr float kAirboatGroundDragForwardBack{ 1.0f };
static constexpr float kAirboatGroundDragUpDown{ 0.8f };

static constexpr float kAirboatDryFrictionScale{ 0.6f };

static constexpr float kAirboatRaycastDist{ 0.35f };
static constexpr float kAirboatRaycastDistWaterLow{ 0.1f };
static constexpr float kAirboatRaycastDistWaterHigh{ 0.35f };

static constexpr float kAirboatWaterNoiseMin{ 0.01f };
static constexpr float kAirboatWaterNoiseMax{ 0.03f };
static constexpr float kAirboatWaterFreqMin{ 1.5f };
static constexpr float kAirboatWaterFreqMax{ 1.5f };
static constexpr float kAirboatWaterPhaseMin{ 0.0f };
static constexpr float kAirboatWaterPhaseMax{ 1.5f };

static constexpr float kAirboatGravity{ 9.81f };

static constexpr float kAirboatBuoyancyScalar{ 1.6f };
static constexpr float kAirboatPontoonArea2D{ 2.8f };
static constexpr float kAirboatPontoonHeight{ 0.41f };

static Vector LocalPointToWorldSrc(const b3WorldTransform& xf, const Vector& localSrc)
{
    return BoxToSource::Distance(b3TransformWorldPoint(xf, SourceToBox::Distance(localSrc)));
}

static Vector LocalDirToWorldSrc(const b3WorldTransform& xf, const Vector& localDir)
{
    return BoxToSource::Unitless(b3RotateVector(xf.q, SourceToBox::Unitless(localDir)));
}

static Vector WorldDirToLocalSrc(const b3WorldTransform& xf, const Vector& worldDir)
{
    return BoxToSource::Unitless(b3InvRotateVector(xf.q, SourceToBox::Unitless(worldDir)));
}

// A metric ray length along a Source-space unit direction (Distance converts m -> in).
static Vector RayEndpointSrc(const Vector& startSrc, const Vector& dirSrc, float lengthMetric)
{
    return startSrc + dirSrc * BoxToSource::Distance(lengthMetric);
}

Box3DVehicleAirboat::Box3DVehicleAirboat(
    const vehicleparams_t& params, Box3DPhysicsEnvironment* pEnv, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace)
    : Box3DVehicleController(params, pEnv, nVehicleType, pGameTrace)
{
    m_nAirboatPontoons = 0;
    std::fill_n(m_AirboatPontoons, kAirboatMaxPontoons, Box3DAirboatPontoon{});
    std::fill_n(m_AirboatImpacts, kAirboatMaxPontoons, Box3DAirboatImpact{});
    m_AirboatState = Box3DAirboatState{};
}

Box3DVehicleAirboat::~Box3DVehicleAirboat()
{
    if (m_hasSavedChassisState && m_pCarBody)
    {
        m_pCarBody->EnableGravity(m_savedChassisGravity);
        m_pCarBody->SetCallbackFlags(m_savedChassisCallbackFlags);
    }
}

float Box3DVehicleAirboat::UpdateBooster(float)
{
    return 0.0f;
}

// AttachWheels runs after the base created the (non-colliding) wheel proxy
// bodies and filled m_wheelPosition_Bs[]. Port of InitAirboat: build the pontoon
// setup from the axle/wheel params, reusing those attach positions, and disable
// box3d gravity so the hull runs its own gravity every step.
void Box3DVehicleAirboat::AttachWheels()
{
    const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : 0.0f;

    const int nAxles = Min(m_vehicleData.axleCount, 2);
    const int nWheelsPerAxle = Min(m_vehicleData.wheelsPerAxle, 2);
    m_nAirboatPontoons = Min(nAxles * nWheelsPerAxle, kAirboatMaxPontoons);

    for (int i = 0; i < m_nAirboatPontoons; ++i)
    {
        const int axleIdx = i / Max(m_vehicleData.wheelsPerAxle, 1);
        const vehicle_axleparams_t& axle = m_vehicleData.axles[axleIdx];

        Box3DAirboatPontoon& pontoon = m_AirboatPontoons[i];
        // Reuse the base's synthesized attach/trace positions (the HL2 airboat
        // model has degenerate wheel attachments the base fixes up).
        pontoon.hp_cs = m_wheelPosition_Bs[i];
        pontoon.raycast_start_cs = m_tracePosition_Bs[i];
        pontoon.raycast_dir_cs = Vector(0.0f, 0.0f, -1.0f);
        pontoon.raycast_length = kAirboatRaycastDist;
        pontoon.spring_constant = axle.suspension.springConstant * bodyMass;
        pontoon.spring_damp_relax = axle.suspension.springDamping * bodyMass;
        pontoon.spring_damp_compress = axle.suspension.springDampingCompression * bodyMass;
        pontoon.friction_of_wheel = 1.0f;
        pontoon.wheel_radius = SourceToBox::Distance(axle.wheels.radius);
        pontoon.raycast_dist = kAirboatRaycastDist;
        pontoon.wheel_is_fixed = true;
    }

    if (m_pCarBody)
    {
        if (!m_hasSavedChassisState)
        {
            m_savedChassisGravity = m_pCarBody->IsGravityEnabled();
            m_savedChassisCallbackFlags = m_pCarBody->GetCallbackFlags();
            m_hasSavedChassisState = true;
        }
        // The hull integrates its own gravity/buoyancy each step, so turn off
        // box3d gravity and the game's fluid simulation on it.
        m_pCarBody->EnableGravity(false);
        m_pCarBody->SetCallbackFlags(m_pCarBody->GetCallbackFlags() & ~CALLBACK_DO_FLUID_SIMULATION);
    }
}

void Box3DVehicleAirboat::Update(float, vehicle_controlparams_t& controls)
{
    m_controls = controls;
    Box3DAirboatState& state = m_AirboatState;

    if (B3_IS_NON_NULL(m_carBodyId) && b3Body_IsValid(m_carBodyId))
    {
        if (controls.steering != 0.0f || controls.throttle != 0.0f || fabsf(state.m_flThrust) > 0.01f)
            b3Body_SetAwake(m_carBodyId, true);
    }

    float flThrottle = controls.throttle;
    const float flAbsSpeed = fabsf(m_currentState.speed);
    const float flMaxSpeed = Max(m_engineMaxSpeed, 1.0f);
    if (flThrottle > 0.0f && flAbsSpeed > flMaxSpeed)
    {
        const float flFrac = flAbsSpeed / flMaxSpeed;
        if (flFrac > m_vehicleData.engine.autobrakeSpeedGain)
            flThrottle = 0.0f;
        flThrottle *= 0.1f;
    }
    if (flThrottle < 0.0f && flAbsSpeed > m_engineMaxRevSpeed)
        flThrottle *= 0.1f;

    if (fabsf(flThrottle) < 0.01f)
        state.m_flThrust = 0.0f;
    else if (flThrottle > 0.0f)
        state.m_flThrust = kAirboatThrustMax * flThrottle;
    else
        state.m_flThrust = kAirboatThrustMaxReverse * flThrottle;

    state.m_bAnalogSteering = controls.bAnalogSteering;
    state.m_SteeringAngle = DEG2RAD(
        controls.steering * Max(m_vehicleData.steering.degreesSlow, m_vehicleData.steering.degreesFast));
}

void Box3DVehicleAirboat::OnPreSimulate(float dt)
{
    if (B3_IS_NULL(m_carBodyId) || !b3Body_IsValid(m_carBodyId))
        return;

    AirboatOnPreSimulate(dt);

    // Operating params the game reads each tick. Speed uses the working forward
    // convention (chassis local +y) so the speedometer is not regressed.
    const b3Vec3 forward = b3Body_GetWorldVector(m_carBodyId, b3Vec3{ 0.0f, 1.0f, 0.0f });
    const float forwardSpeed = b3Dot(forward, b3Body_GetLinearVelocity(m_carBodyId));
    m_currentState.speed = BoxToSource::Distance(forwardSpeed);
    m_currentState.steeringAngle = RAD2DEG(m_AirboatState.m_SteeringAngle);
    m_currentState.wheelsInContact = AirboatCountSurfaceContacts(m_AirboatImpacts);
    m_currentState.wheelsNotInContact = m_nAirboatPontoons - m_currentState.wheelsInContact;
}

void Box3DVehicleAirboat::AirboatOnPreSimulate(float dt)
{
    if (dt <= 0.0f)
        return;

    // Own gravity: box3d gravity is disabled on the hull, apply it as a metric
    // impulse straight down every step.
    {
        const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : m_bodyMass;
        ApplyImpulseCenterMetric(Vector(0.0f, 0.0f, -kAirboatGravity * bodyMass * dt));
    }

    const b3WorldTransform xf = b3Body_GetTransform(m_carBodyId);

    const b3Vec3 linVel = b3Body_GetLinearVelocity(m_carBodyId);
    m_AirboatState.m_flSpeed = b3Length(linVel);
    m_AirboatState.m_vecLocalVelocity = BoxToSource::Unitless(b3InvRotateVector(xf.q, linVel));

    Box3DAirboatImpact* pImpacts = m_AirboatImpacts;
    for (int i = 0; i < m_nAirboatPontoons; ++i)
        pImpacts[i] = Box3DAirboatImpact{};

    AirboatPreRaycasts(pImpacts);
    AirboatDoRaycasts(pImpacts, xf);
    if (!AirboatPostRaycasts(pImpacts, xf))
        return;

    AirboatUpdateAirborne(pImpacts, dt);

    AirboatDoPontoons(pImpacts, dt);
    AirboatDoDrag(pImpacts, dt, xf);
    AirboatDoTurbine(dt, xf);
    AirboatDoSteering(dt, xf);
    AirboatDoKeepUprightPitch(pImpacts, dt, xf);
    AirboatDoKeepUprightRoll(pImpacts, dt, xf);
}

float Box3DVehicleAirboat::AirboatComputeFrontPontoonWaveNoise(int nIndex, float flSpeedRatio, float flCurrentTime) const
{
    const float flNoiseScale = RemapValClamped(1.0f - flSpeedRatio, 0.0f, 1.0f, kAirboatWaterNoiseMin, kAirboatWaterNoiseMax);

    float flPhaseShift = 0.0f;
    if (flSpeedRatio < 0.3f)
        flPhaseShift = float(nIndex) * kAirboatWaterPhaseMax;

    const float flFrequency = RemapValClamped(flSpeedRatio, 0.0f, 1.0f, kAirboatWaterFreqMin, kAirboatWaterFreqMax);
    return flNoiseScale * sinf(flFrequency * (flCurrentTime + flPhaseShift));
}

void Box3DVehicleAirboat::AirboatPreRaycasts(Box3DAirboatImpact*)
{
    for (int i = 0; i < m_nAirboatPontoons; ++i)
        m_AirboatPontoons[i].raycast_length = kAirboatRaycastDist;
}

void Box3DVehicleAirboat::AirboatDoRaycasts(Box3DAirboatImpact* pImpacts, const b3WorldTransform& xf)
{
    if (!m_pGameTrace)
        return;

    void* pVehicleGameData = m_pCarBody ? m_pCarBody->GetGameData() : nullptr;

    const float flForwardSpeedRatioRaw = clamp(m_AirboatState.m_vecLocalVelocity.y / 10.0f, 0.0f, 1.0f);
    const float flSpeedRatio = clamp(m_AirboatState.m_flSpeed / 15.0f, 0.0f, 1.0f);
    const float flForwardSpeedRatio = m_AirboatState.m_flThrust ? flForwardSpeedRatioRaw : flForwardSpeedRatioRaw * 0.5f;
    const float flCurrentTime = float(Plat_FloatTime());

    struct PontoonRayInfo
    {
        Vector start;
        Vector dirWorldSrc;
        float lengthMetric;
    };
    PontoonRayInfo rayInfo[kAirboatMaxPontoons];

    int nFrontPontoonsInWater = 0;
    for (int i = 0; i < m_nAirboatPontoons; ++i)
    {
        Box3DAirboatPontoon& pontoon = m_AirboatPontoons[i];

        const Vector startWorldSrc = LocalPointToWorldSrc(xf, pontoon.raycast_start_cs);
        Vector dirWorldSrc = LocalDirToWorldSrc(xf, pontoon.raycast_dir_cs);

        pImpacts[i].raycast_dir_ws = dirWorldSrc;
        pImpacts[i].bInWater = m_pGameTrace->VehiclePointInWater(startWorldSrc);
        if (pImpacts[i].bInWater)
            dirWorldSrc.Negate();

        float lengthMetric = pontoon.raycast_length;
        Vector endWorldSrc = RayEndpointSrc(startWorldSrc, dirWorldSrc, lengthMetric);

        if (m_pGameTrace->VehiclePointInWater(endWorldSrc))
        {
            lengthMetric = kAirboatRaycastDistWaterLow;
            if (i < 2)
            {
                ++nFrontPontoonsInWater;
                lengthMetric += AirboatComputeFrontPontoonWaveNoise(i, flSpeedRatio, flCurrentTime);
            }
            endWorldSrc = RayEndpointSrc(startWorldSrc, dirWorldSrc, lengthMetric);
        }

        rayInfo[i].start = startWorldSrc;
        rayInfo[i].dirWorldSrc = dirWorldSrc;
        rayInfo[i].lengthMetric = lengthMetric;
    }

    if (nFrontPontoonsInWater == 2)
    {
        for (int i = 0; i < 2; ++i)
        {
            float lengthMetric = RemapValClamped(
                flForwardSpeedRatio, 0.0f, 1.0f, kAirboatRaycastDistWaterLow, kAirboatRaycastDistWaterHigh);
            lengthMetric += AirboatComputeFrontPontoonWaveNoise(i, flSpeedRatio, flCurrentTime);
            rayInfo[i].lengthMetric = lengthMetric;
        }
    }

    trace_t trace;
    for (int i = 0; i < m_nAirboatPontoons; ++i)
    {
        Box3DAirboatPontoon& pontoon = m_AirboatPontoons[i];
        Box3DAirboatImpact& impact = pImpacts[i];

        pontoon.raycast_length = rayInfo[i].lengthMetric;

        const Vector endWorldSrc = RayEndpointSrc(rayInfo[i].start, rayInfo[i].dirWorldSrc, rayInfo[i].lengthMetric);

        Ray_t ray;
        ray.Init(rayInfo[i].start, endWorldSrc);

        if (impact.bInWater)
        {
            m_pGameTrace->VehicleTraceRay(ray, pVehicleGameData, &trace);

            // Water depth from an upward trace: fractionleftsolid * ray length
            // (Source inches), like the base's WaterDepthAt.
            Ray_t waterRay;
            Vector vecUp = rayInfo[i].start;
            vecUp.z += 1000.0f;
            waterRay.Init(rayInfo[i].start, vecUp);

            trace_t waterTrace;
            m_pGameTrace->VehicleTraceRayWithWater(waterRay, pVehicleGameData, &waterTrace);
            impact.flDepth = 1000.0f * waterTrace.fractionleftsolid;
        }
        else
        {
            m_pGameTrace->VehicleTraceRayWithWater(ray, pVehicleGameData, &trace);
        }

        impact.bImpact = false;
        impact.bImpactWater = false;

        if (trace.fraction != 1.0f)
        {
            impact.bImpact = true;
            impact.flDepth = 0.0f;
            if (trace.contents & MASK_WATER)
                impact.bImpactWater = true;

            impact.vecImpactPointWS = trace.endpos;
            impact.vecImpactNormalWS = trace.plane.normal;
            impact.nSurfaceProps = trace.surface.surfaceProps;

            const surfacedata_t* pSurface = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(trace.surface.surfaceProps);
            if (pSurface)
            {
                impact.flDampening = pSurface->physics.dampening;
                impact.flFriction = pSurface->physics.friction;
            }
        }
    }
}

bool Box3DVehicleAirboat::AirboatPostRaycasts(Box3DAirboatImpact* pImpacts, const b3WorldTransform& xf)
{
    for (int i = 0; i < m_nAirboatPontoons; ++i)
    {
        Box3DAirboatPontoon& pontoon = m_AirboatPontoons[i];
        Box3DAirboatImpact& impact = pImpacts[i];

        if (impact.bInWater)
            impact.raycast_dir_ws.Negate();

        const Vector startWorldSrc = LocalPointToWorldSrc(xf, pontoon.raycast_start_cs);

        if (impact.bImpact)
        {
            const Vector deltaSrc = impact.vecImpactPointWS - startWorldSrc;
            pontoon.raycast_dist = SourceToBox::Distance(deltaSrc.Length());

            const float dotND = fabsf(impact.raycast_dir_ws.Dot(impact.vecImpactNormalWS));
            impact.inv_normal_dot_dir = 1.1f / (dotND + 0.1f);
            impact.friction_value = impact.flFriction * pontoon.friction_of_wheel;
        }
        else
        {
            pontoon.raycast_dist = pontoon.raycast_length;
            impact.inv_normal_dot_dir = 1.0f;
            impact.vecImpactNormalWS = -impact.raycast_dir_ws;
            impact.friction_value = 1.0f;
            impact.vecImpactPointWS = RayEndpointSrc(startWorldSrc, impact.raycast_dir_ws, pontoon.raycast_dist);
        }

        const b3Vec3 surfaceSpeedMetric = b3Body_GetWorldPointVelocity(
            m_carBodyId, SourceToBox::Distance(impact.vecImpactPointWS));
        impact.surface_speed_wheel_ws = BoxToSource::Unitless(surfaceSpeedMetric);

        const Vector n = impact.vecImpactNormalWS;
        const float ns = n.Dot(impact.surface_speed_wheel_ws);
        impact.projected_surface_speed_wheel_ws = impact.surface_speed_wheel_ws - n * ns;
    }

    return true;
}

int Box3DVehicleAirboat::AirboatCountSurfaceContacts(const Box3DAirboatImpact* pImpacts) const
{
    int nContacts = 0;
    for (int i = 0; i < m_nAirboatPontoons; ++i)
    {
        if (pImpacts[i].bImpact)
            ++nContacts;
    }
    return nContacts;
}

void Box3DVehicleAirboat::AirboatUpdateAirborne(Box3DAirboatImpact* pImpacts, float dt)
{
    Box3DAirboatState& state = m_AirboatState;
    const int nCount = AirboatCountSurfaceContacts(pImpacts);
    if (!nCount)
    {
        if (!state.m_bAirborne)
        {
            state.m_bAirborne = true;
            state.m_flAirTime = 0.0f;
            if (state.m_flSpeed < 11.0f)
                state.m_bWeakJump = true;
        }
        else
        {
            state.m_flAirTime += dt;
        }
    }
    else
    {
        state.m_bAirborne = false;
        state.m_bWeakJump = false;
    }
}

void Box3DVehicleAirboat::AirboatDoPontoons(Box3DAirboatImpact* pImpacts, float dt)
{
    for (int i = 0; i < m_nAirboatPontoons; ++i)
    {
        Box3DAirboatPontoon& pontoon = m_AirboatPontoons[i];
        Box3DAirboatImpact& impact = pImpacts[i];

        if (impact.bImpact)
            AirboatDoPontoonGround(&pontoon, &impact, dt);
        else if (impact.bInWater)
            AirboatDoPontoonWater(&pontoon, &impact, dt);
    }
}

void Box3DVehicleAirboat::AirboatDoPontoonGround(Box3DAirboatPontoon* pPontoon, Box3DAirboatImpact* pImpact, float dt)
{
    const float flDiff = pPontoon->raycast_dist - pPontoon->raycast_length;
    if (flDiff >= 0.0f)
        return;

    float flForce = -flDiff * pPontoon->spring_constant;
    const float flInvNormalDotDir = clamp(pImpact->inv_normal_dot_dir, 0.0f, 3.0f);
    flForce *= flInvNormalDotDir;

    const Vector vecSpeedDelta = pImpact->projected_surface_speed_wheel_ws - pImpact->surface_speed_wheel_ws;
    const float flSpeed = vecSpeedDelta.Dot(pImpact->raycast_dir_ws);
    if (flSpeed > 0.0f)
        flForce -= pPontoon->spring_damp_relax * flSpeed;
    else
        flForce -= pPontoon->spring_damp_compress * flSpeed;

    if (flForce < 0.0f)
        flForce = 0.0f;

    const float flImpulse = flForce * dt;
    const Vector vImpulseWS = pImpact->vecImpactNormalWS * flImpulse;

    ApplyImpulseAtPointMetric(vImpulseWS, pImpact->vecImpactPointWS * SourceToBox::Factor);
}

void Box3DVehicleAirboat::AirboatDoPontoonWater(Box3DAirboatPontoon*, Box3DAirboatImpact* pImpact, float dt)
{
    const float flDepthMetric = clamp(SourceToBox::Distance(pImpact->flDepth), 0.0f, kAirboatPontoonHeight);

    const float flSubmergedVolume = kAirboatPontoonArea2D * flDepthMetric;
    const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : m_bodyMass;
    const float flForce = kAirboatBuoyancyScalar * 0.25f * bodyMass * flSubmergedVolume * 1000.0f;
    const float flImpulse = flForce * dt;

    const Vector vImpulseWS(0.0f, 0.0f, flImpulse);
    ApplyImpulseAtPointMetric(vImpulseWS, pImpact->vecImpactPointWS * SourceToBox::Factor);
}

void Box3DVehicleAirboat::AirboatDoDrag(Box3DAirboatImpact* pImpacts, float dt, const b3WorldTransform& xf)
{
    const float flSpeed = m_AirboatState.m_flSpeed;

    int nPointsInWater = 0;
    int nPointsOnGround = 0;
    float flGroundFriction = 0.0f;
    for (int i = 0; i < m_nAirboatPontoons; ++i)
    {
        const Box3DAirboatImpact& impact = pImpacts[i];
        if (!impact.bImpact)
            continue;

        if (impact.bImpactWater)
        {
            ++nPointsInWater;
        }
        else
        {
            ++nPointsOnGround;
            flGroundFriction += impact.flFriction;
        }
    }

    const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : m_bodyMass;

    if (nPointsInWater)
    {
        const Vector negDirLS = -m_AirboatState.m_vecLocalVelocity;

        Vector vDragLS(
            kAirboatWaterDragLeftRight * negDirLS.x, kAirboatWaterDragForwardBack * negDirLS.y,
            kAirboatWaterDragUpDown * negDirLS.z);

        vDragLS *= flSpeed * bodyMass * dt;

        const Vector vDragWS = LocalDirToWorldSrc(xf, vDragLS);
        ApplyImpulseCenterMetric(vDragWS);
    }

    if (nPointsOnGround && flSpeed > 0.0f)
    {
        flGroundFriction /= float(nPointsOnGround);

        float flFrictionDrag = bodyMass * kAirboatGravity * kAirboatDryFrictionScale * flGroundFriction;
        flFrictionDrag /= flSpeed;

        const Vector negDirLS = -m_AirboatState.m_vecLocalVelocity;
        Vector vDragLS(
            kAirboatGroundDragLeftRight * negDirLS.x, kAirboatGroundDragForwardBack * negDirLS.y,
            kAirboatGroundDragUpDown * negDirLS.z);

        vDragLS *= flFrictionDrag * dt;

        const Vector vDragWS = LocalDirToWorldSrc(xf, vDragLS);
        ApplyImpulseCenterMetric(vDragWS);
    }
}

void Box3DVehicleAirboat::AirboatDoTurbine(float dt, const b3WorldTransform& xf)
{
    float flThrust = m_AirboatState.m_flThrust;
    if (m_AirboatState.m_bWeakJump || (m_AirboatState.m_bAirborne && flThrust < 0.0f))
        flThrust *= 0.5f;

    const Vector vForwardWS = LocalDirToWorldSrc(xf, Vector(0.0f, 1.0f, 0.0f));

    if (vForwardWS.z > 0.5f && flThrust > 0.0f)
    {
        const float flFactor = 1.0f - vForwardWS.z;
        flThrust *= flFactor;
    }
    else if (vForwardWS.z < -0.5f && flThrust < 0.0f)
    {
        const float flFactor = 1.0f + vForwardWS.z;
        flThrust *= flFactor;
    }

    const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : m_bodyMass;
    const Vector vImpulse = vForwardWS * (flThrust * bodyMass * dt);
    ApplyImpulseCenterMetric(vImpulse);
}

void Box3DVehicleAirboat::AirboatDoSteering(float dt, const b3WorldTransform& xf)
{
    Box3DAirboatState& state = m_AirboatState;

    if (state.m_SteeringAngle == 0.0f || state.m_flThrust != 0.0f)
    {
        if (!state.m_bAnalogSteering)
        {
            if (state.m_flThrust < 0.0f)
                state.m_bSteeringReversed = true;
            else if (state.m_flThrust > 0.0f || state.m_vecLocalVelocity.y > 0.0f)
                state.m_bSteeringReversed = false;
        }
        else
        {
            if (state.m_flThrust < -2.0f)
                state.m_bSteeringReversed = true;
            else if (state.m_flThrust > 2.0f || state.m_vecLocalVelocity.y > 0.0f)
                state.m_bSteeringReversed = false;
        }
    }

    const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : m_bodyMass;
    const float invDt = (dt > 0.0f) ? (1.0f / dt) : 0.0f;

    float flForceSteering = 0.0f;
    if (fabsf(state.m_SteeringAngle) > 0.01f)
    {
        float flSteeringSign = state.m_SteeringAngle < 0.0f ? -1.0f : 1.0f;
        if (state.m_bSteeringReversed)
            flSteeringSign *= -1.0f;

        const float flPrevSign = state.m_flPrevSteeringAngle < 0.0f ? -1.0f : 1.0f;
        if (fabsf(state.m_flPrevSteeringAngle) < 0.01f || flSteeringSign != flPrevSign)
            state.m_flSteerTime = 0.0f;

        float flSteerScale;
        if (!state.m_bAnalogSteering)
            flSteerScale = RemapValClamped(
                state.m_flSteerTime, 0.0f, kAirboatSteeringInterval, kAirboatSteeringRateMin, kAirboatSteeringRateMax);
        else
            flSteerScale = RemapValClamped(
                fabsf(state.m_SteeringAngle), 0.0f, kAirboatSteeringInterval, kAirboatSteeringRateMin, kAirboatSteeringRateMax);

        flForceSteering = flSteerScale * bodyMass * invDt;
        flForceSteering *= -flSteeringSign;

        state.m_flSteerTime += dt;
    }

    state.m_flPrevSteeringAngle = state.m_SteeringAngle * (state.m_bSteeringReversed ? -1.0f : 1.0f);

    // Yaw rate about the chassis up axis (local z).
    const Vector vAngLocal = WorldDirToLocalSrc(xf, BoxToSource::Unitless(b3Body_GetAngularVelocity(m_carBodyId)));
    const float yawRate = -vAngLocal.z;
    const float yawSign = yawRate < 0.0f ? -1.0f : 1.0f;

    const float flRotDrag = kAirboatRotDrag * yawRate * yawRate * bodyMass * invDt * yawSign;
    const float flRotDamp = kAirboatRotDamping * fabsf(yawRate) * bodyMass * invDt * yawSign;

    const float flForceRotational = flForceSteering + flRotDrag + flRotDamp;

    const Vector vAngImpWorld = LocalDirToWorldSrc(xf, Vector(0.0f, 0.0f, flForceRotational));
    ApplyAngularImpulseMetric(vAngImpWorld);
}

void Box3DVehicleAirboat::AirboatDoKeepUprightPitch(Box3DAirboatImpact* pImpacts, float dt, const b3WorldTransform& xf)
{
    Box3DAirboatState& state = m_AirboatState;
    if (state.m_bWeakJump)
        return;
    if (dt <= 0.0f)
        return;

    const float kCos10 = cosf(DEG2RAD(10.0f));
    const float kSin10 = sinf(DEG2RAD(10.0f));
    const Vector vUpCS(0.0f, kSin10, kCos10);

    const Vector vGoalAxisWS(0.0f, 0.0f, 1.0f);
    Vector vGoalAxisCS = WorldDirToLocalSrc(xf, vGoalAxisWS);
    vGoalAxisCS.x = vUpCS.x;
    VectorNormalize(vGoalAxisCS);

    Vector vRotAxisCS = vUpCS.Cross(vGoalAxisCS);
    const float cosine = vUpCS.Dot(vGoalAxisCS);
    const float sine = VectorNormalize(vRotAxisCS);
    const float angle = atan2f(sine, cosine);

    if (AirboatCountSurfaceContacts(pImpacts) > 0)
    {
        state.m_flPitchErrorPrev = angle;
        return;
    }

    const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : m_bodyMass;
    const float invDt = 1.0f / dt;
    Vector vAngImpCS = vRotAxisCS * (bodyMass * (0.1f * angle + 0.04f * invDt * (angle - state.m_flPitchErrorPrev)));
    state.m_flPitchErrorPrev = angle;

    float len = VectorNormalize(vAngImpCS);
    const float maxLen = DEG2RAD(1.5f) * bodyMass;
    if (len > maxLen)
        len = maxLen;
    vAngImpCS *= len;

    const Vector vAngImpWS = LocalDirToWorldSrc(xf, vAngImpCS);
    ApplyAngularImpulseMetric(vAngImpWS);
}

void Box3DVehicleAirboat::AirboatDoKeepUprightRoll(Box3DAirboatImpact* pImpacts, float dt, const b3WorldTransform& xf)
{
    Box3DAirboatState& state = m_AirboatState;
    if (dt <= 0.0f)
        return;

    const float kCos10 = cosf(DEG2RAD(10.0f));
    const float kSin10 = sinf(DEG2RAD(10.0f));
    const Vector vUpCS(0.0f, kSin10, kCos10);

    const Vector vGoalAxisWS(0.0f, 0.0f, 1.0f);
    Vector vGoalAxisCS = WorldDirToLocalSrc(xf, vGoalAxisWS);
    vGoalAxisCS.z = vUpCS.z;
    VectorNormalize(vGoalAxisCS);

    Vector vRotAxisCS = vUpCS.Cross(vGoalAxisCS);
    const float cosine = vUpCS.Dot(vGoalAxisCS);
    const float sine = VectorNormalize(vRotAxisCS);
    const float angle = atan2f(sine, cosine);

    if (AirboatCountSurfaceContacts(pImpacts) > 0)
    {
        state.m_flRollErrorPrev = angle;
        return;
    }

    if (fabsf(angle) < DEG2RAD(10.0f))
    {
        state.m_flRollErrorPrev = angle;
        return;
    }

    const float bodyMass = m_pCarBody ? m_pCarBody->GetMass() : m_bodyMass;
    const float invDt = 1.0f / dt;
    Vector vAngImpCS = vRotAxisCS * (bodyMass * (0.2f * angle + 0.3f * invDt * (angle - state.m_flRollErrorPrev)));
    state.m_flRollErrorPrev = angle;

    float len = VectorNormalize(vAngImpCS);
    const float maxLen = DEG2RAD(2.0f) * bodyMass;
    if (len > maxLen)
        len = maxLen;
    vAngImpCS *= len;

    const Vector vAngImpWS = LocalDirToWorldSrc(xf, vAngImpCS);
    ApplyAngularImpulseMetric(vAngImpWS);
}

void Box3DVehicleAirboat::ApplyImpulseAtPointMetric(const Vector& vImpulseMetric, const Vector& vWorldPosMetric)
{
    if (B3_IS_NULL(m_carBodyId) || !b3Body_IsValid(m_carBodyId))
        return;
    b3Body_ApplyLinearImpulse(
        m_carBodyId, b3Vec3{ vImpulseMetric.x, vImpulseMetric.y, vImpulseMetric.z },
        b3Pos{ vWorldPosMetric.x, vWorldPosMetric.y, vWorldPosMetric.z }, true);
}

void Box3DVehicleAirboat::ApplyImpulseCenterMetric(const Vector& vImpulseMetric)
{
    if (B3_IS_NULL(m_carBodyId) || !b3Body_IsValid(m_carBodyId))
        return;
    b3Body_ApplyLinearImpulseToCenter(m_carBodyId, b3Vec3{ vImpulseMetric.x, vImpulseMetric.y, vImpulseMetric.z }, true);
}

void Box3DVehicleAirboat::ApplyAngularImpulseMetric(const Vector& vAngularImpulseMetric)
{
    if (B3_IS_NULL(m_carBodyId) || !b3Body_IsValid(m_carBodyId))
        return;
    b3Body_ApplyAngularImpulse(
        m_carBodyId, b3Vec3{ vAngularImpulseMetric.x, vAngularImpulseMetric.y, vAngularImpulseMetric.z }, true);
}
