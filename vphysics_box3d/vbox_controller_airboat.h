#pragma once

#include "vbox_controller_vehicle.h"

static constexpr int kAirboatMaxPontoons = 4;

struct Box3DAirboatPontoon
{
    Vector hp_cs{ 0.0f, 0.0f, 0.0f };
    Vector raycast_start_cs{ 0.0f, 0.0f, 0.0f };
    Vector raycast_dir_cs{ 0.0f, 0.0f, -1.0f };
    float raycast_length = 0.0f;
    float spring_constant = 0.0f;
    float spring_damp_relax = 0.0f;
    float spring_damp_compress = 0.0f;
    float friction_of_wheel = 1.0f;
    float wheel_radius = 0.0f;
    float raycast_dist = 0.0f;
    bool wheel_is_fixed = true;
};

struct Box3DAirboatImpact
{
    bool bImpact = false;
    bool bImpactWater = false;
    bool bInWater = false;
    Vector vecImpactPointWS{ 0.0f, 0.0f, 0.0f };
    Vector vecImpactNormalWS{ 0.0f, 0.0f, 0.0f };
    Vector raycast_dir_ws{ 0.0f, 0.0f, 0.0f };
    Vector surface_speed_wheel_ws{ 0.0f, 0.0f, 0.0f };
    Vector projected_surface_speed_wheel_ws{ 0.0f, 0.0f, 0.0f };
    float flDepth = 0.0f;
    float flFriction = 1.0f;
    float flDampening = 0.0f;
    float friction_value = 1.0f;
    float inv_normal_dot_dir = 1.0f;
    int nSurfaceProps = 0;
};

struct Box3DAirboatState
{
    float m_flThrust = 0.0f;
    float m_SteeringAngle = 0.0f;
    bool m_bAnalogSteering = false;
    bool m_bSteeringReversed = false;
    float m_flPrevSteeringAngle = 0.0f;
    float m_flSteerTime = 0.0f;
    bool m_bAirborne = false;
    float m_flAirTime = 0.0f;
    bool m_bWeakJump = false;
    float m_flPitchErrorPrev = 0.0f;
    float m_flRollErrorPrev = 0.0f;
    float m_flSpeed = 0.0f;
    Vector m_vecLocalVelocity{ 0.0f, 0.0f, 0.0f };
};

class Box3DVehicleAirboat final : public Box3DVehicleController
{
public:
    Box3DVehicleAirboat(
        const vehicleparams_t& params, Box3DPhysicsEnvironment* pEnv, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace);
    ~Box3DVehicleAirboat() override;

    void OnPreSimulate(float dt) override;

    // IPhysicsVehicleController
    void Update(float dt, vehicle_controlparams_t& controls) override;
    float UpdateBooster(float dt) override;

private:
    void AttachWheels() override;

    // Airboat simulation (ports of VPhysics-Jolt's Airboat* helpers).
    void AirboatOnPreSimulate(float dt);
    void AirboatPreRaycasts(Box3DAirboatImpact* pImpacts);
    void AirboatDoRaycasts(Box3DAirboatImpact* pImpacts, const b3WorldTransform& xf);
    bool AirboatPostRaycasts(Box3DAirboatImpact* pImpacts, const b3WorldTransform& xf);
    void AirboatUpdateAirborne(Box3DAirboatImpact* pImpacts, float dt);
    int AirboatCountSurfaceContacts(const Box3DAirboatImpact* pImpacts) const;
    float AirboatComputeFrontPontoonWaveNoise(int nIndex, float flSpeedRatio, float flCurrentTime) const;
    void AirboatDoPontoons(Box3DAirboatImpact* pImpacts, float dt);
    void AirboatDoPontoonGround(Box3DAirboatPontoon* pPontoon, Box3DAirboatImpact* pImpact, float dt);
    void AirboatDoPontoonWater(Box3DAirboatPontoon* pPontoon, Box3DAirboatImpact* pImpact, float dt);
    void AirboatDoDrag(Box3DAirboatImpact* pImpacts, float dt, const b3WorldTransform& xf);
    void AirboatDoTurbine(float dt, const b3WorldTransform& xf);
    void AirboatDoSteering(float dt, const b3WorldTransform& xf);
    void AirboatDoKeepUprightPitch(Box3DAirboatImpact* pImpacts, float dt, const b3WorldTransform& xf);
    void AirboatDoKeepUprightRoll(Box3DAirboatImpact* pImpacts, float dt, const b3WorldTransform& xf);

    // Metric (metres-space) impulse application onto the chassis body.
    void ApplyImpulseAtPointMetric(const Vector& vImpulseMetric, const Vector& vWorldPosMetric);
    void ApplyImpulseCenterMetric(const Vector& vImpulseMetric);
    void ApplyAngularImpulseMetric(const Vector& vAngularImpulseMetric);

    int m_nAirboatPontoons;
    Box3DAirboatPontoon m_AirboatPontoons[kAirboatMaxPontoons];
    Box3DAirboatImpact m_AirboatImpacts[kAirboatMaxPontoons];
    Box3DAirboatState m_AirboatState;
    unsigned short m_savedChassisCallbackFlags = 0;
    bool m_savedChassisGravity = true;
    bool m_hasSavedChassisState = false;
};
