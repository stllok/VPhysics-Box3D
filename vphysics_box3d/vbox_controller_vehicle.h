
#pragma once

#include "vbox_interface.h"
#include "vphysics/vehicles.h"

class Box3DPhysicsObject;
class Box3DPhysicsEnvironment;
class IPhysicsGameTrace;

struct RaycastWheelContact
{
    Vector point{ 0.0f, 0.0f, 0.0f };
    Vector normal{ 0.0f, 0.0f, 0.0f };
    int surfaceProps{ 0 };
    float suspensionForce{ 0.0f };
    bool inContact{ false };
    bool inWater{ false };
};

class Box3DVehicleController : public IPhysicsVehicleController
{
public:
    ~Box3DVehicleController() override;

    void InitCarSystem(Box3DPhysicsObject* pBodyObject);

    virtual void OnPreSimulate(float dt) = 0;

    virtual void OnObjectDestroyed(Box3DPhysicsObject* pObject);

    // IPhysicsVehicleController
    void Update(float dt, vehicle_controlparams_t& controls) override = 0;
    float UpdateBooster(float dt) override = 0;
    void SetSpringLength(int wheelIndex, float length) override;
    const vehicle_operatingparams_t& GetOperatingParams() override
    {
        return m_currentState;
    }
    const vehicleparams_t& GetVehicleParams() override
    {
        return m_vehicleData;
    }
    vehicleparams_t& GetVehicleParamsForChange() override
    {
        return m_vehicleData;
    }
    int GetWheelCount(void) override
    {
        return m_wheelCount;
    }
    IPhysicsObject* GetWheel(int index) override;
    bool GetWheelContactPoint(int index, Vector* pContactPoint, int* pSurfaceProps) override;
    void SetWheelFriction(int wheelIndex, float friction) override;

    void SetEngineDisabled(bool bDisable) override
    {
        m_bEngineDisable = bDisable;
    }
    bool IsEngineDisabled(void) override
    {
        return m_bEngineDisable;
    }

    void VehicleDataReload() override;

    // Debug
    void GetCarSystemDebugData(vehicle_debugcarsystem_t& debugCarSystem) override;

    // Entry/Exit
    void OnVehicleEnter(void) override;
    void OnVehicleExit(void) override;

protected:
    Box3DVehicleController(
        const vehicleparams_t& params, Box3DPhysicsEnvironment* pEnv, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace);

    void InitVehicleData(const vehicleparams_t& params);

    void ResetState();
    [[nodiscard]] bool IsRaycastVehicle() const
    {
        return m_nVehicleType == VEHICLE_TYPE_CAR_RAYCAST || m_nVehicleType == VEHICLE_TYPE_JETSKI_RAYCAST
            || m_nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST;
    }
    bool FixDegenerateWheelPositions(Vector positions[VEHICLE_MAX_WHEEL_COUNT]);
    Box3DPhysicsObject* CreateWheel(int wheelIndex, const Vector& wheelPositionLocal);
    // Attach the type-specific per-wheel structures (joints/pontoons) once the
    // wheel bodies exist.
    virtual void AttachWheels() = 0;
    void CastWheel(
        const Vector& start, const Vector& dirDown, float castDist, float* pHitDist, Vector* pNormal, int* pSurfaceProps,
        bool* pInWater) const;
    // Submersion depth (Source inches) of a point below the water surface, via an
    // upward water trace. 0 if not underwater. Used for airboat buoyancy.
    [[nodiscard]] float WaterDepthAt(const Vector& worldPos) const;
    [[nodiscard]] float MaxSteeringAngleRad() const;
    [[nodiscard]] bool WheelContact(int wheelIndex, b3Pos* pPoint, int* pSurfaceProps) const;

    Box3DPhysicsObject* m_pCarBody;
    Box3DPhysicsEnvironment* m_pEnv;
    IPhysicsGameTrace* m_pGameTrace;
    int m_wheelCount;
    vehicleparams_t m_vehicleData;
    vehicle_operatingparams_t m_currentState;
    float m_wheelRadius;
    float m_bodyMass;
    float m_totalWheelMass;
    float m_gravityLength;
    float m_torqueScale;
    float m_engineMaxSpeed;
    float m_engineMaxRevSpeed;
    float m_engineBoostMaxSpeed;
    Box3DPhysicsObject* m_pWheels[VEHICLE_MAX_WHEEL_COUNT];
    Vector m_wheelPosition_Bs[VEHICLE_MAX_WHEEL_COUNT];
    Vector m_tracePosition_Bs[VEHICLE_MAX_WHEEL_COUNT];
    int m_vehicleFlags;
    unsigned int m_nTireType;
    unsigned int m_nVehicleType;
    bool m_bTraceData;
    bool m_bOccupied;
    bool m_bEngineDisable;

    b3BodyId m_carBodyId;
    RaycastWheelContact m_raycastContacts[VEHICLE_MAX_WHEEL_COUNT];
    float m_wheelFrictionOverride[VEHICLE_MAX_WHEEL_COUNT];
    vehicle_controlparams_t m_controls;
};
