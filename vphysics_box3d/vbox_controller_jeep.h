#pragma once

#include "vbox_controller_vehicle.h"

class Box3DVehicleJeep final : public Box3DVehicleController
{
public:
    Box3DVehicleJeep(
        const vehicleparams_t& params, Box3DPhysicsEnvironment* pEnv, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace);
    ~Box3DVehicleJeep() override;

    void OnPreSimulate(float dt) override;
    void OnObjectDestroyed(Box3DPhysicsObject* pObject) override;

    // IPhysicsVehicleController
    void Update(float dt, vehicle_controlparams_t& controls) override;
    float UpdateBooster(float dt) override;
    void SetSpringLength(int wheelIndex, float length) override;

private:
    struct DriverInputs
    {
        float throttle;
        float brake;
        bool handbrake;
        bool boosting;
        float torqueMultiplier;
    };

    void AttachWheels() override;
    void CreateWheelJoint(int wheelIndex);
    void HandleBoostKey();
    void HandleBoostDecay();
    DriverInputs ProcessDriverInputs(float speed);
    [[nodiscard]] float GearRatioSafe(int gear) const;
    float UpdateTransmission(float avgSpin, bool boosting);
    [[nodiscard]] float EngineTorqueAtRedline() const;
    void SimulateWheeled(float dt, float speed, const DriverInputs& inputs);
    void SimulateRaycast(float dt, float speed, const DriverInputs& inputs);
    [[nodiscard]] float SuspensionTranslation(int wheelIndex) const;

    b3JointId m_wheelJoints[VEHICLE_MAX_WHEEL_COUNT];
    float m_flBoostDelay;
    float m_flBoosterRemaining;
    int m_gear;
};
