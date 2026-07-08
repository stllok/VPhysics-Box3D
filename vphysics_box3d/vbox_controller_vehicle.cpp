#include "vbox_controller_vehicle.h"

#include "cbase.h"
#include "vbox_collide.h"
#include "vbox_environment.h"
#include "vbox_object.h"
#include "vbox_surfaceprops.h"

#include "box3d/collision.h"

#include <algorithm>
#include <cmath>
#include <cstring>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static constexpr float kMphToInchesPerSecond{ 5280.0f * 12.0f / 3600.0f };
static constexpr float kWheelRadiusMin{ 1.0f };
static constexpr float kWheelCoincidentDist{ 2.0f };

// Local-space AABB of a collide, returned in Source inches.
static bool LocalCollideBounds(const CPhysCollide* pCollide, Vector* pMins, Vector* pMaxs)
{
    if (!pCollide)
        return false;

    bool any = false;
    b3AABB total{};
    for (int i = 0; i < pCollide->m_Convexes.Count(); ++i)
    {
        const CPhysConvex* pConvex = pCollide->m_Convexes[i];
        if (!pConvex)
            continue;
        const b3HullData* pHull = const_cast<CPhysConvex*>(pConvex)->GetSimHull();
        if (!pHull)
            continue;
        b3AABB box = b3ComputeHullAABB(pHull, b3Transform_identity);
        total = any ? b3AABB_Union(total, box) : box;
        any = true;
    }
    if (!any)
        return false;

    // AABB bounds are box metres -> Source inches.
    *pMins = BoxToSource::Distance(total.lowerBound);
    *pMaxs = BoxToSource::Distance(total.upperBound);
    return true;
}

static int ShapeMaterialIdAtTriangle(b3ShapeId shape, int triangleIndex)
{
    const b3ShapeType type = b3Shape_GetType(shape);
    int materialIndex = 0;

    if (type == b3_meshShape && triangleIndex >= 0)
    {
        const b3Mesh mesh = b3Shape_GetMesh(shape);
        const uint8* pIndices = mesh.data ? b3GetMeshMaterialIndices(mesh.data) : nullptr;
        if (pIndices && triangleIndex < mesh.data->triangleCount)
            materialIndex = pIndices[triangleIndex];
    }
    else if (type == b3_heightShape && triangleIndex >= 0)
    {
        const b3HeightFieldData* pHeightField = b3Shape_GetHeightField(shape);
        const uint8* pIndices = pHeightField ? b3GetHeightFieldMaterialIndices(pHeightField) : nullptr;
        const int nCellCount = pHeightField ? (pHeightField->columnCount - 1) * (pHeightField->rowCount - 1) : 0;
        const int nCellIndex = triangleIndex >> 1;
        if (pIndices && nCellIndex >= 0 && nCellIndex < nCellCount)
            materialIndex = pIndices[nCellIndex];
    }

    const int nMaterialCount = b3Shape_GetMeshMaterialCount(shape);
    if (nMaterialCount <= 0)
        return static_cast<int>(b3Shape_GetSurfaceMaterial(shape).userMaterialId);
    materialIndex = clamp(materialIndex, 0, nMaterialCount - 1);
    return static_cast<int>(b3Shape_GetMeshSurfaceMaterial(shape, materialIndex).userMaterialId);
}

Box3DVehicleController::Box3DVehicleController(
    const vehicleparams_t& params, Box3DPhysicsEnvironment* pEnv, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace)
{
    m_pEnv = pEnv;
    m_pGameTrace = pGameTrace;
    m_nVehicleType = nVehicleType;
    Box3DVehicleController::InitVehicleData(params);
    ResetState();
}

void Box3DVehicleController::ResetState()
{
    std::fill_n(m_pWheels, VEHICLE_MAX_WHEEL_COUNT, nullptr);
    m_pCarBody = nullptr;
    m_torqueScale = 1;
    m_wheelCount = 0;
    m_wheelRadius = 0;
    m_currentState = {};
    m_bodyMass = 0;
    m_totalWheelMass = 0;
    m_gravityLength = 0;
    m_vehicleFlags = 0;
    std::fill_n(m_wheelPosition_Bs, VEHICLE_MAX_WHEEL_COUNT, vec3_origin);
    std::fill_n(m_tracePosition_Bs, VEHICLE_MAX_WHEEL_COUNT, vec3_origin);

    m_bTraceData = false;
    if (m_nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST)
    {
        m_bTraceData = true;
    }

    m_nTireType = VEHICLE_TIRE_NORMAL;

    m_bOccupied = false;
    m_bEngineDisable = false;

    m_carBodyId = b3_nullBodyId;
    std::fill_n(m_raycastContacts, VEHICLE_MAX_WHEEL_COUNT, RaycastWheelContact{});
    std::fill_n(m_wheelFrictionOverride, VEHICLE_MAX_WHEEL_COUNT, -1.0f);
    m_controls = {};
}

Box3DVehicleController::~Box3DVehicleController()
{
    for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++)
    {
        if (m_pWheels[i] && m_pEnv)
        {
            m_pEnv->DestroyObject(m_pWheels[i]);
        }
        m_pWheels[i] = nullptr;
    }
}

void Box3DVehicleController::OnObjectDestroyed(Box3DPhysicsObject* pObject)
{
    if (!pObject)
        return;

    // The game freed the chassis: drop it so nothing here dereferences a stale
    // pointer or its now-invalid body id.
    if (pObject == m_pCarBody)
    {
        m_pCarBody = nullptr;
        m_carBodyId = b3_nullBodyId;
    }

    // A wheel the controller owns is normally destroyed in the dtor, but if the
    // game destroyed one out from under us, forget it (the joint dies with it).
    for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++)
    {
        if (m_pWheels[i] == pObject)
        {
            m_pWheels[i] = nullptr;
        }
    }
}

IPhysicsObject* Box3DVehicleController::GetWheel(int index)
{
    if (index < 0 || index >= m_wheelCount || index >= VEHICLE_MAX_WHEEL_COUNT)
        return nullptr;

    return m_pWheels[index];
}

void Box3DVehicleController::SetWheelFriction(int wheelIndex, float friction)
{
    if (wheelIndex < 0 || wheelIndex >= VEHICLE_MAX_WHEEL_COUNT)
        return;
    if (!IsFinite(friction))
        return;
    if (IsRaycastVehicle())
    {
        m_wheelFrictionOverride[wheelIndex] = friction;
    }
}

bool Box3DVehicleController::GetWheelContactPoint(int index, Vector* pContactPoint, int* pSurfaceProps)
{
    if (pContactPoint)
    {
        pContactPoint->Init();
    }
    if (pSurfaceProps)
    {
        *pSurfaceProps = 0;
    }
    if (index < 0 || index >= m_wheelCount || index >= VEHICLE_MAX_WHEEL_COUNT)
        return false;

    if (IsRaycastVehicle())
    {
        const RaycastWheelContact& state = m_raycastContacts[index];
        if (!state.inContact)
            return false;
        if (pContactPoint)
        {
            *pContactPoint = state.point;
        }
        if (pSurfaceProps)
        {
            *pSurfaceProps = state.surfaceProps;
        }
        return true;
    }

    b3Pos point{};
    int surfaceProps = 0;
    if (!WheelContact(index, &point, &surfaceProps))
        return false;

    if (pContactPoint)
    {
        *pContactPoint = BoxToSource::Distance(point);
    }
    if (pSurfaceProps)
    {
        *pSurfaceProps = surfaceProps;
    }
    return true;
}

void Box3DVehicleController::InitCarSystem(Box3DPhysicsObject* pBodyObject)
{
    // Car body.
    m_pCarBody = pBodyObject;
    m_bodyMass = m_pCarBody ? m_pCarBody->GetMass() : 0.0f;
    m_carBodyId = m_pCarBody ? m_pCarBody->GetBodyID() : b3_nullBodyId;
    if (m_pEnv)
    {
        // Gravity magnitude in box units (m/s^2); used consistently in the
        // box-space force math below.
        m_gravityLength = b3Length(b3World_GetGravity(m_pEnv->GetWorldId()));
    }

    m_wheelCount = m_vehicleData.axleCount * m_vehicleData.wheelsPerAxle;
    if (m_wheelCount > VEHICLE_MAX_WHEEL_COUNT)
    {
        m_wheelCount = VEHICLE_MAX_WHEEL_COUNT;
    }

    m_totalWheelMass = 0.0f;
    for (int i = 0; i < m_vehicleData.axleCount; i++)
    {
        m_totalWheelMass += m_vehicleData.axles[i].wheels.mass * m_vehicleData.wheelsPerAxle;
    }

    if (m_pEnv && m_pCarBody && m_wheelCount > 0 && m_vehicleData.wheelsPerAxle > 0)
    {
        Vector positions[VEHICLE_MAX_WHEEL_COUNT];
        for (int i = 0; i < m_wheelCount; i++)
        {
            const vehicle_axleparams_t& axle = m_vehicleData.axles[i / m_vehicleData.wheelsPerAxle];
            const int wheelInAxle = i % m_vehicleData.wheelsPerAxle;
            positions[i] = axle.offset + ((wheelInAxle % 2 == 1) ? axle.wheelOffset : -axle.wheelOffset);
        }
        if (IsRaycastVehicle())
        {
            FixDegenerateWheelPositions(positions);
        }

        for (int i = 0; i < m_wheelCount; i++)
        {
            m_pWheels[i] = CreateWheel(i, positions[i]);
        }
        AttachWheels();
    }
}

// Raycast vehicles only. The airboat (its model's rear-right wheel attachment
// is misspelled "wheele_rr", so the game never fills axle offsets) reaches us
// with every attach point at the chassis origin, which gives the suspension no
// support base. Synthesize pontoon attach points from the chassis collide
// bounds instead, like the legacy airboat placed its pontoons itself.
bool Box3DVehicleController::FixDegenerateWheelPositions(Vector positions[VEHICLE_MAX_WHEEL_COUNT])
{
    bool degenerate = false;
    for (int i = 0; i < m_wheelCount && !degenerate; i++)
    {
        for (int j = i + 1; j < m_wheelCount; j++)
        {
            if ((positions[i] - positions[j]).Length() < kWheelCoincidentDist)
            {
                degenerate = true;
                break;
            }
        }
    }
    if (!degenerate)
        return false;

    Vector mins, maxs;
    if (!LocalCollideBounds(m_pCarBody ? m_pCarBody->GetCollide() : nullptr, &mins, &maxs))
    {
        float radius = Max(m_wheelRadius, 10.0f);
        mins = Vector(-2.0f * radius, -3.0f * radius, -radius);
        maxs = -mins;
    }

    const Vector center = (mins + maxs) * 0.5f;
    const Vector size = maxs - mins;
    const int axleCount = Max(m_vehicleData.axleCount, 1);
    for (int i = 0; i < m_wheelCount; i++)
    {
        const int axleIndex = i / m_vehicleData.wheelsPerAxle;
        const int wheelInAxle = i % m_vehicleData.wheelsPerAxle;
        const float axleFraction = axleCount > 1 ? static_cast<float>(axleIndex) / (axleCount - 1) : 0.5f;
        positions[i].x = center.x + ((wheelInAxle % 2 == 1) ? 1.0f : -1.0f) * 0.4f * size.x;
        positions[i].y = center.y + (0.5f - axleFraction) * 0.8f * size.y;
        positions[i].z = mins.z;
    }

    Warning("bphys: vehicle wheel positions are degenerate, synthesized from chassis bounds\n");
    return true;
}

Box3DPhysicsObject* Box3DVehicleController::CreateWheel(int wheelIndex, const Vector& wheelPositionLocal)
{
    const int axleIndex = wheelIndex / m_vehicleData.wheelsPerAxle;
    const vehicle_axleparams_t& axle = m_vehicleData.axles[axleIndex];

    matrix3x4_t bodyMatrix;
    m_pCarBody->GetPositionMatrix(&bodyMatrix);
    Vector wheelPositionWorld;
    VectorTransform(wheelPositionLocal, bodyMatrix, wheelPositionWorld);

    Vector bodyPosition;
    QAngle bodyAngles;
    m_pCarBody->GetPosition(&bodyPosition, &bodyAngles);

    const float radius = Max(axle.wheels.radius, kWheelRadiusMin);
    const float width = radius * 0.5f;

    objectparams_t wheelParams{};
    wheelParams.mass = axle.wheels.mass;
    wheelParams.inertia = axle.wheels.inertia;
    wheelParams.damping = axle.wheels.damping;
    wheelParams.rotdamping = axle.wheels.rotdamping;
    wheelParams.pName = "VehicleWheel";
    wheelParams.pGameData = m_pCarBody->GetGameData();
    wheelParams.volume = M_PI_F * width * radius * radius;
    wheelParams.enableCollisions = !IsRaycastVehicle();

    // CreateSphereObject takes the radius/position in Source inches.
    IPhysicsObject* pObject = m_pEnv->CreateSphereObject(
        radius, axle.wheels.materialIndex, wheelPositionWorld, bodyAngles, &wheelParams, false);
    Box3DPhysicsObject* pWheel = static_cast<Box3DPhysicsObject*>(pObject);
    if (!pWheel)
        return nullptr;

    pWheel->SetGameFlags(m_pCarBody->GetGameFlags());
    // Wheels report as wheels only and generate no game collision callbacks
    pWheel->SetCallbackFlags(CALLBACK_IS_VEHICLE_WHEEL);
    pWheel->Wake();

    if (IsRaycastVehicle())
    {
        // Pose/query proxy only: the raycast suspension drives the chassis
        // directly, this body never simulates.
        pWheel->EnableGravity(false);
        pWheel->EnableMotion(false);
    }
    else
    {
        b3BodyId wheelBody = pWheel->GetBodyID();
        b3Body_SetBullet(wheelBody, true);

        if (IsFinite(axle.wheels.frictionScale) && axle.wheels.frictionScale > 0.0f)
        {
            b3ShapeId shapes[1];
            if (b3Body_GetShapes(wheelBody, shapes, 1) > 0)
            {
                b3SurfaceMaterial material = b3Shape_GetSurfaceMaterial(shapes[0]);
                material.friction = axle.wheels.frictionScale;
                b3Shape_SetSurfaceMaterial(shapes[0], material);
            }
        }
    }

    m_wheelPosition_Bs[wheelIndex] = wheelPositionLocal;
    m_tracePosition_Bs[wheelIndex] = wheelPositionLocal;
    if (radius > m_wheelRadius)
    {
        m_wheelRadius = radius;
    }

    return pWheel;
}

float Box3DVehicleController::MaxSteeringAngleRad() const
{
    return DEG2RAD(Max(m_vehicleData.steering.degreesSlow, m_vehicleData.steering.degreesFast));
}

bool Box3DVehicleController::WheelContact(int wheelIndex, b3Pos* pPoint, int* pSurfaceProps) const
{
    const Box3DPhysicsObject* pWheel = m_pWheels[wheelIndex];
    if (!pWheel)
        return false;

    b3BodyId wheelBody = pWheel->GetBodyID();
    if (B3_IS_NULL(wheelBody) || !b3Body_IsValid(wheelBody))
        return false;

    b3ContactData contacts[8];
    const int count = b3Body_GetContactData(wheelBody, contacts, 8);
    for (int c = 0; c < count; ++c)
    {
        const b3ContactData& data = contacts[c];
        for (int m = 0; m < data.manifoldCount; ++m)
        {
            const b3Manifold& manifold = data.manifolds[m];
            if (manifold.pointCount <= 0)
                continue;

            b3BodyId bodyA = b3Shape_GetBody(data.shapeIdA);
            b3BodyId bodyB = b3Shape_GetBody(data.shapeIdB);
            const bool wheelIsA = B3_ID_EQUALS(bodyA, wheelBody);
            if (pPoint)
            {
                const b3BodyId wheelContactBody = wheelIsA ? bodyA : bodyB;
                const b3Vec3 wheelAnchor = wheelIsA ? manifold.points[0].anchorA : manifold.points[0].anchorB;
                *pPoint = b3OffsetPos(b3Body_GetWorldCenter(wheelContactBody), wheelAnchor);
            }
            if (pSurfaceProps)
            {
                b3ShapeId otherShape = wheelIsA ? data.shapeIdB : data.shapeIdA;
                *pSurfaceProps = ShapeMaterialIdAtTriangle(otherShape, manifold.points[0].triangleIndex);
            }
            return true;
        }
    }
    return false;
}

void Box3DVehicleController::VehicleDataReload()
{
    // compute torque normalization factor
    m_torqueScale = 1;
    // Clear accumulation.
    float totalTorqueDistribution = 0.0f;
    for (int i = 0; i < m_vehicleData.axleCount; i++)
    {
        totalTorqueDistribution += m_vehicleData.axles[i].torqueFactor;
    }

    if (totalTorqueDistribution > 0)
    {
        m_torqueScale /= totalTorqueDistribution;
    }
    // Authored input speed is mph. Keep m_vehicleData raw so VehicleDataReload is idempotent and
    // GetVehicleParamsForChange callers do not see already-converted units.
    m_engineMaxSpeed = m_vehicleData.engine.maxSpeed * kMphToInchesPerSecond;
    m_engineMaxRevSpeed = m_vehicleData.engine.maxRevSpeed * kMphToInchesPerSecond;
    m_engineBoostMaxSpeed = m_vehicleData.engine.boostMaxSpeed * kMphToInchesPerSecond;
}

void Box3DVehicleController::InitVehicleData(const vehicleparams_t& params)
{
    m_vehicleData = params;
    VehicleDataReload();
}

void Box3DVehicleController::SetSpringLength(int wheelIndex, float length)
{
    if (wheelIndex < 0 || wheelIndex >= m_wheelCount || m_vehicleData.wheelsPerAxle <= 0 || !IsFinite(length))
        return;

    const int axleIndex = wheelIndex / m_vehicleData.wheelsPerAxle;
    if (axleIndex < 0 || axleIndex >= m_vehicleData.axleCount)
        return;

    m_vehicleData.axles[axleIndex].wheels.springAdditionalLength = Max(length, 0.0f);
    if (m_pCarBody)
        m_pCarBody->Wake();
}

void Box3DVehicleController::CastWheel(
    const Vector& start, const Vector& dirDown, float castDist, float* pHitDist, Vector* pNormal, int* pSurfaceProps,
    bool* pInWater) const
{
    *pHitDist = -1.0f;
    *pNormal = Vector(0.0f, 0.0f, 1.0f);
    *pSurfaceProps = 0;
    *pInWater = false;

    if (m_pGameTrace)
    {
        Ray_t ray;
        ray.Init(start, start + dirDown * castDist);
        trace_t tr;
        // Airboat pontoons (and jetskis) treat the water surface as ground,
        // exactly like the legacy raycast car systems.
        void* pVehicleGameData = m_pCarBody ? m_pCarBody->GetGameData() : nullptr;
        if (m_nVehicleType == VEHICLE_TYPE_CAR_RAYCAST)
        {
            m_pGameTrace->VehicleTraceRay(ray, pVehicleGameData, &tr);
        }
        else
        {
            m_pGameTrace->VehicleTraceRayWithWater(ray, pVehicleGameData, &tr);
        }

        if (tr.startsolid)
        {
            *pHitDist = 0.0f;
        }
        else if (tr.fraction < 1.0f)
        {
            *pHitDist = tr.fraction * castDist;
            *pNormal = tr.plane.normal;
            *pSurfaceProps = tr.surface.surfaceProps;
            *pInWater = (tr.contents & MASK_WATER) != 0;
        }
        return;
    }

    // No game trace: cast against the physics world, skipping the chassis,
    // sensors, and any vehicle wheel.
    struct RayContext
    {
        b3BodyId ignoreBody;
        float fraction;
        b3Vec3 normal;
        uint64_t material;
        bool hit;
    };
    RayContext ctx{ m_carBodyId, 1.0f, b3Vec3{ 0.0f, 0.0f, 1.0f }, 0, false };

    constexpr auto filter = [](b3ShapeId shape, b3Pos, b3Vec3 normal, float fraction, uint64_t materialId, int, int,
                               void* context) -> float {
        RayContext* pCtx = static_cast<RayContext*>(context);
        b3BodyId body = b3Shape_GetBody(shape);
        if (B3_ID_EQUALS(body, pCtx->ignoreBody) || b3Shape_IsSensor(shape))
            return -1.0f;
        Box3DPhysicsObject* pObject = static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(body));
        if (pObject && (pObject->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL))
            return -1.0f;
        if (fraction <= pCtx->fraction)
        {
            pCtx->fraction = fraction;
            pCtx->normal = normal;
            pCtx->material = materialId;
            pCtx->hit = true;
        }
        return fraction;
    };
    // Ray origin/extent are Source distances -> metres.
    b3World_CastRay(
        m_pEnv->GetWorldId(), SourceToBox::Distance(start), SourceToBox::Distance(dirDown * castDist), b3DefaultQueryFilter(),
        static_cast<b3CastResultFcn*>(filter), &ctx);
    if (ctx.hit)
    {
        *pHitDist = ctx.fraction * castDist;
        *pNormal = BoxToSource::Unitless(ctx.normal);
        *pSurfaceProps = static_cast<int>(ctx.material);
    }
}

float Box3DVehicleController::WaterDepthAt(const Vector& worldPos) const
{
    if (!m_pGameTrace)
        return 0.0f;

    // Trace straight up: fractionleftsolid is the fraction of the ray spent in
    // water starting from worldPos, so depth = that fraction * ray length.
    constexpr float kRayLength = 1000.0f;
    Ray_t ray;
    ray.Init(worldPos, worldPos + Vector(0.0f, 0.0f, kRayLength));
    trace_t tr;
    void* pVehicleGameData = m_pCarBody ? m_pCarBody->GetGameData() : nullptr;
    m_pGameTrace->VehicleTraceRayWithWater(ray, pVehicleGameData, &tr);
    return kRayLength * tr.fractionleftsolid;
}

void Box3DVehicleController::GetCarSystemDebugData(vehicle_debugcarsystem_t& debugCarSystem)
{
    memset(&debugCarSystem, 0, sizeof(debugCarSystem));
    if (B3_IS_NULL(m_carBodyId) || !b3Body_IsValid(m_carBodyId) || m_vehicleData.wheelsPerAxle <= 0)
        return;

    b3WorldTransform xf = b3Body_GetTransform(m_carBodyId);

    const int axleCount = Min(m_vehicleData.axleCount, VEHICLE_DEBUGRENDERDATA_MAX_AXLES);
    for (int i = 0; i < axleCount; i++)
    {
        debugCarSystem.vecAxlePos[i] = BoxToSource::Distance(
            b3TransformWorldPoint(xf, SourceToBox::Distance(m_vehicleData.axles[i].offset)));
    }

    const int wheelCount = Min(m_wheelCount, VEHICLE_DEBUGRENDERDATA_MAX_WHEELS);
    for (int i = 0; i < wheelCount; i++)
    {
        if (!m_pWheels[i])
            continue;
        b3BodyId wheelBody = m_pWheels[i]->GetBodyID();
        if (!b3Body_IsValid(wheelBody))
            continue;

        const Vector wheelPos = BoxToSource::Distance(b3Body_GetPosition(wheelBody));
        debugCarSystem.vecWheelPos[i] = wheelPos;
        debugCarSystem.vecWheelRaycasts[i][0] = BoxToSource::Distance(
            b3TransformWorldPoint(xf, SourceToBox::Distance(m_wheelPosition_Bs[i])));
        debugCarSystem.vecWheelRaycasts[i][1] = wheelPos;

        if (IsRaycastVehicle())
        {
            const RaycastWheelContact& state = m_raycastContacts[i];
            debugCarSystem.vecWheelRaycastImpacts[i] = state.inContact ? state.point : wheelPos;
        }
        else
        {
            b3Pos contact{};
            int surfaceProps = 0;
            debugCarSystem.vecWheelRaycastImpacts[i] = WheelContact(i, &contact, &surfaceProps) ? BoxToSource::Distance(contact)
                                                                                                : wheelPos;
        }
    }
}

void Box3DVehicleController::OnVehicleEnter(void)
{
    m_bOccupied = true;

    if (B3_IS_NON_NULL(m_carBodyId) && b3Body_IsValid(m_carBodyId))
    {
        b3Body_SetAwake(m_carBodyId, true);
    }

    if (m_nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST && m_pCarBody)
    {
        float flDampSpeed = 0.0f;
        float flDampRotSpeed = 0.0f;
        m_pCarBody->SetDamping(&flDampSpeed, &flDampRotSpeed);
    }
}

void Box3DVehicleController::OnVehicleExit(void)
{
    m_bOccupied = false;

    if (m_vehicleData.steering.isSkidAllowed)
    {
        m_nTireType = VEHICLE_TIRE_NORMAL;
        m_currentState.skidSpeed = 0.0f;
    }

    if (m_nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST && m_pCarBody)
    {
        float flDampSpeed = 1.0f;
        float flDampRotSpeed = 1.0f;
        m_pCarBody->SetDamping(&flDampSpeed, &flDampRotSpeed);
    }

    SetEngineDisabled(false);
}
