#include <iostream>
#include <vector>
#include <map>




/*******************************************************************************************
*
*   raylib [core] example - 3d camera first person
*
*   This example has been created using raylib 1.3 (www.raylib.com)
*   raylib is licensed under an unmodified zlib/libpng license (View raylib.h for details)
*
*   Copyright (c) 2015 Ramon Santamaria (@raysan5)
*
********************************************************************************************/

#include "raylib.h"
#include "base.h"


#include <btBulletDynamicsCommon.h>

#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#define BV(v) (btVector3((v).x, (v).y, (v).z))
#define BQ(q) (btQuaternion((q).x, (q).y, (q).z, (q).w))

using namespace std;




const Vector3 ZERO= (Vector3){0,0,0};
const Vector3 ONE= (Vector3){1,1,1};

#define PHYSICS_COLLISION_GROUP_DEFAULT btBroadphaseProxy::DefaultFilter
#define PHYSICS_COLLISION_MASK_DEFAULT btBroadphaseProxy::AllFilter

class PhysicsCollisionShape
{
    friend class PhysicsController;
    friend class PhysicsRigidBody;

public:
    enum Type
    {
        SHAPE_NONE,        SHAPE_BOX,        SHAPE_SPHERE,        SHAPE_CAPSULE,        SHAPE_MESH,        SHAPE_HEIGHTFIELD
    };
     PhysicsCollisionShape( Type type, btCollisionShape* shape, btStridingMeshInterface* meshInterface=NULL)
    : _type(type), _shape(shape), _meshInterface(meshInterface)
     {

     }


    ~PhysicsCollisionShape()
    {

    }
private:

float  center[3];
float  extents[3];
float  radius;
float  height;

Type _type;
bool isExplicit;
btCollisionShape* _shape;
bool centerAbsolute;
// Bullet mesh interface for mesh types (NULL otherwise)
btStridingMeshInterface* _meshInterface;

};

class PhysicsCollisionObject
{

public:
    enum Type
    {
        RIGID_BODY,CHARACTER,        GHOST_OBJECT,        VEHICLE,        VEHICLE_WHEEL,        NONE
    };
  PhysicsCollisionObject()
  {
  }

   ~PhysicsCollisionObject()
  {
  }

    class CollisionPair
    {
    public:


        CollisionPair(PhysicsCollisionObject* objectA, PhysicsCollisionObject* objectB)
        {}


        bool operator < (const CollisionPair& collisionPair) const
        {
        }


        PhysicsCollisionObject* objectA;

        PhysicsCollisionObject* objectB;
    };

       class CollisionListener
    {
        friend class PhysicsCollisionObject;
        friend class PhysicsController;

    public:

        enum EventType
        {COLLIDING,            NOT_COLLIDING        };


        virtual ~CollisionListener() { }


        virtual void collisionEvent(PhysicsCollisionObject::CollisionListener::EventType type,
                                    const PhysicsCollisionObject::CollisionPair& collisionPair,
                                    const Vector3& contactPointA = ZERO,
                                    const Vector3& contactPointB = ZERO)
          {
          }
    };

        PhysicsCollisionShape* _collisionShape;

};
class PhysicsRigidBody : public PhysicsCollisionObject
{
public:


/**
     * Rigid body construction parameters.
     */
    struct Parameters
    {
        /**
         * The mass of the rigid body, in kilograms.
         */
        float mass;

        /**
         * The friction of the rigid body (non-zero values give best simulation results).
         */
        float friction;

        /**
         * The restitution of the rigid body (this controls the bounciness of
         * the rigid body; use zero for best simulation results).
         */
        float restitution;

        /**
         * The percentage of linear velocity lost per second (between 0.0 and 1.0).
         */
        float linearDamping;

        /**
         * The percentage of angular velocity lost per second (between 0.0 and 1.0).
         */
        float angularDamping;

        /**
         * Whether the rigid body is kinematic.
         */
        bool kinematic;

        /**
         * The anisotropic friction term for the rigid body.
         */
        Vector3 anisotropicFriction;

        /**
         * Linear factor for the rigid body. x, y, z coordinates correspond to world
         * space motion along these axes. Use 1.0 to allow or 0.0 to disallow motion
         * along certain axis.
         */
        Vector3 linearFactor;

        /**
         * Angular factor for the rigid body. x, y, z coordinates correspond to world
         * space rotation along these axes. Use 1.0 to allow or 0.0 to disallow rotation
         * along certain axis.
         */
        Vector3 angularFactor;

        /**
         * Constructor.
         */
        Parameters() : mass(0.0f), friction(0.5f), restitution(0.0f),
            linearDamping(0.0f), angularDamping(0.0f),
            kinematic(false), anisotropicFriction(ONE), linearFactor(ONE), angularFactor(ONE)
        {
        }

        /**
         * Constructor.
         */
        Parameters(float mass, float friction = 0.5f, float restitution = 0.0f,
            float linearDamping = 0.0f, float angularDamping = 0.0f, bool kinematic = false,
            const Vector3& anisotropicFriction = ONE, const Vector3& linearFactor = ONE,
            const Vector3& angularFactor = ONE)
            : mass(mass), friction(friction), restitution(restitution), linearDamping(linearDamping), angularDamping(angularDamping),
              kinematic(kinematic), anisotropicFriction(anisotropicFriction), linearFactor(linearFactor), angularFactor(angularFactor)
        {
        }
    };
   PhysicsRigidBody(const PhysicsCollisionShape& shape, const Parameters& parameters, int group = PHYSICS_COLLISION_GROUP_DEFAULT, int mask = PHYSICS_COLLISION_MASK_DEFAULT)
   {
   }
 ~PhysicsRigidBody()
 {
 }

    btRigidBody* _body;
    float _mass;
    //std::vector<PhysicsConstraint*>* _constraints;
    bool _inDestructor;
};


class PhysicsController
{

  PhysicsController(): _isUpdating(false), _collisionConfiguration(NULL), _dispatcher(NULL),
    _overlappingPairCache(NULL), _solver(NULL), _world(NULL),
    _debugDrawer(NULL), _status(PhysicsController::Listener::DEACTIVATED), _listeners(NULL),
     _collisionCallback(NULL)
{
   // Default gravity is 9.8 along the negative Y axis.
    _collisionCallback = new CollisionCallback(this);
    _gravity=(Vector3){btScalar(0.0), btScalar(-9.8), btScalar(0.0)};
 }

 virtual ~PhysicsController()
 {
 SAFE_DELETE(_collisionCallback);
    SAFE_DELETE(_ghostPairCallback);
    SAFE_DELETE(_debugDrawer);
    SAFE_DELETE(_listeners);
}
PhysicsCollisionShape* createBox(const Vector3& extents, const Vector3& scale)
{
    btVector3 halfExtents(scale.x * 0.5 * extents.x, scale.y * 0.5 * extents.y, scale.z * 0.5 * extents.z);

    PhysicsCollisionShape* shape;

   // Create the box shape and add it to the cache.
    shape = new PhysicsCollisionShape(PhysicsCollisionShape::SHAPE_BOX, new btBoxShape(halfExtents));


    return shape;
}

void update(float elapsedTime)
{

    _isUpdating = true;

    // Update the physics simulation, with a maximum
    // of 10 simulation steps being performed in a given frame.
    //
    // Note that stepSimulation takes elapsed time in seconds
    // so we divide by 1000 to convert from milliseconds.
    _world->stepSimulation(elapsedTime * 0.001f, 10);
}

void initialize()
{
    _collisionConfiguration = new btDefaultCollisionConfiguration();
    _dispatcher = new btCollisionDispatcher(_collisionConfiguration);
    _overlappingPairCache = new btDbvtBroadphase();
    _solver = new btSequentialImpulseConstraintSolver();

    // Create the world.
    _world = new btDiscreteDynamicsWorld(_dispatcher, _overlappingPairCache, _solver, _collisionConfiguration);
    _world->setGravity(BV(_gravity));

    // Register ghost pair callback so bullet detects collisions with ghost objects (used for character collisions).

    _ghostPairCallback = bullet_new<btGhostPairCallback>();
    _world->getPairCache()->setInternalGhostPairCallback(_ghostPairCallback);
    _world->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;

    // Set up debug drawing.
    _debugDrawer = new DebugDrawer();
    _world->setDebugDrawer(_debugDrawer);
}

private:

    class DebugDrawer : public btIDebugDraw
    {
    public:

        DebugDrawer()
        {}


        ~DebugDrawer()
        {}


        void drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor)
        {
        }
        void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
        {
        }
        void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
        {
        }
        void reportErrorWarning(const char* warningString)
        {
        }
        void draw3dText(const btVector3& location, const char* textString)
        {}
        void setDebugMode(int mode)
        {
        }
        int getDebugMode() const
        {
        }

    private:
        int _mode;

    };

    class CollisionCallback : public btCollisionWorld::ContactResultCallback
    {
    public:
        CollisionCallback(PhysicsController* pc) : _pc(pc)
        {}

    protected:
        btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* a, int partIdA, int indexA, const btCollisionObjectWrapper* b, int partIdB, int indexB)
        {
        }

    private:
        PhysicsController* _pc;
    };

      struct CollisionInfo
    {
        CollisionInfo() : _status(0) { }

        std::vector<PhysicsCollisionObject::CollisionListener*> _listeners;
        int _status;
    };
    class Listener
    {
    public:


        enum EventType
        {

            ACTIVATED,


            DEACTIVATED
        };

        virtual void statusEvent(EventType type)
        {
        }

    protected:


        virtual ~Listener()
        {
        }
    };


    struct HitResult
    {

        PhysicsCollisionObject* object;


        Vector3 point;

        float fraction;

        Vector3 normal;
    };


    class HitFilter
    {
    public:

        HitFilter();

        virtual ~HitFilter()
        {}
        virtual bool filter(PhysicsCollisionObject* object)
        {
        }

        virtual bool hit(const HitResult& result)
        {
        }
    };
    static const int DIRTY;
    static const int COLLISION;
    static const int REGISTERED;
    static const int REMOVE;
private:

    bool _isUpdating;
    btDefaultCollisionConfiguration* _collisionConfiguration;
    btCollisionDispatcher* _dispatcher;
    btBroadphaseInterface* _overlappingPairCache;
    btSequentialImpulseConstraintSolver* _solver;
    btDynamicsWorld* _world;
   btGhostPairCallback* _ghostPairCallback;
//    std::vector<PhysicsCollisionShape*> _shapes;
    DebugDrawer* _debugDrawer;
   Listener::EventType _status;
    std::vector<Listener*>* _listeners;
    Vector3 _gravity;
    std::map<PhysicsCollisionObject::CollisionPair, CollisionInfo> _collisionStatus;
    CollisionCallback* _collisionCallback;

};

const int PhysicsController::DIRTY         = 0x01;
const int PhysicsController::COLLISION     = 0x02;
const int PhysicsController::REGISTERED    = 0x04;
const int PhysicsController::REMOVE        = 0x08;

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1424;
    const int screenHeight = 950;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - 3d camera first person");

    // Define the camera to look into our 3d world (position, target, up vector)
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 20.0f, 20.0f };
    camera.target = (Vector3){ 0.0f, 1.8f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;



    SetCameraMode(camera, CAMERA_FREE); // Set a first person camera mode

    SetTargetFPS(60);                           // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())                // Detect window close button or ESC key
    {
        // Update
                if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera);                  // Update camera
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground((Color){0,0,45,0});

            BeginMode3D(camera);



           DrawGrid(10, 1.0f);         // Draw a grid

            EndMode3D();



             DrawRectangle( 10, 10, 320, 133, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines( 10, 10, 320, 133, BLUE);

            DrawText("Free camera default controls:", 20, 20, 10, BLACK);
            DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, WHITE);
            DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, WHITE);
            DrawText("- Alt + Mouse Wheel Pressed to Rotate", 40, 80, 10, WHITE);
            DrawText("- Alt + Ctrl + Mouse Wheel Pressed for Smooth Zoom", 40, 100, 10, WHITE);
            DrawText("- Z to zoom to (0, 0, 0)", 40, 120, 10, WHITE);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
