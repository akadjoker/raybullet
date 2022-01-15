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
#include "rlgl.h"
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

Vector3 btor(btVector3 v)
{

  Vector3 r;

  r.x=v.getX();
  r.y=v.getY();
  r.z=v.getZ();

  return r;
}


Vector3 VT(float x=0,float y=0,float z=0)
{

  Vector3 r;

  r.x=x;
  r.y=y;
  r.z=z;

  return r;
}
 inline float lerpfloat(float s, float from, float to)
{
    return from + (to - from) * s;
}
 inline unsigned char lerpuchar(float s, unsigned char from, unsigned char to)
{
    return from + (to - from) * (s*255.0f);
}

 inline Color lerpcolor(float s, Color from, Color to)
{
   Color c;
      c.r=lerpuchar(s,from.r,to.r);
      c.g=lerpuchar(s,from.g,to.g);
      c.b=lerpuchar(s,from.b,to.b);
      c.a=lerpuchar(s,from.a,to.a);

     return c;
}
 inline Color lerpcolor(float s, btVector3 from, btVector3 to)
{
   Color c;
      c.r=lerpuchar(s,from.getX()*255,to.getX()*255);
      c.g=lerpuchar(s,from.getY()*255,to.getY()*255);
      c.b=lerpuchar(s,from.getZ()*255,to.getZ()*255);
      c.a=255;
     return c;
}
    class DebugDrawer : public btIDebugDraw
    {
    public:

        DebugDrawer()
        {

       //setDebugMode(btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawContactPoints | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits);

       setDebugMode( btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawContactPoints | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits);

        }


        ~DebugDrawer()
        {}


        void drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor)
        {
          DrawLine3D(btor(from),btor(to),lerpcolor(GetFrameTime(),fromColor,toColor));
         // TraceLog(LOG_INFO," line  %f ",from.getX());
        }
        void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
        {
            drawLine(from, to, color, color);

           // TraceLog(LOG_INFO," line  %f ",from.getX());
        }



        void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
        {
           drawLine(pointOnB, pointOnB + normalOnB, color);
        }
        void reportErrorWarning(const char* warningString)
        {
         TraceLog(0,warningString);
        }
        void draw3dText(const btVector3& location, const char* textString)
        {
        TraceLog(0,textString);
        }

        	 void	setDebugMode(int debugMode)
        	 {
        	 _mode=debugMode;
        	 }

	        int		getDebugMode() const
	        {
	        return _mode;
	        }
   int _mode;

    };


        	class MotionState : public btDefaultMotionState
    	{
	public:
		MotionState(const btTransform &transform) : btDefaultMotionState(transform) {}

		void GetWorldTransform(btScalar* transform)
		{
			btTransform trans;
			getWorldTransform(trans);
			trans.getOpenGLMatrix(transform);
		}
	};

class GameObject
{
	public:
		GameObject(btCollisionShape* pShape, float mass, const btVector3 &color, const btVector3 &initialPosition = btVector3(0,0,0), const btQuaternion &initialRotation = btQuaternion(0,0,1,1))
		{
		m_pShape = pShape;

		// store the color
		m_color = color;

		// create the initial transform
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(initialPosition);
		transform.setRotation(initialRotation);

		// create the motion state from the
		// initial transform
		m_pMotionState = new MotionState(transform);

		// calculate the local inertia
		btVector3 localInertia(0,0,0);

		// objects of infinite mass can't
		// move or rotate
		if (mass != 0.0f)
			pShape->calculateLocalInertia(mass, localInertia);

		// create the rigid body construction
		// info using the mass, motion state
		// and shape
		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, m_pMotionState, pShape, localInertia);

		// create the rigid body
		m_pBody = new btRigidBody(cInfo);
		}
		~GameObject()
		{
			delete m_pBody;
		delete m_pMotionState;
		delete m_pShape;
		}

		// accessors
		btCollisionShape* GetShape() { return m_pShape; }

		btRigidBody* GetRigidBody() { return m_pBody; }

		btMotionState* GetMotionState() { return m_pMotionState; }

		void GetTransform(btScalar* transform)
		{
			if (m_pMotionState) m_pMotionState->GetWorldTransform(transform);
		}

		btVector3 GetColor() { return m_color; }

	protected:
		btCollisionShape*  m_pShape;
		btRigidBody*    m_pBody;
		MotionState*  m_pMotionState;
		btVector3      m_color;
	};

typedef std::vector<GameObject*> GameObjects;

    // core Bullet components
	btBroadphaseInterface* m_pBroadphase;
	btCollisionConfiguration* m_pCollisionConfiguration;
	btCollisionDispatcher* m_pDispatcher;
	btConstraintSolver* m_pSolver;
	btDynamicsWorld* m_pWorld;
    DebugDrawer* _debugDrawer;
    GameObjects m_objects;


GameObject* CreateGameObject(btCollisionShape* pShape,
				const float &mass,
				const btVector3 &color = btVector3(1.0f,1.0f,1.0f),
				const btVector3 &initialPosition = btVector3(0.0f,0.0f,0.0f),
				const btQuaternion &initialRotation = btQuaternion(0,0,1,1))

{
	   GameObject* pObject = new GameObject(pShape, mass, color, initialPosition, initialRotation);

		// push it to the back of the list
		m_objects.push_back(pObject);

		// check if the world object is valid
		if (m_pWorld) {
			// add the object's rigid body to the world
			m_pWorld->addRigidBody(pObject->GetRigidBody());
		}
		return pObject;

}


void DrawBox( const btVector3 &halfSize)
{



	float halfWidth = halfSize.x();
	float halfHeight = halfSize.y();
	float halfDepth = halfSize.z();



	// create the vertex positions
	btVector3 vertices[8]={
	btVector3(halfWidth,halfHeight,halfDepth),
	btVector3(-halfWidth,halfHeight,halfDepth),
	btVector3(halfWidth,-halfHeight,halfDepth),
	btVector3(-halfWidth,-halfHeight,halfDepth),
	btVector3(halfWidth,halfHeight,-halfDepth),
	btVector3(-halfWidth,halfHeight,-halfDepth),
	btVector3(halfWidth,-halfHeight,-halfDepth),
	btVector3(-halfWidth,-halfHeight,-halfDepth)};

	// create the indexes for each triangle, using the
	// vertices above. Make it static so we don't waste
	// processing time recreating it over and over again
	static int indices[36] = {
		0,1,2,
		3,2,1,
		4,0,6,
		6,0,2,
		5,1,4,
		4,1,0,
		7,3,1,
		7,1,5,
		5,4,7,
		7,4,6,
		7,2,3,
		7,6,2};

	// start processing vertices as triangles
	rlBegin (RL_TRIANGLES);



	for (int i = 0; i < 36; i += 3)
	{
		const btVector3 &vert1 = vertices[indices[i]];
		const btVector3 &vert2 = vertices[indices[i+1]];
		const btVector3 &vert3 = vertices[indices[i+2]];

		// create a normal that is perpendicular to the
		// face (use the cross product)
		btVector3 normal = (vert3-vert1).cross(vert2-vert1);
		normal.normalize ();

		// set the normal for the subsequent vertices
		rlNormal3f(normal.getX(),normal.getY(),normal.getZ());

		// create the vertices
		rlVertex3f (vert1.x(), vert1.y(), vert1.z());
		rlVertex3f (vert2.x(), vert2.y(), vert2.z());
		rlVertex3f (vert3.x(), vert3.y(), vert3.z());
	}

	// stop processing vertices
	rlEnd();


}

void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color)
{
    // set the color
    rlColor3f(color.x(), color.y(), color.z());

    // push the matrix stack
    rlPushMatrix();
    rlMultMatrixf(transform);

    // make a different draw call based on the object type
    switch(pShape->getShapeType())
    {
        // an internal enum used by Bullet for boxes
    case BOX_SHAPE_PROXYTYPE:
    {
        const btBoxShape* box = static_cast<const btBoxShape*>(pShape);
        btVector3 halfSize = box->getHalfExtentsWithMargin();
        DrawBox(halfSize);
        break;
    }
    default:
        // unsupported type
        break;
    }

    // pop the stack
    rlPopMatrix();
}
	btVector3 GetPickingRay(int x, int y)
	{
		// calculate the field-of-view
		float tanFov = 1.0f / m_nearPlane;
		float fov = btScalar(2.0) * btAtan(tanFov);

		// get a ray pointing forward from the
		// camera and extend it to the far plane
		btVector3 rayFrom = m_cameraPosition;
		btVector3 rayForward = (m_cameraTarget - m_cameraPosition);
		rayForward.normalize();
		rayForward*= m_farPlane;

		// find the horizontal and vertical vectors
		// relative to the current camera view
		btVector3 ver = m_upVector;
		btVector3 hor = rayForward.cross(ver);
		hor.normalize();
		ver = hor.cross(rayForward);
		ver.normalize();
		hor *= 2.f * m_farPlane * tanFov;
		ver *= 2.f * m_farPlane * tanFov;

		// calculate the aspect ratio
		btScalar aspect = m_screenWidth / (btScalar)m_screenHeight;

		// adjust the forward-ray based on
		// the X/Y coordinates that were clicked
		hor*=aspect;
		btVector3 rayToCenter = rayFrom + rayForward;
		btVector3 dHor = hor * 1.f/float(m_screenWidth);
		btVector3 dVert = ver * 1.f/float(m_screenHeight);
		btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * ver;
		rayTo += btScalar(x) * dHor;
		rayTo -= btScalar(y) * dVert;

		// return the final result
		return rayTo;
	}
void CreateObjects()
{

	CreateGameObject(new btBoxShape(btVector3(1,50,50)), 0, btVector3(0.2f, 0.6f, 0.6f), btVector3(0.0f, 0.0f, 0.0f));
	CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(1.0f, 0.2f, 0.2f), btVector3(0.0f, 10.0f, 0.0f));
	CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.0f, 0.2f, 0.8f), btVector3(1.25f, 20.0f, 0.0f));

}

void InitializePhysics()
{
	// create the collision configuration
	m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
	// create the dispatcher
	m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
	// create the broadphase
	m_pBroadphase = new btDbvtBroadphase();
	// create the constraint solver
	m_pSolver = new btSequentialImpulseConstraintSolver();
	// create the world
	m_pWorld = new btDiscreteDynamicsWorld(m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);
    Vector3 _gravity=(Vector3){btScalar(0.0), btScalar(-9.8), btScalar(0.0)};
    m_pWorld->setGravity(BV(_gravity));

    m_pWorld->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;

	_debugDrawer = new DebugDrawer();
    m_pWorld->setDebugDrawer(_debugDrawer);

/*

    btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50),btScalar(0.2f),btScalar(50)));
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,-1,0));
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(0),myMotionState,groundShape,btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(rbInfo);
    m_pWorld->addRigidBody(body);
*/
	// create our scene's physics objects
	CreateObjects();

}

void ShutdownPhysics()
{
	delete m_pWorld;
	delete m_pSolver;
	delete m_pBroadphase;
	delete m_pDispatcher;
	delete m_pCollisionConfiguration;
	delete _debugDrawer;
}




	void RenderScene()
{
		btScalar transform[16];

	// iterate through all of the objects in our world
	for(GameObjects::iterator i = m_objects.begin(); i != m_objects.end(); ++i)
	{
		// get the object from the iterator
		GameObject* pObj = *i;

		// read the transform
		pObj->GetTransform(transform);

		// get data from the object and draw it
		DrawShape(transform, pObj->GetShape(), pObj->GetColor());
	}

		  m_pWorld->debugDrawWorld();
	}

	void UpdateScene(float dt)
 {
		// check if the world object exists
		if (m_pWorld) {
			// step the simulation through time. This is called
			// every update and the amount of elasped time was
			// determined back in ::Idle() by our clock object.
			m_pWorld->stepSimulation(dt);
		}
	}



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

    InitializePhysics();


    SetCameraMode(camera, CAMERA_FREE); // Set a first person camera mode

    SetTargetFPS(60);                           // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())                // Detect window close button or ESC key
    {
        // Update
                if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
                if (IsKeyReleased(KEY_SPACE))
                {

                CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.0f, 0.2f, 0.8f), btVector3(1.25f, 20.0f, 0.0f));
                }

        //----------------------------------------------------------------------------------
        UpdateCamera(&camera);                  // Update camera
        //----------------------------------------------------------------------------------

        UpdateScene(GetFrameTime());

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground((Color){0,0,45,0});

            BeginMode3D(camera);



           DrawGrid(10, 1.0f);         // Draw a grid
           RenderScene();

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

    ShutdownPhysics();
    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
