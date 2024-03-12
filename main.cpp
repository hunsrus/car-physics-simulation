#define RLIGHTS_IMPLEMENTATION      //Importante para que defina las funciones de rlights y eso
#define PLATFORM_DESKTOP
#include "raylib.h"
#include "raymath.h"
#include "rlights.h"
#include "rlgl.h"

#include <iostream>
#include <stdio.h>

#include "src/btBulletDynamicsCommon.h"

#if defined(PLATFORM_DESKTOP)
    #define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
    #define GLSL_VERSION            100
#endif

#define TARGET_FPS 60

#define CHASIS_WIDTH 5.0f
#define CHASIS_LENGHT 10.0f
#define CHASIS_HEIGHT 0.5f

#define BODY_WIDTH 5.0f
#define BODY_LENGHT 10.0f
#define BODY_HEIGHT 3.5f

#define FLOOR_WIDTH 1000.0f
#define FLOOR_LENGTH 1000.0f
#define FLOOR_HEIGHT 3.0f

typedef struct Gear
{
	float torque;
	float topVel;
}Gear;

static float suspensionHeight = 1.0f;
static float suspensionRadius = 0.2f;

static bool SHOW_FPS = true;
static bool RUN_SIMULATION = false;
static bool EXIT = false;
static bool GAMEPAD_CONNECTED = false;

static float gEngineForce = 0.f;

static float defaultBreakingForce = 10.f;
static float gBreakingForce = 100.f;

static float maxEngineForce = 1000.f;  //this should be engine/velocity dependent
//static float	maxBreakingForce = 100.f;

static float gVehicleSteering = 0.f;
static float steeringIncrement = 0.04f;
static float steeringClamp = 0.3f;
static float wheelRadius = 0.8f;
static float wheelWidth = 1.0f;

btDiscreteDynamicsWorld* dynamicsWorld;

btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape);

#include <fcntl.h>
#include <unistd.h>
#include <linux/types.h>
#include <thread>

#define JS_EVENT_BUTTON		0x01	/* button pressed/released */
#define JS_EVENT_AXIS		0x02	/* joystick moved */
#define JS_EVENT_INIT		0x80	/* initial state of device */

/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

struct js_event {
	__u32 time;	/* event timestamp in milliseconds */
	__s16 value;	/* value */
	__u8 type;	/* event type */
	__u8 number;	/* axis/button number */
};

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes = 0;
	fd_set set;
	struct timeval timeout;

	FD_ZERO(&set); /* clear the set */
	FD_SET(fd, &set); /* add our file descriptor to the set */

	timeout.tv_sec = 0;
  	timeout.tv_usec = 10000;

	int rv = select(fd + 1, &set, NULL, NULL, &timeout);
	if(rv == -1)
	{
		perror("select"); /* an error accured */
		return -1;
	}
	else if(rv == 0)
	{
		//printf("timeout"); /* a timeout occured */
		return 1;
	}
	else
		bytes = read(fd, event, sizeof(*event)); /* there was data to read */

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

bool ACCELERATE = false;
bool BRAKE = false;
bool HANDBRAKE = false;
int WHEEL_DIRECTION = 0;
int GEAR_SHIFT = 0;

void handleGamepad()
{	
	const char *device = "/dev/input/js0";
	int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;
	int gamepad_status = -1; //-1: desconecado, 0: evento recibido, 1: conectado pero sin evento (timeout)

	while(!EXIT)
	{
		if(gamepad_status == -1)
		{
			js = open(device, O_RDONLY);

			if (js == -1)
			{
				//perror("Could not open joystick");
			}
			else
			{
				gamepad_status = 0;
			}
		}else
		{
			gamepad_status = read_event(js, &event);
		}

		if(gamepad_status == 0)
		{
			switch (event.type)
			{
				case JS_EVENT_BUTTON:
					if((event.number == 7) && (event.value == 1)) ACCELERATE = true;
					if((event.number == 7) && (event.value == 0)) ACCELERATE = false;
					if((event.number == 6) && (event.value == 1)) BRAKE = true;
					if((event.number == 6) && (event.value == 0)) BRAKE = false;
					if((event.number == 0) && (event.value == 1)) GEAR_SHIFT = 1;
					if((event.number == 2) && (event.value == 1)) GEAR_SHIFT = -1;
					if((event.number == 9) && (event.value == 1)) RUN_SIMULATION = !RUN_SIMULATION;

					if((event.number == 8) && (event.value == 1)) EXIT = true;
					
					//printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
					break;
				case JS_EVENT_AXIS:
					axis = get_axis_state(&event, axes);
					if(axis == 0) WHEEL_DIRECTION = axes[axis].x;
					if (axis < 3)
						//printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
					break;
				default:
					/* Ignore init events. */
					break;
			}
		}
	}
	
	close(js);
}

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1366;
    const int screenHeight = 768;

    int i;
    char c[100];
    Vector3 bodyPos[20];
	btScalar yaw, pitch, roll;

	int topGear = 5, currentGear = 0;
	Gear gears[topGear+1];

	gears[0] = {400,0};	// punto muerto
	gears[1] = {7000,40};
	gears[2] = {3000,80};
	gears[3] = {2000,120};
	gears[4] = {800,180};
	gears[5] = {500,270};

	btVector3 chasisSize = {CHASIS_WIDTH, CHASIS_HEIGHT, CHASIS_LENGHT};

	btAlignedObjectArray<btCollisionShape*> collisionShapes;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* broadphase;
	btConstraintSolver* solver;
	btRigidBody* carChassis;
	btCollisionShape* wheelShape;

	//	MUNDO

	btCollisionShape* groundShape = new btBoxShape(btVector3(FLOOR_WIDTH/2, FLOOR_HEIGHT/2, FLOOR_LENGTH/2));
	collisionShapes.push_back(groundShape);

	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	broadphase = new btAxisSweep3(worldMin, worldMax);

	solver = new btSequentialImpulseConstraintSolver();

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;  //for direct solver, it is better to solve multiple objects together, small batches have high overhead
	dynamicsWorld->getSolverInfo().m_numIterations = 100;

	dynamicsWorld->setGravity(btVector3(0,-90.8,0));
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -3, 0));

	//either use heightfield or triangle mesh

	//create ground object
	btRigidBody *ground = localCreateRigidBody(0, tr, groundShape);
	ground->setFriction(1);

	//CAHSIS
	btCollisionShape* chassisShape = new btBoxShape(chasisSize/2.0f);
	collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 0, 0));

	compound->addChildShape(localTrans, chassisShape);

	{
		btCollisionShape* suppShape = new btBoxShape(btVector3(BODY_WIDTH/2, 1, BODY_LENGHT/2));
		btTransform suppLocalTrans;
		suppLocalTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		//suppLocalTrans.setOrigin(btVector3(0, CHASIS_HEIGHT+1.0f, 2.5));
		suppLocalTrans.setOrigin(btVector3(0, BODY_HEIGHT/2, 0));
		compound->addChildShape(suppLocalTrans, suppShape);
	}

	const btScalar FALLHEIGHT = CHASIS_HEIGHT+suspensionHeight;
	tr.setOrigin(btVector3(0, FALLHEIGHT, 0));

	const btScalar chassisMass = 140.0f;
	const btScalar bodyMass = 0.1f;
	const btScalar wheelMass = 5.0f;
	carChassis = localCreateRigidBody(chassisMass, tr, compound);  //chassisShape);
	//carChassis->setCollisionFlags(carChassis->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
	//carChassis->setDamping(0.2,0.2);
	carChassis->setFriction(2);

	//RUEDAS
	//wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

	btVector3 wheelPos[4] = {
		btVector3(btScalar(-CHASIS_WIDTH/2.5), btScalar(FALLHEIGHT-suspensionHeight), btScalar(CHASIS_LENGHT/3.3f)),
		btVector3(btScalar(CHASIS_WIDTH/2.5), btScalar(FALLHEIGHT-suspensionHeight), btScalar(CHASIS_LENGHT/3.3f)),
		btVector3(btScalar(CHASIS_WIDTH/2.5), btScalar(FALLHEIGHT-suspensionHeight), btScalar(-CHASIS_LENGHT/3.0f)),
		btVector3(btScalar(-CHASIS_WIDTH/2.5), btScalar(FALLHEIGHT-suspensionHeight), btScalar(-CHASIS_LENGHT/3.0f))};

	for (int i = 0; i < 4; i++)
	{
		// create a Hinge2 joint
		// create two rigid bodies
		// static bodyA (parent) on top:

		btRigidBody* pBodyA = carChassis;
		pBodyA->setActivationState(DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(wheelPos[i]);

		btRigidBody* pBodyB = createRigidBody(wheelMass, tr, wheelShape);
		//pBodyB->setFriction(1110);
		//pBodyB->setFriction(1.7);
		pBodyB->setFriction(0.8);
		pBodyB->setContactStiffnessAndDamping(200000.0f,10.0f);	//afecta los rebotes raros
		// add some data to build constraint frames
		btVector3 parentAxis(0.f, 1.f, 0.f);
		btVector3 childAxis(1.f, 0.f, 0.f);
		btVector3 anchor = tr.getOrigin();

		
		// RUEDAS DELANTERAS DIRECCIONALES

		btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);

		//m_guiHelper->get2dCanvasInterface();

		// steering limits
		pHinge2->setLowerLimit(-SIMD_HALF_PI * 0.4f);
		pHinge2->setUpperLimit(SIMD_HALF_PI * 0.4f);
		
		// add constraint to world
		dynamicsWorld->addConstraint(pHinge2, true);

		// Drive engine.
		pHinge2->enableMotor(3, true);
		pHinge2->setMaxMotorForce(3, 100);
		pHinge2->setTargetVelocity(3, 0);
	

		// Steering engine.
		pHinge2->enableMotor(5, true);
		pHinge2->setMaxMotorForce(5, 10000);
		pHinge2->setTargetVelocity(5, 4);
		pHinge2->setServo(5,true);

		pHinge2->setParam( BT_CONSTRAINT_CFM, 0.15f, 2 );
		//pHinge2->setParam( BT_CONSTRAINT_CFM, 100.5f, 2 );
		//pHinge2->setParam( BT_CONSTRAINT_STOP_CFM, 1.0f, 2 );
		pHinge2->setParam( BT_CONSTRAINT_ERP, 0.35f, 2 );

		pHinge2->setLimit(2, 0.5,1.0);	//CHEQUEAR. así anda bien (para que la suspensión no se extienda exageradamente para abajo)

		pHinge2->setDamping( 2, 50 );
		pHinge2->setStiffness( 2, 40 );

		pHinge2->setDbgDrawSize(btScalar(5.f));
	}

	//CARROCERÍA
	// {
	// 	btCollisionShape* bodyShape = new btBoxShape(btVector3(BODY_WIDTH/2, BODY_HEIGHT/2, BODY_LENGHT/2));
		
	// 	btTransform tr;
	// 	tr.setIdentity();
	// 	//localTrans effectively shifts the center of mass with respect to the chassis
	// 	tr.setOrigin(btVector3(0, FALLHEIGHT, 0));

	// 	btRigidBody* bodyBody = createRigidBody(bodyMass, tr, bodyShape);
	// 	bodyBody->setCollisionFlags(carChassis->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		
	// 	btRigidBody* pBodyA = carChassis;
	// 	pBodyA->setActivationState(DISABLE_DEACTIVATION);
		
	// 	btTransform frameA;
	// 	frameA.setIdentity();
	// 	frameA.setOrigin(btVector3(0,BODY_HEIGHT+suspensionHeight,0));
	// 	btTransform frameB;
	// 	frameB.setIdentity();
	// 	frameB.setOrigin(btVector3(0,0,0));

	// 	btFixedConstraint* fix = new btFixedConstraint(*pBodyA, *bodyBody, frameA, frameB);

	// 	dynamicsWorld->addConstraint(fix, true);
	// }

	//resetForklift()
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	carChassis->setLinearVelocity(btVector3(0, 0, 0));
	carChassis->setAngularVelocity(btVector3(0, 0, 0));
	dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(carChassis->getBroadphaseHandle(), dynamicsWorld->getDispatcher());

    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "bullet car test");

	// CARGAR LOS MODELOS DESPUÉS DE INICIAR LA VENTANA
	Model* bodyModel = new Model(LoadModel(std::string("../models/focus.obj").c_str()));
	bodyModel->materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(LoadImage(std::string("../models/fordfocus128.png").c_str()));
	Model* chasisModel = new Model(LoadModelFromMesh(GenMeshCube(CHASIS_WIDTH,CHASIS_HEIGHT,CHASIS_LENGHT)));
	Model* suspensionModel = new Model(LoadModelFromMesh(GenMeshCylinder(suspensionRadius,suspensionHeight,10)));
	Model* wheel1Model = new Model(LoadModel(std::string("../models/tyre/Scene.gltf").c_str()));
	wheel1Model->materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(LoadImage(std::string("../models/tyre/tyre_base.png").c_str()));
	Model* wheel2Model = new Model(LoadModel(std::string("../models/tyre/Scene.gltf").c_str()));
	wheel2Model->materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(LoadImage(std::string("../models/tyre/tyre_base.png").c_str()));
	Model* wheel3Model = new Model(LoadModel(std::string("../models/tyre/Scene.gltf").c_str()));
	wheel3Model->materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(LoadImage(std::string("../models/tyre/tyre_base.png").c_str()));
	Model* wheel4Model = new Model(LoadModel(std::string("../models/tyre/Scene.gltf").c_str()));
	wheel4Model->materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = LoadTextureFromImage(LoadImage(std::string("../models/tyre/tyre_base.png").c_str()));

       // Define the camera to look into our 3d world
    Camera camera = { {-20.0f, 10.0f, 0.0f}, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    camera.fovy = 80.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Create a RenderTexture2D to be used for render to texture
    RenderTexture2D target = LoadRenderTexture(screenWidth, screenHeight);

    SetCameraMode(camera, CAMERA_THIRD_PERSON);
	//SetCameraMode(camera, CAMERA_ORBITAL);
    HideCursor();

	std::thread t1(handleGamepad);

    SetTargetFPS(TARGET_FPS);
    //--------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------
    while (!EXIT)
    {
        //update image----------------------------------------
        UpdateCamera(&camera);      // Actualizar camara interna y mi camara

		if(RUN_SIMULATION)
		{
			dynamicsWorld->stepSimulation(1.f / TARGET_FPS, 10);
		}

		Vector3 orientation = Vector3Normalize(Vector3Subtract(bodyPos[6],bodyPos[1]));

		btHinge2Constraint* hingeWTI = (btHinge2Constraint*)dynamicsWorld->getConstraint(0);
		btHinge2Constraint* hingeWTD = (btHinge2Constraint*)dynamicsWorld->getConstraint(1);
		btHinge2Constraint* hingeWDI = (btHinge2Constraint*)dynamicsWorld->getConstraint(2);
		btHinge2Constraint* hingeWDD = (btHinge2Constraint*)dynamicsWorld->getConstraint(3);
		// hingeWDI->setTargetVelocity(5,0.0f);
		// hingeWDD->setTargetVelocity(5,0.0f);
		hingeWDI->setServoTarget(5,0);
		hingeWDD->setServoTarget(5,0);
		//hingeWDI->setMaxMotorForce(3, 1000);
		//hingeWDD->setMaxMotorForce(3, 1000);
		hingeWDI->setMaxMotorForce(3, gears[currentGear].torque/2);
		hingeWDD->setMaxMotorForce(3, gears[currentGear].torque/2);
		hingeWDI->setTargetVelocity(3,0.0f);
		hingeWDD->setTargetVelocity(3,0.0f);

		hingeWDI->setServoTarget(5,WHEEL_DIRECTION/32767);
		hingeWDD->setServoTarget(5,WHEEL_DIRECTION/32767);
		
		// umbral de ruptura del constraint (pasado ese impulso se sueltan las ruedas)
		//hingeWTI->setBreakingImpulseThreshold(100);
		//hingeWTD->setBreakingImpulseThreshold(100);
		hingeWTI->enableMotor(3,false);
		hingeWTD->enableMotor(3,false);

		if(IsKeyDown(KEY_ESCAPE))
		{
			EXIT = true;
		}

		if(IsKeyDown(KEY_T))
		{
			btRigidBody* body = btRigidBody::upcast(dynamicsWorld->getCollisionObjectArray()[1]);

			body->applyForce((btVector3){0.0f,100000.0f,0.0f},(btVector3){0,0,0});

		}

		if(IsKeyPressed(KEY_R))
		{
			RUN_SIMULATION = !RUN_SIMULATION;
		}

		if(IsKeyPressed(KEY_UP) || GEAR_SHIFT > 0)
		{
			if(currentGear < topGear) currentGear++;
			GEAR_SHIFT = 0;
		}

		if(IsKeyPressed(KEY_DOWN) || GEAR_SHIFT < 0)
		{
			if(currentGear > 0) currentGear--;
			GEAR_SHIFT = 0;
		}

		if(IsKeyDown(KEY_SPACE) || HANDBRAKE)
		{
			hingeWTI->enableMotor(3,true);
			hingeWTI->setMaxMotorForce(3,100000);
			hingeWTI->setTargetVelocity(3,0);
			hingeWTD->enableMotor(3,true);
			hingeWTD->setMaxMotorForce(3,100000);
			hingeWTD->setTargetVelocity(3,0);
		}

		if(IsKeyDown(KEY_W) || ACCELERATE)
		{
			hingeWDI->setTargetVelocity(3,gears[currentGear].topVel);
			hingeWDD->setTargetVelocity(3,gears[currentGear].topVel);
			hingeWDI->setMaxMotorForce(3, gears[currentGear].torque);
			hingeWDD->setMaxMotorForce(3, gears[currentGear].torque);
		}

		if(IsKeyDown(KEY_S) || BRAKE)
		{
			hingeWDI->setTargetVelocity(3,-gears[currentGear].topVel);
			hingeWDD->setTargetVelocity(3,-gears[currentGear].topVel);
			hingeWDI->setMaxMotorForce(3, gears[currentGear].torque);
			hingeWDD->setMaxMotorForce(3, gears[currentGear].torque);
		}

		if(IsKeyDown(KEY_A))
		{
			//hingeWDI->setTargetVelocity(5,-4.0f);
			//hingeWDD->setTargetVelocity(5,-4.0f);
			hingeWDI->setServoTarget(5,-0.5);
			hingeWDD->setServoTarget(5,-0.5);
		}

		if(IsKeyDown(KEY_D))
		{
			//hingeWDI->setTargetVelocity(5,4.0f);
			//hingeWDD->setTargetVelocity(5,4.0f);
			hingeWDI->setServoTarget(5,1);
			hingeWDD->setServoTarget(5,1);
		}

		if(IsKeyPressed(KEY_F))
		{
			//create a dynamic rigidbody

			//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
			btCollisionShape* colShape = new btSphereShape(btScalar(2.));
			collisionShapes.push_back(colShape);

			/// Create Dynamic Objects
			btTransform startTransform;
			startTransform.setIdentity();

			btScalar mass(100.f);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0, 0, 0);
			if (isDynamic)
				colShape->calculateLocalInertia(mass, localInertia);

			startTransform.setOrigin(btVector3(bodyPos[1].x, bodyPos[1].y+50, bodyPos[1].z));

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
			btRigidBody* body = new btRigidBody(rbInfo);

			dynamicsWorld->addRigidBody(body);
		}

		//print positions of all objects
		for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			btTransform trans;
			if (body && body->getMotionState())
			{
				body->getMotionState()->getWorldTransform(trans);
			}
			else
			{
				trans = obj->getWorldTransform();
			}
			
			bodyPos[j] = {float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ())};
			
			trans.getBasis().getEulerYPR(yaw, pitch, roll);
			if(j == 1)
			{
				chasisModel->transform = MatrixRotateXYZ((Vector3){ -roll, -pitch, -yaw });
				bodyModel->transform = MatrixRotateXYZ((Vector3){ -roll, -pitch, -yaw });
			}else if(j == 2)
			{
				wheel1Model->transform = MatrixRotateXYZ((Vector3){ roll, -pitch+PI, -yaw });	//invierto roll y le sumo PI al pitch para girar el modelo
			}else if(j == 3)
			{
				wheel2Model->transform = MatrixRotateXYZ((Vector3){ -roll, -pitch, -yaw });
			}else if(j == 4)
			{
				wheel3Model->transform = MatrixRotateXYZ((Vector3){ -roll, -pitch, -yaw });
			}else if(j == 5)
			{
				wheel4Model->transform = MatrixRotateXYZ((Vector3){ roll, -pitch+PI, -yaw });	//invierto roll y le sumo PI al pitch para girar el modelo
			}else if(j == 6)
			{
				//bodyModel->transform = MatrixRotateXYZ((Vector3){ -roll, -pitch, -yaw });
			}
			//printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
		}

		//camera.position = (Vector3){bodyPos[1].x,bodyPos[1].y+5,bodyPos[1].z};
		//camera.position = Vector3Add(camera.position, Vector3Scale(Vector3Normalize(Vector3Subtract(bodyPos[1],bodyPos[6])),20.0f));
		//camera.position = Vector3Add(camera.position,(Vector3){0,10.0f,0});
		//camera.position = Vector3AddValue(camera.position,10.0f);
		camera.target = bodyPos[1];
       
        //----------------------------------------------------------------------------------
        // Dibuja
        //----------------------------------------------------------------------------------

        ClearBackground((Color){113,165,212,255});  // Clear texture background

        BeginTextureMode(target);       // Enable drawing to texture
            ClearBackground((Color){113,165,212,255});  // Clear texture background
            BeginMode3D(camera);        // Begin 3d mode drawing
				// piso
				DrawCube(bodyPos[0],FLOOR_WIDTH,FLOOR_HEIGHT,FLOOR_LENGTH,BROWN);
				// chasis
				//DrawCube(bodyPos[1],10.0f,2.0f,10.0f,RED);
				//DrawModel(*chasisModel,bodyPos[1],1.0f,BLACK);
				DrawModel(*bodyModel,bodyPos[1],1.0f,WHITE);
				// ruedas
				//DrawCylinderEx((Vector3){bodyPos[2].x-wheelWidth/2.0f,bodyPos[2].y,bodyPos[2].z},(Vector3){bodyPos[2].x+wheelWidth/2.0f,bodyPos[2].y,bodyPos[2].z},wheelRadius,wheelRadius,10,GREEN);
				DrawModel(*wheel1Model,bodyPos[2],wheelRadius*3.0f,WHITE);	//rueda trasera
				DrawModel(*wheel2Model,bodyPos[3],wheelRadius*3.0f,WHITE);	//rueda trasera
				DrawModel(*wheel3Model,bodyPos[4],wheelRadius*3.0f,WHITE);	//rueda delantera
				DrawModel(*wheel4Model,bodyPos[5],wheelRadius*3.0f,WHITE);	//rueda delantera
				//DrawSphere(bodyPos[8],2.0f,GREEN);
				//DrawCylinderEx((Vector3){bodyPos[2].x-0.25f,bodyPos[2].y,bodyPos[2].z},(Vector3){bodyPos[2].x+0.25f,bodyPos[2].y,bodyPos[2].z},2.5f,2.5f,10,GREEN);
				//DrawCylinderEx((Vector3){bodyPos[3].x-0.25f,bodyPos[3].y,bodyPos[3].z},(Vector3){bodyPos[3].x+0.25f,bodyPos[3].y,bodyPos[3].z},2.5f,2.5f,10,YELLOW);
				//DrawCylinderEx((Vector3){bodyPos[4].x-0.25f,bodyPos[4].y,bodyPos[4].z},(Vector3){bodyPos[4].x+0.25f,bodyPos[4].y,bodyPos[4].z},2.5f,2.5f,10,BLUE);
				//DrawCylinderEx((Vector3){bodyPos[5].x-0.25f,bodyPos[5].y,bodyPos[5].z},(Vector3){bodyPos[5].x+0.25f,bodyPos[5].y,bodyPos[5].z},2.5f,2.5f,10,ORANGE);
                //DrawSphere(bodyPos[4],2.0f,WHITE);
				for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 6; j--)
				{	
					btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
					btRigidBody* body = btRigidBody::upcast(obj);
					btTransform trans;
					if (body && body->getMotionState())
					{
						body->getMotionState()->getWorldTransform(trans);
					}
					else
					{
						trans = obj->getWorldTransform();
					}
					DrawSphere((Vector3){float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ())},2.0f,WHITE);
				}
                DrawGrid(20,20.0f);
                //DrawPlane((Vector3){0.0f,0.0f,0.0f},(Vector2){100.0f,100.0f},WHITE);
            EndMode3D();                // End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode();               // End drawing to texture (now we have a texture available for next passes)

        BeginDrawing();
            ClearBackground((Color){113,165,212,255});
                // BeginShaderMode(shader_pixel);
                    // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                    DrawTextureRec(target.texture, (Rectangle){ 0, 0, (float)target.texture.width, (float)-target.texture.height }, (Vector2){ 0, 0 }, WHITE);
                // EndShaderMode();
            
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[1];
			btRigidBody* body = btRigidBody::upcast(obj);
			
			sprintf(c,"VEL %.2f",(float)body->getLinearVelocity().length());
			DrawText(c, 30, screenHeight-120, 30, BLACK);
			sprintf(c,"GEAR %d",currentGear);
			DrawText(c, 30, screenHeight-60, 30, BLACK);

            if(SHOW_FPS)
            {
                sprintf(c,"FPS %d",GetFPS());
                DrawText(c, 30, 30, 30, BLACK);
            }
        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    CloseWindow();
    //--------------------------------------------------------------------------------------
	
	UnloadModel(*chasisModel);

    //cleanup in the reverse order of creation/initialization

	///-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete dynamicsWorld->getBroadphase()->getOverlappingPairCache();

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();


	std::cout << "Joining gamepad thread...";
	while(!t1.joinable()){}
	t1.join();
	std::cout << " Done." << std::endl;

    return 0;
}

Vector3 Vector3Bullet2Raylib(btVector3 btVector)
{
	return (Vector3){btVector.getX(), btVector.getY(), btVector.getZ()};
}

btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
	body->setWorldTransform(startTransform);
#endif  //

	dynamicsWorld->addRigidBody(body);
	return body;
}

btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
	body->setWorldTransform(startTransform);
#endif  //

	body->setUserIndex(-1);
	dynamicsWorld->addRigidBody(body);
	return body;
}