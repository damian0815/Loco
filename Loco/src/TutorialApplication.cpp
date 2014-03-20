/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "TutorialApplication.h"

#include "OgreBulletDynamicsRigidBody.h"
#include "OgreBulletCollisionsShape.h"
#include "Shapes/OgreBulletCollisionsStaticPlaneShape.h"
#include "Shapes/OgreBulletCollisionsBoxShape.h"
#include <Ogre/OgreMath.h>

#include "ForwardDynamicsSkeletonController.h"
#include "VirtualModelSkeletonController.h"
#include "Utilities.h"

using namespace std;

static const bool PHYSICS_DEBUG_SHAPES = true ;
static const double PHYSICS_FRAME_DURATION = 1.0/180.0;
static const bool DO_ANIMATION = true;
static const std::string DEFAULT_ANIMATION = "<none>";


static const std::string SKELETON_CONTROLLER_FILENAME = "swimmer.posecontroller";



static const float DEFAULT_TIME_MULTIPLIER = 0.0f;
static const float DEFAULT_ANIMATION_SPEED = 1.0f;

static const Ogre::Vector3 ROOT_POSITION = Ogre::Vector3(0,1.5,0);

//#define FLOOR_IS_PLANE

Ogre::SceneNode* createBox(Ogre::SceneManager* sceneMgr, Ogre::String name);

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
: mWorld(NULL), debugDrawer(NULL), mMovingX(false), mMovingY(false), mMovingZ(false), mAnimationState(0), mGroundPlaneBody(0)
{
	arc4random_stir();
}

//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
	for ( std::deque<OgreBulletDynamics::RigidBody*>::iterator it = mRigidBodies.begin(); mRigidBodies.end() != it; ++it ) {
		delete *it;
	}
	for (std::deque<OgreBulletCollisions::CollisionShape*>::iterator it = mShapes.begin(); mShapes.end() != it; ++it) {
		delete *it;
	}
	mRigidBodies.clear();
	mShapes.clear();
	if ( mWorld ) {
		mWorld->setDebugDrawer(0);
		delete mWorld;
	}
	if ( debugDrawer ) {
		delete debugDrawer;
	}
	
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createCamera(void)
{
	mCamera = mSceneMgr->createCamera("PlayerCam");
	/*
	mCamera->setPosition(Ogre::Vector3(0,18,70));
	mCamera->lookAt(Ogre::Vector3(0,0,0));*/
	mCamera->setFOVy(Ogre::Degree(40));
	mCamera->setNearClipDistance(0.01);
	mCamera->setFarClipDistance(10.0f);
	
	mCamera->setPosition(0.9f,1.70f,-3.0f);
	mCamera->pitch(Ogre::Degree(-30.0f));
	mCamera->yaw(Ogre::Degree(170.0f));
	
	mCameraMan = new OgreBites::SdkCameraMan(mCamera);
	mCameraMan->setTopSpeed( 0.5f );
}


void TutorialApplication::createFrameListener(void) {
	BaseApplication::createFrameListener();
	
	
	// create a params panel for displaying sample details
    Ogre::StringVector items;
   // items.push_back("Filtering");
   // items.push_back("Poly Mode");

    mStatusPanel = mTrayMgr->createParamsPanel(OgreBites::TL_BOTTOMRIGHT, "Params", 300, items);
	
	OgreBites::Slider* slider = NULL;

	mTrayMgr->createButton(OgreBites::TL_BOTTOMRIGHT, "Reset pose", "Reset pose");
	mTrayMgr->createButton(OgreBites::TL_BOTTOMRIGHT, "Reload controller", "Reload controller");
	mTrayMgr->createButton(OgreBites::TL_BOTTOMRIGHT, "Go to pose", "Go to pose");
	

	// animations control
	slider = mTrayMgr->createLongSlider(OgreBites::TL_TOPLEFT, "Gravity X", "Gravity X", 400, 150, 50, -9.8, 9.8, 149 );
	slider->setValue(0);
	slider = mTrayMgr->createLongSlider(OgreBites::TL_TOPLEFT, "Gravity Y", "Gravity Y", 400, 150, 50, -9.8, 9.8, 149 );
	slider->setValue(0.0f);
	slider = mTrayMgr->createLongSlider(OgreBites::TL_TOPLEFT, "Gravity Z", "Gravity Z", 400, 150, 50, -9.8, 9.8, 149 );
	slider->setValue(0.0f);
	slider = mTrayMgr->createLongSlider(OgreBites::TL_TOPLEFT, "Time multiplier", "Time multiplier", 400, 150, 50, 0, 1, 1000 );
	slider->setValue(DEFAULT_TIME_MULTIPLIER);
	slider = mTrayMgr->createLongSlider(OgreBites::TL_TOPLEFT, "Animation speed", "Animation speed", 400, 150, 50, 0, 2, 1000 );
	slider->setValue(DEFAULT_ANIMATION_SPEED);
	
	
	// animation selector
	int defaultAnimationIndex = 0;
	Ogre::SkeletonInstance* skel = mFigureEnt->getSkeleton();
	
	Ogre::StringVector animationNames;
	animationNames.push_back("<none>");
	if ( skel ) {
		std::cout << "got skeleton" << std::endl;
		for ( int i=0; i<skel->getNumAnimations(); i++ ) {
			std::string name = skel->getAnimation(i)->getName();
			std::cout << " animation: " << name << std::endl;
			if ( /*i<5 || name == "WalkNew"*/true )
			{
				animationNames.push_back(name);
				if ( name == DEFAULT_ANIMATION ) {
					defaultAnimationIndex = i;
				}
			}
		}
		//mFigureEnt->setDisplaySkeleton(true);
	} else {
		std::cout << "no skeleton" << std::endl;
	}
	mAnimationSelectMenu = mTrayMgr->createThickSelectMenu( OgreBites::TL_TOPLEFT, "Animation", "Animation", 400, animationNames.size());
	if ( animationNames.size() ) {
		mAnimationSelectMenu->setItems(animationNames);
		mAnimationSelectMenu->selectItem(defaultAnimationIndex);
	}
	
	static const int debugDrawStyleCount = 8;
	OgreBites::SelectMenu* select = mTrayMgr->createThickSelectMenu( OgreBites::TL_BOTTOMRIGHT, "Debug draw", "Debug draw", 400, debugDrawStyleCount );
	Ogre::StringVector debugDrawStates;
	const char* debugDrawStateNames[debugDrawStyleCount] =  { "None", "Nodes", "Links", "Faces", "FaceAnchors", "Tetras", "TetraForces", "BadTetras" };
	for ( int i=0; i<debugDrawStyleCount; i++ ) {
		debugDrawStates.push_back(debugDrawStateNames[i]);
	}
	select->setItems(debugDrawStates);
	select->selectItem(0);
	
	
	showControls(false);
	
}

void TutorialApplication::showControls( bool show ) {
	if ( show ) {
		mTrayMgr->getTrayContainer(OgreBites::TL_BOTTOMRIGHT)->show();
		mTrayMgr->getTrayContainer(OgreBites::TL_TOPLEFT)->show();
		mTrayMgr->showCursor();
		mCameraMan->setStyle(OgreBites::CS_MANUAL);
	} else {
		mTrayMgr->getTrayContainer(OgreBites::TL_BOTTOMRIGHT)->hide();
		mTrayMgr->getTrayContainer(OgreBites::TL_TOPLEFT)->hide();
		mTrayMgr->hideCursor();
		mCameraMan->setStyle(OgreBites::CS_FREELOOK);
	}
}

#pragma mark Respond to GUI

void TutorialApplication::loadSkeletonController(std::string filename)
{
	// try to lead settings
	TextFilePtr poseJsonFile = mTextFileManager->load( filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
	const Ogre::String& poseJson = poseJsonFile->getString();
	if ( poseJson.length()>0 ) {
		picojson::value jsonRootVal;
		string err;
		picojson::parse( jsonRootVal, poseJson.c_str(), poseJson.c_str()+poseJson.length(), &err);
		if ( !err.empty() ) {
			BLog("Error parsing json: %s", err.c_str());
		} else {
			
			bool shouldEnable = false;
			bool shouldDoDebugDraw = false;
			if ( !mSkeletonController.isNull() ) {
				shouldEnable = mSkeletonController->isEnabled();
				shouldDoDebugDraw = mSkeletonController->getShowDebugInfo();
			}
			
			// delete the old skeleton controller
			mSkeletonController.setNull();
			// load a new one
			mSkeletonController = Ogre::SharedPtr<SkeletonController>( new VirtualModelSkeletonController(mFigureNode, mFigureEnt->getSkeleton(), mWorld, mGroundPlaneBody, jsonRootVal) );
			
			//picojson::object jsonRoot = jsonRootVal.get<picojson::object>();
			// create forward dynamics for some reason
			//((ForwardDynamicsSkeletonController*)mSkeletonController.get())->createForwardDynamicsSkeleton(jsonRoot["forwardDynamics"] );
			
			mSkeletonController->setEnabled(shouldEnable);
			mSkeletonController->setShowDebugInfo(shouldDoDebugDraw);
		}

	} else {
		OgreAssert(false, "Can't find .posecontroller file");
	}
	
	// don't cache the tetx file
	mTextFileManager->unload(filename);
}

void TutorialApplication::buttonHit(OgreBites::Button *button)
{
	if ( button->getName() == "Reload controller" ) {
		
		// reset the skeleton to rest pos
		ForwardDynamicsSkeletonController* poseController = dynamic_cast<ForwardDynamicsSkeletonController*>(mSkeletonController.get());
		if ( poseController ) {
			poseController->reset();
		}
		
		// reload
		loadSkeletonController(SKELETON_CONTROLLER_FILENAME);
		
	} else if ( button->getName() == "Reset pose" ) {
		ForwardDynamicsSkeletonController* poseController = dynamic_cast<ForwardDynamicsSkeletonController*>(mSkeletonController.get());
		if ( poseController ) {
			poseController->reset();
		}
	} else if ( button->getName() == "Go to pose" ) {
		if ( mAnimationState ) {
			mAnimationState->setEnabled(false);
			mAnimationState = NULL;
		}
		mSkeletonController->setEnabled(true);
		mSkeletonController->setShowDebugInfo( true	 );
	}
}

void TutorialApplication::sliderMoved(OgreBites::Slider* slider)
{

	if ( slider->getName() == "Time multiplier" ) {
		mTimeMultiplier = slider->getValue();
	} else if ( slider->getName() == "Animation speed" ) {
		mAnimationSpeed = slider->getValue();
	} else if ( slider->getName() == "Gravity X" ) {
		Ogre::Vector3 g = mWorld->getGravity();
		g.x = slider->getValue();
		mWorld->setGravity(g);
	} else if ( slider->getName() == "Gravity Y" ) {
		Ogre::Vector3 g = mWorld->getGravity();
		g.y = slider->getValue();
		mWorld->setGravity(g);
	} else if ( slider->getName() == "Gravity Z" ) {
		Ogre::Vector3 g = mWorld->getGravity();
		g.z = slider->getValue();
		mWorld->setGravity(g);
	}
}

void TutorialApplication::checkBoxToggled(OgreBites::CheckBox* checkBox)
{
	if ( checkBox->getName() == "Show debug shapes" ) {
		mWorld->setShowDebugShapes(checkBox->isChecked());
	}
}

void TutorialApplication::itemSelected(OgreBites::SelectMenu* menu)
{
	string selected = menu->getSelectedItem();
	
	if ( menu == mAnimationSelectMenu ) {
		// disable the pose controller
		if ( !mSkeletonController.isNull() ) {
			mSkeletonController->setEnabled(false);
			mSkeletonController->setShowDebugInfo( false );
		}
		if ( mAnimationState ) {
			mAnimationState->setEnabled(false);
		}
		std::cout<<"selected " <<menu->getSelectedItem()<<std::endl;
		if ( menu->getSelectedItem() != "<none>" ) {
			mAnimationState = mFigureEnt->getAnimationState(menu->getSelectedItem());
			mAnimationState->setEnabled(DO_ANIMATION);
			mAnimationState->setLoop(true);
		}

	} else if ( menu->getName() == "Debug draw" ) {
		std::cout<<"debug draw: "<<selected<<std::endl;
		if ( selected == "None" ) {
			mWorld->setShowDebugShapes(false);
		} else {
			mWorld->setShowDebugShapes(true);
		}
	}
			
}
	

#pragma mark - Creation

//-------------------------------------------------------------------------------------
void TutorialApplication::createViewports(void)
{
	// Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0.5,0.5,0.5));
	mCamera->setAspectRatio( Ogre::Real(vp->getActualWidth()/Ogre::Real(vp->getActualHeight())));
	
}

void TutorialApplication::createPhysics(void)
{
	mTimeMultiplier = DEFAULT_TIME_MULTIPLIER;
	mAnimationSpeed = DEFAULT_ANIMATION_SPEED;
	mTimeAccumulator = 0;
	// setup bullet
	Ogre::Vector3 gravityVector( 0, -9.81, 0);
	Ogre::AxisAlignedBox bounds( Ogre::Vector3(-100,-100,-100), Ogre::Vector3(100,100,100));
	mNumEntitiesInstanced = 0;
	// start bullet
	mWorld = new OgreBulletDynamics::DynamicsWorld( mSceneMgr, bounds, gravityVector );
	// add debug info display tool
	debugDrawer = new OgreBulletCollisions::DebugDrawer();
	debugDrawer->setDrawWireframe(false);
	debugDrawer->setDrawAabb(true);
	mWorld->setDebugDrawer(debugDrawer);
	mWorld->setShowDebugShapes(PHYSICS_DEBUG_SHAPES);
	
	// use more iterations for the consraints solver
	mWorld->getBulletDynamicsWorld()->getSolverInfo().m_numIterations = 30;
	
	
	OgreAssert( debugDrawer, "no debug drawer" );
	// create a node for the debug drawer, add to the scene
	Ogre::SceneNode *debugDrawerNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("bulletDebugDrawer", Ogre::Vector3::ZERO);
	debugDrawerNode->attachObject(static_cast<Ogre::SimpleRenderable*>(debugDrawer));

	
	// define a floor plane mesh
	Ogre::Plane p;
	// plane definition
	p.normal = Ogre::Vector3(0,1,0);
	p.d = 0;
	// make the flor plane
	Ogre::MeshManager::getSingleton().createPlane("FloorPlane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, p, 2000, 2000, 20, 20, true, 1, 1000, 1000, Ogre::Vector3::UNIT_Z );
	
	Ogre::Entity* floorEnt = mSceneMgr->createEntity( "floor", "FloorPlane" );
	floorEnt->setMaterialName("Examples/Rockwall");
	floorEnt->setCastShadows(false);
	mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(floorEnt);
	
	// create collision shape
	OgreBulletCollisions::CollisionShape *floorShape = new OgreBulletCollisions::StaticPlaneCollisionShape(Ogre::Vector3(0,1,0), 0); // normal vector, distance
	OgreBulletDynamics::RigidBody* defaultPlaneBody = new OgreBulletDynamics::RigidBody( "BasePlane", mWorld );
	float restitution = 0.0f;
	float friction = 100.0f;
	defaultPlaneBody->setStaticShape(floorShape, restitution, friction); // shape, restitution, friction
	// store
	mShapes.push_back(floorShape);
	mRigidBodies.push_back(defaultPlaneBody);
	mGroundPlaneBody = defaultPlaneBody;
	
}


std::vector<int> stringToIntArray( const Ogre::String& str ) {
	std::stringstream ss(str);
	std::string vertIdxString;
	std::vector<int>verticesToUse;
	while( std::getline(ss,vertIdxString) ) {
		if ( vertIdxString.length() ) {
			int idx = strtol(vertIdxString.c_str(),NULL,10);
			verticesToUse.push_back(idx);
		}
	}
	return verticesToUse;
}
std::vector< std::pair<int,int> > stringToIntPairArray( const Ogre::String& str ) {
	std::stringstream ss(str);
	std::string vertIdxString;
	std::vector< std::pair<int,int> > result;
	while( std::getline(ss,vertIdxString) ) {
		if ( vertIdxString.length() ) {
			// parse "int,int,"
			const char* lineChars = vertIdxString.c_str();
			char* next;
			int idx0 = strtol(lineChars,&next,10);
			// jump over the comma
			int idx1 = strtol(next+1,&next,10);
			result.push_back(std::make_pair(idx0,idx1));
		}
	}
	return result;
}

#pragma mark Create scene

inline btVector3 findNearestPointToLine(const btVector3 &pt1, const btVector3 &pt2, const btVector3 &testPoint, float &uDistance)
{
    const btVector3 A = testPoint - pt1;
    const btVector3 u = (pt2-pt1).normalized();
	uDistance = A.dot(u);
	
    return pt1 + uDistance * u;
};

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
	mElapsedTime = 0;
	createPhysics();
	createLights();

	// create actual soft mesh
	string name = "Swimmer";
	
	string path = "swimmer.mesh";
	Ogre::Entity* entity = mSceneMgr->createEntity( name, path);
	Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode( name + ".SceneNode" );
	node->attachObject(entity);
	node->setPosition(ROOT_POSITION);
	
	// save
	mFigureEnt = entity;
	mFigureNode = node;
	
	mFigureEnt->getSkeleton()->reset();
	
	// pose controller
	loadSkeletonController(SKELETON_CONTROLLER_FILENAME);
	
}

#pragma mark lights

void TutorialApplication::createLights(void) {
	mSceneMgr->setAmbientLight( Ogre::ColourValue(0.1,0.1,0.1) );
	mSceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );
	
	// relate some light positions to the camera position
	Ogre::Vector3 camPos = mCamera->getDerivedPosition();
	Ogre::Vector3 camRight = mCamera->getDerivedRight();
	Ogre::Vector3 camUp = mCamera->getDerivedUp();
	
	Ogre::Light* light = NULL;
	
	bool doShadows = false;
	
	// create yellow key light
	light = mSceneMgr->createLight("spotLight");
	light->setType(Ogre::Light::LT_SPOTLIGHT);
	Ogre::Vector3 p = camPos + camRight*-1.0f + camUp*1.5f;
	//light->setPosition(Ogre::Vector3(4,10,-20));
	light->setPosition(p);
	light->setDiffuseColour(Ogre::ColourValue(.5, .5, .35));
	light->setSpecularColour(Ogre::ColourValue(.5, .5, .35));
	light->setDirection( (ROOT_POSITION-light->getPosition()).normalisedCopy() );
	light->setSpotlightRange(Ogre::Degree(35), Ogre::Degree(50));
	light->setCastShadows(doShadows);
	
	// create red fill light
	light = mSceneMgr->createLight("pointLight");
	light->setType(Ogre::Light::LT_POINT);
	p = camPos + camRight*2.0f + camUp*-2.0f;
	//light->setPosition(Ogre::Vector3(-3, 0.5, -8 ));
	light->setPosition(p);
	light->setDiffuseColour( 1.0, 0.6, 0.6 );
	light->setSpecularColour( 1.0, 0.8, 0.8 );
	light->setCastShadows(doShadows);
	
	
	// create blue back light
	light = mSceneMgr->createLight("directionalLight");
	light->setType(Ogre::Light::LT_DIRECTIONAL);
	light->setDiffuseColour(.25,.25,1);
	light->setSpecularColour(.5,.5,1);
	light->setDirection(Ogre::Vector3(0.3, -.3, -1));
	// back light shouldn't have shadows
	light->setCastShadows(false);
}

Ogre::SceneNode* createBox(Ogre::SceneManager* sceneMgr, Ogre::String name)
{
	// create an ordinary Ogre mesh with texture
	Ogre::Entity* boxEnt = sceneMgr->createEntity(name , "cube.mesh");
	
	// we need the bounding box of the cube.mesh to set the size of the Bullet box
	Ogre::AxisAlignedBox boxBb = boxEnt->getBoundingBox();
	//Ogre::Vector3 size = boxBb.getSize();
	boxEnt->setMaterialName("Examples/BumpyMetal");
	
	// create the node
	Ogre::SceneNode* boxNode = sceneMgr->getRootSceneNode()->createChildSceneNode();
	boxNode->attachObject( boxEnt );
	return boxNode;
}


bool TutorialApplication::keyPressed( const OIS::KeyEvent &arg )
{
	if ( arg.key == OIS::KC_K ) {
		bool controlsShowing = mTrayMgr->getTrayContainer(OgreBites::TL_BOTTOMRIGHT)->isVisible();
		showControls(!controlsShowing);
		cout<<"camera at "<<mCamera->getDerivedPosition()<<endl;
	} else if ( arg.key == OIS::KC_U ) {
		mMovingX = true;
	} else if ( arg.key == OIS::KC_I ) {
		mMovingY = true;
	} else if ( arg.key == OIS::KC_O ) {
		mMovingZ = true;
	} else if ( arg.key == OIS::KC_SPACE ) {
	} else if ( arg.key == OIS::KC_UP ) {
		VirtualModelSkeletonController* vsc = dynamic_cast<VirtualModelSkeletonController*>(mSkeletonController.get());
		if ( vsc ) {
			//Ogre::Vector3 camDir = mCamera->getDerivedDirection();
			Ogre::Vector3 dir(0,1,0);
			float amount = 100.0f;
			vsc->pushModel(dir*amount);
		}
	} else if ( arg.key == OIS::KC_LEFT ) {
		VirtualModelSkeletonController* vsc = dynamic_cast<VirtualModelSkeletonController*>(mSkeletonController.get());
		if ( vsc ) {
			//Ogre::Vector3 camDir = mCamera->getDerivedDirection();
			Ogre::Vector3 dir(-1,0,0);
			float amount = 100.0f;
			vsc->pushModel(dir*amount);
		}
	} else {
		return BaseApplication::keyPressed( arg );
	}
	return true;
	
}

bool TutorialApplication::keyReleased( const OIS::KeyEvent& arg )
{
	if ( arg.key == OIS::KC_U ) {
		mMovingX = false;
	} else if ( arg.key == OIS::KC_I ) {
		mMovingY = false;
	} else if ( arg.key == OIS::KC_O ) {
		mMovingZ = false;
	} else {
		return BaseApplication::keyReleased( arg );
	}
	return true;
}


#pragma mark - Update loop

bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	
	mElapsedTime += evt.timeSinceLastFrame;
	
		
	//printf("stepping bullet %f\n", evt.timeSinceLastFrame );
	mTimeAccumulator += evt.timeSinceLastFrame*mTimeMultiplier;
	while ( mTimeAccumulator>0 ) {
		if ( mAnimationState && mAnimationState->getEnabled() ) {
			mAnimationState->addTime(PHYSICS_FRAME_DURATION*mAnimationSpeed);
		} else {
			// hackity?
			Ogre::AnimationStateSet* animStates = mFigureEnt->getAllAnimationStates();
			// i don't know why i have to call this :/
			animStates->_notifyDirty();
		}
		
		mWorld->stepSimulation( PHYSICS_FRAME_DURATION, 1, PHYSICS_FRAME_DURATION );
		
		while ( mTimeAccumulator > 0 )
			mTimeAccumulator -= PHYSICS_FRAME_DURATION;
		
		if ( mSkeletonController.get() ) {
			mSkeletonController->update( PHYSICS_FRAME_DURATION );
		}

	}
	

	bool ret = BaseApplication::frameRenderingQueued(evt);
	return ret;
	
}


bool TutorialApplication::frameStarted( const Ogre::FrameEvent& evt ) {
	bool ret = BaseApplication::frameStarted(evt);
	return ret;
}
