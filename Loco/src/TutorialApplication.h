/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
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
#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include "BaseApplication.h"
#include "OgreBulletDynamics.h"
#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletCollisions.h"
#include "SkeletonController.h"

class TutorialApplication : public BaseApplication
{
public:
	TutorialApplication(void);
	virtual ~TutorialApplication(void);
	
protected:
	virtual void createScene(void);
	virtual void createCamera(void);
	virtual void createViewports(void);
	virtual void createFrameListener(void);
	
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool frameStarted( const Ogre::FrameEvent& evt );
	
    virtual bool keyPressed( const OIS::KeyEvent &arg );
	virtual bool keyReleased( const OIS::KeyEvent& arg );
	
	virtual void buttonHit(OgreBites::Button *button);
	virtual void sliderMoved(OgreBites::Slider* slider);
	virtual void checkBoxToggled(OgreBites::CheckBox* checkBox);
	virtual void itemSelected(OgreBites::SelectMenu* menu);
private:
	
	void createPhysics(void);
	void createLights(void);

	void showControls( bool show );
	
	OgreBulletDynamics::DynamicsWorld *mWorld;   // OgreBullet World
	OgreBulletCollisions::DebugDrawer *debugDrawer;
	int mNumEntitiesInstanced;
	std::deque<OgreBulletDynamics::RigidBody *>         mRigidBodies;
	std::deque<OgreBulletCollisions::CollisionShape *>  mShapes;

	Ogre::Entity* mFigureEnt;
	Ogre::SceneNode* mFigureNode;
	
	Ogre::AnimationState* mAnimationState;
	
	float mTimeMultiplier;
	float mAnimationSpeed;
	
	double mElapsedTime;
	double mTimeAccumulator;
	
	bool mMovingX, mMovingY, mMovingZ;
	
	OgreBites::ParamsPanel* mStatusPanel;
	OgreBites::SelectMenu* mAnimationSelectMenu;
	
	OgreBulletDynamics::RigidBody* mGroundPlaneBody;
	void loadSkeletonController(std::string filename);
	Ogre::SharedPtr<SkeletonController> mSkeletonController;
	
};

#endif // #ifndef __TutorialApplication_h_
