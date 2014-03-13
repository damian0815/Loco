//
//  LocoAppDelegate.m
//  Loco
//
//  Created on 26/07/13.
//
//

#import "LocoAppDelegate.h"
#import <OGRE/Ogre.h>
#import "TutorialApplication.h"
#import <Quartz/Quartz.h>

@interface LocoAppDelegate()
{
	TutorialApplication app;
	CVDisplayLinkRef displayLink;
}

@property (strong,readwrite,atomic) NSTimer* timer;

@end

@implementation LocoAppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)notification
{
	try {
		app.setup();
	} catch(Ogre::Exception &e) {
		std::cerr<<"exception in "<<e.getFile()<<":"<<e.getLine()<<": "<<e.getFullDescription()<<std::endl;
	}

	
	// create the timer
	NSTimeInterval framePeriod = 1.0f/60.0f;
	
	self.timer = [NSTimer scheduledTimerWithTimeInterval:framePeriod target:self selector:@selector(frameTick:) userInfo:nil repeats:YES];
}

- (void)frameTick:(NSTimer*)timer
{
	bool result = app.renderOneFrame();
	if ( !result ) {
		[[NSApplication sharedApplication] terminate:self];
	}
}

- (void)applicationWillTerminate:(NSNotification *)notification
{
	[self.timer invalidate];
	self.timer = nil;
	app.cleanup();
}

@end
