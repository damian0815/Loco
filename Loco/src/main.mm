//
//  main.m
//  bgOgreTest
//
//  Created by michael on 28/02/13.
//  Copyright (c) 2013 michael. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import <AppKit/AppKit.h>
#import "LocoAppDelegate.h"

@interface LocoApplication: NSApplication

@end

int main(int argc, char *argv[])
{
	@autoreleasepool {
		NSApplication* app = [NSApplication sharedApplication];
		app.delegate = [[LocoAppDelegate alloc] init];
		[app run];
	}
}
