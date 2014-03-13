//
//  TextFile.h
//  bulletTest7
//
//  Created on 02/04/13.
//
//

#ifndef __TEXTFILE_H__
#define __TEXTFILE_H__

#include <OGRE/OgreResourceManager.h>
#include <OGRE/OgreResource.h>



class TextFile : public Ogre::Resource
{
	Ogre::String mString;
	
protected:
	
	// must implement these from the Ogre::Resource interface
	void loadImpl();
	void unloadImpl();
	size_t calculateSize() const;
	
public:
	
	TextFile(Ogre::ResourceManager *creator, const Ogre::String &name,
			 Ogre::ResourceHandle handle, const Ogre::String &group, bool isManual = false,
			 Ogre::ManualResourceLoader *loader = 0);
	
	virtual ~TextFile();
	
	void setString(const Ogre::String &str);
	const Ogre::String &getString() const;
	
	/// split string on \n
	std::vector<Ogre::String> getLines() const;
};

class TextFilePtr : public Ogre::ResourcePtr
{
public:
	TextFilePtr() : Ogre::ResourcePtr() {}
	explicit TextFilePtr(TextFile *rep) : Ogre::ResourcePtr(rep) {}
	TextFilePtr(const TextFilePtr &r) : Ogre::ResourcePtr(r) {}
	TextFilePtr(const Ogre::ResourcePtr &r) : Ogre::ResourcePtr()
	{
		if( r.isNull() )
			return;
		// lock & copy other mutex pointer
		OGRE_LOCK_MUTEX(*r.OGRE_AUTO_MUTEX_NAME)
		OGRE_COPY_AUTO_SHARED_MUTEX(r.OGRE_AUTO_MUTEX_NAME)
		pRep = static_cast<TextFile*>(r.getPointer());
		pUseCount = r.useCountPointer();
		useFreeMethod = r.freeMethod();
		if (pUseCount)
		{
			++(*pUseCount);
		}
	}
	
	/// Operator used to convert a ResourcePtr to a TextFilePtr
	TextFilePtr& operator=(const Ogre::ResourcePtr& r)
	{
		if(pRep == static_cast<TextFile*>(r.getPointer()))
			return *this;
		release();
		if( r.isNull() )
			return *this; // resource ptr is null, so the call to release above has done all we need to do.
		// lock & copy other mutex pointer
		OGRE_LOCK_MUTEX(*r.OGRE_AUTO_MUTEX_NAME)
		OGRE_COPY_AUTO_SHARED_MUTEX(r.OGRE_AUTO_MUTEX_NAME)
		pRep = static_cast<TextFile*>(r.getPointer());
		pUseCount = r.useCountPointer();
		useFreeMethod = r.freeMethod();
		if (pUseCount)
		{
			++(*pUseCount);
		}
		return *this;
	}
	
	TextFile* operator->()
	{
		return static_cast<TextFile*>(getPointer());
	}
};
	
	
class TextFileManager : public Ogre::ResourceManager, public Ogre::Singleton<TextFileManager>
{
protected:
	
	// must implement this from ResourceManager's interface
	Ogre::Resource *createImpl(const Ogre::String &name, Ogre::ResourceHandle handle,
							   const Ogre::String &group, bool isManual, Ogre::ManualResourceLoader *loader,
							   const Ogre::NameValuePairList *createParams);
	
public:
	
	TextFileManager();
	virtual ~TextFileManager();
	
	virtual Ogre::ResourcePtr load(const Ogre::String &name, const Ogre::String &group, bool isManual = false,
							 Ogre::ManualResourceLoader* loader = 0, const Ogre::NameValuePairList* loadParams = 0,
							 bool backgroundThread = false);
	
	static TextFileManager &getSingleton();
	static TextFileManager *getSingletonPtr();
};


	
#endif
