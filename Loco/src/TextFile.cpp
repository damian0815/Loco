//
//
//  TextFile.cpp
//  bulletTest7
//
//  Created on 02/04/13.
//
//

#include "TextFile.h"

#include <OGRE/OgreSerializer.h>

class TextFile; // forward declaration

class TextFileSerializer : public Ogre::Serializer
{
public:
	TextFileSerializer();
	virtual ~TextFileSerializer();
	
	void exportTextFile(const TextFile *pText, const Ogre::String &fileName);
	void importTextFile(Ogre::DataStreamPtr &stream, TextFile *pDest);
};

TextFileSerializer::TextFileSerializer()
{
}

TextFileSerializer::~TextFileSerializer()
{
}

void TextFileSerializer::exportTextFile(const TextFile *pText, const Ogre::String &fileName)
{
	std::ofstream outFile;
	outFile.open(fileName.c_str(), std::ios::out);
	outFile << pText->getString();
	outFile.close();
}

void TextFileSerializer::importTextFile(Ogre::DataStreamPtr &stream, TextFile *pDest)
{
	pDest->setString(stream->getAsString());
}



TextFile::TextFile(Ogre::ResourceManager* creator, const Ogre::String &name,
				   Ogre::ResourceHandle handle, const Ogre::String &group, bool isManual,
				   Ogre::ManualResourceLoader *loader) :
Ogre::Resource(creator, name, handle, group, isManual, loader)
{
	/* If you were storing a pointer to an object, then you would set that pointer to NULL here.
     */
	
	/* For consistency with StringInterface, but we don't add any parameters here
     That's because the Resource implementation of StringInterface is to
     list all the options that need to be set before loading, of which
     we have none as such. Full details can be set through scripts.
     */
	createParamDictionary("TextFile");
}

TextFile::~TextFile()
{
	unload();
}

// farm out to TextFileSerializer
void TextFile::loadImpl()
{
	TextFileSerializer serializer;
	Ogre::StringVector rscs = *Ogre::ResourceGroupManager::getSingleton().findResourceLocation(mGroup, mName);
	for ( auto s: rscs ) {
		std::cout<<"  "<<s<<std::endl;
	}
	if ( Ogre::ResourceGroupManager::getSingleton().resourceExists(mGroup, mName) ) {
		Ogre::DataStreamPtr stream = Ogre::ResourceGroupManager::getSingleton().openResource(mName, mGroup, true, this);
		serializer.importTextFile(stream, this);
	} else {
		std::cerr<<__PRETTY_FUNCTION__<<": couldn't find file for "<<mName<<" in group "<<mGroup<<std::endl;
	}
}

void TextFile::unloadImpl()
{
	/* If you were storing a pointer to an object, then you would check the pointer here,
     and if it is not NULL, you would destruct the object and set its pointer to NULL again.
     */
	
	mString.clear();
}

size_t TextFile::calculateSize() const
{
	return mString.length();
}

void TextFile::setString(const Ogre::String &str)
{
	mString = str;
}

const Ogre::String &TextFile::getString() const
{
	return mString;
}

std::vector<Ogre::String> TextFile::getLines() const
{
	// split string on newlines
	std::vector<Ogre::String> lines;
	size_t pos = 0;
	while ( pos<mString.length() ) {
		size_t nextPos = mString.find('\n',pos);
		if ( nextPos == std::string::npos ) {
			// done
			break;
		}
		Ogre::String line = mString.substr(pos,nextPos-pos);
		lines.push_back(line);
		pos = nextPos+1;
	}
	if ( pos<mString.length() ) {
		lines.push_back(mString.substr(pos));
	}
	return lines;
}




template<> TextFileManager *Ogre::Singleton<TextFileManager>::msSingleton = 0;

TextFileManager *TextFileManager::getSingletonPtr()
{
	return msSingleton;
}

TextFileManager &TextFileManager::getSingleton()
{
	assert(msSingleton);
	return(*msSingleton);
}

TextFileManager::TextFileManager()
{
	mResourceType = "TextFile";
	
	// low, because it will likely reference other resources
	mLoadOrder = 30.0f;
	
	// this is how we register the ResourceManager with OGRE
	Ogre::ResourceGroupManager::getSingleton()._registerResourceManager(mResourceType, this);
}

TextFileManager::~TextFileManager()
{
	// and this is how we unregister it
	Ogre::ResourceGroupManager::getSingleton()._unregisterResourceManager(mResourceType);
}

Ogre::ResourcePtr TextFileManager::load(const Ogre::String &name, const Ogre::String &group, bool isManual,
								  Ogre::ManualResourceLoader* loader, const Ogre::NameValuePairList* loadParams,
								  bool backgroundThread )
{
	TextFilePtr textf = getByName(name);
	
	if (textf.isNull())
		textf = create(name, group);
	
	textf->load();
	return textf;
}

Ogre::Resource *TextFileManager::createImpl(const Ogre::String &name, Ogre::ResourceHandle handle,
											const Ogre::String &group, bool isManual, Ogre::ManualResourceLoader *loader,
											const Ogre::NameValuePairList *createParams)
{
	return new TextFile(this, name, handle, group, isManual, loader);
}
