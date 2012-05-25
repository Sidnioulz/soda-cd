#include "EnvironmentElement.h"
using namespace std;

// -------------------------------------------------------- //
EnvironmentElement::EnvironmentElement()
{
	//initialisation de l'action par defaut.
	m_actionDefault = "";
	// Pool par default
	Ogre::Vector3 pos (0,0,0);
	QString mesh = "";
	int nbObj = 0;
	QPair<QString, int> paire (mesh, nbObj);
	m_Pool = QPair<QPair<QString,int>,Ogre::Vector3> (paire,pos);
}

// -------------------------------------------------------- //
EnvironmentElement::~EnvironmentElement()
{
}

// -------------------------------------------------------- //
QString EnvironmentElement::getId() const
{
	return m_id;
}

// -------------------------------------------------------- //
void EnvironmentElement::setId(QString id)
{
	m_id = id;
}

// -------------------------------------------------------- //
QString EnvironmentElement::getName() const
{
	return m_name;
}

// -------------------------------------------------------- //
void EnvironmentElement::setName(QString name)
{
	m_name = name;
}

// -------------------------------------------------------- //
QString EnvironmentElement::getScreenshot() const
{
	return m_screenshot;
}

// -------------------------------------------------------- //
void EnvironmentElement::setScreenshot(QString screenshot)
{
	m_screenshot = screenshot;
}

// -------------------------------------------------------- //
QString EnvironmentElement::getResource3D() const
{
	return m_resource3D;
}

// -------------------------------------------------------- //
void EnvironmentElement::setResource3D(QString resource3D)
{
	m_resource3D = resource3D;
}

// -------------------------------------------------------- //
QString EnvironmentElement::getAction() const
{
	return m_actionDefault;
}

// -------------------------------------------------------- //
void EnvironmentElement::setAction(QString action)
{
	m_actionDefault = action;
}

// -------------------------------------------------------- //
void EnvironmentElement::setPool(QString poolMesh, int nbObj, Ogre::Vector3 position)
{
	QPair<QString, int> paire (poolMesh, nbObj);
	m_Pool = QPair<QPair<QString, int>,Ogre::Vector3>(paire, position);
}

// -------------------------------------------------------- //
QPair<QPair<QString, int>,Ogre::Vector3> EnvironmentElement::getPool() const
{
	return m_Pool;
}

// -------------------------------------------------------- //
bool EnvironmentElement::operator==(const EnvironmentElement& other) const
{
	return m_name == other.m_name && 
		m_screenshot == other.m_screenshot &&
		m_resource3D == other.m_resource3D;
}

// -------------------------------------------------------- //
void EnvironmentElement::load(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "EnvironmentElement");

	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "id")
				readId(reader);
			else if (reader->name() == "name")
				readName(reader);
			else if (reader->name() == "screenshot")
				readScreenshot(reader);
			else if (reader->name() == "resource3D")
				readResource3D(reader);
						/////////////////////////MODIFICATION BENOIT//////////////////////
			else if (reader->name() == "action")
				readAction(reader);
			/////////////////////////MODIFICATION BENOIT//////////////////////
			else if (reader->name() == "pool")
				readPool(reader);
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void EnvironmentElement::readId(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "id");
	QString id = reader->readElementText();
	setId(id);
}

// -------------------------------------------------------- //
void EnvironmentElement::readName(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "name");
	QString language = reader->attributes().value("language").toString();
	QString name = reader->readElementText();

	QLocale currentLocale;
	QString currentLanguage = QLocale::languageToString(currentLocale.language());
	if (language.compare(currentLanguage, Qt::CaseInsensitive) == 0)
	{
		setName(name);
	}
}

// -------------------------------------------------------- //
void EnvironmentElement::readScreenshot(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "screenshot");
	QString screenshot = reader->readElementText();
	setScreenshot(screenshot);
}

// -------------------------------------------------------- //
void EnvironmentElement::readResource3D(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "resource3D");
	QString resource3D = reader->readElementText();
	setResource3D(resource3D);
}

// -------------------------------------------------------- //
void EnvironmentElement::readAction(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "action");
	QString action = reader->readElementText();
	setAction(action);
}

// -------------------------------------------------------- //
void EnvironmentElement::readPool(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "pool");
	QString test = reader->attributes().value("number").toString();
	int nbObj = test.toInt();
	QString x_Str = reader->attributes().value("x").toString();
	int x = x_Str.toInt();
	QString y_Str = reader->attributes().value("y").toString();
	int y = y_Str.toInt();
	QString z_Str = reader->attributes().value("z").toString();
	int z = z_Str.toInt();
	Ogre::Vector3 position (x,y,z);
	QString pool = reader->readElementText();
	setPool(pool, nbObj, position);
}

// -------------------------------------------------------- //
void EnvironmentElement::readUnknownElement(QXmlStreamReader* reader)
{
   Q_ASSERT(reader->isStartElement());
	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isEndElement())
			break;
		if (reader->isStartElement())
			readUnknownElement(reader);
	}
}