#include "DotSceneElement.h"
#include "UniquenessConstraintException.h"
using namespace std;

// -------------------------------------------------------- //
DotSceneElement::DotSceneElement():
m_Px(0.000000), 
m_Py(0.000000), 
m_Pz(0.000000), 
m_Qx(0.000000), 
m_Qy(0.000000), 
m_Qz(0.000000), 
m_Qw(1.000000), 
m_Sx(1.000000), 
m_Sy(1.000000), 
m_Sz(1.000000),
Ent_Name(QString("")),
Ent_Info(QString("")),
Ent_fov(1.000000),
Ent_nearPlaneDist(0.100000),
Ent_farPlaneDist(1000.000000),
frameStart(0),
number(1),
m_isStatic(true)
{
}

// -------------------------------------------------------- //
DotSceneElement::~DotSceneElement()
{
}

// -------------------------------------------------------- //
void DotSceneElement::load(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "node");
	if (reader->attributes().value("name") != "")
	{
		setName(reader->attributes().value("name").toString());

		while (!reader->atEnd()) 
		{
			reader->readNext();
			if (reader->isEndElement()) break;

			if (reader->isStartElement()) 
			{

				if (reader->name() == "position")
				{
					readPosition(reader);
					reader->readNext();//on finit de lire ce bloc unique
				}
				else if (reader->name() == "quaternion")
				{
					readQuaternion(reader);
					reader->readNext();//on finit de lire ce bloc unique
				}
				else if (reader->name() == "scale")
				{
					readScale(reader);
					reader->readNext();//on finit de lire ce bloc unique
				}
				else if (reader->name() == "status")
				{
					readStatus(reader);
					reader->readNext();//on finit de lire ce bloc unique
				}
				else if (reader->name() == "entity")
				{
					setType(QString("entity"));
					readEntity(reader);
					reader->readNext();//on finit de lire ce bloc unique
				}
				else if (reader->name() == "frameStart")
				{
					readFrameStart(reader);
					reader->readNext();//on finit de lire ce bloc unique
				}
				else if (reader->name() == "number")
				{
					readNumber(reader);
					reader->readNext();//on finit de lire ce bloc unique
				}
				else if (reader->name() == "camera")
				{
					setType(QString("camera"));
					readCamera(reader);
					break;
				}
				else if (reader->name() == "light")
				{
					setType(QString("light"));
					readLight(reader);
					break;
				}
				else
				{
					readUnknownElement(reader);
					//reader->raiseError(QObject::tr("Ce type de node n'est pas prise en compte !"));
					break;
				}
			}
		}
	}	//else reader->raiseError(QObject::tr("Le node n'a aucun nom !"));
}

// -------------------------------------------------------- //
void DotSceneElement::readPosition(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "position");
	float e;
	std::string element = (reader->attributes()).value("x").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setXPos(e);
	element = (reader->attributes()).value("y").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setYPos(e);
	element = (reader->attributes()).value("z").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setZPos(e);
}

// -------------------------------------------------------- //
void DotSceneElement::readQuaternion(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "quaternion");
	float e;
	std::string element = (reader->attributes()).value("x").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setXQuat(e);
	element = (reader->attributes()).value("y").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setYQuat(e);
	element = (reader->attributes()).value("z").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setZQuat(e);
	element = (reader->attributes()).value("w").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setWQuat(e);
}

// -------------------------------------------------------- //
void DotSceneElement::readScale(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "scale");
	float e;
	std::string element = (reader->attributes()).value("x").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setXScale(e);
	element = (reader->attributes()).value("y").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setYScale(e);
	element = (reader->attributes()).value("z").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setZScale(e);
}

// -------------------------------------------------------- //
void DotSceneElement::readEntity(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "entity");
	Ent_Name =  reader->attributes().value("name").toString();
	Ent_Info = reader->attributes().value("meshFile").toString();
}

// -------------------------------------------------------- //
void DotSceneElement::readFrameStart(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "frameStart");
	int e;
	std::string element = (reader->attributes()).value("frame").toString().toStdString();
	sscanf(element.string::c_str(), "%d", &e);
	setFrameStart(e);
}

// -------------------------------------------------------- //
void DotSceneElement::readNumber(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "number");
	int e;
	std::string element = (reader->attributes()).value("value").toString().toStdString();
	sscanf(element.string::c_str(), "%d", &e);
	setNumber(e);
	element = (reader->attributes()).value("frame").toString().toStdString();
	sscanf(element.string::c_str(), "%d", &e);
	setFrameStart(e);
}

// -------------------------------------------------------- //
void DotSceneElement::readCamera(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "camera");
	Ent_Name =  reader->attributes().value("name").toString();
	float e;
	std::string element = reader->attributes().value("fov").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	Ent_fov = e;

	Ent_Info = reader->attributes().value("projectionType").toString();

	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "clipping")
			{
				Q_ASSERT(reader->isStartElement() && reader->name() == "clipping");

				element = reader->attributes().value("nearPlaneDist").toString().toStdString();
				sscanf(element.string::c_str(), "%f", &e);
				Ent_nearPlaneDist = e;
				element = reader->attributes().value("farPlaneDist").toString().toStdString();
				sscanf(element.string::c_str(), "%f", &e);
				Ent_farPlaneDist = e;
				reader->readNext();//on finit de lire ce bloc unique
			}
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void DotSceneElement::readLight(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "Light");
	Ent_Name =  reader->attributes().value("name").toString();
	float e;
	std::string element = (reader->attributes()).value("x").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setXPos(Ogre::Real(e));
	element = (reader->attributes()).value("y").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setYPos(Ogre::Real(e));
	element = (reader->attributes()).value("z").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setZPos(Ogre::Real(e));
}

// -------------------------------------------------------- //
void DotSceneElement::readStatus(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "status");
	QString status =  reader->attributes().value("static").toString();
	if(status == "true")
		m_isStatic = true;
	else
		m_isStatic = false;
}

// -------------------------------------------------------- //
void DotSceneElement::readUnknownElement(QXmlStreamReader* reader)
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