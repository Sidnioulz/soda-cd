#include "Action.h"
#include "UniquenessConstraintException.h"
using namespace std;


// -------------------------------------------------------- //
Action::Action(int frame, Ogre::Real xRot, Ogre::Real yRot, Ogre::Real zRot, Ogre::Real xTrans, Ogre::Real yTrans, Ogre::Real zTrans):
m_frame(frame),
m_xRot(xRot),
m_yRot(yRot),
m_zRot(zRot),
m_xTrans(xTrans),
m_yTrans(yTrans),
m_zTrans(zTrans)
{
}

// -------------------------------------------------------- //
Action::Action()
:
m_frame(1),
m_xRot(0),
m_yRot(0),
m_zRot(0),
m_xTrans(0),
m_yTrans(0),
m_zTrans(0)
{
}

// -------------------------------------------------------- //
Action::~Action()
{
}

// -------------------------------------------------------- //
int Action::getFrame()
{
	return m_frame;
}

// -------------------------------------------------------- //
void Action::setFrame(int frame)
{
	m_frame = frame;
}

// -------------------------------------------------------- //
Ogre::Real Action::getXRot()
{
	return m_xRot;
}

// -------------------------------------------------------- //
void Action::setXRot(Ogre::Real xRot)
{
	m_xRot = xRot;
}

// -------------------------------------------------------- //
Ogre::Real Action::getYRot()
{
	return m_yRot;
}

// -------------------------------------------------------- //
void Action::setYRot(Ogre::Real yRot)
{
	m_yRot = yRot;
}

// -------------------------------------------------------- //
Ogre::Real Action::getZRot()
{
	return m_zRot;
}

// -------------------------------------------------------- //
void Action::setZRot(Ogre::Real zRot)
{
	m_zRot = zRot;
}

// -------------------------------------------------------- //
Ogre::Real Action::getXTrans()
{
	return m_xTrans;
}

// -------------------------------------------------------- //
void Action::setXTrans(Ogre::Real xTrans)
{
	m_xTrans = xTrans;
}

// -------------------------------------------------------- //
Ogre::Real Action::getYTrans()
{
	return m_yTrans;
}

// -------------------------------------------------------- //
void Action::setYTrans(Ogre::Real yTrans)
{
	m_yTrans = yTrans;
}

// -------------------------------------------------------- //
Ogre::Real Action::getZTrans()
{
	return m_zTrans;
}

// -------------------------------------------------------- //
void Action::setZTrans(Ogre::Real zTrans)
{
	m_zTrans = zTrans;
}

// -------------------------------------------------------- //
void Action::clone(Action * act)
{
	m_frame = act->getFrame();
	m_xRot = act->getXRot();
	m_yRot = act->getYRot();
	m_zRot = act->getZRot();
	m_xTrans = act->getXTrans();
	m_yTrans = act->getYTrans();
	m_zTrans = act->getZTrans();
}


// -------------------------------------------------------- //
void Action::load(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "Action");

	if (reader->attributes().value("frame") > 0)
	{
		readFrame(reader);

		while (!reader->atEnd()) 
		{
			reader->readNext();
			if (reader->isEndElement())
				break;
			if (reader->isStartElement()) 
			{
				if (reader->name() == "xRot")
					readXRot(reader);
				else if (reader->name() == "yRot")
					readYRot(reader);
				else if (reader->name() == "zRot")
					readZRot(reader);
				else if (reader->name() == "xTrans")
					readXTrans(reader);
				else if (reader->name() == "yTrans")
					readYTrans(reader);
				else if (reader->name() == "zTrans")
					readZTrans(reader);
				else
					readUnknownElement(reader);
			}
		}
	}	else reader->raiseError(QObject::tr("Les donnees XML d'une frame doivent etre strictement positif"));

}

// -------------------------------------------------------- //
void Action::readFrame(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "Action");
	float e;
	std::string element = (reader->attributes()).value("frame").toString().toStdString();
	sscanf(element.string::c_str(), "%f", &e);
	setFrame(e);
}

// -------------------------------------------------------- //
void Action::readXRot(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "xRot");
	QString element = reader->readElementText();
	float e;
	sscanf(element.toStdString().string::c_str(), "%f", &e);
	setXRot(e);
}

// -------------------------------------------------------- //
void Action::readYRot(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "yRot");
	QString element = reader->readElementText();
	float e;
	sscanf(element.toStdString().string::c_str(), "%f", &e);
	setYRot(e);
}

// -------------------------------------------------------- //
void Action::readZRot(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "zRot");
	QString element = reader->readElementText();
	float e;
	sscanf(element.toStdString().string::c_str(), "%f", &e);
	setZRot(e);
}

// -------------------------------------------------------- //
void Action::readXTrans(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "xTrans");
	QString element = reader->readElementText();
	float e;
	sscanf(element.toStdString().string::c_str(), "%f", &e);
	setXTrans(e);
}

// -------------------------------------------------------- //
void Action::readYTrans(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "yTrans");
	QString element = reader->readElementText();
	float e;
	sscanf(element.toStdString().string::c_str(), "%f", &e);
	setYTrans(e);
}

// -------------------------------------------------------- //
void Action::readZTrans(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "zTrans");
	QString element = reader->readElementText();
	float e;
	sscanf(element.toStdString().string::c_str(), "%f", &e);
	setZTrans(e);
}

// -------------------------------------------------------- //
void Action::readUnknownElement(QXmlStreamReader* reader)
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
