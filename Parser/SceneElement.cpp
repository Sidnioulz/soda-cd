#include "SceneElement.h"

// -------------------------------------------------------- //
SceneElement::SceneElement()
{
}

// -------------------------------------------------------- //
SceneElement::~SceneElement()
{
}

// -------------------------------------------------------- //
QString SceneElement::getId() const
{
	return _id;
}

// -------------------------------------------------------- //
void SceneElement::setId(QString id)
{
	_id = id;
}

// -------------------------------------------------------- //
QString SceneElement::getName() const
{
	return _name;
}

// -------------------------------------------------------- //
void SceneElement::setName(QString name)
{
	_name = name;
}

// -------------------------------------------------------- //
QString SceneElement::getScreenshot() const
{
	return _screenshot;
}

// -------------------------------------------------------- //
void SceneElement::setScreenshot(QString screenshot)
{
	_screenshot = screenshot;
}

// -------------------------------------------------------- //
QString SceneElement::getResource3D() const
{
	return _resource3D;
}

// -------------------------------------------------------- //
void SceneElement::setResource3D(QString resource3D)
{
	_resource3D = resource3D;
}

// -------------------------------------------------------- //
void SceneElement::setMass(int mass)
{
	_mass = mass;
}

// -------------------------------------------------------- //
int SceneElement::getMass()
{
	return _mass;
}

// -------------------------------------------------------- //
bool SceneElement::operator==(const SceneElement& other) const
{
	return _name == other._name && 
		_screenshot == other._screenshot &&
		_resource3D == other._resource3D;
}

// -------------------------------------------------------- //
void SceneElement::load(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "SceneElement");

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
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void SceneElement::readId(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "id");
	QString id = reader->readElementText();
	setId(id);
}

// -------------------------------------------------------- //
void SceneElement::readName(QXmlStreamReader* reader)
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
void SceneElement::readScreenshot(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "screenshot");
	QString screenshot = reader->readElementText();
	setScreenshot(screenshot);
}

// -------------------------------------------------------- //
void SceneElement::readResource3D(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "resource3D");
	QString massStr = reader->attributes().value("mass").toString();
	QString resource3D = reader->readElementText();
	setResource3D(resource3D);
	int mass (massStr.toInt());
	setMass(mass);
}

// -------------------------------------------------------- //
void SceneElement::readUnknownElement(QXmlStreamReader* reader)
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