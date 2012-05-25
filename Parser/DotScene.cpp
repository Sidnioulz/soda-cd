#include "DotScene.h"
#include "DotSceneElement.h"
#include "UniquenessConstraintException.h"

// -------------------------------------------------------- //
DotScene::DotScene():
QObject()
{
}

// -------------------------------------------------------- //
DotScene::~DotScene()
{
	foreach(DotSceneElement* element, m_DotScene)
	{
		delete element;
	}
}

// -------------------------------------------------------- //
DotSceneElement* DotScene::getElement(QString name) const
{
	DotSceneElement* result = NULL;
	foreach(DotSceneElement* element, m_DotScene)
	{
		if (element->getName() == name)
		{
			result = element;
			break;
		}
	}
	return result;
}

// -------------------------------------------------------- //
QList<DotSceneElement*> DotScene::getElements(QString type) const
{
	QList<DotSceneElement*> result;
	foreach(DotSceneElement* element, m_DotScene)
	{
		if (element->getType() == type)
			result.push_back(element);
	}
	return result;
}

// -------------------------------------------------------- //
void DotScene::addElement(DotSceneElement* element)
{
	foreach (DotSceneElement* e, m_DotScene)
	{
		if (e->getName() == element->getName())
		{
			throw new UniquenessConstraintException(tr("Un Node avec ce nom existe deja"), e);

		}
	}
	m_DotScene.push_back(element);
}

// -------------------------------------------------------- //
void DotScene::load(QIODevice* device)
{
	QXmlStreamReader reader(device);
	readDocument(&reader);
}

// -------------------------------------------------------- //
void DotScene::readDocument(QXmlStreamReader* reader)
{
	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isStartElement()) 
		{
			if (reader->name() == "scene" && reader->attributes().value("formatVersion") == "1.0.0")
				readNodes(reader);
			else
				reader->raiseError(QObject::tr("Les donnees XML de la scene ne correspondent pas au format version 1.0.0"));
		}
	}
}

// -------------------------------------------------------- //
void DotScene::readNodes(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "scene");

	while (!reader->atEnd()) 
	{
		reader->readNext();

		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "nodes")
			{
				readNode(reader);
			}
			else readUnknownElement(reader);//tt ce qui en dehors des <nodes> ne nous interesse pas !
		}
	}
}

// -------------------------------------------------------- //
void DotScene::readNode(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "nodes");
	DotSceneElement* element;

	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isStartElement()) 
		{
			if (reader->name() == "node")
			{
				element = new DotSceneElement();
				element->load(reader);
				if (element->getType() == QString("entity") || element->getType() == QString("camera")) 
				{
					addElement(element);
				} 
				else
				{
					delete element;
				}
			}
			else readUnknownElement(reader);//tt ce qui en dehors des <nodes> ne nous interesse pas !
		}
	}
}

// -------------------------------------------------------- //
void DotScene::readUnknownElement(QXmlStreamReader* reader)
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