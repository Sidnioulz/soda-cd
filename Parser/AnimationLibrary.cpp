#include "AnimationLibrary.h"
#include "Animation.h"
#include "UniquenessConstraintException.h"

// -------------------------------------------------------- //
AnimationLibrary::AnimationLibrary():
QObject()
{
}

// -------------------------------------------------------- //
AnimationLibrary::~AnimationLibrary()
{
	foreach(Animation* element, m_Animations)
	{
		delete element;
	}
}

// -------------------------------------------------------- //
Animation* AnimationLibrary::getAnimation(QString name) const
{
	Animation* result = NULL;
	foreach(Animation* element, m_Animations)
	{
		if (element->getId() == name)
		{
			result = element;
			break;
		}
	}
	return result;
}

// -------------------------------------------------------- //
Animation* AnimationLibrary::getAnimationByName(QString name) const
{
	Animation* result = NULL;
	foreach(Animation* element, m_Animations)
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
QList<Animation*> AnimationLibrary::getAnimations() const
{
	return m_Animations;
}

// -------------------------------------------------------- //
QList<QString> AnimationLibrary::getAnimationsName() const
{
	QList<QString>* result = new QList<QString>();

	foreach(Animation* element, m_Animations)
	{
		result->push_back(element->getName());
	}
	return *result;
}

// -------------------------------------------------------- //
void AnimationLibrary::addAnimation(Animation* element)
{
	foreach (Animation* e, m_Animations)
	{
		if (e->getId() == element->getId())
		{
			throw new UniquenessConstraintException(tr("Une animation avec cet identifiant existe deja"), e);
		}
	}
	m_Animations.push_back(element);
}

// -------------------------------------------------------- //
void AnimationLibrary::load(QIODevice* device)
{
	//On ajoute l'animation par defaut
	Animation* element = new Animation();
	element->addActionList(new Action());
	std::string id = "animDefault";
	element->setId(QString(id.c_str()));
	std::string name = "Default Anim";
	element->setName(QString(name.c_str()));
	addAnimation(element);

	QXmlStreamReader reader(device);
	readDocument(&reader);
}

// -------------------------------------------------------- //
void AnimationLibrary::load(QString data)
{
	QXmlStreamReader reader(data);
	readDocument(&reader);
}

// -------------------------------------------------------- //
void AnimationLibrary::readDocument(QXmlStreamReader* reader)
{
	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isStartElement()) 
		{
			if (reader->name() == "AnimationList" && reader->attributes().value("version") == "1.0")
				readAnimations(reader);
			else
				reader->raiseError(QObject::tr("Les donnees XML de la bibliotheque ne correspondent pas au format version 1.0."));
		}
	}
}

// -------------------------------------------------------- //
void AnimationLibrary::readAnimations(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "AnimationList");

	while (!reader->atEnd()) 
	{
		reader->readNext();

		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "Animation")
			{
				Animation* element = new Animation();
				element->load(reader);
				if (!reader->hasError()) 
				{
					// ajoute cet element a la bibliotheque maintenant que tous ses attributs ont ete lus.
					addAnimation(element);
				} 
				else
				{
					// un pb a eu lieu pendant la lecture du fichier XML l'element n'a pas ete ajoute a la bibliotheque => on le detruit
					delete element;
				}
			}
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void AnimationLibrary::readUnknownElement(QXmlStreamReader* reader)
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