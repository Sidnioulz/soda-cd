#include "EnvironmentLibrary.h"
#include "EnvironmentElement.h"
#include "UniquenessConstraintException.h"

// -------------------------------------------------------- //
EnvironmentLibrary::EnvironmentLibrary()
	: QObject()
{
}

// -------------------------------------------------------- //
EnvironmentLibrary::~EnvironmentLibrary()
{
	foreach(EnvironmentElement* element, m_EnvironmentElements)
	{
		delete element;
	}
}

// -------------------------------------------------------- //
EnvironmentElement* EnvironmentLibrary::getEnvironmentElement(QString name) const
{
	EnvironmentElement* result = NULL;
	foreach(EnvironmentElement* element, m_EnvironmentElements)
	{
		if (element->getName() == name)
		{
			result = element;
			break;
		}
	}
	return result;
}

/////////////////////////MODIFICATION BENOIT//////////////////////
// -------------------------------------------------------- //
EnvironmentElement* EnvironmentLibrary::getEnvironmentElementByMesh(QString mesh) const
{
	EnvironmentElement* result = NULL;
	foreach(EnvironmentElement* element, m_EnvironmentElements)
	{
		if (element->getResource3D() == mesh)
		{
			result = element;
			break;
		}
	}
	return result;
}
/////////////////////////MODIFICATION BENOIT//////////////////////

// -------------------------------------------------------- //
QList<QString> EnvironmentLibrary::getEnvironmentElementTypes() const
{
	return m_EnvironmentElements.uniqueKeys();
}

// -------------------------------------------------------- //
QList<EnvironmentElement*> EnvironmentLibrary::getEnvironmentElements(QString type) const
{
	return m_EnvironmentElements.values(type);
}

// -------------------------------------------------------- //
QList<EnvironmentElement*> EnvironmentLibrary::getEnvironmentElements() const
{
	return m_EnvironmentElements.values();
}

// -------------------------------------------------------- //
void EnvironmentLibrary::addEnvironmentElement(QString type, EnvironmentElement* element)
{
	foreach (EnvironmentElement* e, m_EnvironmentElements)
	{
		if (e->getId() == element->getId())
		{
			throw new UniquenessConstraintException(tr("Un mobilier avec cet identifiant existe deja"), e);
		}
	}

	m_EnvironmentElements.insertMulti(type, element);
}

// -------------------------------------------------------- //
void EnvironmentLibrary::load(QIODevice* device)
{
	QXmlStreamReader reader(device);
	readDocument(&reader);
}

// -------------------------------------------------------- //
void EnvironmentLibrary::load(QString data)
{
	QXmlStreamReader reader(data);
	readDocument(&reader);
}

// -------------------------------------------------------- //
bool EnvironmentLibrary::operator==(const EnvironmentLibrary& other) const
{
	if (m_EnvironmentElements.size() != other.m_EnvironmentElements.size())
		return false;

	if (m_EnvironmentElements == other.m_EnvironmentElements)
		return true;

	QMultiHash<QString, EnvironmentElement*>::const_iterator it = m_EnvironmentElements.begin();
	while (it != m_EnvironmentElements.end()) 
	{
		const QString &akey = it.key();
		QMultiHash<QString, EnvironmentElement*>::const_iterator it2 = other.m_EnvironmentElements.find(akey);
		do {
			if (it2 == other.m_EnvironmentElements.end() || !(it2.key() == akey))
				return false;
			EnvironmentElement* EnvironmentElement1 = it.value();
			EnvironmentElement* EnvironmentElement2 = it2.value();
			if (!(*EnvironmentElement1 == *EnvironmentElement2))
				return false;
			++it;
			++it2;
		} while (it != m_EnvironmentElements.end() && it.key() == akey);
	}

	return true;
}

// -------------------------------------------------------- //
void EnvironmentLibrary::readDocument(QXmlStreamReader* reader)
{
	while (!reader->atEnd()) 
	{
       reader->readNext();
	   if (reader->isStartElement()) 
	   {
			if (reader->name() == "Environment" && reader->attributes().value("version") == "1.0")
				readEnvironments(reader);
			else
				reader->raiseError(QObject::tr("Les donnees XML de la bibliotheque ne correspondent pas au format version 1.0."));
	   }
     }
}

// -------------------------------------------------------- //
void EnvironmentLibrary::readEnvironments(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "Environment");
	while (!reader->atEnd()) 
	{
		reader->readNext();

		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "category")
			{
				readCategory(reader);
			}
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void EnvironmentLibrary::readCategory(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "category");
	QString categoryName;
	while (!reader->atEnd()) 
	{
		reader->readNext();

		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "name")
				readCategoryName(reader, &categoryName);
			else if (reader->name() == "EnvironmentElement")
			{
				EnvironmentElement* element = new EnvironmentElement();
				element->load(reader);
				if (!reader->hasError()) 
				{
					// ajoute cet element a la bibliotheque maintenant que tous ses attributs ont ete lus.
					addEnvironmentElement(categoryName, element);
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
void EnvironmentLibrary::readCategoryName(QXmlStreamReader* reader, QString* categoryName)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "name");
	QString language = reader->attributes().value("language").toString();
	QString name = reader->readElementText();

	QLocale currentLocale;
	QString currentLanguage = QLocale::languageToString(currentLocale.language());
	if (language.compare(currentLanguage, Qt::CaseInsensitive) == 0)
	{
		*categoryName = name;
	}
}

// -------------------------------------------------------- //
void EnvironmentLibrary::readUnknownElement(QXmlStreamReader* reader)
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