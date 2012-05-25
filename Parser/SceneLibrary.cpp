#include "SceneLibrary.h"
#include "SceneElement.h"
#include "UniquenessConstraintException.h"

// -------------------------------------------------------- //
SceneLibrary::SceneLibrary()
	: QObject()
{
}

// -------------------------------------------------------- //
SceneLibrary::~SceneLibrary()
{
	foreach(SceneElement* element, m_SceneElements)
	{
		delete element;
	}
}

// -------------------------------------------------------- //
SceneElement* SceneLibrary::getSceneElement(QString id) const
{
	SceneElement* result = NULL;
	foreach(SceneElement* element, m_SceneElements)
	{
		if (element->getId() == id)
		{
			result = element;
			break;
		}
	}
	return result;
}

// -------------------------------------------------------- //
QList<QString> SceneLibrary::getSceneElementTypes() const
{
	return m_SceneElements.uniqueKeys();
}

// -------------------------------------------------------- //
QList<SceneElement*> SceneLibrary::getSceneElements(QString type) const
{
	return m_SceneElements.values(type);
}

// -------------------------------------------------------- //
QList<SceneElement*> SceneLibrary::getSceneElements() const
{
	return m_SceneElements.values();
}

// -------------------------------------------------------- //
QStringList SceneLibrary::getSceneElementsName() const
{
	QList<SceneElement*> list = getSceneElements();
	QStringList listReturn;
	for(int i = 0 ; i < list.size() ; i++)
	{
		listReturn.push_back(list[i]->getName());
	}
	return listReturn;
}

// -------------------------------------------------------- //
void SceneLibrary::addSceneElement(QString type, SceneElement* element)
{
	foreach (SceneElement* e, m_SceneElements)
	{
		if (e->getId() == element->getId())
		{
			throw new UniquenessConstraintException(tr("Un mobilier avec cet identifiant existe deja"), e);
		}
	}

	m_SceneElements.insertMulti(type, element);
}

// -------------------------------------------------------- //
void SceneLibrary::load(QIODevice* device)
{
	QXmlStreamReader reader(device);
	readDocument(&reader);
}

// -------------------------------------------------------- //
void SceneLibrary::load(QString data)
{
	QXmlStreamReader reader(data);
	readDocument(&reader);
}

// -------------------------------------------------------- //
bool SceneLibrary::operator==(const SceneLibrary& other) const
{
	// repris de Qt/4.3.0/src/corelib/tools/qhash.h ligne 835

	// Les tests d'egalite entre collections de pointeurs 
	// portent sur la valeur de chaque pointeur et 
	// non sur la valeur des objets pointes, meme en placant
	// des elements QPointer dans la collection.
	
	if (m_SceneElements.size() != other.m_SceneElements.size())
		return false;

	if (m_SceneElements == other.m_SceneElements)
		return true;

	QMultiHash<QString, SceneElement*>::const_iterator it = m_SceneElements.begin();
	while (it != m_SceneElements.end()) 
	{
		const QString &akey = it.key();
		QMultiHash<QString, SceneElement*>::const_iterator it2 = other.m_SceneElements.find(akey);
		do {
			if (it2 == other.m_SceneElements.end() || !(it2.key() == akey))
				return false;
			SceneElement* SceneElement1 = it.value();
			SceneElement* SceneElement2 = it2.value();
			if (!(*SceneElement1 == *SceneElement2))
				return false;
			++it;
			++it2;
		} while (it != m_SceneElements.end() && it.key() == akey);
	}

	return true;
}

// -------------------------------------------------------- //
void SceneLibrary::readDocument(QXmlStreamReader* reader)
{
	while (!reader->atEnd()) 
	{
       reader->readNext();
	   if (reader->isStartElement()) 
	   {
			if (reader->name() == "SceneLibrary" && reader->attributes().value("version") == "1.0")
				readSceneLibrary(reader);
			else
				reader->raiseError(QObject::tr("Les donnees XML de la bibliotheque ne correspondent pas au format version 1.0."));
	   }
     }
	 if (reader->hasError()) {
		 //Erreur de lecture -->To Do message erreur
		 //throw new QXmlParseException(reader->errorString(), reader->columnNumber(), reader->lineNumber());
	 } 
}

// -------------------------------------------------------- //
void SceneLibrary::readSceneLibrary(QXmlStreamReader* reader)
{
    Q_ASSERT(reader->isStartElement() && reader->name() == "SceneLibrary");
	while (!reader->atEnd()) 
	{
		reader->readNext();

		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "category")
				readCategory(reader);
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void SceneLibrary::readCategory(QXmlStreamReader* reader)
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
			else if (reader->name() == "SceneElement")
			{
				SceneElement* element = new SceneElement();
				element->load(reader);
				if (!reader->hasError()) 
				{
					// ajoute cet element a la bibliotheque maintenant que tous
					// ses attributs ont ete lus.
					addSceneElement(categoryName, element);
				} 
				else
				{
					// un pb a eu lieu pendant la lecture du fichier XML
					// l'element n'a pas ete ajoute a la bibliotheque 
					// => on le detruit
					delete element;
				}
			}
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void SceneLibrary::readCategoryName(QXmlStreamReader* reader, QString* categoryName)
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
void SceneLibrary::readUnknownElement(QXmlStreamReader* reader)
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
