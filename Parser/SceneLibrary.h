#ifndef SCENELIBRARY_H
#define SCENELIBRARY_H

#include <QtCore>

class SceneElement;

//  -------------------------------------------------  //
/** \brief La liste des objets parses via "Library.xml"
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class SceneLibrary : public QObject
{
	
public:
	SceneLibrary();

	virtual ~SceneLibrary();

	/** Mobilier ayant l'identifiant indique. */
	SceneElement* getSceneElement(QString id) const;

	/** Liste des types de mobiliers de cette bibliotheque. */
	QList<QString> getSceneElementTypes() const;

	/** Liste des mobiliers correspondant au type demande. */
	QList<SceneElement*> getSceneElements(QString type) const;

	/** Liste de tous les mobiliers quel que soit leur type. */
	QList<SceneElement*> getSceneElements() const;

	/** Liste de tous les mobiliers quel que soit leur type. */
	QStringList getSceneElementsName() const;

	/** 
	 * Ajoute un mobilier en fonction de son type a la bibliotheque. 
	 * @throw UniquenessConstraintException Un mobilier avec le meme identifiant
	 *                                      existe deja.
	 */
        void addSceneElement(QString type, SceneElement* element);

	/** 
	 * Charge les mobiliers de la bibliotheque stockee dans le fichier XML indique. 
	 * Leve une exception QXmlParseException en cas de probleme 
	 * dans le fichier XML.
	 *
	 * @param device Fichier XML ou est stockee la bibliotheque et son mobilier
	 * @throw QXmlParseException exception levee en cas de probleme lors de la
	 *                           lecture du fichier XML
	 */
	void load(QIODevice* device);

	/** 
	 * Charge les mobiliers de la bibliotheque stockee dans la chaine XML indiquee. 
	 * Leve une exception QXmlParseException en cas de probleme 
	 * dans le fichier XML.
	 *
	 * @param data Chaine XML ou est stockee la bibliotheque et son mobilier
	 * @throw QXmlParseException exception levee en cas de probleme lors de la
	 *                           lecture du fichier XML
	 */
	void load(QString data);

	bool operator==(const SceneLibrary& other) const;

protected:
	void readDocument(QXmlStreamReader* reader);
	void readSceneLibrary(QXmlStreamReader* reader);
	void readCategory(QXmlStreamReader* reader);
	void readCategoryName(QXmlStreamReader* reader, QString* categoryName);
	void readSceneElement(QXmlStreamReader* reader, QString categoryName);
	void readUnknownElement(QXmlStreamReader* reader);

private:

	QMultiHash<QString, SceneElement*> m_SceneElements; //!< Liste des elements disponibles dans la bibliotheque indexes par leur type.

};

/** @} */

#endif // ROOMLIBRARY_H
