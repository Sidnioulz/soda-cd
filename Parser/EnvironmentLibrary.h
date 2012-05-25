#ifndef ENVIRONMENT_LIBRARY_H
#define ENVIRONMENT_LIBRARY_H

#include <Ogre.h>
#include <QtCore>

class EnvironmentElement;

//  -------------------------------------------------  //
/** \brief La liste des environements parses via Environment.xml
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class EnvironmentLibrary : public QObject
{
	
public:
	EnvironmentLibrary();

	virtual ~EnvironmentLibrary();

	/** Mobilier ayant l'identifiant indique. */
	EnvironmentElement* getEnvironmentElement(QString id) const;

	/** Mobilier ayant le mesh indique. */
	EnvironmentElement* getEnvironmentElementByMesh(QString mesh) const;

	/** Liste des types de mobiliers de cette bibliotheque. */
	QList<QString> getEnvironmentElementTypes() const;

	/** Liste des mobiliers correspondant au type demande. */
	QList<EnvironmentElement*> getEnvironmentElements(QString type) const;

	/** Liste de tous les mobiliers quel que soit leur type. */
	QList<EnvironmentElement*> getEnvironmentElements() const;

	/** 
	 * Ajoute un mobilier en fonction de son type a la bibliotheque. 
	 * @throw UniquenessConstraintException Un mobilier avec le meme identifiant
	 *                                      existe deja.
	 */
	void addEnvironmentElement(QString type, EnvironmentElement* element);

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

	bool operator==(const EnvironmentLibrary& other) const;

protected:
	void readDocument(QXmlStreamReader* reader);
	void readEnvironments(QXmlStreamReader* reader);
	void readElement(QXmlStreamReader* reader, QString categoryName);
	void readCategory(QXmlStreamReader* reader);
	void readCategoryName(QXmlStreamReader* reader, QString* categoryName);
	void readUnknownElement(QXmlStreamReader* reader);

private:

	QMultiHash<QString, EnvironmentElement*> m_EnvironmentElements; //!< Liste des elements disponibles dans la bibliotheque indexes par leur type.

};

#endif
