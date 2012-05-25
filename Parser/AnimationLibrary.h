#ifndef ANIMATION_LIBRARY_H
#define ANIMATION_LIBRARY_H

#include <QtCore>

class Animation;

//  -------------------------------------------------  //
/** \brief Classe de librairie d'animations
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class AnimationLibrary : public QObject
{

public:
	AnimationLibrary();

	virtual ~AnimationLibrary();

	/** animation ayant l'identifiant indique. */
	Animation* getAnimation(QString id) const;

	/** animation ayant le nom indique. */
	Animation* getAnimationByName(QString name) const;

	/** Liste de toutes les animations. */
	QList<Animation*> getAnimations() const;

	/** Liste des noms des animations. */
	QList<QString> getAnimationsName() const;

	/** 
	* Ajoute une animation a la bibliotheque. 
	* @throw UniquenessConstraintException Une animation avec le meme identifiant
	*                                      existe deja.
	*/
	void addAnimation(Animation* element);

	/** 
	* Charge les animations de la bibliotheque stockee dans le fichier XML indique. 
	* Leve une exception QXmlParseException en cas de probleme 
	* dans le fichier XML.
	*
	* @param device Fichier XML ou est stockee la bibliotheque 
	* @throw QXmlParseException exception levee en cas de probleme lors de la
	*                           lecture du fichier XML
	*/
	void load(QIODevice* device);

	/** 
	* Charge les animations de la bibliotheque stockee dans la chaine XML indiquee. 
	* Leve une exception QXmlParseException en cas de probleme 
	* dans le fichier XML.
	*
	* @param data Chaine XML ou est stockee la bibliotheque
	* @throw QXmlParseException exception levee en cas de probleme lors de la
	*                           lecture du fichier XML
	*/
	void load(QString data);

protected:
	void readDocument(QXmlStreamReader* reader);
	void readAnimations(QXmlStreamReader* reader);
	void readUnknownElement(QXmlStreamReader* reader);

private:
	QList<Animation*> m_Animations; //!< Liste des elements disponibles dans la bibliotheque.

};

#endif
