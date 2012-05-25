#ifndef ENVIRONMENT_ELEMENT_H
#define ENVIRONMENT_ELEMENT_H

#include <QtCore>
#include <Ogre.h>

//  -------------------------------------------------  //
/** \brief Un environement parse via Environment.xml
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class EnvironmentElement : public QObject
{

public:
	/** Definit un mobilier. */
	EnvironmentElement();

	virtual ~EnvironmentElement();

	/** Identifiant de l'environnement utilise dans le stockage XML. */
	QString getId() const;

	/** Identifiantde l'environnement utilise dans le stockage XML. */
	void setId(QString id);

	/** Nom de l'environnement affiche dans la bibliotheque. */
	QString getName() const;

	/** Definit le nom de l'environnement a afficher dans la bibliotheque. */
	void setName(QString name);

	/** Image de l'environnementa afficher dans la bibliotheque. */
	QString getScreenshot() const;

	/** Image de l'environnement a afficher dans la bibliotheque. */
	void setScreenshot(QString screenshot);

	/** Representation 3D de l'environnement a afficher dans le plan 3D. */
	QString getResource3D() const;

	/** Representation 3D de l'environnement a afficher dans le plan 3D. */
	void setResource3D(QString resource3D);

	/** Triplet contenant le mesh de la pool, le nb d'objets et la position de depart*/
	QPair<QPair<QString, int>,Ogre::Vector3> getPool() const;

	/** Triplet contenant le mesh de la pool, le nb d'objets et la position de depart */
	void setPool(QString poolMesh, int nbObj, Ogre::Vector3 position);

	/** Charge les attributs de l'environnement a partir de la source XML. */
	void load(QXmlStreamReader* reader);

	/** Action par defaut */
	QString getAction() const;
	
	/** Action par defaut */
	void setAction(QString action);

	bool operator==(const EnvironmentElement& other) const;

protected:
	void readId(QXmlStreamReader* reader);
	void readName(QXmlStreamReader* reader);
	void readScreenshot(QXmlStreamReader* reader);
	void readResource3D(QXmlStreamReader* reader);
	void readObjects(QXmlStreamReader* reader);
	void readPool(QXmlStreamReader* reader);
	void readUnknownElement(QXmlStreamReader* reader);
	void readAction(QXmlStreamReader* reader);

private:

	QString										m_id;			//!< Identifiant du mobilier utilise dans le stockage XML
	QString										m_name;			//!< Nom du mobilier affiche dans la bibliotheque.
	QString										m_screenshot;	//!< Copie d'ecran du mobilier a afficher dans la bibliotheque.
	QString										m_resource3D;	//!< Modele 3D du mobilier.
	QPair<QPair<QString, int>,Ogre::Vector3>	m_Pool;			//!< Paire avec le mesh de la pool et le nb d'objets
	QString										m_actionDefault;//!< Action par defaut du mesh 
};

/** @} */
#endif // ROOMELEMENT_H