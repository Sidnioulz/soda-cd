#ifndef SCENEELEMENT_H
#define SCENEELEMENT_H

#include <QtCore>

//  -------------------------------------------------  //
/** \brief Un objet parse via "Library.xml"
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class SceneElement : public QObject
{
	
public:
	/** Definit un mobilier. */
	SceneElement();

	virtual ~SceneElement();

	/** Identifiant du mobilier utilise dans le stockage XML. */
	QString getId() const;

	/** Identifiant du mobilier utilise dans le stockage XML. */
	void setId(QString id);

	/** Nom du mobilier affiche dans la bibliotheque. */
	QString getName() const;

	/** Definit le nom du mobilier a afficher dans la bibliotheque. */
	void setName(QString name);

	/** Image du mobilier a afficher dans la bibliotheque. */
	QString getScreenshot() const;

	/** Image du mobilier a afficher dans la bibliotheque. */
	void setScreenshot(QString screenshot);

	/** Representation 3D du mobilier a afficher dans le plan 3D. */
	QString getResource3D() const;

	/** Representation 3D du mobilier a afficher dans le plan 3D. */
	void setResource3D(QString resource3D);

	/** Masse de l'objet */
	void setMass(int mass);

	/** Masse de l'objet */
	int getMass();

	/** Charge les attributs de ce mobilier a partir de la source XML. */
	void load(QXmlStreamReader* reader);

	bool operator==(const SceneElement& other) const;

protected:
	void readId(QXmlStreamReader* reader);
	void readName(QXmlStreamReader* reader);
	void readScreenshot(QXmlStreamReader* reader);
	void readResource3D(QXmlStreamReader* reader);
	void readUnknownElement(QXmlStreamReader* reader);

private:

	QString _id;			//!< Identifiant du mobilier utilise dans le stockage XML
	QString _name;			//!< Nom du mobilier affiche dans la bibliotheque.
	QString _screenshot;	//!< Copie d'ecran du mobilier a afficher dans la bibliotheque.
	QString _resource3D;	//!< Modele 3D du mobilier.
	int		_mass;			//!< Masse de l'objet
};

/** @} */
#endif // ROOMELEMENT_H