#ifndef DOTSCENE_H
#define DOTSCENE_H

#include <QtCore>

class DotSceneElement;

//  -------------------------------------------------  //
/** \brief Ensemble des fichiers .dotscene parses
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class DotScene : public QObject
{

public:
	DotScene();

	virtual ~DotScene();

	DotSceneElement* getElement(QString name) const;
	QList<DotSceneElement*> getElements(QString type) const;
	void addElement(DotSceneElement* element);
	void load(QIODevice* device);

protected:
	void readDocument(QXmlStreamReader* reader);
	void readNodes(QXmlStreamReader* reader);
	void readNode(QXmlStreamReader* reader);
	void readUnknownElement(QXmlStreamReader* reader);

private:
	QList<DotSceneElement*> m_DotScene; //!< Liste des elements disponibles dans la dotScene.

};

#endif
