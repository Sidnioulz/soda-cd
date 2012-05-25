#ifndef SCENELIBRARYLISTMODEL_H
#define SCENELIBRARYLISTMODEL_H

#include <QtGui>

class SceneLibrary;
class SceneElement;

//  -------------------------------------------------  //
/** \brief Le modele de representation des objets parses
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class SceneLibraryListModel : public QAbstractListModel
{
	
public:
	SceneLibraryListModel(QObject *parent, SceneLibrary* library);

	/** 
	 * Caracteristiques du mobilier de la ligne courante 
	 * (en fonction du type de mobilier affiche). 
	 */
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

	/** Renvoie l'index du nom. */
	int find(const QString name) const;

	/** Renvoie le nb de ligne du modele. */
    int rowCount(const QModelIndex &parent) const;

	/** Type de mobilier a afficher. */
	void setCurrentType(QString type);

private:

	SceneLibrary* m_SceneLibrary;					//!< Bibliotheque comportant tous les mobiliers disponibles.
	QString m_CurrentType;							//!< Type de mobilier affiche par la liste.
	QList<SceneElement*> m_CurrentSceneElements;	//!< Stockage des mobiliers correspondant au type courant.
	mutable QList<QIcon> m_CurrentIcons;			//!< Stockage des icones correspondant aux images des mobiliers affiches.

};

#endif 
