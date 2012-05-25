#ifndef EnvironmentLIBRARYLISTMODEL_H
#define EnvironmentLIBRARYLISTMODEL_H

#include <QtGui>

class EnvironmentLibrary;
class EnvironmentElement;

//  -------------------------------------------------  //
/** \brief La representation liste des environements parses
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class EnvironmentLibraryListModel : public QAbstractListModel
{
	
public:
	EnvironmentLibraryListModel(QObject *parent, EnvironmentLibrary* library);

	/** 
	 * Caracteristiques du mobilier de la ligne courante 
	 * (en fonction du type de mobilier affiche). 
	 */
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

	/** Renvoie le nb de ligne du modele. */
    int rowCount(const QModelIndex &parent) const;

	/** Type de mobilier a afficher. */
	void setCurrentType(QString type);

private:

	EnvironmentLibrary			*m_EnvironmentLibrary;			//!< Bibliotheque comportant tous les mobiliers disponibles.
	QString						m_CurrentType;					//!< Type de mobilier affiche par la liste.
	QList<EnvironmentElement*>	m_CurrentEnvironmentElements;	//!< Stockage des mobiliers correspondant au type courant.
	mutable QList<QIcon>		m_CurrentIcons;					//!< Stockage des icones correspondant aux images des mobiliers affiches.
};

#endif