#ifndef ACTION_ELEMENT_H
#define ACTION_ELEMENT_H

#include <QtCore>
#include "Action.h"

//  -------------------------------------------------  //
/** \brief Classe de gestion d'une animation
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class Animation : public QObject
{

public:
	/** Definit une Animation. */
	Animation();

	virtual ~Animation();

	/** Identifiant de l'action utilisee dans le stockage XML. */
	QString getId() const;

	/** Identifiantde l'action utilisee dans le stockage XML. */
	void setId(QString id);

	/** Nom de l'action affichee dans la bibliotheque. */
	QString getName() const;

	/** Definit le nom de l'action a afficher dans la bibliotheque. */
	void setName(QString name);

	/** Recuperation de la liste d'actions a effectuer */
	std::vector<Action*>  getActionList();

	/** Ajoute une action a la liste*/
	void  addActionList(Action* newAction);

	/** Numero de la frame courante*/
	int getCurrentFrame();

	/** Retourne l'action courante*/
	Action* getCurrentAction();

	/** Numero de la derniere frame de l'action courante  */
	int getFrameEnd();

	/** Remise a 0 des actions */
	void resetActions();

	/** Incremente les frames de l'animation, true si on change d'action, false sinon */
	bool incFrame();

	bool incAction(); //incremente les actions de l'animation, si l'on depasse le nombre total d'action, renvoie false et effectue un reset, true sinon

	bool incFrame(int n); //incremente l'action courante de n frame a condition que n soit inferieur ou egale a la frame de fin de l'action courante.

	/** Retourne le numero de l'action courante */
	int getNumCurrentAction();

	/** Charge les attributs de l'animation a partir de la source XML. */
	void load(QXmlStreamReader* reader);

	/** Copy dans this les parametres de anim*/
	void clone(Animation * anim);


protected:
	void readId(QXmlStreamReader* reader);
	void readName(QXmlStreamReader* reader);
	void readActionList(QXmlStreamReader* reader);
	void readUnknownElement(QXmlStreamReader* reader);

private:

	QString m_id;						//!< Identifiant de la serie d'action utilise dans le stockage XML
	QString m_name;						//!< Nom de la liste d'action affiche dans la bibliotheque.
	std::vector<Action*> m_listAction;	//!< Liste des actions composants l'animation
	int m_currentFrame;					//!< Numero de la frame courante
	size_t m_currentAction;				//!< Numero de l'action courante dans m_listAction (dans un soucis de simplification des calculs)
	int m_FrameEnd;						//!< Numero de la derniere frame de l'action courante. (dans un soucis de simplifications des calculs)
};

/** @} */
#endif 