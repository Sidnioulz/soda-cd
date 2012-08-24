#ifndef ACTION_H
#define ACTION_H

#include "sodaRigidBody.h"
#include <QtCore>

//  -------------------------------------------------  //
/** \brief Wrapper for adding dynamic physics objects
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class Action
{
public:

	// -------BUILDER AND DESTROYER------- //
	Action();
	Action(int frame, Ogre::Real xRot, Ogre::Real yRot, Ogre::Real zRot, Ogre::Real xTrans, Ogre::Real yTrans, Ogre::Real zTrans);
	virtual ~Action();
	// ---------------------------------- //

	// ----------- ACCES METHODS --------- //
	//! get the frame
	int getFrame();
	//! set the frame
	void setFrame(int frame);

	//! get the rotation about x-axis
	Ogre::Real getXRot();
	//! set the rotation about x-axis
	void setXRot(Ogre::Real xRot);

	//! get the rotation about y-axis
	Ogre::Real getYRot();
	//! set the rotation about y-axis
	void setYRot(Ogre::Real yRot);

	//! get the rotation about z-axis
	Ogre::Real getZRot();
	//! set the rotation about z-axis
	void setZRot(Ogre::Real zRot);

	//! get the x-translation
	Ogre::Real getXTrans();
	//! set the x-translation
	void setXTrans(Ogre::Real xTrans);

	//! get the y-translation
	Ogre::Real getYTrans();
	//! set the y-translation
	void setYTrans(Ogre::Real yTrans);

	//! get the z-translation
	Ogre::Real getZTrans();
	//! set the z-translation
	void setZTrans(Ogre::Real zTrans);

	//! copy act's values in this
	void clone(Action * act);

	/** Charge les attributs de l'action a partir de la source XML. */
	void load(QXmlStreamReader* reader);

protected:

void readUnknownElement(QXmlStreamReader* reader);
void readFrame(QXmlStreamReader* reader);
void readXRot(QXmlStreamReader* reader);
void readYRot(QXmlStreamReader* reader);
void readZRot(QXmlStreamReader* reader);
void readXTrans(QXmlStreamReader* reader);
void readYTrans(QXmlStreamReader* reader);
void readZTrans(QXmlStreamReader* reader);

private:

	int m_frame;  //!<
	Ogre::Real m_xRot, m_yRot, m_zRot, m_xTrans, m_yTrans, m_zTrans;  //!<
};

#endif
