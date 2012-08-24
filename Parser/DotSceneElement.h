#ifndef DOTSCENEELEMENT_H
#define DOTSCENEELEMENT_H

#include "sodaRigidBody.h"
#include <QtCore>

//  -------------------------------------------------  //
/** \brief Un fichier .dotScene parse
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class DotSceneElement : public QObject
{
public:

	// -------BUILDER AND DESTROYER------- //
	DotSceneElement();
	virtual ~DotSceneElement();
	// ---------------------------------- //

	// ----------- ACCES METHODS --------- //
	QString getName() {return name;}
	void setName(QString newName) {name = newName;}

	QString getType() {return type;}
	void setType(QString newType) {type = newType;}

	Ogre::Real getXPos() {return m_Px;}
	void setXPos(Ogre::Real xpos){m_Px = xpos;}

	Ogre::Real getYPos() {return m_Py;}
	void setYPos(Ogre::Real ypos){m_Py = ypos;}

	Ogre::Real getZPos() {return m_Pz;}
	void setZPos(Ogre::Real zpos){m_Pz = zpos;}

	Ogre::Real getXQuat() {return m_Qx;}
	void setXQuat(Ogre::Real xquat) {m_Qx = xquat;}

	Ogre::Real getYQuat() {return m_Qy;}
	void setYQuat(Ogre::Real yquat) {m_Qy = yquat;}

	Ogre::Real getZQuat() {return m_Qz;}
	void setZQuat(Ogre::Real zquat) {m_Qz = zquat;}

	Ogre::Real getWQuat() {return m_Qw;}
	void setWQuat(Ogre::Real wquat) {m_Qw = wquat;}

	Ogre::Real getXScale() {return m_Sx;}
	void setXScale(Ogre::Real xscale) {m_Sx = xscale;}

	Ogre::Real getYScale() {return m_Sy;}
	void setYScale(Ogre::Real yscale) {m_Sy = yscale;}

	Ogre::Real getZScale() {return m_Sz;}
	void setZScale(Ogre::Real zscale) {m_Sz = zscale;}

	int getFrameStart() {return frameStart;}
	void setFrameStart(int newFrameStart) {frameStart = newFrameStart;}

	int getNumber() {return number;}
	void setNumber(int n) {number = n;}

	bool getStatus() {return m_isStatic;}
	void setStatus(bool st) {m_isStatic = st;}

	Ogre::Real getFov() {return Ent_fov;}
	Ogre::Real getNearPlaneDist() { return Ent_nearPlaneDist;}
	Ogre::Real getFarPlaneDist() { return Ent_farPlaneDist;}
	QString getEntName() { return Ent_Name;}
	QString getEntInfo() { return Ent_Info;}

	/** Charge les attributs de l'DotSceneElement a partir de la source XML. */
	void load(QXmlStreamReader* reader);

protected:

	void readUnknownElement(QXmlStreamReader* reader);
	void readPosition(QXmlStreamReader* reader);
	void readQuaternion(QXmlStreamReader* reader);
	void readScale(QXmlStreamReader* reader);
	void readEntity(QXmlStreamReader* reader);
	void readCamera(QXmlStreamReader* reader);
	void readFrameStart(QXmlStreamReader* reader);
	void readNumber(QXmlStreamReader* reader);
	void readStatus(QXmlStreamReader* reader);
	void readLight(QXmlStreamReader* reader);

private:

	QString			name, type;														//!<
	Ogre::Real		m_Px, m_Py, m_Pz, m_Qx, m_Qy, m_Qz, m_Qw, m_Sx, m_Sy, m_Sz;		//!<
	QString			Ent_Name, Ent_Info;												//!<
	Ogre::Real		Ent_fov, Ent_nearPlaneDist, Ent_farPlaneDist;					//!<
	int				frameStart, number;												//!<
	bool			m_isStatic;														//!<
};

#endif
