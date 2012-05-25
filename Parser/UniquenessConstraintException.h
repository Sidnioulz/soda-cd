#ifndef UNIQUENESSCONSTRAINTEXCEPTION_H
#define UNIQUENESSCONSTRAINTEXCEPTION_H

#include <QtCore>

//  -------------------------------------------------  //
/** \brief Exception sur la creation
 *	\author Q. Avril, IRISA-INSA de Rennes */
//  -------------------------------------------------  //

class UniquenessConstraintException : public QObject
{
	
public:
	/** 
	 * Cree une exception indiquant qu'on essaie de creer un nouvel element
	 * ayant le meme identifiant qu'un autre existant.
	 * @param reason Message indiquant la raison de l'exception
	 * @param element Lien vers l'element deja existant
	 */
	UniquenessConstraintException(QString reason, QObject* element);

	virtual ~UniquenessConstraintException();

	/** Message indiquant la raison de l'exception. */
	QString getReason();

	/** Lien vers l'element deja existant. */
	QObject* getDuplicatedElement();

private:

	QString _reason;				//!< Message indiquant la raison de l'exception.
	QObject* _duplicatedElement;	//!< Lien vers l'element deja existant. 
};

/** @} */
#endif