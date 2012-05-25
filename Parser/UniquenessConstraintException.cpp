#include "UniquenessConstraintException.h"

// -------------------------------------------------------- //
UniquenessConstraintException::UniquenessConstraintException(QString reason, QObject* element)
: _reason(reason), _duplicatedElement(element)
{
}

// -------------------------------------------------------- //
UniquenessConstraintException::~UniquenessConstraintException()
{
}

// -------------------------------------------------------- //
QString UniquenessConstraintException::getReason()
{
	return _reason;
}

// -------------------------------------------------------- //
QObject* UniquenessConstraintException::getDuplicatedElement()
{
	return _duplicatedElement;
}
