#include "EnvironmentLibraryListModel.h"
#include "EnvironmentLibrary.h"
#include "EnvironmentElement.h"

#include <QtGui>

// -------------------------------------------------------- //
EnvironmentLibraryListModel::EnvironmentLibraryListModel(QObject *parent, EnvironmentLibrary* library)
    : QAbstractListModel(parent), m_EnvironmentLibrary(library)
{
	setCurrentType("");
}

// -------------------------------------------------------- //
void EnvironmentLibraryListModel::setCurrentType(QString type)
{
	m_CurrentType = type;
	if (m_CurrentType.isEmpty())
		m_CurrentEnvironmentElements = m_EnvironmentLibrary->getEnvironmentElements();
	else
		m_CurrentEnvironmentElements = m_EnvironmentLibrary->getEnvironmentElements(m_CurrentType);

	m_CurrentIcons.clear();
	reset();
}

// -------------------------------------------------------- //
QVariant EnvironmentLibraryListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

	EnvironmentElement* EnvironmentElement = m_CurrentEnvironmentElements.at(index.row());

    if (role == Qt::DecorationRole)
	{
		// charge l'icone si ca n'a pas deja ete fait
		if (m_CurrentIcons.size() <= index.row())
		{
			QIcon icon(EnvironmentElement->getScreenshot());
			m_CurrentIcons.append(icon);
			return icon;
		}
		else
		{
			return m_CurrentIcons[index.row()];
		}
	}
    else if (role == Qt::DisplayRole)
		return EnvironmentElement->getName();

    return QVariant();
}

// -------------------------------------------------------- //
int EnvironmentLibraryListModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    else
        return m_CurrentEnvironmentElements.size();
}