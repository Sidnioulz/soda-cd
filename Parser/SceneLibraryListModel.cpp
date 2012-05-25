#include "SceneLibraryListModel.h"
#include "SceneLibrary.h"
#include "SceneElement.h"

#include <QtGui>

// -------------------------------------------------------- //
SceneLibraryListModel::SceneLibraryListModel(QObject *parent, SceneLibrary* library)
    : QAbstractListModel(parent), m_SceneLibrary(library)
{
	setCurrentType("");
}

// -------------------------------------------------------- //
void SceneLibraryListModel::setCurrentType(QString type)
{
	m_CurrentType = type;
	if (m_CurrentType.isEmpty())
		m_CurrentSceneElements = m_SceneLibrary->getSceneElements();
	else
		m_CurrentSceneElements = m_SceneLibrary->getSceneElements(m_CurrentType);

	m_CurrentIcons.clear();
	reset();
}

// -------------------------------------------------------- //
QVariant SceneLibraryListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

	SceneElement* SceneElement = m_CurrentSceneElements.at(index.row());

    if (role == Qt::DecorationRole)
	{
		// charge l'icone si ca n'a pas deja ete fait
		if (m_CurrentIcons.size() <= index.row())
		{
			QIcon icon(SceneElement->getScreenshot());
			m_CurrentIcons.append(icon);
			return icon;
		}
		else
		{
			return m_CurrentIcons[index.row()];
		}
	}
    else if (role == Qt::DisplayRole)
		return SceneElement->getName();

    return QVariant();
}

// -------------------------------------------------------- //
int SceneLibraryListModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    else
        return m_CurrentSceneElements.size();
}

// -------------------------------------------------------- //
int SceneLibraryListModel::find(const QString name) const
{
	for(int i = 0 ; i < m_CurrentSceneElements.size() ; ++i)
	{
		if(m_CurrentSceneElements[i]->getResource3D()==name)
		{
			return i;
		}
	}
	return 0;
}