#include "Action.h"
#include "Animation.h"


// -------------------------------------------------------- //
Animation::Animation():
m_currentFrame(1),
m_currentAction(0),
m_FrameEnd(1)
{
}

// -------------------------------------------------------- //
Animation::~Animation()
{
	for(size_t i = 0 ; i < m_listAction.size() ; ++i)
	{
		delete m_listAction[i];
	}
	m_listAction.clear();
}

// -------------------------------------------------------- //
QString Animation::getId() const
{
	return m_id;
}

// -------------------------------------------------------- //
void Animation::setId(QString id)
{
	m_id = id;
}

// -------------------------------------------------------- //
QString Animation::getName() const
{
	return m_name;
}

// -------------------------------------------------------- //
void Animation::setName(QString name)
{
	m_name = name;
}

// -------------------------------------------------------- //
std::vector<Action*>  Animation::getActionList()
{
	return m_listAction;
}

// -------------------------------------------------------- //
void  Animation::addActionList(Action* newAction)
{
	m_listAction.push_back(newAction);
}

// -------------------------------------------------------- //
int Animation::getCurrentFrame()
{
	return m_currentFrame;
}

// -------------------------------------------------------- //
Action* Animation::getCurrentAction()
{
	return m_listAction[m_currentAction];
}

// -------------------------------------------------------- //
int Animation::getNumCurrentAction()
{
	return (int)m_currentAction;
}

// -------------------------------------------------------- //
int Animation::getFrameEnd()
{
	return m_FrameEnd;
}

// -------------------------------------------------------- //
void Animation::resetActions()
{
	m_currentFrame = 1;
	m_currentAction = 0;
	m_FrameEnd = m_listAction[0]->getFrame();
}

// -------------------------------------------------------- //
bool Animation::incFrame()
{
	m_currentFrame++;

	if(m_currentFrame > m_FrameEnd)//on a depasse la frame de fin de l'action courante.
	{
		m_currentAction++;
		if(m_currentAction >= m_listAction.size())//on a depasse le nombre d'actions composant la frame
		{
			resetActions();
		}
		else
		{
			m_FrameEnd += m_listAction[m_currentAction]->getFrame();
		}
		return true;
	}
	return false;
}

// -------------------------------------------------------- //
bool Animation::incAction()
{

	m_currentAction++;
	m_currentFrame= m_FrameEnd+1;

	if(m_currentAction >= m_listAction.size())//on a depasse le nombre d'actions composant la frame
	{
		resetActions();
		return false;
	}
	else
	{
		m_FrameEnd += m_listAction[m_currentAction]->getFrame();
	}	

	return true;

}

// -------------------------------------------------------- //
bool Animation::incFrame(int n)
{


	if(m_currentFrame+n > m_FrameEnd)//on a depasse la frame de fin de l'action courante.
	{
		return false;
	}
	else
	{
		m_currentFrame+=n;
		return true;
	}
}

// -------------------------------------------------------- //
void Animation::clone(Animation * anim)
{
	m_id = anim->getId();
	m_name = anim->getName();

	std::vector<Action *> list = anim->getActionList();
	int i;

	for(i = 0; i < (int)list.size(); i++)
	{
		m_listAction.push_back(new Action(list[i]->getFrame(), list[i]->getXRot(), list[i]->getYRot(), list[i]->getZRot(), list[i]->getXTrans(), list[i]->getYTrans(), list[i]->getZTrans()));
	}

	m_currentFrame = anim->getCurrentFrame();					
	m_currentAction = anim->getNumCurrentAction();				
	m_FrameEnd = anim->getFrameEnd();	
}

// -------------------------------------------------------- //
void Animation::load(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "Animation");

	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "id")
				readId(reader);
			else if (reader->name() == "name")
				readName(reader);
			else if (reader->name() == "ActionList")
				readActionList(reader);
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void Animation::readId(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "id");
	QString id = reader->readElementText();
	setId(id);
}

// -------------------------------------------------------- //
void Animation::readName(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "name");
	QString language = reader->attributes().value("language").toString();
	QString name = reader->readElementText();

	QLocale currentLocale;
	QString currentLanguage = QLocale::languageToString(currentLocale.language());
	if (language.compare(currentLanguage, Qt::CaseInsensitive) == 0)
	{
		setName(name);
	}
}

// -------------------------------------------------------- //
void Animation::readActionList(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement() && reader->name() == "ActionList");
	bool initEndFrame = false;

	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isEndElement())
			break;
		if (reader->isStartElement()) 
		{
			if (reader->name() == "Action")
			{
				Action* element = new Action();
				element->load(reader);
				if (!reader->hasError()) 
				{
					// ajoute cet element a la bibliotheque maintenant que tous ses attributs ont ete lus.
					addActionList(element);
					if(!initEndFrame)
					{
						initEndFrame = true;
						m_FrameEnd = element->getFrame();
					}
				} 
				else
				{
					// un pb a eu lieu pendant la lecture du fichier XML l'element n'a pas ete ajoute a la bibliotheque => on le detruit
					delete element;
				}
			}
			else
				readUnknownElement(reader);
		}
	}
}

// -------------------------------------------------------- //
void Animation::readUnknownElement(QXmlStreamReader* reader)
{
	Q_ASSERT(reader->isStartElement());
	while (!reader->atEnd()) 
	{
		reader->readNext();
		if (reader->isEndElement())
			break;
		if (reader->isStartElement())
			readUnknownElement(reader);
	}
}