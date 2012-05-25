#include <QObject>
#include <QtTest/QtTest>

#include "QtTestUtil/QtTestUtil.h"

#include "../src/cell.h"

class TestCell : public QObject
{
     Q_OBJECT
	
private slots:
    void initTestCase()
    {
    }

    void cleanupTestCase()
    {
    }

    void testMyMethod()
    {
        // Test default constructor
        Cell c;
        QCOMPARE(c.getOwnerId(), (short)0);
        QCOMPARE( c.getEntities(), (QVector<obEntityWrapper *> const *)0);
    }
};

QTTESTUTIL_REGISTER_TEST(TestCell);
#include "TestCell.moc"
