#include <QObject>
#include <QtTest/QtTest>

#include "QtTestUtil/QtTestUtil.h"

class TestGrid : public QObject {
     Q_OBJECT
	
	private slots:
		void initTestCase() {
		}

		void cleanupTestCase() {
		}

		void testMyMethod() {
			QCOMPARE(1, 0); // Dummy test
		}
};

QTTESTUTIL_REGISTER_TEST(TestGrid);
#include "TestGrid.moc"
