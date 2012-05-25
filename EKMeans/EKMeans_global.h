#ifndef EKMEANS_GLOBAL_H
#define EKMEANS_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(EKMEANS_LIBRARY)
#  define EKMEANSSHARED_EXPORT Q_DECL_EXPORT
#else
#  define EKMEANSSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // EKMEANS_GLOBAL_H
