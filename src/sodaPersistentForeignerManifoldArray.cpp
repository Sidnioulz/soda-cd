#include "sodaPersistentForeignerManifold.h"
#include "sodaPersistentForeignerManifoldArray.h"

int sodaPersistentForeignerManifoldArray::findLinearSearch(const void *local, const void *foreign)
{
    int index=size();
    int i;

    for (i=0;i<size();i++)
    {
        sodaPersistentForeignerManifold *&pfm = at(i);

        if (pfm->getLocalBody() == local && pfm->getForeignBody() == foreign)
        {
            index = i;
            break;
        }
    }
    return index;
}
