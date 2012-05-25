#ifndef OBENTITY_H
#define OBENTITY_H

// Forward declaration
class btLocalGridProxy;

//TODO: move some stuff up here
class obEntity
{
public:
    virtual short getType() const = 0;

    static const short obEntityWrapperType = 1;
    static const short CellBorderEntityType = 2;

    inline void setProxy(btLocalGridProxy *proxy0)
    {
        proxy = proxy0;
    }


    inline void unsetProxy()
    {
        proxy = 0;
    }

    inline btLocalGridProxy *getProxy() const
    {
        return proxy;
    }


private:
    btLocalGridProxy *proxy;
};

#endif // OBENTITY_H
