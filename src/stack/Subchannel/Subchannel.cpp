//
// Created by dcussen on 26/01/2019.
//

#include "Subchannel.h"

// Subchannel constructors
Subchannel::Subchannel()
{
    isFree = true;
}

Subchannel::Subchannel(RbMap rbmap, Sci sci)
{
    rbmap = rbmap;
    sci = sci;
    setRsrp();
    setRssi();
    isFree = false;
    notSensed = false;
}

bool Subchannel::isRsrpLessThan(int Th)
{
    if (getRsrp() < Th)
    {
        return true;
    } else return false;
}