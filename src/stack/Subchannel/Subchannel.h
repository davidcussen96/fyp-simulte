//
// Created by dcussen on 26/01/2019.
//

#ifndef FYP_SIMULTE_SUBCHANNEL_H
#define FYP_SIMULTE_SUBCHANNEL_H


class Subchannel {
    int rbStart;    // May need to be replaced with an rbmap
    int rbEnd;
    Sci sci;
    int rsrp;
    int rssi;
    bool notSensed = false;
    bool isFree;

    // Getters and setters here

    // isFree?
    bool isFree() {return isFree;}

    // Determine if rsrp < Th
    bool isRsrpLessThan(int Th) {
        if (sci->getRsrp() < Th)
        {
            return true;
        } else return false;
    }

    // Return sci's resource reservation interval * 10
    int getRRI() {return sci->resource_res * 10}

};


#endif //FYP_SIMULTE_SUBCHANNEL_H
