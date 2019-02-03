//
// Created by dcussen on 26/01/2019.
//

#ifndef FYP_SIMULTE_SUBCHANNEL_H
#define FYP_SIMULTE_SUBCHANNEL_H

// TODO: Two constructors -> One for empty subchannels where isFree = true;
// -> One for occupied subchannels where all values are filled and isFree = false;
class Subchannel {
private:
    RbMap rbmap;
    Sci sci;
    int rsrp;
    int rssi;
    bool notSensed;
    bool isFree;

public:
    Subchannel();

    Subchannel(RbMap rbmap, Sci sci);
    // Getters and setters here also.
    // Determine if rsrp < Th
    // TODO: Where do we get/calculate the threshold Th.
    bool isRsrpLessThan(int Th);

    RbMap getRbMap() {return rbmap;}
    Sci getSci() {return sci;}
    void setRsrp() {rsrp = sci->getRsrp();}
    int getRsrp() {return rsrp;}
    int getRssi() { return rssi;}
    void setRssi() { rssi=sci->getRssi();}
    // Return sci's resource reservation interval * 10
    int getRRI() {return sci->resource_res * 10}
    bool notSensed() {return notSensed;}
    bool isFree() {return isFree;}

};


#endif //FYP_SIMULTE_SUBCHANNEL_H
