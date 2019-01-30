//
// Created by dcussen on 26/01/2019.
//

#ifndef FYP_SIMULTE_LTEPHYVUEV2X_H
#define FYP_SIMULTE_LTEPHYVUEV2X_H

#include "stack/phy/layer/LtePhyUeD2D.h"

class LtePhyVueV2X : public LtePhyUeD2D
{
protected:

    // D2D Tx Power
    double d2dTxPower_;

    /*
     * Capture Effect for D2D Multicast communications
     */
    bool d2dMulticastEnableCaptureEffect_;
    // Matrix representing our sensing window.
    std::vector<vector<Subchannel*>> sensingWindow(1000, vector<Subchannel*>(3));
    double nearestDistance_;
    std::vector<double> bestRsrpVector_;
    double bestRsrpMean_;
    std::vector<LteAirFrame*> d2dReceivedFrames_; // airframes received in the current TTI. Instead of only decoding one
                                                  // like in d2d we must decode all received airframesOnly one will be
                                                  // decoded.
    cMessage* d2dDecodingTimer_;                  // timer for triggering decoding at the end of the TTI. Started
    // when the first airframe is received
    cMessage* updateSensingWindow_;                // Called every TTI to update our sensing window.
    int pointerToEnd;
    int pointerToStart;
    int elapsedTime;
    std::vector<Subchannel*> listOfSubchannels;
    void storeAirFrame(LteAirFrame* newFrame);
    LteAirFrame* extractAirFrame();
    void decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo);
    // ---------------------------------------------------------------- //

    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleAirFrame(cMessage* msg);
    virtual void handleUpperMessage(cMessage* msg);
    virtual void handleSelfMessage(cMessage *msg);

    virtual void triggerHandover();
    virtual void doHandover();

public:
    LtePhyUeD2D();
    virtual ~LtePhyUeD2D();

    virtual void sendFeedback(LteFeedbackDoubleVector fbDl, LteFeedbackDoubleVector fbUl, FeedbackRequest req);
    virtual double getTxPwr(Direction dir = UNKNOWN_DIRECTION)
    {
        if (dir == D2D)
            return d2dTxPower_;
        return txPower_;
    }
};

#endif  /* _LTE_AIRPHYUED2D_H_ */


#endif //FYP_SIMULTE_LTEPHYVUEV2X_H
