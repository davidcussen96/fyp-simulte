//
// Created by dcussen on 26/01/2019.
//
#include <assert.h>
#include "stack/phy/layer/LtePhyVueV2X.h"
#include "stack/phy/layer/LtePhyUeD2D.h"
#include "stack/phy/packet/LteFeedbackPkt.h"
#include "stack/d2dModeSelection/D2DModeSelectionBase.h"
#include <math.h>

Define_Module(LtePhyVueV2X);

LtePhyVueV2X::LtePhyVueV2X()
{
    // This probably aren't needed for V2X mode 4.
    handoverStarter_ = NULL;
    handoverTrigger_ = NULL;
}

LtePhyVueV2X::~LtePhyVueV2X()
{
}

void LtePhyUeD2D::initialize(int stage)
{
    LtePhyUe::initialize(stage);
    if (stage == 0)
    {
        averageCqiD2D_ = registerSignal("averageCqiD2D");
        d2dTxPower_ = par("d2dTxPower");
        d2dMulticastEnableCaptureEffect_ = par("d2dMulticastCaptureEffect");
        v2xDecodingTimer_ = NULL;
        updatePointers_ = NULL;
        updateSensingWindow_ = NULL;
    }
}

void LtePhyUeD2D::handleSelfMessage(cMessage *msg)
{
    if (msg->isName("v2xDecodingTimer"))
    {
        // clear buffer
        while (!v2xReceivedFrames_.empty())
        {
            // select one frame at a time from the buffer.
            LteAirFrame* frame = extractAirFrame();
            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());
            // decode the selected frame
            decodeAirFrame(frame, lteInfo);
        }

        delete msg;
        v2xDecodingTimer_ = NULL;
    }
    else if (msg->isName("updatePointers"))
    {
        // TODO: How do we trigger this self message every TTI.
        // Reset subframe at position 1000ms to {Subchannel, Subchannel, Subchannel}.
        std::vector<Subchannel*> subchannels =
                {new Subchannel(), new Subchannel(), new Subchannel()};
        sensingWindow[pointerToEnd] = subchannels;

        // Maintain pointer; Bit messy, definitely a better way of doing this.
        if (pointerToStart == 999) {pointerToStart = 0;} else pointerToStart++;

        if (elapsedTime < 1000) {
            pointerToEnd = 0;
        } else {
            pointerToEnd = pointerToStart -1;
            if (pointerToEnd < 0) pointerToEnd += 1000;
        }

        elapsedTime++;
        delete msg;
        updateSensingWindow_ = NULL;
    }
    else if (msg->"updateSensingWindow")
    {
        // For each subchannel in the subchannelList add to sensingWindow
        // First we need to determine when the AirFrame was received.
        curr_time = NOW;
        int secsAgo;
        for (std::vector<Subchannel*>::iterator it=subchannelList.begin(); it != subchannelList.end(); it++) {
            // Determine where subchannel is in sensing window and update it
            secsAgo = floor(curr_time - it->getTime());
            // Depending on the rbmap we can determine the position in the
            // inner vector (ie the subchannel).
            // However we could just insert it and not worry about ordering
            // of subchannel, could run into the problem of overwriting
            (sensingWindow[secsAgo]).push_front(it);
            // or (sensingWindow[secsAgo])[j] = it; j is 0->2.

        }
        subchannelList.clear();
        delete msg;
        updateSensingWindow_ = NULL;
    }
    else
        LtePhyUe::handleSelfMessage(msg);
}

void LtePhyVueV2X::chooseCsr(int threshold, int t1, int t2)
{
    // Assuming T1=0 and T2=100
    // empty subchannel instance or channel -> means subchannel is free
    // nullptr means: cannot use it.
    // TODO: Account T1 <= 4
    curr_time = NOW;
    Subchannel* Sa[t2-t1][3]= {
            {new Subchannel(),new Subchannel(),new Subchannel()}};
    std::vector<vector<Subchannel*>>::iterator frame;
    std::vector<Subchannel*>::iterator channel;
    int i, j, offset, secsAgo, rri, x, numCsrsSa;
    numCsrsSa = 0;
    while (numCsrsSa < 0.2(300)) {
        i = 0;
        for (frame = sensingWindow.begin(); frame != sensingWindow.end(); frame++) {
            if (i <= pointerToEnd)
            {
                offset = 999;
            }
            else offset = 1;

            j = 0;
            secsAgo = offset - pointerToEnd + i;
            for (channel = frame->begin(); channel != frame->end(); channel++) {
                if (!(channel->isSensed())) {
                    j++;
                    continue;
                }

                rri = channel->getRRI();
                x = (secsAgo-100)*-1;
                if (x <= 99) {
                    if (channel->isRsrpLessThan(threshold)) {
                        Sa[x][j] = channel;     // ie. We can use this csr. We can get rb's from subchannels rbmap
                        numCsrsSa++;            // Time from now + index in selection window.
                    } else {
                        Sa[x][j] = nullptr;
                    }
                }
                j++;
            }
            i++;
        }
        threshold += 3;
    }
    // Create Sb and send to MAC layer.
    // Iterate through Sa, set Sb is populated with free csr's and csr with min rssi.
    // How do we store csr with current highest avg rssi value.
    // 1. One initial pass through Sa.
    //      a. Store csr's with rssi values.
    //      b. count and include free csr's.
    Subchannel* Sb[100][3] = Sa;
    // {(rssi, x, y)}
    std::vector< tuple<int, int, int>> rssiValues;
    for( unsigned int x = 0; x < 100; x = x + 1 ) {
        for (unsigned int y = 0; y < 3; y = y + 1) {
            if (Sa[x][y] == nullptr) {
                Sb[x][y] = nullptr; // Free subchannel.
                numCsrsSb++;
            } else {
                rssi = Sa[x][y].getRssi();
                rssiValues.push_back(make_tuple(rssi, x, y));
            }
        }
    }
    // 2. Sort CSR's with rssi values. Continually add CSR's until % is exceeded.
    sort(rssiValues.rbegin(), rssiValues.rend());
    int numCsrsSb = 0;
    tuple<int, int, int> currentRssi;
    while (numCsrsSb < 0.2(300) && !rssiValues.empty()) {
        currentRssi = rssiValues.pop_back();
        Sb[get<1>(currentRssi)][get<2>(currentRssi)] =
                Sa[get<1>(currentRssi)][get<2>(currentRssi)];
        numCsrsSb++;
    }
    // TODO: Send Sb to MAC layer. Not sure this is correct.
    // So therefore I need to create Sb using rssi, how do i decide which csr's in Sa
    // to include in Sb.
    cMessage *msg = new cMessage("csr list");
    msg->addPar("Sa") = (void *) Sa;
    msg->par("Sa").configPointer(NULL, NULL, sizeof(Sa));
    send(msg, upperGateOut_);
}

// TODO: ***reorganize*** method
void LtePhyUeD2D::handleAirFrame(cMessage* msg)
{
    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    connectedNodeId_ = masterId_;
    LteAirFrame* frame = check_and_cast<LteAirFrame*>(msg);
    EV << "LtePhyUeD2D: received new LteAirFrame with ID " << frame->getId() << " from channel" << endl;
    //Update coordinates of this user
    // TODO: Handover is for use with ENB's.
    if (lteInfo->getFrameType() == HANDOVERPKT)
    {
        // check if handover is already in process
        if (handoverTrigger_ != NULL && handoverTrigger_->isScheduled())
        {
            delete lteInfo;
            delete frame;
            return;
        }

        handoverHandler(frame, lteInfo);
        return;
    }

    if (binder_->isInMulticastGroup(nodeId_,lteInfo->getMulticastGroupId()))
    {
        // HACK: if this is a multicast connection, change the destId of the airframe so that upper layers can handle it
        lteInfo->setDestId(nodeId_);
    }

    // send H-ARQ feedback up
    if (lteInfo->getFrameType() == HARQPKT || lteInfo->getFrameType() == GRANTPKT || lteInfo->getFrameType() == RACPKT || lteInfo->getFrameType() == D2DMODESWITCHPKT)
    {
        handleControlMsg(frame, lteInfo);
        return;
    }

    // this is a DATA packet

    // if the packet is a D2D multicast one, store it and decode it at the end of the TTI
    if (d2dMulticastEnableCaptureEffect_ && binder_->isInMulticastGroup(nodeId_,lteInfo->getMulticastGroupId()))
    {
        // if not already started, auto-send a message to signal the presence of data to be decoded
        if (v2xDecodingTimer_ == NULL)
        {
            v2xDecodingTimer_ = new cMessage("v2xDecodingTimer");
            v2xDecodingTimer_->setSchedulingPriority(10);          // last thing to be performed in this TTI
            scheduleAt(NOW, v2xDecodingTimer_);                    // This then calls decodeAirFrame
        }

        // TODO: Needs to be called every TTI. Where?
        if (updatePointers_ == NULL)
        {
            // Update sensing window pointers
            // Needs to occur every TTI
            updateSensingWindow_ = new cMessage("updatePointers");
            updateSensingWindow_->setSchedulingPriority(9);
            scheduleAt(NOW, updatePointers_);
        }

        // store frame, together with related control info
        frame->setControlInfo(lteInfo);
        storeAirFrame(frame);            // implements the capture effect

        return;                          // exit the function, decoding will be done later
    }

    if ((lteInfo->getUserTxParams()) != NULL)
    {
        int cw = lteInfo->getCw();
        if (lteInfo->getUserTxParams()->readCqiVector().size() == 1)
            cw = 0;
        double cqi = lteInfo->getUserTxParams()->readCqiVector()[cw];
        if (lteInfo->getDirection() == DL)
            emit(averageCqiDl_, cqi);
    }
    // apply decider to received packet
    bool result = true;
    RemoteSet r = lteInfo->getUserTxParams()->readAntennaSet();
    if (r.size() > 1)
    {
        // DAS
        for (RemoteSet::iterator it = r.begin(); it != r.end(); it++)
        {
            EV << "LtePhyUeD2D: Receiving Packet from antenna " << (*it) << "\n";

            /*
             * On UE set the sender position
             * and tx power to the sender das antenna
             */

//            cc->updateHostPosition(myHostRef,das_->getAntennaCoord(*it));
            // Set position of sender
//            Move m;
//            m.setStart(das_->getAntennaCoord(*it));
            RemoteUnitPhyData data;
            data.txPower=lteInfo->getTxPower();
            data.m=getRadioPosition();
            frame->addRemoteUnitPhyDataVector(data);
        }
        // apply analog models For DAS
        result=channelModel_->errorDas(frame,lteInfo);
    }
    else
    {
        //RELAY and NORMAL
        result = channelModel_->error(frame,lteInfo);
    }

    // update statistics
    if (result)
        numAirFrameReceived_++;
    else
        numAirFrameNotReceived_++;

    EV << "Handled LteAirframe with ID " << frame->getId() << " with result "
       << ( result ? "RECEIVED" : "NOT RECEIVED" ) << endl;

    cPacket* pkt = frame->decapsulate();

    // here frame has to be destroyed since it is no more useful
    delete frame;

    // attach the decider result to the packet as control info
    lteInfo->setDeciderResult(result);
    pkt->setControlInfo(lteInfo);

    // send decapsulated message along with result control info to upperGateOut_
    // TODO: Is this how we send from Phy to Mac. Need this for sending Sa.
    send(pkt, upperGateOut_);

    if (getEnvir()->isGUI())
        updateDisplayString();
}

// TODO: Do we need the handover methods? I dont think so.
void LtePhyUeD2D::triggerHandover()
{
    // stop active D2D flows (go back to Infrastructure mode)
    // currently, DM is possible only for UEs served by the same cell

    // trigger D2D mode switch
    cModule* enb = getSimulation()->getModule(binder_->getOmnetId(masterId_));
    D2DModeSelectionBase *d2dModeSelection = check_and_cast<D2DModeSelectionBase*>(enb->getSubmodule("lteNic")->getSubmodule("d2dModeSelection"));
    d2dModeSelection->doModeSwitchAtHandover(nodeId_, false);

    LtePhyUe::triggerHandover();
}

void LtePhyUeD2D::doHandover()
{
    // amc calls
    LteAmc *oldAmc = getAmcModule(masterId_);
    LteAmc *newAmc = getAmcModule(candidateMasterId_);
    assert(newAmc != NULL);
    oldAmc->detachUser(nodeId_, D2D);
    newAmc->attachUser(nodeId_, D2D);

    LtePhyUe::doHandover();

    // call mode selection module to check if DM connections are possible
    cModule* enb = getSimulation()->getModule(binder_->getOmnetId(masterId_));
    D2DModeSelectionBase *d2dModeSelection = check_and_cast<D2DModeSelectionBase*>(enb->getSubmodule("lteNic")->getSubmodule("d2dModeSelection"));
    d2dModeSelection->doModeSwitchAtHandover(nodeId_, true);
}

void LtePhyUeD2D::handleUpperMessage(cMessage* msg)
{
    // Tells Phy layer to transmit and on what channel/rb's.
    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    // Store the RBs used for transmission. For interference computation
    RbMap rbMap = lteInfo->getGrantedBlocks();
    UsedRBs info;
    info.time_ = NOW;
    info.rbMap_ = rbMap;

    usedRbs_.push_back(info);

    std::vector<UsedRBs>::iterator it = usedRbs_.begin();
    while (it != usedRbs_.end())  // purge old allocations
    {
        if (it->time_ < NOW - 0.002)
            usedRbs_.erase(it++);
        else
            ++it;
    }
    lastActive_ = NOW;

    if (lteInfo->getFrameType() == DATAPKT && lteInfo->getUserTxParams() != NULL)
    {
        double cqi = lteInfo->getUserTxParams()->readCqiVector()[lteInfo->getCw()];
        if (lteInfo->getDirection() == D2D)
            emit(averageCqiD2D_, cqi);
    }

    EV << NOW << " LtePhyUeD2D::handleUpperMessage - message from stack" << endl;
    LteAirFrame* frame = NULL;

    if (lteInfo->getFrameType() == HARQPKT || lteInfo->getFrameType() == GRANTPKT || lteInfo->getFrameType() == RACPKT)
    {
        frame = new LteAirFrame("harqFeedback-grant");
    }
    else
    {
        // create LteAirFrame and encapsulate the received packet
        frame = new LteAirFrame("airframe");
    }

    frame->encapsulate(check_and_cast<cPacket*>(msg));

    // initialize frame fields

    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(TTI);
    // set current position
    lteInfo->setCoord(getRadioPosition());

    lteInfo->setTxPower(txPower_);
    lteInfo->setD2dTxPower(d2dTxPower_);
    frame->setControlInfo(lteInfo);

    EV << "LtePhyUeD2D::handleUpperMessage - " << nodeTypeToA(nodeType_) << " with id " << nodeId_
       << " sending message to the air channel. Dest=" << lteInfo->getDestId() << endl;

    sendBroadcast(frame);

}

void LtePhyUeD2D::storeAirFrame(LteAirFrame* newFrame)
{
    // When storing an AirFrame we must update the sensing window also?
    // Won't know what's in it until we decode it so should the updating sensing window take place
    // in the decodeAirFrame instead? I think yes!

    UserControlInfo* newInfo = check_and_cast<UserControlInfo*>(newFrame->getControlInfo());
    Coord myCoord = getCoord();
    std::vector<double> rsrpVector;

    if (strcmp(par("d2dMulticastCaptureEffectFactor"),"RSRP") == 0)
    {
        // SCI from AirFrame msg -> get SCI from AirFrame.
        rsrpVector = channelModel_->getRSRP_D2D(newFrame, newInfo, nodeId_, myCoord);
        int rsrp;
        rssiVector = channelModel->getSINR_D2D(newFrame, newInfo, nodeId_, myCoord);
        int rssi;
        // What does an RbMap look like? Does it give you the range of RB's?
        // Can we determine which subchannel the granted blocks belong;(for example) 1-10(SC1), 11-20(SC2), 21-30(SC3).
        // From here we can update the sensing window I think.
        // 1. Determine what subchannel and what subframe AirFrame was sent in.
        // 2. Update sensing window with the information.
        RbMap rbmap = newInfo->getGrantedBlocks();
        RbMap::iterator it;
        std::map<Band, unsigned int>::iterator jt;
        //for each Remote unit used to transmit the packet
        for (it = rbmap.begin(); it != rbmap.end(); ++it)
        {
            //for each logical band used to transmit the packet
            for (jt = it->second.begin(); jt != it->second.end(); ++jt)
            {
                Band band = jt->first;
                if (jt->second == 0) // this Rb is not allocated
                    continue;

                rsrp += rsrpVector.at(band);
                rssi += rssiVector.at(band);
                //allocatedRbs++;
            }
        }
        // Create a Subchannel
        Subchannel* subchannel = new Subchannel(sci, rbmap, rsrp, rssi);
        subchannelList.push_front(subchannel);

        if (updateSensingWindow_ == NULL)
        {
            // Needs to happen every time an AirFrame is receieved.
            updateSensingWindow_ = new cMessage("updateSensingWindow");
            updateSensingWindow_->setSchedulingPriority(10);
            scheduleAt(NOW, updateSensingWindow_);
        }

        EV << NOW << " LtePhyVueV2X::storeAirFrame - from node " << newInfo->getSourceId();
    }

    v2xReceivedFrames_.push_back(newFrame);
    delete newFrame;

}

LteAirFrame* LtePhyUeD2D::extractAirFrame()
{
    // implements the capture effect
    // the vector is storing the frame received from the strongest/nearest transmitter

    return v2xReceivedFrames_.front();
}

void LtePhyUeD2D::decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo)
{
    EV << NOW << " LtePhyUeD2D::decodeAirFrame - Start decoding..." << endl;

    // apply decider to received packet
    bool result = true;

    RemoteSet r = lteInfo->getUserTxParams()->readAntennaSet();
    if (r.size() > 1)
    {
        // DAS
        for (RemoteSet::iterator it = r.begin(); it != r.end(); it++)
        {
            EV << "LtePhyUeD2D::decodeAirFrame: Receiving Packet from antenna " << (*it) << "\n";

            /*
             * On UE set the sender position
             * and tx power to the sender das antenna
             */

//            cc->updateHostPosition(myHostRef,das_->getAntennaCoord(*it));
            // Set position of sender
//            Move m;
//            m.setStart(das_->getAntennaCoord(*it));
            RemoteUnitPhyData data;
            data.txPower=lteInfo->getTxPower();
            data.m=getRadioPosition();
            frame->addRemoteUnitPhyDataVector(data);
        }
        // apply analog models For DAS
        result=channelModel_->errorDas(frame,lteInfo);
    }
    else
    {
        //RELAY and NORMAL
        if (lteInfo->getDirection() == D2D_MULTI)
            result = channelModel_->error_D2D(frame,lteInfo,bestRsrpVector_);
        else
            result = channelModel_->error(frame,lteInfo);
    }

    // update statistics
    if (result)
        numAirFrameReceived_++;
    else
        numAirFrameNotReceived_++;

    EV << "Handled LteAirframe with ID " << frame->getId() << " with result "
       << ( result ? "RECEIVED" : "NOT RECEIVED" ) << endl;

    cPacket* pkt = frame->decapsulate();

    // attach the decider result to the packet as control info
    lteInfo->setDeciderResult(result);
    pkt->setControlInfo(lteInfo);

    // send decapsulated message along with result control info to upperGateOut_
    send(pkt, upperGateOut_);

    if (getEnvir()->isGUI())
        updateDisplayString();
}


void LtePhyUeD2D::sendFeedback(LteFeedbackDoubleVector fbDl, LteFeedbackDoubleVector fbUl, FeedbackRequest req)
{
    // Do we need sendFeedback for v2x mode 4.
    // Feedback is usually sent to the local enodeb on the quality of the channel.
    Enter_Method("SendFeedback");
    EV << "LtePhyUeD2D: feedback from Feedback Generator" << endl;

    //Create a feedback packet
    LteFeedbackPkt* fbPkt = new LteFeedbackPkt();
    //Set the feedback
    fbPkt->setLteFeedbackDoubleVectorDl(fbDl);
    fbPkt->setLteFeedbackDoubleVectorDl(fbUl);
    fbPkt->setSourceNodeId(nodeId_);
    UserControlInfo* uinfo = new UserControlInfo();
    uinfo->setSourceId(nodeId_);
    uinfo->setDestId(masterId_);
    uinfo->setFrameType(FEEDBACKPKT);
    uinfo->setIsCorruptible(false);
    // create LteAirFrame and encapsulate a feedback packet
    LteAirFrame* frame = new LteAirFrame("feedback_pkt");
    frame->encapsulate(check_and_cast<cPacket*>(fbPkt));
    uinfo->feedbackReq = req;
    uinfo->setDirection(UL);
    simtime_t signalLength = TTI;
    uinfo->setTxPower(txPower_);
    uinfo->setD2dTxPower(d2dTxPower_);
    // initialize frame fields

    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(signalLength);

    uinfo->setCoord(getRadioPosition());

    frame->setControlInfo(uinfo);
    //TODO access speed data Update channel index
//    if (coherenceTime(move.getSpeed())<(NOW-lastFeedback_)){
//        deployer_->channelIncrease(nodeId_);
//        deployer_->lambdaIncrease(nodeId_,1);
//    }
    lastFeedback_ = NOW;
    EV << "LtePhy: " << nodeTypeToA(nodeType_) << " with id "
       << nodeId_ << " sending feedback to the air channel" << endl;
    sendUnicast(frame);
}

void LtePhyUeD2D::finish()
{
    if (getSimulation()->getSimulationStage() != CTX_FINISH)
    {
        // do this only at deletion of the module during the simulation

        // amc calls
        LteAmc *amc = getAmcModule(masterId_);
        if (amc != NULL)
            amc->detachUser(nodeId_, D2D);

        LtePhyUe::finish();
    }
}
