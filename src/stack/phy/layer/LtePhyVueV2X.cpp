//
// Created by dcussen on 26/01/2019.
//
#include <assert.h>
#include "stack/phy/layer/LtePhyVueV2X.h"
#include "stack/phy/layer/LtePhyUeD2D.h"
#include "stack/phy/packet/LteFeedbackPkt.h"
#include "stack/d2dModeSelection/D2DModeSelectionBase.h"

Define_Module(LtePhyVueV2X);

LtePhyVueV2X::LtePhyVueV2X()
{
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
        d2dDecodingTimer_ = NULL;
    }
}

void LtePhyUeD2D::handleSelfMessage(cMessage *msg)
{
    if (msg->isName("d2dDecodingTimer"))
    {
        // clear buffer
        while (!d2dReceivedFrames_.empty())
        {
            // select one frame at a time from the buffer.
            LteAirFrame* frame = extractAirFrame();
            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());
            // decode the selected frame
            decodeAirFrame(frame, lteInfo);
        }

        delete msg;
        d2dDecodingTimer_ = NULL;
    }
    else
        LtePhyUe::handleSelfMessage(msg);

    if (msg->isName("updateSensingWindow"))
    {
        // Reset subframe at position 1000ms to {Subchannel, Subchannel, Subchannel}.
        std::vector<Subchannel*> subchannels = 3*{Subchannel*};
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
}

// TODO: ***reorganize*** method
void LtePhyUeD2D::handleAirFrame(cMessage* msg)
{
    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    connectedNodeId_ = masterId_;
    LteAirFrame* frame = check_and_cast<LteAirFrame*>(msg);
    EV << "LtePhyUeD2D: received new LteAirFrame with ID " << frame->getId() << " from channel" << endl;
    //Update coordinates of this user
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

    // Check if the frame is for us ( MacNodeId matches or - if this is a multicast communication - enrolled in multicast group)
    if (lteInfo->getDestId() != nodeId_ && !(binder_->isInMulticastGroup(nodeId_, lteInfo->getMulticastGroupId())))
    {
        EV << "ERROR: Frame is not for us. Delete it." << endl;
        EV << "Packet Type: " << phyFrameTypeToA((LtePhyFrameType)lteInfo->getFrameType()) << endl;
        EV << "Frame MacNodeId: " << lteInfo->getDestId() << endl;
        EV << "Local MacNodeId: " << nodeId_ << endl;
        delete lteInfo;
        delete frame;
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
        if (d2dDecodingTimer_ == NULL)
        {
            d2dDecodingTimer_ = new cMessage("d2dDecodingTimer");
            d2dDecodingTimer_->setSchedulingPriority(10);          // last thing to be performed in this TTI
            scheduleAt(NOW, d2dDecodingTimer_);                    // This then calls decodeAirFrame
        }

        if (updateSensingWindow_ == NULL)
        {
            // Update sensing window pointers
            updateSensingWindow_ = new cMessage("updateSensingWindow");
            updateSensingWindow_->setSchedulingPriority(10);
            scheduleAt(NOW, updateSensingWindow_);
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
    send(pkt, upperGateOut_);

    if (getEnvir()->isGUI())
        updateDisplayString();
}

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
        //rsrpMean = sum / allocatedRbs;
        //EV << NOW << " LtePhyUeD2D::storeAirFrame - Average RSRP from node " << newInfo->getSourceId() << ": " << rsrpMean ;
    }

    d2dReceivedFrames_.push_back(newFrame);
    delete newFrame;

    /*
    if (!d2dReceivedFrames_.empty())
    {
        LteAirFrame* prevFrame = d2dReceivedFrames_.front();
        if (!useRsrp && distance < nearestDistance_)
        {
            EV << "[ < nearestDistance: " << nearestDistance_ << "]" << endl;

            // remove the previous frame
            d2dReceivedFrames_.pop_back();
            delete prevFrame;

            nearestDistance_ = distance;
            d2dReceivedFrames_.push_back(newFrame);
        }
        else if (rsrpMean > bestRsrpMean_)
        {
            EV << "[ > bestRsrp: " << bestRsrpMean_ << "]" << endl;

            // remove the previous frame
            d2dReceivedFrames_.pop_back();
            delete prevFrame;

            bestRsrpMean_ = rsrpMean;
            bestRsrpVector_ = rsrpVector;
            d2dReceivedFrames_.push_back(newFrame);
        }
        else
        {
            // this frame will not be decoded
            delete newFrame;
        }
    }
    */
}

LteAirFrame* LtePhyUeD2D::extractAirFrame()
{
    // implements the capture effect
    // the vector is storing the frame received from the strongest/nearest transmitter

    return d2dReceivedFrames_.front();
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
