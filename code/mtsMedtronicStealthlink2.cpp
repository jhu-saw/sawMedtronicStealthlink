/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsMedtronicStealthlink.cpp 3597 2012-04-12 01:32:14Z wliu25 $

  Author(s): Peter Kazanzides, Anton Deguet, Daniel Mirota
  Created on: 2006

  (C) Copyright 2007-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/



// Stealthlink definitions
#include <sawMedtronicStealthlink/mtsMedtronicStealthlink2.h>

#include <cisstCommon/cmnPortability.h>
#include <cisstCommon/cmnXMLPath.h>
#include <cisstMultiTask/mtsFixedSizeVectorTypes.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#ifdef CISST_HAS_STEALTHLINK
#include <StealthLink/StealthLink.h>
#endif

#ifdef CISST_HAS_STEALTHLINK
#if (CISST_OS == CISST_WINDOWS)
// Prevent inclusion of <winsock.h> from <windows.h>.
#define _WINSOCKAPI_
#endif
#endif

#ifdef sawMedtronicStealthlink_IS_SIMULATOR
void AsCL_MSG(int CMN_UNUSED(verbose_level), char * CMN_UNUSED(msg), ...) {}
#endif

CMN_IMPLEMENT_SERVICES_DERIVED(mtsMedtronicStealthlink, mtsTaskFromSignal)

void mtsMedtronicStealthlink::Init(void)
{

    DataMapContainerInsertReturnValue dataMapInsertReturn;

    mtsInterfaceProvided * provided = 0;

    provided = AddInterfaceProvided("Controller");
    if (provided){
        //provided->AddCommandRead(&mtsMedtronicStealthlink::GetTool, this, "GetTool");
        //provided->AddCommandRead(&mtsMedtronicStealthlink::GetFrame, this, "GetFrame");
    }


    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(new MNavStealthLink::SurgicalPlan(),new mtsMedtronicStealthlink::SurgicalPlan()));
    if (dataMapInsertReturn.second){
        provided = AddInterfaceProvided("SurgicalPlan");
        dataMapInsertReturn.first->second->ConfigureInterfaceProvided(provided);
    }else{
        //something went wrong
    }


    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(new MNavStealthLink::Registration(),new mtsMedtronicStealthlink::Registration()));

    // Add interface for registration, ideally we should standardize such interface commands/payloads
    // maybe we should create a separate state table for registration?  Would only advance if changed.
    if (dataMapInsertReturn.second){
        provided = AddInterfaceProvided("Registration");
        dataMapInsertReturn.first->second->ConfigureInterfaceProvided(provided);
    }else{
        //something went wrong
    }


    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(new MNavStealthLink::Exam(),new mtsMedtronicStealthlink::ExamInformation()));

    if (dataMapInsertReturn.second){
             provided = AddInterfaceProvided("ExamInformation");
             if(provided){
                 provided->AddCommandVoid(&mtsMedtronicStealthlink::RequestExamInformation, this, "RequestExamInformation");
             }
             dataMapInsertReturn.first->second->ConfigureInterfaceProvided(provided);
    } else {
        //something went wrong
    }



}

mtsMedtronicStealthlink::mtsMedtronicStealthlink(const std::string & taskName) :
    mtsTaskFromSignal(taskName),
    StealthlinkPresent(false),
    //CurrentTool(0),
    //CurrentFrame(0),
    //RegistrationData(this),
    //ToolData(this),
    //FrameData(this),
    myCallbackMember(this)
{
    Init();
}


mtsMedtronicStealthlink::~mtsMedtronicStealthlink()
{
    //Cleanup();


    ToolsContainer::iterator it;
    const ToolsContainer::iterator end = Tools.end();
    for (it = Tools.begin(); it != end; it++) {
        //delete (*it);
        //*it = 0;
    }
    Tools.clear();


    DataMapContainer::iterator it2;
    const DataMapContainer::iterator end2 = myDataMap.end();
    for (it2 = myDataMap.begin(); it2 != end2; it2++) {
        if(it2->first)
            delete (it2->first);
        if(it2->second)
            delete (it2->second);
        it2->second = 0;
    }
    myDataMap.clear();


    if (this->StealthServerProxy) {
        delete this->StealthServerProxy;
        this->StealthServerProxy = 0;
    }

}


// Windows defines a SetPort macro
#ifdef SetPort
#undef SetPort
#endif

void mtsMedtronicStealthlink::Configure(const std::string &filename)
{

    //TODO: Update to support myDataMap and myStateTableMap

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    cmnXMLPath config;
    config.SetInputSource(filename);

    // initialize serial port
    std::string ipAddress;
    if (!config.GetXMLValue("/tracker/controller", "@ip", ipAddress, "192.168.0.1")) {
        CMN_LOG_CLASS_INIT_WARNING << "Configure: IP address not found, using default: " << ipAddress << std::endl;
    }

#ifndef sawMedtronicStealthlink_IS_SIMULATOR
    // Configure MedtronicStealthlink interface

    // Set StealthLink server IP address
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting Stealthink IP address = " << ipAddress << std::endl;
    this->StealthServerProxy = new MNavStealthLink::StealthServer(const_cast<char *>(ipAddress.c_str()));
#endif

    std::string xpathString = "count(/tracker/tools/*)";
    std::string toolCountString = "";

    config.GetXMLValue("",xpathString,toolCountString);

    std::stringstream toolCountStream;
    toolCountStream << toolCountString;

    unsigned int toolCount;
    toolCountStream >> toolCount;


    // add pre-defined tools (up to 100)
    unsigned int maxToolCount = std::min(toolCount,static_cast<unsigned int>(100));
    for (unsigned int i = 0; i < maxToolCount; i++) {
        std::stringstream context;
        std::string stealthName, name;
        context << "/tracker/tools/tool[" << i+1 << "]";
        config.GetXMLValue(context.str().c_str(), "@stealthName", stealthName, "");
        config.GetXMLValue(context.str().c_str(), "@name", name, "");
        if (stealthName.empty() && name.empty()) {
            break;
        }
        if (stealthName.empty()) {
            AddTool(name, name);
        } else if (name.empty()) {
            AddTool(stealthName, stealthName);
        } else {
            AddTool(stealthName, name);
        }
    }

#ifdef sawMedtronicStealthlink_IS_SIMULATOR
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using simulated Stealthstation" << std::endl;
    StealthlinkPresent = true;
#else
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: initializing Stealthlink" << std::endl;
    //

    MNavStealthLink::Error StealthlinkError;

    // Connect to the server
    StealthlinkPresent = this->StealthServerProxy->connect(StealthlinkError) ? true : false;

    if (!StealthlinkPresent) {
        CMN_LOG_CLASS_INIT_ERROR << "Failed to connect to Stealth server application on host "
                         << this->StealthServerProxy->getHost() << ", port " << this->StealthServerProxy->getPort() << ": "
                         << StealthlinkError.reason() << std::endl;
        CMN_LOG_CLASS_INIT_ERROR << "Configure: could not Initialize StealthLink" << std::endl;
    }else{



    //attach all of the callbacks


    instrumentSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Instrument>(*(this->StealthServerProxy));
    frameSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Frame>(*(this->StealthServerProxy));
    registrationSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Registration>(*(this->StealthServerProxy));
    //examSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Exam>(*(this->StealthServerProxy));
    surgicalPlanSubscription = new MNavStealthLink::Subscription<MNavStealthLink::SurgicalPlan>(*(this->StealthServerProxy));


    instrumentSubscription->start(this->myCallbackMember);
    frameSubscription->start(this->myCallbackMember);
    registrationSubscription->start(this->myCallbackMember);
    //examSubscription->start(this->myCallbackMember);
    surgicalPlanSubscription->start(this->myCallbackMember);
    }
#endif
}


void mtsMedtronicStealthlink::ResetAllTools(void)
{
    ToolsContainer::iterator it;
    const ToolsContainer::iterator end = Tools.end();
    for (it = Tools.begin(); it != end; it++) {
        (*it)->MarkerPosition.SetValid(false);
        (*it)->TooltipPosition.SetValid(false);
    }
}


mtsMedtronicStealthlink::Tool * mtsMedtronicStealthlink::FindTool(const std::string & stealthName) const
{
    ToolsContainer::const_iterator it;
    const ToolsContainer::const_iterator end = Tools.end();
    for (it = Tools.begin(); it != end; it++) {
        if ((*it)->GetStealthName() == stealthName) {
            return *it;
        }
    }
    return 0;
}


mtsMedtronicStealthlink::Tool * mtsMedtronicStealthlink::AddTool(const std::string & stealthName, const std::string & interfaceName)
{
    //TODO: Update to support myDataMap and myStateTableMap

    // First, check if tool has already been added
    Tool * tool = FindTool(stealthName);
    if (tool) {
        if (tool->GetInterfaceName() == interfaceName) {
            CMN_LOG_CLASS_RUN_WARNING << "AddTool: tool " << stealthName << " already exists with interface "
                                      << interfaceName << std::endl;
            return tool;
        }
        // We could support having the same tool in multiple interfaces, but we would need to maintain
        // an array of CurrentTools, or loop through the entire Tools list in the Run method (to assign the
        // MarkerPosition and TooltipPosition).
        CMN_LOG_CLASS_RUN_ERROR << "AddTool: tool " << stealthName << " already exists in interface "
                                << tool->GetInterfaceName() << ", could not create new interface "
                                << interfaceName << std::endl;
        return 0;
    }

    // Next, check if interface has already been added
    mtsInterfaceProvided * provided = GetInterfaceProvided(interfaceName);
    if (provided) {
        CMN_LOG_CLASS_RUN_ERROR << "AddTool: interface " << interfaceName << " already exists." << std::endl;
        return 0;
    }

    // Create the tool and add it to the list
    tool = new Tool (stealthName, interfaceName);
    Tools.push_back(tool);


    DataMapContainerInsertReturnValue dataMapInsertReturn;

    MNavStealthLink::DataItem * newItem = 0;
    myGenericObject * newObject = tool;

    if (strcmp(interfaceName.c_str(),"Frame") == 0){
        MNavStealthLink::Frame * newFrame = new MNavStealthLink::Frame();
        newFrame->name = stealthName;
        newItem = newFrame;
    } else if (strcmp(interfaceName.c_str(),"Tool") == 0 ){
        MNavStealthLink::Instrument * newTool = new MNavStealthLink::Instrument();
        newTool->name = stealthName;
        newItem = newTool;
    }else{
        //warning not a Frame or Tool assuming Tool
        MNavStealthLink::Instrument * newTool = new MNavStealthLink::Instrument();
        newTool->name = stealthName;
        newItem = newTool;
    }


    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(newItem,newObject));

    if (dataMapInsertReturn.second){
        CMN_LOG_CLASS_RUN_VERBOSE << "AddTool: adding " << stealthName << " to interface " << interfaceName << std::endl;
        provided = AddInterfaceProvided(interfaceName);
        dataMapInsertReturn.first->second->ConfigureInterfaceProvided(provided);
    } else {
        //something went wrong
    }

    return tool;
}


void mtsMedtronicStealthlink::RequestExamInformation(void)
{
    if (StealthlinkPresent) {
        //exam_info the_exam_info;
#ifndef sawMedtronicStealthlink_IS_SIMULATOR
        //this->Client->GetDataForCode(GET_EXAM_INFO,
        //                             reinterpret_cast<void*>(&the_exam_info));
#endif
        //ExamInformationMember.VoxelScale[0] = the_exam_info.voxel_scale[0];
        //ExamInformationMember.VoxelScale[1] = the_exam_info.voxel_scale[1];
        //ExamInformationMember.VoxelScale[2] = the_exam_info.voxel_scale[2];
        //ExamInformationMember.Size[0] = the_exam_info.size[0];
        //ExamInformationMember.Size[1] = the_exam_info.size[1];
        //ExamInformationMember.Size[2] = the_exam_info.size[2];
        //ExamInformationMember.Valid = the_exam_info.valid;
    }
}


void mtsMedtronicStealthlink::RequestSurgicalPlan(void)
{
    if (StealthlinkPresent) {
        //surg_plan the_surg_plan;
#ifndef sawMedtronicStealthlink_IS_SIMULATOR
        //this->Client->GetDataForCode(GET_SURGICAL_PLAN,
        //                             reinterpret_cast<void*>(&the_surg_plan));
#endif
        //SurgicalPlan[0] = the_surg_plan.entry[0];
        //SurgicalPlan[1] = the_surg_plan.entry[1];
        //SurgicalPlan[2] = the_surg_plan.entry[2];
        //SurgicalPlan[3] = the_surg_plan.target[0];
        //SurgicalPlan[4] = the_surg_plan.target[1];
        //SurgicalPlan[5] = the_surg_plan.target[2];
    }
}


/*void mtsMedtronicStealthlink::GetSurgicalPlan(mtsDoubleVec & plan) const
{
    //mtsDoubleVec * currentPlan = dynamic_cast<mtsDoubleVec *>(myPlanIterator->second);
    //plan = *currentPlan;
}*/


void mtsMedtronicStealthlink::Run(void)
{

    ProcessQueuedCommands();

}

void mtsMedtronicStealthlink::Startup(void){
    //create and start stealthlink run thread

    if (StealthlinkPresent) {
        StealthServerProxyThread.Create<mtsMedtronicStealthlink, void *>
                        (this, &mtsMedtronicStealthlink::StealthlinkRun, 0 );
    }


}

void * mtsMedtronicStealthlink::StealthlinkRun(void * ) {
    try {
        StealthServerProxy->run();
    }
    catch (std::exception e) {
        CMN_LOG_CLASS_RUN_ERROR << "Configure: Caught exception while listening to Stealth server: " << e.what() << std::endl;
    }
    catch (...) {
        CMN_LOG_CLASS_RUN_ERROR << "Configure: Caught unknown exception while listening to Stealth server" << std::endl;
    }

    return 0;

}


void mtsMedtronicStealthlink::Cleanup(void)
{
    //if (myLock.GetLockState() == osaMutex::LOCKED)
    //    return;
    //myLock.Lock();

    //Stop stealthlink
    if(StealthServerProxyThread.IsValid() && this->StealthServerProxy){
        this->StealthServerProxy->stop();

        //Wait for it's thread
        //StealthServerProxyThread.Wait();

        //If it timesout kill the thread
        StealthServerProxyThread.Delete();
    }


    if(this->StealthlinkPresent && this->StealthServerProxy){
        this->StealthServerProxy->disconnect();
        this->StealthlinkPresent = this->StealthServerProxy->isConnected();
    }



    CMN_LOG_CLASS_RUN_VERBOSE << "Cleanup: finished" << std::endl;

    //myLock.Unlock();


}


void mtsMedtronicStealthlink::myCallback::operator ()(const MNavStealthLink::DataItem& item_in){
    //the DataMap must be pre-allocated before a callback is called.
    DataMapContainer::iterator current_item = my_parent->myDataMap.find(&item_in);
    if (current_item != my_parent->myDataMap.end()) {
        current_item->second->AssignAndAdvance(item_in);
    }else{
        //CMN_LOG_CLASS_RUN_WARNING << "Run: adding new data at runtime is currently unsupported"  << std::endl;
    }

}


typedef float floatArray44[4][4];

void frameConversion(vctFrm3 & result, const floatArray44 & input) {
    size_t row, col;
    for (row = 0; row < 3; row++) {
        for (col = 0; col < 3; col++) {
            result.Rotation().at(row, col) =  input[row][col];
        }
        result.Translation().at(row) = input[row][3];
    }
}



/* void mtsMedtronicStealthlink::operator()(MNavStealthLink::Instrument& instrument_in){
    ToolData = instrument_in;
    ProbeCal = instrument_in;

    // update tool interfaces data
    if (ToolData.Valid()) {
        if (!CurrentTool || (CurrentTool->GetStealthName() != ToolData.GetName())) {
            CurrentTool = FindTool(ToolData.GetName());
            if (!CurrentTool) {
                CMN_LOG_CLASS_INIT_VERBOSE << "Run: adding new tool \""
                                           << ToolData.GetName() << "\"" << std::endl;
                CurrentTool = AddTool(ToolData.GetName(), ToolData.GetName());
            }
            if (CurrentTool) {
                CMN_LOG_CLASS_RUN_VERBOSE << "Run: current tool is now \""
                                          << CurrentTool->GetInterfaceName() << "\"" << std::endl;
            } else {
                CMN_LOG_CLASS_RUN_ERROR << "Run: unable to add provided interface for new tool \""
                                        << ToolData.GetName() << "\"" << std::endl;
            }
        }
        // rely on older interface to retrieve tool information
        if (CurrentTool) {
            CurrentTool->MarkerPosition.Position() = ToolData.GetFrame();
            CurrentTool->MarkerPosition.SetValid(true);
        }
        // Get tool tip calibration if it is invalid or has changed
        if ((strcmp(ToolData.GetName(), ProbeCal.GetName()) != 0) || !ProbeCal.Valid()) {


            std::cout << "Got probe cal " << ToolData.GetName() << std::endl;
            std::cout << "   probe cal " << ProbeCal.GetName() << std::endl;
            std::cout << "   tip " << ProbeCal.GetTip() << std::endl;
            std::cout << "   hind " << ProbeCal.GetHind() << std::endl;
            std::cout << "   valid " << ProbeCal.Valid() << std::endl;

        }else
        {
            std::cout << "did not get got probe cal " << ToolData.GetName() << std::endl;
        }
        // If we have valid data, then store the result
        if (CurrentTool && ProbeCal.Valid() &&
            (strcmp(ToolData.GetName(), ProbeCal.GetName()) == 0)) {
            CurrentTool->TooltipPosition.Position() = vctFrm3(ToolData.GetFrame().Rotation(),
                                                              ToolData.GetFrame() * ProbeCal.GetTip());
            CurrentTool->TooltipPosition.SetValid(true);
        }else
        {
            if(!CurrentTool)
                std::cout << "CurrentTool not valid" << ProbeCal.Valid() << std::endl;
            if(!ProbeCal.Valid())
                std::cout << "ProbeCal not valid" << ProbeCal.Valid() << std::endl;
            if(!(strcmp(ToolData.GetName(), ProbeCal.GetName()) == 0))
                std::cout << ToolData.GetName() << " does not match " << ProbeCal.GetName() << std::endl;

        }
    }


}
void mtsMedtronicStealthlink::operator()(const MNavStealthLink::Frame& frame_in){
    FrameData = frame_in;

    // update frame interface data
    if (FrameData.Valid()) {
        if (!CurrentFrame || (CurrentFrame->GetStealthName() != FrameData.GetName())) {
            CurrentFrame = FindTool(FrameData.GetName());
            if (!CurrentFrame) {
                CMN_LOG_CLASS_INIT_VERBOSE << "Run: adding new tool \""
                                           << FrameData.GetName() << "\"" << std::endl;
                CurrentFrame = AddTool(FrameData.GetName(), FrameData.GetName());
            }
            if (CurrentFrame) {
                CMN_LOG_CLASS_RUN_VERBOSE << "Run: current tool is now \""
                                          << CurrentFrame->GetInterfaceName() << "\"" << std::endl;
            } else {
                CMN_LOG_CLASS_RUN_ERROR << "Run: unable to add provided interface for new tool \""
                                        << FrameData.GetName() << "\"" << std::endl;
            }
        }
        // rely on older interface to retrieve tool information
        if (CurrentFrame) {
            CurrentFrame->MarkerPosition.Position() = FrameData.GetFrame();
            CurrentFrame->MarkerPosition.SetValid(true);
        }
    }

}
*/



void mtsMedtronicStealthlink::Tool::Assign(const MNavStealthLink::DataItem & item_in){

    if (typeid(item_in) == typeid(const MNavStealthLink::Instrument &) ){
        const MNavStealthLink::Instrument & tool_in = dynamic_cast<const MNavStealthLink::Instrument & >(item_in);

        frameConversion(this->MarkerPosition.Position(),tool_in.localizer_T_instrument);
        this->MarkerPosition.SetValid(true);

        this->TooltipPosition.SetValid(tool_in.isCalibrated);

        if (tool_in.isCalibrated){

        this->TooltipPosition.Position() = vctFrm3(this->MarkerPosition.Position().Rotation(),
                                                          vct3(tool_in.localizerPosition.tip.x,tool_in.localizerPosition.tip.y,tool_in.localizerPosition.tip.y));

        }


    }else if(typeid(item_in) == typeid(const MNavStealthLink::Frame &)){
        const MNavStealthLink::Frame & frame_in = dynamic_cast<const MNavStealthLink::Frame & >(item_in);
        frameConversion(this->MarkerPosition.Position(),frame_in.frame_T_localizer);
        this->MarkerPosition.SetValid(true);

    }


}




void mtsMedtronicStealthlink::Registration::Assign(const MNavStealthLink::DataItem & item_in){

    const MNavStealthLink::Registration& registration_in = dynamic_cast<const MNavStealthLink::Registration&>(item_in);

    frameConversion(this->Transformation, registration_in.regExamMM_T_frame);
    this->Valid = true;
    this->PredictedAccuracy = registration_in.predictedAccuracy;
    this->PredictedAccuracy.SetValid(this->Valid);

}


/*
void mtsMedtronicStealthlink::operator()(const MNavStealthLink::Exam& exam_in){
    ExamInformationMember.VoxelScale[0] = exam_in.scale[0];
    ExamInformationMember.VoxelScale[1] = exam_in.scale[1];
    ExamInformationMember.VoxelScale[2] = exam_in.scale[2];
    ExamInformationMember.Size[0] = exam_in.size[0];
    ExamInformationMember.Size[1] = exam_in.size[1];
    ExamInformationMember.Size[2] = exam_in.size[2];
    ExamInformationMember.Valid = true;

}
void mtsMedtronicStealthlink::operator()(MNavStealthLink::SurgicalPlan& plan_in){
    SurgicalPlan[0] = plan_in.entry[0];
    SurgicalPlan[1] = plan_in.entry[1];
    SurgicalPlan[2] = plan_in.entry[2];
    SurgicalPlan[3] = plan_in.target[0];
    SurgicalPlan[4] = plan_in.target[1];
    SurgicalPlan[5] = plan_in.target[2];

}*/



bool mtsMedtronicStealthlink::less_DataItem::operator() (const MNavStealthLink::DataItem * a, const MNavStealthLink::DataItem * b) const {
    //std::cout << typeid(*a).name() << std::endl;
    //std::cout << typeid(*b).name() << std::endl;
    //std::cout << typeid(const MNavStealthLink::Instrument).name() << std::endl;
    //std::cout << typeid(const MNavStealthLink::Frame).name() << std::endl;
        if (typeid(*a) == typeid(const MNavStealthLink::Instrument) && typeid(*b) == typeid(const MNavStealthLink::Instrument)){
            return strcmp(dynamic_cast<const MNavStealthLink::Instrument *>(a)->name.c_str(),dynamic_cast<const MNavStealthLink::Instrument *>(b)->name.c_str()) < 0;
        }else if(typeid(*a) == typeid(const MNavStealthLink::Frame) && typeid(*b) == typeid(const MNavStealthLink::Frame)){
            return strcmp(dynamic_cast<const MNavStealthLink::Frame *>(a)->name.c_str(),dynamic_cast<const MNavStealthLink::Frame *>(b)->name.c_str()) < 0;
        }else{
            return strcmp(typeid(*a).name(),typeid(*b).name()) < 0;
        }
    }

void  mtsMedtronicStealthlink::SurgicalPlan::Assign(const MNavStealthLink::DataItem & item_in){
   const MNavStealthLink::SurgicalPlan& plan_in = dynamic_cast<const MNavStealthLink::SurgicalPlan&>(item_in);
   MNavStealthLink::Point & entry = const_cast<MNavStealthLink::Point &>(plan_in.entry);
   MNavStealthLink::Point & target = const_cast<MNavStealthLink::Point &>(plan_in.target);
   this->entry[0] = entry[0];
   this->entry[1] = entry[1];
   this->entry[2] = entry[2];
   this->target[3] = target[0];
   this->target[4] = target[1];
   this->target[5] = target[2];
}


void mtsMedtronicStealthlink::ExamInformation::Assign(const MNavStealthLink::DataItem & item_in){
   const MNavStealthLink::Exam& exam_in = dynamic_cast<const MNavStealthLink::Exam&>(item_in);
   this->VoxelScale[0] = exam_in.scale[0];
   this->VoxelScale[1] = exam_in.scale[1];
   this->VoxelScale[2] = exam_in.scale[2];
   this->Size[0] = exam_in.size[0];
   this->Size[1] = exam_in.size[1];
   this->Size[2] = exam_in.size[2];
   this->Valid = true;

}


/*void mtsMedtronicStealthlink::GetTool(mtsStealthTool & tool) const{
    myDataMapIteratorsContainer::iterator it, newestIt;
    mtsStealthTool currentTool;
    double currentNewest = -1;
    double currentTime = 0;
    mtsStealthTool * currentToolptr = 0;
    const myDataMapIteratorsContainer::iterator end = myToolIterators.end();
    for (it = myToolIterators.begin(); it != end; it++) {
        currentToolptr = dynamic_cast<mtsStealthTool * >((*it)->second);
        myStateTableMap[(*it)->second]->GetAccessor(std::string(currentToolptr->GetName())+"Data")->GetLatest(currentTool);
        currentTool.GetTimestamp(currentTime);
        if(currentTime >= currentNewest){
            currentNewest = currentTime;
            newestIt = it;
        }
    }

    currentToolptr = dynamic_cast<mtsStealthTool * >((*newestIt)->second);
    myStateTableMap[(*newestIt)->second]->GetAccessor(std::string(currentToolptr->GetName())+"Data")->GetLatest(tool);*/

   /* mtsStealthTool * currentToolptr = dynamic_cast<mtsStealthTool * >((*(myToolIterators.begin()))->second);

    tool = *currentToolptr;
}*/
/*void mtsMedtronicStealthlink::GetFrame(mtsStealthFrame & frame) const{
    myDataMapIteratorsContainer::iterator it, newestIt;
    mtsStealthFrame currentFrame;
    double currentNewest = -1;
    double currentTime = 0;
    mtsStealthFrame * currentFrameptr = 0;
    const myDataMapIteratorsContainer::iterator end = myFrameIterators.end();
    for (it = myFrameIterators.begin(); it != end; it++) {
        currentFrameptr = dynamic_cast<mtsStealthFrame * >((*it)->second);
        myStateTableMap[(*it)->second]->GetAccessor(std::string(currentFrameptr->GetName())+"Data")->GetLatest(currentFrame);
        currentFrame.GetTimestamp(currentTime);
        if (currentTime >= currentNewest){
            currentNewest = currentTime;
            newestIt = it;
        }
    }

    currentFrameptr = dynamic_cast<mtsStealthFrame * >((*it)->second);
    myStateTableMap[(*newestIt)->second]->GetAccessor(std::string(currentFrameptr->GetName())+"Data")->GetLatest(frame);*/

   /* mtsStealthFrame * currentFrameptr = dynamic_cast<mtsStealthFrame * >((*(myFrameIterators.begin()))->second);

    frame = *currentFrameptr;
}*/


void mtsMedtronicStealthlink::SurgicalPlan::ConfigureInterfaceProvided(mtsInterfaceProvided * provided_in)
{
    if (provided_in) {
        provided_in->AddCommandReadState(this->myStateTable, this->entry, "GetEntry");
        provided_in->AddCommandReadState(this->myStateTable, this->target, "GetTarget");
    }
}


void mtsMedtronicStealthlink::Registration::ConfigureInterfaceProvided(mtsInterfaceProvided * provided_in)
{
    if (provided_in) {
            provided_in->AddCommandReadState(this->myStateTable, this->Transformation, "GetTransformation");
            provided_in->AddCommandReadState(this->myStateTable, this->Valid, "GetValid");
            provided_in->AddCommandReadState(this->myStateTable, this->PredictedAccuracy, "GetPredictedAccuracy");
    }
}

void mtsMedtronicStealthlink::ExamInformation::ConfigureInterfaceProvided(mtsInterfaceProvided * provided_in)
{
    if (provided_in) {
            provided_in->AddCommandReadState(this->myStateTable, this->VoxelScale, "GetVoxelScale");
            provided_in->AddCommandReadState(this->myStateTable, this->Size, "GetSize");
            provided_in->AddCommandReadState(this->myStateTable, this->Valid, "GetValid");
    }
}

void mtsMedtronicStealthlink::Tool::ConfigureInterfaceProvided(mtsInterfaceProvided * provided_in)
{
    if (provided_in) {
        provided_in->AddCommandReadState(this->myStateTable, this->TooltipPosition, "GetPositionCartesian");
        provided_in->AddCommandReadState(this->myStateTable, this->MarkerPosition, "GetMarkerCartesian");
    }
}


