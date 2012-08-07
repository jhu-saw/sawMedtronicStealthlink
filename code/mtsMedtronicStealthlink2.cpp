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

    //SurgicalPlan.SetSize(6);

    // Stealth Tool -- the position of the tracked tool, as a frame
    //StateTable.AddData(ToolData, "ToolData");
    // Stealth Frame -- the position of the base frame, as a frame
    //StateTable.AddData(FrameData, "FrameData");
    // Stealth Registration
    DataMapContainerInsertReturnValue dataMapInsertReturn;
    StateTableMapContainerInsertReturnValue stateTableMapInsertReturn;

    mtsInterfaceProvided * provided = AddInterfaceProvided("Controller");
    mtsStateTable * currentStateTable = 0;

    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(new MNavStealthLink::Registration(),new mtsStealthRegistration(this)));

    if (dataMapInsertReturn.second){
         stateTableMapInsertReturn = myStateTableMap.insert(StateTableMapContainerItem(dataMapInsertReturn.first->second,new mtsStateTable(256,"RegistrationDataTable")));
         if (stateTableMapInsertReturn.second){
             mtsStealthRegistration * stealthRegistrationInstance = dynamic_cast<mtsStealthRegistration *>(dataMapInsertReturn.first->second);
             currentStateTable = stateTableMapInsertReturn.first->second;
             currentStateTable->AddData(*stealthRegistrationInstance,"RegistrationData");
             if (provided) {
                 provided->AddCommandReadState(*(currentStateTable), *stealthRegistrationInstance, "GetRegistration");
             }
         }else{
            //something else went wrong
         }
    } else {
        //something went wrong
    }

    //StateTable.AddData(RegistrationData, "RegistrationData");
    // Stealth Tool Calibration
    //StateTable.AddData(ProbeCal, "ProbeCalibration");


    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(new MNavStealthLink::SurgicalPlan(),new my_mtsDoubleVec(6)));
    myPlanIterator = dataMapInsertReturn.first;
    if (dataMapInsertReturn.second){
        stateTableMapInsertReturn = myStateTableMap.insert(StateTableMapContainerItem(dataMapInsertReturn.first->second,new mtsStateTable(256,"SurgicalPlanTable")));
        if (stateTableMapInsertReturn.second){
            mtsDoubleVec * doubleVecInstance = dynamic_cast<mtsDoubleVec *>(dataMapInsertReturn.first->second);
            stateTableMapInsertReturn.first->second->AddData(*doubleVecInstance,"SurgicalPlan");
            if (provided) {
                provided->AddCommandReadState(*(stateTableMapInsertReturn.first->second), *doubleVecInstance, "GetSurgicalPlan");
            }
        }else{
            //something else went wrong
        }
    }else{
        //something went wrong
    }


    if (provided) {
        provided->AddCommandRead(&mtsMedtronicStealthlink::GetTool, this, "GetTool");
        provided->AddCommandRead(&mtsMedtronicStealthlink::GetFrame, this, "GetFrame");
        /*provided->AddCommandReadState(StateTable, RegistrationData, "GetRegistration");*/
        provided->AddCommandRead(&mtsMedtronicStealthlink::GetProbeCalibration, this, "GetProbeCalibration");
        provided->AddCommandVoid(&mtsMedtronicStealthlink::RequestSurgicalPlan, this, "RequestSurgicalPlan");
        /*provided->AddCommandRead(&mtsMedtronicStealthlink::GetSurgicalPlan, this, "GetSurgicalPlan", dynamic_cast<mtsDoubleVec &>(SurgicalPlan));*/
    }

    // Add interface for registration, ideally we should standardize such interface commands/payloads
    // maybe we should create a separate state table for registration?  Would only advance if changed.
    currentStateTable->AddData(RegistrationMember.Transformation, "RegistrationTransformation");
    currentStateTable->AddData(RegistrationMember.Valid, "RegistrationValid");
    currentStateTable->AddData(RegistrationMember.PredictedAccuracy, "RegistrationPredictedAccuracy");
    provided = AddInterfaceProvided("Registration");
    if (provided) {
        provided->AddCommandReadState(*currentStateTable, RegistrationMember.Transformation, "GetTransformation");
        provided->AddCommandReadState(*currentStateTable, RegistrationMember.Valid, "GetValid");
        provided->AddCommandReadState(*currentStateTable, RegistrationMember.PredictedAccuracy, "GetPredictedAccuracy");
    }



    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(new MNavStealthLink::Exam(),new ExamInformation()));

    if (dataMapInsertReturn.second){
         stateTableMapInsertReturn = myStateTableMap.insert(StateTableMapContainerItem(dataMapInsertReturn.first->second,new mtsStateTable(256,"ExamInformationTable")));
         if (stateTableMapInsertReturn.second){
             ExamInformation * examInformationInstance = dynamic_cast<ExamInformation *>(dataMapInsertReturn.first->second);
             currentStateTable = stateTableMapInsertReturn.first->second;
             // Add interface for exam information
             currentStateTable->AddData(examInformationInstance->VoxelScale, "ExamInformationVoxelScale");
             currentStateTable->AddData(examInformationInstance->Size, "ExamInformationSize");
             currentStateTable->AddData(examInformationInstance->Valid, "ExamInformationValid");
             provided = AddInterfaceProvided("ExamInformation");
             if (provided) {
                 provided->AddCommandVoid(&mtsMedtronicStealthlink::RequestExamInformation, this, "RequestExamInformation");
                 provided->AddCommandReadState(*currentStateTable, examInformationInstance->VoxelScale, "GetVoxelScale");
                 provided->AddCommandReadState(*currentStateTable, examInformationInstance->Size, "GetSize");
                 provided->AddCommandReadState(*currentStateTable, examInformationInstance->Valid, "GetValid");
             }
         }else{
            //something else went wrong
         }
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
    Cleanup();
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

    // add pre-defined tools (up to 100)
    for (unsigned int i = 0; i < 100; i++) {
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
        CMN_LOG_CLASS_RUN_WARNING << "Failed to connect to Stealth server application on host "
                         << this->StealthServerProxy->getHost() << ", port " << this->StealthServerProxy->getPort() << ": "
                         << StealthlinkError.reason() << std::endl;
        CMN_LOG_CLASS_RUN_WARNING << "Configure: could not Initialize StealthLink" << std::endl;
    }else{



    //attach all of the callbacks


    instrumentSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Instrument>(*(this->StealthServerProxy));
    frameSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Frame>(*(this->StealthServerProxy));
    registrationSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Registration>(*(this->StealthServerProxy));
    examSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Exam>(*(this->StealthServerProxy));
    surgicalPlanSubscription = new MNavStealthLink::Subscription<MNavStealthLink::SurgicalPlan>(*(this->StealthServerProxy));


    instrumentSubscription->start(this->myCallbackMember);
    frameSubscription->start(this->myCallbackMember);
    registrationSubscription->start(this->myCallbackMember);
    examSubscription->start(this->myCallbackMember);
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
    StateTableMapContainerInsertReturnValue stateTableMapInsertReturn;

    mtsStateTable * currentStateTable = 0;

    MNavStealthLink::DataItem * newItem = 0;
    myGenericObject * newObject = 0;

    if (strcmp(interfaceName.c_str(),"Frame") == 0){
        newItem = new MNavStealthLink::Frame();
        newObject = new mtsStealthFrame();
    } else if (strcmp(interfaceName.c_str(),"Tool") == 0 || strcmp(interfaceName.c_str(),"Tool") != 0){
        newItem = new MNavStealthLink::Instrument();
        newObject = new mtsStealthTool();
    }


    dataMapInsertReturn = myDataMap.insert(DataMapContainerItem(newItem,newObject));

    if (dataMapInsertReturn.second){
         stateTableMapInsertReturn = myStateTableMap.insert(StateTableMapContainerItem(dataMapInsertReturn.first->second,new mtsStateTable(256,interfaceName + "Table")));
         if (stateTableMapInsertReturn.second){
             currentStateTable = stateTableMapInsertReturn.first->second;
             if (strcmp(interfaceName.c_str(),"Frame") == 0){
                 mtsStealthFrame * currentFrame = dynamic_cast<mtsStealthFrame *>(dataMapInsertReturn.first->second);
                 myFrameIterators.push_back(dataMapInsertReturn.first);
                currentStateTable->AddData(*currentFrame,interfaceName + "Data");
             }else{

             /*if (typeid(dataMapInsertReturn.first->second) == typeid(mtsStealthTool *)){*/
                 mtsStealthTool * currentTool = dynamic_cast<mtsStealthTool *>(dataMapInsertReturn.first->second);
                 myToolIterators.push_back(dataMapInsertReturn.first);
                currentStateTable->AddData(*currentTool,interfaceName + "Data");
             }



         }else{
            //something else went wrong
         }
    } else {
        //something went wrong
    }



    CMN_LOG_CLASS_RUN_VERBOSE << "AddTool: adding " << stealthName << " to interface " << interfaceName << std::endl;
    provided = AddInterfaceProvided(interfaceName);
    if (provided) {
        currentStateTable->AddData(tool->TooltipPosition, interfaceName + "Position");
        provided->AddCommandReadState(*currentStateTable, tool->TooltipPosition, "GetPositionCartesian");
        currentStateTable->AddData(tool->MarkerPosition, interfaceName + "Marker");
        provided->AddCommandReadState(*currentStateTable, tool->MarkerPosition, "GetMarkerCartesian");
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


void mtsMedtronicStealthlink::GetSurgicalPlan(mtsDoubleVec & plan) const
{
    mtsDoubleVec * currentPlan = dynamic_cast<mtsDoubleVec *>(myPlanIterator->second);
    plan = *currentPlan;
}


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
    //Stop stealthlink
    this->StealthServerProxy->stop();

    //Wait for it's thread
    //StealthServerProxyThread.Wait();

    //If it timesout kill the thread
    StealthServerProxyThread.Delete();

    this->StealthServerProxy->disconnect();

    ToolsContainer::iterator it;
    const ToolsContainer::iterator end = Tools.end();
    for (it = Tools.begin(); it != end; it++) {
        delete (*it);
        *it = 0;
    }
    Tools.clear();


    DataMapContainer::iterator it2;
    const DataMapContainer::iterator end2 = myDataMap.end();
    for (it2 = myDataMap.begin(); it2 != end2; it2++) {
        delete (it2->first);
        delete (it2->second);
        it2->second = 0;
    }
    myDataMap.clear();
    myToolIterators.clear();
    myFrameIterators.clear();


    StateTableMapContainer::iterator it3;
    const StateTableMapContainer::iterator end3 = myStateTableMap.end();
    for (it3 = myStateTableMap.begin(); it3 != end3; it3++) {
        delete (it3->second);
        it3->second = 0;
    }
    myStateTableMap.clear();



    if (this->StealthServerProxy) {
        delete this->StealthServerProxy;
        this->StealthServerProxy = 0;
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "Cleanup: finished" << std::endl;
}


void mtsMedtronicStealthlink::myCallback::operator ()(const MNavStealthLink::DataItem& item_in){
    //the DataMap must be pre-allocated before a callback is called.
    DataMapContainer::iterator current_item = my_parent->myDataMap.find(&item_in);
    if (current_item != my_parent->myDataMap.end()) {
        *(current_item->second) = item_in;
        my_parent->myStateTableMap[current_item->second]->Advance();
    }else{
        //CMN_LOG_CLASS_RUN_WARNING << "Run: adding new data at runtime is currently unsupported"  << std::endl;
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
void mtsMedtronicStealthlink::operator()(const MNavStealthLink::Registration& registration_in){
    RegistrationData = registration_in;

    // update registration interface data
    this->RegistrationMember.Transformation = RegistrationData.GetFrame();
    this->RegistrationMember.Valid = RegistrationData.Valid();
    this->RegistrationMember.PredictedAccuracy = RegistrationData.GetAccuracy();
    this->RegistrationMember.PredictedAccuracy.SetValid(RegistrationData.Valid());

}
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
        if (typeid(a) == typeid(const MNavStealthLink::Instrument *) && typeid(b) == typeid(const MNavStealthLink::Instrument *)){
            return strcmp(dynamic_cast<const MNavStealthLink::Instrument *>(a)->name.c_str(),dynamic_cast<const MNavStealthLink::Instrument *>(b)->name.c_str()) == 0;
        }else if(typeid(a) == typeid(const MNavStealthLink::Frame *) && typeid(b) == typeid(const MNavStealthLink::Frame *)){
            return strcmp(dynamic_cast<const MNavStealthLink::Frame *>(a)->name.c_str(),dynamic_cast<const MNavStealthLink::Frame *>(b)->name.c_str()) == 0;
        }else{
            return typeid(a) == typeid(b);
        }
    }

myGenericObject * mtsMedtronicStealthlink::my_mtsDoubleVec::operator=(const MNavStealthLink::DataItem & item_in){
   const MNavStealthLink::SurgicalPlan& plan_in = dynamic_cast<const MNavStealthLink::SurgicalPlan&>(item_in);
   MNavStealthLink::Point & entry = const_cast<MNavStealthLink::Point &>(plan_in.entry);
   MNavStealthLink::Point & target = const_cast<MNavStealthLink::Point &>(plan_in.target);
   (*this)[0] = entry[0];
   (*this)[1] = entry[1];
   (*this)[2] = entry[2];
   (*this)[3] = target[0];
   (*this)[4] = target[1];
   (*this)[5] = target[2];

   return this;
}


myGenericObject * mtsMedtronicStealthlink::ExamInformation::operator =(const MNavStealthLink::DataItem & item_in){
   const MNavStealthLink::Exam& exam_in = dynamic_cast<const MNavStealthLink::Exam&>(item_in);
   this->VoxelScale[0] = exam_in.scale[0];
   this->VoxelScale[1] = exam_in.scale[1];
   this->VoxelScale[2] = exam_in.scale[2];
   this->Size[0] = exam_in.size[0];
   this->Size[1] = exam_in.size[1];
   this->Size[2] = exam_in.size[2];
   this->Valid = true;

   return this;
}


void mtsMedtronicStealthlink::GetTool(mtsStealthTool & tool) const{
   /* myDataMapIteratorsContainer::iterator it, newestIt;
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

    mtsStealthTool * currentToolptr = dynamic_cast<mtsStealthTool * >((*(myToolIterators.begin()))->second);

    tool = *currentToolptr;
}
void mtsMedtronicStealthlink::GetFrame(mtsStealthFrame & frame) const{
    /*myDataMapIteratorsContainer::iterator it, newestIt;
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

    mtsStealthFrame * currentFrameptr = dynamic_cast<mtsStealthFrame * >((*(myFrameIterators.begin()))->second);

    frame = *currentFrameptr;
}
