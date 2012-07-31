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
    // create Stealthlink objects
#ifdef sawMedtronicStealthlink_IS_SIMULATOR
    this->Client = 0;
#else
    //this->Client = new AsCL_Client;
#endif

    //this->Utils = new mtsMedtronicStealthlink_AsCL_Utils;

    SurgicalPlan.SetSize(6);

    // Stealth Tool -- the position of the tracked tool, as a frame
    ToolDataStateTable.AddData(ToolData, "ToolData");
    // Stealth Frame -- the position of the base frame, as a frame
    FrameDataStateTable.AddData(FrameData, "FrameData");
    // Stealth Registration
    RegistrationDataStateTable.AddData(RegistrationData, "RegistrationData");
    // Stealth Tool Calibration
    ProbeCalStateTable.AddData(ProbeCal, "ProbeCalibration");

    mtsInterfaceProvided * provided = AddInterfaceProvided("Controller");
    if (provided) {
        provided->AddCommandReadState(ToolDataStateTable, ToolData, "GetTool");
        provided->AddCommandReadState(FrameDataStateTable, FrameData, "GetFrame");
        provided->AddCommandReadState(RegistrationDataStateTable, RegistrationData, "GetRegistration");
        provided->AddCommandReadState(ProbeCalStateTable, ProbeCal, "GetProbeCalibration");
        provided->AddCommandVoid(&mtsMedtronicStealthlink::RequestSurgicalPlan, this, "RequestSurgicalPlan");
        provided->AddCommandRead(&mtsMedtronicStealthlink::GetSurgicalPlan, this, "GetSurgicalPlan", SurgicalPlan);
    }

    // Add interface for registration, ideally we should standardize such interface commands/payloads
    // maybe we should create a separate state table for registration?  Would only advance if changed.
    RegistrationStateTable.AddData(RegistrationMember.Transformation, "RegistrationTransformation");
    RegistrationStateTable.AddData(RegistrationMember.Valid, "RegistrationValid");
    RegistrationStateTable.AddData(RegistrationMember.PredictedAccuracy, "RegistrationPredictedAccuracy");
    provided = AddInterfaceProvided("Registration");
    if (provided) {
        provided->AddCommandReadState(RegistrationStateTable, RegistrationMember.Transformation, "GetTransformation");
        provided->AddCommandReadState(RegistrationStateTable, RegistrationMember.Valid, "GetValid");
        provided->AddCommandReadState(RegistrationStateTable, RegistrationMember.PredictedAccuracy, "GetPredictedAccuracy");
    }

    // Add interface for exam information
    ExamInformationStateTable.AddData(ExamInformationMember.VoxelScale, "ExamInformationVoxelScale");
    ExamInformationStateTable.AddData(ExamInformationMember.Size, "ExamInformationSize");
    ExamInformationStateTable.AddData(ExamInformationMember.Valid, "ExamInformationValid");
    provided = AddInterfaceProvided("ExamInformation");
    if (provided) {
        provided->AddCommandVoid(&mtsMedtronicStealthlink::RequestExamInformation, this, "RequestExamInformation");
        provided->AddCommandReadState(ExamInformationStateTable, ExamInformationMember.VoxelScale, "GetVoxelScale");
        provided->AddCommandReadState(ExamInformationStateTable, ExamInformationMember.Size, "GetSize");
        provided->AddCommandReadState(ExamInformationStateTable, ExamInformationMember.Valid, "GetValid");
    }
}

mtsMedtronicStealthlink::mtsMedtronicStealthlink(const std::string & taskName) :
    mtsTaskFromSignal(taskName),
    StealthlinkPresent(false),
    CurrentTool(0),
    CurrentFrame(0)
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
    }



    //attach all of the callbacks


    instrumentSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Instrument>(*(this->StealthServerProxy));
    frameSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Frame>(*(this->StealthServerProxy));
    registrationSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Registration>(*(this->StealthServerProxy));
    examSubscription = new MNavStealthLink::Subscription<MNavStealthLink::Exam>(*(this->StealthServerProxy));
    surgicalPlanSubscription = new MNavStealthLink::Subscription<MNavStealthLink::SurgicalPlan>(*(this->StealthServerProxy));


    instrumentSubscription->start(*this);
    frameSubscription->start(*this);
    registrationSubscription->start(*this);
    examSubscription->start(*this);
    surgicalPlanSubscription->start(*this);

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
    tool = new Tool(stealthName, interfaceName);
    Tools.push_back(tool);
    CMN_LOG_CLASS_RUN_VERBOSE << "AddTool: adding " << stealthName << " to interface " << interfaceName << std::endl;
    provided = AddInterfaceProvided(interfaceName);
    if (provided) {
        StateTable.AddData(tool->TooltipPosition, interfaceName + "Position");
        provided->AddCommandReadState(StateTable, tool->TooltipPosition, "GetPositionCartesian");
        StateTable.AddData(tool->MarkerPosition, interfaceName + "Marker");
        provided->AddCommandReadState(StateTable, tool->MarkerPosition, "GetMarkerCartesian");
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
    plan = SurgicalPlan;
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

    if (this->StealthServerProxy) {
        delete this->StealthServerProxy;
        this->StealthServerProxy = 0;
    }

    if (this->Client) {
        delete this->Client;
        this->Client = 0;
    }
    if (this->Utils) {
        delete this->Utils;
        this->Utils = 0;
    }
    CMN_LOG_CLASS_RUN_VERBOSE << "Cleanup: finished" << std::endl;
}



void mtsMedtronicStealthlink::operator()(MNavStealthLink::Instrument& instrument_in){
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

    ToolDataStateTable.Advance();
    ProbeCalStateTable.Advance();
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

    FrameDataStateTable.Advance();
}
void mtsMedtronicStealthlink::operator()(const MNavStealthLink::Registration& registration_in){
    RegistrationData = registration_in;

    // update registration interface data
    this->RegistrationMember.Transformation = RegistrationData.GetFrame();
    this->RegistrationMember.Valid = RegistrationData.Valid();
    this->RegistrationMember.PredictedAccuracy = RegistrationData.GetAccuracy();
    this->RegistrationMember.PredictedAccuracy.SetValid(RegistrationData.Valid());

    RegistrationDataStateTable.Advance();
}
void mtsMedtronicStealthlink::operator()(const MNavStealthLink::Exam& exam_in){
    ExamInformationMember.VoxelScale[0] = exam_in.scale[0];
    ExamInformationMember.VoxelScale[1] = exam_in.scale[1];
    ExamInformationMember.VoxelScale[2] = exam_in.scale[2];
    ExamInformationMember.Size[0] = exam_in.size[0];
    ExamInformationMember.Size[1] = exam_in.size[1];
    ExamInformationMember.Size[2] = exam_in.size[2];
    ExamInformationMember.Valid = true;

    ExamInformationStateTable.Advance();
}
void mtsMedtronicStealthlink::operator()(MNavStealthLink::SurgicalPlan& plan_in){
    SurgicalPlan[0] = plan_in.entry[0];
    SurgicalPlan[1] = plan_in.entry[1];
    SurgicalPlan[2] = plan_in.entry[2];
    SurgicalPlan[3] = plan_in.target[0];
    SurgicalPlan[4] = plan_in.target[1];
    SurgicalPlan[5] = plan_in.target[2];

    SurgicalPlanStateTable.Advance();
}



