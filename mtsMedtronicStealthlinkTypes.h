/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$
  Author(s): Peter Kazanzides, Anton Deguet
  Created on: 2007

  (C) Copyright 2007-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsMedtronicStealthlinkTypes_h
#define _mtsMedtronicStealthlinkTypes_h

#include <cisstMultiTask/mtsGenericObject.h>
// Add support for prmPositionCartesianGet
#include <cisstParameterTypes/prmPositionCartesianGet.h>

// Always include last
#include <sawMedtronicStealthlink/sawMedtronicStealthlinkExport.h>

#ifndef NAME_LENGTH
#define NAME_LENGTH 64  // from GRI.h
#endif

// Forward declarations of Stealthlink types
struct tool;
struct frame;
struct registration;
struct probe_calibration;

class CISST_EXPORT mtsStealthTool: public mtsGenericObject {
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
 private:
    vctFrm3 XForm;
    double GeometryError;
    char Name[NAME_LENGTH];
 public:
    mtsStealthTool():mtsGenericObject(), XForm(), GeometryError(0) {Name[0]='\0';}
    ~mtsStealthTool() {}
    const vctFrm3 & GetFrame(void) const { return XForm; }
    const char * GetName(void) const { return Name; }
    double GetGeometryError(void) const { return GeometryError; }

    void Assign(const mtsStealthTool & that);
    mtsStealthTool & operator= (const mtsStealthTool & that) { this->Assign(that); return *this; }
    void Assign(const struct tool & griTool);
    mtsStealthTool & operator= (const tool & griTool) { this->Assign(griTool); return *this; }

    void Assign(const prmPositionCartesianGet & that);
    mtsStealthTool & operator= (const prmPositionCartesianGet & that) {this->Assign(that); return *this;}

    std::string ToString(void) const;
    void ToStream(std::ostream & outputStream) const;
    void ToStreamRaw(std::ostream & outputStream, const char delimiter = ' ',
                     bool headerOnly = false, const std::string & headerPrefix = "") const;
    void SerializeRaw(std::ostream & outputStream) const;
    void DeSerializeRaw(std::istream & inputStream);
};
CMN_DECLARE_SERVICES_INSTANTIATION(mtsStealthTool);


class CISST_EXPORT mtsStealthFrame : public mtsGenericObject {
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
 private:
    vctFrm3 XForm;
    double GeometryError;
    char Name[NAME_LENGTH];
 public:
    mtsStealthFrame():XForm(), GeometryError(0) { Name[0]='\0';};
    ~mtsStealthFrame() {};
    const vctFrm3 & GetFrame(void) const { return XForm; }
    double GetGeometryError(void) const { return GeometryError; }
    const char * GetName(void) const { return Name; }
    void Assign(const mtsStealthFrame & that);
    mtsStealthFrame & operator= (const mtsStealthFrame & that) { this->Assign(that); return *this; }
    void Assign(const struct frame & griFrame);
    mtsStealthFrame & operator= (const frame & griFrame) { this->Assign(griFrame); return *this; }

    void Assign(const prmPositionCartesianGet & that);
    mtsStealthFrame & operator= (const prmPositionCartesianGet & that) {this->Assign(that); return *this;}

    std::string ToString(void) const;
    void ToStream(std::ostream & outputStream) const;
    void ToStreamRaw(std::ostream & outputStream, const char delimiter = ' ',
                     bool headerOnly = false, const std::string & headerPrefix = "") const;
};
CMN_DECLARE_SERVICES_INSTANTIATION(mtsStealthFrame);


class CISST_EXPORT mtsStealthRegistration : public mtsGenericObject {
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
 private:
    vctFrm3 XForm;
    double predictedAccuracy;
 public:
    mtsStealthRegistration():XForm(), predictedAccuracy(0) {};
    ~mtsStealthRegistration() {};
    const vctFrm3 & GetFrame(void) const { return XForm; }
    double GetAccuracy(void) const { return predictedAccuracy; }
    void Assign(const mtsStealthRegistration & that);
    mtsStealthRegistration & operator= (const mtsStealthRegistration & that) { this->Assign(that); return *this; }
    void Assign(const struct registration & griRegistration);
    mtsStealthRegistration & operator= (const registration & griRegistration) { this->Assign(griRegistration); return *this; }

    void Assign(const vctFrm3 & tmpFrm, const double & tmpAccuracy, const bool & tmpValid);

    std::string ToString(void) const;
    void ToStream(std::ostream & outputStream) const;
    void ToStreamRaw(std::ostream & outputStream, const char delimiter = ' ',
                     bool headerOnly = false, const std::string & headerPrefix = "") const;
};
CMN_DECLARE_SERVICES_INSTANTIATION(mtsStealthRegistration);


class CISST_EXPORT mtsStealthProbeCal : public mtsGenericObject {
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
 private:
    char Name[NAME_LENGTH];
    vct3 Tip;
    vct3 Hind;
 public:

    mtsStealthProbeCal() { Name[0]='\0'; };
    ~mtsStealthProbeCal() {};
    const char * GetName(void) const { return Name; }
    const vct3 & GetTip(void) const { return Tip; }
    const vct3 & GetHind(void) const { return Hind; }
    void Assign(const mtsStealthProbeCal & that);
    mtsStealthProbeCal & operator= (const mtsStealthProbeCal & that) { this->Assign(that); return *this; }
    void Assign (const struct probe_calibration & griProbeCal);
    mtsStealthProbeCal & operator= (const struct probe_calibration & griProbeCal) { this->Assign(griProbeCal); return *this; }
    std::string ToString(void) const;
    void ToStream(std::ostream & outputStream) const;
    void ToStreamRaw(std::ostream & outputStream, const char delimiter = ' ',
                     bool headerOnly = false, const std::string & headerPrefix = "") const;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsStealthProbeCal);

#endif  // _mtsMedtronicStealthlinkTypes_h
