//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_MSGS_H
#define UBLOX_MSGS_H

#include <ubloxf9p_msgs/NavATT.h>
#include <ubloxf9p_msgs/NavCLOCK.h>
#include <ubloxf9p_msgs/NavDGPS.h>
#include <ubloxf9p_msgs/NavDOP.h>
#include <ubloxf9p_msgs/NavPOSECEF.h>
#include <ubloxf9p_msgs/NavPOSLLH.h>
#include <ubloxf9p_msgs/NavRELPOSNED.h>
#include <ubloxf9p_msgs/NavSBAS.h>
#include <ubloxf9p_msgs/NavSOL.h>
#include <ubloxf9p_msgs/NavPVT.h>
#include <ubloxf9p_msgs/NavPVT7.h>
#include <ubloxf9p_msgs/NavSTATUS.h>
#include <ubloxf9p_msgs/NavSAT.h>
#include <ubloxf9p_msgs/NavSVIN.h>
#include <ubloxf9p_msgs/NavSVINFO.h>
#include <ubloxf9p_msgs/NavTIMEGPS.h>
#include <ubloxf9p_msgs/NavTIMEUTC.h>
#include <ubloxf9p_msgs/NavVELECEF.h>
#include <ubloxf9p_msgs/NavVELNED.h>

#include <ubloxf9p_msgs/RxmALM.h>
#include <ubloxf9p_msgs/RxmEPH.h>
#include <ubloxf9p_msgs/RxmRAW.h>
#include <ubloxf9p_msgs/RxmRAW_SV.h>
#include <ubloxf9p_msgs/RxmRAWX.h>
#include <ubloxf9p_msgs/RxmRAWX_Meas.h>
#include <ubloxf9p_msgs/RxmRTCM.h>
#include <ubloxf9p_msgs/RxmSFRB.h>
#include <ubloxf9p_msgs/RxmSFRBX.h>
#include <ubloxf9p_msgs/RxmSVSI.h>

#include <ubloxf9p_msgs/Inf.h>
#include <ubloxf9p_msgs/Ack.h>

#include <ubloxf9p_msgs/CfgANT.h>
#include <ubloxf9p_msgs/CfgCFG.h>
#include <ubloxf9p_msgs/CfgDAT.h>
#include <ubloxf9p_msgs/CfgDGNSS.h>
#include <ubloxf9p_msgs/CfgGNSS.h>
#include <ubloxf9p_msgs/CfgHNR.h>
#include <ubloxf9p_msgs/CfgINF.h>
#include <ubloxf9p_msgs/CfgINF_Block.h>
#include <ubloxf9p_msgs/CfgMSG.h>
#include <ubloxf9p_msgs/CfgNAV5.h>
#include <ubloxf9p_msgs/CfgNAVX5.h>
#include <ubloxf9p_msgs/CfgNMEA.h>
#include <ubloxf9p_msgs/CfgNMEA6.h>
#include <ubloxf9p_msgs/CfgNMEA7.h>
#include <ubloxf9p_msgs/CfgPRT.h>
#include <ubloxf9p_msgs/CfgRATE.h>
#include <ubloxf9p_msgs/CfgRST.h>
#include <ubloxf9p_msgs/CfgSBAS.h>
#include <ubloxf9p_msgs/CfgTMODE3.h>
#include <ubloxf9p_msgs/CfgUSB.h>

#include <ubloxf9p_msgs/UpdSOS.h>
#include <ubloxf9p_msgs/UpdSOS_Ack.h>

#include <ubloxf9p_msgs/MonGNSS.h>
#include <ubloxf9p_msgs/MonHW.h>
#include <ubloxf9p_msgs/MonHW6.h>
#include <ubloxf9p_msgs/MonVER.h>

#include <ubloxf9p_msgs/AidALM.h>
#include <ubloxf9p_msgs/AidEPH.h>
#include <ubloxf9p_msgs/AidHUI.h>

#include <ubloxf9p_msgs/EsfINS.h>
#include <ubloxf9p_msgs/EsfMEAS.h>
#include <ubloxf9p_msgs/EsfRAW.h>
#include <ubloxf9p_msgs/EsfSTATUS.h>
#include <ubloxf9p_msgs/EsfSTATUS_Sens.h>

#include <ubloxf9p_msgs/MgaGAL.h>

#include <ubloxf9p_msgs/HnrPVT.h>

#include <ubloxf9p_msgs/TimTM2.h>

namespace ubloxf9p_msgs {

namespace Class {
  static const uint8_t NAV = 0x01; //!< Navigation Result Messages: Position, 
                                   //!< Speed, Time, Acceleration, Heading, 
                                   //!< DOP, SVs used
  static const uint8_t RXM = 0x02; //!< Receiver Manager Messages: 
                                   //!< Satellite Status, RTC Status
  static const uint8_t INF = 0x04; //!< Information Messages: 
                                   //!< Printf-Style Messages, with IDs such as
                                   //!< Error, Warning, Notice
  static const uint8_t ACK = 0x05; //!< Ack/Nack Messages: Acknowledge or Reject 
                                   //!< messages to CFG input messages
  static const uint8_t CFG = 0x06; //!< Configuration Input Messages: Set 
                                   //!< Dynamic Model, Set DOP Mask, Set Baud 
                                   //!< Rate, etc.
  static const uint8_t UPD = 0x09; //!< Firmware Update Messages: i.e. 
                                   //!< Memory/Flash erase/write, Reboot, Flash 
                                   //!< identification, etc.
                                   //!< Used to update the firmware and identify 
                                   //!< any attached flash device
  static const uint8_t MON = 0x0A; //!< Monitoring Messages: Communication 
                                   //!< Status, CPU Load, Stack Usage, 
                                   //!< Task Status
  static const uint8_t AID = 0x0B; //!< AssistNow Aiding Messages: Ephemeris, 
                                   //!< Almanac, other A-GPS data input
  static const uint8_t TIM = 0x0D; //!< Timing Messages: Timepulse Output, 
                                   //!< Timemark Results
  static const uint8_t ESF = 0x10; //!< External Sensor Fusion Messages: 
                                   //!< External sensor measurements and status 
                                   //!< information
  static const uint8_t MGA = 0x13; //!< Multiple GNSS Assistance Messages: 
                                   //!< Assistance data for various GNSS
  static const uint8_t LOG = 0x21; //!< Logging Messages: Log creation, 
                                   //!< deletion, info and retrieval
  static const uint8_t SEC = 0x27; //!< Security Feature Messages
  static const uint8_t HNR = 0x28; //!< High Rate Navigation Results Messages: 
                                   //!< High rate time, position, speed, heading
  static const uint8_t RTCM = 0xF5; //!< RTCM Configuration Messages
}

namespace Message {
  namespace NAV {
    static const uint8_t ATT = NavATT::MESSAGE_ID;
    static const uint8_t CLOCK = NavCLOCK::MESSAGE_ID;
    static const uint8_t DGPS = NavDGPS::MESSAGE_ID;
    static const uint8_t DOP = NavDOP::MESSAGE_ID;
    static const uint8_t POSECEF = NavPOSECEF::MESSAGE_ID;
    static const uint8_t POSLLH = NavPOSLLH::MESSAGE_ID;
    static const uint8_t RELPOSNED = NavRELPOSNED::MESSAGE_ID;
    static const uint8_t SBAS = NavSBAS::MESSAGE_ID;
    static const uint8_t SOL = NavSOL::MESSAGE_ID;
    static const uint8_t PVT = NavPVT::MESSAGE_ID;
    static const uint8_t SAT = NavSAT::MESSAGE_ID;
    static const uint8_t STATUS = NavSTATUS::MESSAGE_ID;
    static const uint8_t SVINFO = NavSVINFO::MESSAGE_ID;
    static const uint8_t SVIN = NavSVIN::MESSAGE_ID;
    static const uint8_t TIMEGPS = NavTIMEGPS::MESSAGE_ID;
    static const uint8_t TIMEUTC = NavTIMEUTC::MESSAGE_ID;
    static const uint8_t VELECEF = NavVELECEF::MESSAGE_ID;
    static const uint8_t VELNED = NavVELNED::MESSAGE_ID;
  }

  namespace RXM {
    static const uint8_t ALM = RxmALM::MESSAGE_ID;
    static const uint8_t EPH = RxmEPH::MESSAGE_ID;
    static const uint8_t RAW = RxmRAW::MESSAGE_ID;
    static const uint8_t RAWX = RxmRAWX::MESSAGE_ID;
    static const uint8_t RTCM = RxmRTCM::MESSAGE_ID;
    static const uint8_t SFRB = RxmSFRB::MESSAGE_ID;
    static const uint8_t SFRBX = RxmSFRBX::MESSAGE_ID;
    static const uint8_t SVSI = RxmSVSI::MESSAGE_ID;
  }

  namespace INF {
    static const uint8_t ERROR = 0x00;
    static const uint8_t WARNING = 0x01;
    static const uint8_t NOTICE = 0x02;
    static const uint8_t TEST = 0x03;
    static const uint8_t DEBUG = 0x04;
  }

  namespace ACK {
    static const uint8_t NACK = 0x00; 
    static const uint8_t ACK = 0x01; 
  }

  namespace AID {
    static const uint8_t ALM = AidALM::MESSAGE_ID;
    static const uint8_t EPH = AidEPH::MESSAGE_ID;
    static const uint8_t HUI = AidHUI::MESSAGE_ID;
  }

  namespace CFG {
    static const uint8_t ANT = CfgANT::MESSAGE_ID;
    static const uint8_t CFG = CfgCFG::MESSAGE_ID;
    static const uint8_t DAT = CfgDAT::MESSAGE_ID;
    static const uint8_t GNSS = CfgGNSS::MESSAGE_ID;
    static const uint8_t HNR = CfgHNR::MESSAGE_ID;
    static const uint8_t INF = CfgINF::MESSAGE_ID;
    static const uint8_t DGNSS = CfgDGNSS::MESSAGE_ID;
    static const uint8_t MSG = CfgMSG::MESSAGE_ID;
    static const uint8_t NAV5 = CfgNAV5::MESSAGE_ID;
    static const uint8_t NAVX5 = CfgNAVX5::MESSAGE_ID;
    static const uint8_t NMEA = CfgNMEA::MESSAGE_ID;
    static const uint8_t PRT = CfgPRT::MESSAGE_ID;
    static const uint8_t RATE = CfgRATE::MESSAGE_ID;
    static const uint8_t RST = CfgRST::MESSAGE_ID;
    static const uint8_t SBAS = CfgSBAS::MESSAGE_ID;
    static const uint8_t TMODE3 = CfgTMODE3::MESSAGE_ID;
    static const uint8_t USB = CfgUSB::MESSAGE_ID;
  }

  namespace UPD {
    //! SOS and SOS_Ack have the same message ID, but different lengths
    static const uint8_t SOS = UpdSOS::MESSAGE_ID;
  }
  
  namespace MON {
    static const uint8_t GNSS = MonGNSS::MESSAGE_ID;
    static const uint8_t HW = MonHW::MESSAGE_ID;
    static const uint8_t VER = MonVER::MESSAGE_ID;
  }

  namespace ESF {
    static const uint8_t INS = EsfINS::MESSAGE_ID;
    static const uint8_t MEAS = EsfMEAS::MESSAGE_ID;
    static const uint8_t RAW = EsfRAW::MESSAGE_ID;
    static const uint8_t STATUS = EsfSTATUS::MESSAGE_ID;
  }

  namespace MGA {
    static const uint8_t GAL = MgaGAL::MESSAGE_ID;
  }

  namespace HNR {
    static const uint8_t PVT = HnrPVT::MESSAGE_ID;
  }

  namespace TIM {
    static const uint8_t TM2 = TimTM2::MESSAGE_ID;
  }
}

} //!< namespace ubloxf9p_msgs

#endif //!< UBLOX_MSGS_H
