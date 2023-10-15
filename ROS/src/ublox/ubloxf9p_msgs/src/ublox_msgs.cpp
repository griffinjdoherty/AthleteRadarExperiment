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

#include <ubloxf9p/serialization/ublox_msgs.h>

template <typename T>
std::vector<std::pair<uint8_t,uint8_t> > ublox::Message<T>::keys_;

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::ATT, 
                      ubloxf9p_msgs, NavATT);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::CLOCK, 
                      ubloxf9p_msgs, NavCLOCK);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::DGPS, 
                      ubloxf9p_msgs, NavDGPS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::DOP, 
                      ubloxf9p_msgs, NavDOP);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::POSECEF, 
                      ubloxf9p_msgs, NavPOSECEF);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::POSLLH, 
                      ubloxf9p_msgs, NavPOSLLH);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, 
                      ubloxf9p_msgs::Message::NAV::RELPOSNED, 
                      ubloxf9p_msgs, 
                      NavRELPOSNED);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::SBAS, 
                      ubloxf9p_msgs, NavSBAS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::SOL, 
                      ubloxf9p_msgs, NavSOL);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::PVT, 
                      ubloxf9p_msgs, NavPVT);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::PVT, 
                      ubloxf9p_msgs, NavPVT7);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::SAT, 
                      ubloxf9p_msgs, NavSAT);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::STATUS, 
                      ubloxf9p_msgs, NavSTATUS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::SVIN, 
                      ubloxf9p_msgs, NavSVIN);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::SVINFO, 
                      ubloxf9p_msgs, NavSVINFO);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::TIMEGPS, 
                      ubloxf9p_msgs, NavTIMEGPS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::TIMEUTC, 
                      ubloxf9p_msgs, NavTIMEUTC);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::VELECEF, 
                      ubloxf9p_msgs, NavVELECEF);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::NAV, ubloxf9p_msgs::Message::NAV::VELNED, 
                      ubloxf9p_msgs, NavVELNED);

// ACK messages are declared differently because they both have the same 
// protocol, so only 1 ROS message is used
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::ACK, ubloxf9p_msgs::Message::ACK::NACK, 
                      ubloxf9p_msgs, Ack);
DECLARE_UBLOX_MESSAGE_ID(ubloxf9p_msgs::Class::ACK, ubloxf9p_msgs::Message::ACK::ACK, 
                      ubloxf9p_msgs, Ack, ACK);

// INF messages are declared differently because they all have the same 
// protocol, so only 1 ROS message is used. DECLARE_UBLOX_MESSAGE can only
// be called once, and DECLARE_UBLOX_MESSAGE_ID is called for the following
// messages
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::INF, ubloxf9p_msgs::Message::INF::ERROR, 
                      ubloxf9p_msgs, Inf);
DECLARE_UBLOX_MESSAGE_ID(ubloxf9p_msgs::Class::INF, 
                         ubloxf9p_msgs::Message::INF::WARNING, 
                         ubloxf9p_msgs, Inf, WARNING);
DECLARE_UBLOX_MESSAGE_ID(ubloxf9p_msgs::Class::INF, 
                         ubloxf9p_msgs::Message::INF::NOTICE, 
                         ubloxf9p_msgs, Inf, NOTICE);
DECLARE_UBLOX_MESSAGE_ID(ubloxf9p_msgs::Class::INF, 
                         ubloxf9p_msgs::Message::INF::TEST, 
                         ubloxf9p_msgs, Inf, TEST);
DECLARE_UBLOX_MESSAGE_ID(ubloxf9p_msgs::Class::INF, 
                         ubloxf9p_msgs::Message::INF::DEBUG, 
                         ubloxf9p_msgs, Inf, DEBUG);

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::ALM, 
                      ubloxf9p_msgs, RxmALM);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::EPH, 
                      ubloxf9p_msgs, RxmEPH);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::RAW, 
                      ubloxf9p_msgs, RxmRAW);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::RAWX, 
                      ubloxf9p_msgs, RxmRAWX);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::RTCM, 
                      ubloxf9p_msgs, RxmRTCM);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::SFRB, 
                      ubloxf9p_msgs, RxmSFRB);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::SFRBX, 
                      ubloxf9p_msgs, RxmSFRBX);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::RXM, ubloxf9p_msgs::Message::RXM::SVSI, 
                      ubloxf9p_msgs, RxmSVSI);

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::ANT, 
                      ubloxf9p_msgs, CfgANT);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::CFG, 
                      ubloxf9p_msgs, CfgCFG);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::DAT, 
                      ubloxf9p_msgs, CfgDAT);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::DGNSS, 
                      ubloxf9p_msgs, CfgDGNSS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::GNSS, 
                      ubloxf9p_msgs, CfgGNSS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::HNR,
                      ubloxf9p_msgs, CfgHNR);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::INF,
                      ubloxf9p_msgs, CfgINF);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::MSG, 
                      ubloxf9p_msgs, CfgMSG);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::NAV5, 
                      ubloxf9p_msgs, CfgNAV5);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::NAVX5, 
                      ubloxf9p_msgs, CfgNAVX5);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::NMEA, 
                      ubloxf9p_msgs, CfgNMEA);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::NMEA, 
                      ubloxf9p_msgs, CfgNMEA6);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::NMEA, 
                      ubloxf9p_msgs, CfgNMEA7);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::PRT, 
                      ubloxf9p_msgs, CfgPRT);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::RATE, 
                      ubloxf9p_msgs, CfgRATE);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::RST, 
                      ubloxf9p_msgs, CfgRST);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::TMODE3, 
                      ubloxf9p_msgs, CfgTMODE3);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::CFG, ubloxf9p_msgs::Message::CFG::USB, 
                      ubloxf9p_msgs, CfgUSB);

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::UPD, ubloxf9p_msgs::Message::UPD::SOS, 
                      ubloxf9p_msgs, UpdSOS);
// SOS and SOS_Ack have the same message ID, but different lengths
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::UPD, ubloxf9p_msgs::Message::UPD::SOS, 
                      ubloxf9p_msgs, UpdSOS_Ack);

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::MON, ubloxf9p_msgs::Message::MON::GNSS, 
                      ubloxf9p_msgs, MonGNSS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::MON, ubloxf9p_msgs::Message::MON::HW, 
                      ubloxf9p_msgs, MonHW);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::MON, ubloxf9p_msgs::Message::MON::HW, 
                      ubloxf9p_msgs, MonHW6);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::MON, ubloxf9p_msgs::Message::MON::VER, 
                      ubloxf9p_msgs, MonVER);

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::AID, ubloxf9p_msgs::Message::AID::ALM, 
                      ubloxf9p_msgs, AidALM);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::AID, ubloxf9p_msgs::Message::AID::EPH, 
                      ubloxf9p_msgs, AidEPH);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::AID, ubloxf9p_msgs::Message::AID::HUI, 
                      ubloxf9p_msgs, AidHUI);

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::ESF, ubloxf9p_msgs::Message::ESF::INS,
                      ubloxf9p_msgs, EsfINS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::ESF, ubloxf9p_msgs::Message::ESF::MEAS, 
                      ubloxf9p_msgs, EsfMEAS);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::ESF, ubloxf9p_msgs::Message::ESF::RAW, 
                      ubloxf9p_msgs, EsfRAW);
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::ESF, ubloxf9p_msgs::Message::ESF::STATUS, 
                      ubloxf9p_msgs, EsfSTATUS);


DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::MGA, ubloxf9p_msgs::Message::MGA::GAL, 
                      ubloxf9p_msgs, MgaGAL);

DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::HNR, ubloxf9p_msgs::Message::HNR::PVT, 
                      ubloxf9p_msgs, HnrPVT);

// TIM messages
DECLARE_UBLOX_MESSAGE(ubloxf9p_msgs::Class::TIM, ubloxf9p_msgs::Message::TIM::TM2,
		      ubloxf9p_msgs, TimTM2);

