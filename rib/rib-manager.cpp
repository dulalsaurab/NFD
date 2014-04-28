/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014  Regents of the University of California,
 *                     Arizona Board of Regents,
 *                     Colorado State University,
 *                     University Pierre & Marie Curie, Sorbonne University,
 *                     Washington University in St. Louis,
 *                     Beijing Institute of Technology,
 *                     The University of Memphis
 *
 * This file is part of NFD (Named Data Networking Forwarding Daemon).
 * See AUTHORS.md for complete list of NFD authors and contributors.
 *
 * NFD is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NFD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NFD, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 **/

#include "rib-manager.hpp"
#include "core/global-io.hpp"
#include "core/logger.hpp"

namespace nfd {
namespace rib {

NFD_LOG_INIT("RibManager");

const Name RibManager::COMMAND_PREFIX = "/localhost/nfd/rib";
const Name RibManager::REMOTE_COMMAND_PREFIX = "/localhop/nfd/rib";

const size_t RibManager::COMMAND_UNSIGNED_NCOMPS =
  RibManager::COMMAND_PREFIX.size() +
  1 + // verb
  1;  // verb options

const size_t RibManager::COMMAND_SIGNED_NCOMPS =
  RibManager::COMMAND_UNSIGNED_NCOMPS +
  4; // (timestamp, nonce, signed info tlv, signature tlv)

const RibManager::VerbAndProcessor RibManager::COMMAND_VERBS[] =
  {
    VerbAndProcessor(
                     Name::Component("register"),
                     &RibManager::registerEntry
                     ),

    VerbAndProcessor(
                     Name::Component("unregister"),
                     &RibManager::unregisterEntry
                     ),
  };

inline static void
NullDeleter(boost::asio::io_service* variable)
{
  // do nothing
}

RibManager::RibManager()
  : m_face(shared_ptr<boost::asio::io_service>(&getGlobalIoService(), &NullDeleter))
  , m_nfdController(new ndn::nfd::Controller(m_face))
  , m_validator(m_face)
  , m_faceMonitor(m_face)
  , m_verbDispatch(COMMAND_VERBS,
                   COMMAND_VERBS + (sizeof(COMMAND_VERBS) / sizeof(VerbAndProcessor)))
{
}

void
RibManager::registerWithNfd()
{
  //check whether the components of localhop and localhost prefixes are same
  BOOST_ASSERT(COMMAND_PREFIX.size() == REMOTE_COMMAND_PREFIX.size());

  NFD_LOG_INFO("Setting interest filter on: " << COMMAND_PREFIX);
  m_face.setController(m_nfdController);
  m_face.setInterestFilter(COMMAND_PREFIX,
                           bind(&RibManager::onRibRequest, this, _2),
                           bind(&RibManager::setInterestFilterFailed, this, _1, _2));

  NFD_LOG_INFO("Setting interest filter on: " << REMOTE_COMMAND_PREFIX);
  m_face.setInterestFilter(REMOTE_COMMAND_PREFIX,
                           bind(&RibManager::onRibRequest, this, _2),
                           bind(&RibManager::setInterestFilterFailed, this, _1, _2));

  NFD_LOG_INFO("Start monitoring face create/destroy events");
  m_faceMonitor.addSubscriber(boost::bind(&RibManager::onNotification, this, _1));
  m_faceMonitor.startNotifications();
}

void
RibManager::setConfigFile(ConfigFile& configFile)
{
  configFile.addSectionHandler("rib_security",
                               bind(&RibManager::onConfig, this, _1, _2, _3));
}

void
RibManager::onConfig(const ConfigSection& configSection,
                     bool isDryRun,
                     const std::string& filename)
{
  /// \todo remove check after validator-conf replaces settings on each load
  if (!isDryRun)
    m_validator.load(configSection, filename);
}

void
RibManager::setInterestFilterFailed(const Name& name, const std::string& msg)
{
  NFD_LOG_ERROR("Error in setting interest filter (" << name << "): " << msg);
  m_face.shutdown();
}

void
RibManager::sendResponse(const Name& name,
                         const ControlResponse& response)
{
  const Block& encodedControl = response.wireEncode();

  Data responseData(name);
  responseData.setContent(encodedControl);

  m_keyChain.sign(responseData);
  m_face.put(responseData);
}

void
RibManager::sendResponse(const Name& name,
                         uint32_t code,
                         const std::string& text)
{
  ControlResponse response(code, text);
  sendResponse(name, response);
}

void
RibManager::onRibRequest(const Interest& request)
{
  m_validator.validate(request,
                       bind(&RibManager::onCommandValidated, this, _1),
                       bind(&RibManager::onCommandValidationFailed, this, _1, _2));
}

void
RibManager::onCommandValidated(const shared_ptr<const Interest>& request)
{
  // REMOTE_COMMAND_PREFIX number of componenets are same as
  // NRD_COMMAND_PREFIX's so no extra checks are required.

  const Name& command = request->getName();
  const Name::Component& verb = command[COMMAND_PREFIX.size()];
  const Name::Component& parameterComponent = command[COMMAND_PREFIX.size() + 1];

  VerbDispatchTable::const_iterator verbProcessor = m_verbDispatch.find(verb);
  if (verbProcessor != m_verbDispatch.end())
    {
      ControlParameters parameters;
      if (!extractParameters(parameterComponent, parameters))
        {
          NFD_LOG_DEBUG("command result: malformed verb: " << verb);
          sendResponse(command, 400, "Malformed command");
          return;
        }

      if (!parameters.hasFaceId() || parameters.getFaceId() == 0)
        {
          parameters.setFaceId(request->getIncomingFaceId());
        }

      NFD_LOG_DEBUG("command result: processing verb: " << verb);
      (verbProcessor->second)(this, request, parameters);
    }
  else
    {
      NFD_LOG_DEBUG("Unsupported command: " << verb);
      sendResponse(request->getName(), 501, "Unsupported command");
    }
}

void
RibManager::registerEntry(const shared_ptr<const Interest>& request,
                          ControlParameters& parameters)
{
  ndn::nfd::RibRegisterCommand command;

  if (!validateParameters(command, parameters))
    {
      NFD_LOG_DEBUG("register result: FAIL reason: malformed");
      sendResponse(request->getName(), 400, "Malformed command");
      return;
    }

  RibEntry ribEntry;
  ribEntry.name = parameters.getName();
  ribEntry.faceId = parameters.getFaceId();
  ribEntry.origin = parameters.getOrigin();
  ribEntry.cost = parameters.getCost();
  ribEntry.flags = parameters.getFlags();
  ribEntry.expires = time::steady_clock::now() + parameters.getExpirationPeriod();

  NFD_LOG_TRACE("register prefix: " << ribEntry);

  // For right now, just pass the options to fib as it is,
  // without processing flags. Later options will be first added to
  // Rib tree, then nrd will generate fib updates based on flags and then
  // will add next hops one by one..
  m_managedRib.insert(ribEntry);
  m_nfdController->start<ndn::nfd::FibAddNextHopCommand>(
    ControlParameters()
      .setName(ribEntry.name)
      .setFaceId(ribEntry.faceId)
      .setCost(ribEntry.cost),
    bind(&RibManager::onRegSuccess, this, request, parameters, ribEntry),
    bind(&RibManager::onCommandError, this, _1, _2, request, ribEntry));
}

void
RibManager::unregisterEntry(const shared_ptr<const Interest>& request,
                            ControlParameters& parameters)
{
  ndn::nfd::RibUnregisterCommand command;

  if (!validateParameters(command, parameters))
    {
      NFD_LOG_DEBUG("register result: FAIL reason: malformed");
      sendResponse(request->getName(), 400, "Malformed command");
      return;
    }

  RibEntry ribEntry;
  ribEntry.name = parameters.getName();
  ribEntry.faceId = parameters.getFaceId();
  ribEntry.origin = parameters.getOrigin();

  NFD_LOG_TRACE("unregister prefix: " << ribEntry);

  m_nfdController->start<ndn::nfd::FibRemoveNextHopCommand>(
    ControlParameters()
      .setName(ribEntry.name)
      .setFaceId(ribEntry.faceId),
    bind(&RibManager::onUnRegSuccess, this, request, parameters, ribEntry),
    bind(&RibManager::onCommandError, this, _1, _2, request, ribEntry));
}

void
RibManager::onCommandValidationFailed(const shared_ptr<const Interest>& request,
                                      const std::string& failureInfo)
{
  NFD_LOG_DEBUG("RibRequestValidationFailed: " << failureInfo);
  sendResponse(request->getName(), 403, failureInfo);
}


bool
RibManager::extractParameters(const Name::Component& parameterComponent,
                              ControlParameters& extractedParameters)
{
  try
    {
      Block rawParameters = parameterComponent.blockFromValue();
      extractedParameters.wireDecode(rawParameters);
    }
  catch (const ndn::Tlv::Error& e)
    {
      return false;
    }

  NFD_LOG_DEBUG("Parameters parsed OK");
  return true;
}

bool
RibManager::validateParameters(const ControlCommand& command,
                               ControlParameters& parameters)
{
  try
    {
      command.validateRequest(parameters);
    }
  catch (const ControlCommand::ArgumentError&)
    {
      return false;
    }

  command.applyDefaultsToRequest(parameters);

  return true;
}

void
RibManager::onCommandError(uint32_t code, const std::string& error,
                           const shared_ptr<const Interest>& request,
                           const RibEntry& ribEntry)
{
  NFD_LOG_ERROR("NFD returned an error: " << error << " (code: " << code << ")");

  ControlResponse response;

  if (code == 404)
    {
      response.setCode(code);
      response.setText(error);
    }
  else
    {
      response.setCode(533);
      std::ostringstream os;
      os << "Failure to update NFD " << "(NFD Error: " << code << " " << error << ")";
      response.setText(os.str());
    }

  sendResponse(request->getName(), response);
  m_managedRib.erase(ribEntry);
}

void
RibManager::onRegSuccess(const shared_ptr<const Interest>& request,
                         const ControlParameters& parameters,
                         const RibEntry& ribEntry)
{
  ControlResponse response;

  response.setCode(200);
  response.setText("Success");
  response.setBody(parameters.wireEncode());

  NFD_LOG_TRACE("onRegSuccess: registered " << ribEntry);

  sendResponse(request->getName(), response);
}


void
RibManager::onUnRegSuccess(const shared_ptr<const Interest>& request,
                           const ControlParameters& parameters,
                           const RibEntry& ribEntry)
{
  ControlResponse response;

  response.setCode(200);
  response.setText("Success");
  response.setBody(parameters.wireEncode());

  NFD_LOG_TRACE("onUnRegSuccess: unregistered " << ribEntry);

  sendResponse(request->getName(), response);
  m_managedRib.erase(ribEntry);
}

void
RibManager::onControlHeaderSuccess()
{
  NFD_LOG_DEBUG("Local control header enabled");
}

void
RibManager::onControlHeaderError(uint32_t code, const std::string& reason)
{
  NFD_LOG_ERROR("Error: couldn't enable local control header "
                << "(code: " << code << ", info: " << reason << ")");
  m_face.shutdown();
}

void
RibManager::enableLocalControlHeader()
{
  m_nfdController->start<ndn::nfd::FaceEnableLocalControlCommand>(
    ControlParameters()
      .setLocalControlFeature(ndn::nfd::LOCAL_CONTROL_FEATURE_INCOMING_FACE_ID),
    bind(&RibManager::onControlHeaderSuccess, this),
    bind(&RibManager::onControlHeaderError, this, _1, _2));
}

void
RibManager::onNotification(const FaceEventNotification& notification)
{
  /// \todo A notification can be missed, in this case check Facelist
  NFD_LOG_TRACE("onNotification: " << notification);
  if (notification.getKind() == ndn::nfd::FACE_EVENT_DESTROYED) { //face destroyed
    m_managedRib.erase(notification.getFaceId());
  }
}

} // namespace rib
} // namespace nfd