/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2022, Saurab Dulal, The University of Memphis.
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
 */

#ifndef NFD_DAEMON_FACE_MULTICAST_SUPPRESSION_HPP
#define NFD_DAEMON_FACE_MULTICAST_SUPPRESSION_HPP

// #include "common/global.hpp"
// #include <ndn-cxx/util/random.hpp>

#include "common/global.hpp"
#include "common/logger.hpp"
#include "core/common.hpp"
<<<<<<< HEAD

#include <ndn-cxx/util/scheduler.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <tuple>
#include <functional>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <stdio.h>
#include <random>
#include <math.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
=======
#include <ndn-cxx/util/random.hpp>

#include <chrono>
// #include "algorithm.hpp"
#include <vector>
#include <sys/stat.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>


#ifndef NFD_DAEMON_FACE_AMS_MULTICAST_SUPPRESSION_HPP
#define NFD_DAEMON_FACE_AMS_MULTICAST_SUPPRESSION_HPP

namespace nfd {
namespace face {
namespace ams {

class _FIFO
{
public:
  
  _FIFO(char *fifo_suppression_value, char *fifo_object_details);
  
  void
  fifo_write(const std::string& content);
  
  time::milliseconds 
  fifo_read();

private:
  char* m_fifo_suppression_value;
  char* m_fifo_object_details;

};


/* Simple trie implementaiton for the name tree, and prefix match */
class NameTree
{
public:
  std::map<char, NameTree*> children;
  bool isLeaf;
  double suppressionTime;

  NameTree();

  void
  insert(std::string prefix, double value);

  double
  longestPrefixMatch(const std::string& prefix); //longest prefix match

  time::milliseconds
  getSuppressionTimer(const std::string& prefix);

};

class EMAMeasurements
{

public:
  EMAMeasurements(double expMovingAverage, int lastDuplicateCount, double suppressionTime);

  void
  addUpdateEMA(int duplicateCount, bool wasForwarded, std::string name);

  scheduler::EventId&
  getEMAExpiration()
  {
    return this->m_expirationId;
  }

  void
  setEMAExpiration(scheduler::EventId& expirationId)
  {
    this->m_expirationId = expirationId;
  }

  float
  getEMACurrent()
  {
    return this->m_expMovingAverageCurrent;
  }

  float
  getEMAPrev()
  {
    return this->m_expMovingAveragePrev;
  }

  void
  updateDelayTime(bool wasForwarded, std::string name);

  double
  getCurrentSuppressionTime()
  {
    return m_currentSuppressionTime;
  }

  void
  setSSthress(double val, int factor = 2)
  {
    m_ssthress = val/factor;
  }

  double
  getMinimumSuppressionTime()
  {
    return m_minSuppressionTime;
  }

private:
  double m_expMovingAveragePrev;
  double m_expMovingAverageCurrent;
  double m_currentSuppressionTime;
  scheduler::EventId m_expirationId;
  double m_computedMaxSuppressionTime;
  int m_lastDuplicateCount;
  int m_maxDuplicateCount;
  double m_minSuppressionTime;
  double m_ssthress;
  int ignore;
  _FIFO m_fifo;
};


class MulticastSuppression
{
public:
  
  // MulticastSuppression();

  struct ObjectHistory
  {
    int counter;
    bool isForwarded;
  };

  void
  recordInterest(const Interest interest, bool isForwarded = false);

  void
  recordData(const Data data, bool isForwarded = false);

  int
  getDuplicateCount(const Name name, char type)
  {
    auto temp_map = getRecorder(type);
    auto it = temp_map->find(name);
    if (it != temp_map->end())
      return it->second.counter;
    return 0;
  }

  std::map<Name, ObjectHistory>*
  getRecorder(char type)
  {
    return (type == 'i') ? &m_interestHistory : &m_dataHistory;
  }

  std::map<Name, std::shared_ptr<EMAMeasurements>>*
  getEMARecorder(char type)
  {
    return (type =='i') ? &m_EMA_interest : &m_EMA_data;
  }

  NameTree*
  getNameTree(char type)
  {
    return (type =='i') ? &m_interestNameTree : &m_dataNameTree;
  }

  bool
  interestInflight(const Interest interest) const
  {
    auto name = interest.getName();
    return (m_interestHistory.find(name) != m_interestHistory.end() );
  }

  bool
  dataInflight(const Data data) const
  {
    auto name = data.getName();
    return (m_dataHistory.find(name) != m_dataHistory.end());
  }

  time::milliseconds
  getRandomTime()
  {
    return time::milliseconds(1 + (std::rand() % (10)));
  }

  void
  updateMeasurement(Name name, char type);

  // set interest or data expiration
  void
  setUpdateExpiration(time::milliseconds entryLifetime, Name name, char type);

  time::milliseconds
  getDelayTimer(Name name, char type);

  bool
  getForwardedStatus(ndn::Name prefix, char type)
  {
    auto recorder = getRecorder(type);
    auto it = recorder->find(prefix);
    return it != recorder->end() ? it->second.isForwarded : false; // if record exist, send whatever is the status else send false
  }

private:
  std::map<ndn::Name, ObjectHistory> m_dataHistory;
  std::map<ndn::Name, ObjectHistory> m_interestHistory;
  std::map <Name, scheduler::EventId> m_objectExpirationTimer;
  std::map<Name, std::shared_ptr<EMAMeasurements>> m_EMA_data;
  std::map<Name, std::shared_ptr<EMAMeasurements>> m_EMA_interest;
  NameTree m_dataNameTree;
  NameTree m_interestNameTree;
  _FIFO m_fifo;
};
} //namespace ams
} //namespace face
} //namespace nfd

#endif // NFD_DAEMON_FACE_MULTICAST_SUPPRESSION_HPP
