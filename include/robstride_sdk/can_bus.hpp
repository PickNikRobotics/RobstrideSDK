// Copyright 2026 Minjo Kim
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modified by PickNik Inc., 2026: testable receive dispatch with corrected fault
// source routing. See NOTICE for source history.

#ifndef ROBSTRIDE_SDK__CAN_BUS_HPP_
#define ROBSTRIDE_SDK__CAN_BUS_HPP_

#include <linux/can.h>
#include <atomic>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "robstride_sdk/robstride_motor.hpp"

namespace robstride_sdk
{

// One SocketCAN raw socket on a single interface, with an epoll read thread
// and batched sendmmsg()/recvmmsg() I/O.
class CanBus
{
public:
  explicit CanBus(std::string ifname);
  ~CanBus();

  CanBus(const CanBus &) = delete;
  CanBus & operator=(const CanBus &) = delete;

  // Opens and configures the socket. Must be called before Start().
  bool Open();
  void Close();

  // Associates a motor with its CAN id. Must be called before Start().
  void RegisterMotor(RobstrideMotor * motor);

  void Start();
  void Stop();

  // Returns the number of frames queued to the kernel.
  int SendFrames(const std::vector<can_frame> & frames);

  bool SendFrame(const can_frame & frame);

  // errno of the last incomplete send, or 0. ENOBUFS means the interface is
  // not draining its transmit queue, typically bus-off.
  int last_send_errno() const {return last_send_errno_.load(std::memory_order_relaxed);}

  const std::string & interface_name() const {return ifname_;}

private:
  friend struct CanBusTestAccess;

  void ReadThreadMain();
  void DispatchFrame(const can_frame & frame);

  std::string ifname_;
  int fd_ = -1;
  int epoll_fd_ = -1;
  std::atomic<bool> running_{false};
  std::atomic<int> last_send_errno_{0};
  std::thread read_thread_;
  std::unordered_map<uint8_t, RobstrideMotor *> motors_;
};

}  // namespace robstride_sdk

#endif  // ROBSTRIDE_SDK__CAN_BUS_HPP_
