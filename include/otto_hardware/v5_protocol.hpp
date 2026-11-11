// v5_protocol.hpp
//
// THE message format between the Jetson and the V5 Brain. One file, included by BOTH sides:
//   Jetson: ros2_ws/src/otto_hardware/src/omni_wheel_hardware.cpp   (g++ on Ubuntu)
//   Brain:  v5_bridge_cpp/src/main.cpp                               (PROS, arm-none-eabi-g++)
// Readable version: Documentation/15-jetson-brain-protocol.md
//
// To change the format: change that page first, then this file, then rebuild and upload BOTH sides.
// Plain C++17 with no ROS and no PROS includes, so both compilers accept it.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "v5_protocol.hpp assumes a little-endian machine (the Jetson and the V5 Brain both are)."
#endif

namespace v5_protocol {

// ---- Magic numbers: the first 8 bytes of every message say what kind of message it is ----

// Jetson -> Brain
constexpr std::uint64_t MOTOR_COMMAND_MAGIC = 0xFEFAABCD1234BEEFULL;   // Packet: target speeds
constexpr std::uint64_t RESET_ENCODERS_MAGIC = 0xABCD5869B0B1CCCCULL;  // 8 bytes, magic only

// Brain -> Jetson
constexpr std::uint64_t POSITIONS_MAGIC = 0xF23BDEAD6789ACBDULL;  // Packet: how far each has turned
constexpr std::uint64_t SPEEDS_MAGIC = 0xF81CB3924EA7C0CAULL;     // Packet: current speeds

// ---- The standard packet: same layout for commands, positions and speeds ----
// Wheels:       rad/s (speeds) or rad (positions) AT THE WHEEL.
// intake1..3:   rad/s or rad AT THE MOTOR (any 3 extra motors; the names match the ROS joints).
#pragma pack(push, 1)
struct Packet {
  std::uint64_t magic;
  float front_left;
  float front_right;
  float back_left;
  float back_right;
  float intake1;
  float intake2;
  float intake3;
};
#pragma pack(pop)

constexpr std::size_t MAGIC_SIZE = sizeof(std::uint64_t);  // 8
constexpr std::size_t PACKET_SIZE = sizeof(Packet);         // 36

// If either compiler laid the packet out differently, the build stops here instead of the
// robot silently misreading messages.
static_assert(sizeof(float) == 4, "float must be 4 bytes");
static_assert(PACKET_SIZE == 36, "Packet must be 36 bytes with no padding");
static_assert(offsetof(Packet, front_left) == 8, "Packet layout changed");
static_assert(offsetof(Packet, front_right) == 12, "Packet layout changed");
static_assert(offsetof(Packet, back_left) == 16, "Packet layout changed");
static_assert(offsetof(Packet, back_right) == 20, "Packet layout changed");
static_assert(offsetof(Packet, intake1) == 24, "Packet layout changed");
static_assert(offsetof(Packet, intake2) == 28, "Packet layout changed");
static_assert(offsetof(Packet, intake3) == 32, "Packet layout changed");

// ---- Timing both sides agree on ----

// The Brain stops every motor if no motor command has arrived for this long.
constexpr std::uint32_t COMMAND_TIMEOUT_MS = 250;

// The Brain sends Positions + Speeds this often. The Jetson's controller runs at 100 Hz.
constexpr std::uint32_t REPORT_PERIOD_MS = 10;

// ---- Reading messages out of a stream of bytes ----

// How many bytes the message starting with this magic takes, or 0 if it isn't a known magic.
constexpr std::size_t message_size(std::uint64_t magic) {
  return (magic == MOTOR_COMMAND_MAGIC || magic == POSITIONS_MAGIC || magic == SPEEDS_MAGIC)
             ? PACKET_SIZE
             : (magic == RESET_ENCODERS_MAGIC ? MAGIC_SIZE : 0);
}

// Reads 8 bytes as a magic number (memcpy is safe for bytes at any address).
inline std::uint64_t read_magic(const std::uint8_t* bytes) {
  std::uint64_t magic;
  std::memcpy(&magic, bytes, MAGIC_SIZE);
  return magic;
}

// Reads 36 bytes as a Packet.
inline Packet read_packet(const std::uint8_t* bytes) {
  Packet packet;
  std::memcpy(&packet, bytes, PACKET_SIZE);
  return packet;
}

// Finds every complete message in bytes[0 .. length), oldest first, and calls
// on_message(magic, pointer_to_message_start) for each one. Bytes that aren't part of a known
// message are skipped. Returns how many bytes at the front are finished with, so the caller can
// erase them. Anything after that is the start of a message that hasn't fully arrived yet.
template <typename OnMessage>
std::size_t parse_stream(const std::uint8_t* bytes, std::size_t length, OnMessage&& on_message) {
  std::size_t i = 0;
  while (i + MAGIC_SIZE <= length) {
    const std::uint64_t magic = read_magic(bytes + i);
    const std::size_t size = message_size(magic);
    if (size == 0) {  // not the start of a message: move on one byte
      ++i;
      continue;
    }
    if (i + size > length) {  // message started but not complete yet: keep it for next time
      break;
    }
    on_message(magic, bytes + i);
    i += size;
  }
  return i;
}

}  // namespace v5_protocol
