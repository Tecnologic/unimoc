/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file firmware_update_test.cpp
 *    @brief Tests of the firmware image header, the download into the bank and uavcan.file.Read.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "cyphal/firmware_update.hpp"
#include <zephyr/ztest.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <span>
#include <string_view>
#include "can_test_bus.hpp"
#include "cyphal/canard_transport.hpp"
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/file_service.hpp"
#include "cyphal/firmware_image.hpp"
#include "cyphal/node.hpp"

namespace unimoc::cyphal::test {
namespace {

using unimoc::test::CanTestBus;
using unimoc::test::CanTestMedia;

constexpr std::string_view kBoard = "native_sim";
constexpr std::size_t kPayloadSize = 1000U;  // with the header: four full chunks and a short one
constexpr std::size_t kImageSize = FirmwareImageHeader::kSize + kPayloadSize;

// ----- A RAM bank ---------------------------------------------------------------

constexpr uint32_t kBankSize = 16384U;
std::array<std::byte, kBankSize> g_bank{};
bool g_bank_fails{false};

bool g_activated{false};
ImageStatus g_activate_result{ImageStatus::kOk};

bool Begin() noexcept {
  g_bank.fill(std::byte{0xFF});
  g_activated = false;
  return true;
}

bool Finish() noexcept { return true; }

ImageStatus Activate() noexcept {
  g_activated = (g_activate_result == ImageStatus::kOk);
  return g_activate_result;
}

bool Write(const uint32_t offset, const std::span<const std::byte> data) noexcept {
  if (g_bank_fails || offset + data.size() > g_bank.size()) {
    return false;
  }
  std::copy(data.begin(), data.end(), g_bank.begin() + offset);
  return true;
}

bool ReadBank(const uint32_t offset, const std::span<std::byte> data) noexcept {
  if (offset + data.size() > g_bank.size()) {
    return false;
  }
  std::copy_n(g_bank.begin() + offset, data.size(), data.begin());
  return true;
}

constexpr FirmwareBank kBank{Begin, Write, Finish, ReadBank, Activate, kBankSize};
constexpr FirmwareBank kSmallBank{Begin, Write, Finish, ReadBank, Activate, 512U};
constexpr FirmwareBank kNoBank{};

// ----- Images -------------------------------------------------------------------

void PutLe32(std::span<std::byte> bytes, const std::size_t offset, const uint32_t value) {
  for (std::size_t i = 0U; i < 4U; ++i) {
    bytes[offset + i] = static_cast<std::byte>((value >> (8U * i)) & 0xFFU);
  }
}

/// An image as scripts/make_firmware_image.py writes it.
template <std::size_t kPayloadBytes = kPayloadSize>
std::array<std::byte, FirmwareImageHeader::kSize + kPayloadBytes> MakeImage(
    const std::string_view board = kBoard) {
  std::array<std::byte, FirmwareImageHeader::kSize + kPayloadBytes> image{};
  const std::span<std::byte> kPayload{image.data() + FirmwareImageHeader::kSize, kPayloadBytes};
  for (std::size_t i = 0U; i < kPayload.size(); ++i) {
    kPayload[i] = static_cast<std::byte>((i * 7U) & 0xFFU);
  }
  std::copy(FirmwareImageHeader::kMagic.begin(),
            FirmwareImageHeader::kMagic.end(),
            reinterpret_cast<uint8_t*>(image.data()));
  image[4] = std::byte{1U};  // header version
  image[6] = std::byte{2U};  // software 2.5
  image[7] = std::byte{5U};
  PutLe32(image, 8U, 0x89ABCDEFU);
  PutLe32(image, 12U, 0x01234567U);
  PutLe32(image, 16U, kPayloadBytes);
  PutLe32(image, 20U, Crc32Final(Crc32Update(Crc32Init(), kPayload)));
  std::copy(board.begin(), board.end(), reinterpret_cast<char*>(image.data() + 24U));
  return image;
}

/// Answers every request of @p update from @p image until it ends.
void Serve(FirmwareUpdate& update, const std::span<const std::byte> image, uint64_t& now_us) {
  for (unsigned i = 0U; i < 100U && update.GetState() == FirmwareUpdate::State::kDownloading; ++i) {
    now_us += 1000U;
    const auto kRequest = update.Poll(now_us);
    if (!kRequest.has_value()) {
      continue;
    }
    const std::size_t kOffset = std::min<std::size_t>(kRequest->offset, image.size());
    const std::size_t kLength = std::min(FirmwareUpdate::kChunk, image.size() - kOffset);
    update.OnResponse(0U, image.subspan(kOffset, kLength));
  }
}

ZTEST(firmware_update, test_crc32_is_ieee_802_3) {
  constexpr std::string_view kCheck = "123456789";
  const std::span<const std::byte> kBytes{reinterpret_cast<const std::byte*>(kCheck.data()), kCheck.size()};
  zexpect_equal(Crc32Final(Crc32Update(Crc32Init(), kBytes)), 0xCBF43926U);
  // Incremental: two parts give the same.
  const uint32_t kParts = Crc32Update(Crc32Update(Crc32Init(), kBytes.first(4U)), kBytes.subspan(4U));
  zexpect_equal(Crc32Final(kParts), 0xCBF43926U);
}

ZTEST(firmware_update, test_header_is_parsed_and_checked) {
  auto image = MakeImage();
  FirmwareImageHeader header{};
  zassert_equal(ParseImageHeader(image, header), ImageStatus::kOk);
  zexpect_equal(header.software_major, 2U);
  zexpect_equal(header.software_minor, 5U);
  zexpect_equal(header.vcs_revision, 0x0123456789ABCDEFULL);
  zexpect_equal(header.payload_size, kPayloadSize);
  zexpect_true(header.Board() == kBoard);

  zexpect_equal(ParseImageHeader(std::span<const std::byte>{image}.first(39U), header),
                ImageStatus::kTooShort);
  image[4] = std::byte{2U};
  zexpect_equal(ParseImageHeader(image, header), ImageStatus::kBadVersion);
  image[0] = std::byte{'X'};
  zexpect_equal(ParseImageHeader(image, header), ImageStatus::kBadMagic);
}

ZTEST(firmware_update, test_a_good_image_is_ready_in_the_bank) {
  const auto kImage = MakeImage();
  FirmwareUpdate update{kBank, kBoard};
  zexpect_equal(update.GetState(), FirmwareUpdate::State::kIdle);
  zassert_true(update.Begin(7U, "unimoc.bin"));
  zexpect_false(update.Begin(7U, "again.bin"));  // one download at a time

  uint64_t now = 0U;
  const auto kFirst = update.Poll(now);
  zassert_true(kFirst.has_value());
  zexpect_equal(kFirst->server, 7U);
  zexpect_equal(kFirst->offset, 0U);
  zexpect_true(kFirst->path == "unimoc.bin");
  zexpect_false(update.Poll(now + 1000U).has_value());  // waits for the answer
  update.OnResponse(0U, std::span<const std::byte>{kImage}.first(FirmwareUpdate::kChunk));

  Serve(update, kImage, now);
  zassert_equal(update.GetState(), FirmwareUpdate::State::kReady);
  zexpect_equal(update.Result(), ImageStatus::kOk);
  zexpect_equal(update.Received(), kImageSize);
  zexpect_equal(update.Header().software_minor, 5U);
  // The bank holds the payload behind the UNIMOC header, and the bootloader was told to take it over.
  zexpect_true(std::equal(kImage.begin() + FirmwareImageHeader::kSize, kImage.end(), g_bank.begin()));
  zexpect_true(g_activated);
}

ZTEST(firmware_update, test_a_file_of_whole_chunks_ends_with_an_empty_read) {
  std::array<std::byte, 512U> image{};
  const auto kGood = MakeImage();
  std::copy_n(kGood.begin(), FirmwareImageHeader::kSize, image.begin());
  const std::span<std::byte> kPayload{image.data() + FirmwareImageHeader::kSize,
                                      image.size() - FirmwareImageHeader::kSize};
  PutLe32(image, 16U, static_cast<uint32_t>(kPayload.size()));
  PutLe32(image, 20U, Crc32Final(Crc32Update(Crc32Init(), kPayload)));
  FirmwareUpdate update{kBank, kBoard};
  zassert_true(update.Begin(7U, "a"));
  uint64_t now = 0U;
  Serve(update, image, now);
  zexpect_equal(update.GetState(), FirmwareUpdate::State::kReady);
  zexpect_equal(update.Received(), image.size());
}

ZTEST(firmware_update, test_a_bad_image_fails_with_its_reason) {
  uint64_t now = 0U;
  {
    auto image = MakeImage();
    image[FirmwareImageHeader::kSize + 3U] ^= std::byte{0x01};
    FirmwareUpdate update{kBank, kBoard};
    zassert_true(update.Begin(7U, "a"));
    Serve(update, image, now);
    zexpect_equal(update.GetState(), FirmwareUpdate::State::kFailed);
    zexpect_equal(update.Result(), ImageStatus::kBadChecksum);
  }
  {
    const auto kImage = MakeImage("unimoc_jacobi");
    FirmwareUpdate update{kBank, kBoard};
    zassert_true(update.Begin(7U, "a"));
    Serve(update, kImage, now);
    zexpect_equal(update.Result(), ImageStatus::kWrongBoard);
  }
  {
    const auto kImage = MakeImage();
    FirmwareUpdate update{kBank, kBoard};
    zassert_true(update.Begin(7U, "a"));
    Serve(update, std::span<const std::byte>{kImage}.first(kImageSize - 1U), now);  // cut short
    zexpect_equal(update.Result(), ImageStatus::kWrongSize);
  }
  {
    const auto kImage = MakeImage();
    FirmwareUpdate update{kSmallBank, kBoard};
    zassert_true(update.Begin(7U, "a"));
    Serve(update, kImage, now);
    zexpect_equal(update.Result(), ImageStatus::kTooLarge);
  }
  {
    FirmwareUpdate update{kBank, kBoard};
    zassert_true(update.Begin(7U, "a"));
    (void)update.Poll(now);
    update.OnResponse(2U, {});  // uavcan.file.Error NOT_FOUND
    zexpect_equal(update.GetState(), FirmwareUpdate::State::kFailed);
    zexpect_equal(update.FileError(), 2U);
  }
  {
    const auto kImage = MakeImage();
    g_bank_fails = true;
    FirmwareUpdate update{kBank, kBoard};
    zassert_true(update.Begin(7U, "a"));
    Serve(update, kImage, now);
    g_bank_fails = false;
    zexpect_equal(update.FileError(), FirmwareUpdate::kErrorBank);
  }
}

ZTEST(firmware_update, test_the_bootloader_can_refuse_the_image) {
  const auto kImage = MakeImage();
  uint64_t now = 0U;
  for (const ImageStatus kRefusal : {ImageStatus::kNotNewer, ImageStatus::kNoBootImage}) {
    g_activate_result = kRefusal;
    FirmwareUpdate update{kBank, kBoard};
    zassert_true(update.Begin(7U, "a"));
    Serve(update, kImage, now);
    zexpect_equal(update.GetState(), FirmwareUpdate::State::kFailed);
    zexpect_equal(update.Result(), kRefusal);
    zexpect_false(g_activated);
  }
  g_activate_result = ImageStatus::kOk;
}

ZTEST(firmware_update, test_a_wrong_image_is_refused_before_the_bank_is_written) {
  const auto kImage = MakeImage("unimoc_jacobi");
  g_bank.fill(std::byte{0x5A});
  FirmwareUpdate update{kBank, kBoard};
  zassert_true(update.Begin(7U, "a"));
  g_bank.fill(std::byte{0x5A});
  uint64_t now = 0U;
  Serve(update, kImage, now);
  zexpect_equal(update.Result(), ImageStatus::kWrongBoard);
  zexpect_equal(update.Received(), 0U);
  zexpect_true(std::all_of(g_bank.begin(), g_bank.end(), [](const std::byte kByte) {
    return kByte == std::byte{0x5A};
  }));
}

ZTEST(firmware_update, test_a_silent_server_is_retried_then_given_up) {
  FirmwareUpdate update{kBank, kBoard};
  zassert_true(update.Begin(7U, "a"));
  uint64_t now = 0U;
  zassert_true(update.Poll(now).has_value());
  unsigned repeats = 0U;
  for (unsigned i = 0U; i < 10U; ++i) {
    now += FirmwareUpdate::kTimeoutUs;
    const auto kRequest = update.Poll(now);
    if (kRequest.has_value()) {
      ++repeats;
      zexpect_equal(kRequest->offset, 0U);  // the same request again
    }
  }
  zexpect_equal(repeats, FirmwareUpdate::kRetries);
  zexpect_equal(update.GetState(), FirmwareUpdate::State::kFailed);
  zexpect_equal(update.FileError(), FirmwareUpdate::kErrorTimeout);
}

ZTEST(firmware_update, test_begin_is_refused_without_a_bank_or_path) {
  FirmwareUpdate none{kNoBank, kBoard};
  zexpect_false(none.Begin(7U, "a"));
  FirmwareUpdate update{kBank, kBoard};
  zexpect_false(update.Begin(7U, ""));
  zexpect_false(update.Begin(7U, std::string_view{"x", 0U}));
  std::array<char, FirmwareUpdate::kMaxPath + 1U> long_path{};
  long_path.fill('a');
  zexpect_false(update.Begin(7U, std::string_view{long_path.data(), long_path.size()}));
}

// ----- Two nodes on one bus: uavcan.file.Read end to end ------------------------------

constexpr uint8_t kDriveId = 20U;
constexpr uint8_t kServerId = 5U;

/// Serves one file, the image.
class ImageSource final : public FileSource {
 public:
  explicit ImageSource(std::span<const std::byte> image) : image_{image} {}
  [[nodiscard]] FileReadResult Read(const std::string_view path,
                                    const uint64_t offset,
                                    const std::span<std::byte> data) noexcept override {
    ++reads_;
    if (path != "fw/unimoc.bin") {
      return {file_error::kNotFound, 0U};
    }
    const std::size_t kOffset = std::min<std::size_t>(offset, image_.size());
    const std::size_t kLength = std::min(data.size(), image_.size() - kOffset);
    std::copy_n(image_.begin() + static_cast<std::ptrdiff_t>(kOffset), kLength, data.begin());
    return {file_error::kOk, kLength};
  }
  [[nodiscard]] unsigned Reads() const { return reads_; }

 private:
  std::span<const std::byte> image_;
  unsigned reads_{0U};
};

struct Pair {
  explicit Pair(std::span<const std::byte> image)
      : drive_media{bus, kCanFdMtu},
        server_media{bus, kCanFdMtu},
        drive_interfaces{&drive_media},
        server_interfaces{&server_media},
        drive_transport{drive_arena.bytes, drive_interfaces},
        server_transport{server_arena.bytes, server_interfaces},
        drive{drive_transport},
        server{server_transport},
        source{image},
        file_server{source},
        update{kBank, kBoard} {}

  void Run(const unsigned rounds) {
    for (unsigned i = 0U; i < rounds; ++i) {
      now += 1000U;
      drive.Spin(now);  // first: the transport clock dates the request of update.Spin()
      update.Spin(now);
      server.Spin(now);
    }
  }

  CanardArena<32768> drive_arena;
  CanardArena<32768> server_arena;
  CanTestBus bus;
  CanTestMedia drive_media;
  CanTestMedia server_media;
  std::array<CanMedia*, 1> drive_interfaces;
  std::array<CanMedia*, 1> server_interfaces;
  CanardTransport drive_transport;
  CanardTransport server_transport;
  CyphalNode drive;
  CyphalNode server;
  ImageSource source;
  FileServer file_server;
  FirmwareUpdateService update;
  uint64_t now{1'000'000U};
};

Pair& MakePair(std::span<const std::byte> image) {
  alignas(Pair) static std::array<std::byte, sizeof(Pair)> storage;
  static Pair* pair = nullptr;
  if (pair != nullptr) {
    pair->~Pair();
  }
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): placement new into static storage.
  pair = new (storage.data()) Pair{image};
  return *pair;
}

std::span<const uint8_t> Path(const std::string_view path) {
  return std::span<const uint8_t>{reinterpret_cast<const uint8_t*>(path.data()), path.size()};
}

uint32_t ReadNatural(const RegisterProvider& provider, const std::size_t index) {
  RegisterValue value{};
  (void)provider.Read(index, value);
  return static_cast<uint32_t>(GetInteger(value).value_or(-1));
}

ZTEST(firmware_update, test_the_service_downloads_over_file_read) {
  static const auto kImage = MakeImage();
  Pair& pair = MakePair(kImage);
  zassert_true(pair.update.Start(pair.drive));
  zassert_true(pair.server.AddServer(port::kFileRead, FileServer::kRequestExtent, pair.file_server));
  zassert_true(pair.drive.Start(kDriveId, pair.now));
  zassert_true(pair.server.Start(kServerId, pair.now));
  zexpect_equal(pair.update.Count(), 5U);
  zexpect_true(pair.update.Name(0U) == "unimoc.update.state");

  zexpect_equal(pair.update.Begin(kNodeIdUnset, Path("fw/unimoc.bin")).status, cmd::kStatusBadParameter);
  zexpect_equal(pair.update.Begin(kServerId, Path("")).status, cmd::kStatusBadParameter);
  zassert_equal(pair.update.Begin(kServerId, Path("fw/unimoc.bin")).status, cmd::kStatusSuccess);
  zexpect_true(pair.update.Downloading());
  zexpect_equal(pair.update.Begin(kServerId, Path("fw/unimoc.bin")).status, cmd::kStatusBadState);

  pair.Run(100U);
  zassert_equal(pair.update.Update().GetState(), FirmwareUpdate::State::kReady);
  zexpect_equal(pair.source.Reads(), 5U);  // 1040 bytes: four full reads of 256 bytes and a short one
  zexpect_equal(ReadNatural(pair.update, 0U), 2U);  // ready
  zexpect_equal(ReadNatural(pair.update, 1U), 0U);  // ok
  zexpect_equal(ReadNatural(pair.update, 3U), kImageSize);
  zexpect_true(std::equal(kImage.begin() + FirmwareImageHeader::kSize, kImage.end(), g_bank.begin()));
}

ZTEST(firmware_update, test_a_long_download_wraps_the_five_bit_transfer_id) {
  // 12 KB: 49 requests, so the transfer-ID of the request wraps at 32; the response carries it modulo 32.
  static const auto kImage = MakeImage<12000U>();
  Pair& pair = MakePair(kImage);
  zassert_true(pair.update.Start(pair.drive));
  zassert_true(pair.server.AddServer(port::kFileRead, FileServer::kRequestExtent, pair.file_server));
  zassert_true(pair.drive.Start(kDriveId, pair.now));
  zassert_true(pair.server.Start(kServerId, pair.now));
  zassert_equal(pair.update.Begin(kServerId, Path("fw/unimoc.bin")).status, cmd::kStatusSuccess);
  pair.Run(400U);
  zassert_equal(pair.update.Update().GetState(), FirmwareUpdate::State::kReady);
  zexpect_equal(pair.source.Reads(), 48U);  // 12040 bytes: 47 full reads and a short one
  zexpect_true(std::equal(kImage.begin() + FirmwareImageHeader::kSize, kImage.end(), g_bank.begin()));
}

ZTEST(firmware_update, test_an_unknown_file_fails_with_not_found) {
  static const auto kImage = MakeImage();
  Pair& pair = MakePair(kImage);
  zassert_true(pair.update.Start(pair.drive));
  zassert_true(pair.server.AddServer(port::kFileRead, FileServer::kRequestExtent, pair.file_server));
  zassert_true(pair.drive.Start(kDriveId, pair.now));
  zassert_true(pair.server.Start(kServerId, pair.now));
  zassert_equal(pair.update.Begin(kServerId, Path("fw/other.bin")).status, cmd::kStatusSuccess);
  pair.Run(20U);
  zexpect_equal(pair.update.Update().GetState(), FirmwareUpdate::State::kFailed);
  zexpect_equal(ReadNatural(pair.update, 2U), file_error::kNotFound);
}

ZTEST(firmware_update, test_clients_are_checked) {
  static const auto kImage = MakeImage();
  Pair& pair = MakePair(kImage);
  zexpect_false(pair.drive.Request(port::kFileRead, kServerId, 0U, {}));  // not added
  zassert_true(pair.update.Start(pair.drive));
  zexpect_false(pair.drive.AddClient(port::kFileRead, 8U, pair.update));  // taken
  zexpect_false(pair.drive.AddClient(512U, 8U, pair.update));             // no service-ID
  zexpect_false(pair.drive.Request(port::kFileRead, kServerId, 0U, {}));  // no node-ID yet
  zassert_true(pair.drive.Start(kDriveId, pair.now));
  zexpect_false(pair.drive.Request(port::kFileRead, 200U, 0U, {}));  // no server
}

}  // namespace

ZTEST_SUITE(firmware_update, nullptr, nullptr, nullptr, nullptr, nullptr);

}  // namespace unimoc::cyphal::test
