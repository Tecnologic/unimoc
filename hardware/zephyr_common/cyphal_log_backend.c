/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_log_backend.c
 *    @brief Zephyr log backend that hands log lines to the Cyphal thread,
 *           which publishes them as uavcan.diagnostic.Record.
 *
 *    The backend runs in the log processing thread. It formats each message
 *    without timestamp and level ("<module>: <text>") and queues it; lines
 *    above CONFIG_UNIMOC_CYPHAL_LOG_LEVEL and lines that do not fit into the
 *    queue are dropped. The console and RTT backends are not affected.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "cyphal_log_backend.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>

/* Longest text of a diagnostic record line (uavcan.diagnostic.Record allows 255). */
enum { kLineLength = 112 };

struct LogLine {
  uint8_t level;
  char text[kLineLength + 1];
};

K_MSGQ_DEFINE(cyphal_log_queue, sizeof(struct LogLine), 8, 4);

static struct LogLine line;
static size_t line_length;
static uint8_t output_buffer[32];

// NOLINTNEXTLINE(readability-non-const-parameter): the signature of log_output_func_t.
static int CharOut(uint8_t* data, size_t length, void* ctx) {
  ARG_UNUSED(ctx);
  for (size_t i = 0; i < length; ++i) {
    if (data[i] != '\r' && data[i] != '\n' && line_length < kLineLength) {
      line.text[line_length++] = (char)data[i];
    }
  }
  return (int)length;
}

LOG_OUTPUT_DEFINE(cyphal_log_output, CharOut, output_buffer, sizeof(output_buffer));

static void Process(const struct log_backend* const backend, union log_msg_generic* msg) {
  ARG_UNUSED(backend);
  const uint8_t kLevel = log_msg_get_level(&msg->log);

  if (kLevel == LOG_LEVEL_NONE || kLevel > CONFIG_UNIMOC_CYPHAL_LOG_LEVEL) {
    return;
  }
  line_length = 0;
  line.level = kLevel;
  log_output_msg_process(&cyphal_log_output, &msg->log, LOG_OUTPUT_FLAG_CRLF_NONE);
  line.text[line_length] = '\0';
  (void)k_msgq_put(&cyphal_log_queue, &line, K_NO_WAIT);
}

static void Dropped(const struct log_backend* const backend, uint32_t cnt) {
  ARG_UNUSED(backend);
  ARG_UNUSED(cnt);
}

static void Panic(const struct log_backend* const backend) { ARG_UNUSED(backend); }

static const struct log_backend_api kCyphalLogBackendApi = {
    .process = Process,
    .dropped = Dropped,
    .panic = Panic,
};

LOG_BACKEND_DEFINE(unimoc_cyphal_log_backend, kCyphalLogBackendApi, true);

bool UnimocCyphalLogGet(uint8_t* level, char* text, size_t size) {
  struct LogLine queued;

  if (size == 0 || k_msgq_get(&cyphal_log_queue, &queued, K_NO_WAIT) != 0) {
    return false;
  }
  *level = queued.level;
  size_t length = 0;
  while (length + 1 < size && queued.text[length] != '\0') {
    text[length] = queued.text[length];
    ++length;
  }
  text[length] = '\0';
  return true;
}
