// Copyright 2025 DeepMind Technologies Limited
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

#include "experimental/studio/llm/llm_claude.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {
namespace {

// --- Minimal JSON helpers (we only need to build a request and pull the text
// blocks out of the response; no general-purpose parser required). ----------

// Wraps `s` in quotes and escapes it as a JSON string.
std::string JsonString(const std::string& s) {
  std::string o = "\"";
  for (unsigned char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      case '\b': o += "\\b"; break;
      case '\f': o += "\\f"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          o += buf;
        } else {
          o += static_cast<char>(c);
        }
    }
  }
  o += "\"";
  return o;
}

void AppendUtf8(std::string& out, unsigned cp) {
  if (cp < 0x80) {
    out += static_cast<char>(cp);
  } else if (cp < 0x800) {
    out += static_cast<char>(0xC0 | (cp >> 6));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else {
    out += static_cast<char>(0xE0 | (cp >> 12));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  }
}

unsigned Hex4(const std::string& s, size_t i) {
  unsigned v = 0;
  for (size_t k = 0; k < 4 && i + k < s.size(); ++k) {
    char c = s[i + k];
    v <<= 4;
    if (c >= '0' && c <= '9') v |= (c - '0');
    else if (c >= 'a' && c <= 'f') v |= (c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') v |= (c - 'A' + 10);
  }
  return v;
}

// Parses the JSON string token whose opening quote is at `pos`. Appends the
// decoded contents to `out`. Returns the index just past the closing quote, or
// std::string::npos on malformed input.
size_t ParseJsonStringAt(const std::string& s, size_t pos, std::string& out) {
  if (pos >= s.size() || s[pos] != '"') return std::string::npos;
  size_t i = pos + 1;
  while (i < s.size()) {
    char c = s[i];
    if (c == '"') return i + 1;
    if (c == '\\') {
      if (i + 1 >= s.size()) return std::string::npos;
      char e = s[i + 1];
      switch (e) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'n': out += '\n'; break;
        case 't': out += '\t'; break;
        case 'r': out += '\r'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'u': {
          unsigned cp = Hex4(s, i + 2);
          AppendUtf8(out, cp);
          i += 4;  // the four hex digits (the \\u is consumed below)
          break;
        }
        default: out += e; break;
      }
      i += 2;
    } else {
      out += c;
      ++i;
    }
  }
  return std::string::npos;
}

// Concatenates the text of every {"type":"text", ...} content block.
std::string ExtractAssistantText(const std::string& json) {
  std::string out;
  const std::string key = "\"type\":\"text\"";
  size_t i = 0;
  while ((i = json.find(key, i)) != std::string::npos) {
    size_t t = json.find("\"text\":", i + key.size());
    if (t == std::string::npos) break;
    size_t q = json.find('"', t + 7);  // 7 == strlen("\"text\":")
    if (q == std::string::npos) break;
    std::string piece;
    size_t after = ParseJsonStringAt(json, q, piece);
    if (after == std::string::npos) break;
    out += piece;
    i = after;
  }
  return out;
}

// Pulls the error message out of an Anthropic error response, if present.
std::string ExtractErrorMessage(const std::string& json) {
  size_t e = json.find("\"type\":\"error\"");
  if (e == std::string::npos) e = 0;
  size_t m = json.find("\"message\":", e);
  if (m == std::string::npos) return "";
  size_t q = json.find('"', m + 10);  // 10 == strlen("\"message\":")
  if (q == std::string::npos) return "";
  std::string out;
  if (ParseJsonStringAt(json, q, out) == std::string::npos) return "";
  return out;
}

std::string BuildRequestBody(const std::string& model, int max_tokens,
                             const std::string& system,
                             const std::vector<LlmMessage>& messages) {
  std::string body = "{";
  body += "\"model\":" + JsonString(model) + ",";
  body += "\"max_tokens\":" + std::to_string(max_tokens) + ",";
  body += "\"thinking\":{\"type\":\"adaptive\"},";
  if (!system.empty()) {
    body += "\"system\":" + JsonString(system) + ",";
  }
  body += "\"messages\":[";
  for (size_t i = 0; i < messages.size(); ++i) {
    if (i) body += ",";
    body += "{\"role\":" + JsonString(messages[i].role) +
            ",\"content\":" + JsonString(messages[i].text) + "}";
  }
  body += "]}";
  return body;
}

}  // namespace

std::string ClaudeProvider::KeyFromEnv() {
  const char* k = std::getenv("ANTHROPIC_API_KEY");
  return (k && *k) ? std::string(k) : std::string();
}

ClaudeProvider::ClaudeProvider(std::string api_key)
    : api_key_(std::move(api_key)) {}

}  // namespace mujoco::studio

// ---------------------------------------------------------------------------
// Platform transport. Kept at the bottom and isolated so <windows.h> never
// bleeds into the rest of the studio sources.
// ---------------------------------------------------------------------------

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <winhttp.h>

namespace mujoco::studio {
namespace {

std::wstring Widen(const std::string& s) {
  std::wstring w;
  w.reserve(s.size());
  for (unsigned char c : s) w += static_cast<wchar_t>(c);  // ASCII headers only.
  return w;
}

// POSTs `body` to https://api.anthropic.com<path>. Returns false on a transport
// error (sets `err`); otherwise sets `status` and `response`.
bool HttpsPost(const std::string& headers, const std::string& body,
               int& status, std::string& response, std::string& err) {
  status = 0;
  response.clear();
  err.clear();

  HINTERNET session = WinHttpOpen(L"MuJoCoStudio/1.0",
                                  WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                                  WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS,
                                  0);
  if (!session) {
    err = "WinHttpOpen failed";
    return false;
  }
  HINTERNET connect =
      WinHttpConnect(session, L"api.anthropic.com", INTERNET_DEFAULT_HTTPS_PORT,
                     0);
  if (!connect) {
    err = "WinHttpConnect failed";
    WinHttpCloseHandle(session);
    return false;
  }
  HINTERNET request = WinHttpOpenRequest(
      connect, L"POST", L"/v1/messages", nullptr, WINHTTP_NO_REFERER,
      WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
  if (!request) {
    err = "WinHttpOpenRequest failed";
    WinHttpCloseHandle(connect);
    WinHttpCloseHandle(session);
    return false;
  }

  bool ok = false;
  std::wstring wheaders = Widen(headers);
  if (WinHttpAddRequestHeaders(request, wheaders.c_str(),
                               static_cast<DWORD>(-1),
                               WINHTTP_ADDREQ_FLAG_ADD) &&
      WinHttpSendRequest(request, WINHTTP_NO_ADDITIONAL_HEADERS, 0,
                         const_cast<char*>(body.data()),
                         static_cast<DWORD>(body.size()),
                         static_cast<DWORD>(body.size()), 0) &&
      WinHttpReceiveResponse(request, nullptr)) {
    DWORD code = 0;
    DWORD code_size = sizeof(code);
    WinHttpQueryHeaders(
        request, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
        WINHTTP_HEADER_NAME_BY_INDEX, &code, &code_size, WINHTTP_NO_HEADER_INDEX);
    status = static_cast<int>(code);

    for (;;) {
      DWORD avail = 0;
      if (!WinHttpQueryDataAvailable(request, &avail) || avail == 0) break;
      std::string chunk(avail, '\0');
      DWORD read = 0;
      if (!WinHttpReadData(request, chunk.data(), avail, &read)) break;
      chunk.resize(read);
      response += chunk;
    }
    ok = true;
  } else {
    err = "WinHttp request failed (code " + std::to_string(GetLastError()) + ")";
  }

  WinHttpCloseHandle(request);
  WinHttpCloseHandle(connect);
  WinHttpCloseHandle(session);
  return ok;
}

}  // namespace

LlmResult ClaudeProvider::Send(const std::string& system,
                               const std::vector<LlmMessage>& messages) {
  LlmResult r;
  if (api_key_.empty()) {
    r.error = "ANTHROPIC_API_KEY is not set.";
    return r;
  }

  const std::string body = BuildRequestBody(model_, max_tokens_, system, messages);
  const std::string headers = "x-api-key: " + api_key_ + "\r\n" +
                              "anthropic-version: 2023-06-01\r\n" +
                              "content-type: application/json";

  int status = 0;
  std::string response, err;
  if (!HttpsPost(headers, body, status, response, err)) {
    r.error = err;
    return r;
  }
  if (status != 200) {
    std::string msg = ExtractErrorMessage(response);
    r.error = "HTTP " + std::to_string(status) +
              (msg.empty() ? (": " + response) : (": " + msg));
    return r;
  }
  r.text = ExtractAssistantText(response);
  if (r.text.empty()) {
    r.error = "Empty response from Claude.";
    return r;
  }
  r.ok = true;
  return r;
}

}  // namespace mujoco::studio

#else  // !_WIN32

namespace mujoco::studio {

LlmResult ClaudeProvider::Send(const std::string& /*system*/,
                               const std::vector<LlmMessage>& /*messages*/) {
  LlmResult r;
  r.error =
      "Claude transport is only implemented for Windows (WinHTTP) in this "
      "build.";
  return r;
}

}  // namespace mujoco::studio

#endif  // _WIN32
