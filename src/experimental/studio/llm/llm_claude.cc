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

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {
namespace {

constexpr int kMaxToolIterations = 20;

// --- Minimal JSON helpers. We build the request by hand and pull the pieces we
// need out of the response; no general-purpose parser required. --------------

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

// Parses the JSON string token whose opening quote is at `pos`, appending the
// decoded contents to `out`. Returns the index past the closing quote, or npos.
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
          AppendUtf8(out, Hex4(s, i + 2));
          i += 4;
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

// Given `pos` at a '{' or '[', returns the index just past the matching close,
// respecting strings/escapes. npos on malformed input.
size_t SkipJsonValue(const std::string& s, size_t pos) {
  if (pos >= s.size() || (s[pos] != '{' && s[pos] != '[')) {
    return std::string::npos;
  }
  const char open = s[pos];
  const char close = (open == '{') ? '}' : ']';
  int depth = 0;
  bool in_str = false;
  for (size_t i = pos; i < s.size(); ++i) {
    char c = s[i];
    if (in_str) {
      if (c == '\\') { ++i; continue; }
      if (c == '"') in_str = false;
    } else if (c == '"') {
      in_str = true;
    } else if (c == open) {
      ++depth;
    } else if (c == close) {
      if (--depth == 0) return i + 1;
    }
  }
  return std::string::npos;
}

// Reads a JSON string value that follows `key` (e.g. "\"id\":") in `s`.
std::string ReadStringField(const std::string& s, const std::string& key) {
  size_t k = s.find(key);
  if (k == std::string::npos) return "";
  size_t q = s.find('"', k + key.size());
  if (q == std::string::npos) return "";
  std::string out;
  ParseJsonStringAt(s, q, out);
  return out;
}

// Concatenates the text of every {"type":"text", ...} content block.
std::string ExtractAssistantText(const std::string& json) {
  std::string out;
  const std::string key = "\"type\":\"text\"";
  size_t i = 0;
  while ((i = json.find(key, i)) != std::string::npos) {
    size_t t = json.find("\"text\":", i + key.size());
    if (t == std::string::npos) break;
    size_t q = json.find('"', t + 7);
    if (q == std::string::npos) break;
    std::string piece;
    size_t after = ParseJsonStringAt(json, q, piece);
    if (after == std::string::npos) break;
    out += piece;
    i = after;
  }
  return out;
}

std::string ExtractErrorMessage(const std::string& json) {
  size_t e = json.find("\"type\":\"error\"");
  size_t m = json.find("\"message\":", e == std::string::npos ? 0 : e);
  if (m == std::string::npos) return "";
  size_t q = json.find('"', m + 10);
  if (q == std::string::npos) return "";
  std::string out;
  ParseJsonStringAt(json, q, out);
  return out;
}

// Returns the raw top-level "content":[...] array substring (with brackets).
std::string ExtractRawContentArray(const std::string& json) {
  size_t c = json.find("\"content\":");
  if (c == std::string::npos) return "";
  size_t b = json.find('[', c);
  if (b == std::string::npos) return "";
  size_t e = SkipJsonValue(json, b);
  if (e == std::string::npos) return "";
  return json.substr(b, e - b);
}

struct ToolUse {
  std::string id;
  std::string name;
  std::string input;  // raw JSON object
};

// Finds every {"type":"tool_use", ...} block (Anthropic emits "type" first).
std::vector<ToolUse> ExtractToolUseBlocks(const std::string& json) {
  std::vector<ToolUse> out;
  const std::string key = "\"type\":\"tool_use\"";
  size_t i = 0;
  while ((i = json.find(key, i)) != std::string::npos) {
    size_t bs = json.rfind('{', i);
    size_t be = (bs == std::string::npos) ? std::string::npos
                                          : SkipJsonValue(json, bs);
    if (bs == std::string::npos || be == std::string::npos) {
      i += key.size();
      continue;
    }
    const std::string block = json.substr(bs, be - bs);
    ToolUse tu;
    tu.id = ReadStringField(block, "\"id\":");
    tu.name = ReadStringField(block, "\"name\":");
    size_t ip = block.find("\"input\":");
    if (ip != std::string::npos) {
      size_t ob = block.find('{', ip + 8);
      if (ob != std::string::npos) {
        size_t oe = SkipJsonValue(block, ob);
        if (oe != std::string::npos) tu.input = block.substr(ob, oe - ob);
      }
    }
    if (tu.input.empty()) tu.input = "{}";
    out.push_back(std::move(tu));
    i = be;
  }
  return out;
}

// Serializes conversation turns as the inner contents of a "messages" array.
std::string SerializeMessages(const std::vector<LlmMessage>& messages) {
  std::string inner;
  for (size_t i = 0; i < messages.size(); ++i) {
    if (i) inner += ",";
    inner += "{\"role\":" + JsonString(messages[i].role) +
             ",\"content\":" + JsonString(messages[i].text) + "}";
  }
  return inner;
}

// Serializes the tools array (input_schema is already a JSON object string).
std::string SerializeTools(const std::vector<ToolDef>& tools) {
  if (tools.empty()) return "";
  std::string t = "[";
  for (size_t i = 0; i < tools.size(); ++i) {
    if (i) t += ",";
    t += "{\"name\":" + JsonString(tools[i].name) +
         ",\"description\":" + JsonString(tools[i].description) +
         ",\"input_schema\":" + tools[i].input_schema + "}";
  }
  t += "]";
  return t;
}

std::string BuildRequestBody(const std::string& model, int max_tokens,
                             const std::string& system,
                             const std::string& messages_inner,
                             const std::string& tools_inner) {
  std::string body = "{";
  body += "\"model\":" + JsonString(model) + ",";
  body += "\"max_tokens\":" + std::to_string(max_tokens) + ",";
  body += "\"thinking\":{\"type\":\"adaptive\"},";
  if (!system.empty()) body += "\"system\":" + JsonString(system) + ",";
  if (!tools_inner.empty()) body += "\"tools\":" + tools_inner + ",";
  body += "\"messages\":[" + messages_inner + "]}";
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
// Platform transport + the tool-use loop. Kept at the bottom and isolated so
// <windows.h> never bleeds into the rest of the studio sources.
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

bool HttpsPost(const std::string& headers, const std::string& body,
               int& status, std::string& response, std::string& err) {
  status = 0;
  response.clear();
  err.clear();

  HINTERNET session = WinHttpOpen(L"MuJoCoStudio/1.0",
                                  WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                                  WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS,
                                  0);
  if (!session) { err = "WinHttpOpen failed"; return false; }
  // Generous timeouts (ms): resolve, connect, send, receive. Claude can take a
  // while on complex prompts (adaptive thinking + a long tool-use program), so
  // allow several minutes for the response rather than WinHTTP's 30s default.
  WinHttpSetTimeouts(session, 60000, 60000, 300000, 300000);
  HINTERNET connect = WinHttpConnect(
      session, L"api.anthropic.com", INTERNET_DEFAULT_HTTPS_PORT, 0);
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
                               const std::vector<LlmMessage>& messages,
                               const std::vector<ToolDef>& tools,
                               const ToolExecutor& exec) {
  LlmResult r;
  if (api_key_.empty()) {
    r.error = "ANTHROPIC_API_KEY is not set.";
    return r;
  }

  const std::string headers = "x-api-key: " + api_key_ + "\r\n" +
                              "anthropic-version: 2023-06-01\r\n" +
                              "content-type: application/json";
  const std::string convo = SerializeMessages(messages);
  const std::string tools_inner = SerializeTools(tools);
  std::string extra;  // assistant/tool_result turns appended across the loop.

  // Opt-in transcript: dumps each assistant turn (text + the tool_use ops it
  // emitted) and each tool_result, so the whole conversation can be inspected.
  const bool verbose = std::getenv("MUJOCO_STUDIO_LLM_VERBOSE") != nullptr;
  auto vtrunc = [](const std::string& s, size_t n) {
    return s.size() > n ? s.substr(0, n) + " ...(truncated)" : s;
  };

  for (int iter = 0; iter < kMaxToolIterations; ++iter) {
    const std::string body =
        BuildRequestBody(model_, max_tokens_, system, convo + extra, tools_inner);

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

    if (verbose) {
      const std::string atext = ExtractAssistantText(response);
      if (!atext.empty()) {
        std::fprintf(stderr, "\n===== assistant (turn %d) =====\n%s\n", iter,
                     atext.c_str());
      }
    }

    std::vector<ToolUse> calls = ExtractToolUseBlocks(response);
    if (calls.empty()) {
      r.text = ExtractAssistantText(response);
      if (r.text.empty()) {
        r.error = "Empty response from Claude.";
        return r;
      }
      if (verbose) {
        std::fprintf(stderr, "\n===== final assistant reply =====\n%s\n",
                     r.text.c_str());
      }
      r.ok = true;
      return r;
    }

    // Echo the assistant turn (text + thinking + tool_use blocks) verbatim,
    // then answer each tool_use with a tool_result, and loop.
    const std::string raw_content = ExtractRawContentArray(response);
    if (raw_content.empty()) {
      r.error = "Could not parse tool_use content.";
      return r;
    }
    extra += ",{\"role\":\"assistant\",\"content\":" + raw_content + "}";

    std::string results = "[";
    for (size_t k = 0; k < calls.size(); ++k) {
      if (verbose) {
        std::fprintf(stderr, "\n----- tool_use (turn %d): %s -----\n%s\n", iter,
                     calls[k].name.c_str(), vtrunc(calls[k].input, 2000).c_str());
      }
      std::string out =
          exec ? exec(calls[k].name, calls[k].input) : std::string("(no executor)");
      if (verbose) {
        std::fprintf(stderr, "----- tool_result: %s -----\n%s\n",
                     calls[k].name.c_str(), vtrunc(out, 2000).c_str());
      }
      if (k) results += ",";
      results += "{\"type\":\"tool_result\",\"tool_use_id\":" +
                 JsonString(calls[k].id) + ",\"content\":" + JsonString(out) +
                 "}";
    }
    results += "]";
    extra += ",{\"role\":\"user\",\"content\":" + results + "}";
  }

  r.error = "Tool-use loop did not converge.";
  return r;
}

}  // namespace mujoco::studio

#else  // !_WIN32

namespace mujoco::studio {

LlmResult ClaudeProvider::Send(const std::string& /*system*/,
                               const std::vector<LlmMessage>& /*messages*/,
                               const std::vector<ToolDef>& /*tools*/,
                               const ToolExecutor& /*exec*/) {
  LlmResult r;
  r.error =
      "Claude transport is only implemented for Windows (WinHTTP) in this "
      "build.";
  return r;
}

}  // namespace mujoco::studio

#endif  // _WIN32
