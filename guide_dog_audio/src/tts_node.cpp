// Speaks text through the Go2's own speaker.
//
// The Jetson has no speaker, so text is turned into a WAV here and played by
// the Go2 audio hub (/api/audiohub/*, the DDS API the Unitree app uses). The
// voice is Google TTS (engine "gtts", much clearer than espeak on the robot's
// speaker; needs internet and sends the text to Google) with espeak-ng as the
// offline fallback, or espeak-ng only (engine "espeak"). Each phrase is
// uploaded once as a WAV named after a hash of its text and voice settings;
// later requests only send "play" for that file, so a repeated phrase starts at
// once and needs no network. Uploaded phrases stay on the robot across restarts.
//
// The Go2 only plays a clip in full while a WebRTC client is connected; without
// one it stops after ~0.3 s. go2_rtc_keepalive (same package, started by the
// launch file) holds that session open and keeps a status file fresh while it
// is; each clip waits for that file (rtc_status_file, up to rtc_wait_sec).
//
// This node is C++ because on Foxy rclpy crashes ("free(): invalid pointer")
// decoding the hub's file-list reply (~100 KB from the robot). rclcpp decodes it.
//
//   /speak     guide_dog_interfaces/srv/Speak  returns once the phrase has played
//   /announce  std_msgs/String  fire-and-forget status lines; while the node is
//              speaking only the newest one is kept

#include <openssl/evp.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <iterator>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "guide_dog_interfaces/srv/speak.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_age.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

// Foxy's topic statistics assume a field named "header" has a stamp. The
// unitree_api header has none, so subscribing to Response needs this to compile.
namespace libstatistics_collector
{
namespace topic_statistics_collector
{
template<>
struct TimeStamp<unitree_api::msg::Response, void>
{
  static std::pair<bool, int64_t> value(const unitree_api::msg::Response &) {return {false, 0};}
};
}  // namespace topic_statistics_collector
}  // namespace libstatistics_collector

// espeak-ng's C API (speak_lib.h). Only libespeak-ng.so.1 is installed on the
// Jetson, without dev headers, so the few calls used are declared here.
extern "C" {
struct espeak_EVENT;
typedef int (t_espeak_callback)(short *, int, espeak_EVENT *);
int espeak_Initialize(int output, int buflength, const char * path, int options);
void espeak_SetSynthCallback(t_espeak_callback * callback);
int espeak_SetVoiceByName(const char * name);
int espeak_SetParameter(int parameter, int value, int relative);
int espeak_Synth(
  const void * text, size_t size, unsigned int position, int position_type,
  unsigned int end_position, unsigned int flags, unsigned int * unique_identifier,
  void * user_data);
}

extern char ** environ;

namespace
{
using namespace std::chrono_literals;
using nlohmann::json;

constexpr int kEspeakOutputSynchronous = 2;     // AUDIO_OUTPUT_SYNCHRONOUS
constexpr int kEspeakRate = 1;                  // espeakRATE
constexpr int kEspeakPosCharacter = 1;          // POS_CHARACTER
constexpr unsigned int kEspeakCharsUtf8 = 1;    // espeakCHARS_UTF8

// Go2 audio hub API ids
constexpr int64_t kGetAudioList = 1001;
constexpr int64_t kSelectStartPlay = 1002;
constexpr int64_t kUploadAudioFile = 2001;

constexpr size_t kUploadBlockChars = 16384;     // base64 characters per upload block
constexpr auto kReplyTimeout = 3s;
// The robot answers slowly when it stores a file or lists ~1000 of them
constexpr auto kSlowReplyTimeout = 15s;
constexpr double kTailSilenceSec = 0.2;         // keeps the last word from being clipped
// The hub may finish another request (an upload, a delete) before it starts playing
constexpr auto kPlaybackStartTimeout = 5s;
constexpr auto kMaxClipDuration = 60s;
// go2_rtc_keepalive refreshes its status file every 2 s while the session is open
constexpr std::time_t kRtcStatusMaxAgeSec = 5;

// espeak-ng hands samples to a plain C callback; synthesize() points this at its buffer
std::vector<int16_t> * g_synth_buffer = nullptr;

int on_synth(short * wav, int num_samples, espeak_EVENT *)
{
  if (wav != nullptr && num_samples > 0 && g_synth_buffer != nullptr) {
    g_synth_buffer->insert(g_synth_buffer->end(), wav, wav + num_samples);
  }
  return 0;
}

// Plain 44-byte header, as Python's wave module writes: the format verified to
// play on the Go2.
std::string make_wav(const std::vector<int16_t> & pcm, uint32_t rate)
{
  std::string out;
  auto put16 = [&out](uint16_t v) {
      out.push_back(static_cast<char>(v & 0xff));
      out.push_back(static_cast<char>(v >> 8));
    };
  auto put32 = [&put16](uint32_t v) {
      put16(static_cast<uint16_t>(v & 0xffff));
      put16(static_cast<uint16_t>(v >> 16));
    };
  const uint32_t data_bytes = static_cast<uint32_t>(pcm.size() * 2);
  out.reserve(44 + data_bytes);
  out += "RIFF";
  put32(36 + data_bytes);
  out += "WAVEfmt ";
  put32(16);         // fmt chunk size
  put16(1);          // PCM
  put16(1);          // mono
  put32(rate);
  put32(rate * 2);   // byte rate
  put16(2);          // block align
  put16(16);         // bits per sample
  out += "data";
  put32(data_bytes);
  for (int16_t s : pcm) {
    put16(static_cast<uint16_t>(s));
  }
  return out;
}

std::string base64(const std::string & in)
{
  static const char * table =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((in.size() + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 2 < in.size(); i += 3) {
    const uint32_t n = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8) | uint8_t(in[i + 2]);
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += table[(n >> 6) & 63];
    out += table[n & 63];
  }
  if (i < in.size()) {
    uint32_t n = uint8_t(in[i]) << 16;
    if (i + 1 < in.size()) {
      n |= uint8_t(in[i + 1]) << 8;
    }
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += (i + 1 < in.size()) ? table[(n >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

// The upload API requires the file's MD5 as a transfer checksum (not used for security)
std::string md5_hex(const std::string & data)
{
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int len = 0;
  EVP_Digest(data.data(), data.size(), digest, &len, EVP_md5(), nullptr);
  std::string hex;
  char buf[3];
  for (unsigned int i = 0; i < len; ++i) {
    std::snprintf(buf, sizeof(buf), "%02x", digest[i]);
    hex += buf;
  }
  return hex;
}

// Stable short id for a phrase (FNV-1a 64), used as the file name on the robot
std::string phrase_id(const std::string & key)
{
  uint64_t h = 1469598103934665603ULL;
  for (unsigned char c : key) {
    h ^= c;
    h *= 1099511628211ULL;
  }
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
  return buf;
}

// Runs a program with the given arguments (no shell, so text from callers
// cannot inject commands). Returns true if it exited 0 within the timeout.
bool run_program(std::vector<std::string> args, std::chrono::milliseconds timeout)
{
  std::vector<char *> argv;
  for (auto & arg : args) {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);
  pid_t pid;
  if (posix_spawn(&pid, argv[0], nullptr, nullptr, argv.data(), environ) != 0) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int status = 0;
  while (waitpid(pid, &status, WNOHANG) == 0) {
    if (std::chrono::steady_clock::now() > deadline) {
      kill(pid, SIGKILL);
      waitpid(pid, &status, 0);
      return false;
    }
    std::this_thread::sleep_for(50ms);
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

// <workspace>/venv/bin/python, made by scripts/setup_venv.sh. The package is
// installed at <workspace>/install/guide_dog_audio.
std::string workspace_venv_python()
{
  try {
    std::string ws = ament_index_cpp::get_package_prefix("guide_dog_audio");
    for (int i = 0; i < 2; ++i) {
      ws = ws.substr(0, ws.find_last_of('/'));
    }
    return ws + "/venv/bin/python";
  } catch (const std::exception &) {
    return "";
  }
}
}  // namespace

class TtsNode : public rclcpp::Node
{
public:
  TtsNode()
  : Node("tts_node")
  {
    engine_ = declare_parameter<std::string>("engine", "gtts");
    gtts_lang_ = declare_parameter<std::string>("gtts_lang", "en");
    gtts_python_ = declare_parameter<std::string>("gtts_python", workspace_venv_python());
    gtts_timeout_ = std::chrono::milliseconds(
      static_cast<int64_t>(declare_parameter<double>("gtts_timeout_sec", 20.0) * 1000));
    voice_ = declare_parameter<std::string>("voice", "en-us");
    rate_wpm_ = declare_parameter<int>("rate_wpm", 160);
    name_prefix_ = declare_parameter<std::string>("file_name_prefix", "guide_dog_tts_");
    // Must match go2_rtc_keepalive's GO2_RTC_STATUS_FILE; rtc_wait_sec 0 disables the wait
    rtc_status_file_ = declare_parameter<std::string>(
      "rtc_status_file", std::string(std::getenv("HOME") ? std::getenv("HOME") : "") +
      "/.ros/go2_rtc_keepalive.up");
    rtc_wait_ = std::chrono::milliseconds(
      static_cast<int64_t>(declare_parameter<double>("rtc_wait_sec", 10.0) * 1000));

    if (engine_ == "gtts") {
      try {
        gtts_script_ = ament_index_cpp::get_package_prefix("guide_dog_audio") +
          "/lib/guide_dog_audio/gtts_synth.py";
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "gtts_synth.py not found (%s)", e.what());
      }
      if (gtts_script_.empty() || access(gtts_python_.c_str(), X_OK) != 0) {
        RCLCPP_WARN(
          get_logger(), "Google TTS needs %s (a Python with gTTS); using espeak-ng only",
          gtts_python_.c_str());
        engine_ = "espeak";
      }
    } else if (engine_ != "espeak") {
      RCLCPP_WARN(get_logger(), "Unknown engine '%s'; using espeak-ng", engine_.c_str());
      engine_ = "espeak";
    }

    sample_rate_ = espeak_Initialize(kEspeakOutputSynchronous, 0, nullptr, 0);
    if (sample_rate_ <= 0) {
      throw std::runtime_error("espeak-ng failed to initialise");
    }
    espeak_SetSynthCallback(on_synth);
    if (espeak_SetVoiceByName(voice_.c_str()) != 0) {
      RCLCPP_WARN(get_logger(), "espeak-ng voice '%s' not found; using the default", voice_.c_str());
    }
    espeak_SetParameter(kEspeakRate, rate_wpm_, 0);

    next_request_id_ = std::mt19937_64(std::random_device{}())() >> 2;

    // Speech blocks for the length of the clip, while hub replies must keep
    // arriving to finish an upload, so the two run in separate groups.
    speech_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    hub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    hub_request_pub_ = create_publisher<unitree_api::msg::Request>("/api/audiohub/request", 10);
    rclcpp::SubscriptionOptions hub_options;
    hub_options.callback_group = hub_group_;
    hub_response_sub_ = create_subscription<unitree_api::msg::Response>(
      "/api/audiohub/response", 10,
      std::bind(&TtsNode::on_hub_response, this, std::placeholders::_1), hub_options);
    player_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/audiohub/player/state", 10,
      std::bind(&TtsNode::on_player_state, this, std::placeholders::_1), hub_options);

    speak_srv_ = create_service<guide_dog_interfaces::srv::Speak>(
      "/speak",
      [this](
        const std::shared_ptr<guide_dog_interfaces::srv::Speak::Request> request,
        std::shared_ptr<guide_dog_interfaces::srv::Speak::Response> response) {
        response->success = speak(request->text);
      },
      rmw_qos_profile_services_default, speech_group_);

    // Depth 1: announcements that arrive while speaking collapse to the newest
    rclcpp::SubscriptionOptions announce_options;
    announce_options.callback_group = speech_group_;
    announce_sub_ = create_subscription<std_msgs::msg::String>(
      "/announce", rclcpp::QoS(1),
      [this](std_msgs::msg::String::SharedPtr msg) {speak(msg->data);},
      announce_options);

    // Load the robot's file list before the first request so it plays at once
    startup_timer_ = create_wall_timer(
      100ms, [this]() {
        startup_timer_->cancel();
        for (int attempt = 0; attempt < 3 && !cache_loaded_; ++attempt) {
          cache_loaded_ = refresh_cache();
        }
        RCLCPP_INFO(
          get_logger(),
          "TTS ready on '/speak' and '/announce' (voice: %s, %zu phrases already on the robot)",
          engine_ == "gtts" ? "Google TTS, espeak-ng fallback" : "espeak-ng", cache_.size());
      }, speech_group_);
  }

private:
  struct Reply
  {
    int32_t code;
    std::string data;
  };

  void on_hub_response(const unitree_api::msg::Response::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(hub_mutex_);
    const int64_t id = msg->header.identity.id;
    if (pending_.count(id) == 0) {
      return;  // a reply to another client of the hub
    }
    replies_[id] = Reply{msg->header.status.code, msg->data};
    hub_cv_.notify_all();
  }

  void on_player_state(const std_msgs::msg::String::SharedPtr msg)
  {
    try {
      const json state = json::parse(msg->data);
      std::lock_guard<std::mutex> lock(player_mutex_);
      player_playing_ = state.value("is_playing", false);
      player_file_id_ = state.value("current_audio_unique_id", std::string());
    } catch (const json::exception &) {
      return;
    }
    player_cv_.notify_all();
  }

  // Returns once the hub reports the clip has finished. Without a WebRTC
  // session (go2_rtc_keepalive) the robot stops every clip after ~0.3 s, and
  // this returns then too.
  void wait_for_playback(const std::string & file_id)
  {
    std::unique_lock<std::mutex> lock(player_mutex_);
    if (!player_cv_.wait_for(
        lock, kPlaybackStartTimeout,
        [&]() {return player_playing_ && player_file_id_ == file_id;}))
    {
      RCLCPP_WARN(get_logger(), "The Go2 did not report the clip playing");
      return;
    }
    player_cv_.wait_for(
      lock, kMaxClipDuration, [&]() {return !player_playing_ || player_file_id_ != file_id;});
  }

  // Sends one hub request and waits for its reply. Returns the reply data, or
  // nullopt on timeout or a non-zero status.
  std::optional<std::string> call_hub(
    int64_t api_id, const std::string & parameter, std::chrono::milliseconds timeout)
  {
    unitree_api::msg::Request request;
    request.header.identity.api_id = api_id;
    request.parameter = parameter;
    std::unique_lock<std::mutex> lock(hub_mutex_);
    const int64_t id = next_request_id_++;
    request.header.identity.id = id;
    pending_.insert(id);
    lock.unlock();
    hub_request_pub_->publish(request);
    lock.lock();
    const bool answered = hub_cv_.wait_for(lock, timeout, [&]() {return replies_.count(id) > 0;});
    pending_.erase(id);
    if (!answered) {
      RCLCPP_WARN(get_logger(), "Audio hub did not answer api %ld", api_id);
      return std::nullopt;
    }
    Reply reply = std::move(replies_[id]);
    replies_.erase(id);
    if (reply.code != 0) {
      RCLCPP_WARN(get_logger(), "Audio hub api %ld failed with code %d", api_id, reply.code);
      return std::nullopt;
    }
    return reply.data;
  }

  // Rebuilds the phrase name -> file id map from the robot's file list
  bool refresh_cache()
  {
    const auto data = call_hub(kGetAudioList, "{}", kSlowReplyTimeout);
    if (!data) {
      return false;
    }
    try {
      const json list = json::parse(*data);
      cache_.clear();
      for (const auto & item : list.at("audio_list")) {
        const auto name = item.value("CUSTOM_NAME", std::string());
        const auto id = item.value("UNIQUE_ID", std::string());
        if (name.rfind(name_prefix_, 0) == 0 && !id.empty()) {
          cache_[name] = id;
        }
      }
    } catch (const json::exception & e) {
      RCLCPP_ERROR(get_logger(), "Unreadable audio list from the robot: %s", e.what());
      return false;
    }
    return true;
  }

  std::optional<std::string> synthesize_espeak(const std::string & text)
  {
    std::vector<int16_t> pcm;
    g_synth_buffer = &pcm;
    espeak_Synth(
      text.c_str(), text.size() + 1, 0, kEspeakPosCharacter, 0, kEspeakCharsUtf8,
      nullptr, nullptr);
    g_synth_buffer = nullptr;
    if (pcm.empty()) {
      RCLCPP_ERROR(get_logger(), "espeak-ng produced no audio for '%s'", text.c_str());
      return std::nullopt;
    }
    pcm.resize(pcm.size() + static_cast<size_t>(kTailSilenceSec * sample_rate_), 0);
    return make_wav(pcm, static_cast<uint32_t>(sample_rate_));
  }

  // Google TTS through gtts_synth.py; needs network and sends the text to Google
  std::optional<std::string> synthesize_gtts(const std::string & text)
  {
    char path[] = "/tmp/guide_dog_tts_XXXXXX";
    const int fd = mkstemp(path);
    if (fd < 0) {
      return std::nullopt;
    }
    close(fd);
    std::optional<std::string> wav;
    if (run_program({gtts_python_, gtts_script_, gtts_lang_, path, text}, gtts_timeout_)) {
      std::ifstream file(path, std::ios::binary);
      wav = std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
      if (wav->size() <= 44) {
        wav.reset();
      }
    }
    unlink(path);
    return wav;
  }

  // Uploads a WAV under the given name and returns its file id on the robot
  std::string upload(const std::string & name, const std::string & wav)
  {
    const std::string b64 = base64(wav);
    const std::string md5 = md5_hex(wav);
    const size_t blocks = (b64.size() + kUploadBlockChars - 1) / kUploadBlockChars;
    const int64_t create_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    for (size_t i = 0; i < blocks; ++i) {
      const std::string block = b64.substr(i * kUploadBlockChars, kUploadBlockChars);
      const json parameter = {
        {"file_name", name}, {"file_type", "wav"}, {"file_size", wav.size()},
        {"current_block_index", i + 1}, {"total_block_number", blocks},
        {"block_content", block}, {"current_block_size", block.size()},
        {"file_md5", md5}, {"create_time", create_time}};
      // The robot stores the file when the last block arrives, which takes ~3 s
      const auto timeout = (i + 1 == blocks) ? kSlowReplyTimeout : kReplyTimeout;
      if (!call_hub(kUploadAudioFile, parameter.dump(), timeout)) {
        RCLCPP_ERROR(get_logger(), "Upload of %s failed at block %zu/%zu", name.c_str(), i + 1, blocks);
        return "";
      }
    }
    // The upload reply carries no file id; the list is the only way to get it
    if (!refresh_cache()) {
      return "";
    }
    const auto it = cache_.find(name);
    if (it == cache_.end()) {
      RCLCPP_ERROR(get_logger(), "Uploaded %s but it is not in the robot's file list", name.c_str());
      return "";
    }
    return it->second;
  }

  // True while go2_rtc_keepalive reports an open WebRTC session. A file left
  // behind by a killed keepalive goes stale and stops counting.
  bool rtc_session_up() const
  {
    struct stat st;
    return stat(rtc_status_file_.c_str(), &st) == 0 &&
           std::time(nullptr) - st.st_mtime <= kRtcStatusMaxAgeSec;
  }

  // The session takes ~1 s to open at launch, and up to ~6 s to come back after
  // the Unitree app took it. After one timeout later clips do not wait until the
  // session is seen again, so a keepalive that is not running costs one delay.
  void wait_for_rtc_session()
  {
    if (rtc_wait_ <= 0ms) {
      return;
    }
    if (rtc_session_up()) {
      rtc_wait_timed_out_ = false;
      return;
    }
    if (rtc_wait_timed_out_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for the WebRTC session (go2_rtc_keepalive) before speaking");
    const auto deadline = std::chrono::steady_clock::now() + rtc_wait_;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(100ms);
      if (rtc_session_up()) {
        return;
      }
    }
    rtc_wait_timed_out_ = true;
    RCLCPP_WARN(
      get_logger(), "No WebRTC session after %.0f s (is go2_rtc_keepalive running?); "
      "speaking anyway, so the Go2 will cut the clip short", rtc_wait_.count() / 1000.0);
  }

  bool play(const std::string & file_id)
  {
    wait_for_rtc_session();
    return call_hub(kSelectStartPlay, json{{"unique_id", file_id}}.dump(), kReplyTimeout).has_value();
  }

  // Plays the robot's clip stored under name, creating and uploading it with
  // make_wav first if the robot does not have it
  bool play_phrase(
    const std::string & name, const std::string & text,
    const std::function<std::optional<std::string>()> & make_wav)
  {
    const auto cached = cache_.find(name);
    const bool was_cached = cached != cache_.end();
    std::string file_id = was_cached ? cached->second : "";
    if (!was_cached) {
      const auto wav = make_wav();
      if (!wav) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "First use of this phrase; uploading it to the robot");
      file_id = upload(name, *wav);
    }
    bool started = !file_id.empty() && play(file_id);
    if (!started && was_cached) {
      // The file may have been deleted on the robot since it was listed
      cache_.erase(name);
      const auto wav = make_wav();
      file_id = wav ? upload(name, *wav) : "";
      started = !file_id.empty() && play(file_id);
    }
    if (!started) {
      RCLCPP_ERROR(get_logger(), "Could not play '%s' on the Go2 speaker", text.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Speaking: '%s'", text.c_str());
    wait_for_playback(file_id);
    return true;
  }

  bool speak(const std::string & text)
  {
    if (text.empty()) {
      return true;
    }
    if (!cache_loaded_) {
      cache_loaded_ = refresh_cache();
    }
    if (engine_ == "gtts") {
      const std::string name = name_prefix_ + phrase_id("gtts|" + gtts_lang_ + "|" + text);
      if (play_phrase(name, text, [&]() {return synthesize_gtts(text);})) {
        return true;
      }
      RCLCPP_WARN(get_logger(), "Google TTS unavailable (no internet?); using espeak-ng");
    }
    const std::string name =
      name_prefix_ + phrase_id(voice_ + "|" + std::to_string(rate_wpm_) + "|" + text);
    return play_phrase(name, text, [&]() {return synthesize_espeak(text);});
  }

  std::string engine_;
  std::string gtts_lang_;
  std::string gtts_python_;
  std::string gtts_script_;
  std::chrono::milliseconds gtts_timeout_;
  std::string voice_;
  int rate_wpm_;
  std::string name_prefix_;
  std::string rtc_status_file_;
  std::chrono::milliseconds rtc_wait_;
  int sample_rate_;

  rclcpp::CallbackGroup::SharedPtr speech_group_;
  rclcpp::CallbackGroup::SharedPtr hub_group_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr hub_request_pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr hub_response_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr player_state_sub_;
  rclcpp::Service<guide_dog_interfaces::srv::Speak>::SharedPtr speak_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr announce_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // Hub request/reply matching, shared between the speech and hub threads
  std::mutex hub_mutex_;
  std::condition_variable hub_cv_;
  int64_t next_request_id_;
  std::unordered_set<int64_t> pending_;
  std::unordered_map<int64_t, Reply> replies_;

  // Latest /audiohub/player/state, written by the hub thread
  std::mutex player_mutex_;
  std::condition_variable player_cv_;
  bool player_playing_ = false;
  std::string player_file_id_;

  // Only touched from speech_group_ callbacks
  bool cache_loaded_ = false;
  bool rtc_wait_timed_out_ = false;
  std::unordered_map<std::string, std::string> cache_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TtsNode>();
  // One thread speaks (and blocks); the other delivers audio hub replies
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
