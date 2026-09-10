# guide_cognition

Decides **what the robot says** when it recognises someone. Publishes that as
plain text; it makes no sound itself.

## Where it sits in the pipeline

```
/camera/color/image_raw
        |
        v
[face_recognition_node]        (guide_perception_extended package)
        |
        |  /recognized_person       JSON, fires on EVERY processed camera frame
        v
[greeting_node]                (this package)
        |
        |  /speech_out              plain text, one greeting per person
        v
[speaker / TTS node]           (not built yet)
```

Keeping the text and the audio in separate nodes means the text-to-speech
engine can be chosen later without touching this package. `/speech_out` is a
plain `std_msgs/String` — whatever reads it aloud is somebody else's problem.

## Setup — the only thing to fill in is your Claude API key

```bash
pip install -r wso2_unitree_quadruped/guide_cognition/requirements.txt
export ANTHROPIC_API_KEY="sk-ant-..."
ros2 run guide_cognition greeting_node
```

Everything else is already set up. If you would rather paste the key than
export it, put it in `api_key` in
[`config/guide_cognition_params.yaml`](config/guide_cognition_params.yaml).

> ⚠️ That config file **is tracked by git**. A key pasted there and committed
> is a leaked key, and Anthropic will disable it on detection. Fine for a
> quick local test — for anything you push, use the environment variable.

**With no key at all the node still runs** — it just uses the built-in
greeting. It never goes silent and never crashes.

## The cooldown — read this before changing anything

`face_recognition_node` publishes **on every processed camera frame**, not
once per person. Someone standing in front of the robot produces a steady
stream of identical messages.

Without a guard, this node would greet them several times a second and fire an
API call for each one. `greet_cooldown_seconds` (default 300 = 5 minutes) is
that guard: after greeting somebody, their name is ignored until it expires.

The record lives in a plain dictionary in memory, so it resets on restart.
That is deliberate — worst case is one extra greeting, and it keeps this
package dependency-free.

> If you later need *persistent* visit history — "this is your 4th visit",
> "I haven't seen you in 3 days" — that is a storage concern and belongs in
> its own node, not bolted onto this one.

## Subscribed topic: `/recognized_person`

`std_msgs/String` containing JSON, from `face_recognition_node`:

```json
{
  "timestamp": 1720521143.82,
  "face_detected": true,
  "name": "randil",
  "confidence": 0.9131,
  "bbox": [220.0, 130.0, 410.0, 380.0]
}
```

`name` is `null` when no face is visible, and when a face does not match
anyone enrolled. Both are ignored.

`confidence` is **not** re-checked here. `face_recognition_node` already
applies its own `similarity_threshold` and nulls the name below it, so a name
arriving at all means it was confident enough. Two thresholds would be two
things to keep in sync.

## Published topic: `/speech_out`

`std_msgs/String`, plain text. One message per greeting:

```
Hello randil. I am Oxy. Follow me.
```

## The robot always says something

Every failure path ends at the built-in greeting:

- `use_llm: false`
- the `anthropic` package not installed
- no API key in `api_key` and none in the environment
- no network, timeout, bad key, rate limit
- Claude returns no usable text

A silent robot standing in front of a visitor is the one outcome a demo cannot
survive, so there is no path through this node that publishes nothing.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/recognized_person` | face results from `face_recognition_node` |
| `output_topic` | `/speech_out` | greeting text for the speaker node |
| `greet_cooldown_seconds` | `300.0` | ignore a person this long after greeting them |
| `use_llm` | `true` | `false` = always use built-in greetings |
| `api_key` | `""` | empty = read `ANTHROPIC_API_KEY` from the environment |
| `model` | `claude-sonnet-5` | which Claude model writes the greeting |
| `max_tokens` | `1000` | reply ceiling (the model also needs thinking room) |
| `timeout_seconds` | `8.0` | give up on the API after this long |
| `robot_name` | `Oxy` | used by the built-in greeting |

## Build and run

```bash
colcon build --packages-select guide_cognition
source install/setup.bash
ros2 run guide_cognition greeting_node
```

## Checking it by hand

```bash
# terminal 1 — short cooldown so you are not waiting 5 minutes
ros2 run guide_cognition greeting_node --ros-args -p greet_cooldown_seconds:=5.0

# terminal 2
ros2 topic echo /speech_out

# terminal 3 — pretend the camera recognised somebody
ros2 topic pub --once /recognized_person std_msgs/String \
  '{data: "{\"name\": \"randil\", \"face_detected\": true, \"confidence\": 0.9}"}'
```

Publish the same message again straight away and **nothing happens** — that is
the cooldown working. Wait 5 seconds and it greets again.

To check the fallback, unset the key and run it again — you should still get a
greeting on `/speech_out`, plus a warning in the log:

```bash
ANTHROPIC_API_KEY= ros2 run guide_cognition greeting_node
```

## Known limitation

The API call blocks inside the subscription callback, so the node pauses for
up to `timeout_seconds`. Incoming frames queue behind it, but the cooldown
discards them anyway, so nothing important is lost. If this node ever takes on
higher-rate work, the call should move to a background thread.

This node does **not** publish `/heartbeat`, so it can never stall the
`guide_actuation` safety watchdog.
