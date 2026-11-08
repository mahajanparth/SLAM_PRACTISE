# Sensor Config Parser

The robot's sensor rig is described in a text file, one sensor per line:
```
name=lidar_front; frame=base_link; rate=10; id=3
```
A malformed line must **not** throw or crash the driver. It returns `std::nullopt`, and the caller logs and skips it.
Parsing must not allocate per token, so work with `std::string_view`.

```cpp
struct SensorConfig { std::string name; std::string frame; double rate_hz; int id; };
```

| Function | Behaviour |
|---|---|
| `std::vector<std::string_view> split(std::string_view s, char delim)` | All tokens between delimiters, **including empty ones**. `split("a;;b", ';')` → `{"a","","b"}`; `split("", ';')` → `{""}` |
| `std::string_view trim(std::string_view s)` | Strip leading and trailing spaces and tabs |
| `std::optional<double> parseDouble(std::string_view s)` | The **entire** trimmed token must be a number (`"10"`, `"-2.5"`, `"1e3"`); otherwise `nullopt` |
| `std::optional<int> parseInt(std::string_view s)` | Same for integers (`"3.5"` → `nullopt`) |
| `std::optional<SensorConfig> parseSensor(std::string_view line)` | See rules below |
| `std::vector<std::string> fastSensors(const std::map<std::string, SensorConfig>& rig, double min_rate)` | Keys of sensors with `rate_hz >= min_rate`, in map order |

**`parseSensor` rules.** Segments are separated by `;`. Segments that are empty after trimming (e.g. a trailing `;`) are ignored. Every other
segment must be `key=value`. Keys and values are trimmed. The keys are `name`, `frame`, `rate`, `id`, in any order, and
each must appear **exactly once**. Return `nullopt` on: a missing `=`, an unknown key, a duplicate key, a missing key, an empty
`name` or `frame`, a non-numeric or non-finite `rate`, `rate <= 0`, or `id` that is not an integer or is `< 0`.

### Think about
- Which of your `string_view`s would dangle if `parseSensor` took `const std::string&` and the caller passed a temporary?
- Why `std::from_chars` rather than `std::stod` / `atof` here?
- `optional` vs exceptions vs error codes in a 1 kHz driver thread?
- How would you report *why* a line failed? (`std::variant<SensorConfig, ParseError>`, `expected` in C++23.)
