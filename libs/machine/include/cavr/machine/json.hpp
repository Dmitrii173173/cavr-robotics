#pragma once

// Minimal, dependency-free JSON value with a recursive-descent parser and a
// stable serializer. The project deliberately avoids third-party dependencies
// (see the hand-rolled manifest parsing in cavr::replay), so configuration and
// session I/O is built on this small value type. Object members preserve
// insertion order so exported files are stable and diff-friendly.

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace cavr::json {

class Value;
using Array = std::vector<Value>;
using Member = std::pair<std::string, Value>;
using Object = std::vector<Member>;  // ordered

class Value final {
 public:
  using Storage = std::variant<std::nullptr_t, bool, double, std::string, Array, Object>;

  Value() : storage_(nullptr) {}
  Value(std::nullptr_t) : storage_(nullptr) {}
  Value(bool value) : storage_(value) {}
  Value(int value) : storage_(static_cast<double>(value)) {}
  Value(std::int64_t value) : storage_(static_cast<double>(value)) {}
  Value(std::size_t value) : storage_(static_cast<double>(value)) {}
  Value(double value) : storage_(value) {}
  Value(const char* value) : storage_(std::string(value)) {}
  Value(std::string value) : storage_(std::move(value)) {}
  Value(Array value) : storage_(std::move(value)) {}
  Value(Object value) : storage_(std::move(value)) {}

  [[nodiscard]] bool is_object() const { return std::holds_alternative<Object>(storage_); }
  [[nodiscard]] bool is_array() const { return std::holds_alternative<Array>(storage_); }
  [[nodiscard]] bool is_string() const { return std::holds_alternative<std::string>(storage_); }
  [[nodiscard]] bool is_number() const { return std::holds_alternative<double>(storage_); }
  [[nodiscard]] bool is_bool() const { return std::holds_alternative<bool>(storage_); }
  [[nodiscard]] bool is_null() const { return std::holds_alternative<std::nullptr_t>(storage_); }

  [[nodiscard]] const Array& as_array() const { return std::get<Array>(storage_); }
  [[nodiscard]] const Object& as_object() const { return std::get<Object>(storage_); }

  [[nodiscard]] std::string as_string(std::string_view fallback = {}) const {
    return is_string() ? std::get<std::string>(storage_) : std::string(fallback);
  }
  [[nodiscard]] double as_number(double fallback = 0.0) const {
    return is_number() ? std::get<double>(storage_) : fallback;
  }
  [[nodiscard]] std::int64_t as_int(std::int64_t fallback = 0) const {
    return is_number() ? static_cast<std::int64_t>(std::get<double>(storage_)) : fallback;
  }
  [[nodiscard]] bool as_bool(bool fallback = false) const {
    return is_bool() ? std::get<bool>(storage_) : fallback;
  }

  // Object lookup. Returns nullptr when missing or not an object.
  [[nodiscard]] const Value* find(std::string_view key) const {
    if (!is_object()) return nullptr;
    for (const auto& [name, value] : std::get<Object>(storage_)) {
      if (name == key) return &value;
    }
    return nullptr;
  }
  [[nodiscard]] const Value& at(std::string_view key) const {
    static const Value kNull;
    const Value* found = find(key);
    return found ? *found : kNull;
  }

  void set(std::string key, Value value) {
    if (!is_object()) storage_ = Object{};
    auto& object = std::get<Object>(storage_);
    for (auto& member : object) {
      if (member.first == key) {
        member.second = std::move(value);
        return;
      }
    }
    object.emplace_back(std::move(key), std::move(value));
  }

  [[nodiscard]] std::string dump(int indent = 2) const {
    std::ostringstream out;
    write(out, indent, 0);
    return out.str();
  }

 private:
  void write(std::ostringstream& out, int indent, int depth) const;
  Storage storage_;
};

namespace detail {

inline void write_string(std::ostringstream& out, const std::string& text) {
  out << '"';
  for (const char c : text) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << c; break;
    }
  }
  out << '"';
}

inline void pad(std::ostringstream& out, int indent, int depth) {
  if (indent > 0) {
    out << '\n';
    for (int i = 0; i < indent * depth; ++i) out << ' ';
  }
}

}  // namespace detail

inline void Value::write(std::ostringstream& out, int indent, int depth) const {
  if (is_null()) {
    out << "null";
  } else if (is_bool()) {
    out << (std::get<bool>(storage_) ? "true" : "false");
  } else if (is_number()) {
    const double number = std::get<double>(storage_);
    if (std::floor(number) == number && std::abs(number) < 1e15) {
      out << static_cast<std::int64_t>(number);
    } else {
      std::ostringstream tmp;
      tmp.precision(10);
      tmp << number;
      out << tmp.str();
    }
  } else if (is_string()) {
    detail::write_string(out, std::get<std::string>(storage_));
  } else if (is_array()) {
    const auto& array = std::get<Array>(storage_);
    if (array.empty()) { out << "[]"; return; }
    out << '[';
    for (std::size_t i = 0; i < array.size(); ++i) {
      detail::pad(out, indent, depth + 1);
      array[i].write(out, indent, depth + 1);
      if (i + 1 < array.size()) out << ',';
    }
    detail::pad(out, indent, depth);
    out << ']';
  } else {
    const auto& object = std::get<Object>(storage_);
    if (object.empty()) { out << "{}"; return; }
    out << '{';
    for (std::size_t i = 0; i < object.size(); ++i) {
      detail::pad(out, indent, depth + 1);
      detail::write_string(out, object[i].first);
      out << (indent > 0 ? ": " : ":");
      object[i].second.write(out, indent, depth + 1);
      if (i + 1 < object.size()) out << ',';
    }
    detail::pad(out, indent, depth);
    out << '}';
  }
}

// ----------------------------------------------------------------- parser
class Parser final {
 public:
  explicit Parser(std::string_view text) : text_(text) {}

  [[nodiscard]] std::optional<Value> parse(std::string& error) {
    skip_ws();
    Value value;
    if (!parse_value(value, error)) return std::nullopt;
    skip_ws();
    if (pos_ != text_.size()) {
      error = "trailing characters after JSON value";
      return std::nullopt;
    }
    return value;
  }

 private:
  void skip_ws() {
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++pos_;
      else break;
    }
  }

  bool parse_value(Value& out, std::string& error) {
    skip_ws();
    if (pos_ >= text_.size()) { error = "unexpected end of input"; return false; }
    const char c = text_[pos_];
    switch (c) {
      case '{': return parse_object(out, error);
      case '[': return parse_array(out, error);
      case '"': { std::string s; if (!parse_string(s, error)) return false; out = Value(std::move(s)); return true; }
      case 't': case 'f': return parse_bool(out, error);
      case 'n': return parse_null(out, error);
      default: return parse_number(out, error);
    }
  }

  bool parse_object(Value& out, std::string& error) {
    Object object;
    ++pos_;  // {
    skip_ws();
    if (pos_ < text_.size() && text_[pos_] == '}') { ++pos_; out = Value(std::move(object)); return true; }
    while (true) {
      skip_ws();
      std::string key;
      if (!parse_string(key, error)) return false;
      skip_ws();
      if (pos_ >= text_.size() || text_[pos_] != ':') { error = "expected ':' in object"; return false; }
      ++pos_;
      Value value;
      if (!parse_value(value, error)) return false;
      object.emplace_back(std::move(key), std::move(value));
      skip_ws();
      if (pos_ >= text_.size()) { error = "unterminated object"; return false; }
      if (text_[pos_] == ',') { ++pos_; continue; }
      if (text_[pos_] == '}') { ++pos_; break; }
      error = "expected ',' or '}' in object";
      return false;
    }
    out = Value(std::move(object));
    return true;
  }

  bool parse_array(Value& out, std::string& error) {
    Array array;
    ++pos_;  // [
    skip_ws();
    if (pos_ < text_.size() && text_[pos_] == ']') { ++pos_; out = Value(std::move(array)); return true; }
    while (true) {
      Value value;
      if (!parse_value(value, error)) return false;
      array.push_back(std::move(value));
      skip_ws();
      if (pos_ >= text_.size()) { error = "unterminated array"; return false; }
      if (text_[pos_] == ',') { ++pos_; continue; }
      if (text_[pos_] == ']') { ++pos_; break; }
      error = "expected ',' or ']' in array";
      return false;
    }
    out = Value(std::move(array));
    return true;
  }

  bool parse_string(std::string& out, std::string& error) {
    if (pos_ >= text_.size() || text_[pos_] != '"') { error = "expected string"; return false; }
    ++pos_;
    std::string result;
    while (pos_ < text_.size()) {
      const char c = text_[pos_++];
      if (c == '"') { out = std::move(result); return true; }
      if (c == '\\') {
        if (pos_ >= text_.size()) break;
        const char esc = text_[pos_++];
        switch (esc) {
          case '"': result += '"'; break;
          case '\\': result += '\\'; break;
          case '/': result += '/'; break;
          case 'n': result += '\n'; break;
          case 'r': result += '\r'; break;
          case 't': result += '\t'; break;
          default: result += esc; break;
        }
      } else {
        result += c;
      }
    }
    error = "unterminated string";
    return false;
  }

  bool parse_number(Value& out, std::string& error) {
    const std::size_t start = pos_;
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') ++pos_;
      else break;
    }
    if (pos_ == start) { error = "invalid number"; return false; }
    const std::string token(text_.substr(start, pos_ - start));
    try {
      out = Value(std::stod(token));
    } catch (...) {
      error = "invalid number: " + token;
      return false;
    }
    return true;
  }

  bool parse_bool(Value& out, std::string& error) {
    if (text_.compare(pos_, 4, "true") == 0) { pos_ += 4; out = Value(true); return true; }
    if (text_.compare(pos_, 5, "false") == 0) { pos_ += 5; out = Value(false); return true; }
    error = "invalid literal";
    return false;
  }

  bool parse_null(Value& out, std::string& error) {
    if (text_.compare(pos_, 4, "null") == 0) { pos_ += 4; out = Value(nullptr); return true; }
    error = "invalid literal";
    return false;
  }

  std::string_view text_;
  std::size_t pos_{0};
};

[[nodiscard]] inline std::optional<Value> parse(std::string_view text, std::string& error) {
  return Parser(text).parse(error);
}

}  // namespace cavr::json
