// SPDX-License-Identifier: MIT
//
// HTTP/1.1 request head parser (RFC 9112), zero copy: bytes are appended to a
// fixed buffer and the request line and headers are string_views into it.
// Bodies: Content-Length only, and only as large as the buffer left after the head (small API
// requests such as a login); Transfer-Encoding is refused (no chunked bodies, and no request
// smuggling between two length headers), as is a second or non-numeric Content-Length.
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string_view>

namespace rtos::net {

[[nodiscard]] constexpr bool iequals(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        char x = a[i], y = b[i];
        x = (x >= 'A' && x <= 'Z') ? static_cast<char>(x + 32) : x;
        y = (y >= 'A' && y <= 'Z') ? static_cast<char>(y + 32) : y;
        if (x != y) {
            return false;
        }
    }
    return true;
}

/// Case-insensitive search for `token` in a comma-separated header value
/// ("keep-alive, Upgrade" contains "upgrade").
[[nodiscard]] constexpr bool contains_token(std::string_view list, std::string_view token) noexcept {
    while (!list.empty()) {
        const std::size_t comma = list.find(',');
        std::string_view item = list.substr(0, comma);
        while (!item.empty() && (item.front() == ' ' || item.front() == '\t')) {
            item.remove_prefix(1);
        }
        while (!item.empty() && (item.back() == ' ' || item.back() == '\t')) {
            item.remove_suffix(1);
        }
        if (const std::size_t semi = item.find(';'); semi != std::string_view::npos) {
            item = item.substr(0, semi);  // "br;q=1.0"
        }
        if (iequals(item, token)) {
            return true;
        }
        if (comma == std::string_view::npos) {
            break;
        }
        list.remove_prefix(comma + 1);
    }
    return false;
}

template <std::size_t Capacity = 2048, std::size_t MaxHeaders = 24>
class HttpRequestParser {
public:
    enum class State : std::uint8_t { Incomplete, Complete, Error, TooLarge };

    struct Header {
        std::string_view name;
        std::string_view value;
    };

    /// No request bytes buffered (connection idle between requests).
    [[nodiscard]] bool empty() const noexcept { return used_ == 0; }

    /// Space for more request bytes (the caller reads into it, then calls commit()).
    [[nodiscard]] std::span<char> free_space() noexcept { return {buffer_.data() + used_, Capacity - used_}; }

    /// `n` bytes were written into free_space(). Parses once the head is complete.
    State commit(std::size_t n) noexcept {
        used_ += n;
        if (state_ != State::Incomplete) {
            return state_;
        }
        const std::string_view data(buffer_.data(), used_);
        const std::size_t end = data.find("\r\n\r\n");
        if (end == std::string_view::npos) {
            if (used_ == Capacity) {
                state_ = State::TooLarge;
            }
            return state_;
        }
        if (head_size_ == 0) {
            head_size_ = end + 4;
            if (!parse(data.substr(0, end + 2)) || !parse_length()) {
                state_ = State::Error;
                return state_;
            }
            if (body_size_ > Capacity - head_size_) {
                state_ = State::TooLarge;
                return state_;
            }
        }
        if (used_ < head_size_ + body_size_) {
            return state_;  // the body is still coming
        }
        state_ = State::Complete;
        return state_;
    }

    /// Prepare for the next request on a persistent connection (keeps pipelined bytes).
    void next() noexcept {
        const std::size_t consumed = head_size_ + body_size_;
        const std::size_t rest = used_ - consumed;
        for (std::size_t i = 0; i < rest; ++i) {
            buffer_[i] = buffer_[consumed + i];
        }
        used_ = rest;
        state_ = State::Incomplete;
        header_count_ = 0;
        head_size_ = 0;
        body_size_ = 0;
        if (rest != 0) {
            commit(0);
        }
    }

    [[nodiscard]] State state() const noexcept { return state_; }
    [[nodiscard]] std::string_view method() const noexcept { return method_; }
    [[nodiscard]] std::string_view target() const noexcept { return target_; }
    [[nodiscard]] std::string_view version() const noexcept { return version_; }
    /// The request body (Content-Length bytes after the head; empty without one).
    [[nodiscard]] std::string_view body() const noexcept {
        return state_ == State::Complete ? std::string_view(buffer_.data() + head_size_, body_size_) : std::string_view{};
    }

    [[nodiscard]] std::optional<std::string_view> header(std::string_view name) const noexcept {
        for (std::size_t i = 0; i < header_count_; ++i) {
            if (iequals(headers_[i].name, name)) {
                return headers_[i].value;
            }
        }
        return std::nullopt;
    }

    /// HTTP/1.1 default keep-alive unless "Connection: close".
    [[nodiscard]] bool keep_alive() const noexcept {
        const auto c = header("Connection");
        if (version_ == "HTTP/1.0") {
            return c && contains_token(*c, "keep-alive");
        }
        return !(c && contains_token(*c, "close"));
    }

private:
    bool parse(std::string_view head) noexcept {
        std::size_t eol = head.find("\r\n");
        const std::string_view line = head.substr(0, eol);
        const std::size_t sp1 = line.find(' ');
        const std::size_t sp2 = line.rfind(' ');
        if (sp1 == std::string_view::npos || sp2 == sp1) {
            return false;
        }
        method_ = line.substr(0, sp1);
        target_ = line.substr(sp1 + 1, sp2 - sp1 - 1);
        version_ = line.substr(sp2 + 1);
        if (method_.empty() || target_.empty() || target_.front() != '/' ||
            (version_ != "HTTP/1.1" && version_ != "HTTP/1.0")) {
            return false;
        }
        head.remove_prefix(eol + 2);
        while (!head.empty()) {
            eol = head.find("\r\n");
            const std::string_view h = head.substr(0, eol);
            const std::size_t colon = h.find(':');
            if (colon == std::string_view::npos || colon == 0 || header_count_ == MaxHeaders) {
                return false;
            }
            std::string_view value = h.substr(colon + 1);
            while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
                value.remove_prefix(1);
            }
            while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
                value.remove_suffix(1);
            }
            headers_[header_count_++] = {h.substr(0, colon), value};
            head.remove_prefix(eol + 2);
        }
        return true;
    }

    /// Content-Length: digits only, at most once; Transfer-Encoding: refused.
    bool parse_length() noexcept {
        body_size_ = 0;
        bool seen = false;
        for (std::size_t i = 0; i < header_count_; ++i) {
            if (iequals(headers_[i].name, "Transfer-Encoding")) {
                return false;
            }
            if (!iequals(headers_[i].name, "Content-Length")) {
                continue;
            }
            const std::string_view v = headers_[i].value;
            if (seen || v.empty() || v.size() > 6) {
                return false;
            }
            for (char ch : v) {
                if (ch < '0' || ch > '9') {
                    return false;
                }
                body_size_ = body_size_ * 10 + static_cast<std::size_t>(ch - '0');
            }
            seen = true;
        }
        return true;
    }

    std::array<char, Capacity> buffer_{};
    std::size_t used_ = 0;
    std::size_t head_size_ = 0;
    std::size_t body_size_ = 0;
    State state_ = State::Incomplete;
    std::string_view method_, target_, version_;
    std::array<Header, MaxHeaders> headers_{};
    std::size_t header_count_ = 0;
};

}  // namespace rtos::net
