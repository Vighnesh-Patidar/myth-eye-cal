#include "mec/render/websocket_render_server.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mec {
namespace {

// --- SHA-1 (RFC 3174) -----------------------------------------------------
struct Sha1 {
    uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    static uint32_t rol(uint32_t v, int b) { return (v << b) | (v >> (32 - b)); }

    static std::array<uint8_t, 20> hash(const std::string& msg) {
        Sha1 s;
        std::string m = msg;
        const uint64_t bits = static_cast<uint64_t>(m.size()) * 8;
        m.push_back(static_cast<char>(0x80));
        while (m.size() % 64 != 56) m.push_back('\0');
        for (int i = 7; i >= 0; --i)
            m.push_back(static_cast<char>((bits >> (i * 8)) & 0xFF));

        for (size_t chunk = 0; chunk < m.size(); chunk += 64) {
            uint32_t w[80];
            for (int i = 0; i < 16; ++i) {
                w[i] = (static_cast<uint8_t>(m[chunk + i * 4]) << 24) |
                       (static_cast<uint8_t>(m[chunk + i * 4 + 1]) << 16) |
                       (static_cast<uint8_t>(m[chunk + i * 4 + 2]) << 8) |
                       (static_cast<uint8_t>(m[chunk + i * 4 + 3]));
            }
            for (int i = 16; i < 80; ++i)
                w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

            uint32_t a = s.h[0], b = s.h[1], c = s.h[2], d = s.h[3], e = s.h[4];
            for (int i = 0; i < 80; ++i) {
                uint32_t f, k;
                if (i < 20)      { f = (b & c) | (~b & d);            k = 0x5A827999u; }
                else if (i < 40) { f = b ^ c ^ d;                     k = 0x6ED9EBA1u; }
                else if (i < 60) { f = (b & c) | (b & d) | (c & d);   k = 0x8F1BBCDCu; }
                else             { f = b ^ c ^ d;                     k = 0xCA62C1D6u; }
                const uint32_t tmp = rol(a, 5) + f + e + k + w[i];
                e = d; d = c; c = rol(b, 30); b = a; a = tmp;
            }
            s.h[0] += a; s.h[1] += b; s.h[2] += c; s.h[3] += d; s.h[4] += e;
        }
        std::array<uint8_t, 20> out{};
        for (int i = 0; i < 5; ++i) {
            out[i * 4]     = (s.h[i] >> 24) & 0xFF;
            out[i * 4 + 1] = (s.h[i] >> 16) & 0xFF;
            out[i * 4 + 2] = (s.h[i] >> 8) & 0xFF;
            out[i * 4 + 3] = (s.h[i]) & 0xFF;
        }
        return out;
    }
};

std::string base64(const uint8_t* data, size_t n) {
    static const char* t =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((n + 2) / 3 * 4);
    for (size_t i = 0; i < n; i += 3) {
        const uint32_t b0 = data[i];
        const uint32_t b1 = (i + 1 < n) ? data[i + 1] : 0;
        const uint32_t b2 = (i + 2 < n) ? data[i + 2] : 0;
        const uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
        out.push_back(t[(triple >> 18) & 0x3F]);
        out.push_back(t[(triple >> 12) & 0x3F]);
        out.push_back((i + 1 < n) ? t[(triple >> 6) & 0x3F] : '=');
        out.push_back((i + 2 < n) ? t[triple & 0x3F] : '=');
    }
    return out;
}

bool set_nonblocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

// Case-insensitive search for an HTTP header value.
std::string header_value(const std::string& req, const std::string& name) {
    std::string lower = req;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char ch) { return std::tolower(ch); });
    std::string key = name;
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char ch) { return std::tolower(ch); });
    size_t pos = lower.find(key + ":");
    if (pos == std::string::npos) return {};
    pos += key.size() + 1;
    size_t end = req.find("\r\n", pos);
    std::string v = req.substr(pos, end - pos);
    size_t a = v.find_first_not_of(" \t");
    size_t b = v.find_last_not_of(" \t");
    return (a == std::string::npos) ? std::string{} : v.substr(a, b - a + 1);
}

constexpr const char* kWsMagic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

} // namespace

// --- static helpers -------------------------------------------------------

std::string WebSocketRenderServer::compute_accept_key(const std::string& key) {
    const auto digest = Sha1::hash(key + kWsMagic);
    return base64(digest.data(), digest.size());
}

std::string WebSocketRenderServer::encode_text_frame(const std::string& payload) {
    std::string f;
    f.push_back(static_cast<char>(0x81)); // FIN + opcode 0x1 (text)
    const size_t n = payload.size();
    if (n < 126) {
        f.push_back(static_cast<char>(n));
    } else if (n <= 0xFFFF) {
        f.push_back(static_cast<char>(126));
        f.push_back(static_cast<char>((n >> 8) & 0xFF));
        f.push_back(static_cast<char>(n & 0xFF));
    } else {
        f.push_back(static_cast<char>(127));
        for (int i = 7; i >= 0; --i)
            f.push_back(static_cast<char>((static_cast<uint64_t>(n) >> (i * 8)) & 0xFF));
    }
    f += payload;
    return f;
}

// --- lifecycle ------------------------------------------------------------

WebSocketRenderServer::~WebSocketRenderServer() { stop(); }

bool WebSocketRenderServer::start(uint16_t port, const char* bind_addr) {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;

    int yes = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, bind_addr, &addr.sin_addr) != 1) { stop(); return false; }

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        stop(); return false;
    }
    if (::listen(listen_fd_, 16) != 0) { stop(); return false; }

    socklen_t len = sizeof(addr);
    if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len) == 0)
        port_ = ntohs(addr.sin_port);

    set_nonblocking(listen_fd_);
    return true;
}

void WebSocketRenderServer::stop() {
    for (auto& c : clients_)
        if (c.fd >= 0) ::close(c.fd);
    clients_.clear();
    if (listen_fd_ >= 0) ::close(listen_fd_);
    listen_fd_ = -1;
    port_ = 0;
}

// --- event loop -----------------------------------------------------------

void WebSocketRenderServer::accept_new() {
    for (;;) {
        const int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd < 0) break; // EAGAIN: no more pending
        set_nonblocking(fd);
        clients_.push_back(Client{fd, false, {}});
    }
}

bool WebSocketRenderServer::do_handshake(Client& c) {
    if (c.inbuf.find("\r\n\r\n") == std::string::npos) return true; // wait for more
    const std::string key = header_value(c.inbuf, "Sec-WebSocket-Key");
    if (key.empty()) return false; // not a valid WS upgrade
    const std::string resp =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + compute_accept_key(key) + "\r\n\r\n";
    if (::send(c.fd, resp.data(), resp.size(), MSG_NOSIGNAL) < 0) return false;
    c.handshaked = true;
    c.inbuf.clear();
    return true;
}

void WebSocketRenderServer::consume_frames(Client& c, bool& drop) {
    // Parse complete client->server frames; we only care about close/ping.
    size_t off = 0;
    const std::string& b = c.inbuf;
    for (;;) {
        if (b.size() - off < 2) break;
        const uint8_t b0 = static_cast<uint8_t>(b[off]);
        const uint8_t b1 = static_cast<uint8_t>(b[off + 1]);
        const uint8_t opcode = b0 & 0x0F;
        const bool masked = (b1 & 0x80) != 0;
        uint64_t len = b1 & 0x7F;
        size_t hdr = 2;
        if (len == 126) {
            if (b.size() - off < 4) break;
            len = (static_cast<uint8_t>(b[off + 2]) << 8) | static_cast<uint8_t>(b[off + 3]);
            hdr = 4;
        } else if (len == 127) {
            if (b.size() - off < 10) break;
            len = 0;
            for (int i = 0; i < 8; ++i)
                len = (len << 8) | static_cast<uint8_t>(b[off + 2 + i]);
            hdr = 10;
        }
        const size_t mask_len = masked ? 4 : 0;
        if (b.size() - off < hdr + mask_len + len) break; // incomplete frame

        if (opcode == 0x8) { drop = true; break; }      // close
        // 0x9 ping / 0xA pong / 0x1 text from client: ignored for the render
        // path (clients only receive). Advance past the frame.
        off += hdr + mask_len + static_cast<size_t>(len);
    }
    if (off > 0) c.inbuf.erase(0, off);
}

bool WebSocketRenderServer::service_client(Client& c) {
    char buf[4096];
    for (;;) {
        const ssize_t n = ::recv(c.fd, buf, sizeof(buf), 0);
        if (n > 0) {
            c.inbuf.append(buf, static_cast<size_t>(n));
            if (c.inbuf.size() > (1u << 20)) return false; // runaway guard
            continue;
        }
        if (n == 0) return false; // peer closed
        break;                    // EAGAIN
    }
    if (!c.handshaked) return do_handshake(c);
    bool drop = false;
    consume_frames(c, drop);
    return !drop;
}

void WebSocketRenderServer::poll_events(int timeout_ms) {
    if (listen_fd_ < 0) return;

    std::vector<pollfd> pfds;
    pfds.reserve(clients_.size() + 1);
    pfds.push_back(pollfd{listen_fd_, POLLIN, 0});
    for (auto& c : clients_) pfds.push_back(pollfd{c.fd, POLLIN, 0});

    if (::poll(pfds.data(), pfds.size(), timeout_ms) <= 0) return;

    if (pfds[0].revents & POLLIN) accept_new();

    for (size_t i = 0; i < clients_.size(); ++i) {
        // pfds[i+1] corresponds to clients_[i] (order preserved; accept_new
        // only appends, so indices < clients_.size() are still valid).
        if (i + 1 >= pfds.size()) break;
        const short re = pfds[i + 1].revents;
        if (re & (POLLIN | POLLHUP | POLLERR)) {
            if (!service_client(clients_[i])) {
                ::close(clients_[i].fd);
                clients_[i].fd = -1;
            }
        }
    }
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
                                  [](const Client& c) { return c.fd < 0; }),
                   clients_.end());
}

int WebSocketRenderServer::broadcast(const std::string& msg) {
    const std::string frame = encode_text_frame(msg);
    int sent = 0;
    for (auto& c : clients_) {
        if (!c.handshaked) continue;
        size_t off = 0;
        bool ok = true;
        while (off < frame.size()) {
            const ssize_t n =
                ::send(c.fd, frame.data() + off, frame.size() - off, MSG_NOSIGNAL);
            if (n > 0) { off += static_cast<size_t>(n); continue; }
            ok = false; // EAGAIN on a full buffer or error: drop this frame
            break;
        }
        if (ok) ++sent;
        else { ::close(c.fd); c.fd = -1; }
    }
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
                                  [](const Client& c) { return c.fd < 0; }),
                   clients_.end());
    return sent;
}

size_t WebSocketRenderServer::client_count() const {
    size_t n = 0;
    for (const auto& c : clients_) if (c.handshaked) ++n;
    return n;
}

} // namespace mec
