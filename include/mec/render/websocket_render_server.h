#pragma once

// WebSocketRenderServer - minimal, dependency-free RFC 6455 server for the
// render path (ARCHITECTURE.md §6). Broadcasts "pose_frame" text frames to
// connected browsers (viewer/myth-eye-cal-viewer.html).
//
// DEVIATION (§15.5): the doc specifies vendored uWebSockets. For the v0.1
// render path - small JSON frames, a handful of LAN browser clients, server
// -> client broadcast only - a hand-rolled poll() server keeps the project
// dependency-free and testable on Linux, consistent with §11's "no heavy
// deps" ethos. Swap in uWebSockets later if TLS / scale demands it; the
// public interface (start/poll_events/broadcast/stop) is intentionally small.
//
// Single-threaded by design: the owner calls poll_events() then broadcast()
// from its render loop. No internal threads, no locks.

#include <cstdint>
#include <string>
#include <vector>

namespace mec {

class WebSocketRenderServer {
public:
    WebSocketRenderServer() = default;
    ~WebSocketRenderServer();

    WebSocketRenderServer(const WebSocketRenderServer&) = delete;
    WebSocketRenderServer& operator=(const WebSocketRenderServer&) = delete;

    // Bind + listen on `port` (0 = ephemeral; query with port()). Returns
    // false on failure. `bind_addr` defaults to all interfaces so phones on
    // the LAN can connect.
    bool start(uint16_t port, const char* bind_addr = "0.0.0.0");
    void stop();

    // Accept new connections, complete handshakes, and drain client control
    // frames (close/ping). Blocks up to timeout_ms waiting for activity
    // (0 = non-blocking poll). Call once per render tick.
    void poll_events(int timeout_ms = 0);

    // Send `msg` as a single text frame to every open client. Dead clients
    // are dropped. Returns the number of clients written to.
    int broadcast(const std::string& msg);

    size_t client_count() const;
    uint16_t port() const { return port_; }
    bool running() const { return listen_fd_ >= 0; }

    // Exposed for testing: RFC 6455 Sec-WebSocket-Accept derivation.
    static std::string compute_accept_key(const std::string& sec_websocket_key);
    // Exposed for testing: encode a server->client text frame (unmasked).
    static std::string encode_text_frame(const std::string& payload);

private:
    struct Client {
        int         fd = -1;
        bool        handshaked = false;
        std::string inbuf;   // pending request bytes / partial frames
    };

    void accept_new();
    // Returns false if the client should be dropped.
    bool service_client(Client& c);
    bool do_handshake(Client& c);
    void consume_frames(Client& c, bool& drop);

    int      listen_fd_ = -1;
    uint16_t port_ = 0;
    std::vector<Client> clients_;
};

} // namespace mec
