// Loopback test for WebSocketRenderServer: RFC 6455 handshake derivation,
// frame encoding, and a real end-to-end connect -> handshake -> broadcast ->
// receive over a localhost socket.

#include "mec/render/websocket_render_server.h"
#include "../unit/test_util.h"

#include <cstring>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

using namespace mec;

static int connect_client(uint16_t port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    timeval tv{2, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
        return -1;
    return fd;
}

int main() {
    // --- RFC 6455 §1.3 worked example ---
    CHECK(WebSocketRenderServer::compute_accept_key("dGhlIHNhbXBsZSBub25jZQ==")
          == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");

    // --- text frame header encoding ---
    {
        const std::string f = WebSocketRenderServer::encode_text_frame("hi");
        CHECK(f.size() == 4);
        CHECK(static_cast<uint8_t>(f[0]) == 0x81); // FIN + text
        CHECK(static_cast<uint8_t>(f[1]) == 0x02); // unmasked, len 2
        CHECK(f.substr(2) == "hi");
    }

    // --- end-to-end loopback ---
    WebSocketRenderServer server;
    CHECK(server.start(0, "127.0.0.1")); // ephemeral port
    const uint16_t port = server.port();
    CHECK(port != 0);

    const int cfd = connect_client(port);
    CHECK(cfd >= 0);

    const std::string req =
        "GET /pose HTTP/1.1\r\n"
        "Host: 127.0.0.1\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n";
    CHECK(::send(cfd, req.data(), req.size(), 0) == static_cast<ssize_t>(req.size()));

    // Let the server accept + handshake.
    for (int i = 0; i < 50 && server.client_count() == 0; ++i)
        server.poll_events(10);
    CHECK(server.client_count() == 1);

    // Client reads the 101 response.
    char buf[1024];
    const ssize_t n = ::recv(cfd, buf, sizeof(buf) - 1, 0);
    CHECK(n > 0);
    if (n > 0) {
        buf[n] = '\0';
        const std::string resp(buf, n);
        CHECK(resp.find("101") != std::string::npos);
        CHECK(resp.find("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != std::string::npos);
    }

    // Server broadcasts a pose frame; client receives and decodes it.
    const std::string payload = "{\"type\":\"pose_frame\"}";
    CHECK(server.broadcast(payload) == 1);

    const ssize_t fn = ::recv(cfd, buf, sizeof(buf) - 1, 0);
    CHECK(fn >= 2);
    if (fn >= 2) {
        CHECK((static_cast<uint8_t>(buf[0]) & 0x0F) == 0x1); // text opcode
        const size_t len = static_cast<uint8_t>(buf[1]) & 0x7F;
        CHECK(len == payload.size());
        const std::string got(buf + 2, len);
        CHECK(got == payload);
    }

    // Client disconnects; server reaps it.
    ::close(cfd);
    for (int i = 0; i < 50 && server.client_count() == 1; ++i)
        server.poll_events(10);
    CHECK(server.client_count() == 0);

    server.stop();
    RUN_TESTS_RETURN();
}
