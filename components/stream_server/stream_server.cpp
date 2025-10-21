/* Copyright (C) 2020-2022 Oxan van Leeuwen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

#include <cerrno>
#include <chrono>

static const char *TAG = "streamserver";

using namespace esphome;

static constexpr size_t IO_BUF_SIZE = 128;
static constexpr size_t MAX_CLIENTS = 8;

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    struct sockaddr_in bind_addr = {
        .sin_len = sizeof(struct sockaddr_in),
        .sin_family = AF_INET,
        .sin_port = htons(this->port_),
        .sin_addr = {
            .s_addr = ESPHOME_INADDR_ANY,
        }
    };

    this->socket_ = socket::socket(AF_INET, SOCK_STREAM, PF_INET);
    if (!this->socket_) {
        ESP_LOGE(TAG, "Failed to create listening socket");
        return;
    }

    // Allow quick restart of server (avoid 'address already in use')
    int yes = 1;
    this->socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes));

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000; // ESPHome recommends 20-30 ms max for timeouts

    this->socket_->setsockopt(SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
    // Fix: second option should be send timeout, not receive again
    this->socket_->setsockopt(SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

    // Make listening socket non-blocking so accept() doesn't block the main loop
    this->socket_->setblocking(false);

    // bind & listen (no return checks in original API, keep calls but guard with try/catch if implemented)
    try {
        this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(struct sockaddr_in));
        this->socket_->listen(static_cast<int>(MAX_CLIENTS));
    } catch (...) {
        ESP_LOGE(TAG, "Exception while binding/listening on socket");
        // If API throws, ensure socket cleared
        this->socket_.reset();
        return;
    }
}

void StreamServerComponent::loop() {
    this->accept();
    this->read();
    this->write();
    this->cleanup();
}

void StreamServerComponent::accept() {
    if (!this->socket_)
        return;

    // limit number of clients
    if (this->clients_.size() >= MAX_CLIENTS) {
        return;
    }

    struct sockaddr_in client_addr;
    socklen_t client_addrlen = sizeof(struct sockaddr_in);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    socket->setblocking(false);
    std::string identifier = socket->getpeername();
    this->clients_.emplace_back(std::move(socket), identifier);
    ESP_LOGD(TAG, "New client connected from %s (total clients: %u)", identifier.c_str(), (unsigned)this->clients_.size());
}

void StreamServerComponent::cleanup() {
    // Ensure graceful close of sockets for disconnected clients
    for (auto &client : this->clients_) {
        if (client.disconnected && client.socket) {
            client.socket->shutdown(SHUT_RDWR);
            client.socket.reset();
        }
    }

    auto discriminator = [](const Client &client) { return !client.disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    if (last_client != this->clients_.end()) {
        ESP_LOGD(TAG, "Cleaning up %u disconnected clients", (unsigned)std::distance(last_client, this->clients_.end()));
    }
    this->clients_.erase(last_client, this->clients_.end());
}

void StreamServerComponent::flush_client_out(Client &client) {
    // Try to flush pending out_buffer to client.socket
    if (!client.socket || client.out_buffer.empty())
        return;

    size_t offset = 0;
    while (offset < client.out_buffer.size()) {
        ssize_t wrote = client.socket->write(client.out_buffer.data() + offset, client.out_buffer.size() - offset);
        if (wrote > 0) {
            offset += static_cast<size_t>(wrote);
            continue;
        }
        if (wrote == 0) {
            ESP_LOGD(TAG, "Client %s disconnected during write", client.identifier.c_str());
            client.disconnected = true;
            break;
        }
        // wrote < 0: error / would block
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Can't write more now; keep remaining in buffer for next loop
            break;
        } else {
            ESP_LOGW(TAG, "Write error to client %s errno=%d", client.identifier.c_str(), errno);
            client.disconnected = true;
            break;
        }
    }
    if (offset > 0 && !client.disconnected) {
        // erase sent prefix
        client.out_buffer.erase(client.out_buffer.begin(), client.out_buffer.begin() + static_cast<std::ptrdiff_t>(offset));
    }
    // if disconnected, we'll cleanup later in cleanup()
}

void StreamServerComponent::read() {
    if (!this->stream_)
        return;

    int len;
    while ((len = this->stream_->available()) > 0) {
        uint8_t buf[IO_BUF_SIZE];
        int to_read = std::min(len, static_cast<int>(IO_BUF_SIZE));
        this->stream_->read_array(buf, to_read);

        // write into each client's pending buffer, then try flush
        for (Client &client : this->clients_) {
            if (client.disconnected || !client.socket)
                continue;

            // Append data to client's out buffer
            client.out_buffer.insert(client.out_buffer.end(), buf, buf + to_read);
            // Try to flush immediately
            this->flush_client_out(client);
        }
    }
}

void StreamServerComponent::write() {
    uint8_t buf[IO_BUF_SIZE];
    ssize_t len;
    for (Client &client : this->clients_) {
        if (client.disconnected || !client.socket)
            continue;

        // First, try to flush any pending outbound data to client
        if (!client.out_buffer.empty()) {
            this->flush_client_out(client);
            if (client.disconnected)
                continue;
        }

        // Then, read any data from client and forward to stream
        while ((len = client.socket->read(buf, sizeof(buf))) > 0) {
            // write to stream (could be blocking as it's local stream)
            this->stream_->write_array(buf, static_cast<size_t>(len));
        }

        if (len == 0) {
            // orderly shutdown by peer
            ESP_LOGD(TAG, "Client %s disconnected (read==0)", client.identifier.c_str());
            client.disconnected = true;
            continue;
        } else if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // no data available now; normal for non-blocking sockets
                continue;
            } else {
                ESP_LOGW(TAG, "Read error from client %s errno=%d", client.identifier.c_str(), errno);
                client.disconnected = true;
                continue;
            }
        }
    }
}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
    std::string ip_str = "";
    for (auto &ip : network::get_ip_addresses()) {
      if (ip.is_set())
        ip_str += " " + ip.str();
    }
    ESP_LOGCONFIG(TAG, "  Address:%s", ip_str.c_str());
    ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
    ESP_LOGCONFIG(TAG, "  Max clients: %u", (unsigned)MAX_CLIENTS);
}

void StreamServerComponent::on_shutdown() {
    for (Client &client : this->clients_) {
        if (client.socket) {
            client.socket->shutdown(SHUT_RDWR);
            client.socket.reset();
        }
        client.disconnected = true;
    }
    if (this->socket_) {
        this->socket_->shutdown(SHUT_RDWR);
        this->socket_.reset();
    }
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier)
    : socket(std::move(socket)), identifier{identifier}
{
}
