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

#include <cstring>
#include <sys/time.h>
#include <errno.h>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

static const char *TAG = "streamserver";

using namespace esphome;

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // Portable initialization: zero the struct then set fields
    struct sockaddr_in bind_addr;
    std::memset(&bind_addr, 0, sizeof(bind_addr));
#if defined(__APPLE__) || defined(BSD)
    bind_addr.sin_len = sizeof(struct sockaddr_in);
#endif
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(this->port_);
    bind_addr.sin_addr.s_addr = ESPHOME_INADDR_ANY;

    this->socket_ = socket::socket(AF_INET, SOCK_STREAM, PF_INET);
    if (!this->socket_) {
        ESP_LOGE(TAG, "Failed to create socket");
        return;
    }

    // Set non-blocking accept/read on the listening socket as well if supported.
    // (leave as blocking if setblocking fails)
    this->socket_->setblocking(false);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000; // ESPHome recommends 20-30 ms max for timeouts

    // Set receive timeout
    this->socket_->setsockopt(SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
    // ORIGINAL BUG: SO_RCVTIMEO was set twice. Second should be send timeout.
    this->socket_->setsockopt(SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

    // Bind
    if (!this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(struct sockaddr_in))) {
        ESP_LOGE(TAG, "Failed to bind socket to port %u", this->port_);
        // try to close socket by resetting pointer
        this->socket_.reset();
        return;
    }

    if (!this->socket_->listen(8)) {
        ESP_LOGE(TAG, "Failed to listen on socket");
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

    struct sockaddr_in client_addr;
    socklen_t client_addrlen = sizeof(struct sockaddr_in);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    // set non-blocking for client socket
    socket->setblocking(false);

    std::string identifier = socket->getpeername();
    this->clients_.emplace_back(std::move(socket), identifier);
    ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
}

void StreamServerComponent::cleanup() {
    auto discriminator = [](const Client &client) { return !client.disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    // close sockets for removed clients if necessary
    for (auto it = last_client; it != this->clients_.end(); ++it) {
        if (it->socket)
            it->socket->shutdown(SHUT_RDWR);
    }
    this->clients_.erase(last_client, this->clients_.end());
}

void StreamServerComponent::read() {
    if (!this->stream_)
        return;

    int len;
    while ((len = this->stream_->available()) > 0) {
        char buf[128];
        len = std::min(len, 128);
        // read_array returns void; ensure we actually have len > 0
        this->stream_->read_array(reinterpret_cast<uint8_t*>(buf), len);
        for (const Client &client : this->clients_) {
            if (client.disconnected || !client.socket)
                continue;
            // socket->write may return negative on error; we ignore here but could check return value
            client.socket->write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(len));
        }
    }
}

void StreamServerComponent::write() {
    uint8_t buf[128];
    ssize_t len;
    for (Client &client : this->clients_) {
        if (client.disconnected || !client.socket)
            continue;

        // read from client; handle non-blocking behavior and errors
        while ((len = client.socket->read(&buf, sizeof(buf))) > 0) {
            // write_array expects size_t
            this->stream_->write_array(buf, static_cast<size_t>(len));
        }

        if (len == 0) {
            // orderly EOF -> remote closed connection
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
            continue;
        }

        if (len < 0) {
            // read error. If would block, ignore; otherwise mark disconnected.
            int err = errno;
#ifdef EWOULDBLOCK
            if (err == EWOULDBLOCK || err == EAGAIN) {
                // no data available right now for non-blocking socket
                continue;
            }
#endif
            ESP_LOGD(TAG, "Client %s read error (errno=%d), closing", client.identifier.c_str(), err);
            client.disconnected = true;
            continue;
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
}

void StreamServerComponent::on_shutdown() {
    for (const Client &client : this->clients_) {
        if (client.socket)
            client.socket->shutdown(SHUT_RDWR);
    }
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier)
    : socket(std::move(socket)), identifier{identifier}
{
}
