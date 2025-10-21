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

#include <cstring>
#include <cerrno>

static const char *TAG = "streamserver";

using namespace esphome;

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    struct sockaddr_in bind_addr;
    std::memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(this->port_);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    this->socket_ = socket::socket(AF_INET, SOCK_STREAM, PF_INET);
    if (!this->socket_) {
        ESP_LOGE(TAG, "Failed to create listening socket");
        return;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000; // ESPHome recommends 20-30 ms max for timeouts

    // set receive and send timeouts
    this->socket_->setsockopt(SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));
    this->socket_->setsockopt(SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout));

    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(struct sockaddr_in));
    this->socket_->listen(8);

    ESP_LOGCONFIG(TAG, "Stream server listening on port %u", this->port_);
}

void StreamServerComponent::loop() {
    this->accept();
    this->read();
    this->write();
    this->cleanup();
}

void StreamServerComponent::accept() {
    struct sockaddr_in client_addr;
    socklen_t client_addrlen = sizeof(struct sockaddr_in);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    // make client socket non-blocking
    socket->setblocking(false);
    std::string identifier = socket->getpeername();
    this->clients_.emplace_back(std::move(socket), std::move(identifier));
    ESP_LOGD(TAG, "New client connected from %s", this->clients_.back().identifier.c_str());
}

void StreamServerComponent::cleanup() {
    auto discriminator = [](const Client &client) { return !client.disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    if (last_client != this->clients_.end()) {
        ESP_LOGD(TAG, "Cleaning up %u disconnected client(s)", static_cast<unsigned>(std::distance(last_client, this->clients_.end())));
    }
    this->clients_.erase(last_client, this->clients_.end());
}

void StreamServerComponent::read() {
    int len;
    while ((len = this->stream_->available()) > 0) {
        char buf[128];
        int to_read = std::min(len, static_cast<int>(sizeof(buf)));
        this->stream_->read_array(reinterpret_cast<uint8_t*>(buf), to_read);
        for (const Client &client : this->clients_) {
            if (!client.disconnected)
                client.socket->write(reinterpret_cast<const uint8_t*>(buf), to_read);
        }
    }
}

void StreamServerComponent::write() {
    uint8_t buf[128];
    ssize_t len;
    for (Client &client : this->clients_) {
        if (client.disconnected)
            continue;

        while ((len = client.socket->read(buf, sizeof(buf))) > 0) {
            // write data from client into the UART stream
            this->stream_->write_array(buf, static_cast<size_t>(len));
        }

        if (len == 0) {
            // graceful close by peer
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
            continue;
        } else if (len < 0) {
            // non-blocking socket commonly returns EAGAIN/EWOULDBLOCK when no data is available
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // no data available right now, that's fine
                continue;
            } else {
                // real error
                ESP_LOGE(TAG, "Error reading from client %s: errno=%d", client.identifier.c_str(), errno);
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
}

void StreamServerComponent::on_shutdown() {
    for (const Client &client : this->clients_) {
        if (client.socket)
            client.socket->shutdown(SHUT_RDWR);
    }
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier)
    : socket(std::move(socket)), identifier(std::move(identifier))
{
}
