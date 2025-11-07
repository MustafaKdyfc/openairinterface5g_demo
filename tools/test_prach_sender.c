// Simple test sender for PRACH UDP
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include "sockVars.h"

/* Functions in sockVars.c */
int prach_socket_init(const char *ip, int port);
void prach_socket_close(void);

int main() {
    // Initialize socket (127.0.0.1:5678)
    if (prach_socket_init("127.0.0.1", 5678) < 0) {
        fprintf(stderr, "Failed to init socket\n");
        return 1;
    }

    // Create test packet with header + payload
    typedef struct {
        uint32_t frame;
        uint32_t slot;
        uint8_t antenna;
        uint8_t prachOccasion;
        uint16_t N_ZC;
        uint16_t data_len_bytes;
    } __attribute__((packed)) prach_header_t;

    prach_header_t header = {
        .frame = htonl(123),          // frame 123
        .slot = htonl(45),           // slot 45
        .antenna = 0,                // antenna 0
        .prachOccasion = 1,          // occasion 1
        .N_ZC = htons(139),          // N_ZC=139
        .data_len_bytes = htons(278)  // 139 complex samples (278 int16)
    };

    // Create test payload: ascending pattern
    int16_t payload[278];  // 139 complex samples
    for (int i = 0; i < 278; i++) {
        payload[i] = (int16_t)(i - 139);  // -139..138
    }

    // Combine header + payload
    size_t total_size = sizeof(header) + sizeof(payload);
    uint8_t *packet = malloc(total_size);
    if (!packet) {
        fprintf(stderr, "Failed to allocate packet buffer\n");
        return 1;
    }
    memcpy(packet, &header, sizeof(header));
    memcpy(packet + sizeof(header), payload, sizeof(payload));

    // Send 5 packets with 100ms delay
    for (int i = 0; i < 5; i++) {
        ssize_t sent = sendto(prach_sockfd, packet, total_size, 0,
                (struct sockaddr*)&prach_server_addr,
                sizeof(prach_server_addr));
        if (sent < 0) {
            perror("sendto failed");
            free(packet);
            return 1;
        }
        if ((size_t)sent != total_size) {
            fprintf(stderr, "Partial send: %zd/%zu\n", sent, total_size);
            free(packet);
            return 1;
        }
        printf("Sent test packet %d: %zd bytes OK\n", i+1, sent);
        usleep(100000);  // 100ms
    }

    free(packet);
    prach_socket_close();
    return 0;
}