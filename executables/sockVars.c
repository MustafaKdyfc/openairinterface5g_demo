/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */
/*! \file ru_control.c
 * \brief Top-level threads for RU entity
 * \author R. Knopp, F. Kaltenberger, Navid Nikaein
 * \date 2018
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr, navid.nikaein@eurecom.fr
 * \note
 * \warning
 */
 
 // sockVars.c
#include "sockVars.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

/* Keep both legacy names: prach_sockfd used in this file and
    sockfd/server_addr declared in sockVars.h and used elsewhere. */
int prach_sockfd = -1;
int sockfd = -1;
struct sockaddr_in prach_server_addr;
struct sockaddr_in server_addr;

int prach_socket_init(const char *ip, int port)
{
    prach_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (prach_sockfd < 0) {
        perror("prach socket");
        return -1;
    }

    /* mirror socket and addr into the symbols expected by other modules */
    sockfd = prach_sockfd;

    memset(&prach_server_addr, 0, sizeof(prach_server_addr));
    prach_server_addr.sin_family = AF_INET;
    prach_server_addr.sin_port = htons(port);

    if (inet_aton(ip, &prach_server_addr.sin_addr) == 0) {
        fprintf(stderr, "Invalid PRACH target IP: %s\n", ip);
        close(prach_sockfd);
        prach_sockfd = -1;
        sockfd = -1;
        return -1;
    }

    /* also set the global server_addr used by other files */
    memcpy(&server_addr, &prach_server_addr, sizeof(server_addr));

    // Optional: connect() could be used to set default peer (not required)
    return 0;
}

void prach_socket_close(void)
{
    if (prach_sockfd >= 0) {
        close(prach_sockfd);
        prach_sockfd = -1;
    }
    if (sockfd >= 0 && sockfd != prach_sockfd) {
        close(sockfd);
        sockfd = -1;
    }
}


