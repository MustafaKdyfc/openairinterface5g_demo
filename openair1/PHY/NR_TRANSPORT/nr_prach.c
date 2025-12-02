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

/*! \file PHY/NR_TRANSPORT/nr_prach.c
 * \brief Top-level routines for generating and decoding the PRACH physical channel V15.4 2018-12
 * \author R. Knopp
 * \date 2019
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr
 * \note
 * \warning
 */
#include <math.h>
#include <stdio.h>
#include "PHY/defs_gNB.h"
#include "SCHED_NR/sched_nr.h"
#include "PHY/NR_TRANSPORT/nr_transport_proto.h"
#include "PHY/NR_TRANSPORT/nr_transport_common_proto.h"
#include "openair1/PHY/NR_TRANSPORT/nr_prach.h"
#include "sockVars.h"
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <arpa/inet.h>

/*
 * Provide weak fallback definitions for PRACH UDP globals.
 * The project has a strong definition in `executables/sockVars.c` when
 * building the full executable, but `libPHY_NR.a` can be built/linked
 * without that object. Defining weak symbols here resolves linker
 * errors when `sockVars.c` is not linked into the library. If a
 * strong symbol exists elsewhere it will override these.
 */
#if defined(__GNUC__)
__attribute__((weak)) int prach_sockfd = -1;
__attribute__((weak)) struct sockaddr_in prach_server_addr;
#if 1
/* Provide weak fallbacks for the legacy names `sockfd`/`server_addr` too.
 * This prevents undefined-reference linker errors when some translation
 * units reference `sockfd`/`server_addr` but the strong definitions
 * live in an executable-only source (e.g. `executables/sockVars.c`).
 * If a strong definition exists elsewhere it will override these.
 */
__attribute__((weak)) int sockfd = -1;
__attribute__((weak)) struct sockaddr_in server_addr;
#endif
#else
int prach_sockfd = -1;
struct sockaddr_in prach_server_addr;
#if 1
int sockfd = -1;
struct sockaddr_in server_addr;
#endif
#endif

/*
 * Provide a weak fallback implementation of prach_socket_init/prach_socket_close
 * so that `nr_prach.c` can lazily initialize the UDP socket even when
 * `executables/sockVars.c` is not linked into the final binary. If a
 * strong implementation exists elsewhere it will override these weak
 * definitions.
 */
#if defined(__GNUC__)
__attribute__((weak)) int prach_socket_init(const char *ip, int port) {
  (void)ip;
  (void)port;
  /* Try to create a UDP socket and bind destination address if possible.
   * We'll set prach_sockfd to -1 on failure to indicate no socket.
   */
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) {
    prach_sockfd = -1;
    return -1;
  }

  prach_sockfd = s;
  memset(&prach_server_addr, 0, sizeof(prach_server_addr));
  prach_server_addr.sin_family = AF_INET;
  prach_server_addr.sin_port = htons((uint16_t)port);
  if (inet_aton(ip, &prach_server_addr.sin_addr) == 0) {
    /* invalid ip string */
    close(prach_sockfd);
    prach_sockfd = -1;
    return -1;
  }

  return 0;
}

__attribute__((weak)) void prach_socket_close(void) {
  if (prach_sockfd >= 0) {
    close(prach_sockfd);
    prach_sockfd = -1;
  }
}
#endif

/* === ISAC/DF: per-antenna snapshot containers (dominant PRACH bin) === */
#ifndef OAI_DF_PRACH_MIN
#define OAI_DF_PRACH_MIN 1
#endif
#if OAI_DF_PRACH_MIN
static float df_y_re[16], df_y_im[16];  /* up to 16 RX antennas */
#endif
/* ===================================================================== */


// --- helper: complex int16 IQ dump ---
static inline void dump_iq_s16(const char* path, const int16_t* iq, size_t n_complex) {
  FILE* f = fopen(path, "wb");
  if (!f) return;
  fwrite(iq, sizeof(int16_t), 2*n_complex, f); // I,Q ardışık int16
  fclose(f);
}

void init_prach_list(PHY_VARS_gNB *gNB)
{
  AssertFatal(gNB != NULL, "gNB is null\n");
  for (int i = 0; i < NUMBER_OF_NR_PRACH_MAX; i++){
    gNB->prach_vars.list[i].frame = -1;
    gNB->prach_vars.list[i].slot = -1;
    gNB->prach_vars.list[i].num_slots = -1;
  }
}

void free_nr_prach_entry(PHY_VARS_gNB *gNB, int prach_id)
{
  gNB->prach_vars.list[prach_id].frame = -1;
  gNB->prach_vars.list[prach_id].slot = -1;
  gNB->prach_vars.list[prach_id].num_slots = -1;
  free_and_zero(gNB->prach_vars.list[prach_id].beam_nb);
}

int16_t find_nr_prach(PHY_VARS_gNB *gNB,int frame, int slot, find_type_t type) {

  AssertFatal(gNB!=NULL,"gNB is null\n");
  for (uint16_t i=0; i<NUMBER_OF_NR_PRACH_MAX; i++) {
    gNB_PRACH_list_t *p = gNB->prach_vars.list + i;
    LOG_D(PHY, "searching for PRACH in %d.%d prach_index %d=> %d.%d\n", frame, slot, i, p->frame, p->slot);
    if ((type == SEARCH_EXIST_OR_FREE) && (p->frame == -1) && (p->slot == -1)) {
      return i;
    } else if ((type == SEARCH_EXIST) && (p->frame == frame) && (p->slot + p->num_slots - 1 == slot)) {
      return i;
    }
  }
  return -1;
}

int nr_fill_prach(PHY_VARS_gNB *gNB, int SFN, int Slot, nfapi_nr_prach_pdu_t *prach_pdu)
{
  int prach_id = find_nr_prach(gNB, SFN, Slot, SEARCH_EXIST_OR_FREE);
  AssertFatal(((prach_id >= 0) && (prach_id < NUMBER_OF_NR_PRACH_MAX)), "illegal or no prach_id found!!! prach_id %d\n", prach_id);
  gNB_PRACH_list_t *prach = &gNB->prach_vars.list[prach_id];
  prach->frame = SFN;
  prach->slot = Slot;
  const int format = prach_pdu->prach_format;
  prach->num_slots = (format < 4) ? get_long_prach_dur(format, gNB->frame_parms.numerology_index) : 1;
  if (gNB->common_vars.beam_id) {
    int n_symb = get_nr_prach_duration(prach_pdu->prach_format);
    prach->beam_nb = calloc(prach_pdu->beamforming.dig_bf_interface, sizeof(*prach->beam_nb));
    for (int i = 0; i < prach_pdu->beamforming.dig_bf_interface; i++) {
      int fapi_beam_idx = prach_pdu->beamforming.prgs_list[0].dig_bf_interface_list[i].beam_idx;
      int start_symb = prach_pdu->prach_start_symbol + i * n_symb;
      int bitmap = SL_to_bitmap(start_symb, n_symb);
      prach->beam_nb[i] = beam_index_allocation(gNB->enable_analog_das,
                                                fapi_beam_idx,
                                                &gNB->gNB_config.analog_beamforming_ve,
                                                &gNB->common_vars,
                                                Slot,
                                                NR_NUMBER_OF_SYMBOLS_PER_SLOT,
                                                bitmap);
    }
  }
  LOG_D(NR_PHY,"Copying prach pdu %d bytes to index %d\n", (int)sizeof(*prach_pdu), prach_id);
  memcpy(&prach->pdu, prach_pdu, sizeof(*prach_pdu));
  return prach_id;
}

void init_prach_ru_list(RU_t *ru)
{
  AssertFatal(ru != NULL, "ru is null\n");
  for (int i = 0; i < NUMBER_OF_NR_RU_PRACH_MAX; i++) {
    ru->prach_list[i].frame = -1;
    ru->prach_list[i].slot = -1;
    ru->prach_list[i].num_slots = -1;
  }
  pthread_mutex_init(&ru->prach_list_mutex, NULL);
}

int16_t find_nr_prach_ru(RU_t *ru,int frame,int slot, find_type_t type)
{
  AssertFatal(ru != NULL, "ru is null\n");
  pthread_mutex_lock(&ru->prach_list_mutex);
  for (int i = 0; i < NUMBER_OF_NR_RU_PRACH_MAX; i++) {
    RU_PRACH_list_t *p = ru->prach_list + i;
    LOG_D(PHY, "searching for PRACH in %d.%d : prach_index %d=> %d.%d\n", frame, slot, i, p->frame, p->slot);
    if ((type == SEARCH_EXIST_OR_FREE) && (p->frame == -1) && (p->slot == -1)) {
      pthread_mutex_unlock(&ru->prach_list_mutex);
      return i;
    } else if ((type == SEARCH_EXIST) && (p->frame == frame) && (p->slot + p->num_slots - 1 == slot)) {
      pthread_mutex_unlock(&ru->prach_list_mutex);
      return i;
    }
  }
  pthread_mutex_unlock(&ru->prach_list_mutex);
  return -1;
}

void nr_fill_prach_ru(RU_t *ru, int SFN, int Slot, nfapi_nr_prach_pdu_t *prach_pdu, int *beam_id)
{
  int prach_id = find_nr_prach_ru(ru, SFN, Slot, SEARCH_EXIST_OR_FREE);
  AssertFatal((prach_id >= 0) && (prach_id < NUMBER_OF_NR_PRACH_MAX),
              "illegal or no prach_id found!!! prach_id %d\n",
              prach_id);

  pthread_mutex_lock(&ru->prach_list_mutex);
  ru->prach_list[prach_id].frame = SFN;
  ru->prach_list[prach_id].slot = Slot;
  const int fmt = prach_pdu->prach_format;
  ru->prach_list[prach_id].num_slots = (fmt < 4) ? get_long_prach_dur(fmt, ru->nr_frame_parms->numerology_index) : 1;
  ru->prach_list[prach_id].fmt = fmt;
  ru->prach_list[prach_id].numRA = prach_pdu->num_ra;
  ru->prach_list[prach_id].beam = beam_id;
  ru->prach_list[prach_id].prachStartSymbol = prach_pdu->prach_start_symbol;
  ru->prach_list[prach_id].num_prach_ocas = prach_pdu->num_prach_ocas;
  pthread_mutex_unlock(&ru->prach_list_mutex);
}

void free_nr_ru_prach_entry(RU_t *ru, int prach_id)
{
  pthread_mutex_lock(&ru->prach_list_mutex);
  ru->prach_list[prach_id].frame = -1;
  ru->prach_list[prach_id].slot  = -1;
  pthread_mutex_unlock(&ru->prach_list_mutex);
}

// --- ADD: helper to dump complex int16 IQ ---


void rx_nr_prach_ru(RU_t *ru,
                    int prachFormat,
                    int numRA,
                    int beam,
                    int prachStartSymbol,
                    int prachStartSlot,
                    int prachOccasion,
                    int frame,
                    int slot)
{
  AssertFatal(ru != NULL,"ru is null\n");

  /* Log RU index and configured number of RX antennas for easier debugging */
  LOG_I(PHY, "rx_nr_prach_ru: ru_idx=%d nb_rx=%d prachOcc=%d frame=%d slot=%d\n", ru->idx, ru->nb_rx, prachOccasion, frame, slot);

  NR_DL_FRAME_PARMS *fp = ru->nr_frame_parms;
  int slot2 = slot;
  int16_t *prach[ru->nb_rx];
  int prach_sequence_length = ru->config.prach_config.prach_sequence_length.value;
  int msg1_frequencystart = ru->config.prach_config.num_prach_fd_occasions_list[numRA].k1.value;
  const uint8_t prach_mu = ru->config.prach_config.prach_sub_c_spacing.value;

  int sample_offset_slot;
  if (prachStartSymbol == 0) {
    sample_offset_slot = 0;
  } else if (fp->slots_per_subframe == 1) {
    if (prachStartSymbol <= 7)
      sample_offset_slot = (fp->ofdm_symbol_size + fp->nb_prefix_samples) * (prachStartSymbol - 1) + (fp->ofdm_symbol_size + fp->nb_prefix_samples0);
    else
      sample_offset_slot = (fp->ofdm_symbol_size + fp->nb_prefix_samples) * (prachStartSymbol - 2) + (fp->ofdm_symbol_size + fp->nb_prefix_samples0) * 2;
  } else {
    if (!(slot%(fp->slots_per_subframe/2)))
      sample_offset_slot = (fp->ofdm_symbol_size + fp->nb_prefix_samples) * (prachStartSymbol - 1) + (fp->ofdm_symbol_size + fp->nb_prefix_samples0);
    else
      sample_offset_slot = (fp->ofdm_symbol_size + fp->nb_prefix_samples) * prachStartSymbol;
  }

  LOG_D(PHY,
        "frame %d, slot %d: doing rx_nr_prach_ru for format %d, numRA %d, prachStartSymbol %d, prachOccasion %d\n",
        frame,
        slot,
        prachFormat,
        numRA,
        prachStartSymbol,
        prachOccasion);

  int16_t **rxsigF = ru->prach_rxsigF[prachOccasion];

  AssertFatal(ru->if_south == LOCAL_RF || ru->if_south == REMOTE_IF5,
              "we shouldn't call this if if_south != LOCAL_RF or REMOTE_IF5\n");

  for (int aa = 0; aa < ru->nb_rx; aa++) { 
    if (prach_sequence_length == 0)
      slot2 = prachStartSlot;
    int idx = aa + beam * ru->nb_rx;
    prach[aa] = (int16_t*)&ru->common.rxdata[idx][fp->get_samples_slot_timestamp(slot2, fp, 0) + sample_offset_slot - ru->N_TA_offset];
  } 

  int reps;
  int Ncp;
  int dftlen;
  int mu = fp->numerology_index;

  if (prach_sequence_length == 0) {
    LOG_D(PHY,
          "PRACH (ru %d) in %d.%d, format %d, msg1_frequencyStart %d\n",
	  ru->idx,
	  frame,
	  slot2,
	  prachFormat,
	  msg1_frequencystart);
    switch (prachFormat) {
    case 0:
      reps = 1;
      Ncp = 3168;
      dftlen = 24576;
      break;

    case 1:
      reps = 2;
      Ncp = 21024;
      dftlen = 24576;
      break;

    case 2:
      reps = 4;
      Ncp = 4688;
      dftlen = 24576;
      break;

    case 3:
      reps = 4;
      Ncp = 3168;
      dftlen = 6144;
      break;

    default:
      AssertFatal(1==0, "Illegal prach format %d for length 839\n", prachFormat);
      break;
    }
  }
  else {
    LOG_D(PHY,
          "PRACH (ru %d) in %d.%d, format %s, msg1_frequencyStart %d,startSymbol %d\n",
	  ru->idx,
	  frame,
	  slot,
	  prachfmt[prachFormat],
	  msg1_frequencystart,
	  prachStartSymbol);
    switch (prachFormat) {
    case 4: //A1
      reps = 2;
      Ncp = 288 >> mu;
      break;

    case 5: //A2
      reps = 4;
      Ncp = 576 >> mu;
      break;

    case 6: //A3
      reps = 6;
      Ncp = 864 >> mu;
      break;

    case 7: //B1
      reps = 2;
      Ncp = 216 >> mu;
      break;

    /*
    // B2 and B3 do not exist in FAPI
    case 4: //B2
      reps = 4;
      Ncp = 360 >> mu;
      break;

    case 5: //B3
      reps = 6;
      Ncp = 504 >> mu;
      break;
    */

    case 8: //B4
      reps = 12;
      Ncp = 936 >> mu;
      break;

    case 9: //C0
      reps = 1;
      Ncp = 1240 >> mu;
      break;

    case 10: //C2
      reps = 4;
      Ncp = 2048 >> mu;
      break;

    default:
      AssertFatal(1==0,"unknown prach format %x\n",prachFormat);
      break;
    }
    dftlen = 2048 >> mu;
  }

  //actually what we should be checking here is how often the current prach crosses a 0.5ms boundary.
  //I am not quite sure for which paramter set this would be the case,
  //so I will ignore it for now and just check if the prach starts on a 0.5ms boundary
  if(fp->numerology_index == 0) {
    if (prachStartSymbol == 0 || prachStartSymbol == 7)
      Ncp += 16;
  }
  else {
    if (slot%(fp->slots_per_subframe/2)==0 && prachStartSymbol == 0)
      Ncp += 16;
  }

  switch(fp->samples_per_subframe) {
  case 7680:
    // 5 MHz @ 7.68 Ms/s
    Ncp >>= 2;
    dftlen >>= 2;
    break;

  case 15360:
    // 10, 15 MHz @ 15.36 Ms/s
    Ncp >>= 1;
    dftlen >>= 1;
    break;

  case 23040:
    // 20 MHz @ 23.04 Ms/s
    Ncp = (Ncp * 3) / 4;
    dftlen = (dftlen * 3) / 4;
    break;

  case 30720:
    // 20, 25, 30 MHz @ 30.72 Ms/s
    break;

  case 46080:
    // 40 MHz @ 46.08 Ms/s
    Ncp = (Ncp*3)/2;
    dftlen = (dftlen*3)/2;
    break;

  case 61440:
    // 40, 50, 60 MHz @ 61.44 Ms/s
    Ncp <<= 1;
    dftlen <<= 1;
    break;

  case 92160:
    // 50, 60, 70, 80, 90 MHz @ 92.16 Ms/s
    Ncp *= 3;
    dftlen *= 3;
    break;

  case 122880:
    // 70, 80, 90, 100 MHz @ 122.88 Ms/s
    Ncp <<= 2;
    dftlen <<= 2;
    break;

  case 184320:
    // 100 MHz @ 184.32 Ms/s
    Ncp = Ncp*6;
    dftlen = dftlen*6;
    break;

  case 245760:
    // 200 MHz @ 245.76 Ms/s
    Ncp <<= 3;
    dftlen <<= 3;
    break;

  default:
    AssertFatal(1==0,"sample rate %f MHz not supported for numerology %d\n", fp->samples_per_subframe / 1000.0, mu);
  }

  const dft_size_idx_t dftsize = get_dft(dftlen);

  // Do forward transform
  if (LOG_DEBUGFLAG(DEBUG_PRACH)) {
    LOG_I(PHY, "rx_prach: Doing PRACH FFT for nb_rx:%d Ncp:%d dftlen:%d\n", ru->nb_rx, Ncp, dftlen);
  }

  const unsigned int K = get_prach_K(prach_sequence_length, prachFormat, fp->numerology_index, prach_mu);
  const uint8_t kbar = get_PRACH_k_bar(prach_mu, fp->numerology_index);

  int n_ra_prb            = msg1_frequencystart;
  int k                   = (12*n_ra_prb) - 6*fp->N_RB_UL;

  int N_ZC = (prach_sequence_length==0)?839:139;

  if (k<0) k+=(fp->ofdm_symbol_size);
  
  k*=K;
  k+=kbar;

  static int dump_counter = 0;
  static const int MAX_DUMPS = 4;  // Sadece 4 kez kaydet

  for (int aa=0; aa<ru->nb_rx; aa++) {
      AssertFatal(prach[aa]!=NULL,"prach[%d] is null\n",aa);

      // do DFT
      int16_t *prach2 = prach[aa] + (2*Ncp);
      for (int i = 0; i < reps; i++)
        dft(dftsize, prach2 + 2*dftlen*i, rxsigF[aa] + 2*dftlen*i, 1);

      // ============ CRITICAL: SEND RAW PRACH DATA BEFORE COMBINING ============
      int16_t *rxsigF2 = rxsigF[aa];
      int k2 = k << 1;
      
      // Allocate buffer for RAW extraction (before combining)
      int16_t raw_iq[N_ZC * 2];  // I,Q pairs
      
      // Extract N_ZC samples starting from k (wrapping around if needed)
      for (int j = 0; j < (N_ZC << 1); j++, k2++) {
          if (k2 >= (dftlen << 1)) k2 = 0;
          raw_iq[j] = rxsigF2[k2];  // Take FIRST repetition only
      }

    // Send RAW IQ data over UDP (BEFORE combining)
    // Lazy-init the PRACH socket so nr_prach can send without an external
    // sender having to initialize sockets beforehand.
    if (prach_sockfd < 0) {
      if (prach_socket_init("127.0.0.1", 5678) < 0) {
        LOG_W(PHY, "PRACH: lazy socket init failed, will skip UDP send\n");
      } else {
        LOG_I(PHY, "PRACH: lazy socket initialized to 127.0.0.1:5678\n");
      }
    }

    /* Do not send raw PRACH from the RU path to avoid transmitting every
     * PRACH buffer before detection. Keep local dumps for debugging only. */
    {
      /* quick diagnostic: compute energy (sum of squares) to detect all-zero buffers */
      uint64_t raw_energy = 0;
      for (int ej = 0; ej < (N_ZC << 1); ej += 2) {
        int32_t re = raw_iq[ej];
        int32_t im = raw_iq[ej+1];
        raw_energy += (uint64_t)(re * re) + (uint64_t)(im * im);
      }
      if (raw_energy == 0) {
        LOG_W(PHY, "PRACH RAW energy==0 ant%d fr=%d sl=%d occ=%d — not sending (RU)\n", aa, frame, slot, prachOccasion);
      } else {
        LOG_D(PHY, "PRACH RAW energy=%llu ant%d fr=%d sl=%d occ=%d — dumped to /tmp (RU)\n",
              (unsigned long long)raw_energy, aa, frame, slot, prachOccasion);
      }
    }
      
      // Optional: Dump RAW data to file for verification
      char fn[160];
      snprintf(fn, sizeof(fn), "/tmp/prach_ru_raw_occ%03d_ant%02d_fr%04d_sl%03d.bin",
              prachOccasion, aa, frame, slot);
      dump_iq_s16(fn, raw_iq, N_ZC);

      // ============ NOW DO THE COMBINING (for internal processing) ============
      int16_t rxsigF_tmp[N_ZC << 1];
      k2 = k << 1;
      
      for (int j = 0; j < (N_ZC << 1); j++, k2++) {
          if (k2 >= (dftlen << 1)) k2 = 0;
          rxsigF_tmp[j] = rxsigF2[k2];
          // Combine repetitions
          for (int i = 1; i < reps; i++) {
              int idx = k2 + (i * dftlen << 1);
              rxsigF_tmp[j] += rxsigF2[idx];
          }
      }
      memcpy((void*)rxsigF2, (void*)rxsigF_tmp, N_ZC << 2);
      
      // Dump COMBINED data (for comparison)
      snprintf(fn, sizeof(fn), "/tmp/prach_ru_combined_occ%03d_ant%02d_fr%04d_sl%03d.bin",
              prachOccasion, aa, frame, slot);
      dump_iq_s16(fn, (const int16_t*)rxsigF2, N_ZC);
  }
}

void rx_nr_prach(PHY_VARS_gNB *gNB,
                 nfapi_nr_prach_pdu_t *prach_pdu,
                 int prachOccasion,
                 int frame,
                 int slot,
                 uint16_t *max_preamble,
                 uint16_t *max_preamble_energy,
                 uint16_t *max_preamble_delay)
{
  AssertFatal(gNB != NULL, "Can only be called from gNB\n");
  nfapi_nr_prach_config_t *cfg = &gNB->gNB_config.prach_config;
  NR_DL_FRAME_PARMS *fp;

  uint16_t rootSequenceIndex;
  int numrootSequenceIndex;
  uint8_t restricted_set;
  uint8_t n_ra_prb=0xFF;
  int nb_rx;

  int16_t **rxsigF = gNB->prach_vars.rxsigF;

  // --- ADD: dump gNB-side PRACH-F before correlation (antenna 0) ---
{
  char fn[160];
  // Not: prachOccasion bu fonksiyona argüman olarak geliyor
  snprintf(fn, sizeof(fn), "/tmp/prach_gnb_f_occ%03d_fr%04d_sl%03d_ant00.bin",
           prachOccasion, frame, slot);
  dump_iq_s16(fn, (const int16_t*)rxsigF[0], (gNB->gNB_config.prach_config.prach_sequence_length.value==0)?839:139);
}


  uint8_t preamble_index;
  uint16_t NCS=99,NCS2;
  uint16_t preamble_offset=0,preamble_offset_old;
  int16_t preamble_shift=0;
  uint32_t preamble_shift2;
  uint16_t preamble_index0=0,n_shift_ra=0,n_shift_ra_bar;
  uint16_t d_start=0;
  uint16_t numshift=0;
  const uint16_t *prach_root_sequence_map;
  uint8_t not_found;
  uint16_t u;
  int16_t *Xu=0;
  uint16_t first_nonzero_root_idx=0;
  uint8_t new_dft=0;
  int32_t lev;
  int16_t levdB;
  int log2_ifft_size=10;
  int16_t prach_ifft_tmp[2048*2] __attribute__((aligned(32)));
  int32_t *prach_ifft=(int32_t*)NULL;
  
  fp = &gNB->frame_parms;

  nb_rx = gNB->gNB_config.carrier_config.num_rx_ant.value;
  rootSequenceIndex = cfg->num_prach_fd_occasions_list[prach_pdu->num_ra].prach_root_sequence_index.value;
  numrootSequenceIndex = cfg->num_prach_fd_occasions_list[prach_pdu->num_ra].num_root_sequences.value;
  NCS = prach_pdu->num_cs;//cfg->num_prach_fd_occasions_list[0].prach_zero_corr_conf.value;
  int prach_sequence_length = cfg->prach_sequence_length.value;
  int msg1_frequencystart = cfg->num_prach_fd_occasions_list[prach_pdu->num_ra].k1.value;
  //  int num_unused_root_sequences = cfg->num_prach_fd_occasions_list[0].num_unused_root_sequences.value;
  // cfg->num_prach_fd_occasions_list[0].unused_root_sequences_list

  restricted_set = cfg->restricted_set_config.value;

  uint8_t prach_fmt = prach_pdu->prach_format;
  uint16_t N_ZC = (prach_sequence_length==0)?839:139;

  LOG_D(PHY,"L1 PRACH RX: rooSequenceIndex %d, numRootSeqeuences %d, NCS %d, N_ZC %d, format %d \n",rootSequenceIndex,numrootSequenceIndex,NCS,N_ZC,prach_fmt);

  prach_ifft = gNB->prach_vars.prach_ifft;
  if (LOG_DEBUGFLAG(DEBUG_PRACH)) {
    if ((frame&1023) < 20) LOG_D(PHY,"PRACH (gNB) : running rx_prach for slot %d, msg1_frequencystart %d, rootSequenceIndex %d\n", slot, msg1_frequencystart, rootSequenceIndex);
  }

  start_meas(&gNB->rx_prach);

  prach_root_sequence_map = (cfg->prach_sequence_length.value==0) ? prach_root_sequence_map_0_3 : prach_root_sequence_map_abc;

  // PDP is oversampled, e.g. 1024 sample instead of 839
  // Adapt the NCS (zero-correlation zones) with oversampling factor e.g. 1024/839
  NCS2 = (N_ZC==839) ? ((NCS<<10)/839) : ((NCS<<8)/139);

  if (NCS2==0) NCS2 = N_ZC;


  preamble_offset_old = 99;

  
  *max_preamble_energy = 0;
  *max_preamble_delay = 0;
  *max_preamble = 0;
  int16_t prachF[2 * 1024];

  
  for (preamble_index=0 ; preamble_index<64 ; preamble_index++) {
    if (LOG_DEBUGFLAG(DEBUG_PRACH)) {
      int en = dB_fixed(signal_energy((int32_t*)&rxsigF[0][0],(N_ZC==839) ? 840: 140));
      

      if (en > 0) {  // enerji varsa yaz
      char fn[160];
      snprintf(fn, sizeof(fn),
           "/tmp/prach_gnb_f_occ%03d_fr%04d_sl%03d_ant00.bin",
           prachOccasion, frame, slot);
      dump_iq_s16(fn, (const int16_t*)rxsigF[0], N_ZC);
  }     
    else {
    LOG_W(PHY,"[DUMP] rxsigF energy=0, dump atlanıyor (fr=%d, sl=%d, occ=%d)\n", frame, slot, prachOccasion);
  }
      if (en>60) LOG_D(PHY,"frame %d, slot %d : Trying preamble %d \n",frame,slot,preamble_index);
    }
    if (restricted_set == 0) {
      // This is the relative offset in the root sequence table (5.7.2-4 from 36.211) for the given preamble index
      preamble_offset = ((NCS==0)? preamble_index : (preamble_index/(N_ZC/NCS)));

      if (preamble_offset != preamble_offset_old) {
        preamble_offset_old = preamble_offset;
        new_dft = 1;
        // This is the \nu corresponding to the preamble index
        preamble_shift  = 0;
      }

      else {
        preamble_shift  -= NCS;

        if (preamble_shift < 0)
          preamble_shift+=N_ZC;
      }
    } else { // This is the high-speed case
      new_dft = 0;
      uint16_t nr_du[NR_PRACH_SEQ_LEN_L - 1];
      nr_fill_du(N_ZC, prach_root_sequence_map, nr_du);
      // set preamble_offset to initial rootSequenceIndex and look if we need more root sequences for this
      // preamble index and find the corresponding cyclic shift
      // Check if all shifts for that root have been processed
      if (preamble_index0 == numshift) {
        not_found = 1;
        new_dft   = 1;
        preamble_index0 -= numshift;
        //(preamble_offset==0 && numshift==0) ? (preamble_offset) : (preamble_offset++);

        while (not_found == 1) {
          // current root depending on rootSequenceIndex
          int index = (rootSequenceIndex + preamble_offset) % N_ZC;

          u = prach_root_sequence_map[index];
	  
          uint16_t n_group_ra = 0;
	  
          if ((nr_du[u] < (N_ZC / 3)) && (nr_du[u] >= NCS) ) {
            n_shift_ra = nr_du[u] / NCS;
            d_start = (nr_du[u] << 1) + (n_shift_ra * NCS);
            n_group_ra = N_ZC / d_start;
            n_shift_ra_bar = max(0, (N_ZC-(nr_du[u] << 1) - (n_group_ra * d_start)) / N_ZC);
          } else if  ((nr_du[u] >= (N_ZC / 3)) && (nr_du[u] <= ((N_ZC - NCS) >> 1))) {
            n_shift_ra = (N_ZC - (nr_du[u] << 1)) / NCS;
            d_start = N_ZC - (nr_du[u] << 1) + (n_shift_ra * NCS);
            n_group_ra = nr_du[u] / d_start;
            n_shift_ra_bar = min(n_shift_ra, max(0, (nr_du[u]- (n_group_ra * d_start)) / NCS));
          } else {
            n_shift_ra = 0;
            n_shift_ra_bar = 0;
          }

          // This is the number of cyclic shifts for the current root u
          numshift = (n_shift_ra * n_group_ra) + n_shift_ra_bar;
          // skip to next root and recompute parameters if numshift==0
          (numshift>0) ? (not_found = 0) : (preamble_offset++);
        }
      }

      if (n_shift_ra>0)
        preamble_shift = -((d_start * (preamble_index0/n_shift_ra)) + ((preamble_index0%n_shift_ra)*NCS)); // minus because the channel is h(t -\tau + Cv)
      else
        preamble_shift = 0;

      if (preamble_shift < 0)
        preamble_shift+=N_ZC;

      preamble_index0++;

      if (preamble_index == 0)
        first_nonzero_root_idx = preamble_offset;
    }

    // Compute DFT of RX signal (conjugate input, results in conjugate output) for each new rootSequenceIndex
    if (LOG_DEBUGFLAG(DEBUG_PRACH)) {
      int en = dB_fixed(signal_energy((int32_t*)&rxsigF[0][0],840));
      if (en>60)
        LOG_D(PHY,
              "frame %d, slot %d : preamble index %d, NCS %d, N_ZC/NCS %d: offset %d, preamble shift %d , en %d)\n",
              frame,
              slot,
              preamble_index,
              NCS,
              N_ZC / NCS,
              preamble_offset,
              preamble_shift,
              en);
    }

    LOG_D(PHY,"PRACH RX preamble_index %d, preamble_offset %d\n",preamble_index,preamble_offset);


    if (new_dft == 1) {
      new_dft = 0;

      Xu = (int16_t*)gNB->X_u[preamble_offset-first_nonzero_root_idx];

      LOG_D(PHY,"PRACH RX new dft preamble_offset-first_nonzero_root_idx %d\n",preamble_offset-first_nonzero_root_idx);

      memset(prach_ifft,0,((N_ZC==839) ? 2048 : 256)*sizeof(int32_t));
      memset(prachF, 0, sizeof(int16_t) * 2 * 1024);
      if (LOG_DUMPFLAG(DEBUG_PRACH)) {
        LOG_M("prach_rxF0.m","prach_rxF0",rxsigF[0],N_ZC,1,1);
        LOG_M("prach_rxF1.m","prach_rxF1",rxsigF[1],6144,1,1);
      }

      for (int aa = 0; aa < nb_rx; aa++) {
	// Do componentwise product with Xu* on each antenna 

        for (int offset = 0; offset < (N_ZC << 1); offset += 2) {
          prachF[offset] = (int16_t)(((int32_t)Xu[offset]*rxsigF[aa][offset] + (int32_t)Xu[offset+1]*rxsigF[aa][offset+1])>>15);
          prachF[offset+1] = (int16_t)(((int32_t)Xu[offset]*rxsigF[aa][offset+1] - (int32_t)Xu[offset+1]*rxsigF[aa][offset])>>15);
        }
        #if OAI_DF_PRACH_MIN
          /* --- ISAC/DF: take the dominant PRACH bin in prachF for this antenna --- */
          if (aa == 0) { int z; for (z=0; z<16; z++) { df_y_re[z]=0.0f; df_y_im[z]=0.0f; } }
          {
            int best = 0;
            long long bestp = -1;
            for (int idx = 0; idx < (N_ZC<<1); idx += 2) {
              int re = (int)prachF[idx];
              int im = (int)prachF[idx+1];
              long long p = (long long)re*re + (long long)im*im;
              if (p > bestp) { bestp = p; best = idx; }
            }
            df_y_re[aa] = (float)prachF[best];
            df_y_im[aa] = (float)prachF[best+1];
          }
          /* ---------------------------------------------------------------------- */
        #endif
	
        // Now do IFFT of size 1024 (N_ZC=839) or 256 (N_ZC=139)
        if (N_ZC == 839) {
          idft(IDFT_1024, prachF, prach_ifft_tmp, 1);
          // compute energy and accumulate over receive antennas
          for (int i = 0; i < 1024; i++)
            prach_ifft[i] += (int32_t)prach_ifft_tmp[i<<1]*(int32_t)prach_ifft_tmp[i<<1] + (int32_t)prach_ifft_tmp[1+(i<<1)]*(int32_t)prach_ifft_tmp[1+(i<<1)];
        } else {
          idft(IDFT_256, prachF, prach_ifft_tmp, 1);
          log2_ifft_size = 8;
          // compute energy and accumulate over receive antennas and repetitions for BR
          for (int i = 0; i < 256; i++)
            prach_ifft[i] += (int32_t)prach_ifft_tmp[i<<1]*(int32_t)prach_ifft_tmp[(i<<1)] + (int32_t)prach_ifft_tmp[1+(i<<1)]*(int32_t)prach_ifft_tmp[1+(i<<1)];
        }

        if (LOG_DUMPFLAG(DEBUG_PRACH)) {
          if (aa == 0)
            LOG_M("prach_rxF_comp0.m","prach_rxF_comp0", prachF, 1024, 1, 1);
          if (aa == 1)
            LOG_M("prach_rxF_comp1.m","prach_rxF_comp1", prachF, 1024, 1, 1);
        }

      } // antennas_rx

      // Normalization of energy over ifft and receive antennas
      if (N_ZC == 839) {
        log2_ifft_size = 10;
        for (int i = 0; i < 1024; i++)
          prach_ifft[i] = (prach_ifft[i]>>log2_ifft_size)/nb_rx;
      } else {
        log2_ifft_size = 8;
        for (int i = 0; i < 256; i++)
          prach_ifft[i] = (prach_ifft[i]>>log2_ifft_size)/nb_rx;
      }

    } // new dft
    
    // check energy in nth time shift, for 

    preamble_shift2 = ((preamble_shift==0) ? 0 : ((preamble_shift<<log2_ifft_size)/N_ZC));

    for (int i = 0; i < NCS2; i++) {
      lev = (int32_t)prach_ifft[(preamble_shift2+i)];
      levdB = dB_fixed_times10(lev);
      if (levdB>*max_preamble_energy) {
        LOG_D(PHY,"preamble_index %d, delay %d en %d dB > %d dB\n",preamble_index,i,levdB,*max_preamble_energy);
        *max_preamble_energy = levdB;
        *max_preamble_delay = i; // Note: This has to be normalized to the 30.72 Ms/s sampling rate
        *max_preamble = preamble_index;
      }
    }
  }// preamble_index


  // The conversion from *max_preamble_delay from TA value is done here.
  // It is normalized to the 30.72 Ms/s, considering the numerology, N_RB and the sampling rate
  // See table 6.3.3.1 -1 and -2 in 38211.

  // Format 0, 1, 2: 24576 samples @ 30.72 Ms/s, 98304 samples @ 122.88 Ms/s
  // By solving:
  // max_preamble_delay * ( (24576*(fs/30.72M)) / 1024 ) / fs = TA * 16 * 64 / 2^mu * Tc

  // Format 3: 6144 samples @ 30.72 Ms/s, 24576 samples @ 122.88 Ms/s
  // By solving:
  // max_preamble_delay * ( (6144*(fs/30.72M)) / 1024 ) / fs = TA * 16 * 64 / 2^mu * Tc

  // Format >3: 2048/2^mu samples @ 30.72 Ms/s, 2048/2^mu * 4 samples @ 122.88 Ms/s
  // By solving:
  // max_preamble_delay * ( (2048/2^mu*(fs/30.72M)) / 256 ) / fs = TA * 16 * 64 / 2^mu * Tc
  uint16_t *TA = max_preamble_delay;
  int mu = fp->numerology_index;
  if (cfg->prach_sequence_length.value == 0) {
    if (prach_fmt == 0 || prach_fmt == 1 || prach_fmt == 2)
      *TA = *TA * 3 * (1 << mu) / 2;
    else if (prach_fmt == 3)
      *TA = *TA * 3 * (1 << mu) / 8;
  }
  else *TA = *TA/2;
  
  #if OAI_DF_PRACH_MIN
    /* ================== ISAC/DF: Tikhonov-weighted steering scan ================== */
    do {
      int M = nb_rx;
      if (M < 2) break;

      /* 2.1 Phase calibration to antenna-0 (like cal_data.m, single snapshot) */
      {
        float r0 = df_y_re[0], i0 = df_y_im[0];
        float p0 = r0*r0 + i0*i0; if (p0 < 1e-6f) p0 = 1e-6f;
        for (int a=0; a<M; a++) {
          float rx = df_y_re[a], ix = df_y_im[a];
          /* multiply by conj(ref)/|ref| */
          float t_re =  rx*r0 + ix*i0;
          float t_im = -rx*i0 + ix*r0;
          df_y_re[a] = t_re / p0;
          df_y_im[a] = t_im / p0;
        }
        df_y_re[0] = 1.0f; df_y_im[0] = 0.0f;
      }

      /* 2.2 Array geometry (meters) — your X410 non-uniform 4-elt NULA */
      float el_m[16] = {0};
      if (M>1) el_m[1] = 0.0475f;
      if (M>2) el_m[2] = 0.1050f;
      if (M>3) el_m[3] = 0.1525f;

      /* 2.3 Wavelength from UL/DL carrier (fallback 2.60 GHz) */
      const float c0 = 299792458.0f;
      float fc = (float)gNB->frame_parms.ul_CarrierFreq;
      if (fc <= 1.0f) fc = (float)gNB->frame_parms.dl_CarrierFreq;
      if (fc <= 1.0f) fc = 2.60e9f;
      float lambda = c0 / fc;

      /* 2.4 Tikhonov-weighted score:
            score(θ) = |a(θ)^H y|^2 / (a(θ)^H a(θ) + α)
        With normalized y, a^H a is ~M; choose small α to stabilize.
      */
      const float alpha = 1e-3f;  /* small Tikhonov; tune if needed */

      int   Kscan = 181;
      float bestS = -1.0f;
      int   bestk = 0;

      for (int k=0; k<Kscan; k++) {
        float theta_deg = -90.0f + (float)k;
        float st = sinf(theta_deg * (3.14159265358979323846f/180.0f));

        /* a(θ) and dot = a^H y */
        float dot_re = 0.0f, dot_im = 0.0f;
        float a2 = 0.0f;
        for (int a=0; a<M; a++) {
          float phase = 2.0f*3.14159265358979323846f * (el_m[a]/lambda) * st;
          float ar = cosf(phase);     /* real steering */
          float ai = sinf(phase);     /* imag steering */
          /* conj(a) * y -> (ar - j ai) * (y_re + j y_im) */
          dot_re +=  ar*df_y_re[a] + ai*df_y_im[a];
          dot_im += -ai*df_y_re[a] + ar*df_y_im[a];
          a2     +=  ar*ar + ai*ai;   /* =1 per element, sums to M */
        }
        float num = dot_re*dot_re + dot_im*dot_im;   /* |a^H y|^2 */
        float den = a2 + alpha;                      /* M + α */
        float score = num / den;

        if (score > bestS) { bestS = score; bestk = k; }
      }

      float doa_hat = -90.0f + (float)bestk;

      /* Optional: only print on real PRACH detections + once per frame.slot */
      #ifndef PRACH_DF_MIN_ENERGY_X10
      #define PRACH_DF_MIN_ENERGY_X10 400  /* 40.0 dB default threshold */
      #endif
      
      static int df_last_frame = -1, df_last_slot = -1, df_last_pre = -1;
      if (*max_preamble_energy >= PRACH_DF_MIN_ENERGY_X10 &&
          (frame != df_last_frame || slot != df_last_slot || *max_preamble != df_last_pre)) {
        LOG_I(PHY, "[PRACH-DF(OptA)] M=%d, fc=%.3f GHz, DoA=%.1f deg, maxEn=%d, TA=%u, RAPID=%u, %d.%d\n",
              M, fc*1e-9f, doa_hat, (int)(*max_preamble_energy), (unsigned int)(*max_preamble_delay),
              (unsigned)(*max_preamble), frame, slot);
        df_last_frame = frame; df_last_slot = slot; df_last_pre = *max_preamble;
      }
    } while(0);
    /* =========================================================================== */
    #endif


  if (LOG_DUMPFLAG(DEBUG_PRACH)) {
    //int en = dB_fixed(signal_energy((int32_t*)&rxsigF[0][0],840));
    //    if (en>60) {
      int k = (12 * n_ra_prb) - 6 * fp->N_RB_UL;
      if (k < 0)
        k += fp->ofdm_symbol_size;

      k*=12;
      k+=13;
      k*=2;

      LOG_M("rxsigF.m","prach_rxF", &rxsigF[0][0], 12288, 1, 1);
      LOG_M("prach_rxF_comp0.m","prach_rxF_comp0", prachF, 1024, 1, 1);
      LOG_M("Xu.m","xu", Xu, N_ZC, 1, 1);
      LOG_M("prach_ifft0.m","prach_t0", prach_ifft, 1024, 1, 1);
      //    }
  }
  /*
    if (sockfd >= 0) {
  const int nb_rx = gNB->gNB_config.carrier_config.num_rx_ant.value;
  /* compute bytes dynamically: N_ZC complex samples -> N_ZC*2 int16_t values */
  //const size_t nbytes = (size_t)N_ZC * 2 * sizeof(int16_t);
    /*
    LOG_I(PHY, "[PRACH-RAW] Sending raw rxsigF before correlation @ %d.%d\n", 
          frame, slot);
    
    for (int aa = 0; aa < nb_rx; aa++) {
      // rxsigF[aa] = ham frekans-domain PRACH verisi (henüz işlenmemiş)
      const int16_t *iq = (const int16_t *)rxsigF[aa];
      
      ssize_t sent = sendto(sockfd, iq, nbytes, 0,
                            (struct sockaddr*)&server_addr, 
                            sizeof(server_addr));
      
      if (sent == nbytes) {
        LOG_I(PHY, "[PRACH-RAW] ant %d: %zd bytes sent OK\n", aa, sent);
      } else {
        LOG_E(PHY, "[PRACH-RAW] ant %d FAILED: %zd/%zu (%s)\n", 
              aa, sent, nbytes, sent < 0 ? strerror(errno) : "partial");
      }
      
      usleep(1000);  // 1ms between antennas
    }
  } else {
    LOG_W(PHY, "[PRACH-RAW] Socket not initialized (sockfd=%d)\n", sockfd);
    */
  
  
  /* === Optional: forward raw per-antenna PRACH over UDP ONLY when a
   * detected preamble equals 60. This avoids spamming the receiver with
   * raw buffers for every PRACH occasion. The socket is lazily
   * initialized by `prach_socket_init` (weak fallback above) if needed.
   */
  do {
    /* Ensure we have a valid PRACH socket (lazy init) */
    if (prach_sockfd < 0) {
      if (prach_socket_init("127.0.0.1", 5678) < 0) {
        LOG_W(PHY, "PRACH: prach_socket_init failed, skipping UDP forward\n");
        break;
      } else {
        LOG_I(PHY, "PRACH: lazy socket initialized to 127.0.0.1:5678 for forwarding\n");
      }
    }

    /* Only forward when detected preamble equals 60 */
    if ((uint16_t)(*max_preamble) == 60) {
      const size_t nbytes = (size_t)N_ZC * 2 * sizeof(int16_t); /* N_ZC complex samples */
      LOG_I(PHY, "[PRACH-RAW] Forwarding raw PRACH (preamble=60) nb_rx=%d N_ZC=%u bytes=%zu at %d.%d\n", nb_rx, (unsigned)N_ZC, nbytes, frame, slot);

      for (int aa = 0; aa < nb_rx; aa++) {
        const int16_t *iq = (const int16_t *)rxsigF[aa];
        /* Build a 14-byte header (network order) and prepend a 4-byte
         * magic "PRCH" so the receiver can reliably detect valid
         * PRACH-RAW packets. Final header on the wire is:
         *  4 bytes magic 'P' 'R' 'C' 'H'
         *  4 bytes frame (network)
         *  4 bytes slot  (network)
         *  1 byte  antenna
         *  1 byte  prachOccasion
         *  2 bytes N_ZC (network)
         *  2 bytes data_len (number of int16 values, network)
         */
        uint16_t data_len_field = (uint16_t)(N_ZC * 2); /* int16 count */
        uint8_t hdr[14];
        uint32_t f_n = htonl((uint32_t)frame);
        uint32_t s_n = htonl((uint32_t)slot);
        memcpy(hdr + 0, &f_n, 4);
        memcpy(hdr + 4, &s_n, 4);
        hdr[8] = (uint8_t)aa; /* antenna index (0-based) */
        hdr[9] = (uint8_t)prachOccasion;
        uint16_t N_ZC_n = htons((uint16_t)N_ZC);
        memcpy(hdr + 10, &N_ZC_n, 2);
        uint16_t dl_n = htons(data_len_field);
        memcpy(hdr + 12, &dl_n, 2);
        const uint8_t magic[4] = {'P','R','C','H'};

        /* Build payload by extracting N_ZC complex samples from the
         * frequency-domain buffer starting at subcarrier index `k` (as in
         * the RU path). This matches the receiver's expectation and
         * avoids issues caused by earlier CP removal/DFT alignment.
         */
        size_t bytes_payload = (size_t)data_len_field * sizeof(int16_t);
        size_t total = 4 + 14 + bytes_payload; /* magic + header + payload */
        uint8_t *buf = malloc(total);
        if (!buf) {
          LOG_E(PHY, "[PRACH-RAW] malloc failed for ant %d\n", aa);
          continue;
        }
        memcpy(buf, magic, 4);
        memcpy(buf + 4, hdr, 14);

        /* For now copy the first bytes_payload bytes from rxsigF[aa].
         * This keeps the wire format consistent (magic+header+IQ)
         * while avoiding reliance on internal k/dftlen alignment.
         */
        const int16_t *iq_ptr = (const int16_t *)rxsigF[aa];
        memcpy(buf + 4 + 14, (const void *)iq_ptr, bytes_payload);

        ssize_t sent = sendto(prach_sockfd, buf, total, 0,
                              (struct sockaddr *)&prach_server_addr,
                              sizeof(prach_server_addr));
        if (sent == (ssize_t)total) {
          LOG_D(PHY, "[PRACH-RAW] ant %d: %zd bytes sent (incl header)\n", aa, sent);
        } else {
          LOG_E(PHY, "[PRACH-RAW] ant %d FAILED: %zd/%zu (%s)\n",
                aa, sent, total, sent < 0 ? strerror(errno) : "partial");
        }
        free(buf);
        usleep(1000); /* 1ms gap between antenna forwards */
      }
    } else {
      LOG_D(PHY, "[PRACH-RAW] Detected preamble=%u != 60; not forwarding\n", (unsigned)(*max_preamble));
    }
  } while (0);

  stop_meas(&gNB->rx_prach);

}

