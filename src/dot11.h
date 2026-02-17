#pragma once
#include <cstdint>

#define PACKETBUF_80211_SIZE 24

#define FRAME_TYPE_MANAGEMENT 0
#define FRAME_TYPE_CONTROL 1
#define FRAME_TYPE_DATA 2
#define FRAME_SUBTYPE_DATA 0
#define FRAME_SUBTYPE_PROBE_REQUEST 0x04
#define FRAME_SUBTYPE_PROBE_RESPONSE 0x05
#define FRAME_SUBTYPE_BEACON 0x08
#define FRAME_SUBTYPE_AUTH 0x0b
#define FRAME_SUBTYPE_DEAUTH 0x0c

struct framectrl_80211_st {
    uint8_t Protocol:2;
    uint8_t Type:2;
    uint8_t Subtype:4;
    uint8_t ToDS:1;
    uint8_t FromDS:1;
    uint8_t MoreFlag:1;
    uint8_t Retry:1;
    uint8_t PwrMgmt:1;
    uint8_t MoreData:1;
    uint8_t Protectedframe:1;
    uint8_t Order:1;
};
typedef struct framectrl_80211_st framectrl_80211_t;
typedef framectrl_80211_t *framectrl_80211_p;

struct ieee80211_hdr_st {
	framectrl_80211_t frame_control;
	uint16_t duration_id;
	uint8_t addr1[6];
	uint8_t addr2[6];
	uint8_t addr3[6];
	uint16_t seq_ctrl;
	//uint8_t addr4[6];
} __attribute__ ((packed));
typedef struct ieee80211_hdr_st ieee80211_hdr_t;
typedef ieee80211_hdr_t *ieee80211_hdr_p;