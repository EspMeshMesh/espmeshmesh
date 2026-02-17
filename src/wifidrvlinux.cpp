#ifdef USE_LINUX
#include "wifidrvlinux.h"
#include "log.h"
#include "dot11.h"

#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/wireless.h>
#include <netinet/if_ether.h>
#include <arpa/inet.h>

static const char *TAG = "WIFILINUX";

#define RADIOTAP_MIN_HEADER_LEN 8
#define RADIOTAP_LEN_OFFSET 2
#define RECV_SELECT_TIMEOUT_MS 200

namespace espmeshmesh {

WifiDrvLinux::WifiDrvLinux() {
}

WifiDrvLinux::~WifiDrvLinux() {
    if(mRunning) driverShutdown();
}

std::string WifiDrvLinux::encryptPassword(std::string password) {
    return password;
}

uint16_t WifiDrvLinux::getRadiotapLength(const uint8_t *data, size_t len) const {
    if (len < RADIOTAP_MIN_HEADER_LEN) {
        return 0;
    }
    return data[RADIOTAP_LEN_OFFSET] | (data[RADIOTAP_LEN_OFFSET + 1] << 8);
}

int8_t WifiDrvLinux::getRadiotapRssi(const uint8_t *data, size_t len) const {
    if (len < RADIOTAP_MIN_HEADER_LEN + 6) {
        return -100;
    }
    return data[RADIOTAP_MIN_HEADER_LEN + 6];
}

bool WifiDrvLinux::setMonitorMode() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        LIB_LOGE(TAG, "setMonitorMode: cannot create socket: %s", strerror(errno));
        return false;
    }

    struct iwreq iwr;
    memset(&iwr, 0, sizeof(iwr));
    strncpy(iwr.ifr_name, mInterface.c_str(), IFNAMSIZ - 1);
    iwr.u.mode = IW_MODE_MONITOR;

    if (ioctl(sock, SIOCSIWMODE, &iwr) < 0) {
        LIB_LOGW(TAG, "setMonitorMode: SIOCSIWMODE failed (interface may already be in monitor mode): %s", strerror(errno));
        close(sock);
        return false;
    }

    close(sock);
    LIB_LOGD(TAG, "Interface %s set to monitor mode", mInterface.c_str());
    return true;
}

void WifiDrvLinux::driverSetup() {
    unsigned int ifindex = if_nametoindex(mInterface.c_str());
    if (ifindex == 0) {
        LIB_LOGE(TAG, "driverSetup: interface %s not found: %s", mInterface.c_str(), strerror(errno));
        return;
    }

    setMonitorMode();

    mSocketFd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (mSocketFd < 0) {
        LIB_LOGE(TAG, "driverSetup: cannot create raw socket: %s", strerror(errno));
        return;
    }

    struct sockaddr_ll saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sll_family = AF_PACKET;
    saddr.sll_ifindex = ifindex;
    saddr.sll_protocol = htons(ETH_P_ALL);

    if (bind(mSocketFd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        LIB_LOGE(TAG, "driverSetup: bind failed: %s", strerror(errno));
        close(mSocketFd);
        mSocketFd = -1;
        return;
    }

    mRunning = true;
    LIB_LOGD(TAG, "Raw socket bound to %s, capture started", mInterface.c_str());
}

void WifiDrvLinux::driverLoop() {
    if(mRunning && mSocketFd >= 0) {
        recvLoop();
    }
}

void WifiDrvLinux::driverShutdown() {
    mRunning = false;
    if (mSocketFd >= 0) {
        ::shutdown(mSocketFd, SHUT_RDWR);
        close(mSocketFd);
        mSocketFd = -1;
    }
}

void WifiDrvLinux::dump_config() {
    WifiDrv::dump_config();
    LIB_LOGCONFIG(TAG, "Interface: %s", mInterface.c_str());
}

void WifiDrvLinux::recvLoop() {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(mSocketFd, &readfds);

    struct timeval tv;
    tv.tv_sec = RECV_SELECT_TIMEOUT_MS / 1000;
    tv.tv_usec = (RECV_SELECT_TIMEOUT_MS % 1000) * 1000;

    int ret = select(mSocketFd + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
        if (errno == EINTR) return;
        LIB_LOGE(TAG, "recvLoop: select error: %s", strerror(errno));
        return;
    }
    if (ret == 0) {
        return;
    }
    if (!FD_ISSET(mSocketFd, &readfds)) {
        return;
    }

    uint8_t buffer[1500];
    size_t bufferSize = sizeof(buffer);

    ssize_t n = recv(mSocketFd, buffer, bufferSize, 0);
    if (n <= 0) {
        if (errno == EINTR || errno == EAGAIN) return;
        LIB_LOGE(TAG, "recvLoop: recv error: %s", strerror(errno));
        return;
    }


    if (!mCaptureFrameCallback) return;

    size_t totalLen = static_cast<size_t>(n);

    uint16_t radiotapLen = getRadiotapLength(buffer, totalLen);
    if (radiotapLen == 0 || radiotapLen > totalLen) {
        return;
    }

    const uint8_t *dot11Data = buffer + radiotapLen;
    uint16_t dot11Len = static_cast<uint16_t>(totalLen - radiotapLen);

    if (dot11Len < PACKETBUF_80211_SIZE + 5) {
        return;
    }


    framectrl_80211_t *fc = (framectrl_80211_t *)dot11Data;
    if (fc->Type != FRAME_TYPE_DATA && fc->Subtype != FRAME_SUBTYPE_DATA) {
        return;
    }

    LIB_LOGD(TAG, "recvLoop: addr1: %02X %02X %02X %02X %02X %02X", dot11Data[4], dot11Data[5], dot11Data[6], dot11Data[7], dot11Data[8], dot11Data[9]);
    LIB_LOGD(TAG, "recvLoop: addr2: %02X %02X %02X %02X %02X %02X", dot11Data[10], dot11Data[11], dot11Data[12], dot11Data[13], dot11Data[14], dot11Data[15]);

    if (dot11Data[4] == 0xFE  && (dot11Data[5]) == 0x7F && ((dot11Data[10] == 0xFE && dot11Data[11] == 0x7F) || (dot11Data[10] == 0xFF && dot11Data[11] == 0xFF))) {
        int16_t rssi = getRadiotapRssi(buffer, totalLen);
        LIB_LOGD(TAG, "recvLoop: frame control type: %d, subtype: %d, rssi: %d", fc->Type, fc->Subtype, rssi);
    
        mCaptureFrameCallback(const_cast<uint8_t *>(dot11Data), dot11Len, rssi);
    }
}

bool WifiDrvLinux::injectFrame(uint8_t *data, uint16_t len) {
    if (mSocketFd < 0) {
        LIB_LOGE(TAG, "injectFrame: socket not open");
        if (mInjectFrameCallback) mInjectFrameCallback(1);
        return false;
    }

    unsigned int ifindex = if_nametoindex(mInterface.c_str());
    if (ifindex == 0) {
        LIB_LOGE(TAG, "injectFrame: interface %s not found", mInterface.c_str());
        if (mInjectFrameCallback) mInjectFrameCallback(1);
        return false;
    }

    struct sockaddr_ll saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sll_family = AF_PACKET;
    saddr.sll_ifindex = ifindex;
    saddr.sll_halen = ETH_ALEN;
    memset(saddr.sll_addr, 0xff, ETH_ALEN);

    ssize_t sent = sendto(mSocketFd, data, len, 0, (struct sockaddr *)&saddr, sizeof(saddr));
    if (sent < 0 || (size_t)sent != len) {
        LIB_LOGE(TAG, "injectFrame: sendto failed: %s", strerror(errno));
        if (mInjectFrameCallback) mInjectFrameCallback(1);
        return false;
    }

    if (mInjectFrameCallback) mInjectFrameCallback(0);
    return true;
}

}
#endif
