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
#define RADIOTAP_HEADER_LEN     10       // Min len + TXFlags
#define RADIOTAP_LEN_OFFSET 2
#define RADIOTAP_PRESENT_B1_OFFSET 4
#define RADIOTAP_PRESENT_B2_OFFSET 5
#define DOT11_FRAGMENT_OFFSET 30
#define DOT11_SEQUENCE_OFFSET 31
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

void WifiDrvLinux::getRadioMacAddress(uint8_t macAddress[6]) const {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        LIB_LOGE(TAG, "getRadioMacAddress: cannot create socket: %s", strerror(errno));
        return;
    }
    
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, mInterface.c_str(), IFNAMSIZ - 1);

    if (ioctl(sock, SIOCGIFHWADDR, &ifr) < 0) {
        LIB_LOGE(TAG, "getRadioMacAddress: SIOCGIFHWADDR failed");
        close(sock);
        return;
    }

    memcpy(macAddress, ifr.ifr_hwaddr.sa_data, 6);
    close(sock);
    LIB_LOGD(TAG, "getRadioMacAddress: %02X:%02X:%02X:%02X:%02X:%02X", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
    return;
}

void WifiDrvLinux::setRadioMacAddress(const uint8_t macAddress[6]) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        LIB_LOGE(TAG, "setRadioMacAddress: cannot create socket: %s", strerror(errno));
        return;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, mInterface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_hwaddr.sa_family = ARPHRD_ETHER;
    memcpy(ifr.ifr_hwaddr.sa_data, macAddress, 6);

    if (ioctl(sock, SIOCSIFHWADDR, &ifr) < 0) {
        LIB_LOGE(TAG, "setRadioMacAddress: SIOCSIFHWADDR failed: %s", strerror(errno));
        close(sock);
        return;
    }

    close(sock);
    return;
}

void WifiDrvLinux::setRadioChannel(uint8_t channel) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        LIB_LOGE(TAG, "setRadioChannel: cannot create socket: %s", strerror(errno));
        return;
    }

    struct iwreq iwr;
    memset(&iwr, 0, sizeof(iwr));
    strncpy(iwr.ifr_name, mInterface.c_str(), IFNAMSIZ - 1);

    // Imposta i parametri della frequenza per il canale richiesto
    iwr.u.freq.m = channel;          // Il campo 'm' contiene il numero del canale
    iwr.u.freq.e = 0;                // Esponente a 0 per indicare un canale
    iwr.u.freq.i = 0;                // Indice non utilizzato
    iwr.u.freq.flags = 0;            // Nessun flag

    if (ioctl(sock, SIOCSIWFREQ, &iwr) < 0) {
        LIB_LOGE(TAG, "setRadioChannel: SIOCSIWFREQ failed: %s", strerror(errno));
    }

    close(sock);
}

void WifiDrvLinux::setNetInterfaceUpOrDown(bool up) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        LIB_LOGE(TAG, "setNetInterfaceUpOrDown: cannot create socket: %s", strerror(errno));
        return;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, mInterface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_flags = up ? IFF_UP : 0;

    if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
        LIB_LOGE(TAG, "setNetInterfaceUpOrDown: SIOCSIFFLAGS failed: %s", strerror(errno));
        close(sock);
        return;
    }

    close(sock);
    LIB_LOGD(TAG, "interface %s %s", mInterface.c_str(), up ? "up" : "down");
    return;
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

    {
        /*setNetInterfaceUpOrDown(false);
        setMonitorMode();
        uint8_t macAddress[6];
        getRadioMacAddress(macAddress);
        macAddress[0] = 0x1E;
        macAddress[1] = 0x7F;
        macAddress[2] = 0x00;
        macAddress[3] = (mMacAddress >> 16) & 0xFF;
        macAddress[4] = (mMacAddress >> 8) & 0xFF;
        macAddress[5] = mMacAddress & 0xFF;
        setRadioMacAddress(macAddress);
        setNetInterfaceUpOrDown(true);*/
        setRadioChannel(mChannel);
    }

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

    //LIB_LOGD(TAG, "recvLoop: addr1: %02X %02X %02X %02X %02X %02X", dot11Data[4], dot11Data[5], dot11Data[6], dot11Data[7], dot11Data[8], dot11Data[9]);
    //LIB_LOGD(TAG, "recvLoop: addr2: %02X %02X %02X %02X %02X %02X", dot11Data[10], dot11Data[11], dot11Data[12], dot11Data[13], dot11Data[14], dot11Data[15]);

    if (dot11Data[4] == 0xFE  && (dot11Data[5]) == 0x7F && ((dot11Data[10] == 0xFE && dot11Data[11] == 0x7F) || (dot11Data[10] == 0xFF && dot11Data[11] == 0xFF))) {
        int16_t rssi = getRadiotapRssi(buffer, totalLen);
        LIB_LOGD(TAG, "recvLoop: frame control type: %d, subtype: %d, rssi: %d", fc->Type, fc->Subtype, rssi);

        uint32_t targetId = dot11Data[7] << 16 | dot11Data[8] << 8 | dot11Data[9];
        LIB_LOGD(TAG, "recvLoop: targetId: %06X myId: %06X", targetId, mMacAddress);
        if (targetId != mMacAddress && targetId != 0xFFFFFF) {
            return;
        }
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
    saddr.sll_protocol = htons(ETH_P_ALL);

    size_t radioTapLen = RADIOTAP_HEADER_LEN + len;
    uint8_t *radioTap = (uint8_t *)malloc(radioTapLen);
    if (radioTap == nullptr) {
        LIB_LOGE(TAG, "injectFrame: cannot allocate radio tap");
        if (mInjectFrameCallback) mInjectFrameCallback(1);
        return false;
    }

    memset(radioTap, 0, radioTapLen);
    radioTap[RADIOTAP_LEN_OFFSET] = RADIOTAP_HEADER_LEN;
    radioTap[RADIOTAP_PRESENT_B2_OFFSET] = 0x80;
    memcpy(radioTap + RADIOTAP_HEADER_LEN, data, len);

    radioTap[DOT11_FRAGMENT_OFFSET] = (mSequence & 0xF) << 4;
    radioTap[DOT11_SEQUENCE_OFFSET] = (mSequence >> 4) & 0xFF;

    radioTap[RADIOTAP_MIN_HEADER_LEN] = 0x08; // TxFlag: NoAck
    mSequence++;

    LIB_LOGD(TAG, "injectFrame: sending len: %d", len);

    bool error = false;
    ssize_t sent = sendto(mSocketFd, radioTap, radioTapLen, 0, (struct sockaddr *)&saddr, sizeof(saddr));
    if (sent < 0 || (size_t)sent != radioTapLen) {
        LIB_LOGE(TAG, "injectFrame: failed: %s", strerror(errno));
        error = true;
    }

    free(radioTap);
    if (mInjectFrameCallback) mInjectFrameCallback(error ? 1 : 0);
    return !error;
}

}
#endif
